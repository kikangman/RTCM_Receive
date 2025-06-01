//git kikangman
/*
cd /Users/kikang/Desktop/ki/summershot/RTK/RTCM_Receive
./save_push.sh
*/

#include <RadioLib.h>
#include <TinyGPS++.h>

// ---------- LoRa 핀 정의 ----------
#define CUSTOM_MOSI 12
#define CUSTOM_MISO 13
#define CUSTOM_SCLK 11
#define CUSTOM_NSS 10
#define CUSTOM_DIO1 2
#define CUSTOM_NRST 4
#define CUSTOM_BUSY 3

SPIClass customSPI(HSPI);
SX1280 radio = new Module(CUSTOM_NSS, CUSTOM_DIO1, CUSTOM_NRST, CUSTOM_BUSY, customSPI);

// ---------- GNSS UART ----------
#define RX_GNSS 9
#define TX_GNSS 8
HardwareSerial GNSS(1);

// ---------- RTCM 버퍼 ----------
#define MAX_STREAM_SIZE 8192
uint8_t streamBuffer[MAX_STREAM_SIZE];
size_t streamLen = 0;
unsigned long lastPacketTime = 0;
const unsigned long PACKET_TIMEOUT_MS = 100;
volatile bool receivedFlag = false;

// ---------- NMEA 파서 ----------
TinyGPSPlus gps;
String nmeaLine = "";

// ---------- LoRa 수신 인터럽트 ----------
ICACHE_RAM_ATTR void setFlag() {
  receivedFlag = true;
}

// ---------- GNSS 명령 전송 함수 ----------
void sendCommandWithChecksum(const String &cmdBody) {
  byte checksum = 0;
  for (int i = 0; i < cmdBody.length(); i++) checksum ^= cmdBody[i];
  char command[128];
  snprintf(command, sizeof(command), "$%s*%02X\r\n", cmdBody.c_str(), checksum);
  GNSS.print(command);
  Serial.print("Sent: ");
  Serial.print(command);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("=== RTCM 수신기 + GNSS 설정 시작 ===");

  // ---------- GNSS UART 시작 ----------
  GNSS.begin(115200, SERIAL_8N1, RX_GNSS, TX_GNSS);
  delay(300);

  // ---------- LoRa 초기화 ----------
  customSPI.begin(CUSTOM_SCLK, CUSTOM_MISO, CUSTOM_MOSI, CUSTOM_NSS);
  if (radio.begin() != RADIOLIB_ERR_NONE) {
    Serial.println("[LoRa] ❌ 초기화 실패");
    while (true);
  }

  radio.setOutputPower(13);
  radio.setFrequency(2400.0);
  radio.setBandwidth(812.5);
  radio.setSpreadingFactor(7);
  radio.setPacketReceivedAction(setFlag);
  radio.startReceive();
  Serial.println("[LoRa] 수신 대기 중...");

  // ---------- GNSS 설정 ----------
  delay(500);
  sendCommandWithChecksum("PQTMCFGRCVRMODE,W,1");     // RTK Rover 모드
  delay(100);
  sendCommandWithChecksum("PQTMCFGFIXRATE,W,1000");   // 위치 고정 주기 1Hz
  delay(100);
  sendCommandWithChecksum("PQTMSAVEPAR");             // 설정 저장
  delay(100);
  sendCommandWithChecksum("PQTMGNSSSTART");           // GNSS 시작
}

void loop() {
  // ---------- RTCM 수신 처리 ----------
  if (receivedFlag) {
    receivedFlag = false;

    uint8_t buffer[256];
    int state = radio.readData(buffer, sizeof(buffer));

    if (state == RADIOLIB_ERR_NONE) {
      size_t packetLen = radio.getPacketLength();

      if (packetLen > 4) {
        if (streamLen == 0) memset(streamBuffer, 0, MAX_STREAM_SIZE);

        if (streamLen + packetLen - 4 < MAX_STREAM_SIZE) {
          memcpy(streamBuffer + streamLen, buffer + 4, packetLen - 4);
          streamLen += packetLen - 4;
          lastPacketTime = millis();
        } else {
          Serial.println("[❌] streamBuffer overflow! 초기화");
          streamLen = 0;
        }
      }
    } else {
      Serial.printf("[LoRa] ❌ 수신 실패 (%d)\n", state);
    }

    radio.startReceive();
  }

  if (streamLen > 0 && millis() - lastPacketTime > PACKET_TIMEOUT_MS) {
    Serial.printf("\n[RTCM] ✅ chunk 수신 완료 (%zu bytes)\n", streamLen);
    for (size_t i = 0; i < streamLen; i++) {
      GNSS.write(streamBuffer[i]);
    }
    streamLen = 0;
  }

  // ---------- GNSS 수신 & TinyGPS++ 파싱 ----------
  while (GNSS.available()) {
    char c = GNSS.read();
    gps.encode(c);

    // 동시에 GGA 문장도 수집 (RTK Fix 확인용)
    if (c == '\n') {
      if (nmeaLine.startsWith("$GNGGA")) {
        int fixIndex = 0;
        int commaCount = 0;
        for (int i = 0; i < nmeaLine.length(); i++) {
          if (nmeaLine[i] == ',') commaCount++;
          if (commaCount == 6) {
            fixIndex = i + 1;
            break;
          }
        }
        int fixType = nmeaLine.substring(fixIndex, fixIndex + 1).toInt();

        if (gps.location.isValid()) {
          Serial.printf("[location] %.8f,%.8f | Fix: %d ",
                        gps.location.lat(), gps.location.lng(), fixType);
          switch (fixType) {
            case 1: Serial.println("(GPS)"); break;
            case 2: Serial.println("(DGPS)"); break;
            case 4: Serial.println("(RTK Fixed ✅)"); break;
            case 5: Serial.println("(RTK Float ⚠️)"); break;
            default: Serial.println("(Unknown)"); break;
          }
        }
      }
      nmeaLine = "";
    } else {
      nmeaLine += c;
    }
  }
}