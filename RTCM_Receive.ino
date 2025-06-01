//git kikangman
/*
cd /Users/kikang/Desktop/ki/summershot/RTK/RTCM_Receive
./save_push.sh
*/


#include <RadioLib.h>

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

// ---------- NMEA 버퍼 ----------
String nmeaBuffer = "";

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
    while (true)
      ;
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
  sendCommandWithChecksum("PQTMCFGRCVRMODE,W,1");  // RTK Rover 모드
  delay(100);
  sendCommandWithChecksum("PQTMCFGFIXRATE,W,1000");  // fixrate 1000
  delay(100);
  // sendCommandWithChecksum("PQTMCFGMSGRATE,W,GGA,1");
  // sendCommandWithChecksum("PQTMCFGMSGRATE,W,RMC,1");
  // sendCommandWithChecksum("PQTMCFGMSGRATE,W,GLL,1");
  // sendCommandWithChecksum("PQTMCFGMSGRATE,W,VTG,1");
  // sendCommandWithChecksum("PQTMCFGMSGRATE,W,GSA,1");
  // sendCommandWithChecksum("PQTMCFGMSGRATE,W,GSV,1");
  sendCommandWithChecksum("PQTMSAVEPAR");  // 설정 저장
  delay(100);
  sendCommandWithChecksum("PQTMGNSSSTART");  // GNSS 시작
}

void loop() {
  // ---------- LoRa 수신 및 RTCM 전달 ----------
  if (receivedFlag) {
    receivedFlag = false;

    uint8_t buffer[256];
    int state = radio.readData(buffer, sizeof(buffer));

    if (state == RADIOLIB_ERR_NONE) {
      size_t packetLen = radio.getPacketLength();

      if (packetLen > 4) {
        if (streamLen == 0) memset(streamBuffer, 0, MAX_STREAM_SIZE);

        if (streamLen + packetLen - 4 < MAX_STREAM_SIZE) {
          memcpy(streamBuffer + streamLen, buffer + 4, packetLen - 4);  // 헤더 4바이트 제거
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

  // ---------- NMEA 수신 및 전체 출력 ----------
  while (GNSS.available()) {
    char ch = GNSS.read();

    if (ch == '\n') {
      nmeaBuffer.trim();
      if (nmeaBuffer.startsWith("$")) {
        Serial.println("[NMEA] " + nmeaBuffer);
      }
      nmeaBuffer = "";
    } else {
      nmeaBuffer += ch;
    }
  }
}