//git kikangman
/*
cd /Users/kikang/Desktop/ki/summershot/RTK/RTCM_Receive
./save_push.sh "??"

*/
#include <RadioLib.h>
#include <TinyGPS++.h>

#define CUSTOM_MOSI 12
#define CUSTOM_MISO 13
#define CUSTOM_SCLK 11
#define CUSTOM_NSS 10
#define CUSTOM_DIO1 2
#define CUSTOM_NRST 4
#define CUSTOM_BUSY 3

#define RX_GNSS 9
#define TX_GNSS 8

SPIClass customSPI(HSPI);
SX1280 radio = new Module(CUSTOM_NSS, CUSTOM_DIO1, CUSTOM_NRST, CUSTOM_BUSY, customSPI);
HardwareSerial GNSS(1);
TinyGPSPlus gps;

const int MAX_QUEUE_SIZE = 10;
String locationQueue[MAX_QUEUE_SIZE];
int queueStart = 0, queueEnd = 0, queueCount = 0;
int lastFixType = 0;

#define MAX_STREAM_SIZE 8192
uint8_t streamBuffer[MAX_STREAM_SIZE];
size_t streamLen = 0;
unsigned long lastPacketTime = 0;
const unsigned long PACKET_TIMEOUT_MS = 100;
volatile bool receivedFlag = false;

String nmeaLine = "";

ICACHE_RAM_ATTR void setFlag() {
  receivedFlag = true;
}

void sendCommandWithChecksum(const String &cmdBody) {
  byte checksum = 0;
  for (int i = 0; i < cmdBody.length(); i++) checksum ^= cmdBody[i];
  char command[128];
  snprintf(command, sizeof(command), "$%s*%02X\r\n", cmdBody.c_str(), checksum);
  GNSS.print(command);
  Serial.print("Sent: ");
  Serial.print(command);
}

void enqueueLocation(float lat, float lng) {
  if (queueCount >= MAX_QUEUE_SIZE) {
    queueStart = (queueStart + 1) % MAX_QUEUE_SIZE;
    queueCount--;
  }
  char buf[40];
  snprintf(buf, sizeof(buf), "%.8f,%.8f", lat, lng);
  locationQueue[queueEnd] = String(buf);
  queueEnd = (queueEnd + 1) % MAX_QUEUE_SIZE;
  queueCount++;
}

void processLocationQueue() {
  if (queueCount > 0 && gps.time.isValid()) {
    String msg;

    char timeStr[10];
    snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());

    String latStr = String(gps.location.lat(), 8);
    String lonStr = String(gps.location.lng(), 8);
    msg = String(timeStr) + "," + latStr + "," + lonStr + "," + String(lastFixType);

    queueStart = (queueStart + 1) % MAX_QUEUE_SIZE;
    queueCount--;

    radio.standby();
    int state = radio.transmit((uint8_t *)msg.c_str(), msg.length());
    if (state == RADIOLIB_ERR_NONE) {
      radio.finishTransmit();
      Serial.println("[LoRa] 📨 send: " + msg);
    } else {
      Serial.printf("[LoRa] ❌ 전송 실패 (%d)\n", state);
    }
    radio.startReceive();
  }
}

void setup() {
  Serial.begin(115200);
  GNSS.begin(115200, SERIAL_8N1, RX_GNSS, TX_GNSS);
  delay(300);

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
  Serial.println("[LoRa] 🟢 수신 대기 시작");

  delay(500);
  sendCommandWithChecksum("PQTMCFGRCVRMODE,W,1");
  delay(100);
  sendCommandWithChecksum("PQTMCFGFIXRATE,W,1000");
  delay(100);
  sendCommandWithChecksum("PQTMSAVEPAR");
  delay(100);
  sendCommandWithChecksum("PQTMGNSSSTART");
}

void loop() {
  if (receivedFlag) {
    receivedFlag = false;

    uint8_t buffer[256];
    int state = radio.readData(buffer, sizeof(buffer));
    if (state == RADIOLIB_ERR_NONE) {
      size_t packetLen = radio.getPacketLength();
      if (packetLen > 4 && streamLen + packetLen - 4 < MAX_STREAM_SIZE) {
        memcpy(streamBuffer + streamLen, buffer + 4, packetLen - 4);
        streamLen += packetLen - 4;
        lastPacketTime = millis();

        if (buffer[3] & 0x80) {
          Serial.printf("[RTCM] ✅ 최종청크 수신 (%zu bytes)\n", streamLen);
          for (size_t i = 0; i < streamLen; i++) {
            GNSS.write(streamBuffer[i]);
          }
          streamLen = 0;
          processLocationQueue();
        }
      } else {
        Serial.println("[❌] RTCM 버퍼 오버플로우");
        streamLen = 0;
      }
    }
    radio.startReceive();
  }

  if (streamLen > 0 && millis() - lastPacketTime > 1000) {
    Serial.println("[⚠️] RTCM 누적 데이터 오래됨 → 초기화");
    streamLen = 0;
  }

  while (GNSS.available()) {
    char c = GNSS.read();
    gps.encode(c);

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
        lastFixType = fixType;

        if (gps.location.isValid()) {
          char timeStr[10] = "??:??:??";
          if (gps.time.isValid()) {
            snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
          }
          Serial.printf("[GPS] %s | %.8f, %.8f | Fix: %d\n", timeStr, gps.location.lat(), gps.location.lng(), fixType);
          enqueueLocation(gps.location.lat(), gps.location.lng());
        }
      }
      nmeaLine = "";
    } else {
      nmeaLine += c;
    }
  }
}