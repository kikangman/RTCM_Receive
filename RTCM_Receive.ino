//git kikangman
/*
cd /Users/kikang/Desktop/ki/summershot/RTK/RTCM_Receive
./save_push.sh
*/


#include <RadioLib.h>

#define CUSTOM_MOSI 12
#define CUSTOM_MISO 13
#define CUSTOM_SCLK 11
#define CUSTOM_NSS 10
#define CUSTOM_DIO1 2
#define CUSTOM_NRST 4
#define CUSTOM_BUSY 3

SPIClass customSPI(HSPI);
SX1280 radio = new Module(CUSTOM_NSS, CUSTOM_DIO1, CUSTOM_NRST, CUSTOM_BUSY, customSPI);

// --- 덩어리 버퍼 ---
#define MAX_STREAM_SIZE 8192
uint8_t streamBuffer[MAX_STREAM_SIZE];
size_t streamLen = 0;
unsigned long lastPacketTime = 0;
const unsigned long PACKET_TIMEOUT_MS = 100;

volatile bool receivedFlag = false;

ICACHE_RAM_ATTR void setFlag() {
  receivedFlag = true;
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("=== RTCM 수신기 시작 ===");

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
}

void loop() {
  // 1️⃣ LoRa 패킷 수신
  if (receivedFlag) {
    receivedFlag = false;

    uint8_t buffer[256];
    int state = radio.readData(buffer, sizeof(buffer));

    if (state == RADIOLIB_ERR_NONE) {
      size_t packetLen = radio.getPacketLength();

      if (packetLen > 4) {
        // streamBuffer 새로 시작 시 초기화
        if (streamLen == 0) {
          memset(streamBuffer, 0, MAX_STREAM_SIZE);
        }

        if (streamLen + packetLen - 4 < MAX_STREAM_SIZE) {
          memcpy(streamBuffer + streamLen, buffer + 4, packetLen - 4);  // ✅ chunk 헤더 제거
          streamLen += packetLen - 4;
          lastPacketTime = millis();
        } else {
          Serial.println("[❌] streamBuffer overflow! 초기화");
          streamLen = 0;
        }
      } else {
        Serial.println("[⚠️] Skipped: not RTCM start");
      }
    } else {
      Serial.printf("[LoRa] ❌ 수신 실패 (%d)\n", state);
    }

    radio.startReceive();
  }

  // 2️⃣ 덩어리 수신 완료 판단
  if (streamLen > 0 && millis() - lastPacketTime > PACKET_TIMEOUT_MS) {
    Serial.printf("\n[RTCM] ✅ 덩어리 수신 완료 (%zu bytes):\n[Hex] ", streamLen);
    for (size_t i = 0; i < streamLen; i++) {
      Serial.printf("%02X ", streamBuffer[i]);
      if ((i + 1) % 32 == 0) Serial.println();
    }
    Serial.println();

    streamLen = 0;  // 초기화
  }
}