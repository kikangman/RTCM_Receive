#include <RadioLib.h>

// Define custom SPI pins
#define CUSTOM_MOSI 12
#define CUSTOM_MISO 13
#define CUSTOM_SCLK 11
#define CUSTOM_NSS 10
#define CUSTOM_DIO1 4
#define CUSTOM_NRST 6
#define CUSTOM_BUSY 5

// Define custom UART pins for GNSS
#define RX_GNSS 8
#define TX_GNSS 9

// Create a custom SPI instance
SPIClass customSPI(HSPI);  // Use VSPI or HSPI depending on your setup

// Create an SX1280 instance with custom SPI
SX1280 radio = new Module(CUSTOM_NSS, CUSTOM_DIO1, CUSTOM_NRST, CUSTOM_BUSY, customSPI);

// Use UART2 for GNSS
HardwareSerial GNSS(2);  // Use Serial2 (UART2)

// Timing for UART read
unsigned long lastUartRead = 0;
const unsigned long uartInterval = 1000;  // 1 second

// Flag to indicate that a packet was received
volatile bool receivedFlag = false;

// ISR for packet received
#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void setFlag(void) {
  receivedFlag = true;
}

void setup() {
  Serial.begin(115200);  // Debug Serial
  GNSS.begin(115200, SERIAL_8N1, RX_GNSS, TX_GNSS);  // GNSS Serial

  Serial.println(F("[Setup] Initializing LoRa and GNSS UART..."));

  customSPI.begin(CUSTOM_SCLK, CUSTOM_MISO, CUSTOM_MOSI, CUSTOM_NSS);

  Serial.print(F("[SX1280] Initializing ... "));
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }

  radio.setPacketReceivedAction(setFlag);

  Serial.print(F("[SX1280] Starting to listen ... "));
  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }
}

void loop() {
  // Handle LoRa packet reception
  if (receivedFlag) {
    receivedFlag = false;

    String str;
    int state = radio.readData(str);

    if (state == RADIOLIB_ERR_NONE) {
      Serial.println(F("[SX1280] Received packet!"));
      Serial.print(F("[SX1280] Data:\t\t"));
      Serial.println(str);
      Serial.print(F("[SX1280] RSSI:\t\t"));
      Serial.print(radio.getRSSI());
      Serial.println(F(" dBm"));
      Serial.print(F("[SX1280] SNR:\t\t"));
      Serial.print(radio.getSNR());
      Serial.println(F(" dB"));
      Serial.print(F("[SX1280] Frequency Error:\t"));
      Serial.print(radio.getFrequencyError());
      Serial.println(F(" Hz"));
    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      Serial.println(F("CRC error!"));
    } else {
      Serial.print(F("failed, code "));
      Serial.println(state);
    }

    // Restart receive mode
    radio.startReceive();
  }

  // Handle GNSS UART reading every 1 second
  if (millis() - lastUartRead >= uartInterval) {
    lastUartRead = millis();

    // Read and print GNSS UART data (non-blocking)
    String gnssData = "";
    while (GNSS.available()) {
      char c = GNSS.read();
      gnssData += c;
    }

    if (gnssData.length() > 0) {
      Serial.println(F("[GNSS] UART Data:"));
      Serial.println(gnssData);
    }
  }
}
