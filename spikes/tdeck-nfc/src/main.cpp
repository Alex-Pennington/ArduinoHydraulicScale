// THROWAWAY SPIKE: T-Deck + PN532 V3 (HSU). Scan a card, show its UID, press RECORD.
// RECORD SENDS NOTHING. PLC #4 is in service; this spike has no WiFi and no MQTT on purpose.
//
// Pins are the LilyGO T-Deck's (examples/UnitTest/utilities.h); the Grove plug P8 carries GND,
// VDD3V3, UAR0_TX and UAR0_RX (schematic.pdf), which are WROOM-1 TXD0/RXD0 = GPIO43/GPIO44.

#include <Arduino.h>
#include <Wire.h>
#include <LovyanGFX.hpp>
#include <Adafruit_PN532.h>

static constexpr int PIN_POWERON = 10;
static constexpr int PIN_I2C_SDA = 18, PIN_I2C_SCL = 8;
static constexpr int PIN_TFT_CS = 12, PIN_TFT_DC = 11, PIN_TFT_BL = 42;
static constexpr int PIN_SPI_MOSI = 41, PIN_SPI_MISO = 38, PIN_SPI_SCK = 40;
static constexpr int PIN_SD_CS = 39, PIN_RADIO_CS = 9;
static constexpr int PIN_TRACKBALL_PRESS = 0; // active low
static constexpr int PIN_GROVE_TX = 43, PIN_GROVE_RX = 44;
static constexpr uint8_t KB_ADDR = 0x55;

class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789 panel;
  lgfx::Bus_SPI bus;
  lgfx::Light_PWM light;

public:
  LGFX() {
    {
      auto c = bus.config();
      c.spi_host = SPI2_HOST;
      c.spi_mode = 0;
      c.freq_write = 40000000;
      c.freq_read = 16000000;
      c.pin_sclk = PIN_SPI_SCK;
      c.pin_mosi = PIN_SPI_MOSI;
      c.pin_miso = PIN_SPI_MISO;
      c.pin_dc = PIN_TFT_DC;
      bus.config(c);
      panel.setBus(&bus);
    }
    {
      auto c = panel.config();
      c.pin_cs = PIN_TFT_CS;
      c.pin_rst = -1;
      c.panel_width = 240;
      c.panel_height = 320;
      c.invert = true; // as the mesh firmware's ST7789 block sets it
      c.readable = false;
      c.bus_shared = false;
      panel.config(c);
    }
    {
      auto c = light.config();
      c.pin_bl = PIN_TFT_BL;
      c.invert = false;
      light.config(c);
      panel.setLight(&light);
    }
    setPanel(&panel);
  }
};

static LGFX tft;
static Adafruit_PN532 nfc(-1, &Serial1);

static bool readerOk = false;
static char lastUid[3 * 7 + 1] = "";
static uint32_t lastSeenMs = 0;
static uint32_t scans = 0, records = 0;

static void show(const char *status, uint16_t colour) {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(8, 8);
  tft.println("RFID PENDANT SPIKE");
  tft.setTextSize(1);
  tft.setCursor(8, 30);
  tft.println("T-Deck + PN532 NFC. Throwaway. RECORD sends nothing.");

  tft.setTextSize(2);
  tft.setCursor(8, 60);
  tft.printf("Reader: %s\n", readerOk ? "PN532 OK" : "NOT ANSWERING");
  tft.setCursor(8, 90);
  tft.println("Tag:");
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(8, 112);
  tft.println(lastUid[0] ? lastUid : "(none yet)");

  tft.setTextColor(colour, TFT_BLACK);
  tft.setCursor(8, 150);
  tft.println(status);

  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(8, 200);
  tft.printf("scans %lu   records %lu", (unsigned long)scans, (unsigned long)records);
  tft.setCursor(8, 216);
  tft.println("RECORD = trackball press or Enter");
}

static bool keyboardEnter() {
  if (Wire.requestFrom(KB_ADDR, (uint8_t)1) != 1) return false;
  int c = Wire.read();
  return c == '\r' || c == '\n';
}

static bool trackballPressed() {
  static bool was = false;
  bool now = digitalRead(PIN_TRACKBALL_PRESS) == LOW;
  bool edge = now && !was;
  was = now;
  return edge;
}

void setup() {
  pinMode(PIN_POWERON, OUTPUT);
  digitalWrite(PIN_POWERON, HIGH); // peripherals off without it on battery
  delay(100);

  // Keep the other SPI devices off the bus.
  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, HIGH);
  pinMode(PIN_RADIO_CS, OUTPUT);
  digitalWrite(PIN_RADIO_CS, HIGH);
  pinMode(PIN_TRACKBALL_PRESS, INPUT_PULLUP);

  Serial.begin(115200);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  tft.init();
  tft.setRotation(1); // landscape, as the mesh firmware sets it
  tft.setBrightness(160);

  Serial1.begin(115200, SERIAL_8N1, PIN_GROVE_RX, PIN_GROVE_TX);
  nfc.begin();
  uint32_t ver = nfc.getFirmwareVersion();
  readerOk = ver != 0;
  if (readerOk) {
    Serial.printf("PN5%02lX firmware %lu.%lu\n", (ver >> 24) & 0xFF, (ver >> 16) & 0xFF, (ver >> 8) & 0xFF);
    nfc.SAMConfig();
  } else {
    Serial.println("PN532 not answering: check HSU switches (both OFF), TX/RX crossed, 3V3 and GND");
  }
  show(readerOk ? "Hold a card to the reader" : "Check wiring", readerOk ? TFT_GREEN : TFT_RED);
}

void loop() {
  if (readerOk) {
    uint8_t uid[7];
    uint8_t len = 0;
    // Short timeout so the buttons stay responsive.
    if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &len, 100)) {
      char buf[sizeof(lastUid)];
      char *p = buf;
      for (uint8_t i = 0; i < len && i < 7; i++) p += sprintf(p, i ? ":%02X" : "%02X", uid[i]);
      bool fresh = strcmp(buf, lastUid) != 0 || millis() - lastSeenMs > 2000;
      lastSeenMs = millis();
      if (fresh) {
        strcpy(lastUid, buf);
        scans++;
        Serial.printf("SCAN %s\n", lastUid);
        show("Scanned. Press RECORD", TFT_CYAN);
      }
    }
  }

  if (trackballPressed() || keyboardEnter()) {
    if (lastUid[0]) {
      records++;
      Serial.printf("RECORD (not sent) tag=%s\n", lastUid);
      show("RECORD: not sent (spike)", TFT_ORANGE);
    } else {
      show("No tag scanned yet", TFT_RED);
    }
  }
  delay(20);
}
