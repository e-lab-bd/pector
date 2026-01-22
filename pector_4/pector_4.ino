/*
::::::::::::::Author:::::::::::::::
:: @MD. Touhiduzzaman Turja      ::
:: - Jan 22, 2026 (V0.4.26.1.0)  ::
:::::::::::::::::::::::::::::::::::
*/

/* CONFIG */
#define EMULATE_MODBUS 0  // <<< 1 = EMULATED, 0 = REAL RS485 MODBUS

#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <stdarg.h>
#include <string.h>

#if !EMULATE_MODBUS
#include <ModbusMaster.h>
#include <HardwareSerial.h>
#endif

/* PRODUCT */
static const char* PRODUCT_NAME = "PECTOR";

/* WIFI / MDNS */
static const char* WIFI_SSID = "E LAB PRIME";
static const char* WIFI_PASSWORD = "@20140110";

static const char* MDNS_NAME = "pector_4";
static const uint16_t TCP_PORT = 3134;

/* OTA */
static const char* OTA_PASSWORD = "pector1234";
volatile bool otaInProgress = false;
static bool otaReady = false;

/* MODE */
enum class RunMode : uint8_t {
  Normal,
  Ota
};
static volatile RunMode runMode = RunMode::Normal;

/* LCD */
static constexpr uint8_t LCD_COLS = 16;
static constexpr uint8_t LCD_ROWS = 2;
LiquidCrystal_I2C lcd(0x27, LCD_COLS, LCD_ROWS);

/* TCP */
WiFiServer tcpServer(TCP_PORT);
WiFiClient tcpClient;
static bool tcpReady = false;

static unsigned long lastTcpActivity = 0;
static constexpr unsigned long TCP_IDLE_TIMEOUT_MS = 30000;

static char tcpLineBuf[64];
static uint8_t tcpLineLen = 0;

/* FLAGS */
static bool wifiReady = false;
static bool mdnsReady = false;

/* MODBUS */
#if !EMULATE_MODBUS
#define RS485_CONTROL 14
ModbusMaster node;
HardwareSerial modbusSerial(2);

static void preTransmission() {
  digitalWrite(RS485_CONTROL, HIGH);
}
static void postTransmission() {
  digitalWrite(RS485_CONTROL, LOW);
}
#endif

/* MODBUS UI/STATE */
static bool modbusFaultActive = false;
static uint8_t lastModbusErr = 0;

/* LOGGING (Serial + TCP) */
static void logf(const char* fmt, ...) {
  char msg[192];

  va_list ap;
  va_start(ap, fmt);
  vsnprintf(msg, sizeof(msg), fmt, ap);
  va_end(ap);

  Serial.println(msg);

  if (tcpClient && tcpClient.connected()) {
    tcpClient.print("# ");
    tcpClient.print(msg);
    tcpClient.print('\n');
    lastTcpActivity = millis();
  }
}

/* LCD HELPERS */
static void lcdWriteLine(uint8_t row, const char* text) {
  char line[LCD_COLS + 1];
  snprintf(line, sizeof(line), "%-16s", (text ? text : ""));
  lcd.setCursor(0, row);
  lcd.print(line);
}

static void lcdShowSplash() {
  lcd.clear();
  const uint8_t nameLen = (uint8_t)strlen(PRODUCT_NAME);
  const uint8_t col = (nameLen < LCD_COLS) ? (uint8_t)((LCD_COLS - nameLen) / 2) : 0;
  lcd.setCursor(col, 0);
  lcd.print(PRODUCT_NAME);
  lcdWriteLine(1, "");
}

static void lcdShowHeader() {
  lcdWriteLine(0, "MOIST| EC | PH ");
}

static void lcdShowValues(uint16_t moist, uint16_t ec, uint16_t ph) {
  char line[LCD_COLS + 1];
  memset(line, ' ', LCD_COLS);
  line[LCD_COLS] = '\0';

  char tmp[8];
  // Moist at col 0 width 4
  snprintf(tmp, sizeof(tmp), "%4u", moist);
  memcpy(line + 0, tmp, 4);
  // EC at col 6 width 4
  snprintf(tmp, sizeof(tmp), "%4u", ec);
  memcpy(line + 6, tmp, 4);
  // pH at col 12 width 3
  snprintf(tmp, sizeof(tmp), "%3u", ph);
  memcpy(line + 12, tmp, 3);

  lcd.setCursor(0, 1);
  lcd.print(line);
}

static void lcdShowModbusError(uint8_t errCode) {
  // ONLY modbus related issues are allowed on LCD
  lcdShowHeader();

  char msg[LCD_COLS + 1];
  // Fits within 16 columns: "MB ERR 0xXX"
  snprintf(msg, sizeof(msg), "MB ERR 0x%02X", errCode);
  lcdWriteLine(1, msg);
}

/* TCP SERVER HELPERS */
static void stopTcpServer() {
  if (tcpClient) {
    tcpClient.stop();
  }
  if (tcpReady) {
    tcpServer.end();
    tcpReady = false;
    logf("[TCP] Server stopped");
  }
}

static void startTcpServer() {
  if (tcpReady) return;
  tcpServer.begin();
  tcpServer.setNoDelay(true);
  tcpReady = true;
  logf("[TCP] Server started on port %u", (unsigned)TCP_PORT);
}

/* MDNS */
static void startMdnsAndServices() {
  if (mdnsReady) return;

  MDNS.end();
  if (MDNS.begin(MDNS_NAME)) {
    MDNS.addService("pector", "tcp", TCP_PORT);
    MDNS.addService("arduino", "tcp", 3232);
    mdnsReady = true;
    logf("[mDNS] Started as %s.local", MDNS_NAME);
  } else {
    logf("[mDNS] Failed to start");
  }
}

/* OTA */
static void startOta() {
  if (otaReady) return;

  ArduinoOTA.setHostname(MDNS_NAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA.onStart([]() {
    otaInProgress = true;
    logf("[OTA] Start");
  });

  ArduinoOTA.onEnd([]() {
    logf("[OTA] End");
    otaInProgress = false;
  });

  ArduinoOTA.onError([](ota_error_t e) {
    logf("[OTA] Error %u", (unsigned)e);
    otaInProgress = false;
  });

  ArduinoOTA.begin();
  otaReady = true;
  logf("[OTA] Ready (port 3232)");
}

/* WIFI / SERVICE */
static void ensureWiFiAndServices() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiReady) {
      wifiReady = true;

      logf("[WiFi] Connected. IP: %s", WiFi.localIP().toString().c_str());

      startMdnsAndServices();

      if (runMode == RunMode::Normal) {
        startTcpServer();
      } else {
        stopTcpServer();
        startOta();
      }
    }
    return;
  }

  // Lost WiFi after connect
  if (wifiReady) {
    wifiReady = false;
    mdnsReady = false;
    otaReady = false;

    stopTcpServer();
    MDNS.end();

    logf("[WiFi] Disconnected. Reconnecting...");
  }

  static unsigned long lastTry = 0;
  if (millis() - lastTry > 3000) {
    lastTry = millis();
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.setHostname(MDNS_NAME);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
}

/* MODE SWITCH */
static void enterOtaMode() {
  if (runMode == RunMode::Ota) return;

  logf("[MODE] OTA only");
  runMode = RunMode::Ota;

  stopTcpServer();
  otaReady = false;
  otaInProgress = false;

  if (WiFi.status() == WL_CONNECTED) {
    startOta();
  }
}

/* TCP CLIENT HANDLING */
static void tcpSendGreeting() {
  if (!(tcpClient && tcpClient.connected())) return;
  tcpClient.print("# PECTOR online\n");
  tcpClient.printf("# Host: %s.local\n", MDNS_NAME);
  tcpClient.printf("# Mode: %s\n", (runMode == RunMode::Normal) ? "NORMAL" : "OTA");
  tcpClient.printf("# IP: %s\n", WiFi.localIP().toString().c_str());
  lastTcpActivity = millis();
}

static void handleTcpClient() {
  if (runMode != RunMode::Normal) return;
  if (otaInProgress) return;
  if (WiFi.status() != WL_CONNECTED) return;

  if (tcpClient && tcpClient.connected()) {
    if (millis() - lastTcpActivity > TCP_IDLE_TIMEOUT_MS) {
      logf("[TCP] Client timeout (idle)");
      tcpClient.stop();
      tcpLineLen = 0;
    }
  }

  if (!tcpClient || !tcpClient.connected()) {
    WiFiClient c = tcpServer.available();
    if (c) {
      tcpClient = c;
      tcpClient.setNoDelay(true);
      lastTcpActivity = millis();
      tcpLineLen = 0;

      logf("[TCP] Client connected");
      tcpSendGreeting();
    }
  }

  while (tcpClient && tcpClient.available()) {
    char c = (char)tcpClient.read();
    lastTcpActivity = millis();

    if (c == '\r') continue;

    if (c == '\n') {
      tcpLineBuf[tcpLineLen] = '\0';
      if (!strcmp(tcpLineBuf, "ota_init")) {
        tcpClient.print("OK:OTA\n");
        tcpClient.flush();
        tcpClient.stop();
        enterOtaMode();
      } else if (tcpLineLen > 0) {
        logf("[TCP] Unknown cmd: %s", tcpLineBuf);
      }

      tcpLineLen = 0;
    } else if (tcpLineLen < sizeof(tcpLineBuf) - 1) {
      tcpLineBuf[tcpLineLen++] = c;
    } else {
      tcpLineLen = 0;
      logf("[TCP] Line overflow (discarded)");
    }
  }
}

static void sendEcOverTcp(uint16_t moist_tcp, uint16_t ec_tcp, uint16_t ph_tcp, uint16_t temp_tcp) {
  if (tcpClient && tcpClient.connected()) {
    tcpClient.printf("MO=%u EC=%u pH=%u TMP=%u\n", moist_tcp, ec_tcp, ph_tcp, temp_tcp);
    lastTcpActivity = millis();
  }
}

/* MODBUS READ */
static bool readModbusValues(uint16_t& moist, uint16_t& ec, uint16_t& ph, uint16_t& temp, uint8_t& errOut) {
#if EMULATE_MODBUS
  (void)errOut;

  static uint16_t em = 400, ee = 1000, ep = 65, et = 250;
  em = (em + 5) % 1000;
  ee = (ee + 13) % 3000;
  ep = 60 + (millis() / 1000) % 20;
  et = 200 + (millis() / 1000) % 80;

  moist = em;
  ec = ee;
  ph = ep;
  temp = et;
  return true;
#else
  uint8_t res = node.readHoldingRegisters(0, 4);
  if (res != node.ku8MBSuccess) {
    errOut = res;
    return false;
  }

  temp = node.getResponseBuffer(0);
  moist = node.getResponseBuffer(1);
  ec = node.getResponseBuffer(2);
  ph = node.getResponseBuffer(3);
  errOut = node.ku8MBSuccess;
  return true;
#endif
}

static void onModbusOk(uint16_t moist, uint16_t ec, uint16_t ph) {
  if (modbusFaultActive) {
    modbusFaultActive = false;
    logf("[Modbus] Recovered");
    lcdShowHeader();
  }
  lcdShowValues(moist, ec, ph);
}

static void onModbusError(uint8_t errCode) {
  const bool isNew = (!modbusFaultActive) || (errCode != lastModbusErr);
  modbusFaultActive = true;
  lastModbusErr = errCode;
  if (isNew) {
    logf("[Modbus] ERR 0x%02X", errCode);
  }
  lcdShowModbusError(errCode);
}

void setup() {
  Serial.begin(115200);
  Serial.println("\nBooting...");

#if !EMULATE_MODBUS
  pinMode(RS485_CONTROL, OUTPUT);
  digitalWrite(RS485_CONTROL, LOW);

  modbusSerial.begin(9600, SERIAL_8N1, 16, 17);
  node.begin(0x12, modbusSerial);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  logf("[Modbus] REAL mode (RS485)");
#else
  logf("[Modbus] EMULATION mode");
#endif

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setHostname(MDNS_NAME);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Wire.begin(18, 19);
  lcd.init();
  lcd.backlight();

  lcdShowSplash();
  delay(1000);

  lcdShowHeader();
  lcdWriteLine(1, "");
}

void loop() {
  ensureWiFiAndServices();

  if (otaReady) {
    ArduinoOTA.handle();
  }

  if (runMode == RunMode::Ota) {
    delay(1);
    return;
  }

  handleTcpClient();

  static unsigned long lastPoll = 0;
  if (millis() - lastPoll >= 2000) {
    lastPoll = millis();

    uint16_t moist = 0, ec = 0, ph = 0, temp = 0;
    uint8_t mbErr = 0;

    if (readModbusValues(moist, ec, ph, temp, mbErr)) {
      Serial.printf("MO=%u EC=%u pH=%u TMP=%u\n", moist, ec, ph, temp);
      onModbusOk(moist, ec, ph);
      sendEcOverTcp(moist, ec, ph, temp);
    } else {
      onModbusError(mbErr);
    }
  }

  delay(1);
}
