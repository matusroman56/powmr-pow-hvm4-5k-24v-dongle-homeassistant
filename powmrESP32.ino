#include <NimBLEDevice.h>
#include <NimBLEScan.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <cstring>
#include <Arduino.h>

const char* WIFI_SSID     = "YOUR SSID";
const char* WIFI_PASS     = "YOUR PASSWORD";
const char* MQTT_SERVER   = "192.168.4.250";
const int   MQTT_PORT     = 1883;
const char* MQTT_USER     = "esp32";
const char* MQTT_PASS_STR = "YOUR MQTT PASSWORD";
const char* TOPIC_BASE    = "powmr/inverter";
const char* BLE_MAC       = "Aa:Bb:Cc:Dd:Ee:Ff";
const char* SVC_UUID      = "0000ff00-0000-1000-8000-00805f9b34fb";
const char* CHAR_UUID     = "0000ff01-0000-1000-8000-00805f9b34fb";

const uint8_t CMD_QUERY[] = {0x7B,0x22,0x74,0x79,0x70,0x65,0x22,0x3A,0x22,0x73,0x65,0x72,0x69,0x61,0x6C,0x22,0x2C,0x22,0x66,0x75,0x6E,0x63,0x22,0x3A,0x22,0x61,0x75,0x74,0x6F,0x22,0x7D};
const size_t CMD_LEN      = sizeof(CMD_QUERY);
const int    POLL_MS      = 15000;
const int    SCAN_SEC     = 12;
const int    MAX_RETRIES  = 4;
const int    RETRY_DELAY  = 1200;

WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);
NimBLEClient* bleClient = nullptr;
NimBLERemoteCharacteristic* pChar = nullptr;
bool bleInitialized = false;
std::vector<int> lastJsonVals;
std::vector<int> lastNotifyVals;

std::vector<int> parseCSV(const char* p) {
  std::vector<int> vals;
  std::string token;
  while (*p && *p != '"' && *p != '}') {
    char c = *p++;
    if (c == ',') {
      if (!token.empty()) { vals.push_back(strtol(token.c_str(), nullptr, 16)); token.clear(); }
    } else if ((c>='0'&&c<='9')||(c>='a'&&c<='f')||(c>='A'&&c<='F')) {
      token += c;
    }
  }
  if (!token.empty()) vals.push_back(strtol(token.c_str(), nullptr, 16));
  return vals;
}

void publishValues(std::vector<int>& j, std::vector<int>& n) {
  auto u16j = [&](int i) -> int { return j[i]+j[i+1]*256; };
  auto u16n = [&](int i) -> int { return n[i]+n[i+1]*256; };
  auto s16n = [&](int i) -> int { int r=u16n(i); return r>32767?r-65536:r; };

  char topic[64], val[16];
  auto pub = [&](const char* name, float v, int dec) {
    snprintf(topic, sizeof(topic), "%s/%s", TOPIC_BASE, name);
    dtostrf(v, 1, dec, val);
    mqtt.publish(topic, val, true);
    Serial.printf("  %s = %s\n", topic, val);
  };

  Serial.println("\n--- Publikujem spracované dáta do MQTT ---");

  // 1. FOTOVOLTIKA (Rámec N - Notify)
  float pvW_real = 0.0f;
  float pvV_real = 0.0f;

  if ((int)n.size() > 37) {
    // PV Watty - Index 36 ukázal čistú zhodu s celkovým výkonom panelov
    pvW_real = (float)u16n(36);
    pub("pv_watt", pvW_real, 0);
    pub("pv_power", pvW_real, 0);       // Záložný topik pre HA kartu (Produkcia)
  }

  if ((int)n.size() > 33) {
    // PV Napätie - Index 32
    pvV_real = u16n(32) / 10.0f;
    pub("pv_voltage", pvV_real, 1);
  }

  // Výpočet reálneho prúdu z panelov (Ochrana pred zaseknutou hodnotou 0.4A)
  if (pvV_real > 10.0f && pvW_real > 0.0f) {
    float pvI_calc = pvW_real / pvV_real;
    pub("pv_current", pvI_calc, 2);
  } else {
    pub("pv_current", 0.0f, 1);
  }

  // 2. BATÉRIA (Rámec N - Notify)
  if ((int)n.size() > 27) {
    // Napätie batérie - Index 24
    float batV = u16n(24) / 100.0f;
    pub("battery_voltage", batV, 2);
    
    // Prúd batérie - OPRAVENÉ NA INDEX 26 na základe úspešného detektíva
    float batI = s16n(26) / 10.0f;
    pub("battery_current", batI, 1);
  }

  // 3. MENIČ A ODBER DOMU (Rámec J - JSON)
  if ((int)j.size() > 59) {
    // AC Napätie a Frekvencia meniča
    pub("inverter_voltage",   u16j(50) / 10.0f,  1);
    pub("inverter_frequency", u16j(54) / 100.0f, 2);
    
    // Odber domu (Záťaž) - Index 58 z JSON rámca, odstráni stav "Neznámy"
    float loadW = (float)u16j(58);
    pub("load_watt", loadW, 0);
    pub("pv_odber", loadW, 0);          // Záložný topik pre HA kartu (Odber)
  }

  Serial.println("-----------------------------------------\n");
}

void parseJsonFrame(const std::string& raw) {
  const char* p = strstr(raw.c_str(), "-0\":\"");
  if (!p) { Serial.println("JSON: nenajdeny"); return; }
  p += 5;
  lastJsonVals = parseCSV(p);
  Serial.printf("JSON: %d B\n", (int)lastJsonVals.size());
  if (!lastNotifyVals.empty()) publishValues(lastJsonVals, lastNotifyVals);
}

void parseNotifyFrame(const std::string& raw) {
  lastNotifyVals = parseCSV(raw.c_str());
  Serial.printf("Notify: %d B\n", (int)lastNotifyVals.size());
  if (!lastJsonVals.empty()) publishValues(lastJsonVals, lastNotifyVals);
}

void parseData(const std::string& raw) {
  if (raw.empty()) return;
  if (raw[0] == '{') parseJsonFrame(raw);
  else parseNotifyFrame(raw);
}

void notifyCB(NimBLERemoteCharacteristic* chr, uint8_t* data, size_t len, bool notify) {
  std::string str((char*)data, len);
  parseData(str);
}

bool requestData() {
  if (!bleClient || !bleClient->isConnected() || !pChar) return false;
  Serial.println("Posielam prikaz...");
  pChar->writeValue(CMD_QUERY, CMD_LEN, true);
  delay(1500);
  std::string resp = pChar->readValue();
  if (!resp.empty()) parseData(resp);
  return true;
}

bool bleConnect() {
  if (!bleInitialized) {
    NimBLEDevice::init("ESP32-PowMr");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
    bleInitialized = true;
  }
  if (bleClient) {
    if (bleClient->isConnected()) bleClient->disconnect();
    NimBLEDevice::deleteClient(bleClient);
    bleClient = nullptr;
    pChar = nullptr;
    delay(600);
  }

  Serial.println("Skenujem BLE...");
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->clearResults();
  scan->setActiveScan(true);
  scan->setInterval(300);
  scan->setWindow(280);
  scan->setDuplicateFilter(false);
  NimBLEScanResults res = scan->start(SCAN_SEC);
  scan->stop();

  NimBLEAdvertisedDevice* dev = nullptr;
  for (int i = 0; i < res.getCount(); i++) {
    NimBLEAdvertisedDevice d = res.getDevice(i);
    String mac = d.getAddress().toString().c_str();
    if (mac.equalsIgnoreCase(BLE_MAC)) {
      dev = new NimBLEAdvertisedDevice(d);
      Serial.println("Dongle najdeny");
    }
  }
  if (!dev) { Serial.println("Dongle nevidim"); return false; }

  bleClient = NimBLEDevice::createClient();
  bleClient->setConnectionParams(90, 90, 0, 2500);

  bool connected = false;
  for (int i = 0; i < MAX_RETRIES; i++) {
    if (bleClient->connect(dev)) { connected = true; break; }
    delay(RETRY_DELAY);
  }
  delete dev;

  if (!connected) { Serial.println("Pripojenie zlyhalo"); return false; }
  Serial.println("BLE OK!");

  NimBLERemoteService* svc = bleClient->getService(SVC_UUID);
  if (!svc) { bleClient->disconnect(); return false; }
  pChar = svc->getCharacteristic(CHAR_UUID);
  if (!pChar) { bleClient->disconnect(); return false; }
  if (pChar->canNotify()) pChar->subscribe(true, notifyCB);
  return true;
}

void reconnectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  for (int i = 0; i < 12 && WiFi.status() != WL_CONNECTED; i++) delay(500);
  if (WiFi.status() == WL_CONNECTED)
    Serial.printf("WiFi OK: %s\n", WiFi.localIP().toString().c_str());
}

void reconnectMQTT() {
  if (mqtt.connected()) return;
  if (mqtt.connect("ESP32-PowMr", MQTT_USER, MQTT_PASS_STR))
    Serial.println("MQTT OK");
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("=== PowMr PV Monitor ===");
  reconnectWiFi();
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setBufferSize(2048);
}

unsigned long lastPoll = 0;

void loop() {
  reconnectWiFi();
  reconnectMQTT();
  mqtt.loop();
  unsigned long now = millis();
  if (now - lastPoll >= POLL_MS) {
    lastPoll = now;
    if (!requestData()) bleConnect();
  }
  delay(100);
}
