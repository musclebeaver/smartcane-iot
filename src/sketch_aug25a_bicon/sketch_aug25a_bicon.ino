/*  PAYNODE (Beacon) for XIAO ESP32S3 Sense
    - No buttons / no vibration / no Wi-Fi
    - BLE GATT Server advertising a payment service.
    - Every second scans for MAIN board ("CANE-ESP") and logs distance by RSSI.
    - When MAIN writes "PAY_REQ", approve only if:
        recently seen (<= VALID_SEEN_MS) AND distance <= g_allowDistM (default 0.7 m).
    - Notifications:
        "PAY_DONE"     (approved)
        "PAY_TOO_FAR"  (rejected)
        "PONG"
        "RANGE_OK x.xx"
    - Change range at runtime: write "PAY_RANGE:0.7"
*/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <math.h>

// ---------- IDs / Names ----------
static const char* MAIN_NAME     = "CANE-ESP";   // main board BLE name
static const char* PAYNODE_NAME  = "PAYNODE";    // this board's advertising name

// Payment service UUIDs (must match main board)
static const char* PAY_SVC_UUID  = "7B1485A0-7D0C-4C9E-8C4C-1E7F9950ABCD";
static const char* PAY_CHAR_UUID = "7B1485A1-7D0C-4C9E-8C4C-1E7F9950ABCD";

// ---------- Proximity gate ----------
static float    g_allowDistM     = 0.7f;     // ★ 기본 허용 거리 0.7 m
static int      g_lastMainRSSI   = -127;     // last seen RSSI
static float    g_lastMainDist   = 999.f;    // last estimated distance
static uint32_t g_lastMainSeenMs = 0;        // last time we saw MAIN
static const uint32_t VALID_SEEN_MS = 10000; // seen within 10s considered "recent"

// RSSI -> distance approximation (txPower=-59 dBm @1m, n=2)
static float estimateDistanceM(int rssi, int txPower=-59, float n=2.0f){
  return powf(10.0f, ((float)txPower - (float)rssi) / (10.0f * n));
}

// ---------- BLE server/char ----------
static BLEServer*         g_srv   = nullptr;
static BLECharacteristic* g_char  = nullptr;
static bool               g_centralConnected = false;

class SrvCB : public BLEServerCallbacks{
  void onConnect(BLEServer*) override {
    g_centralConnected = true;
    Serial.println("[PAYNODE] central connected");
  }
  void onDisconnect(BLEServer* s) override {
    g_centralConnected = false;
    Serial.println("[PAYNODE] central disconnected");
    s->getAdvertising()->start(); // keep advertising
  }
};

static void reply(const String& s){
  if(g_char){
    g_char->setValue((uint8_t*)s.c_str(), s.length());
    g_char->notify();
  }
}

class PayRXCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    String cmd = String((const char*)c->getValue().c_str()); cmd.trim();
    Serial.print("[PAYNODE RX] "); Serial.println(cmd);

    if (cmd == "PING") {
      reply("PONG");
    }
    else if (cmd.startsWith("PAY_RANGE:")){
      float m = cmd.substring(strlen("PAY_RANGE:")).toFloat();
      if (m < 0.2f) m = 0.2f; if (m > 1.5f) m = 1.5f;
      g_allowDistM = m;
      reply(String("RANGE_OK ") + String(g_allowDistM, 2));
      Serial.printf("[PAYNODE] allow distance set to %.2fm\n", g_allowDistM);
    }
    else if (cmd == "PAY_REQ") {
      // recent & close check
      uint32_t now = millis();
      bool recent = (now - g_lastMainSeenMs) <= VALID_SEEN_MS;
      bool close  = (g_lastMainDist <= g_allowDistM);

      Serial.printf("[PAYNODE] PAY_REQ: recent=%d  dist=%.2fm  limit=%.2fm\n",
                    recent?1:0, g_lastMainDist, g_allowDistM);

      if (recent && close) {
        reply("PAY_DONE");
        Serial.println("[PAYNODE] -> PAY_DONE");
      } else {
        reply("PAY_TOO_FAR");
        Serial.println("[PAYNODE] -> PAY_TOO_FAR");
      }
    }
  }
};

// ---------- Scan for MAIN (logs every hit) ----------
static uint32_t g_nextScanMs = 0;

static void scanOnceForMain(){
  BLEScan* scan = BLEDevice::getScan();
  scan->setActiveScan(true);

  // Use pointer-return variant for broad compatibility
  BLEScanResults* res = scan->start(1 /*sec*/, false);
  if (!res) return;

  int cnt = res->getCount();
  bool seen = false;

  for (int i=0; i<cnt; i++){
    BLEAdvertisedDevice d = res->getDevice(i);

    // normalize to Arduino String for compatibility
    String nm = String(d.getName().c_str());
    if (nm.length() == 0) continue;

    if (nm == MAIN_NAME){
      seen = true;
      g_lastMainRSSI   = d.getRSSI();
      g_lastMainDist   = estimateDistanceM(g_lastMainRSSI);
      g_lastMainSeenMs = millis();

      // ★ 실시간 로그: 거리/허용거리/판정(RSSI 포함)
      Serial.printf("[PAYNODE] MAIN: %.2fm (limit=%.2fm)  %s  RSSI=%d\n",
                    g_lastMainDist, g_allowDistM,
                    (g_lastMainDist <= g_allowDistM ? "NEAR" : "FAR"),
                    g_lastMainRSSI);
    }
  }

  // (옵션) 못 봤을 때도 로그를 남기고 싶으면 주석 해제
  // if (!seen) Serial.println("[PAYNODE] MAIN not seen");
}

// ---------- setup / loop ----------
void setup(){
  Serial.begin(115200);
  delay(100);

  BLEDevice::init(PAYNODE_NAME);
  BLEDevice::setPower(ESP_PWR_LVL_P9);

  g_srv = BLEDevice::createServer();
  g_srv->setCallbacks(new SrvCB());

  BLEService* svc = g_srv->createService(PAY_SVC_UUID);
  g_char = svc->createCharacteristic(
    PAY_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
  );
  g_char->setCallbacks(new PayRXCB());
  svc->start();

  BLEAdvertising* adv = g_srv->getAdvertising();
  adv->addServiceUUID(PAY_SVC_UUID);
  adv->setScanResponse(true);
  adv->start();

  // Scanner parameters
  BLEDevice::getScan()->setInterval(320);
  BLEDevice::getScan()->setWindow(160);

  Serial.printf("[PAYNODE] ready. Advertising payment service. allowDist=%.2fm\n", g_allowDistM);
}

void loop(){
  // scan & log every second
  uint32_t now = millis();
  if (now >= g_nextScanMs){
    g_nextScanMs = now + 1000;
    scanOnceForMain();
  }
  delay(5);
}
