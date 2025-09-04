#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include "esp32-hal-ledc.h"
#include "board_config.h"
#include "esp_http_server.h"
#include <stdarg.h>
#include <HTTPClient.h>
#include <math.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLEClient.h>

// ========= Wi-Fi =========
const char* ssid     = "AI_STICK";
const char* password = "12345678";

// ── 버튼 설정 ───────────────────────────────────────────────
#define BTN_ACTIVE_LOW   0
#define BTN_PULL_MODE    (BTN_ACTIVE_LOW ? INPUT_PULLUP : INPUT_PULLDOWN)
#define PIN_VIBE  3
#define PIN_BTN1  4
#define PIN_BTN2  5
#define PIN_BTN3  6

struct Btn { uint8_t pin; bool lastStable, lastRead; uint32_t tLast; uint32_t tDown; };
Btn btn1{PIN_BTN1, !BTN_ACTIVE_LOW, !BTN_ACTIVE_LOW, 0, 0};
Btn btn2{PIN_BTN2, !BTN_ACTIVE_LOW, !BTN_ACTIVE_LOW, 0, 0};
Btn btn3{PIN_BTN3, !BTN_ACTIVE_LOW, !BTN_ACTIVE_LOW, 0, 0};
const uint16_t DEBOUNCE_MS  = 35;
const uint16_t LONGPRESS_MS = 1000;

// ── 결제 상태 ───────────────────────────────────────────────
enum { PAY_IDLE=0, PAY_WAITING, PAY_SUCCESS, PAY_FAIL };
static uint8_t  g_payState = PAY_IDLE;
static bool     g_paid     = false;
static uint32_t g_payReqTs = 0;
const  uint32_t PAY_TIMEOUT_MS   = 5000;
static uint32_t g_btn1LastMs     = 0;
const  uint32_t BTN1_COOLDOWN_MS = 800;

// ── 기타 상태 ───────────────────────────────────────────────
static bool   g_phoneConnected = false;
static float  g_payMaxDistanceM = 0.7f;
static String pendingUploadUrl;

// ── 로깅(/serial, /logs) ────────────────────────────────────
#define LOG_BUF_LINES 200
static String s_logBuf[LOG_BUF_LINES];
static volatile uint16_t s_logHead=0;
static void logLine(const String& line){ Serial.println(line); s_logBuf[s_logHead]=line; s_logHead=(s_logHead+1)%LOG_BUF_LINES; }
static void logf(const char* fmt, ...){ char b[256]; va_list ap; va_start(ap,fmt); vsnprintf(b,sizeof(b),fmt,ap); va_end(ap); logLine(String(b)); }

// ── 진동 ────────────────────────────────────────────────────
static inline void vibeMax(uint16_t ms){ digitalWrite(PIN_VIBE, HIGH); delay(ms); digitalWrite(PIN_VIBE, LOW); }
static void vibeStrongN(uint8_t n){
  if (n <= 1){ vibeMax(500); return; }
  if (n == 2){ vibeMax(240); delay(60); vibeMax(240); return; }
  vibeMax(150); delay(50); vibeMax(150); delay(50); vibeMax(150);
}

// ── HTTP (/serial, /logs, /last.jpg, /vibe) ─────────────────
static httpd_handle_t g_httpd = NULL;
static esp_err_t logs_handler(httpd_req_t *req){
  httpd_resp_set_type(req,"text/plain; charset=utf-8");
  String out; out.reserve(4096);
  uint16_t idx=s_logHead;
  for(int i=0;i<LOG_BUF_LINES;++i){ const String& ln=s_logBuf[idx]; if(ln.length()){ out+=ln; out+="\n"; } idx=(idx+1)%LOG_BUF_LINES; }
  return httpd_resp_send(req,out.c_str(),out.length());
}
static const char* kSerialHTML =
"<!doctype html><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>"
"<title>ESP32 Serial(한글)</title><style>body{font-family:system-ui,Arial;margin:0}#log{white-space:pre-wrap;padding:10px}"
"header{background:#222;color:#fff;padding:8px 12px;position:sticky;top:0}</style>"
"<header>ESP32 무선 시리얼(/logs 1초 폴링)</header><div id=log></div>"
"<script>async function f(){try{let r=await fetch('/logs');if(!r.ok)return;let t=await r.text();"
"let e=document.getElementById('log');e.textContent=t;window.scrollTo(0,document.body.scrollHeight)}catch(e){}}"
"setInterval(f,1000);f();</script>";
static esp_err_t serial_handler(httpd_req_t *req){ httpd_resp_set_type(req,"text/html; charset=utf-8"); return httpd_resp_send(req,kSerialHTML,HTTPD_RESP_USE_STRLEN); }

static uint8_t* s_lastJpg = nullptr;
static size_t   s_lastJpgLen = 0;
static SemaphoreHandle_t s_jpgMtx;
static void freeLastJpg(){ if(s_lastJpg){ free(s_lastJpg); s_lastJpg=nullptr; s_lastJpgLen=0; } }
static esp_err_t lastjpg_handler(httpd_req_t *req){
  xSemaphoreTake(s_jpgMtx, portMAX_DELAY);
  if(!s_lastJpgLen){
    xSemaphoreGive(s_jpgMtx);
    httpd_resp_set_status(req,"404 NOT FOUND");
    return httpd_resp_send(req,"아직 사진이 없습니다. BTN3(촬영)을 눌러주세요.", HTTPD_RESP_USE_STRLEN);
  }
  httpd_resp_set_type(req,"image/jpeg");
  esp_err_t r=httpd_resp_send(req,(const char*)s_lastJpg,s_lastJpgLen);
  xSemaphoreGive(s_jpgMtx);
  return r;
}
static esp_err_t vibe_handler(httpd_req_t *req){
  int n = 1; size_t qlen = httpd_req_get_url_query_len(req) + 1;
  if (qlen > 1){
    char* qbuf=(char*)malloc(qlen);
    if (qbuf && httpd_req_get_url_query_str(req, qbuf, qlen) == ESP_OK){
      char val[16]; if (httpd_query_key_value(qbuf,"n",val,sizeof(val))==ESP_OK){ int t=atoi(val); if (t>=1) n=t; }
    }
    if (qbuf) free(qbuf);
  }
  if (n > 3) n = 3;
  vibeStrongN(n);
  logf("🌀 [웹] 강진동 %d회 실행(패턴)", n);
  httpd_resp_set_type(req,"text/plain; charset=utf-8");
  return httpd_resp_sendstr(req, "OK");
}
static void startLiteHttp(){
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = 80; cfg.max_uri_handlers = 8;
  if (httpd_start(&g_httpd, &cfg) == ESP_OK){
    httpd_uri_t u1={.uri="/logs",.method=HTTP_GET,.handler=logs_handler,.user_ctx=NULL};
    httpd_uri_t u2={.uri="/serial",.method=HTTP_GET,.handler=serial_handler,.user_ctx=NULL};
    httpd_uri_t u3={.uri="/last.jpg",.method=HTTP_GET,.handler=lastjpg_handler,.user_ctx=NULL};
    httpd_uri_t u4={.uri="/vibe",.method=HTTP_GET,.handler=vibe_handler,.user_ctx=NULL};
    httpd_register_uri_handler(g_httpd,&u1);
    httpd_register_uri_handler(g_httpd,&u2);
    httpd_register_uri_handler(g_httpd,&u3);
    httpd_register_uri_handler(g_httpd,&u4);
  }
}

// ── 카메라 ──────────────────────────────────────────────────
static bool captureAndStore(){
  camera_fb_t* fb = esp_camera_fb_get();
  if(!fb){ logLine("📷 [오류] 카메라 캡처 실패"); return false; }
  if(fb->format != PIXFORMAT_JPEG){ esp_camera_fb_return(fb); logLine("📷 [오류] JPEG 포맷이 아님"); return false; }
  xSemaphoreTake(s_jpgMtx, portMAX_DELAY);
  freeLastJpg();
  s_lastJpg = (uint8_t*)malloc(fb->len);
  if(!s_lastJpg){ xSemaphoreGive(s_jpgMtx); esp_camera_fb_return(fb); logLine("📷 [오류] 메모리 부족"); return false; }
  memcpy(s_lastJpg, fb->buf, fb->len); s_lastJpgLen = fb->len;
  xSemaphoreGive(s_jpgMtx);
  esp_camera_fb_return(fb);
  logf("📷 촬영 완료: %u 바이트 저장됨 (/last.jpg)", (unsigned)s_lastJpgLen);
  return true;
}

// ── BLE 서버 (앱 연동: NUS) + 바이너리 프레임 ───────────────
static const char* NUS_SVC_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char* NUS_RX_UUID  = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
static const char* NUS_TX_UUID  = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";
static BLEServer*          g_bleServer = nullptr;
static BLECharacteristic*  g_txChar    = nullptr;

#define TX_MIRROR_ASCII 0              // 바이너리 미러 텍스트 전송 비활성화(혼선 방지)

// TYPE 매핑
enum : uint8_t {
  T_PAY_SUCCESS      = 0x01,
  T_PAY_FAIL         = 0x02,
  T_PAY_TIMEOUT      = 0x03,
  T_PAY_BEACON_FOUND = 0x04,
  T_PAY_CONNECTED    = 0x05,
  T_PAY_REQ_SENT     = 0x06,
  T_APP_CONNECTED    = 0x10,
  T_APP_DISCONNECTED = 0x11,
  T_CAM_SHOT_SAVED   = 0x20,
  T_UPLOAD_DONE      = 0x21,
  T_UPLOAD_FAIL      = 0x22,
};

// 단순 type 전송 (페이로드 없음)
static void bleSendType(uint8_t type){
  if(!(g_phoneConnected && g_txChar)) return;
  uint8_t buf[1] = {type};
  g_txChar->setValue(buf, 1);
  g_txChar->notify();

}

// 페이로드가 있는 경우
static void bleSendTypeWithPayload(uint8_t type, const uint8_t* payload, uint8_t len){
  if(!(g_phoneConnected && g_txChar)) return;
  uint8_t buf[64]; uint8_t idx=0;
  buf[idx++]=type;
  for(uint8_t i=0;i<len;i++){ buf[idx++]=payload[i]; }
  g_txChar->setValue(buf, idx);
  g_txChar->notify();
  logf("📱 [BLE 전송] type=0x%02X (페이로드 길이=%d)", type, len); // 시리얼 로그 추가
}

// 32비트 값 전송
static void send_u32_type(uint8_t type, uint32_t v){
  uint8_t p[4]={ (uint8_t)(v&0xFF), (uint8_t)((v>>8)&0xFF), (uint8_t)((v>>16)&0xFF), (uint8_t)((v>>24)&0xFF) };
  bleSendTypeWithPayload(type, p, 4);
}

// RSSI 및 거리 전송
static void send_rssi_dist(int8_t rssi, uint16_t cm){
  uint8_t p[3]={ (uint8_t)rssi, (uint8_t)(cm&0xFF), (uint8_t)(cm>>8) };
  bleSendTypeWithPayload(T_PAY_BEACON_FOUND, p, 3);
}

class MyServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override { g_phoneConnected=true;  logLine("📱 [BLE] 휴대폰 연결됨");      bleSendType(T_APP_CONNECTED); }
  void onDisconnect(BLEServer* s) override { g_phoneConnected=false; logLine("📱 [BLE] 휴대폰 연결 해제"); bleSendType(T_APP_DISCONNECTED); s->getAdvertising()->start(); }
};

// ── 앱→보드(BLE) 명령 처리(문자열 수신만) ───────────────────
class PhoneRXCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    String cmd = String((const char*)c->getValue().c_str()); cmd.trim();
    logf("📱 [BLE 수신] %s", cmd.c_str());
    if (cmd.startsWith("VIBE:")) {
      int n = cmd.substring(5).toInt(); if (n < 1) n = 1; if (n > 3) n = 3;
      vibeStrongN(n);
    } else if (cmd.startsWith("PRESIGNED:")) {
      pendingUploadUrl = cmd.substring(strlen("PRESIGNED:"));
      logLine("☁️ 업로드 URL 수신(PRESIGNED)");
    } else if (cmd.startsWith("PAY_RANGE:")) {
      float m = cmd.substring(strlen("PAY_RANGE:")).toFloat();
      if (m<0.2f) m=0.2f; if (m>1.5f) m=1.5f; g_payMaxDistanceM = m;
      logf("💳 결제 허용 거리 변경: %.2fm", g_payMaxDistanceM);
    } else if (cmd == "PAY_CONFIRMED") {
      g_paid = true;
      logLine("💳 [결제] 앱으로부터 결제 완료 수신");
    }
  }
};

// ── 업로드 (PUT presigned) ──────────────────────────────────
static bool uploadToURL(uint8_t* data, size_t len, const String& url){
  if (!len) return false;
  WiFiClient *cli = new WiFiClient();
  HTTPClient http; bool ok=false;
  if (http.begin(*cli, url)) {
    http.addHeader("Content-Type","image/jpeg");
    int code = http.sendRequest("PUT", data, len);
    logf("☁️ 업로드 응답 코드: %d", code);
    ok = (code>=200 && code<300);
    http.end();
  } else { logLine("☁️ [오류] 업로드 초기화 실패(http.begin)"); }
  delete cli; return ok;
}

// ── PAYNODE(BLE Client) ─────────────────────────────────────
static const char* PAY_SVC_UUID  = "7B1485A0-7D0C-4C9E-8C4C-1E7F9950ABCD";
static const char* PAY_CHAR_UUID = "7B1485A1-7D0C-4C9E-8C4C-1E7F9950ABCD";
static const char* PAY_NODE_NAME = "PAYNODE";
static BLEClient*               g_payClient = nullptr;
static BLERemoteCharacteristic* g_payChar   = nullptr;

static float estimateDistanceM(int rssi, int txPower=-59, float n=2.0f){
  return powf(10.0f, ((float)txPower - (float)rssi) / (10.0f * n));
}

static void payFailToIdle(const char* reason, uint8_t typeCode){
  g_payState = PAY_FAIL; 
  bleSendType(typeCode);     // 0x02 실패 ㅋㅋ
  vibeStrongN(2);
  logf("💳 [결제] 타입코드ㅋㅋ: 0x%d", typeCode);
  logf("💳 [결제] 실패: %s", reason);
  if (g_payClient && g_payClient->isConnected()) g_payClient->disconnect();
  g_payReqTs = 0; g_payState = PAY_IDLE;
}

static void payNotifyCB(BLERemoteCharacteristic* c, uint8_t* data, size_t len, bool){
  String s; for(size_t i=0;i<len;++i) s += (char)data[i]; s.trim();
  if (s == "PAY_DONE") {
    g_paid = true; g_payState = PAY_SUCCESS;
    bleSendType(T_PAY_SUCCESS);     // 0x01
    logf("💳 [결제] 성공ㅋㅋ: %s", T_PAY_SUCCESS);
    logLine("💳 [결제] 비콘 응답: 승인(PAY_DONE)");
    g_payReqTs = 0; g_payState = PAY_IDLE;
  } else if (s == "PAY_TOO_FAR") {
    bleSendType(T_PAY_FAIL);        // 0x02
    logLine("💳 [결제] 비콘 응답: 거리 초과(PAY_TOO_FAR)");
    vibeStrongN(2);
    if (g_payClient && g_payClient->isConnected()) g_payClient->disconnect();
    g_payReqTs = 0; g_payState = PAY_IDLE;
  } else {
    logf("💳 [결제] 비콘 응답: %s", s.c_str());
  }
}

static BLEAdvertisedDevice* findPaynodeWithinRange(float maxM, int scanSec=3){
  BLEScan* scan = BLEDevice::getScan(); scan->setActiveScan(true);
  BLEScanResults* res = scan->start(scanSec, false);
  BLEAdvertisedDevice* found = nullptr; float best=999.0f;
  if(res){
    int cnt=res->getCount();
    for(int i=0;i<cnt;i++){
      BLEAdvertisedDevice d = res->getDevice(i);
      if (String(d.getName().c_str())==PAY_NODE_NAME){
        float dist = estimateDistanceM(d.getRSSI());
        if (dist<=maxM && dist<best){ if(found) delete found; found=new BLEAdvertisedDevice(d); best=dist; }
      }
    }
  }
  if(!found) logf("💳 [결제] 비콘 탐색 실패(%.2fm 이내 없음)", maxM);
  else {
    int rssi = found->getRSSI();
    float dm = estimateDistanceM(rssi);
    logf("💳 [결제] 비콘 발견: RSSI=%d  추정거리≈%.2fm", rssi, dm);
    uint16_t cm = (uint16_t)roundf(dm*100.0f);
    send_rssi_dist((int8_t)rssi, cm);   // 0x04
  }
  return found;
}

static bool ensurePayConnected(float maxM){
  if (g_payClient && g_payClient->isConnected()) return true;
  BLEAdvertisedDevice* dev = findPaynodeWithinRange(maxM);
  if(!dev) return false;
  g_payClient = BLEDevice::createClient();
  bool ok=g_payClient->connect(dev); delete dev;
  if(!ok){ logLine("💳 [결제] 비콘 연결 실패"); return false; }
  BLERemoteService* svc=g_payClient->getService(PAY_SVC_UUID);     if(!svc){ logLine("💳 [결제] 서비스 없음"); return false; }
  g_payChar = svc->getCharacteristic(PAY_CHAR_UUID);               if(!g_payChar){ logLine("💳 [결제] 캐릭터리스틱 없음"); return false; }
  if (g_payChar->canNotify()) {
    g_payChar->registerForNotify(payNotifyCB);
    BLERemoteDescriptor* cccd = g_payChar->getDescriptor(BLEUUID((uint16_t)0x2902));
    if (cccd) { uint8_t on[2]={0x01,0x00}; cccd->writeValue(on,2,true); }
  }
  logLine("💳 [결제] 비콘 연결 및 알림 구독 완료");

  // 사용자 요청: 구독 완료 시 결제 성공으로 간주
  g_paid = true; g_payState = PAY_SUCCESS;
  bleSendType(T_PAY_SUCCESS);            // 0x01
  vibeStrongN(1);                        // 성공 진동 (1회)
  logLine("💳 [결제] 구독 완료 → 성공으로 처리");
  g_payReqTs = 0; g_payState = PAY_IDLE;

  return true;
}

static void sendPayReq(){
  if (g_payState != PAY_WAITING) return;
  if (!ensurePayConnected(g_payMaxDistanceM)) { payFailToIdle("근처에 비콘 없음/연결 실패", T_PAY_FAIL); return; }
  if (g_payChar && g_payChar->canWrite()){
    const char* msg="PAY_REQ";
    g_payChar->writeValue((uint8_t*)msg, strlen(msg), true);
    logLine("💳 [결제] PAY_REQ 전송 완료");
   
    g_payReqTs = millis();
  } else { payFailToIdle("캐릭터리스틱 Write 불가", T_PAY_FAIL); }
}

// ── 버튼 처리 ───────────────────────────────────────────────
static inline bool readPressed(uint8_t pin){ int v = digitalRead(pin); return BTN_ACTIVE_LOW ? (v==LOW) : (v==HIGH); }
static inline bool debounce(Btn &b){
  bool cur = readPressed(b.pin);
  if (cur != b.lastRead){ b.lastRead=cur; b.tLast=millis(); }
  if ((millis()-b.tLast)>DEBOUNCE_MS && cur!=b.lastStable){ b.lastStable=cur; return true; }
  return false;
}

void BTN1_short(){
  uint32_t now = millis();
  if (now - g_btn1LastMs < BTN1_COOLDOWN_MS) { logLine("🖱️ [버튼1:결제] (무시) 연타 쿨다운 중"); return; }
  if (g_payState == PAY_WAITING) { logLine("🖱️ [버튼1:결제] (무시) 결제 진행 중"); return; }
  g_btn1LastMs = now;
  logf("🖱️ [버튼1:결제] 짧게 누름 → 허용거리 ≤ %.2fm", g_payMaxDistanceM);
  g_payState = PAY_WAITING;
  sendPayReq();
}

void BTN2_short(){
  // 필요하면 여기도 TYPE로 정의해서 보내세요.
}

void BTN3_short(){
  logLine("🖱️ [버튼3:촬영] 사진 촬영 시작");
  if(!captureAndStore()){ return; }
  send_u32_type(T_CAM_SHOT_SAVED, (uint32_t)s_lastJpgLen);   // 0x20
  if (pendingUploadUrl.length()){
    logLine("☁️ 업로드 시작(PRESIGNED URL)");
    bool ok=false; xSemaphoreTake(s_jpgMtx,portMAX_DELAY);
    ok=uploadToURL(s_lastJpg,s_lastJpgLen,pendingUploadUrl);
    xSemaphoreGive(s_jpgMtx);
    if(ok){ bleSendType(T_UPLOAD_DONE); vibeStrongN(1); logLine("☁️ 업로드 완료"); }
    else  { bleSendType(T_UPLOAD_FAIL); logLine("☁️ [오류] 업로드 실패"); }
    pendingUploadUrl="";
  }else{
    logLine("🔗 업로드 URL 미설정(PRESIGNED:...)");
  }
}

// ── setup / loop ────────────────────────────────────────────
void setup() {
  Serial.begin(115200); delay(150);
  s_jpgMtx = xSemaphoreCreateMutex();
  pinMode(PIN_VIBE,OUTPUT); digitalWrite(PIN_VIBE,LOW);
  pinMode(PIN_BTN1, BTN_PULL_MODE);
  pinMode(PIN_BTN2, BTN_PULL_MODE);
  pinMode(PIN_BTN3, BTN_PULL_MODE);

  vibeStrongN(2); logLine("🌀 부팅 자가진단: 강진동 2회 OK");

  // 카메라
  camera_config_t cfg;
  cfg.ledc_timer=LEDC_TIMER_1; cfg.ledc_channel=LEDC_CHANNEL_1;
  cfg.pin_d0=Y2_GPIO_NUM; cfg.pin_d1=Y3_GPIO_NUM; cfg.pin_d2=Y4_GPIO_NUM; cfg.pin_d3=Y5_GPIO_NUM;
  cfg.pin_d4=Y6_GPIO_NUM; cfg.pin_d5=Y7_GPIO_NUM; cfg.pin_d6=Y8_GPIO_NUM; cfg.pin_d7=Y9_GPIO_NUM;
  cfg.pin_xclk=XCLK_GPIO_NUM; cfg.pin_pclk=PCLK_GPIO_NUM; cfg.pin_vsync=VSYNC_GPIO_NUM; cfg.pin_href=HREF_GPIO_NUM;
  cfg.pin_sccb_sda=SIOD_GPIO_NUM; cfg.pin_sccb_scl=SIOC_GPIO_NUM;
#ifdef PWDN_GPIO_NUM
  cfg.pin_pwdn=PWDN_GPIO_NUM;
#else
  cfg.pin_pwdn=-1;
#endif
#ifdef RESET_GPIO_NUM
  cfg.pin_reset=RESET_GPIO_NUM;
#else
  cfg.pin_reset=-1;
#endif
  cfg.xclk_freq_hz=20000000; cfg.pixel_format=PIXFORMAT_JPEG; cfg.frame_size=FRAMESIZE_QVGA;
  cfg.grab_mode=CAMERA_GRAB_LATEST; cfg.fb_location=CAMERA_FB_IN_PSRAM; cfg.jpeg_quality=12; cfg.fb_count=2;
  if(!psramFound()){ cfg.frame_size=FRAMESIZE_SVGA; cfg.fb_location=CAMERA_FB_IN_DRAM; cfg.fb_count=1; }
  if(esp_camera_init(&cfg)!=ESP_OK) logLine("📷 [오류] 카메라 초기화 실패");

  // Wi-Fi & HTTP
  WiFi.begin(ssid,password); WiFi.setSleep(false);
  Serial.print("Wi-Fi 연결 중");
  while(WiFi.status()!=WL_CONNECTED){ delay(400); Serial.print("."); }
  Serial.printf("\n🌐 Wi-Fi 연결됨, IP: %s\n", WiFi.localIP().toString().c_str());
  startLiteHttp();

  // BLE 서버(앱용)
  BLEDevice::init("CANE-ESP");
  BLEDevice::setPower(ESP_PWR_LVL_P9);
  g_bleServer = BLEDevice::createServer();
  g_bleServer->setCallbacks(new MyServerCB());
  BLEService* nus = g_bleServer->createService(NUS_SVC_UUID);
  BLECharacteristic* rx = nus->createCharacteristic(NUS_RX_UUID, BLECharacteristic::PROPERTY_WRITE);
  g_txChar = nus->createCharacteristic(NUS_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  rx->setCallbacks(new PhoneRXCB());
  nus->start();
  g_bleServer->getAdvertising()->addServiceUUID(nus->getUUID());
  g_bleServer->getAdvertising()->start();
}

void loop() {
  if (debounce(btn1)){ if (btn1.lastStable) btn1.tDown=millis(); else { if (millis()-btn1.tDown < LONGPRESS_MS) BTN1_short(); } }
  if (debounce(btn2)){ if (btn2.lastStable) btn2.tDown=millis(); else { if (millis()-btn2.tDown < LONGPRESS_MS) BTN2_short(); } }
  if (debounce(btn3)){ if (btn3.lastStable) btn3.tDown=millis(); else { if (millis()-btn3.tDown < LONGPRESS_MS) BTN3_short(); } }

  if (g_payState == PAY_WAITING){
    if (g_payReqTs != 0 && (millis() - g_payReqTs > PAY_TIMEOUT_MS)){
      payFailToIdle("응답 타임아웃", T_PAY_TIMEOUT);   // 0x03
    }
  }
  delay(1);
}
