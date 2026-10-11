/***************************************************************************************
 *  XIAO ESP32S3 Sense + OV3660  —  "one photo every 10 s" web server
 *
 *  - A FreeRTOS task on core 0 grabs a JPEG every CAPTURE_INTERVAL_MS and
 *    stores it in PSRAM. Photos are taken even if no client is connected.
 *  - Soft-AP mode (default): join WiFi "XIAO-Cam", open http://192.168.4.1/
 *    Set USE_SOFT_AP to 0 to join your home WiFi instead (IP on Serial).
 *
 *  Endpoints:  /  dashboard   /capture  newest JPEG   /status  JSON
 *              /snap  capture immediately   /stream  MJPEG (1 client at a time)
 *
 *  Arduino IDE settings (important!):
 *    Board:            XIAO_ESP32S3
 *    PSRAM:            OPI PSRAM            <-- required
 *    Flash Size:       8MB
 *    USB CDC On Boot:  Enabled (for Serial output)
 ***************************************************************************************/

#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

/* ------------------- user settings ------------------- */
#define USE_SOFT_AP          1                // 1 = own hotspot, 0 = join home WiFi
const char *AP_SSID  = "XIAO-Cam";
const char *AP_PASS  = "cam12345";            // WPA2, >= 8 chars (nullptr = open AP)
const char *STA_SSID = "YourWiFiName";
const char *STA_PASS = "YourWiFiPassword";

#define CAPTURE_INTERVAL_MS  10000UL          // <-- one photo every 10 s
#define FRAME_SIZE           FRAMESIZE_UXGA   // OV3660: QXGA/UXGA/SXGA/XGA/SVGA/VGA...
#define JPEG_QUALITY         12               // 10 = best/biggest ... 63 = smallest

#define MAX_JPEG_BYTES       (512UL * 1024UL) // safety cap for the stored photo
#define HEARTBEAT_LED_PIN    21               // on-board LED blinks per photo (-1 = off)

/* ------- XIAO ESP32S3 Sense camera pins (Seeed wiki) ------- */
#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM   10
#define SIOD_GPIO_NUM   40                    // SCCB/I2C SDA
#define SIOC_GPIO_NUM   39                    // SCCB/I2C SCL
#define Y9_GPIO_NUM     48
#define Y8_GPIO_NUM     11
#define Y7_GPIO_NUM     12
#define Y6_GPIO_NUM     14
#define Y5_GPIO_NUM     16
#define Y4_GPIO_NUM     18
#define Y3_GPIO_NUM     17
#define Y2_GPIO_NUM     15
#define VSYNC_GPIO_NUM  38
#define HREF_GPIO_NUM   47
#define PCLK_GPIO_NUM   13

/* ------------------- globals ------------------- */
WebServer server(80);

static uint8_t *jpegBuf = nullptr;            // PSRAM: newest photo
static size_t   jpegLen = 0;
static volatile uint32_t frameCount = 0;
static uint32_t frameMs  = 0;
static uint16_t frameW = 0, frameH = 0;
static SemaphoreHandle_t frameMutex = nullptr;
static EventGroupHandle_t evtGroup  = nullptr;
#define EVT_SNAP (1UL << 0)

/* ------------------- the web page (same as index.html) ------------------- */
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>XIAO ESP32S3 Sense · Cam</title>
<style>
*{box-sizing:border-box}
body{margin:0;font-family:system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;
     background:#0f1115;color:#e8eaf0;min-height:100vh;display:flex;flex-direction:column}
header{display:flex;align-items:center;justify-content:space-between;gap:10px;
       padding:10px 14px;background:#171a21;border-bottom:1px solid #262b36}
h1{font-size:1rem;margin:0;font-weight:600;white-space:nowrap}
#connbox{display:flex;align-items:center;gap:6px;font-size:.78rem;color:#9aa3b5}
#dot{width:9px;height:9px;border-radius:50%;background:#f0b429}
#dot.ok{background:#31c48d}#dot.bad{background:#e74c3c}
main{flex:1;display:flex;flex-direction:column;align-items:center;gap:12px;padding:14px}
#frame{position:relative;width:min(100%,640px);aspect-ratio:4/3;max-height:68vh;
       background:#000;border:1px solid #262b36;border-radius:10px;overflow:hidden}
#frame.empty::after{content:'waiting for the first photo...';position:absolute;inset:0;
       display:flex;align-items:center;justify-content:center;color:#8b94a6;font-size:.9rem}
#photo{display:block;width:100%;height:100%;object-fit:contain}
#controls{display:flex;gap:8px;flex-wrap:wrap;justify-content:center}
button{background:#2b3444;color:#e8eaf0;border:1px solid #3a4356;border-radius:8px;
       padding:9px 14px;font-size:.9rem;cursor:pointer}
button:active{transform:scale(.97)}
#btnSnap{background:#3b6fd4;border-color:#3b6fd4}
.hint{font-size:.75rem;color:#8b94a6;margin:0;text-align:center}
footer{padding:8px 12px;background:#171a21;border-top:1px solid #262b36;font-size:.75rem;
       color:#8b94a6;display:flex;gap:14px;flex-wrap:wrap;justify-content:center}
footer b{color:#e8eaf0;font-weight:600}
</style>
</head>
<body>
<header>
  <h1>📸 XIAO ESP32S3 Sense · OV3660</h1>
  <div id="connbox"><span id="dot"></span><span id="conn">connecting…</span></div>
</header>
<main>
  <div id="frame" class="empty"><img id="photo" alt="photo"></div>
  <div id="controls">
    <button id="btnSnap">⚡ Capture now</button>
    <button id="btnSave">💾 Save photo</button>
    <button id="btnLive">▶ Live view</button>
  </div>
  <p class="hint">The ESP32 takes a photo every 10 s — this page updates automatically.</p>
</main>
<footer>
  <span>photo <b id="fFrame">–</b></span>
  <span>size <b id="fSize">–</b></span>
  <span>next in <b id="fNext">–</b></span>
  <span>uptime <b id="fUp">–</b></span>
  <span>free heap <b id="fHeap">–</b></span>
</footer>
<script>
"use strict";
const $=i=>document.getElementById(i);
const photo=$("photo"),frame=$("frame"),dot=$("dot"),conn=$("conn");
let lastFrame=-1,lastFrameAt=0,live=false,busy=false,interval=10000;

const fmtB=b=>b>=1048576?(b/1048576).toFixed(1)+" MB":(b/1024).toFixed(0)+" KB";
const fmtUp=ms=>{const s=Math.floor(ms/1000),
  h=String(Math.floor(s/3600)).padStart(2,"0"),
  m=String(Math.floor(s%3600/60)).padStart(2,"0");
  return h+":"+m+":"+String(s%60).padStart(2,"0")};

async function poll(){
  if(live||busy)return;
  busy=true;
  try{
    const r=await fetch("/status",{cache:"no-store"});
    const s=await r.json();
    interval=s.interval||10000;
    dot.className="ok";conn.textContent="connected";
    if(s.frames!==lastFrame&&s.size>0){
      lastFrame=s.frames;lastFrameAt=Date.now();
      photo.src="/capture?f="+s.frames;          // unique URL = no caching
      frame.classList.remove("empty");
      $("fFrame").textContent="#"+s.frames+" · "+s.width+"×"+s.height;
      $("fSize").textContent=fmtB(s.size);
    }
    $("fUp").textContent=fmtUp(s.uptime);
    $("fHeap").textContent=fmtB(s.heap);
  }catch(e){
    dot.className="bad";conn.textContent="reconnecting…";
  }finally{busy=false}
}
setInterval(poll,1000);poll();

setInterval(()=>{
  $("fNext").textContent=live?"live":
    lastFrameAt?Math.max(0,Math.ceil((lastFrameAt+interval-Date.now())/1000))+" s":"–";
},250);

photo.onerror=()=>{
  if(live){live=false;$("btnLive").textContent="▶ Live view";conn.textContent="reconnecting…"}
  frame.classList.add("empty");
};

 $("btnSnap").onclick=async()=>{try{await fetch("/snap",{cache:"no-store"})}catch(e){}};

 $("btnSave").onclick=async()=>{
  try{
    const r=await fetch("/capture?f="+(lastFrame<0?0:lastFrame)+"&t="+Date.now(),{cache:"no-store"});
    if(!r.ok)throw 0;
    const blob=await r.blob(),d=new Date(),p=n=>String(n).padStart(2,"0");
    const a=document.createElement("a");
    a.href=URL.createObjectURL(blob);
    a.download="xiao-cam_"+d.getFullYear()+"-"+p(d.getMonth()+1)+"-"+p(d.getDate())+
               "_"+p(d.getHours())+"-"+p(d.getMinutes())+"-"+p(d.getSeconds())+".jpg";
    document.body.appendChild(a);a.click();a.remove();
    setTimeout(()=>URL.revokeObjectURL(a.href),5000);
  }catch(e){alert("Download failed")}
};

 $("btnLive").onclick=()=>{
  live=!live;
  if(live){
    photo.src="/stream";frame.classList.remove("empty");
    $("btnLive").textContent="⏹ Stop live";dot.className="ok";conn.textContent="live";
  }else{
    $("btnLive").textContent="▶ Live view";lastFrame=-1;poll();
  }
};
</script>
</body>
</html>
)rawliteral";

/* ------------------- camera ------------------- */
static bool initCamera() {
  camera_config_t cfg = {};                 // zero-init (safe defaults for LEDC etc.)
  cfg.pin_pwdn     = PWDN_GPIO_NUM;
  cfg.pin_reset    = RESET_GPIO_NUM;
  cfg.pin_xclk     = XCLK_GPIO_NUM;
  cfg.pin_sccb_sda = SIOD_GPIO_NUM;
  cfg.pin_sccb_scl = SIOC_GPIO_NUM;
  cfg.pin_d0 = Y2_GPIO_NUM;  cfg.pin_d1 = Y3_GPIO_NUM;
  cfg.pin_d2 = Y4_GPIO_NUM;  cfg.pin_d3 = Y5_GPIO_NUM;
  cfg.pin_d4 = Y6_GPIO_NUM;  cfg.pin_d5 = Y7_GPIO_NUM;
  cfg.pin_d6 = Y8_GPIO_NUM;  cfg.pin_d7 = Y9_GPIO_NUM;
  cfg.pin_vsync = VSYNC_GPIO_NUM;
  cfg.pin_href  = HREF_GPIO_NUM;
  cfg.pin_pclk  = PCLK_GPIO_NUM;

  cfg.xclk_freq_hz = 20000000;
  cfg.pixel_format = PIXFORMAT_JPEG;        // JPEG straight from the sensor

  if (psramFound()) {
    cfg.frame_size   = FRAME_SIZE;
    cfg.jpeg_quality = JPEG_QUALITY;
    cfg.fb_count     = 2;
    cfg.fb_location  = CAMERA_FB_IN_PSRAM;
    cfg.grab_mode    = CAMERA_GRAB_LATEST;  // always hand out the freshest frame
  } else {                                  // graceful fallback without PSRAM
    cfg.frame_size   = FRAMESIZE_VGA;
    cfg.jpeg_quality = 15;
    cfg.fb_count     = 1;
    cfg.fb_location  = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&cfg);
  if (err != ESP_OK) {
    Serial.printf("[cam] esp_camera_init() failed: 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  Serial.printf("[cam] sensor PID 0x%04X (%s)\n", (unsigned)s->id.PID,
                s->id.PID == OV3660_PID ? "OV3660" :
                s->id.PID == OV5640_PID ? "OV5640" :
                s->id.PID == OV2640_PID ? "OV2640" : "unknown");

  // Many OV3660 modules are mounted upside-down / slightly saturated — uncomment if needed:
  // if (s->id.PID == OV3660_PID) {
  //   s->set_vflip(s, 1);
  //   s->set_hmirror(s, 1);
  //   s->set_brightness(s, 1);
  //   s->set_saturation(s, -2);
  // }

  // discard a few frames so auto-exposure / AWB can settle
  for (int i = 0; i < 4; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
    delay(250);
  }
  return true;
}

/* ------------------- WiFi ------------------- */
static void startNetwork() {
#if USE_SOFT_AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);            // nullptr = open network (not recommended)
  Serial.printf("[wifi] Soft AP \"%s\" up — join it, then open http://%s/\n",
                AP_SSID, WiFi.softAPIP().toString().c_str());
#else
  Serial.printf("[wifi] Connecting to \"%s\"", STA_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(STA_SSID, STA_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(400); Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[wifi] Connected — open http://%s/  (or http://xiao-cam.local)\n",
                  WiFi.localIP().toString().c_str());
    if (MDNS.begin("xiao-cam")) MDNS.addService("http", "tcp", 80);
  } else {
    Serial.println("\n[wifi] Could not connect — starting Soft AP instead");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("[wifi] Join \"%s\" and open http://%s/\n",
                  AP_SSID, WiFi.softAPIP().toString().c_str());
  }
#endif
}

/* ------------------- capture task (core 0) ------------------- */
static void capturePhoto() {
  uint32_t t0 = millis();
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) { Serial.println("[cam] esp_camera_fb_get() failed"); return; }
  uint32_t grabMs = millis() - t0;

  if (fb->len == 0 || fb->len > MAX_JPEG_BYTES) {
    Serial.printf("[cam] bad frame length (%u) — skipped\n", (unsigned)fb->len);
  } else if (xSemaphoreTake(frameMutex, portMAX_DELAY) == pdTRUE) {
    memcpy(jpegBuf, fb->buf, fb->len);
    jpegLen = fb->len;
    frameW  = fb->width;
    frameH  = fb->height;
    frameMs = millis();
    frameCount++;
    xSemaphoreGive(frameMutex);
    Serial.printf("[photo] #%lu  %ux%u  %u KB  (grab %lu ms)\n",
                  (unsigned long)frameCount, (unsigned)frameW, (unsigned)frameH,
                  (unsigned)(fb->len / 1024), (unsigned long)grabMs);
  }
  esp_camera_fb_return(fb);

  if (HEARTBEAT_LED_PIN >= 0) {             // short blink = "photo taken"
    digitalWrite(HEARTBEAT_LED_PIN, HIGH);
    delay(40);
    digitalWrite(HEARTBEAT_LED_PIN, LOW);
  }
}

static void captureTask(void *arg) {
  (void)arg;
  for (;;) {
    capturePhoto();                                          // one photo ...
    xEventGroupWaitBits(evtGroup, EVT_SNAP, pdTRUE, pdFALSE, // ... then wait 10 s,
                        pdMS_TO_TICKS(CAPTURE_INTERVAL_MS)); // or wake on /snap
  }
}

/* ------------------- HTTP handlers ------------------- */
static void handleCapture() {
  if (xSemaphoreTake(frameMutex, pdMS_TO_TICKS(2000)) == pdTRUE) {
    if (jpegLen > 0) {
      server.sendHeader("Cache-Control", "no-store");
      server.sendHeader("Content-Disposition", "inline; filename=photo.jpg");
      server.setContentLength(jpegLen);
      server.send(200, "image/jpeg", "");
      server.client().write(jpegBuf, jpegLen);               // the JPEG body
    } else {
      server.send(503, "text/plain", "No photo yet, try again in a second.");
    }
    xSemaphoreGive(frameMutex);
  } else {
    server.send(503, "text/plain", "Camera busy, try again.");
  }
}

static void handleStatus() {
  uint32_t frames = 0, len = 0, w = 0, h = 0;
  if (xSemaphoreTake(frameMutex, pdMS_TO_TICKS(250)) == pdTRUE) {
    frames = frameCount; len = jpegLen; w = frameW; h = frameH;
    xSemaphoreGive(frameMutex);
  }
  char json[176];
  snprintf(json, sizeof(json),
           "{\"frames\":%lu,\"size\":%lu,\"width\":%lu,\"height\":%lu,"
           "\"interval\":%lu,\"uptime\":%lu,\"heap\":%lu}",
           (unsigned long)frames, (unsigned long)len, (unsigned long)w,
           (unsigned long)h, (unsigned long)CAPTURE_INTERVAL_MS,
           (unsigned long)millis(), (unsigned long)ESP.getFreeHeap());
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

static void handleSnap() {
  xEventGroupSetBits(evtGroup, EVT_SNAP);   // wake the capture task immediately
  server.send(200, "text/plain", "ok");
}

// Bonus: MJPEG stream, one part per capture. NOTE: the web server is single
// threaded, so while this is open the dashboard pauses its polling.
static void handleStream() {
  WiFiClient client = server.client();
  client.print("HTTP/1.1 200 OK\r\n"
               "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
               "Cache-Control: no-store\r\n"
               "Connection: close\r\n\r\n");
  uint32_t sent = 0;
  while (client.connected()) {
    bool pushed = false;
    if (xSemaphoreTake(frameMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
      if (jpegLen > 0 && frameCount != sent) {
        sent = frameCount;
        client.print("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ");
        client.println((unsigned long)jpegLen);
        client.print("\r\n");
        size_t n = client.write(jpegBuf, jpegLen);
        client.print("\r\n");
        pushed = true;
        xSemaphoreGive(frameMutex);
        if (n != jpegLen) break;            // client went away
      } else {
        xSemaphoreGive(frameMutex);
      }
    }
    if (!pushed) delay(120);
  }
}

/* ------------------- setup / loop ------------------- */
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== XIAO ESP32S3 Sense + OV3660 — 10 s photo server ===");

  if (HEARTBEAT_LED_PIN >= 0) pinMode(HEARTBEAT_LED_PIN, OUTPUT);

  if (!psramFound()) {
    Serial.println("FATAL: PSRAM not found. Tools -> PSRAM: 'OPI PSRAM', then re-upload.");
    while (true) delay(1000);
  }
  jpegBuf = (uint8_t *)ps_malloc(MAX_JPEG_BYTES);
  if (!jpegBuf) { Serial.println("FATAL: PSRAM alloc failed."); while (true) delay(1000); }

  if (!initCamera()) {
    Serial.println("FATAL: camera init failed — check flex connector & PSRAM setting.");
    while (true) delay(1000);
  }

  startNetwork();

  frameMutex = xSemaphoreCreateMutex();
  evtGroup   = xEventGroupCreate();

  server.on("/",            HTTP_GET, []() { server.send_P(200, "text/html", INDEX_HTML); });
  server.on("/index.html",  HTTP_GET, []() { server.send_P(200, "text/html", INDEX_HTML); });
  server.on("/capture",     HTTP_GET, handleCapture);
  server.on("/status",      HTTP_GET, handleStatus);
  server.on("/snap",        HTTP_GET, handleSnap);
  server.on("/stream",      HTTP_GET, handleStream);
  server.on("/favicon.ico", HTTP_GET, []() { server.send(204, "text/plain", ""); });
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
  server.begin();

  // capture task on core 0, web server keeps running on core 1 (loop)
  xTaskCreatePinnedToCore(captureTask, "capture", 8192, nullptr, 2, nullptr, 0);

  Serial.println("---------------------------------------------------");
  Serial.println("Soft-AP mode: open  http://192.168.4.1/");
  Serial.printf("Photos: every %lu ms\n", (unsigned long)CAPTURE_INTERVAL_MS);
  Serial.println("---------------------------------------------------");
}

void loop() {
  server.handleClient();
  delay(2);
}
