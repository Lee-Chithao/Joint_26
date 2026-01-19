/**
 * Joint_CM_2026 v2 — ESP32-CAM (AI Thinker / OV2640) Web UI
 * 
 * ITERATION 2 FEATURES:
 * - Physical button interaction:
 *   • Single click: Capture photo
 *   • Long press (hold): Start video recording, release to stop
 * - SD Card storage for button-triggered captures
 * - Web UI download capability for photos and videos
 * - BreadVolt powered (single layer ESP32-CAM)
 *
 * Original Features (preserved):
 * - Dual Wi-Fi (WPA2-Enterprise + PSK fallback)
 * - Web UI with Photo/Video modes, Flash control
 * - Circular preview window, rotated 180°
 * - Gallery stored in page memory
 * - Terminal panel (SSE) with rich logs
 *
 * HARDWARE CONNECTIONS:
 * - Button: GPIO13 to GND (internal pull-up used)
 *   Note: GPIO13 is safe to use on ESP32-CAM (directly accessible)
 * - SD Card: Built-in (uses GPIO12, 13, 14, 15, 2)
 *   IMPORTANT: Since we're using GPIO13 for button, we need GPIO12 or another free pin
 *   REVISED: Button on GPIO12 (safe when SD not actively writing)
 *   OR use GPIO16 if available on your board variant
 * 
 * BUTTON PIN OPTIONS (choose based on your wiring):
 * - GPIO12: Works but shared with SD card (avoid during SD writes)
 * - GPIO13: Works but shared with SD card  
 * - GPIO16: Best choice if your board exposes it (not all do)
 * - GPIO33: Built-in red LED pin (can be repurposed)
 *
 * Endpoints:
 *   GET  /              UI
 *   GET  /capture       JPEG (web capture)
 *   GET  /stream        MJPEG
 *   GET  /flash?on=1|0  flash control
 *   GET  /events        SSE terminal
 *   GET  /log/clear     clear log buffer
 *   GET  /sd/list       list SD card files (JSON)
 *   GET  /sd/download?file=<name>  download file from SD
 *   GET  /sd/delete?file=<name>    delete file from SD
 *   GET  /sd/status     SD card status
 */

#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "SD_MMC.h"
#include "FS.h"

#include <esp_wifi.h>
extern "C" {
  #include "esp_http_server.h"
  #include "esp_wpa2.h"
}

#include <cstdarg>

// Forward declarations
static void sse_send_line(httpd_req_t *req, const char* s);
static void sse_send_recent(httpd_req_t *req, int max_lines);

// ============================ CONFIG ============================

// -------- Home Wi-Fi (PSK) --------
static const char* WIFI_PSK_SSID = "VM2049066";
static const char* WIFI_PSK_PASS = "mccxsaZddeda84ua";

// -------- Uni Wi-Fi (WPA2-Enterprise) --------
static const char* WIFI_ENT_SSID  = "UAL-WiFi";
static const char* WIFI_ENT_USER  = "21005976";
static const char* WIFI_ENT_PASS  = "#35L79Z57vb";
static const char* WIFI_ENT_IDENT = "";

// UI name
static const char* DEVICE_NAME = "Joint_CM_2026 v2";

// Flash LED pin (AI Thinker)
static const int FLASH_LED_PIN = 4;

// -------- BUTTON CONFIG --------
// GPIO for physical button (choose based on your wiring)
// Option 1: GPIO12 - available but shared with SD (careful timing)
// Option 2: GPIO16 - if your board exposes it
// Option 3: GPIO33 - red LED pin, can repurpose
static const int BUTTON_PIN = 12;  // Change this based on your wiring

// Button timing thresholds (milliseconds)
static const uint32_t DEBOUNCE_MS = 50;
static const uint32_t LONG_PRESS_MS = 800;  // Hold > 800ms = video recording

// ============================ CAMERA PINS (AI Thinker) ============================
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ============================ LOG BUFFER (SSE terminal) ============================
static const int LOG_CAP = 320;
static const int LOG_LEN = 220;

static char g_log[LOG_CAP][LOG_LEN];
static volatile uint32_t g_log_seq = 0;
static volatile int g_log_head = 0;
static portMUX_TYPE g_log_mux = portMUX_INITIALIZER_UNLOCKED;

static uint32_t g_boot_ms = 0;
static uint32_t g_last_status_ms = 0;

static httpd_handle_t g_httpd = nullptr;

// ============================ BUTTON STATE ============================
static volatile bool g_button_pressed = false;
static volatile uint32_t g_button_press_time = 0;
static volatile uint32_t g_button_release_time = 0;
static volatile bool g_button_event_pending = false;
static volatile bool g_is_long_press = false;

// ============================ RECORDING STATE ============================
static volatile bool g_is_recording = false;
static volatile uint32_t g_recording_start_ms = 0;
static char g_current_video_path[64] = {0};
static File g_video_file;
static uint32_t g_video_frame_count = 0;

// ============================ SD CARD STATE ============================
static bool g_sd_available = false;
static uint32_t g_photo_counter = 0;
static uint32_t g_video_counter = 0;

// ------------------ helpers ------------------
static bool has_text(const char* s) { return s && s[0] != '\0'; }

static const char* reset_reason_str(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT_RESET";
    case ESP_RST_SW:        return "SW_RESET";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "UNKNOWN";
  }
}

static void log_clear() {
  portENTER_CRITICAL(&g_log_mux);
  g_log_seq = 0;
  g_log_head = 0;
  for (int i = 0; i < LOG_CAP; i++) g_log[i][0] = '\0';
  portEXIT_CRITICAL(&g_log_mux);
}

static void log_pushf(const char* fmt, ...) {
  char line[LOG_LEN];
  va_list args;
  va_start(args, fmt);
  vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);

  Serial.println(line);

  portENTER_CRITICAL(&g_log_mux);
  strncpy(g_log[g_log_head], line, LOG_LEN - 1);
  g_log[g_log_head][LOG_LEN - 1] = '\0';
  g_log_head = (g_log_head + 1) % LOG_CAP;
  g_log_seq++;
  portEXIT_CRITICAL(&g_log_mux);
}

static void set_flash(bool on) {
  digitalWrite(FLASH_LED_PIN, on ? HIGH : LOW);
}

// ============================ SD CARD FUNCTIONS ============================

static bool init_sd_card() {
  // SD_MMC uses 1-bit mode on ESP32-CAM
  // This frees up some pins but is slower
  if (!SD_MMC.begin("/sdcard", true)) {  // true = 1-bit mode
    log_pushf("[sd] mount failed");
    return false;
  }
  
  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    log_pushf("[sd] no card detected");
    return false;
  }
  
  const char* typeStr = "UNKNOWN";
  switch (cardType) {
    case CARD_MMC:  typeStr = "MMC"; break;
    case CARD_SD:   typeStr = "SD"; break;
    case CARD_SDHC: typeStr = "SDHC"; break;
  }
  
  uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
  uint64_t usedSpace = SD_MMC.usedBytes() / (1024 * 1024);
  
  log_pushf("[sd] card type=%s size=%lluMB used=%lluMB", typeStr, cardSize, usedSpace);
  
  // Create directories if they don't exist
  if (!SD_MMC.exists("/photos")) {
    SD_MMC.mkdir("/photos");
    log_pushf("[sd] created /photos directory");
  }
  if (!SD_MMC.exists("/videos")) {
    SD_MMC.mkdir("/videos");
    log_pushf("[sd] created /videos directory");
  }
  
  // Find highest existing file numbers to continue sequence
  File root = SD_MMC.open("/photos");
  if (root) {
    File file = root.openNextFile();
    while (file) {
      const char* name = file.name();
      uint32_t num = 0;
      if (sscanf(name, "IMG_%u.jpg", &num) == 1) {
        if (num >= g_photo_counter) g_photo_counter = num + 1;
      }
      file = root.openNextFile();
    }
    root.close();
  }
  
  root = SD_MMC.open("/videos");
  if (root) {
    File file = root.openNextFile();
    while (file) {
      const char* name = file.name();
      uint32_t num = 0;
      if (sscanf(name, "VID_%u.mjpeg", &num) == 1) {
        if (num >= g_video_counter) g_video_counter = num + 1;
      }
      file = root.openNextFile();
    }
    root.close();
  }
  
  log_pushf("[sd] next photo=%u next video=%u", g_photo_counter, g_video_counter);
  
  return true;
}

static bool save_photo_to_sd(camera_fb_t* fb, char* out_filename, size_t out_len) {
  if (!g_sd_available || !fb) return false;
  
  snprintf(out_filename, out_len, "/photos/IMG_%04u.jpg", g_photo_counter++);
  
  File file = SD_MMC.open(out_filename, FILE_WRITE);
  if (!file) {
    log_pushf("[sd] failed to create %s", out_filename);
    return false;
  }
  
  size_t written = file.write(fb->buf, fb->len);
  file.close();
  
  if (written != fb->len) {
    log_pushf("[sd] write error: %u/%u bytes", written, fb->len);
    return false;
  }
  
  log_pushf("[sd] saved %s (%u bytes)", out_filename, fb->len);
  return true;
}

static bool start_video_recording() {
  if (!g_sd_available || g_is_recording) return false;
  
  snprintf(g_current_video_path, sizeof(g_current_video_path), 
           "/videos/VID_%04u.mjpeg", g_video_counter++);
  
  g_video_file = SD_MMC.open(g_current_video_path, FILE_WRITE);
  if (!g_video_file) {
    log_pushf("[sd] failed to create video file");
    return false;
  }
  
  g_is_recording = true;
  g_recording_start_ms = millis();
  g_video_frame_count = 0;
  
  log_pushf("[rec] started: %s", g_current_video_path);
  return true;
}

static bool write_video_frame(camera_fb_t* fb) {
  if (!g_is_recording || !g_video_file) return false;
  
  // Write MJPEG frame with boundary marker
  // Format: --frame\r\nContent-Length: <len>\r\n\r\n<jpeg data>\r\n
  char header[64];
  int hlen = snprintf(header, sizeof(header), 
                      "--frame\r\nContent-Length: %u\r\n\r\n", fb->len);
  
  g_video_file.write((uint8_t*)header, hlen);
  g_video_file.write(fb->buf, fb->len);
  g_video_file.write((uint8_t*)"\r\n", 2);
  
  g_video_frame_count++;
  return true;
}

static void stop_video_recording() {
  if (!g_is_recording) return;
  
  if (g_video_file) {
    g_video_file.close();
  }
  
  uint32_t duration_ms = millis() - g_recording_start_ms;
  float fps = (duration_ms > 0) ? (g_video_frame_count * 1000.0f / duration_ms) : 0;
  
  log_pushf("[rec] stopped: %u frames, %.1fs, %.1f fps", 
            g_video_frame_count, duration_ms / 1000.0f, fps);
  
  g_is_recording = false;
  g_video_frame_count = 0;
  g_current_video_path[0] = '\0';
}

// ============================ BUTTON HANDLING ============================

// Button ISR - minimal work, just record timing
static void IRAM_ATTR button_isr() {
  uint32_t now = millis();
  bool pressed = (digitalRead(BUTTON_PIN) == LOW);  // Active low with pull-up
  
  if (pressed && !g_button_pressed) {
    // Button just pressed
    if (now - g_button_release_time > DEBOUNCE_MS) {
      g_button_pressed = true;
      g_button_press_time = now;
    }
  } else if (!pressed && g_button_pressed) {
    // Button just released
    g_button_pressed = false;
    g_button_release_time = now;
    g_button_event_pending = true;
    g_is_long_press = (now - g_button_press_time >= LONG_PRESS_MS);
  }
}

// Process button events (called from main loop, not ISR)
static void process_button_events() {
  // Check for long press start (while still holding)
  if (g_button_pressed && !g_is_recording) {
    uint32_t hold_time = millis() - g_button_press_time;
    if (hold_time >= LONG_PRESS_MS) {
      // Start recording
      log_pushf("[btn] long press detected - starting recording");
      if (start_video_recording()) {
        // Flash LED briefly to indicate recording started
        set_flash(true);
        delay(100);
        set_flash(false);
      }
    }
  }
  
  // If recording and button released, stop recording
  if (g_is_recording && !g_button_pressed) {
    stop_video_recording();
    // Flash LED twice to indicate recording stopped
    set_flash(true);
    delay(100);
    set_flash(false);
    delay(100);
    set_flash(true);
    delay(100);
    set_flash(false);
  }
  
  // Process pending button event (release)
  if (g_button_event_pending) {
    g_button_event_pending = false;
    
    uint32_t press_duration = g_button_release_time - g_button_press_time;
    log_pushf("[btn] released after %ums", press_duration);
    
    // Short press = capture photo (only if wasn't recording)
    if (press_duration < LONG_PRESS_MS && !g_is_recording) {
      log_pushf("[btn] short press - capturing photo");
      
      // Capture photo
      camera_fb_t* fb = esp_camera_fb_get();
      if (fb) {
        char filename[64];
        if (save_photo_to_sd(fb, filename, sizeof(filename))) {
          // Flash LED to confirm capture
          set_flash(true);
          delay(50);
          set_flash(false);
        }
        esp_camera_fb_return(fb);
      } else {
        log_pushf("[btn] capture failed - no frame buffer");
      }
    }
  }
  
  // If recording, capture frames
  if (g_is_recording) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
      write_video_frame(fb);
      esp_camera_fb_return(fb);
    }
    
    // Log progress periodically
    static uint32_t last_rec_log = 0;
    if (millis() - last_rec_log > 2000) {
      last_rec_log = millis();
      uint32_t duration = (millis() - g_recording_start_ms) / 1000;
      log_pushf("[rec] recording... %us, %u frames", duration, g_video_frame_count);
    }
  }
}

// ============================ WPA2-Enterprise enable wrapper ============================
static void wpa2_ent_enable_wrapper() {
#if defined(WPA2_CONFIG_INIT_DEFAULT)
  esp_wpa2_config_t cfg = WPA2_CONFIG_INIT_DEFAULT();
  esp_wifi_sta_wpa2_ent_enable(&cfg);
#else
  esp_wifi_sta_wpa2_ent_enable();
#endif
}

// ============================ WI-FI EVENTS ============================
static void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      log_pushf("[wifi] connected to AP");
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      log_pushf("[wifi] disconnected, reason=%d", (int)info.wifi_sta_disconnected.reason);
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      log_pushf("[wifi] got ip=%s", WiFi.localIP().toString().c_str());
      break;
    default:
      break;
  }
}

// ============================ WI-FI CONNECT (DUAL) ============================
static void wifi_common_setup() {
  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_NONE);
}

static bool connect_wifi_enterprise(uint32_t timeout_ms = 30000) {
  if (!has_text(WIFI_ENT_SSID) || !has_text(WIFI_ENT_USER) || !has_text(WIFI_ENT_PASS)) return false;

  log_pushf("[wifi] trying Enterprise SSID: %s", WIFI_ENT_SSID);

  WiFi.disconnect(true);
  delay(200);

  wifi_common_setup();

  const char* ident = has_text(WIFI_ENT_IDENT) ? WIFI_ENT_IDENT : WIFI_ENT_USER;

  esp_wifi_sta_wpa2_ent_set_identity((uint8_t*)ident, strlen(ident));
  esp_wifi_sta_wpa2_ent_set_username((uint8_t*)WIFI_ENT_USER, strlen(WIFI_ENT_USER));
  esp_wifi_sta_wpa2_ent_set_password((uint8_t*)WIFI_ENT_PASS, strlen(WIFI_ENT_PASS));

  wpa2_ent_enable_wrapper();

  WiFi.begin(WIFI_ENT_SSID);

  const uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < timeout_ms) {
    delay(300);
  }

  if (WiFi.status() == WL_CONNECTED) {
    log_pushf("[wifi] Enterprise connected");
    log_pushf("[wifi] ip=%s rssi=%d dBm", WiFi.localIP().toString().c_str(), WiFi.RSSI());
    return true;
  }

  log_pushf("[wifi] Enterprise failed / timeout");
  esp_wifi_sta_wpa2_ent_disable();
  return false;
}

static bool connect_wifi_psk(uint32_t timeout_ms = 25000) {
  if (!has_text(WIFI_PSK_SSID)) return false;

  log_pushf("[wifi] trying PSK SSID: %s", WIFI_PSK_SSID);

  esp_wifi_sta_wpa2_ent_disable();

  WiFi.disconnect(true);
  delay(200);

  wifi_common_setup();

  WiFi.begin(WIFI_PSK_SSID, WIFI_PSK_PASS);

  const uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < timeout_ms) {
    delay(250);
  }

  if (WiFi.status() == WL_CONNECTED) {
    log_pushf("[wifi] PSK connected");
    log_pushf("[wifi] ip=%s rssi=%d dBm", WiFi.localIP().toString().c_str(), WiFi.RSSI());
    return true;
  }

  log_pushf("[wifi] PSK failed / timeout");
  return false;
}

static bool connect_wifi_dual() {
  if (connect_wifi_enterprise()) return true;
  if (connect_wifi_psk()) return true;
  log_pushf("[wifi] no connection (check credentials / network type)");
  return false;
}

// ============================ CAMERA SETUP ============================
static void setup_camera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;

  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;

  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;

  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;

  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  config.frame_size   = FRAMESIZE_VGA;  // 640x480 for better quality
  config.jpeg_quality = 10;             // Better quality for SD storage
  config.fb_count     = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    log_pushf("[cam] init failed: 0x%x", (unsigned)err);
    while (true) delay(1000);
  }

  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    log_pushf("[cam] sensor PID=0x%04x VER=0x%04x", (unsigned)s->id.PID, (unsigned)s->id.VER);
  }
  log_pushf("[cam] config framesize=%d quality=%d fb_count=%d",
            (int)config.frame_size, (int)config.jpeg_quality, (int)config.fb_count);
}

// ============================ HTTP: SSE helpers ============================
static void sse_send_line(httpd_req_t *req, const char* s) {
  httpd_resp_send_chunk(req, "data: ", 6);
  httpd_resp_send_chunk(req, s, strlen(s));
  httpd_resp_send_chunk(req, "\n\n", 2);
}

static void sse_send_recent(httpd_req_t *req, int max_lines) {
  portENTER_CRITICAL(&g_log_mux);
  uint32_t seq = g_log_seq;
  int head = g_log_head;
  portEXIT_CRITICAL(&g_log_mux);

  uint32_t count = (seq < (uint32_t)max_lines) ? seq : (uint32_t)max_lines;
  if (count == 0) return;

  int idx = head - (int)count;
  while (idx < 0) idx += LOG_CAP;

  for (uint32_t i = 0; i < count; i++) {
    char line[LOG_LEN];
    portENTER_CRITICAL(&g_log_mux);
    strncpy(line, g_log[idx], LOG_LEN);
    portEXIT_CRITICAL(&g_log_mux);

    if (line[0] != '\0') sse_send_line(req, line);
    idx = (idx + 1) % LOG_CAP;
  }
}

// ============================ HTTP HANDLERS ============================

static esp_err_t log_clear_handler(httpd_req_t *req) {
  log_pushf("HTTP /log/clear");
  log_clear();
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_sendstr(req, "ok");
}

static esp_err_t events_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/event-stream");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_set_hdr(req, "Connection", "keep-alive");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  sse_send_line(req, "[SSE] connected");
  sse_send_recent(req, 120);

  uint32_t last_keepalive_ms = millis();
  uint32_t last_seq = 0;

  portENTER_CRITICAL(&g_log_mux);
  last_seq = g_log_seq;
  portEXIT_CRITICAL(&g_log_mux);

  while (true) {
    uint32_t seq_now;
    int head_now;

    portENTER_CRITICAL(&g_log_mux);
    seq_now = g_log_seq;
    head_now = g_log_head;
    portEXIT_CRITICAL(&g_log_mux);

    if (seq_now != last_seq) {
      uint32_t diff = seq_now - last_seq;
      if (diff > (uint32_t)LOG_CAP) diff = LOG_CAP;

      int idx = head_now - (int)diff;
      while (idx < 0) idx += LOG_CAP;

      for (uint32_t i = 0; i < diff; i++) {
        char line[LOG_LEN];
        portENTER_CRITICAL(&g_log_mux);
        strncpy(line, g_log[idx], LOG_LEN);
        portEXIT_CRITICAL(&g_log_mux);

        if (line[0] != '\0') {
          if (httpd_resp_send_chunk(req, "", 0) != ESP_OK) return ESP_FAIL;
          sse_send_line(req, line);
        }
        idx = (idx + 1) % LOG_CAP;
      }
      last_seq = seq_now;
    }

    if (millis() - last_keepalive_ms > 5000) {
      if (httpd_resp_send_chunk(req, ": keepalive\n\n", strlen(": keepalive\n\n")) != ESP_OK) return ESP_FAIL;
      last_keepalive_ms = millis();
    }

    vTaskDelay(pdMS_TO_TICKS(120));
  }
}

static esp_err_t flash_handler(httpd_req_t *req) {
  char query[64] = {0};
  bool on = false;

  if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
    char on_str[8] = {0};
    if (httpd_query_key_value(query, "on", on_str, sizeof(on_str)) == ESP_OK) {
      on = (strcmp(on_str, "1") == 0);
    }
  }

  set_flash(on);
  log_pushf("HTTP /flash %s", on ? "ON" : "OFF");

  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_sendstr(req, on ? "flash=on" : "flash=off");
}

static esp_err_t capture_handler(httpd_req_t *req) {
  const uint32_t t0 = millis();
  log_pushf("HTTP /capture start");

  // Flush stale frame
  camera_fb_t *fb = esp_camera_fb_get();
  if (fb) {
    esp_camera_fb_return(fb);
    fb = nullptr;
  }
  vTaskDelay(pdMS_TO_TICKS(15));

  fb = esp_camera_fb_get();
  if (!fb) {
    log_pushf("HTTP /capture failed (fb null)");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  log_pushf("HTTP /capture bytes=%u", (unsigned)fb->len);

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  httpd_resp_set_hdr(req, "Pragma", "no-cache");
  // Add download header for optional download
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=\"capture.jpg\"");

  esp_err_t res = httpd_resp_send(req, (const char*)fb->buf, fb->len);
  esp_camera_fb_return(fb);

  log_pushf("HTTP /capture done %ums", (unsigned)(millis() - t0));
  return res;
}

static esp_err_t stream_handler(httpd_req_t *req) {
  log_pushf("HTTP /stream start");

  httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_set_hdr(req, "Connection", "close");

  uint32_t frames = 0;
  uint32_t t_fps = millis();

  while (true) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      log_pushf("[cam] stream fb null");
      vTaskDelay(pdMS_TO_TICKS(30));
      continue;
    }

    esp_err_t res = httpd_resp_send_chunk(req, "--frame\r\n", strlen("--frame\r\n"));
    if (res != ESP_OK) { esp_camera_fb_return(fb); break; }

    char hdr[128];
    int hdr_len = snprintf(hdr, sizeof(hdr),
                           "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                           fb->len);

    res = httpd_resp_send_chunk(req, hdr, hdr_len);
    if (res != ESP_OK) { esp_camera_fb_return(fb); break; }

    res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;

    res = httpd_resp_send_chunk(req, "\r\n", 2);
    if (res != ESP_OK) break;

    frames++;
    const uint32_t now = millis();
    if (now - t_fps > 3000) {
      float fps = frames * 1000.0f / (float)(now - t_fps);
      log_pushf("HTTP /stream fps=%.1f", fps);
      frames = 0;
      t_fps = now;
    }

    vTaskDelay(pdMS_TO_TICKS(30));
  }

  httpd_resp_send_chunk(req, nullptr, 0);
  log_pushf("HTTP /stream end");
  return ESP_OK;
}

// ============================ SD CARD HTTP HANDLERS ============================

static esp_err_t sd_status_handler(httpd_req_t *req) {
  char json[256];
  
  if (!g_sd_available) {
    snprintf(json, sizeof(json), 
             "{\"available\":false,\"error\":\"SD card not mounted\"}");
  } else {
    uint64_t total = SD_MMC.totalBytes() / (1024 * 1024);
    uint64_t used = SD_MMC.usedBytes() / (1024 * 1024);
    uint64_t free = total - used;
    
    snprintf(json, sizeof(json),
             "{\"available\":true,\"total_mb\":%llu,\"used_mb\":%llu,\"free_mb\":%llu,"
             "\"photo_count\":%u,\"video_count\":%u,\"recording\":%s}",
             total, used, free, g_photo_counter, g_video_counter,
             g_is_recording ? "true" : "false");
  }
  
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_sendstr(req, json);
}

static esp_err_t sd_list_handler(httpd_req_t *req) {
  if (!g_sd_available) {
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_sendstr(req, "{\"error\":\"SD card not available\",\"files\":[]}");
  }
  
  // Get optional type filter from query
  char query[32] = {0};
  char type_filter[16] = {0};  // "photo", "video", or "" for all
  
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
    httpd_query_key_value(query, "type", type_filter, sizeof(type_filter));
  }
  
  // Build JSON response
  String json = "{\"files\":[";
  bool first = true;
  
  // List photos
  if (strlen(type_filter) == 0 || strcmp(type_filter, "photo") == 0) {
    File dir = SD_MMC.open("/photos");
    if (dir) {
      File file = dir.openNextFile();
      while (file) {
        if (!file.isDirectory()) {
          if (!first) json += ",";
          first = false;
          
          json += "{\"name\":\"";
          json += file.name();
          json += "\",\"path\":\"/photos/";
          json += file.name();
          json += "\",\"size\":";
          json += String(file.size());
          json += ",\"type\":\"photo\"}";
        }
        file = dir.openNextFile();
      }
      dir.close();
    }
  }
  
  // List videos
  if (strlen(type_filter) == 0 || strcmp(type_filter, "video") == 0) {
    File dir = SD_MMC.open("/videos");
    if (dir) {
      File file = dir.openNextFile();
      while (file) {
        if (!file.isDirectory()) {
          if (!first) json += ",";
          first = false;
          
          json += "{\"name\":\"";
          json += file.name();
          json += "\",\"path\":\"/videos/";
          json += file.name();
          json += "\",\"size\":";
          json += String(file.size());
          json += ",\"type\":\"video\"}";
        }
        file = dir.openNextFile();
      }
      dir.close();
    }
  }
  
  json += "]}";
  
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_sendstr(req, json.c_str());
}

static esp_err_t sd_download_handler(httpd_req_t *req) {
  if (!g_sd_available) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "SD card not available");
    return ESP_FAIL;
  }
  
  char query[128] = {0};
  char filepath[96] = {0};
  
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing file parameter");
    return ESP_FAIL;
  }
  
  if (httpd_query_key_value(query, "file", filepath, sizeof(filepath)) != ESP_OK) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing file parameter");
    return ESP_FAIL;
  }
  
  // URL decode the filepath (handle %2F for /)
  // Simple decode for common cases
  String path = filepath;
  path.replace("%2F", "/");
  path.replace("%20", " ");
  
  log_pushf("HTTP /sd/download file=%s", path.c_str());
  
  // Security check: prevent directory traversal
  if (path.indexOf("..") >= 0) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid path");
    return ESP_FAIL;
  }
  
  File file = SD_MMC.open(path.c_str(), FILE_READ);
  if (!file) {
    log_pushf("[sd] file not found: %s", path.c_str());
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
    return ESP_FAIL;
  }
  
  // Determine content type
  const char* content_type = "application/octet-stream";
  if (path.endsWith(".jpg") || path.endsWith(".jpeg")) {
    content_type = "image/jpeg";
  } else if (path.endsWith(".mjpeg")) {
    content_type = "video/x-motion-jpeg";
  }
  
  // Extract filename for Content-Disposition
  int lastSlash = path.lastIndexOf('/');
  String filename = (lastSlash >= 0) ? path.substring(lastSlash + 1) : path;
  
  char disposition[128];
  snprintf(disposition, sizeof(disposition), "attachment; filename=\"%s\"", filename.c_str());
  
  httpd_resp_set_type(req, content_type);
  httpd_resp_set_hdr(req, "Content-Disposition", disposition);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  
  // Stream file in chunks
  const size_t CHUNK_SIZE = 4096;
  uint8_t* buffer = (uint8_t*)malloc(CHUNK_SIZE);
  if (!buffer) {
    file.close();
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory error");
    return ESP_FAIL;
  }
  
  size_t total_sent = 0;
  size_t file_size = file.size();
  
  while (file.available()) {
    size_t bytes_read = file.read(buffer, CHUNK_SIZE);
    if (bytes_read > 0) {
      if (httpd_resp_send_chunk(req, (const char*)buffer, bytes_read) != ESP_OK) {
        free(buffer);
        file.close();
        return ESP_FAIL;
      }
      total_sent += bytes_read;
    }
  }
  
  free(buffer);
  file.close();
  
  // End chunked response
  httpd_resp_send_chunk(req, NULL, 0);
  
  log_pushf("[sd] download complete: %u bytes", total_sent);
  return ESP_OK;
}

static esp_err_t sd_delete_handler(httpd_req_t *req) {
  if (!g_sd_available) {
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"success\":false,\"error\":\"SD card not available\"}");
  }
  
  char query[128] = {0};
  char filepath[96] = {0};
  
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
      httpd_query_key_value(query, "file", filepath, sizeof(filepath)) != ESP_OK) {
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"success\":false,\"error\":\"Missing file parameter\"}");
  }
  
  String path = filepath;
  path.replace("%2F", "/");
  path.replace("%20", " ");
  
  log_pushf("HTTP /sd/delete file=%s", path.c_str());
  
  if (path.indexOf("..") >= 0) {
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"success\":false,\"error\":\"Invalid path\"}");
  }
  
  if (!SD_MMC.exists(path.c_str())) {
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"success\":false,\"error\":\"File not found\"}");
  }
  
  if (SD_MMC.remove(path.c_str())) {
    log_pushf("[sd] deleted: %s", path.c_str());
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_sendstr(req, "{\"success\":true}");
  } else {
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"success\":false,\"error\":\"Delete failed\"}");
  }
}

// ============================ HTML UI (Updated for v2) ============================

static esp_err_t index_handler(httpd_req_t *req) {
  log_pushf("HTTP / (UI loaded)");

  static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover" />
  <title>Joint_CM_2026 v2</title>
  <style>
    :root{ --r:16px; --b:1px solid #e8e8e8; --pad:14px; }
    body{
      font-family: system-ui,-apple-system,Segoe UI,Roboto,Helvetica,Arial,sans-serif;
      margin:12px; max-width:1400px;
      padding-bottom: env(safe-area-inset-bottom);
    }
    h2{ margin:0 0 10px 0; font-size:22px; }
    .row{ display:flex; gap:10px; flex-wrap:wrap; align-items:center; }
    .grow{ flex:1; }
    button{
      padding:10px 12px; border:1px solid #cfcfcf; border-radius:12px;
      background:#fff; cursor:pointer; font-size:14px;
    }
    button.primary{ border-color:#111; }
    button.danger{ border-color:#b00020; color:#b00020; }
    button.success{ border-color:#007700; color:#007700; }
    button:disabled{ opacity:.5; cursor:not-allowed; }
    .pill{
      display:inline-block; padding:6px 10px; border-radius:999px;
      border:var(--b); font-size:13px; background:#fff;
    }
    .pill.recording{ background:#ff4444; color:#fff; border-color:#ff4444; animation: pulse 1s infinite; }
    @keyframes pulse { 0%,100%{opacity:1} 50%{opacity:0.7} }
    .muted{ opacity:.75; }
    .panel{ border:var(--b); border-radius:var(--r); padding:var(--pad); margin-top:12px; background:#fff; }

    .layout{ display:grid; grid-template-columns: 1fr 380px; gap:12px; align-items:start; }
    @media (max-width: 900px){ .layout{ grid-template-columns: 1fr; } }

    .toolbar{
      display:flex; gap:10px; flex-wrap:wrap; align-items:center;
      justify-content: space-between; margin-top:8px;
    }
    .toolbarLeft,.toolbarRight{ display:flex; gap:10px; flex-wrap:wrap; align-items:center; }

    .previewWrap{
      border:var(--b); border-radius:50%; overflow:hidden; position:relative;
      width:100%; max-width:720px; aspect-ratio:1/1; margin-top:10px;
      background:#f4f4f4;
    }
    .previewWrap.idle{ background: linear-gradient(135deg,#f0f0f0,#dedede); }
    .previewWrap img{
      width:100%; height:100%; object-fit:cover; object-position:center; display:block;
      transform: rotate(180deg); transform-origin: center center;
    }
    .placeholder{
      width:100%; height:100%;
      display:flex; align-items:center; justify-content:center;
      color:#666; font-size:14px; letter-spacing:.2px; padding:12px; text-align:center;
    }
    .overlayTag{
      position:absolute; top:10px; left:10px;
      background: rgba(0,0,0,.55); color:#fff; padding:6px 10px;
      border-radius:999px; font-size:12px; backdrop-filter: blur(6px);
    }
    .downloadBtn{
      position:absolute; bottom:10px; right:10px;
      background: rgba(0,0,0,.55); color:#fff; padding:8px 12px;
      border-radius:12px; font-size:12px; backdrop-filter: blur(6px);
      border:none; cursor:pointer;
    }
    .downloadBtn:hover{ background: rgba(0,0,0,.75); }

    .galleryHead{ display:flex; align-items:center; gap:10px; margin-top:12px; }
    .tabs{ display:flex; gap:5px; }
    .tab{ padding:6px 12px; border:var(--b); border-radius:8px; cursor:pointer; font-size:13px; background:#fff; }
    .tab.active{ background:#111; color:#fff; border-color:#111; }
    
    .thumbs{ display:flex; gap:10px; flex-wrap:wrap; margin-top:10px; }
    .thumb{
      width:128px; aspect-ratio:1/1; border:var(--b); border-radius:14px; overflow:hidden; background:#fff;
      position:relative;
    }
    .thumb img{
      width:100%; height:100%; object-fit:cover; display:block; cursor:pointer;
      transform: rotate(180deg); transform-origin: center center;
    }
    .thumb .badge{
      position:absolute; bottom:4px; left:4px;
      background:rgba(0,0,0,.6); color:#fff; padding:2px 6px;
      border-radius:6px; font-size:10px;
    }
    .thumb .actions{
      position:absolute; top:4px; right:4px;
      display:flex; gap:4px; opacity:0; transition: opacity 0.2s;
    }
    .thumb:hover .actions{ opacity:1; }
    .thumb .actionBtn{
      width:24px; height:24px; border-radius:6px;
      background:rgba(0,0,0,.6); color:#fff; border:none;
      cursor:pointer; font-size:12px; display:flex; align-items:center; justify-content:center;
    }

    .sidebar{ display:flex; flex-direction:column; gap:12px; }
    
    .sdPanel{ border:var(--b); border-radius:var(--r); padding:var(--pad); background:#fff; }
    .sdHead{ display:flex; align-items:center; gap:10px; margin-bottom:10px; }
    .sdStatus{ font-size:13px; }
    .sdBar{ height:8px; background:#eee; border-radius:4px; overflow:hidden; margin:8px 0; }
    .sdBarFill{ height:100%; background: linear-gradient(90deg, #4CAF50, #8BC34A); border-radius:4px; }
    .sdFiles{ max-height:300px; overflow:auto; }
    .sdFile{
      display:flex; align-items:center; gap:8px; padding:8px;
      border-bottom:1px solid #eee; font-size:13px;
    }
    .sdFile:last-child{ border-bottom:none; }
    .sdFile .icon{ font-size:18px; }
    .sdFile .name{ flex:1; word-break:break-all; }
    .sdFile .size{ color:#888; font-size:12px; }
    .sdFile button{ padding:4px 8px; font-size:11px; }

    .term{ border:var(--b); border-radius:var(--r); padding:var(--pad); background:#fff; }
    .termHead{ display:flex; gap:8px; align-items:center; margin-bottom:10px; }
    .termBox{
      height:400px; overflow:auto; background:#3a3a3a; color:#f1f1f1;
      border-radius:14px; padding:10px;
      font-family: ui-monospace,SFMono-Regular,Menlo,Monaco,Consolas,monospace;
      font-size:12px; line-height:1.35; white-space: pre-wrap; word-break: break-word;
    }
    @media (max-width: 430px){ .termBox{ height:280px; } button{ font-size:13px; } }
  </style>
</head>
<body>
  <h2>Joint_CM_2026 v2</h2>

  <div class="layout">
    <div>
      <div class="row">
        <span class="pill" id="modePill">Mode: Photo</span>
        <span class="pill muted" id="statusPill">Idle</span>
        <span class="pill" id="recPill" style="display:none">● REC</span>
        <div class="grow"></div>
        <button id="photoModeBtn" class="primary">Photo mode</button>
        <button id="videoModeBtn">Video mode</button>
      </div>

      <div class="panel">
        <div class="toolbar">
          <div class="toolbarLeft">
            <button id="captureBtn" class="primary">📷 Capture</button>
            <button id="startVideoBtn">▶ Start video</button>
            <button id="stopVideoBtn" class="danger" disabled>■ Stop video</button>
          </div>
          <div class="toolbarRight">
            <button id="flashOnBtn">💡 Flash on</button>
            <button id="flashOffBtn">Flash off</button>
          </div>
        </div>

        <div class="previewWrap idle" id="previewWrap">
          <div class="overlayTag" id="overlayTag">Idle</div>
          <div class="placeholder" id="placeholder">Preview window ready<br><small>Use button: Click=Photo, Hold=Video</small></div>
          <img id="previewImg" alt="" style="display:none" />
          <button class="downloadBtn" id="previewDownload" style="display:none">⬇ Download</button>
        </div>

        <div class="galleryHead">
          <strong>Gallery</strong>
          <div class="tabs">
            <div class="tab active" data-tab="memory">Memory</div>
            <div class="tab" data-tab="sd">SD Card</div>
          </div>
          <span class="pill muted" id="galleryCount">0 items</span>
          <div class="grow"></div>
          <button id="refreshGalleryBtn">↻ Refresh</button>
          <button id="clearGalleryBtn">Clear</button>
        </div>
        <div class="thumbs" id="thumbs"></div>

        <p class="muted" style="margin:10px 0 0 0;font-size:13px">
          <b>Memory:</b> Web captures (clears on refresh) &nbsp;|&nbsp; <b>SD Card:</b> Button captures (persistent)
        </p>
      </div>
    </div>

    <aside class="sidebar">
      <div class="sdPanel">
        <div class="sdHead">
          <strong>💾 SD Card</strong>
          <span class="pill muted" id="sdStatusPill">Checking...</span>
          <div class="grow"></div>
          <button id="sdRefreshBtn">↻</button>
        </div>
        <div id="sdStatusArea">
          <div class="sdStatus" id="sdStatusText">Loading...</div>
          <div class="sdBar"><div class="sdBarFill" id="sdBarFill" style="width:0%"></div></div>
        </div>
        <div class="sdFiles" id="sdFiles"></div>
      </div>

      <div class="term">
        <div class="termHead">
          <strong>Terminal</strong>
          <span class="pill muted" id="termStatus">Connecting…</span>
          <div class="grow"></div>
          <button id="termClearBtn">Clear</button>
        </div>
        <div class="termBox" id="termBox"></div>
      </div>
    </aside>
  </div>

<script>
  // Elements
  const modePill = document.getElementById('modePill');
  const statusPill = document.getElementById('statusPill');
  const recPill = document.getElementById('recPill');
  const overlayTag = document.getElementById('overlayTag');

  const photoModeBtn = document.getElementById('photoModeBtn');
  const videoModeBtn = document.getElementById('videoModeBtn');

  const captureBtn = document.getElementById('captureBtn');
  const startVideoBtn = document.getElementById('startVideoBtn');
  const stopVideoBtn = document.getElementById('stopVideoBtn');

  const flashOnBtn = document.getElementById('flashOnBtn');
  const flashOffBtn = document.getElementById('flashOffBtn');

  const previewWrap = document.getElementById('previewWrap');
  const placeholder = document.getElementById('placeholder');
  const previewImg = document.getElementById('previewImg');
  const previewDownload = document.getElementById('previewDownload');

  const thumbs = document.getElementById('thumbs');
  const galleryCount = document.getElementById('galleryCount');
  const clearGalleryBtn = document.getElementById('clearGalleryBtn');
  const refreshGalleryBtn = document.getElementById('refreshGalleryBtn');

  const termBox = document.getElementById('termBox');
  const termStatus = document.getElementById('termStatus');
  const termClearBtn = document.getElementById('termClearBtn');

  const sdStatusPill = document.getElementById('sdStatusPill');
  const sdStatusText = document.getElementById('sdStatusText');
  const sdBarFill = document.getElementById('sdBarFill');
  const sdFiles = document.getElementById('sdFiles');
  const sdRefreshBtn = document.getElementById('sdRefreshBtn');

  let mode = "photo";
  let streaming = false;
  let memoryGallery = [];
  let sdGallery = [];
  let currentTab = "memory";
  let lastObjectUrl = null;
  let currentPreviewBlob = null;

  function setStatus(text){ statusPill.textContent = text; }
  function setOverlay(text){ overlayTag.textContent = text; }

  function showIdle(label){
    streaming = false;
    previewWrap.classList.add('idle');
    previewImg.style.display = 'none';
    placeholder.style.display = 'flex';
    placeholder.innerHTML = label;
    previewDownload.style.display = 'none';
    setOverlay("Idle");

    if (lastObjectUrl) {
      URL.revokeObjectURL(lastObjectUrl);
      lastObjectUrl = null;
    }
    currentPreviewBlob = null;
    previewImg.removeAttribute("src");
  }

  function showImage(src, overlayText, blob = null){
    previewWrap.classList.remove('idle');
    placeholder.style.display = 'none';
    previewImg.style.display = 'block';
    previewImg.src = src;
    setOverlay(overlayText);
    currentPreviewBlob = blob;
    previewDownload.style.display = blob ? 'block' : 'none';
  }

  function setMode(newMode){
    mode = newMode;
    if (streaming) stopStream();

    if (mode === "photo") {
      modePill.textContent = "Mode: Photo";
      captureBtn.disabled = false;
      startVideoBtn.disabled = true;
      stopVideoBtn.disabled = true;
      photoModeBtn.classList.add("primary");
      videoModeBtn.classList.remove("primary");
      showIdle("Photo mode: ready<br><small>Click button or use web capture</small>");
      setStatus("Photo ready");
    } else {
      modePill.textContent = "Mode: Video";
      captureBtn.disabled = true;
      startVideoBtn.disabled = false;
      stopVideoBtn.disabled = true;
      photoModeBtn.classList.remove("primary");
      videoModeBtn.classList.add("primary");
      showIdle("Video mode: ready<br><small>Hold button or use web controls</small>");
      setStatus("Video ready");
    }
  }

  function updateGalleryUI(){
    const items = currentTab === "memory" ? memoryGallery : sdGallery;
    galleryCount.textContent = `${items.length} item${items.length === 1 ? "" : "s"}`;
    
    thumbs.innerHTML = "";
    items.forEach((item, idx) => {
      const card = document.createElement('div');
      card.className = 'thumb';

      const img = document.createElement('img');
      if (item.type === 'memory') {
        img.src = item.url;
      } else {
        // For SD files, use a placeholder or first frame
        img.src = item.isVideo ? '/capture' : `/sd/download?file=${encodeURIComponent(item.path)}`;
      }
      img.alt = item.name || "capture";
      img.onclick = () => {
        if (item.type === 'memory') {
          showImage(item.url, "Photo", item.blob);
        } else {
          window.open(`/sd/download?file=${encodeURIComponent(item.path)}`, '_blank');
        }
        setStatus("Viewing: " + (item.name || "Photo"));
      };

      const badge = document.createElement('div');
      badge.className = 'badge';
      badge.textContent = item.isVideo ? '🎬' : '📷';
      if (item.size) badge.textContent += ` ${formatSize(item.size)}`;

      const actions = document.createElement('div');
      actions.className = 'actions';

      const dlBtn = document.createElement('button');
      dlBtn.className = 'actionBtn';
      dlBtn.innerHTML = '⬇';
      dlBtn.title = 'Download';
      dlBtn.onclick = (e) => {
        e.stopPropagation();
        downloadItem(item);
      };

      const delBtn = document.createElement('button');
      delBtn.className = 'actionBtn';
      delBtn.innerHTML = '✕';
      delBtn.title = 'Delete';
      delBtn.onclick = (e) => {
        e.stopPropagation();
        deleteItem(item, idx);
      };

      actions.appendChild(dlBtn);
      actions.appendChild(delBtn);

      card.appendChild(img);
      card.appendChild(badge);
      card.appendChild(actions);
      thumbs.prepend(card);
    });
  }

  function formatSize(bytes) {
    if (bytes < 1024) return bytes + 'B';
    if (bytes < 1024*1024) return (bytes/1024).toFixed(1) + 'KB';
    return (bytes/1024/1024).toFixed(1) + 'MB';
  }

  function downloadItem(item) {
    if (item.type === 'memory' && item.blob) {
      const a = document.createElement('a');
      a.href = item.url;
      a.download = item.name || `capture_${Date.now()}.jpg`;
      a.click();
    } else if (item.path) {
      const a = document.createElement('a');
      a.href = `/sd/download?file=${encodeURIComponent(item.path)}`;
      a.download = item.name;
      a.click();
    }
  }

  async function deleteItem(item, idx) {
    if (item.type === 'memory') {
      URL.revokeObjectURL(item.url);
      memoryGallery.splice(idx, 1);
      updateGalleryUI();
    } else if (item.path) {
      if (!confirm(`Delete ${item.name}?`)) return;
      try {
        const res = await fetch(`/sd/delete?file=${encodeURIComponent(item.path)}`);
        const data = await res.json();
        if (data.success) {
          await loadSDFiles();
          updateGalleryUI();
        } else {
          alert('Delete failed: ' + (data.error || 'Unknown error'));
        }
      } catch (e) {
        alert('Delete failed');
      }
    }
  }

  function addToMemoryGallery(blob, name){
    const url = URL.createObjectURL(blob);
    memoryGallery.push({ type: 'memory', url, blob, name: name || `IMG_${Date.now()}.jpg`, size: blob.size });
    if (currentTab === 'memory') updateGalleryUI();
  }

  async function capturePhoto(){
    setStatus("Capturing…");
    setOverlay("Capturing");
    previewWrap.classList.remove('idle');
    placeholder.style.display = 'flex';
    placeholder.textContent = "Capturing…";
    previewImg.style.display = 'none';
    previewDownload.style.display = 'none';

    try {
      const res = await fetch("/capture?t=" + Date.now(), { cache: "no-store" });
      if (!res.ok) throw new Error("capture failed");
      const blob = await res.blob();

      if (lastObjectUrl) URL.revokeObjectURL(lastObjectUrl);
      lastObjectUrl = URL.createObjectURL(blob);

      showImage(lastObjectUrl, "Photo", blob);
      setStatus("Photo captured");
      addToMemoryGallery(blob);
    } catch (e) {
      showIdle("Capture failed (check connection)");
      setStatus("Capture failed");
    }
  }

  function startStream(){
    setStatus("Streaming…");
    streaming = true;
    startVideoBtn.disabled = true;
    stopVideoBtn.disabled = false;
    previewDownload.style.display = 'none';
    showImage("/stream", "Live");
  }

  function stopStream(){
    streaming = false;
    startVideoBtn.disabled = false;
    stopVideoBtn.disabled = true;
    previewImg.removeAttribute("src");
    setTimeout(() => showIdle("Stream stopped"), 100);
    setStatus("Stream stopped");
  }

  async function flash(on){
    setStatus(on ? "Flash on" : "Flash off");
    try {
      await fetch("/flash?on=" + (on ? "1" : "0"), { cache: "no-store" });
    } catch (e) {
      setStatus("Flash request failed");
    }
  }

  function clearGallery(){
    if (currentTab === 'memory') {
      for (const item of memoryGallery) URL.revokeObjectURL(item.url);
      memoryGallery = [];
    }
    updateGalleryUI();
    setStatus("Gallery cleared");
  }

  // SD Card functions
  async function loadSDStatus() {
    try {
      const res = await fetch('/sd/status');
      const data = await res.json();
      
      if (data.available) {
        sdStatusPill.textContent = 'Ready';
        sdStatusPill.classList.remove('recording');
        
        const usedPct = (data.used_mb / data.total_mb * 100).toFixed(1);
        sdBarFill.style.width = usedPct + '%';
        sdStatusText.textContent = `${data.used_mb}MB / ${data.total_mb}MB (${data.free_mb}MB free)`;
        
        if (data.recording) {
          sdStatusPill.textContent = '● REC';
          sdStatusPill.classList.add('recording');
          recPill.style.display = 'inline-block';
          recPill.classList.add('recording');
        } else {
          recPill.style.display = 'none';
          recPill.classList.remove('recording');
        }
      } else {
        sdStatusPill.textContent = 'No Card';
        sdStatusText.textContent = data.error || 'SD card not available';
        sdBarFill.style.width = '0%';
      }
    } catch (e) {
      sdStatusPill.textContent = 'Error';
      sdStatusText.textContent = 'Failed to load SD status';
    }
  }

  async function loadSDFiles() {
    try {
      const res = await fetch('/sd/list');
      const data = await res.json();
      
      sdGallery = (data.files || []).map(f => ({
        type: 'sd',
        name: f.name,
        path: f.path,
        size: f.size,
        isVideo: f.type === 'video'
      }));
      
      // Update SD panel file list
      sdFiles.innerHTML = '';
      sdGallery.slice().reverse().forEach(f => {
        const div = document.createElement('div');
        div.className = 'sdFile';
        div.innerHTML = `
          <span class="icon">${f.isVideo ? '🎬' : '📷'}</span>
          <span class="name">${f.name}</span>
          <span class="size">${formatSize(f.size)}</span>
          <button onclick="window.open('/sd/download?file=${encodeURIComponent(f.path)}')">⬇</button>
        `;
        sdFiles.appendChild(div);
      });
      
      if (currentTab === 'sd') updateGalleryUI();
    } catch (e) {
      sdFiles.innerHTML = '<div class="sdFile">Failed to load files</div>';
    }
  }

  // Preview download
  previewDownload.onclick = () => {
    if (currentPreviewBlob) {
      const a = document.createElement('a');
      a.href = URL.createObjectURL(currentPreviewBlob);
      a.download = `capture_${Date.now()}.jpg`;
      a.click();
      URL.revokeObjectURL(a.href);
    }
  };

  // Tab switching
  document.querySelectorAll('.tab').forEach(tab => {
    tab.onclick = () => {
      document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
      tab.classList.add('active');
      currentTab = tab.dataset.tab;
      updateGalleryUI();
    };
  });

  // Terminal (SSE)
  function termAppend(line){
    const div = document.createElement('div');
    div.textContent = line;
    
    // Highlight recording events
    if (line.includes('[rec]') || line.includes('[btn]')) {
      div.style.color = '#4CAF50';
    }
    
    termBox.appendChild(div);
    while (termBox.childNodes.length > 1200) termBox.removeChild(termBox.firstChild);
    termBox.scrollTop = termBox.scrollHeight;
  }

  let es;
  function connectTerminal(){
    termStatus.textContent = "Connecting…";
    es = new EventSource('/events');
    es.onopen = () => termStatus.textContent = "Live";
    es.onerror = () => termStatus.textContent = "Disconnected";
    es.onmessage = (ev) => {
      if (ev.data && ev.data !== "keepalive") termAppend(ev.data);
    };
  }
  connectTerminal();

  termClearBtn.onclick = async () => {
    termBox.innerHTML = "";
    try { await fetch('/log/clear', { cache:'no-store' }); } catch {}
  };

  // Bind UI
  photoModeBtn.onclick = () => setMode("photo");
  videoModeBtn.onclick = () => setMode("video");

  captureBtn.onclick = capturePhoto;
  startVideoBtn.onclick = startStream;
  stopVideoBtn.onclick = stopStream;

  flashOnBtn.onclick = () => flash(true);
  flashOffBtn.onclick = () => flash(false);

  clearGalleryBtn.onclick = clearGallery;
  refreshGalleryBtn.onclick = () => {
    loadSDStatus();
    loadSDFiles();
    setStatus("Gallery refreshed");
  };
  sdRefreshBtn.onclick = () => {
    loadSDStatus();
    loadSDFiles();
  };

  window.addEventListener("beforeunload", () => { try { clearGallery(); } catch {} });

  // Initial load
  updateGalleryUI();
  setMode("photo");
  loadSDStatus();
  loadSDFiles();

  // Periodic SD status refresh (every 3s)
  setInterval(() => {
    loadSDStatus();
  }, 3000);
</script>
</body>
</html>
)HTML";

  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

// ============================ WEBSERVER START ============================
static void start_webserver() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.max_uri_handlers = 12;  // Increased for new endpoints

  if (httpd_start(&g_httpd, &config) != ESP_OK) {
    log_pushf("[http] start failed");
    return;
  }

  // Original endpoints
  httpd_uri_t uri_index     = { .uri="/",          .method=HTTP_GET, .handler=index_handler,     .user_ctx=nullptr };
  httpd_uri_t uri_capture   = { .uri="/capture",   .method=HTTP_GET, .handler=capture_handler,   .user_ctx=nullptr };
  httpd_uri_t uri_stream    = { .uri="/stream",    .method=HTTP_GET, .handler=stream_handler,    .user_ctx=nullptr };
  httpd_uri_t uri_flash     = { .uri="/flash",     .method=HTTP_GET, .handler=flash_handler,     .user_ctx=nullptr };
  httpd_uri_t uri_events    = { .uri="/events",    .method=HTTP_GET, .handler=events_handler,    .user_ctx=nullptr };
  httpd_uri_t uri_log_clear = { .uri="/log/clear", .method=HTTP_GET, .handler=log_clear_handler, .user_ctx=nullptr };
  
  // New SD card endpoints
  httpd_uri_t uri_sd_status   = { .uri="/sd/status",   .method=HTTP_GET, .handler=sd_status_handler,   .user_ctx=nullptr };
  httpd_uri_t uri_sd_list     = { .uri="/sd/list",     .method=HTTP_GET, .handler=sd_list_handler,     .user_ctx=nullptr };
  httpd_uri_t uri_sd_download = { .uri="/sd/download", .method=HTTP_GET, .handler=sd_download_handler, .user_ctx=nullptr };
  httpd_uri_t uri_sd_delete   = { .uri="/sd/delete",   .method=HTTP_GET, .handler=sd_delete_handler,   .user_ctx=nullptr };

  httpd_register_uri_handler(g_httpd, &uri_index);
  httpd_register_uri_handler(g_httpd, &uri_capture);
  httpd_register_uri_handler(g_httpd, &uri_stream);
  httpd_register_uri_handler(g_httpd, &uri_flash);
  httpd_register_uri_handler(g_httpd, &uri_events);
  httpd_register_uri_handler(g_httpd, &uri_log_clear);
  httpd_register_uri_handler(g_httpd, &uri_sd_status);
  httpd_register_uri_handler(g_httpd, &uri_sd_list);
  httpd_register_uri_handler(g_httpd, &uri_sd_download);
  httpd_register_uri_handler(g_httpd, &uri_sd_delete);

  log_pushf("[http] server started with SD endpoints");
}

// ============================ SETUP / LOOP ============================
void setup() {
  Serial.begin(115200);
  delay(250);

  pinMode(FLASH_LED_PIN, OUTPUT);
  set_flash(false);

  // Setup button with internal pull-up
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  g_boot_ms = millis();
  log_clear();

  log_pushf("%s boot…", DEVICE_NAME);
  log_pushf("[sys] reset=%s", reset_reason_str(esp_reset_reason()));
  log_pushf("[sys] cpu=%u MHz", (unsigned)getCpuFrequencyMhz());
  log_pushf("[sys] heap=%u min_heap=%u", (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap());
  log_pushf("[sys] psram=%s", psramFound() ? "FOUND" : "NOT FOUND");
  if (psramFound()) log_pushf("[sys] psram_free=%u", (unsigned)ESP.getFreePsram());

  // Initialize SD card before camera (they share some pins in 4-bit mode)
  log_pushf("[sd] initializing...");
  g_sd_available = init_sd_card();
  if (!g_sd_available) {
    log_pushf("[sd] WARNING: SD card not available - button captures will fail");
  }

  setup_camera();

  // Attach button interrupt
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), button_isr, CHANGE);
  log_pushf("[btn] button on GPIO%d ready", BUTTON_PIN);

  WiFi.onEvent(onWiFiEvent);
  bool ok = connect_wifi_dual();
  if (!ok) {
    log_pushf("[wifi] continuing without Wi-Fi (UI unreachable)");
  } else {
    log_pushf("[wifi] connected ok");
  }

  start_webserver();

  if (WiFi.status() == WL_CONNECTED) {
    log_pushf("[open] http://%s/", WiFi.localIP().toString().c_str());
  }

  log_pushf("[ready] Single click=Photo, Long press=Video");
}

void loop() {
  const uint32_t now = millis();

  // Process button events
  process_button_events();

  // Heartbeat
  if (now - g_last_status_ms > 5000) {
    g_last_status_ms = now;

    const bool wifi_ok = (WiFi.status() == WL_CONNECTED);
    const int rssi = wifi_ok ? WiFi.RSSI() : 0;
    const uint32_t up_s = (now - g_boot_ms) / 1000;

    log_pushf("[status] up=%us wifi=%s rssi=%d heap=%u psram=%u sd=%s%s",
              (unsigned)up_s,
              wifi_ok ? "OK" : "DOWN",
              rssi,
              (unsigned)ESP.getFreeHeap(),
              psramFound() ? (unsigned)ESP.getFreePsram() : 0u,
              g_sd_available ? "OK" : "NO",
              g_is_recording ? " REC" : "");
  }

  // Small delay, but keep responsive for recording
  delay(g_is_recording ? 10 : 50);
}
