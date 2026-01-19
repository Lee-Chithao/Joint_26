/**
 * Joint_CM_2026 v2.3 — ESP32-CAM with Web-Based Eye Tracking
 *
 * NEW IN v2.3:
 * - WiFi Auto-reconnect: Automatically reconnects if WiFi connection drops
 *   - Checks connection every 10 seconds
 *   - Attempts reconnection with 30-second intervals between retries
 *   - Supports both PSK and Enterprise WiFi with automatic fallback
 *   - Logs reconnection attempts and success in terminal
 *
 * FEATURES FROM v2.2:
 * - Web-based eye tracking using TensorFlow.js Face Landmarks Detection
 * - Laptop webcam detects when user is looking at the camera
 * - Configurable capture probability when gaze is detected
 * - Captures stored in /eyetrack/ folder on SD card
 * - Round eye-tracking visualization window in Web UI
 * - Real-time eye position and direction display
 *
 * ARCHITECTURE:
 * 1. Laptop webcam → TensorFlow.js face-landmarks-detection model
 * 2. Browser calculates gaze direction from iris landmarks
 * 3. When "looking at camera" detected → random check → trigger ESP32 capture
 * 4. ESP32 receives /eyetrack/capture request → saves to SD card
 *
 * ALL CORE FEATURES:
 * - Physical button: click=photo, hold=video
 * - SD card storage for button captures
 * - Web UI with download capability
 * - Dual WiFi (Enterprise + PSK) with auto-reconnect
 * - SSE terminal with logs
 * - Performance optimizations
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
static void set_stream_mode();
static void set_capture_mode();

// ============================ CONFIG ============================

// WiFi credentials
static const char* WIFI_PSK_SSID = "VM2049066";
static const char* WIFI_PSK_PASS = "mccxsaZddeda84ua";

static const char* WIFI_ENT_SSID  = "UAL-WiFi";
static const char* WIFI_ENT_USER  = "21005976";
static const char* WIFI_ENT_PASS  = "#35L79Z57vb";
static const char* WIFI_ENT_IDENT = "";

static const char* DEVICE_NAME = "Joint_CM_2026 v2.3 EyeTrack+AutoReconnect";

// Hardware pins
static const int FLASH_LED_PIN = 4;
static const int BUTTON_PIN = 12;

// Button timing
static const uint32_t DEBOUNCE_MS = 50;
static const uint32_t LONG_PRESS_MS = 800;

// ============================ PERFORMANCE CONFIG ============================

#define STREAM_FRAMESIZE  FRAMESIZE_QVGA
#define CAPTURE_FRAMESIZE FRAMESIZE_VGA
#define STREAM_QUALITY  12
#define CAPTURE_QUALITY 8
#define TARGET_STREAM_FPS 20
#define MIN_FRAME_TIME_MS (1000 / TARGET_STREAM_FPS)

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

// ============================ LOG BUFFER ============================
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

// ============================ WIFI RECONNECT STATE ============================
static uint32_t g_last_wifi_check_ms = 0;
static uint32_t g_wifi_reconnect_attempts = 0;
static const uint32_t WIFI_CHECK_INTERVAL_MS = 10000; // Check every 10 seconds
static const uint32_t WIFI_RECONNECT_DELAY_MS = 30000; // Wait 30s between reconnect attempts

// ============================ SD CARD STATE ============================
static bool g_sd_available = false;
static uint32_t g_photo_counter = 0;
static uint32_t g_video_counter = 0;
static uint32_t g_eyetrack_counter = 0;

// ============================ EYE TRACKING STATS ============================
static uint32_t g_eyetrack_captures = 0;
static uint32_t g_eyetrack_triggers = 0;

// ============================ HELPERS ============================
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
  if (!SD_MMC.begin("/sdcard", true)) {
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
  log_pushf("[sd] type=%s size=%lluMB", typeStr, cardSize);
  
  // Create directories including new eyetrack folder
  if (!SD_MMC.exists("/photos")) SD_MMC.mkdir("/photos");
  if (!SD_MMC.exists("/videos")) SD_MMC.mkdir("/videos");
  if (!SD_MMC.exists("/eyetrack")) SD_MMC.mkdir("/eyetrack");
  
  // Find highest existing file numbers
  File root = SD_MMC.open("/photos");
  if (root) {
    File file = root.openNextFile();
    while (file) {
      uint32_t num = 0;
      if (sscanf(file.name(), "IMG_%u.jpg", &num) == 1) {
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
      uint32_t num = 0;
      if (sscanf(file.name(), "VID_%u.mjpeg", &num) == 1) {
        if (num >= g_video_counter) g_video_counter = num + 1;
      }
      file = root.openNextFile();
    }
    root.close();
  }
  
  // Find highest eyetrack counter
  root = SD_MMC.open("/eyetrack");
  if (root) {
    File file = root.openNextFile();
    while (file) {
      uint32_t num = 0;
      if (sscanf(file.name(), "EYE_%u.jpg", &num) == 1) {
        if (num >= g_eyetrack_counter) g_eyetrack_counter = num + 1;
      }
      file = root.openNextFile();
    }
    root.close();
  }
  
  log_pushf("[sd] next: photo=%u video=%u eye=%u", g_photo_counter, g_video_counter, g_eyetrack_counter);
  return true;
}

static bool save_photo_to_sd(camera_fb_t* fb, char* out_filename, size_t out_len) {
  if (!g_sd_available || !fb) return false;
  
  snprintf(out_filename, out_len, "/photos/IMG_%04u.jpg", g_photo_counter++);
  
  File file = SD_MMC.open(out_filename, FILE_WRITE);
  if (!file) {
    log_pushf("[sd] create failed: %s", out_filename);
    return false;
  }
  
  size_t written = file.write(fb->buf, fb->len);
  file.close();
  
  if (written != fb->len) {
    log_pushf("[sd] write error: %u/%u", written, fb->len);
    return false;
  }
  
  return true;
}

// New function for eye-track captures
static bool save_eyetrack_photo(camera_fb_t* fb, char* out_filename, size_t out_len) {
  if (!g_sd_available || !fb) return false;
  
  snprintf(out_filename, out_len, "/eyetrack/EYE_%04u.jpg", g_eyetrack_counter++);
  
  File file = SD_MMC.open(out_filename, FILE_WRITE);
  if (!file) {
    log_pushf("[eye] create failed: %s", out_filename);
    return false;
  }
  
  size_t written = file.write(fb->buf, fb->len);
  file.close();
  
  if (written != fb->len) {
    log_pushf("[eye] write error: %u/%u", written, fb->len);
    return false;
  }
  
  return true;
}

static bool start_video_recording() {
  if (!g_sd_available || g_is_recording) return false;
  
  snprintf(g_current_video_path, sizeof(g_current_video_path), 
           "/videos/VID_%04u.mjpeg", g_video_counter++);
  
  g_video_file = SD_MMC.open(g_current_video_path, FILE_WRITE);
  if (!g_video_file) {
    log_pushf("[sd] video create failed");
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
  
  const char* boundary = "--frame\r\nContent-Type: image/jpeg\r\n\r\n";
  g_video_file.write((uint8_t*)boundary, strlen(boundary));
  g_video_file.write(fb->buf, fb->len);
  g_video_file.write((uint8_t*)"\r\n", 2);
  
  g_video_frame_count++;
  return true;
}

static void stop_video_recording() {
  if (!g_is_recording) return;
  
  g_video_file.close();
  g_is_recording = false;
  
  uint32_t duration_ms = millis() - g_recording_start_ms;
  float fps = (duration_ms > 0) ? (g_video_frame_count * 1000.0f / duration_ms) : 0;
  
  log_pushf("[rec] stopped: %s %u frames %.1ffps %ums",
            g_current_video_path, g_video_frame_count, fps, duration_ms);
}

// ============================ BUTTON ISR ============================

static void IRAM_ATTR button_isr() {
  uint32_t now = millis();
  bool pressed = (digitalRead(BUTTON_PIN) == LOW);
  
  if (pressed && !g_button_pressed) {
    if (now - g_button_release_time > DEBOUNCE_MS) {
      g_button_pressed = true;
      g_button_press_time = now;
    }
  } else if (!pressed && g_button_pressed) {
    if (now - g_button_press_time > DEBOUNCE_MS) {
      g_button_pressed = false;
      g_button_release_time = now;
      g_is_long_press = (now - g_button_press_time >= LONG_PRESS_MS);
      g_button_event_pending = true;
    }
  }
}

static void process_button_events() {
  if (g_is_recording) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
      write_video_frame(fb);
      esp_camera_fb_return(fb);
    }
  }
  
  if (!g_button_event_pending) return;
  g_button_event_pending = false;
  
  if (g_is_long_press) {
    if (g_is_recording) {
      stop_video_recording();
      set_flash(true); delay(100); set_flash(false);
      delay(100);
      set_flash(true); delay(100); set_flash(false);
    } else {
      if (start_video_recording()) {
        set_flash(true); delay(50); set_flash(false);
      }
    }
  } else {
    log_pushf("[btn] photo trigger");
    set_capture_mode();
    
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
    
    fb = esp_camera_fb_get();
    if (fb) {
      char filename[64];
      if (save_photo_to_sd(fb, filename, sizeof(filename))) {
        log_pushf("[btn] saved: %s (%u bytes)", filename, fb->len);
        set_flash(true); delay(100); set_flash(false);
      }
      esp_camera_fb_return(fb);
    }
    
    set_stream_mode();
  }
}

// ============================ CAMERA ============================

static void set_stream_mode() {
  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_framesize(s, STREAM_FRAMESIZE);
    s->set_quality(s, STREAM_QUALITY);
  }
}

static void set_capture_mode() {
  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_framesize(s, CAPTURE_FRAMESIZE);
    s->set_quality(s, CAPTURE_QUALITY);
    delay(10);
  }
}

static void setup_camera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  config.frame_size = STREAM_FRAMESIZE;
  config.jpeg_quality = STREAM_QUALITY;
  config.fb_count = 3;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    log_pushf("[cam] init failed: 0x%x", err);
    return;
  }

  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_brightness(s, 1);
    s->set_contrast(s, 1);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 1);
    s->set_gain_ctrl(s, 1);
    s->set_gainceiling(s, GAINCEILING_4X);
    s->set_lenc(s, 1);
    s->set_dcw(s, 1);
    s->set_bpc(s, 1);
    s->set_wpc(s, 1);
    s->set_raw_gma(s, 1);
  }

  log_pushf("[cam] init OK (PSRAM=%s)", psramFound() ? "YES" : "NO");
}

// ============================ WIFI ============================

static void onWiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      log_pushf("[wifi] IP: %s", WiFi.localIP().toString().c_str());
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      log_pushf("[wifi] disconnected");
      break;
  }
}

static bool connect_wifi_psk() {
  log_pushf("[wifi] trying PSK: %s", WIFI_PSK_SSID);
  WiFi.begin(WIFI_PSK_SSID, WIFI_PSK_PASS);
  
  for (int i = 0; i < 40; i++) {
    if (WiFi.status() == WL_CONNECTED) {
      log_pushf("[wifi] PSK connected");
      return true;
    }
    delay(250);
  }
  return false;
}

static bool connect_wifi_enterprise() {
  if (!has_text(WIFI_ENT_SSID)) return false;
  
  log_pushf("[wifi] trying WPA2-Enterprise: %s", WIFI_ENT_SSID);
  
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(100);
  
  esp_wifi_sta_wpa2_ent_set_identity((uint8_t*)WIFI_ENT_IDENT, strlen(WIFI_ENT_IDENT));
  esp_wifi_sta_wpa2_ent_set_username((uint8_t*)WIFI_ENT_USER, strlen(WIFI_ENT_USER));
  esp_wifi_sta_wpa2_ent_set_password((uint8_t*)WIFI_ENT_PASS, strlen(WIFI_ENT_PASS));
  esp_wifi_sta_wpa2_ent_enable();
  
  WiFi.begin(WIFI_ENT_SSID);
  
  for (int i = 0; i < 60; i++) {
    if (WiFi.status() == WL_CONNECTED) {
      log_pushf("[wifi] Enterprise connected");
      return true;
    }
    delay(250);
  }
  return false;
}

static void connect_wifi_dual() {
  WiFi.mode(WIFI_STA);

  if (connect_wifi_psk()) return;
  log_pushf("[wifi] PSK failed, trying Enterprise...");

  WiFi.disconnect(true);
  delay(100);

  if (connect_wifi_enterprise()) return;
  log_pushf("[wifi] Enterprise failed");
  log_pushf("[wifi] continuing offline");
}

// ============================ WIFI AUTO-RECONNECT ============================

static void check_and_reconnect_wifi() {
  uint32_t now = millis();

  // Only check at specified intervals
  if (now - g_last_wifi_check_ms < WIFI_CHECK_INTERVAL_MS) {
    return;
  }

  g_last_wifi_check_ms = now;

  // If WiFi is connected, reset reconnect attempts counter
  if (WiFi.status() == WL_CONNECTED) {
    if (g_wifi_reconnect_attempts > 0) {
      log_pushf("[wifi] reconnect successful after %u attempts", g_wifi_reconnect_attempts);
      g_wifi_reconnect_attempts = 0;
    }
    return;
  }

  // WiFi is disconnected - attempt reconnection
  // Add delay between attempts to avoid hammering the network
  static uint32_t last_reconnect_attempt = 0;
  if (now - last_reconnect_attempt < WIFI_RECONNECT_DELAY_MS) {
    return;
  }

  last_reconnect_attempt = now;
  g_wifi_reconnect_attempts++;

  log_pushf("[wifi] disconnected, attempting reconnect #%u", g_wifi_reconnect_attempts);

  // Try to reconnect using the dual WiFi function
  WiFi.disconnect(true);
  delay(100);
  connect_wifi_dual();

  if (WiFi.status() == WL_CONNECTED) {
    log_pushf("[wifi] reconnected successfully");
    g_wifi_reconnect_attempts = 0;
  } else {
    log_pushf("[wifi] reconnect attempt #%u failed, will retry in %us",
              g_wifi_reconnect_attempts, WIFI_RECONNECT_DELAY_MS / 1000);
  }
}

// ============================ HTTP HANDLERS ============================

static esp_err_t capture_handler(httpd_req_t *req) {
  log_pushf("[http] capture request");
  
  set_capture_mode();
  
  camera_fb_t* fb = esp_camera_fb_get();
  if (fb) esp_camera_fb_return(fb);
  
  fb = esp_camera_fb_get();
  if (!fb) {
    set_stream_mode();
    log_pushf("[cam] capture failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  esp_err_t res = httpd_resp_send(req, (const char*)fb->buf, fb->len);
  
  esp_camera_fb_return(fb);
  set_stream_mode();
  
  return res;
}

// ============================ EYE TRACK CAPTURE HANDLER ============================

static esp_err_t eyetrack_capture_handler(httpd_req_t *req) {
  g_eyetrack_triggers++;
  log_pushf("[eye] capture trigger #%u", g_eyetrack_triggers);
  
  if (!g_sd_available) {
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"success\":false,\"error\":\"SD card not available\"}", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
  }
  
  set_capture_mode();
  
  camera_fb_t* fb = esp_camera_fb_get();
  if (fb) esp_camera_fb_return(fb);
  
  fb = esp_camera_fb_get();
  if (!fb) {
    set_stream_mode();
    log_pushf("[eye] capture failed - no frame");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"success\":false,\"error\":\"Camera capture failed\"}", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
  }

  char filename[64];
  bool saved = save_eyetrack_photo(fb, filename, sizeof(filename));
  
  esp_camera_fb_return(fb);
  set_stream_mode();
  
  if (saved) {
    g_eyetrack_captures++;
    log_pushf("[eye] saved: %s (total=%u)", filename, g_eyetrack_captures);
    
    set_flash(true);
    delay(50);
    set_flash(false);
    
    char response[128];
    snprintf(response, sizeof(response), 
             "{\"success\":true,\"filename\":\"%s\",\"total\":%u}", 
             filename, g_eyetrack_captures);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
  } else {
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"success\":false,\"error\":\"Failed to save\"}", HTTPD_RESP_USE_STRLEN);
  }
  
  return ESP_OK;
}

// ============================ EYE TRACK STATS HANDLER ============================

static esp_err_t eyetrack_stats_handler(httpd_req_t *req) {
  uint32_t file_count = 0;
  uint64_t total_size = 0;
  
  if (g_sd_available) {
    File root = SD_MMC.open("/eyetrack");
    if (root) {
      File file = root.openNextFile();
      while (file) {
        if (!file.isDirectory()) {
          file_count++;
          total_size += file.size();
        }
        file = root.openNextFile();
      }
      root.close();
    }
  }
  
  char response[256];
  snprintf(response, sizeof(response), 
           "{\"triggers\":%u,\"captures\":%u,\"files\":%u,\"size_mb\":%.2f,\"sd_available\":%s}",
           g_eyetrack_triggers, g_eyetrack_captures, file_count,
           total_size / (1024.0 * 1024.0),
           g_sd_available ? "true" : "false");
  
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t stream_handler(httpd_req_t *req) {
  log_pushf("[stream] client connected");
  
  esp_err_t res = ESP_OK;
  
  static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=frame";
  static const char* STREAM_BOUNDARY = "\r\n--frame\r\n";
  static const char* STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
  
  httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "X-Framerate", "20");
  
  set_stream_mode();
  
  for (int i = 0; i < 2; i++) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
  }
  
  char part_buf[64];
  uint32_t frame_count = 0;
  uint32_t start_time = millis();
  uint32_t last_fps_time = start_time;
  uint32_t fps_frame_count = 0;
  
  while (true) {
    uint32_t frame_start = millis();
    
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      log_pushf("[stream] frame failed");
      res = ESP_FAIL;
      break;
    }
    
    size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, fb->len);
    
    if (httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY)) != ESP_OK ||
        httpd_resp_send_chunk(req, part_buf, hlen) != ESP_OK ||
        httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len) != ESP_OK) {
      esp_camera_fb_return(fb);
      break;
    }
    
    esp_camera_fb_return(fb);
    
    frame_count++;
    fps_frame_count++;
    
    uint32_t now = millis();
    if (now - last_fps_time >= 5000) {
      float fps = fps_frame_count * 1000.0f / (now - last_fps_time);
      log_pushf("[stream] %u frames, %.1f fps", frame_count, fps);
      fps_frame_count = 0;
      last_fps_time = now;
    }
    
    uint32_t frame_time = millis() - frame_start;
    if (frame_time < MIN_FRAME_TIME_MS) {
      delay(MIN_FRAME_TIME_MS - frame_time);
    }
  }
  
  log_pushf("[stream] ended after %u frames", frame_count);
  return res;
}

static esp_err_t flash_handler(httpd_req_t *req) {
  char buf[8];
  if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
    bool on = (buf[3] == '1');
    set_flash(on);
    log_pushf("[flash] %s", on ? "ON" : "OFF");
  }
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, "OK", 2);
}

// ============================ SSE EVENTS ============================

static void sse_send_line(httpd_req_t *req, const char* s) {
  char buf[256];
  int len = snprintf(buf, sizeof(buf), "data: %s\n\n", s);
  httpd_resp_send_chunk(req, buf, len);
}

static void sse_send_recent(httpd_req_t *req, int max_lines) {
  portENTER_CRITICAL(&g_log_mux);
  int head = g_log_head;
  int count = (g_log_seq < LOG_CAP) ? g_log_seq : LOG_CAP;
  portEXIT_CRITICAL(&g_log_mux);
  
  int start = (count < max_lines) ? 0 : (count - max_lines);
  for (int i = start; i < count; i++) {
    int idx = (head - count + i + LOG_CAP) % LOG_CAP;
    if (g_log[idx][0]) {
      sse_send_line(req, g_log[idx]);
    }
  }
}

static esp_err_t events_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/event-stream");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  httpd_resp_set_hdr(req, "Connection", "keep-alive");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  
  sse_send_recent(req, 50);
  
  uint32_t last_seq = g_log_seq;
  while (true) {
    if (g_log_seq != last_seq) {
      portENTER_CRITICAL(&g_log_mux);
      int head = g_log_head;
      uint32_t seq = g_log_seq;
      portEXIT_CRITICAL(&g_log_mux);
      
      int idx = (head - 1 + LOG_CAP) % LOG_CAP;
      if (g_log[idx][0]) {
        char buf[256];
        int len = snprintf(buf, sizeof(buf), "data: %s\n\n", g_log[idx]);
        if (httpd_resp_send_chunk(req, buf, len) != ESP_OK) break;
      }
      last_seq = seq;
    }
    delay(100);
  }
  
  return ESP_OK;
}

static esp_err_t log_clear_handler(httpd_req_t *req) {
  log_clear();
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, "OK", 2);
}

// ============================ SD CARD HANDLERS ============================

static esp_err_t sd_status_handler(httpd_req_t *req) {
  char buf[128];
  
  if (g_sd_available) {
    uint64_t total = SD_MMC.totalBytes() / (1024 * 1024);
    uint64_t used = SD_MMC.usedBytes() / (1024 * 1024);
    snprintf(buf, sizeof(buf), 
             "{\"available\":true,\"total_mb\":%llu,\"used_mb\":%llu,\"recording\":%s}",
             total, used, g_is_recording ? "true" : "false");
  } else {
    snprintf(buf, sizeof(buf), "{\"available\":false}");
  }
  
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t sd_list_handler(httpd_req_t *req) {
  if (!g_sd_available) {
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"files\":[]}", HTTPD_RESP_USE_STRLEN);
  }
  
  String json = "{\"files\":[";
  bool first = true;
  
  const char* dirs[] = {"/photos", "/videos", "/eyetrack"};
  const char* types[] = {"photo", "video", "eyetrack"};
  
  for (int d = 0; d < 3; d++) {
    File root = SD_MMC.open(dirs[d]);
    if (!root) continue;
    
    File file = root.openNextFile();
    while (file) {
      if (!file.isDirectory()) {
        if (!first) json += ",";
        first = false;
        
        char entry[128];
        snprintf(entry, sizeof(entry), 
                 "{\"name\":\"%s\",\"path\":\"%s/%s\",\"size\":%u,\"type\":\"%s\"}",
                 file.name(), dirs[d], file.name(), file.size(), types[d]);
        json += entry;
      }
      file = root.openNextFile();
    }
    root.close();
  }
  
  json += "]}";
  
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json.c_str(), json.length());
}

static esp_err_t sd_download_handler(httpd_req_t *req) {
  char query[128] = {0};
  char filepath[96] = {0};
  
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
    return httpd_resp_send_404(req);
  }
  
  if (httpd_query_key_value(query, "file", filepath, sizeof(filepath)) != ESP_OK) {
    return httpd_resp_send_404(req);
  }
  
  char decoded[96];
  int di = 0;
  for (int i = 0; filepath[i] && di < 95; i++) {
    if (filepath[i] == '%' && filepath[i+1] && filepath[i+2]) {
      char hex[3] = {filepath[i+1], filepath[i+2], 0};
      decoded[di++] = (char)strtol(hex, NULL, 16);
      i += 2;
    } else {
      decoded[di++] = filepath[i];
    }
  }
  decoded[di] = 0;
  
  File file = SD_MMC.open(decoded);
  if (!file || file.isDirectory()) {
    return httpd_resp_send_404(req);
  }
  
  httpd_resp_set_type(req, "application/octet-stream");
  
  char header[128];
  snprintf(header, sizeof(header), "attachment; filename=\"%s\"", file.name());
  httpd_resp_set_hdr(req, "Content-Disposition", header);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  
  char buf[1024];
  size_t read;
  while ((read = file.read((uint8_t*)buf, sizeof(buf))) > 0) {
    if (httpd_resp_send_chunk(req, buf, read) != ESP_OK) {
      file.close();
      return ESP_FAIL;
    }
  }
  
  file.close();
  httpd_resp_send_chunk(req, NULL, 0);
  return ESP_OK;
}

static esp_err_t sd_delete_handler(httpd_req_t *req) {
  char query[128] = {0};
  char filepath[96] = {0};
  
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
    return httpd_resp_send_404(req);
  }
  
  if (httpd_query_key_value(query, "file", filepath, sizeof(filepath)) != ESP_OK) {
    return httpd_resp_send_404(req);
  }
  
  char decoded[96];
  int di = 0;
  for (int i = 0; filepath[i] && di < 95; i++) {
    if (filepath[i] == '%' && filepath[i+1] && filepath[i+2]) {
      char hex[3] = {filepath[i+1], filepath[i+2], 0};
      decoded[di++] = (char)strtol(hex, NULL, 16);
      i += 2;
    } else {
      decoded[di++] = filepath[i];
    }
  }
  decoded[di] = 0;
  
  bool success = SD_MMC.remove(decoded);
  log_pushf("[sd] delete %s: %s", decoded, success ? "OK" : "FAIL");
  
  char response[64];
  snprintf(response, sizeof(response), "{\"success\":%s}", success ? "true" : "false");
  
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
}

// ============================ INDEX HTML WITH EYE TRACKING ============================

static esp_err_t index_handler(httpd_req_t *req) {
  static const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Joint_CM_2026 v2.3 - Eye Tracking + Auto-Reconnect</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',system-ui,sans-serif;background:#0a0a0f;color:#e0e0e0;min-height:100vh;padding:15px}
.header{text-align:center;padding:15px 0;border-bottom:1px solid #333;margin-bottom:15px}
.header h1{font-size:1.4em;color:#00d4ff;margin-bottom:5px}
.header .ver{font-size:0.8em;color:#888}
.main-grid{display:grid;grid-template-columns:1fr 300px;gap:15px;max-width:1400px;margin:0 auto}
.left-col{display:flex;flex-direction:column;gap:15px}
.card{background:#151520;border-radius:12px;padding:15px;border:1px solid #252530}
.card-title{font-size:0.9em;color:#00d4ff;margin-bottom:10px;display:flex;align-items:center;gap:8px}
.preview-card{position:relative}
#previewWrap{position:relative;width:100%;aspect-ratio:4/3;background:#000;border-radius:8px;overflow:hidden}
#previewImg{width:100%;height:100%;object-fit:contain;display:none}
#placeholder{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;color:#555;font-size:0.9em}
#fpsDisplay{position:absolute;top:8px;right:8px;background:rgba(0,0,0,0.7);padding:4px 8px;border-radius:4px;font-size:0.8em;color:#0f0;display:none}
#dlBtn{position:absolute;bottom:8px;right:8px;background:#00d4ff;color:#000;border:none;padding:6px 12px;border-radius:4px;cursor:pointer;display:none}
.idle #placeholder{display:flex}
.controls{display:flex;gap:8px;flex-wrap:wrap;margin-top:10px}
.btn{padding:8px 16px;border:none;border-radius:6px;cursor:pointer;font-size:0.85em;transition:all 0.2s}
.btn-primary{background:#00d4ff;color:#000}
.btn-primary:hover{background:#00a8cc}
.btn-secondary{background:#333;color:#fff}
.btn-secondary:hover{background:#444}
.btn-danger{background:#ff4757;color:#fff}
.btn-danger:hover{background:#cc3a47}
.btn:disabled{opacity:0.5;cursor:not-allowed}
.mode-btns{display:flex;gap:5px;margin-bottom:10px}
.mode-btn{flex:1;padding:10px;background:#222;border:2px solid #333;border-radius:8px;color:#888;cursor:pointer;text-align:center}
.mode-btn.active{border-color:#00d4ff;color:#00d4ff;background:#1a2a3a}
.status-bar{display:flex;gap:10px;align-items:center;padding:10px;background:#1a1a25;border-radius:8px;margin-top:10px}
.status-pill{padding:4px 10px;border-radius:12px;font-size:0.75em;background:#333}
.status-pill.ok{background:#2d5a2d;color:#5f5}
.status-pill.rec{background:#5a2d2d;color:#f55;animation:pulse 1s infinite}
@keyframes pulse{50%{opacity:0.5}}
#statusText{flex:1;font-size:0.85em;color:#888}
.gallery{margin-top:10px}
.tabs{display:flex;gap:5px;margin-bottom:10px}
.tab{flex:1;padding:8px;background:#222;border:none;color:#888;cursor:pointer;border-radius:6px}
.tab.active{background:#00d4ff;color:#000}
#thumbs{display:grid;grid-template-columns:repeat(auto-fill,minmax(70px,1fr));gap:8px;max-height:200px;overflow-y:auto}
.thumb{position:relative;aspect-ratio:1;border-radius:6px;overflow:hidden;cursor:pointer}
.thumb img{width:100%;height:100%;object-fit:cover}
.thumb .badge{position:absolute;bottom:2px;right:2px;font-size:0.7em}
.eyetrack-card{border:2px solid #ff6b35}
.eyetrack-card .card-title{color:#ff6b35}
#eyetrackWrap{position:relative;width:250px;height:250px;margin:0 auto}
#eyetrackCircle{width:250px;height:250px;border-radius:50%;overflow:hidden;border:3px solid #ff6b35;background:#000;position:relative}
#webcamVideo{width:100%;height:100%;object-fit:cover;transform:scaleX(-1)}
#eyeCanvas{position:absolute;top:0;left:0;width:100%;height:100%;pointer-events:none}
.eye-status{text-align:center;margin-top:10px;font-size:0.85em}
.eye-status .looking{color:#0f0;font-weight:bold}
.eye-status .not-looking{color:#888}
#gazeIndicator{width:20px;height:20px;border-radius:50%;background:#ff6b35;position:absolute;transform:translate(-50%,-50%);transition:all 0.1s;opacity:0;box-shadow:0 0 10px #ff6b35}
#gazeIndicator.active{opacity:1}
.eye-data{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:10px;font-size:0.75em}
.eye-data-item{background:#1a1a25;padding:8px;border-radius:6px;text-align:center}
.eye-data-item .label{color:#888;display:block;margin-bottom:2px}
.eye-data-item .value{color:#00d4ff;font-family:monospace}
.eyetrack-stats{display:grid;grid-template-columns:repeat(2,1fr);gap:8px;margin-top:10px}
.stat-box{background:#1a1a25;padding:10px;border-radius:6px;text-align:center}
.stat-box .num{font-size:1.5em;color:#ff6b35;font-weight:bold}
.stat-box .lbl{font-size:0.7em;color:#888;margin-top:2px}
.prob-slider{margin-top:15px}
.prob-slider label{display:block;margin-bottom:5px;font-size:0.85em}
.prob-slider input[type="range"]{width:100%;accent-color:#ff6b35}
.prob-slider .prob-value{text-align:center;font-size:1.2em;color:#ff6b35;margin-top:5px}
.eyetrack-toggle{display:flex;gap:10px;margin-top:10px}
.eyetrack-toggle .btn{flex:1}
.sd-panel{margin-top:10px}
.sd-bar-wrap{height:8px;background:#222;border-radius:4px;overflow:hidden;margin:8px 0}
#sdBar{height:100%;background:linear-gradient(90deg,#00d4ff,#00ff88);width:0%;transition:width 0.3s}
#sdFiles{max-height:150px;overflow-y:auto}
.sdFile{display:flex;align-items:center;gap:8px;padding:6px;background:#1a1a25;border-radius:4px;margin-bottom:4px;font-size:0.8em}
.sdFile .name{flex:1;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.sdFile button{background:#333;border:none;padding:4px 8px;border-radius:4px;color:#fff;cursor:pointer}
.term-header{display:flex;justify-content:space-between;align-items:center}
#termBox{height:200px;overflow-y:auto;background:#0a0a0f;border-radius:6px;padding:8px;font-family:monospace;font-size:0.75em;line-height:1.4;margin-top:8px}
#termBox div{padding:1px 0}
@media(max-width:900px){.main-grid{grid-template-columns:1fr}.right-col{order:-1}}
</style>
</head>
<body>
<div class="header">
<h1>Joint_CM_2026</h1>
<div class="ver">v2.3 Eye Tracking + WiFi Auto-Reconnect</div>
</div>
<div class="main-grid">
<div class="left-col">
<div class="card eyetrack-card">
<div class="card-title">Eye Tracking (Laptop Webcam)</div>
<div id="eyetrackWrap">
<div id="eyetrackCircle">
<video id="webcamVideo" autoplay playsinline muted></video>
<canvas id="eyeCanvas"></canvas>
<div id="gazeIndicator"></div>
</div>
</div>
<div class="eye-status">
<span id="eyeStatusText" class="not-looking">Initializing...</span>
</div>
<div class="eye-data">
<div class="eye-data-item"><span class="label">Left Eye</span><span class="value" id="leftEyePos">--</span></div>
<div class="eye-data-item"><span class="label">Right Eye</span><span class="value" id="rightEyePos">--</span></div>
<div class="eye-data-item"><span class="label">Gaze Direction</span><span class="value" id="gazeDir">--</span></div>
<div class="eye-data-item"><span class="label">Confidence</span><span class="value" id="gazeConf">--</span></div>
</div>
<div class="prob-slider">
<label>Capture Probability when Looking: <span id="probVal">50%</span></label>
<input type="range" id="probSlider" min="0" max="100" value="50">
</div>
<div class="eyetrack-stats">
<div class="stat-box"><div class="num" id="triggerCount">0</div><div class="lbl">Triggers</div></div>
<div class="stat-box"><div class="num" id="captureCount">0</div><div class="lbl">Captures</div></div>
</div>
<div class="eyetrack-toggle">
<button class="btn btn-primary" id="startEyetrack">Start Tracking</button>
<button class="btn btn-danger" id="stopEyetrack" disabled>Stop</button>
</div>
</div>
<div class="card preview-card">
<div class="card-title">ESP32-CAM Preview</div>
<div class="mode-btns">
<div class="mode-btn active" id="photoBtn">Photo</div>
<div class="mode-btn" id="videoBtn">Video</div>
</div>
<div id="previewWrap" class="idle">
<img id="previewImg">
<div id="placeholder">Click Stream or Capture</div>
<div id="fpsDisplay">-- fps</div>
<button id="dlBtn">Download</button>
</div>
<div class="controls">
<button class="btn btn-primary" id="captureBtn">Capture</button>
<button class="btn btn-secondary" id="startBtn">Stream</button>
<button class="btn btn-danger" id="stopBtn" disabled>Stop</button>
<button class="btn btn-secondary" id="flashOn">Flash On</button>
<button class="btn btn-secondary" id="flashOff">Flash Off</button>
</div>
<div class="status-bar">
<span class="status-pill ok" id="sdPill">SD</span>
<span class="status-pill rec" id="recPill" style="display:none">REC</span>
<span id="statusText">Ready</span>
</div>
</div>
</div>
<div class="right-col">
<div class="card">
<div class="card-title">Gallery <span id="galCount">0</span></div>
<div class="tabs">
<button class="tab active" data-tab="mem">Memory</button>
<button class="tab" data-tab="sd">SD Card</button>
<button class="tab" data-tab="eye">EyeTrack</button>
</div>
<div id="thumbs"></div>
<div class="controls" style="margin-top:10px">
<button class="btn btn-secondary" id="clearBtn">Clear</button>
<button class="btn btn-secondary" id="refreshBtn">Refresh</button>
</div>
</div>
<div class="card">
<div class="card-title">SD Card</div>
<div class="sd-panel">
<div style="display:flex;justify-content:space-between;font-size:0.85em">
<span id="sdStatus">Loading...</span>
<button class="btn btn-secondary" id="sdRefresh" style="padding:4px 8px;font-size:0.75em">Refresh</button>
</div>
<div class="sd-bar-wrap"><div id="sdBar"></div></div>
<div id="sdFiles"></div>
</div>
</div>
<div class="card">
<div class="term-header">
<div class="card-title" style="margin:0">Terminal <span class="status-pill" id="termPill">Offline</span></div>
<button class="btn btn-secondary" id="termClear" style="padding:4px 8px;font-size:0.75em">Clear</button>
</div>
<div id="termBox"></div>
</div>
</div>
</div>
<script src="https://cdn.jsdelivr.net/npm/@tensorflow/tfjs@4.10.0/dist/tf.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/@tensorflow-models/face-landmarks-detection@1.0.5/dist/face-landmarks-detection.min.js"></script>
<script>
const $=id=>document.getElementById(id);
let mode='photo',streaming=false,tab='mem',memGal=[],sdGal=[],curBlob=null,frameCount=0,lastFpsTime=0;
let eyetrackActive=false,detector=null,webcamStream=null,captureProb=0.5,lastCaptureTime=0,captureCooldown=3000,triggerCount=0,captureCount=0;
const LEFT_IRIS=[468,469,470,471,472],RIGHT_IRIS=[473,474,475,476,477];
const LEFT_EYE=[33,7,163,144,145,153,154,155,133,173,157,158,159,160,161,246];
const RIGHT_EYE=[362,382,381,380,374,373,390,249,263,466,388,387,386,385,384,398];

function setStatus(t){$('statusText').textContent=t;}
function showImg(src,label,blob){const img=$('previewImg');img.src=src;img.style.display='block';$('placeholder').style.display='none';$('previewWrap').classList.remove('idle');if(blob){curBlob=blob;$('dlBtn').style.display='block';}else{curBlob=null;$('dlBtn').style.display='none';}}
function showIdle(msg){$('previewImg').style.display='none';$('placeholder').style.display='flex';$('placeholder').textContent=msg;$('previewWrap').classList.add('idle');$('fpsDisplay').style.display='none';}
function setMode(m){mode=m;document.querySelectorAll('.mode-btn').forEach(b=>b.classList.remove('active'));$(m+'Btn').classList.add('active');showIdle(m==='photo'?'Photo mode ready':'Video mode ready');setStatus(m+' ready');}

function updateGal(){
  let items;
  if(tab==='mem')items=memGal;
  else if(tab==='sd')items=sdGal.filter(x=>x.type!=='eyetrack');
  else items=sdGal.filter(x=>x.type==='eyetrack');
  $('galCount').textContent=items.length;
  $('thumbs').innerHTML='';
  items.forEach(it=>{
    const d=document.createElement('div');d.className='thumb';
    const img=document.createElement('img');
    img.src=it.type==='mem'?it.url:`/sd/download?file=${encodeURIComponent(it.path)}`;
    img.onclick=()=>{if(it.type==='mem')showImg(it.url,'Photo',it.blob);else window.open(`/sd/download?file=${encodeURIComponent(it.path)}`);};
    const badge=document.createElement('div');badge.className='badge';
    badge.textContent=it.isVideo?'Vid':(it.type==='eyetrack'?'Eye':'Pic');
    d.appendChild(img);d.appendChild(badge);$('thumbs').prepend(d);
  });
}

function addMem(blob){const url=URL.createObjectURL(blob);memGal.push({type:'mem',url,blob,size:blob.size});if(tab==='mem')updateGal();}

async function capture(){
  setStatus('Capturing...');$('placeholder').textContent='Capturing...';$('previewImg').style.display='none';$('placeholder').style.display='flex';
  try{const r=await fetch('/capture?t='+Date.now(),{cache:'no-store'});if(!r.ok)throw 0;const blob=await r.blob();const url=URL.createObjectURL(blob);showImg(url,'Photo',blob);setStatus('Captured');addMem(blob);}
  catch(e){showIdle('Capture failed');setStatus('Error');}
}

function startStream(){setStatus('Streaming...');streaming=true;$('startBtn').disabled=true;$('stopBtn').disabled=false;$('dlBtn').style.display='none';$('fpsDisplay').style.display='block';frameCount=0;lastFpsTime=Date.now();const img=$('previewImg');img.onload=()=>{frameCount++;const now=Date.now();if(now-lastFpsTime>=1000){const fps=frameCount*1000/(now-lastFpsTime);$('fpsDisplay').textContent=fps.toFixed(1)+' fps';frameCount=0;lastFpsTime=now;}};showImg('/stream?'+Date.now(),'Live');}
function stopStream(){streaming=false;$('startBtn').disabled=false;$('stopBtn').disabled=true;$('previewImg').onload=null;$('previewImg').src='';setTimeout(()=>showIdle('Stopped'),100);setStatus('Stopped');}
async function flash(on){setStatus(on?'Flash on':'Flash off');try{await fetch('/flash?on='+(on?'1':'0'))}catch{}}
function clearGal(){if(tab==='mem'){memGal.forEach(x=>URL.revokeObjectURL(x.url));memGal=[];}updateGal();setStatus('Cleared');}

async function loadSD(){
  try{const r=await fetch('/sd/status');const d=await r.json();if(d.available){$('sdPill').textContent='OK';$('sdStatus').textContent=`${d.used_mb}/${d.total_mb}MB`;$('sdBar').style.width=(d.used_mb/d.total_mb*100)+'%';$('recPill').style.display=d.recording?'inline-block':'none';}else{$('sdPill').textContent='No Card';$('sdStatus').textContent='Not available';}}catch{$('sdPill').textContent='Error';}
  try{const r=await fetch('/sd/list');const d=await r.json();sdGal=(d.files||[]).map(f=>({type:f.type,name:f.name,path:f.path,size:f.size,isVideo:f.type==='video'}));$('sdFiles').innerHTML='';sdGal.slice().reverse().forEach(f=>{const div=document.createElement('div');div.className='sdFile';const icon=f.isVideo?'Vid':(f.type==='eyetrack'?'Eye':'Pic');div.innerHTML=`<span>${icon}</span><span class="name">${f.name}</span><button onclick="window.open('/sd/download?file=${encodeURIComponent(f.path)}')">DL</button>`;$('sdFiles').appendChild(div);});if(tab!=='mem')updateGal();}catch{}
  try{const r=await fetch('/eyetrack/stats');const d=await r.json();$('triggerCount').textContent=d.triggers||0;$('captureCount').textContent=d.captures||0;triggerCount=d.triggers||0;captureCount=d.captures||0;}catch{}
}

async function initEyeTracking(){
  $('eyeStatusText').textContent='Loading TensorFlow.js model...';
  try{const model=faceLandmarksDetection.SupportedModels.MediaPipeFaceMesh;const detectorConfig={runtime:'tfjs',refineLandmarks:true,maxFaces:1};detector=await faceLandmarksDetection.createDetector(model,detectorConfig);console.log('Face mesh model loaded');$('eyeStatusText').textContent='Model loaded. Click Start.';}
  catch(e){console.error('Failed to load model:',e);$('eyeStatusText').textContent='Failed to load model: '+e.message;}
}

async function startWebcam(){
  try{webcamStream=await navigator.mediaDevices.getUserMedia({video:{width:640,height:480,facingMode:'user'},audio:false});$('webcamVideo').srcObject=webcamStream;return true;}
  catch(e){console.error('Webcam error:',e);$('eyeStatusText').textContent='Webcam access denied';return false;}
}

function stopWebcam(){if(webcamStream){webcamStream.getTracks().forEach(t=>t.stop());webcamStream=null;}$('webcamVideo').srcObject=null;}

function calculateGazeDirection(face){
  const kp=face.keypoints;
  const leftIris=kp.filter((_,i)=>LEFT_IRIS.includes(i));
  const rightIris=kp.filter((_,i)=>RIGHT_IRIS.includes(i));
  const leftEye=kp.filter((_,i)=>LEFT_EYE.includes(i));
  const rightEye=kp.filter((_,i)=>RIGHT_EYE.includes(i));
  if(leftIris.length===0||rightIris.length===0)return null;
  const leftEyeCenter={x:leftEye.reduce((s,p)=>s+p.x,0)/leftEye.length,y:leftEye.reduce((s,p)=>s+p.y,0)/leftEye.length};
  const rightEyeCenter={x:rightEye.reduce((s,p)=>s+p.x,0)/rightEye.length,y:rightEye.reduce((s,p)=>s+p.y,0)/rightEye.length};
  const leftIrisCenter={x:leftIris.reduce((s,p)=>s+p.x,0)/leftIris.length,y:leftIris.reduce((s,p)=>s+p.y,0)/leftIris.length};
  const rightIrisCenter={x:rightIris.reduce((s,p)=>s+p.x,0)/rightIris.length,y:rightIris.reduce((s,p)=>s+p.y,0)/rightIris.length};
  const leftEyeWidth=Math.max(...leftEye.map(p=>p.x))-Math.min(...leftEye.map(p=>p.x));
  const rightEyeWidth=Math.max(...rightEye.map(p=>p.x))-Math.min(...rightEye.map(p=>p.x));
  const leftGazeX=(leftIrisCenter.x-leftEyeCenter.x)/(leftEyeWidth/2);
  const rightGazeX=(rightIrisCenter.x-rightEyeCenter.x)/(rightEyeWidth/2);
  const avgGazeX=(leftGazeX+rightGazeX)/2;
  const avgGazeY=((leftIrisCenter.y-leftEyeCenter.y)+(rightIrisCenter.y-rightEyeCenter.y))/2;
  const gazeThreshold=0.3;
  const isLooking=Math.abs(avgGazeX)<gazeThreshold&&Math.abs(avgGazeY)<20;
  const confidence=face.box?Math.min(100,Math.round((1-Math.abs(avgGazeX))*100)):0;
  return{leftIris:leftIrisCenter,rightIris:rightIrisCenter,leftEye:leftEyeCenter,rightEye:rightEyeCenter,gazeX:avgGazeX,gazeY:avgGazeY,isLooking,confidence};
}

function drawEyeTracking(face,canvas,video){
  const ctx=canvas.getContext('2d');
  const scaleX=canvas.width/video.videoWidth;
  const scaleY=canvas.height/video.videoHeight;
  ctx.clearRect(0,0,canvas.width,canvas.height);
  if(!face||!face.keypoints)return;
  ctx.strokeStyle='#00d4ff';ctx.lineWidth=2;
  const leftEyePoints=face.keypoints.filter((_,i)=>LEFT_EYE.includes(i));
  const rightEyePoints=face.keypoints.filter((_,i)=>RIGHT_EYE.includes(i));
  ctx.beginPath();leftEyePoints.forEach((p,i)=>{const x=(video.videoWidth-p.x)*scaleX;const y=p.y*scaleY;if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);});ctx.closePath();ctx.stroke();
  ctx.beginPath();rightEyePoints.forEach((p,i)=>{const x=(video.videoWidth-p.x)*scaleX;const y=p.y*scaleY;if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);});ctx.closePath();ctx.stroke();
  ctx.fillStyle='#ff6b35';
  const leftIris=face.keypoints.filter((_,i)=>LEFT_IRIS.includes(i));
  const rightIris=face.keypoints.filter((_,i)=>RIGHT_IRIS.includes(i));
  leftIris.forEach(p=>{const x=(video.videoWidth-p.x)*scaleX;const y=p.y*scaleY;ctx.beginPath();ctx.arc(x,y,3,0,Math.PI*2);ctx.fill();});
  rightIris.forEach(p=>{const x=(video.videoWidth-p.x)*scaleX;const y=p.y*scaleY;ctx.beginPath();ctx.arc(x,y,3,0,Math.PI*2);ctx.fill();});
}

async function triggerEyetrackCapture(){
  const now=Date.now();
  if(now-lastCaptureTime<captureCooldown)return;
  if(Math.random()>captureProb){console.log('Gaze detected but random check failed');return;}
  lastCaptureTime=now;triggerCount++;$('triggerCount').textContent=triggerCount;
  console.log('Eye track capture triggered!');
  try{const r=await fetch('/eyetrack/capture?t='+now);const d=await r.json();if(d.success){captureCount++;$('captureCount').textContent=captureCount;console.log('Eye track photo saved:',d.filename);$('eyetrackCircle').style.borderColor='#0f0';setTimeout(()=>{$('eyetrackCircle').style.borderColor='#ff6b35';},200);setTimeout(loadSD,500);}}
  catch(e){console.error('Eye track capture failed:',e);}
}

let detectLoop=null;
async function runDetection(){
  if(!eyetrackActive||!detector)return;
  const video=$('webcamVideo');const canvas=$('eyeCanvas');
  if(video.readyState<2){detectLoop=requestAnimationFrame(runDetection);return;}
  canvas.width=video.videoWidth;canvas.height=video.videoHeight;
  try{
    const faces=await detector.estimateFaces(video);
    if(faces.length>0){
      const face=faces[0];const gaze=calculateGazeDirection(face);
      drawEyeTracking(face,canvas,video);
      if(gaze){
        $('leftEyePos').textContent=`(${Math.round(gaze.leftIris.x)}, ${Math.round(gaze.leftIris.y)})`;
        $('rightEyePos').textContent=`(${Math.round(gaze.rightIris.x)}, ${Math.round(gaze.rightIris.y)})`;
        $('gazeDir').textContent=`X:${gaze.gazeX.toFixed(2)} Y:${gaze.gazeY.toFixed(2)}`;
        $('gazeConf').textContent=`${gaze.confidence}%`;
        const indicator=$('gazeIndicator');const circleRect=$('eyetrackCircle').getBoundingClientRect();
        const centerX=circleRect.width/2;const centerY=circleRect.height/2;
        const indicatorX=centerX-gaze.gazeX*60;const indicatorY=centerY+gaze.gazeY*2;
        indicator.style.left=indicatorX+'px';indicator.style.top=indicatorY+'px';indicator.classList.add('active');
        if(gaze.isLooking){$('eyeStatusText').textContent='LOOKING AT CAMERA';$('eyeStatusText').className='looking';indicator.style.background='#0f0';indicator.style.boxShadow='0 0 15px #0f0';await triggerEyetrackCapture();}
        else{$('eyeStatusText').textContent='Looking away...';$('eyeStatusText').className='not-looking';indicator.style.background='#ff6b35';indicator.style.boxShadow='0 0 10px #ff6b35';}
      }
    }else{
      $('eyeStatusText').textContent='No face detected';$('eyeStatusText').className='not-looking';
      $('leftEyePos').textContent='--';$('rightEyePos').textContent='--';$('gazeDir').textContent='--';$('gazeConf').textContent='--';
      $('gazeIndicator').classList.remove('active');
      const ctx=canvas.getContext('2d');ctx.clearRect(0,0,canvas.width,canvas.height);
    }
  }catch(e){console.error('Detection error:',e);}
  detectLoop=requestAnimationFrame(runDetection);
}

async function startEyeTracking(){if(!detector){$('eyeStatusText').textContent='Model not loaded yet';return;}const started=await startWebcam();if(!started)return;eyetrackActive=true;$('startEyetrack').disabled=true;$('stopEyetrack').disabled=false;$('eyeStatusText').textContent='Tracking active...';runDetection();}
function stopEyeTracking(){eyetrackActive=false;if(detectLoop){cancelAnimationFrame(detectLoop);detectLoop=null;}stopWebcam();$('startEyetrack').disabled=false;$('stopEyetrack').disabled=true;$('eyeStatusText').textContent='Stopped';$('eyeStatusText').className='not-looking';$('gazeIndicator').classList.remove('active');const canvas=$('eyeCanvas');const ctx=canvas.getContext('2d');ctx.clearRect(0,0,canvas.width,canvas.height);}

$('dlBtn').onclick=()=>{if(curBlob){const a=document.createElement('a');a.href=URL.createObjectURL(curBlob);a.download='capture_'+Date.now()+'.jpg';a.click();}};
document.querySelectorAll('.tab').forEach(t=>{t.onclick=()=>{document.querySelectorAll('.tab').forEach(x=>x.classList.remove('active'));t.classList.add('active');tab=t.dataset.tab;updateGal();};});
$('probSlider').oninput=function(){captureProb=this.value/100;$('probVal').textContent=this.value+'%';};
$('startEyetrack').onclick=startEyeTracking;$('stopEyetrack').onclick=stopEyeTracking;

let es;
function connectTerm(){$('termPill').textContent='Connecting...';es=new EventSource('/events');es.onopen=()=>$('termPill').textContent='Live';es.onerror=()=>$('termPill').textContent='Offline';es.onmessage=e=>{if(e.data){const d=document.createElement('div');d.textContent=e.data;if(e.data.includes('[stream]'))d.style.color='#8f8';if(e.data.includes('[rec]')||e.data.includes('[btn]'))d.style.color='#ff8';if(e.data.includes('[eye]'))d.style.color='#ff6b35';$('termBox').appendChild(d);while($('termBox').childNodes.length>500)$('termBox').removeChild($('termBox').firstChild);$('termBox').scrollTop=$('termBox').scrollHeight;}};}
connectTerm();

$('termClear').onclick=async()=>{$('termBox').innerHTML='';try{await fetch('/log/clear')}catch{}};
$('photoBtn').onclick=()=>setMode('photo');$('videoBtn').onclick=()=>setMode('video');
$('captureBtn').onclick=capture;$('startBtn').onclick=startStream;$('stopBtn').onclick=stopStream;
$('flashOn').onclick=()=>flash(true);$('flashOff').onclick=()=>flash(false);
$('clearBtn').onclick=clearGal;$('refreshBtn').onclick=loadSD;$('sdRefresh').onclick=loadSD;

setMode('photo');updateGal();loadSD();setInterval(loadSD,5000);initEyeTracking();
</script>
</body>
</html>
)HTML";

  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

// ============================ WEBSERVER ============================
static void start_webserver() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.max_uri_handlers = 14;
  config.stack_size = 8192;

  if (httpd_start(&g_httpd, &config) != ESP_OK) {
    log_pushf("[http] start failed");
    return;
  }

  httpd_uri_t uris[] = {
    {"/",               HTTP_GET, index_handler,           NULL},
    {"/capture",        HTTP_GET, capture_handler,         NULL},
    {"/stream",         HTTP_GET, stream_handler,          NULL},
    {"/flash",          HTTP_GET, flash_handler,           NULL},
    {"/events",         HTTP_GET, events_handler,          NULL},
    {"/log/clear",      HTTP_GET, log_clear_handler,       NULL},
    {"/sd/status",      HTTP_GET, sd_status_handler,       NULL},
    {"/sd/list",        HTTP_GET, sd_list_handler,         NULL},
    {"/sd/download",    HTTP_GET, sd_download_handler,     NULL},
    {"/sd/delete",      HTTP_GET, sd_delete_handler,       NULL},
    {"/eyetrack/capture", HTTP_GET, eyetrack_capture_handler, NULL},
    {"/eyetrack/stats",   HTTP_GET, eyetrack_stats_handler,   NULL},
  };

  for (auto& u : uris) httpd_register_uri_handler(g_httpd, &u);
  
  log_pushf("[http] server ready (12 endpoints)");
}

// ============================ SETUP ============================
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(FLASH_LED_PIN, OUTPUT);
  set_flash(false);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  g_boot_ms = millis();
  log_clear();

  log_pushf("=== %s ===", DEVICE_NAME);
  log_pushf("[sys] reset=%s cpu=%uMHz", reset_reason_str(esp_reset_reason()), getCpuFrequencyMhz());
  log_pushf("[sys] heap=%u psram=%s", ESP.getFreeHeap(), psramFound() ? "YES" : "NO");

  log_pushf("[sd] init...");
  g_sd_available = init_sd_card();

  setup_camera();

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), button_isr, CHANGE);
  log_pushf("[btn] GPIO%d ready", BUTTON_PIN);

  WiFi.onEvent(onWiFiEvent);
  connect_wifi_dual();

  start_webserver();

  if (WiFi.status() == WL_CONNECTED) {
    log_pushf("[url] http://%s/", WiFi.localIP().toString().c_str());
  }

  log_pushf("[ready] Click=Photo, Hold=Video, EyeTrack=Web");
}

// ============================ LOOP ============================
void loop() {
  process_button_events();

  // Auto-reconnect WiFi if connection is lost
  check_and_reconnect_wifi();

  uint32_t now = millis();
  if (now - g_last_status_ms > 5000) {
    g_last_status_ms = now;
    log_pushf("[stat] up=%us wifi=%s rssi=%d heap=%u sd=%s eye=%u/%u%s",
              (now - g_boot_ms) / 1000,
              WiFi.status() == WL_CONNECTED ? "OK" : "DOWN",
              WiFi.RSSI(),
              ESP.getFreeHeap(),
              g_sd_available ? "OK" : "NO",
              g_eyetrack_captures, g_eyetrack_triggers,
              g_is_recording ? " REC" : "");
  }

  delay(g_is_recording ? 5 : 20);
}
