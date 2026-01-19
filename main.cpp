/**
 * Joint_CM_2026 v2.1 — ESP32-CAM Optimized for Performance
 * 
 * OPTIMIZATIONS IN THIS VERSION:
 * - Dual resolution mode (QVGA stream, VGA capture)
 * - CAMERA_GRAB_LATEST for fresh frames
 * - Triple buffering with PSRAM
 * - Adaptive frame timing (targets 20fps)
 * - Reduced capture latency
 * - Sensor-level image optimizations
 * - Improved WiFi task priority
 * 
 * FEATURES:
 * - Physical button: click=photo, hold=video
 * - SD card storage for button captures
 * - Web UI with download capability
 * - Dual WiFi (Enterprise + PSK)
 * - SSE terminal with logs
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

static const char* DEVICE_NAME = "Joint_CM_2026 v2.1";

// Hardware pins
static const int FLASH_LED_PIN = 4;
static const int BUTTON_PIN = 12;

// Button timing
static const uint32_t DEBOUNCE_MS = 50;
static const uint32_t LONG_PRESS_MS = 800;

// ============================ PERFORMANCE CONFIG ============================

// Resolution presets (balance quality vs speed)
#define STREAM_FRAMESIZE  FRAMESIZE_QVGA   // 320x240 - fast streaming
#define CAPTURE_FRAMESIZE FRAMESIZE_VGA    // 640x480 - quality photos

// JPEG quality (4-63, lower = better quality but slower)
#define STREAM_QUALITY  12   // Fast streaming
#define CAPTURE_QUALITY 8    // High quality photos

// Target frame rate
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

// ============================ SD CARD STATE ============================
static bool g_sd_available = false;
static uint32_t g_photo_counter = 0;
static uint32_t g_video_counter = 0;

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
  
  if (!SD_MMC.exists("/photos")) SD_MMC.mkdir("/photos");
  if (!SD_MMC.exists("/videos")) SD_MMC.mkdir("/videos");
  
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
  
  log_pushf("[sd] next: photo=%u video=%u", g_photo_counter, g_video_counter);
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
  
  if (g_video_file) g_video_file.close();
  
  uint32_t duration_ms = millis() - g_recording_start_ms;
  float fps = (duration_ms > 0) ? (g_video_frame_count * 1000.0f / duration_ms) : 0;
  
  log_pushf("[rec] stopped: %u frames, %.1fs, %.1f fps", 
            g_video_frame_count, duration_ms / 1000.0f, fps);
  
  g_is_recording = false;
  g_video_frame_count = 0;
}

// ============================ OPTIMIZED CAMERA SETUP ============================

static void set_stream_mode() {
  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_framesize(s, STREAM_FRAMESIZE);
    s->set_quality(s, STREAM_QUALITY);
  }
}

static void set_capture_mode() {
  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_framesize(s, CAPTURE_FRAMESIZE);
    s->set_quality(s, CAPTURE_QUALITY);
  }
}

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

  // Optimized settings based on PSRAM
  if (psramFound()) {
    log_pushf("[cam] PSRAM found - using optimized config");
    config.frame_size   = STREAM_FRAMESIZE;
    config.jpeg_quality = STREAM_QUALITY;
    config.fb_count     = 3;  // Triple buffering
    config.fb_location  = CAMERA_FB_IN_PSRAM;
    config.grab_mode    = CAMERA_GRAB_LATEST;  // Always get fresh frame
  } else {
    log_pushf("[cam] No PSRAM - using conservative config");
    config.frame_size   = FRAMESIZE_QVGA;
    config.jpeg_quality = 15;
    config.fb_count     = 1;
    config.fb_location  = CAMERA_FB_IN_DRAM;
    config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    log_pushf("[cam] init failed: 0x%x", (unsigned)err);
    while (true) delay(1000);
  }

  // Sensor optimizations
  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    log_pushf("[cam] sensor PID=0x%04x", (unsigned)s->id.PID);
    
    // Image quality settings
    s->set_brightness(s, 1);
    s->set_contrast(s, 1);
    s->set_saturation(s, 0);
    
    // Auto-exposure/gain
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_wb_mode(s, 0);
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 1);
    s->set_gain_ctrl(s, 1);
    s->set_gainceiling(s, GAINCEILING_4X);
    
    // Corrections
    s->set_lenc(s, 1);
    s->set_dcw(s, 1);
    s->set_bpc(s, 1);
    s->set_wpc(s, 1);
    s->set_raw_gma(s, 1);
  }

  log_pushf("[cam] ready: fb_count=%d grab=LATEST", (int)config.fb_count);
}

// ============================ BUTTON HANDLING ============================

static void IRAM_ATTR button_isr() {
  uint32_t now = millis();
  bool pressed = (digitalRead(BUTTON_PIN) == LOW);
  
  if (pressed && !g_button_pressed) {
    if (now - g_button_release_time > DEBOUNCE_MS) {
      g_button_pressed = true;
      g_button_press_time = now;
    }
  } else if (!pressed && g_button_pressed) {
    g_button_pressed = false;
    g_button_release_time = now;
    g_button_event_pending = true;
    g_is_long_press = (now - g_button_press_time >= LONG_PRESS_MS);
  }
}

static void process_button_events() {
  // Long press detection (start recording)
  if (g_button_pressed && !g_is_recording) {
    uint32_t hold_time = millis() - g_button_press_time;
    if (hold_time >= LONG_PRESS_MS) {
      log_pushf("[btn] long press - recording");
      if (start_video_recording()) {
        set_flash(true);
        delay(100);
        set_flash(false);
      }
    }
  }
  
  // Stop recording on release
  if (g_is_recording && !g_button_pressed) {
    stop_video_recording();
    set_flash(true); delay(100); set_flash(false);
    delay(100);
    set_flash(true); delay(100); set_flash(false);
  }
  
  // Short press = capture photo
  if (g_button_event_pending) {
    g_button_event_pending = false;
    
    uint32_t press_duration = g_button_release_time - g_button_press_time;
    
    if (press_duration < LONG_PRESS_MS && !g_is_recording) {
      log_pushf("[btn] short press - capture");
      
      // Switch to capture mode for quality
      set_capture_mode();
      vTaskDelay(pdMS_TO_TICKS(20));
      
      // Flush stale frame
      camera_fb_t* fb = esp_camera_fb_get();
      if (fb) esp_camera_fb_return(fb);
      
      // Capture
      fb = esp_camera_fb_get();
      if (fb) {
        char filename[64];
        if (save_photo_to_sd(fb, filename, sizeof(filename))) {
          log_pushf("[btn] saved: %s (%uB)", filename, fb->len);
          set_flash(true); delay(50); set_flash(false);
        }
        esp_camera_fb_return(fb);
      }
      
      // Restore stream mode
      set_stream_mode();
    }
  }
  
  // Recording loop
  if (g_is_recording) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
      write_video_frame(fb);
      esp_camera_fb_return(fb);
    }
    
    static uint32_t last_rec_log = 0;
    if (millis() - last_rec_log > 2000) {
      last_rec_log = millis();
      log_pushf("[rec] %us, %u frames", 
                (millis() - g_recording_start_ms) / 1000, g_video_frame_count);
    }
  }
}

// ============================ WPA2 ENTERPRISE ============================
static void wpa2_ent_enable_wrapper() {
#if defined(WPA2_CONFIG_INIT_DEFAULT)
  esp_wpa2_config_t cfg = WPA2_CONFIG_INIT_DEFAULT();
  esp_wifi_sta_wpa2_ent_enable(&cfg);
#else
  esp_wifi_sta_wpa2_ent_enable();
#endif
}

// ============================ WIFI ============================
static void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      log_pushf("[wifi] connected");
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      log_pushf("[wifi] disconnected: %d", (int)info.wifi_sta_disconnected.reason);
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      log_pushf("[wifi] ip=%s", WiFi.localIP().toString().c_str());
      break;
    default: break;
  }
}

static void wifi_common_setup() {
  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_NONE);  // Disable power saving for streaming
}

static bool connect_wifi_enterprise(uint32_t timeout_ms = 30000) {
  if (!has_text(WIFI_ENT_SSID) || !has_text(WIFI_ENT_USER)) return false;

  log_pushf("[wifi] trying Enterprise: %s", WIFI_ENT_SSID);
  WiFi.disconnect(true);
  delay(200);
  wifi_common_setup();

  const char* ident = has_text(WIFI_ENT_IDENT) ? WIFI_ENT_IDENT : WIFI_ENT_USER;
  esp_wifi_sta_wpa2_ent_set_identity((uint8_t*)ident, strlen(ident));
  esp_wifi_sta_wpa2_ent_set_username((uint8_t*)WIFI_ENT_USER, strlen(WIFI_ENT_USER));
  esp_wifi_sta_wpa2_ent_set_password((uint8_t*)WIFI_ENT_PASS, strlen(WIFI_ENT_PASS));
  wpa2_ent_enable_wrapper();
  WiFi.begin(WIFI_ENT_SSID);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < timeout_ms) delay(300);

  if (WiFi.status() == WL_CONNECTED) {
    log_pushf("[wifi] Enterprise OK, rssi=%d", WiFi.RSSI());
    return true;
  }

  log_pushf("[wifi] Enterprise failed");
  esp_wifi_sta_wpa2_ent_disable();
  return false;
}

static bool connect_wifi_psk(uint32_t timeout_ms = 25000) {
  if (!has_text(WIFI_PSK_SSID)) return false;

  log_pushf("[wifi] trying PSK: %s", WIFI_PSK_SSID);
  esp_wifi_sta_wpa2_ent_disable();
  WiFi.disconnect(true);
  delay(200);
  wifi_common_setup();
  WiFi.begin(WIFI_PSK_SSID, WIFI_PSK_PASS);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < timeout_ms) delay(250);

  if (WiFi.status() == WL_CONNECTED) {
    log_pushf("[wifi] PSK OK, rssi=%d", WiFi.RSSI());
    return true;
  }

  log_pushf("[wifi] PSK failed");
  return false;
}

static bool connect_wifi_dual() {
  if (connect_wifi_enterprise()) return true;
  if (connect_wifi_psk()) return true;
  log_pushf("[wifi] no connection");
  return false;
}

// ============================ SSE HELPERS ============================
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

  uint32_t last_keepalive = millis();
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

    if (millis() - last_keepalive > 5000) {
      if (httpd_resp_send_chunk(req, ": keepalive\n\n", 13) != ESP_OK) return ESP_FAIL;
      last_keepalive = millis();
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

static esp_err_t flash_handler(httpd_req_t *req) {
  char query[64] = {0};
  bool on = false;

  if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
    char val[8] = {0};
    if (httpd_query_key_value(query, "on", val, sizeof(val)) == ESP_OK) {
      on = (strcmp(val, "1") == 0);
    }
  }

  set_flash(on);
  log_pushf("[flash] %s", on ? "ON" : "OFF");

  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_sendstr(req, on ? "on" : "off");
}

// ============================ OPTIMIZED CAPTURE ============================

static esp_err_t capture_handler(httpd_req_t *req) {
  uint32_t t0 = millis();
  
  // Switch to capture mode
  set_capture_mode();
  vTaskDelay(pdMS_TO_TICKS(10));
  
  // Flush stale frame
  camera_fb_t *fb = esp_camera_fb_get();
  if (fb) esp_camera_fb_return(fb);
  
  // Get fresh frame
  fb = esp_camera_fb_get();
  if (!fb) {
    log_pushf("[cap] failed");
    set_stream_mode();
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=\"capture.jpg\"");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  esp_err_t res = httpd_resp_send(req, (const char*)fb->buf, fb->len);
  
  log_pushf("[cap] %ux%u %uB %ums", fb->width, fb->height, fb->len, millis() - t0);
  
  esp_camera_fb_return(fb);
  set_stream_mode();
  
  return res;
}

// ============================ OPTIMIZED STREAM ============================

static esp_err_t stream_handler(httpd_req_t *req) {
  log_pushf("[stream] started");
  
  set_stream_mode();
  
  httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  static const char* BOUNDARY = "\r\n--frame\r\n";
  static const char* PART_HDR = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
  char part_buf[64];

  uint32_t frame_count = 0;
  uint32_t total_bytes = 0;
  uint32_t fps_timer = millis();

  // Flush initial stale frames
  for (int i = 0; i < 2; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
  }

  while (true) {
    uint32_t frame_start = millis();
    
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    if (httpd_resp_send_chunk(req, BOUNDARY, strlen(BOUNDARY)) != ESP_OK) {
      esp_camera_fb_return(fb);
      break;
    }

    size_t hlen = snprintf(part_buf, sizeof(part_buf), PART_HDR, fb->len);
    if (httpd_resp_send_chunk(req, part_buf, hlen) != ESP_OK) {
      esp_camera_fb_return(fb);
      break;
    }

    if (httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len) != ESP_OK) {
      esp_camera_fb_return(fb);
      break;
    }

    total_bytes += fb->len;
    esp_camera_fb_return(fb);
    frame_count++;

    // Adaptive delay
    uint32_t frame_time = millis() - frame_start;
    int32_t delay_needed = MIN_FRAME_TIME_MS - frame_time;
    if (delay_needed > 1) {
      vTaskDelay(pdMS_TO_TICKS(delay_needed));
    } else {
      vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Log stats every 5 seconds
    uint32_t now = millis();
    if (now - fps_timer >= 5000) {
      float fps = frame_count * 1000.0f / (now - fps_timer);
      float kbps = total_bytes * 8.0f / (now - fps_timer);
      log_pushf("[stream] %.1f fps, %.0f kbps", fps, kbps);
      frame_count = 0;
      total_bytes = 0;
      fps_timer = now;
    }
  }

  httpd_resp_send_chunk(req, NULL, 0);
  log_pushf("[stream] ended");
  return ESP_OK;
}

// ============================ SD CARD HANDLERS ============================

static esp_err_t sd_status_handler(httpd_req_t *req) {
  char json[256];
  
  if (!g_sd_available) {
    snprintf(json, sizeof(json), "{\"available\":false}");
  } else {
    uint64_t total = SD_MMC.totalBytes() / (1024 * 1024);
    uint64_t used = SD_MMC.usedBytes() / (1024 * 1024);
    snprintf(json, sizeof(json),
             "{\"available\":true,\"total_mb\":%llu,\"used_mb\":%llu,\"free_mb\":%llu,\"recording\":%s}",
             total, used, total - used, g_is_recording ? "true" : "false");
  }
  
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_sendstr(req, json);
}

static esp_err_t sd_list_handler(httpd_req_t *req) {
  if (!g_sd_available) {
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"files\":[]}");
  }
  
  String json = "{\"files\":[";
  bool first = true;
  
  // Photos
  File dir = SD_MMC.open("/photos");
  if (dir) {
    File file = dir.openNextFile();
    while (file) {
      if (!file.isDirectory()) {
        if (!first) json += ",";
        first = false;
        json += "{\"name\":\"" + String(file.name()) + 
                "\",\"path\":\"/photos/" + String(file.name()) + 
                "\",\"size\":" + String(file.size()) + 
                ",\"type\":\"photo\"}";
      }
      file = dir.openNextFile();
    }
    dir.close();
  }
  
  // Videos
  dir = SD_MMC.open("/videos");
  if (dir) {
    File file = dir.openNextFile();
    while (file) {
      if (!file.isDirectory()) {
        if (!first) json += ",";
        first = false;
        json += "{\"name\":\"" + String(file.name()) + 
                "\",\"path\":\"/videos/" + String(file.name()) + 
                "\",\"size\":" + String(file.size()) + 
                ",\"type\":\"video\"}";
      }
      file = dir.openNextFile();
    }
    dir.close();
  }
  
  json += "]}";
  
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_sendstr(req, json.c_str());
}

static esp_err_t sd_download_handler(httpd_req_t *req) {
  if (!g_sd_available) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No SD");
    return ESP_FAIL;
  }
  
  char query[128] = {0};
  char filepath[96] = {0};
  
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
      httpd_query_key_value(query, "file", filepath, sizeof(filepath)) != ESP_OK) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing file");
    return ESP_FAIL;
  }
  
  String path = filepath;
  path.replace("%2F", "/");
  
  if (path.indexOf("..") >= 0) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid path");
    return ESP_FAIL;
  }
  
  File file = SD_MMC.open(path.c_str(), FILE_READ);
  if (!file) {
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Not found");
    return ESP_FAIL;
  }
  
  const char* ctype = path.endsWith(".jpg") ? "image/jpeg" : "application/octet-stream";
  int lastSlash = path.lastIndexOf('/');
  String fname = (lastSlash >= 0) ? path.substring(lastSlash + 1) : path;
  
  char disp[128];
  snprintf(disp, sizeof(disp), "attachment; filename=\"%s\"", fname.c_str());
  
  httpd_resp_set_type(req, ctype);
  httpd_resp_set_hdr(req, "Content-Disposition", disp);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  
  const size_t CHUNK = 4096;
  uint8_t* buf = (uint8_t*)malloc(CHUNK);
  if (!buf) {
    file.close();
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory");
    return ESP_FAIL;
  }
  
  while (file.available()) {
    size_t n = file.read(buf, CHUNK);
    if (n > 0 && httpd_resp_send_chunk(req, (char*)buf, n) != ESP_OK) break;
  }
  
  free(buf);
  file.close();
  httpd_resp_send_chunk(req, NULL, 0);
  
  return ESP_OK;
}

static esp_err_t sd_delete_handler(httpd_req_t *req) {
  if (!g_sd_available) {
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"success\":false}");
  }
  
  char query[128] = {0};
  char filepath[96] = {0};
  
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
      httpd_query_key_value(query, "file", filepath, sizeof(filepath)) != ESP_OK) {
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"success\":false}");
  }
  
  String path = filepath;
  path.replace("%2F", "/");
  
  bool ok = SD_MMC.remove(path.c_str());
  log_pushf("[sd] delete %s: %s", path.c_str(), ok ? "OK" : "FAIL");
  
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_sendstr(req, ok ? "{\"success\":true}" : "{\"success\":false}");
}

// ============================ HTML UI ============================

static esp_err_t index_handler(httpd_req_t *req) {
  log_pushf("[ui] loaded");

  static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover"/>
  <title>Joint_CM v2.1</title>
  <style>
    :root{--r:16px;--b:1px solid #e8e8e8;--pad:14px}
    *{box-sizing:border-box}
    body{font-family:system-ui,-apple-system,sans-serif;margin:12px;max-width:1400px;padding-bottom:env(safe-area-inset-bottom)}
    h2{margin:0 0 10px;font-size:22px}
    .row{display:flex;gap:10px;flex-wrap:wrap;align-items:center}
    .grow{flex:1}
    button{padding:10px 12px;border:1px solid #cfcfcf;border-radius:12px;background:#fff;cursor:pointer;font-size:14px}
    button.primary{border-color:#111}
    button.danger{border-color:#b00020;color:#b00020}
    button:disabled{opacity:.5;cursor:not-allowed}
    .pill{display:inline-block;padding:6px 10px;border-radius:999px;border:var(--b);font-size:13px;background:#fff}
    .pill.rec{background:#ff4444;color:#fff;border-color:#ff4444;animation:pulse 1s infinite}
    @keyframes pulse{0%,100%{opacity:1}50%{opacity:.7}}
    .muted{opacity:.75}
    .panel{border:var(--b);border-radius:var(--r);padding:var(--pad);margin-top:12px;background:#fff}
    .layout{display:grid;grid-template-columns:1fr 380px;gap:12px;align-items:start}
    @media(max-width:900px){.layout{grid-template-columns:1fr}}
    .toolbar{display:flex;gap:10px;flex-wrap:wrap;align-items:center;justify-content:space-between;margin-top:8px}
    .toolbarLeft,.toolbarRight{display:flex;gap:10px;flex-wrap:wrap;align-items:center}
    .previewWrap{border:var(--b);border-radius:50%;overflow:hidden;position:relative;width:100%;max-width:720px;aspect-ratio:1/1;margin-top:10px;background:#f4f4f4}
    .previewWrap.idle{background:linear-gradient(135deg,#f0f0f0,#dedede)}
    .previewWrap img{width:100%;height:100%;object-fit:cover;display:block;transform:rotate(180deg)}
    .placeholder{width:100%;height:100%;display:flex;align-items:center;justify-content:center;color:#666;font-size:14px;text-align:center;padding:12px}
    .overlayTag{position:absolute;top:10px;left:10px;background:rgba(0,0,0,.55);color:#fff;padding:6px 10px;border-radius:999px;font-size:12px;backdrop-filter:blur(6px)}
    .downloadBtn{position:absolute;bottom:10px;right:10px;background:rgba(0,0,0,.55);color:#fff;padding:8px 12px;border-radius:12px;font-size:12px;backdrop-filter:blur(6px);border:none;cursor:pointer}
    .galleryHead{display:flex;align-items:center;gap:10px;margin-top:12px}
    .tabs{display:flex;gap:5px}
    .tab{padding:6px 12px;border:var(--b);border-radius:8px;cursor:pointer;font-size:13px;background:#fff}
    .tab.active{background:#111;color:#fff;border-color:#111}
    .thumbs{display:flex;gap:10px;flex-wrap:wrap;margin-top:10px}
    .thumb{width:100px;aspect-ratio:1/1;border:var(--b);border-radius:14px;overflow:hidden;background:#fff;position:relative}
    .thumb img{width:100%;height:100%;object-fit:cover;cursor:pointer;transform:rotate(180deg)}
    .thumb .badge{position:absolute;bottom:4px;left:4px;background:rgba(0,0,0,.6);color:#fff;padding:2px 6px;border-radius:6px;font-size:10px}
    .sidebar{display:flex;flex-direction:column;gap:12px}
    .sdPanel{border:var(--b);border-radius:var(--r);padding:var(--pad);background:#fff}
    .sdHead{display:flex;align-items:center;gap:10px;margin-bottom:10px}
    .sdBar{height:8px;background:#eee;border-radius:4px;overflow:hidden;margin:8px 0}
    .sdBarFill{height:100%;background:linear-gradient(90deg,#4CAF50,#8BC34A);border-radius:4px}
    .sdFiles{max-height:250px;overflow:auto}
    .sdFile{display:flex;align-items:center;gap:8px;padding:6px;border-bottom:1px solid #eee;font-size:12px}
    .sdFile .name{flex:1;word-break:break-all}
    .sdFile button{padding:4px 8px;font-size:11px}
    .term{border:var(--b);border-radius:var(--r);padding:var(--pad);background:#fff}
    .termHead{display:flex;gap:8px;align-items:center;margin-bottom:10px}
    .termBox{height:350px;overflow:auto;background:#3a3a3a;color:#f1f1f1;border-radius:14px;padding:10px;font-family:ui-monospace,monospace;font-size:11px;line-height:1.3;white-space:pre-wrap;word-break:break-word}
    .fps{position:absolute;top:10px;right:10px;background:rgba(0,0,0,.55);color:#0f0;padding:4px 8px;border-radius:8px;font-size:11px;font-family:monospace}
  </style>
</head>
<body>
  <h2>Joint_CM v2.1 ⚡</h2>
  <div class="layout">
    <div>
      <div class="row">
        <span class="pill" id="modePill">Photo</span>
        <span class="pill muted" id="statusPill">Idle</span>
        <span class="pill rec" id="recPill" style="display:none">● REC</span>
        <div class="grow"></div>
        <button id="photoBtn" class="primary">📷 Photo</button>
        <button id="videoBtn">🎬 Video</button>
      </div>
      <div class="panel">
        <div class="toolbar">
          <div class="toolbarLeft">
            <button id="captureBtn" class="primary">Capture</button>
            <button id="startBtn">▶ Stream</button>
            <button id="stopBtn" class="danger" disabled>■ Stop</button>
          </div>
          <div class="toolbarRight">
            <button id="flashOn">💡 On</button>
            <button id="flashOff">Off</button>
          </div>
        </div>
        <div class="previewWrap idle" id="previewWrap">
          <div class="overlayTag" id="overlay">Idle</div>
          <div class="fps" id="fpsDisplay" style="display:none">-- fps</div>
          <div class="placeholder" id="placeholder">Ready<br><small>Button: Click=Photo, Hold=Video</small></div>
          <img id="previewImg" alt="" style="display:none"/>
          <button class="downloadBtn" id="dlBtn" style="display:none">⬇ Download</button>
        </div>
        <div class="galleryHead">
          <strong>Gallery</strong>
          <div class="tabs">
            <div class="tab active" data-tab="mem">Memory</div>
            <div class="tab" data-tab="sd">SD Card</div>
          </div>
          <span class="pill muted" id="galCount">0</span>
          <div class="grow"></div>
          <button id="refreshBtn">↻</button>
          <button id="clearBtn">Clear</button>
        </div>
        <div class="thumbs" id="thumbs"></div>
      </div>
    </div>
    <aside class="sidebar">
      <div class="sdPanel">
        <div class="sdHead">
          <strong>💾 SD Card</strong>
          <span class="pill muted" id="sdPill">...</span>
          <div class="grow"></div>
          <button id="sdRefresh">↻</button>
        </div>
        <div id="sdStatus">Loading...</div>
        <div class="sdBar"><div class="sdBarFill" id="sdBar" style="width:0"></div></div>
        <div class="sdFiles" id="sdFiles"></div>
      </div>
      <div class="term">
        <div class="termHead">
          <strong>Terminal</strong>
          <span class="pill muted" id="termPill">...</span>
          <div class="grow"></div>
          <button id="termClear">Clear</button>
        </div>
        <div class="termBox" id="termBox"></div>
      </div>
    </aside>
  </div>
<script>
const $ = id => document.getElementById(id);
let mode='photo',streaming=false,memGal=[],sdGal=[],tab='mem',curBlob=null;

// FPS tracking
let frameCount=0,lastFpsTime=Date.now();

function setStatus(t){$('statusPill').textContent=t}
function setOverlay(t){$('overlay').textContent=t}

function showIdle(t){
  streaming=false;
  $('previewWrap').classList.add('idle');
  $('previewImg').style.display='none';
  $('placeholder').style.display='flex';
  $('placeholder').innerHTML=t;
  $('dlBtn').style.display='none';
  $('fpsDisplay').style.display='none';
  setOverlay('Idle');
  curBlob=null;
}

function showImg(src,label,blob=null){
  $('previewWrap').classList.remove('idle');
  $('placeholder').style.display='none';
  $('previewImg').style.display='block';
  $('previewImg').src=src;
  setOverlay(label);
  curBlob=blob;
  $('dlBtn').style.display=blob?'block':'none';
}

function setMode(m){
  mode=m;
  if(streaming)stopStream();
  if(m==='photo'){
    $('modePill').textContent='Photo';
    $('captureBtn').disabled=false;
    $('startBtn').disabled=true;
    $('stopBtn').disabled=true;
    $('photoBtn').classList.add('primary');
    $('videoBtn').classList.remove('primary');
    showIdle('Photo mode ready');
    setStatus('Photo ready');
  }else{
    $('modePill').textContent='Video';
    $('captureBtn').disabled=true;
    $('startBtn').disabled=false;
    $('stopBtn').disabled=true;
    $('photoBtn').classList.remove('primary');
    $('videoBtn').classList.add('primary');
    showIdle('Video mode ready');
    setStatus('Video ready');
  }
}

function updateGal(){
  const items=tab==='mem'?memGal:sdGal;
  $('galCount').textContent=items.length;
  $('thumbs').innerHTML='';
  items.forEach((it,i)=>{
    const d=document.createElement('div');d.className='thumb';
    const img=document.createElement('img');
    img.src=it.type==='mem'?it.url:`/sd/download?file=${encodeURIComponent(it.path)}`;
    img.onclick=()=>{
      if(it.type==='mem')showImg(it.url,'Photo',it.blob);
      else window.open(`/sd/download?file=${encodeURIComponent(it.path)}`);
    };
    const badge=document.createElement('div');badge.className='badge';
    badge.textContent=it.isVideo?'🎬':'📷';
    d.appendChild(img);d.appendChild(badge);
    $('thumbs').prepend(d);
  });
}

function addMem(blob){
  const url=URL.createObjectURL(blob);
  memGal.push({type:'mem',url,blob,size:blob.size});
  if(tab==='mem')updateGal();
}

async function capture(){
  setStatus('Capturing...');setOverlay('Capturing');
  $('previewWrap').classList.remove('idle');
  $('placeholder').style.display='flex';
  $('placeholder').textContent='Capturing...';
  $('previewImg').style.display='none';
  try{
    const r=await fetch('/capture?t='+Date.now(),{cache:'no-store'});
    if(!r.ok)throw 0;
    const blob=await r.blob();
    const url=URL.createObjectURL(blob);
    showImg(url,'Photo',blob);
    setStatus('Captured');
    addMem(blob);
  }catch(e){
    showIdle('Capture failed');
    setStatus('Error');
  }
}

function startStream(){
  setStatus('Streaming...');
  streaming=true;
  $('startBtn').disabled=true;
  $('stopBtn').disabled=false;
  $('dlBtn').style.display='none';
  $('fpsDisplay').style.display='block';
  frameCount=0;lastFpsTime=Date.now();
  
  const img=$('previewImg');
  img.onload=()=>{
    frameCount++;
    const now=Date.now();
    if(now-lastFpsTime>=1000){
      const fps=frameCount*1000/(now-lastFpsTime);
      $('fpsDisplay').textContent=fps.toFixed(1)+' fps';
      frameCount=0;lastFpsTime=now;
    }
  };
  showImg('/stream?'+Date.now(),'Live');
}

function stopStream(){
  streaming=false;
  $('startBtn').disabled=false;
  $('stopBtn').disabled=true;
  $('previewImg').onload=null;
  $('previewImg').src='';
  setTimeout(()=>showIdle('Stopped'),100);
  setStatus('Stopped');
}

async function flash(on){
  setStatus(on?'Flash on':'Flash off');
  try{await fetch('/flash?on='+(on?'1':'0'))}catch{}
}

function clearGal(){
  if(tab==='mem'){memGal.forEach(x=>URL.revokeObjectURL(x.url));memGal=[];}
  updateGal();setStatus('Cleared');
}

async function loadSD(){
  try{
    const r=await fetch('/sd/status');
    const d=await r.json();
    if(d.available){
      $('sdPill').textContent='OK';
      $('sdStatus').textContent=`${d.used_mb}/${d.total_mb}MB`;
      $('sdBar').style.width=(d.used_mb/d.total_mb*100)+'%';
      if(d.recording){$('recPill').style.display='inline-block';}
      else{$('recPill').style.display='none';}
    }else{
      $('sdPill').textContent='No Card';$('sdStatus').textContent='Not available';
    }
  }catch{$('sdPill').textContent='Error';}
  
  try{
    const r=await fetch('/sd/list');
    const d=await r.json();
    sdGal=(d.files||[]).map(f=>({type:'sd',name:f.name,path:f.path,size:f.size,isVideo:f.type==='video'}));
    $('sdFiles').innerHTML='';
    sdGal.slice().reverse().forEach(f=>{
      const div=document.createElement('div');div.className='sdFile';
      div.innerHTML=`<span>${f.isVideo?'🎬':'📷'}</span><span class="name">${f.name}</span><button onclick="window.open('/sd/download?file=${encodeURIComponent(f.path)}')">⬇</button>`;
      $('sdFiles').appendChild(div);
    });
    if(tab==='sd')updateGal();
  }catch{}
}

$('dlBtn').onclick=()=>{
  if(curBlob){
    const a=document.createElement('a');
    a.href=URL.createObjectURL(curBlob);
    a.download='capture_'+Date.now()+'.jpg';
    a.click();
  }
};

document.querySelectorAll('.tab').forEach(t=>{
  t.onclick=()=>{
    document.querySelectorAll('.tab').forEach(x=>x.classList.remove('active'));
    t.classList.add('active');
    tab=t.dataset.tab;
    updateGal();
  };
});

// SSE Terminal
let es;
function connectTerm(){
  $('termPill').textContent='Connecting...';
  es=new EventSource('/events');
  es.onopen=()=>$('termPill').textContent='Live';
  es.onerror=()=>$('termPill').textContent='Offline';
  es.onmessage=e=>{
    if(e.data){
      const d=document.createElement('div');
      d.textContent=e.data;
      if(e.data.includes('[stream]'))d.style.color='#8f8';
      if(e.data.includes('[rec]')||e.data.includes('[btn]'))d.style.color='#ff8';
      $('termBox').appendChild(d);
      while($('termBox').childNodes.length>500)$('termBox').removeChild($('termBox').firstChild);
      $('termBox').scrollTop=$('termBox').scrollHeight;
    }
  };
}
connectTerm();

$('termClear').onclick=async()=>{$('termBox').innerHTML='';try{await fetch('/log/clear')}catch{}};
$('photoBtn').onclick=()=>setMode('photo');
$('videoBtn').onclick=()=>setMode('video');
$('captureBtn').onclick=capture;
$('startBtn').onclick=startStream;
$('stopBtn').onclick=stopStream;
$('flashOn').onclick=()=>flash(true);
$('flashOff').onclick=()=>flash(false);
$('clearBtn').onclick=clearGal;
$('refreshBtn').onclick=loadSD;
$('sdRefresh').onclick=loadSD;

setMode('photo');
updateGal();
loadSD();
setInterval(loadSD,5000);
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
  config.max_uri_handlers = 12;
  config.stack_size = 8192;  // Increase stack for streaming

  if (httpd_start(&g_httpd, &config) != ESP_OK) {
    log_pushf("[http] start failed");
    return;
  }

  httpd_uri_t uris[] = {
    {"/",           HTTP_GET, index_handler,       NULL},
    {"/capture",    HTTP_GET, capture_handler,     NULL},
    {"/stream",     HTTP_GET, stream_handler,      NULL},
    {"/flash",      HTTP_GET, flash_handler,       NULL},
    {"/events",     HTTP_GET, events_handler,      NULL},
    {"/log/clear",  HTTP_GET, log_clear_handler,   NULL},
    {"/sd/status",  HTTP_GET, sd_status_handler,   NULL},
    {"/sd/list",    HTTP_GET, sd_list_handler,     NULL},
    {"/sd/download",HTTP_GET, sd_download_handler, NULL},
    {"/sd/delete",  HTTP_GET, sd_delete_handler,   NULL},
  };

  for (auto& u : uris) httpd_register_uri_handler(g_httpd, &u);
  
  log_pushf("[http] server ready");
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

  log_pushf("[ready] Click=Photo, Hold=Video");
}

// ============================ LOOP ============================
void loop() {
  process_button_events();

  uint32_t now = millis();
  if (now - g_last_status_ms > 5000) {
    g_last_status_ms = now;
    log_pushf("[stat] up=%us wifi=%s rssi=%d heap=%u sd=%s%s",
              (now - g_boot_ms) / 1000,
              WiFi.status() == WL_CONNECTED ? "OK" : "DOWN",
              WiFi.RSSI(),
              ESP.getFreeHeap(),
              g_sd_available ? "OK" : "NO",
              g_is_recording ? " REC" : "");
  }

  delay(g_is_recording ? 5 : 20);
}
