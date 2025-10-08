#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"

#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

const char* ssid = "ASUS_68";
const char* password = "kitchen_9544";

static httpd_handle_t control_httpd = NULL;
static httpd_handle_t stream_httpd  = NULL;

// ===== /set handler (port 80) =====
static esp_err_t set_handler(httpd_req_t *req) {
  sensor_t *s = esp_camera_sensor_get();
  if (!s) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no sensor");
    return ESP_FAIL;
  }

  char buf[256];
  auto get_i = [&](const char* key, int def=0) {
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
      char val[16];
      if (httpd_query_key_value(buf, key, val, sizeof(val)) == ESP_OK) {
        return atoi(val);
      }
    }
    return def;
  };
  auto get_s = [&](const char* key) {
    String out;
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
      char val[16];
      if (httpd_query_key_value(buf, key, val, sizeof(val)) == ESP_OK) {
        out = val;
      }
    }
    return out;
  };

  String size = get_s("size");
  if (size.length()) {
    framesize_t fs = FRAMESIZE_QVGA;
    if (size == "qqvga") fs = FRAMESIZE_QQVGA;
    else if (size == "vga") fs = FRAMESIZE_VGA;
    s->set_framesize(s, fs);
  }
  if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
    int v;
    v = get_i("quality", -1000);    if (v != -1000) s->set_quality(s, v);            // 10..30
    v = get_i("brightness", 999);   if (v != 999)   s->set_brightness(s, v);         // -2..2
    v = get_i("contrast", 999);     if (v != 999)   s->set_contrast(s, v);           // -2..2
    v = get_i("saturation", 999);   if (v != 999)   s->set_saturation(s, v);         // -2..2
    v = get_i("aec", 999);          if (v != 999)   s->set_exposure_ctrl(s, v!=0);   // 0/1
    v = get_i("ae_level", 999);     if (v != 999)   s->set_ae_level(s, v);           // -2..2
    v = get_i("aec_value", -1);     if (v >= 0)     s->set_aec_value(s, v);          // 0..1200
    v = get_i("agc", 999);          if (v != 999)   s->set_gain_ctrl(s, v!=0);       // 0/1
    v = get_i("agc_gain", -1);      if (v >= 0)     s->set_agc_gain(s, v);           // 0..30
    v = get_i("awb", 999);          if (v != 999)   s->set_whitebal(s, v!=0);        // 0/1
    v = get_i("wb_mode", -1);       if (v >= 0)     s->set_wb_mode(s, v);            // 0..4
    v = get_i("hmirror", 999);      if (v != 999)   s->set_hmirror(s, v!=0);
    v = get_i("vflip", 999);        if (v != 999)   s->set_vflip(s, v!=0);
  }

  httpd_resp_set_type(req, "text/plain");
  httpd_resp_send(req, "ok", 2);
  return ESP_OK;
}

static const httpd_uri_t uri_set = {
  .uri      = "/set",
  .method   = HTTP_GET,
  .handler  = set_handler,
  .user_ctx = NULL
};

#define PART_BOUNDARY "frameboundary"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace; boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY     = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART         = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t stream_handler(httpd_req_t *req) {
  esp_err_t res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;

  char part_buf[64];

  while (true) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      res = ESP_FAIL;
      break;
    }

    if ((res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY))) != ESP_OK) {
      esp_camera_fb_return(fb);
      break;
    }
    // write headers
    size_t hlen = snprintf(part_buf, sizeof(part_buf), _STREAM_PART, fb->len);
    if ((res = httpd_resp_send_chunk(req, part_buf, hlen)) != ESP_OK) {
      esp_camera_fb_return(fb);
      break;
    }
    // write jpeg
    if ((res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len)) != ESP_OK) {
      esp_camera_fb_return(fb);
      break;
    }
    esp_camera_fb_return(fb);
    // small yield
    vTaskDelay(1);
  }
  return res;
}

static const httpd_uri_t uri_stream = {
  .uri      = "/stream",
  .method   = HTTP_GET,
  .handler  = stream_handler,
  .user_ctx = NULL
};

void startServers() {
  // control (port 80)
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.ctrl_port = 32768;  // default
  if (httpd_start(&control_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(control_httpd, &uri_set);
  }

  // stream (port 81)
  httpd_config_t stream_cfg = HTTPD_DEFAULT_CONFIG();
  stream_cfg.server_port = 81;
  stream_cfg.ctrl_port = 32769;
  stream_cfg.recv_wait_timeout = 1;
  stream_cfg.send_wait_timeout = 10;
  if (httpd_start(&stream_httpd, &stream_cfg) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &uri_stream);
  }
}

void setup() {
  Serial.begin(115200);

  camera_config_t c{};
  c.ledc_channel=LEDC_CHANNEL_0; c.ledc_timer=LEDC_TIMER_0;
  c.pin_d0=Y2_GPIO_NUM; c.pin_d1=Y3_GPIO_NUM; c.pin_d2=Y4_GPIO_NUM; c.pin_d3=Y5_GPIO_NUM;
  c.pin_d4=Y6_GPIO_NUM; c.pin_d5=Y7_GPIO_NUM; c.pin_d6=Y8_GPIO_NUM; c.pin_d7=Y9_GPIO_NUM;
  c.pin_xclk=XCLK_GPIO_NUM; c.pin_pclk=PCLK_GPIO_NUM; c.pin_vsync=VSYNC_GPIO_NUM; c.pin_href=HREF_GPIO_NUM;
  c.pin_sccb_sda=SIOD_GPIO_NUM; c.pin_sccb_scl=SIOC_GPIO_NUM; c.pin_pwdn=PWDN_GPIO_NUM; c.pin_reset=RESET_GPIO_NUM;
  c.xclk_freq_hz=20000000; c.pixel_format=PIXFORMAT_JPEG;
  c.grab_mode=CAMERA_GRAB_LATEST; c.fb_location=CAMERA_FB_IN_PSRAM;
  if (psramFound()) { c.frame_size=FRAMESIZE_QVGA; c.jpeg_quality=20; c.fb_count=2; }
  else              { c.frame_size=FRAMESIZE_QQVGA; c.jpeg_quality=24; c.fb_count=1; }

  if (esp_camera_init(&c)!=ESP_OK) { Serial.println("Cam init fail"); return; }

  WiFi.mode(WIFI_STA); WiFi.setSleep(false);
  WiFi.begin(ssid, password);
  while (WiFi.status()!=WL_CONNECTED) { delay(300); Serial.print("."); }
  Serial.printf("\nIP: %s\n", WiFi.localIP().toString().c_str());

  startServers();
  Serial.println("Stream:  http://<IP>:81/stream");
  Serial.println("Control: http://<IP>/set?contrast=1&brightness=1");
}

void loop() { }
