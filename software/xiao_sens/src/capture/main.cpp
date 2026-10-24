// Dataset-capture firmware for the XIAO ESP32S3 Sense.
//
// Port 80:  /            live view + sensor controls
//           /capture     one JPEG, exactly as the tracker will see it
//           /control?var=<name>&val=<int>   change a sensor setting
//           /save        store the current settings in NVS (the tracker loads them)
//           /status      settings as JSON
// Port 81:  /stream      MJPEG (own server so it doesn't block /capture)
//
// tools/grab_frames.py pulls /capture into a dataset folder.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_http_server.h>
#include "secrets.h"
#include "xiao_cam.h"

static CamSettings cam;
static httpd_handle_t http_main   = nullptr;
static httpd_handle_t http_stream = nullptr;

static const char INDEX_HTML[] = R"HTML(<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>XIAO capture</title>
<style>
body{font-family:system-ui,sans-serif;margin:16px;background:#111;color:#eee}
img{max-width:100%;border:1px solid #444}
label{display:inline-block;width:9em}
.row{margin:4px 0}
button{padding:6px 14px;margin-right:6px}
#crop{position:relative;display:inline-block}
#sq{position:absolute;border:1px dashed #0f0;pointer-events:none}
</style></head><body>
<h3>XIAO ESP32S3 Sense - dataset capture</h3>
<div id="crop"><img id="v"><div id="sq"></div></div>
<p>Green square: the centre crop the model sees (Edge Impulse "fit shortest axis").</p>
<div id="ctl"></div>
<p><button onclick="save()">Save settings for tracker</button><span id="msg"></span></p>
<script>
const F=[["framesize",0,10,"0..10 (5=QVGA, 8=VGA)"],["quality",4,63],["brightness",-2,2],["contrast",-2,2],
["saturation",-2,2],["ae_level",-2,2],["aec",0,1],["aec_value",0,1200],["agc",0,1],["agc_gain",0,30],
["awb",0,1],["vflip",0,1],["hmirror",0,1]];
const v=document.getElementById('v');
v.src=location.protocol+'//'+location.hostname+':81/stream';
v.onload=()=>{const s=Math.min(v.width,v.height),q=document.getElementById('sq');
 q.style.width=q.style.height=s+'px';q.style.left=(v.width-s)/2+'px';q.style.top=(v.height-s)/2+'px';};
fetch('/status').then(r=>r.json()).then(st=>{
 const c=document.getElementById('ctl');
 for(const[n,lo,hi,h]of F){const d=document.createElement('div');d.className='row';
  d.innerHTML=`<label>${n}</label><input type=range min=${lo} max=${hi} value=${st[n]} id=${n}> <span id=${n}_v>${st[n]}</span> ${h||''}`;
  c.appendChild(d);const i=d.querySelector('input');
  i.onchange=()=>{document.getElementById(n+'_v').textContent=i.value;
   fetch(`/control?var=${n}&val=${i.value}`);};}
});
function save(){fetch('/save').then(r=>r.text()).then(t=>document.getElementById('msg').textContent=' '+t);}
</script></body></html>)HTML";

static esp_err_t h_index(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t h_capture(httpd_req_t* req) {
    // Drop one frame: with GRAB_LATEST the first buffer can predate a settings change.
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
    fb = esp_camera_fb_get();
    if (!fb) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    char hdr[48];
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    snprintf(hdr, sizeof(hdr), "%ux%u", fb->width, fb->height);
    httpd_resp_set_hdr(req, "X-Frame-Size", hdr);
    esp_err_t r = httpd_resp_send(req, (const char*)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    return r;
}

static esp_err_t h_stream(httpd_req_t* req) {
    static const char* BOUNDARY = "\r\n--frame\r\n";
    httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    char part[80];
    while (true) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) return ESP_FAIL;
        int n = snprintf(part, sizeof(part), "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", fb->len);
        esp_err_t r = httpd_resp_send_chunk(req, BOUNDARY, strlen(BOUNDARY));
        if (r == ESP_OK) r = httpd_resp_send_chunk(req, part, n);
        if (r == ESP_OK) r = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
        esp_camera_fb_return(fb);
        if (r != ESP_OK) return r;  // client went away
    }
}

static esp_err_t h_control(httpd_req_t* req) {
    char q[64], var[24], val[12];
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) != ESP_OK ||
        httpd_query_key_value(q, "var", var, sizeof(var)) != ESP_OK ||
        httpd_query_key_value(q, "val", val, sizeof(val)) != ESP_OK ||
        !xcam::set(cam, var, atoi(val))) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "usage: /control?var=<name>&val=<int>");
        return ESP_FAIL;
    }
    xcam::apply(cam);
    return httpd_resp_sendstr(req, "ok");
}

static esp_err_t h_save(httpd_req_t* req) {
    return httpd_resp_sendstr(req, xcam::save(cam) ? "saved" : "save failed");
}

static esp_err_t h_status(httpd_req_t* req) {
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_sendstr(req, xcam::to_json(cam).c_str());
}

static void start_servers() {
    httpd_config_t c = HTTPD_DEFAULT_CONFIG();
    c.server_port    = 80;
    c.ctrl_port      = 32768;
    if (httpd_start(&http_main, &c) == ESP_OK) {
        const httpd_uri_t uris[] = {
            {"/", HTTP_GET, h_index, nullptr},          {"/capture", HTTP_GET, h_capture, nullptr},
            {"/control", HTTP_GET, h_control, nullptr}, {"/save", HTTP_GET, h_save, nullptr},
            {"/status", HTTP_GET, h_status, nullptr},
        };
        for (auto& u : uris) httpd_register_uri_handler(http_main, &u);
    }
    c.server_port = 81;
    c.ctrl_port   = 32769;
    if (httpd_start(&http_stream, &c) == ESP_OK) {
        httpd_uri_t u = {"/stream", HTTP_GET, h_stream, nullptr};
        httpd_register_uri_handler(http_stream, &u);
    }
}

static void start_wifi() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.setHostname("xiao-capture");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.printf("WiFi: joining %s", WIFI_SSID);
    for (int i = 0; i < 30 && WiFi.status() != WL_CONNECTED; i++) {
        delay(500);
        Serial.print('.');
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\nWiFi: http://%s/\n", WiFi.localIP().toString().c_str());
        return;
    }
    // No network: run our own, so capture still works on a bare bench.
    WiFi.mode(WIFI_AP);
    WiFi.softAP("xiao-capture", "wormbot123");
    Serial.printf("\nWiFi: no network, AP 'xiao-capture' / 'wormbot123', http://%s/\n",
                  WiFi.softAPIP().toString().c_str());
}

void setup() {
    Serial.begin(115200);
    delay(500);
    if (!psramFound()) Serial.println("WARNING: no PSRAM - check board settings");
    bool saved = xcam::load(cam);
    Serial.printf("camera settings: %s %s\n", saved ? "loaded" : "defaults", xcam::to_json(cam).c_str());
    if (!xcam::begin(cam)) {
        Serial.println("camera init failed, halting");
        while (true) delay(1000);
    }
    start_wifi();
    start_servers();
}

void loop() {
    delay(1000);
}
