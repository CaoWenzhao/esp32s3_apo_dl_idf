#include "web_control.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"

static const char *TAG = "web_control";

typedef struct {
    SemaphoreHandle_t lock;
    web_control_config_t config;
    web_control_command_t command;
    web_control_telemetry_t telemetry;
    uint64_t last_command_us;
} web_state_t;

static web_state_t s_state;

static float clampf(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static uint64_t now_us(void)
{
    return (uint64_t)esp_timer_get_time();
}

static void state_lock(void)
{
    xSemaphoreTake(s_state.lock, portMAX_DELAY);
}

static void state_unlock(void)
{
    xSemaphoreGive(s_state.lock);
}

web_control_config_t web_control_default_config(void)
{
    web_control_config_t config = {
        .ap_ssid = "Algorithm6-Control",
        .ap_password = "",
        .command_timeout_ms = 1000,
        .max_manual_linear_mps = 0.8f,
        .max_manual_angular_rps = 1.6f,
        .default_follow_speed_pct = 100,
        .default_follow_turn_pct = 100,
        .default_remote_speed_pct = 100,
    };
    return config;
}

static esp_err_t send_text(httpd_req_t *request, const char *text,
                           const char *content_type)
{
    httpd_resp_set_type(request, content_type);
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, text, HTTPD_RESP_USE_STRLEN);
}

static const char s_index_html[] =
    "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1,viewport-fit=cover'>"
    "<title>Follow / Remote Control</title><style>"
    "*{box-sizing:border-box}body{font-family:Arial,sans-serif;background:#101114;color:#f2f3f5;margin:0;"
    "padding:16px;max-width:720px;margin-inline:auto;-webkit-user-select:none;user-select:none;-webkit-touch-callout:none}"
    ".top{display:flex;align-items:center;justify-content:space-between;gap:12px;margin-bottom:12px}"
    "h1{font-size:28px;margin:0;letter-spacing:0}.pill{font-size:13px;padding:7px 10px;border-radius:8px;background:#34373d;color:#d8dbe0}"
    ".pill.ok{background:#087f5b;color:#fff}.pill.bad{background:#8b2f35;color:#fff}"
    "button{border:0;border-radius:8px;background:#30343b;color:#f4f5f6;font-size:17px;padding:13px 16px;"
    "touch-action:none;-webkit-user-select:none;user-select:none;-webkit-touch-callout:none}button:active{filter:brightness(1.2)}"
    "#estop{width:100%;height:82px;background:#d60000;font-size:34px;font-weight:700;margin-bottom:10px}"
    ".actions{display:grid;grid-template-columns:1fr 1fr;gap:10px}.arm{background:#009768;font-weight:700}"
    ".state{background:#4a4d53}.cards{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin:12px 0}"
    ".card{background:#191d22;border:1px solid #3a4654;border-radius:8px;padding:12px;min-height:92px}"
    ".label{font-size:13px;color:#aeb8c4}.value{font-size:27px;color:#83c5ff;font-weight:700;margin-top:6px;overflow-wrap:anywhere}"
    ".sub{font-size:12px;color:#9da7b2;margin-top:3px}.panel{background:#191d22;border:1px solid #3a4654;border-radius:8px;padding:12px;margin:12px 0}"
    ".modes{display:grid;grid-template-columns:1fr 1fr;gap:10px}.mode.active{background:#087ed1;color:#fff}"
    ".slider-head{display:flex;justify-content:space-between;color:#c4ccd5;font-size:14px;margin-top:13px}"
    "input[type=range]{width:100%;height:28px;accent-color:#1689e8;margin:0}"
    ".dpad{display:grid;grid-template-columns:78px 78px 78px;grid-template-rows:78px 78px 78px;gap:9px;justify-content:center}"
    ".drive{width:78px;height:78px;padding:0;background:#354558;font-size:34px;line-height:1}.drive.active{background:#1689e8}"
    ".drive.stop{background:#80363a;font-size:17px;font-weight:700}.blank{width:78px;height:78px}"
    ".sensor-row{display:grid;grid-template-columns:repeat(3,1fr);gap:7px;margin-top:10px}.sensor{padding:8px;border-radius:6px;background:#34373d;"
    "font-size:12px;text-align:center;color:#c7ccd2}.sensor.ok{background:#176346;color:#fff}.sensor.bad{background:#653238;color:#fff}"
    ".summary{font-size:14px;line-height:1.55;color:#c8d0d9}.summary strong{color:#83c5ff;font-size:20px}"
    ".error{min-height:18px;color:#ff7b7b;font-size:13px}details{margin-top:10px;color:#aeb8c4}"
    "pre{white-space:pre-wrap;overflow-wrap:anywhere;background:#111318;padding:10px;border-radius:6px;font-size:11px;color:#c8d0d9}"
    "@media(max-width:380px){body{padding:10px}h1{font-size:24px}.dpad{grid-template-columns:72px 72px 72px;grid-template-rows:72px 72px 72px}.drive,.blank{width:72px;height:72px}}"
    "</style></head><body>"
    "<div class='top'><h1>Follow / Remote</h1><span id='connection' class='pill bad'>Connecting</span></div>"
    "<button id='estop' onclick='emergencyStop()'>STOP</button>"
    "<div class='actions'><button class='arm' onclick='arm()'>CLEAR / ARM</button><button id='stateButton' class='state'>BOOT</button></div>"
    "<div class='cards'>"
    "<div class='card'><div class='label'>Battery</div><div id='battery' class='value'>--%</div><div id='batterySub' class='sub'>-- V</div></div>"
    "<div class='card'><div class='label'>Weight</div><div id='weight' class='value'>-- kg</div><div id='fsrSub' class='sub'>FSR</div></div>"
    "<div class='card'><div class='label'>Target distance</div><div id='distance' class='value'>-- m</div><div class='sub'>UWB</div></div>"
    "<div class='card'><div class='label'>Target bearing</div><div id='bearing' class='value'>-- deg</div><div class='sub'>Left + / Right -</div></div>"
    "<div class='card'><div class='label'>Lidar front</div><div id='lidarDistance' class='value'>-- m</div><div class='sub'>Forward cone</div></div>"
    "<div class='card'><div class='label'>Left ultrasonic</div><div id='ultraLeftDistance' class='value'>-- m</div><div class='sub'>Left side</div></div>"
    "<div class='card'><div class='label'>Right ultrasonic</div><div id='ultraRightDistance' class='value'>-- m</div><div class='sub'>Right side</div></div>"
    "</div>"
    "<div class='panel'><div class='modes'>"
    "<button id='autoMode' class='mode' onclick=\"setMode('auto')\">&#36319;&#38543;&#27169;&#24335;</button>"
    "<button id='manualMode' class='mode' onclick=\"setMode('manual')\">&#36965;&#25511;&#27169;&#24335;</button></div>"
    "<div class='slider-head'><span>Follow speed</span><b><span id='followValue'>100</span>%</b></div>"
    "<input id='followSpeed' type='range' min='0' max='100' value='100' oninput='speedChanged()' onchange='speedChanged(true)'>"
    "<div class='slider-head'><span>Follow turn</span><b><span id='turnValue'>100</span>%</b></div>"
    "<input id='followTurn' type='range' min='0' max='100' value='100' oninput='speedChanged()' onchange='speedChanged(true)'>"
    "<div class='slider-head'><span>Remote speed</span><b><span id='remoteValue'>100</span>%</b></div>"
    "<input id='remoteSpeed' type='range' min='0' max='100' value='100' oninput='speedChanged()' onchange='speedChanged(true)'></div>"
    "<div id='remotePanel' class='panel'><div class='dpad'>"
    "<div class='blank'></div><button class='drive' data-v='1' data-w='0'>&#9650;</button><div class='blank'></div>"
    "<button class='drive' data-v='0' data-w='1'>&#9664;</button><button class='drive stop' data-stop='1'>STOP</button>"
    "<button class='drive' data-v='0' data-w='-1'>&#9654;</button>"
    "<div class='blank'></div><button class='drive' data-v='-1' data-w='0'>&#9660;</button><div class='blank'></div>"
    "</div></div>"
    "<div class='panel summary'><strong id='modeText'>BOOT</strong><br>"
    "Motor pulse: <span id='pulses'>1500 / 1500 us</span><br>Measured: <span id='motion'>0.00 m/s, 0.00 rad/s</span>"
    "<div class='sensor-row'><span id='uwbChip' class='sensor'>UWB</span><span id='lidarChip' class='sensor'>LIDAR</span>"
    "<span id='ultraLChip' class='sensor'>ULTRA L</span><span id='ultraRChip' class='sensor'>ULTRA R</span>"
    "<span id='encoderChip' class='sensor'>ENCODER</span><span id='fsrChip' class='sensor'>FSR</span></div></div>"
    "<div id='error' class='error'></div><details><summary>Live data</summary><pre id='telemetry'>Loading...</pre></details>"
    "<script>"
    "const byId=id=>document.getElementById(id);let hold=null,holdV=0,holdW=0,pollBusy=false,speedTimer=null,editingUntil=0;"
    "const fmt=(x,d)=>Number.isFinite(Number(x))?Number(x).toFixed(d):'--';"
    "async function post(url){try{const r=await fetch(url,{method:'POST',cache:'no-store'});if(!r.ok)throw new Error('HTTP '+r.status);byId('error').textContent='';return true;}catch(e){byId('error').textContent='Command failed: '+e.message;return false;}}"
    "async function emergencyStop(){stopDrive();await post('/estop');poll();}"
    "async function arm(){await post('/clear');poll();}"
    "async function setMode(mode){stopDrive();await post('/mode?value='+mode);poll();}"
    "function updateSpeedLabels(){byId('followValue').textContent=byId('followSpeed').value;byId('turnValue').textContent=byId('followTurn').value;byId('remoteValue').textContent=byId('remoteSpeed').value;}"
    "function sendSpeeds(){const u='/speed?follow='+byId('followSpeed').value+'&turn='+byId('followTurn').value+'&remote='+byId('remoteSpeed').value;post(u);}"
    "function speedChanged(now){editingUntil=Date.now()+1600;updateSpeedLabels();clearTimeout(speedTimer);if(now)sendSpeeds();else speedTimer=setTimeout(sendSpeeds,70);}"
    "function sendDrive(v,w){holdV=Number(v);holdW=Number(w);post('/cmd?v='+holdV+'&w='+holdW);}"
    "function startDrive(button,pointerId){if(button.dataset.stop){stopDrive();return;}stopDrive(false);holdV=Number(button.dataset.v);holdW=Number(button.dataset.w);button.classList.add('active');try{button.setPointerCapture(pointerId);}catch(e){}sendDrive(holdV,holdW);hold=setInterval(()=>sendDrive(holdV,holdW),180);}"
    "function stopDrive(send=true){if(hold){clearInterval(hold);hold=null;}document.querySelectorAll('.drive.active').forEach(b=>b.classList.remove('active'));holdV=0;holdW=0;if(send)post('/cmd?v=0&w=0');}"
    "function chip(id,ok){const e=byId(id);e.className='sensor '+(ok?'ok':'bad');}"
    "function render(j){const connected=!!j.client&&!j.estop;byId('connection').textContent=j.estop?'E-STOP':(j.client?'Connected':'Timed out');byId('connection').className='pill '+(connected?'ok':'bad');"
    "byId('stateButton').textContent=j.state||'BOOT';byId('modeText').textContent=(j.mode==='auto'?'FOLLOW':'REMOTE')+' / '+(j.state||'BOOT');"
    "byId('autoMode').className='mode '+(j.mode==='auto'?'active':'');byId('manualMode').className='mode '+(j.mode==='manual'?'active':'');byId('remotePanel').style.display=j.mode==='manual'?'block':'none';"
    "if(Date.now()>editingUntil&&!['followSpeed','followTurn','remoteSpeed'].includes(document.activeElement&&document.activeElement.id)){byId('followSpeed').value=j.follow_speed_pct;byId('followTurn').value=j.follow_turn_pct;byId('remoteSpeed').value=j.remote_speed_pct;}updateSpeedLabels();"
    "byId('distance').textContent=fmt(j.target_m,2)+' m';byId('bearing').textContent=fmt(Number(j.bearing_rad)*57.2958,1)+' deg';"
    "byId('lidarDistance').textContent=Number(j.lidar_front_m)>0?fmt(j.lidar_front_m,2)+' m':'-- m';byId('ultraLeftDistance').textContent=j.ultra_left?fmt(j.ultra_left_m,2)+' m':'-- m';byId('ultraRightDistance').textContent=j.ultra_right?fmt(j.ultra_right_m,2)+' m':'-- m';byId('battery').textContent=fmt(j.battery_pct,0)+'%';byId('batterySub').textContent=fmt(j.battery_v,2)+' V';byId('weight').textContent=fmt(j.fsr_kg,1)+' kg';byId('fsrSub').textContent=fmt(j.fsr_v,3)+' V / raw '+j.fsr_raw;"
    "byId('pulses').textContent=j.left_us+' / '+j.right_us+' us';byId('motion').textContent=fmt(j.measured_v,2)+' m/s, '+fmt(j.measured_w,2)+' rad/s';"
    "chip('uwbChip',j.uwb);chip('lidarChip',j.lidar);chip('ultraLChip',j.ultra_left);chip('ultraRChip',j.ultra_right);chip('encoderChip',j.encoder);chip('fsrChip',j.fsr);byId('telemetry').textContent=JSON.stringify(j,null,2);}"
    "async function poll(){if(pollBusy)return;pollBusy=true;try{const r=await fetch('/status',{cache:'no-store'});if(!r.ok)throw new Error('HTTP '+r.status);render(await r.json());}catch(e){byId('connection').textContent='Disconnected';byId('connection').className='pill bad';}finally{pollBusy=false;}}"
    "document.addEventListener('contextmenu',e=>{if(e.target.closest('.drive'))e.preventDefault()});document.addEventListener('selectstart',e=>{if(e.target.closest('.drive'))e.preventDefault()});"
    "document.addEventListener('pointerdown',e=>{const b=e.target.closest('.drive');if(!b)return;e.preventDefault();startDrive(b,e.pointerId)});"
    "document.addEventListener('pointerup',e=>{if(e.target.closest('.drive')){e.preventDefault();stopDrive()}});document.addEventListener('pointercancel',()=>stopDrive());window.addEventListener('blur',()=>stopDrive());"
    "setInterval(()=>post('/heartbeat'),300);setInterval(poll,350);poll();"
    "</script></body></html>";

static esp_err_t root_handler(httpd_req_t *request)
{
    return send_text(request, s_index_html, "text/html");
}

static void mark_command_received(void)
{
    s_state.last_command_us = now_us();
    s_state.command.client_alive = true;
}

void web_control_heartbeat(void)
{
    state_lock();
    mark_command_received();
    state_unlock();
}

void web_control_emergency_stop(void)
{
    state_lock();
    s_state.command.estop_latched = true;
    s_state.command.manual_linear_mps = 0.0f;
    s_state.command.manual_angular_rps = 0.0f;
    mark_command_received();
    state_unlock();
}

void web_control_arm(void)
{
    state_lock();
    s_state.command.estop_latched = false;
    s_state.command.manual_linear_mps = 0.0f;
    s_state.command.manual_angular_rps = 0.0f;
    mark_command_received();
    state_unlock();
}

esp_err_t web_control_set_mode(web_control_mode_t mode)
{
    if (mode != WEB_CONTROL_MODE_AUTO && mode != WEB_CONTROL_MODE_MANUAL) {
        return ESP_ERR_INVALID_ARG;
    }
    state_lock();
    s_state.command.mode = mode;
    s_state.command.manual_linear_mps = 0.0f;
    s_state.command.manual_angular_rps = 0.0f;
    mark_command_received();
    state_unlock();
    return ESP_OK;
}

esp_err_t web_control_set_manual(float linear, float angular)
{
    state_lock();
    if (s_state.command.mode != WEB_CONTROL_MODE_MANUAL) {
        state_unlock();
        return ESP_ERR_INVALID_STATE;
    }
    const float scale = (float)s_state.command.remote_speed_pct / 100.0f;
    s_state.command.manual_linear_mps =
        clampf(linear, -1.0f, 1.0f) *
        s_state.config.max_manual_linear_mps * scale;
    s_state.command.manual_angular_rps =
        clampf(angular, -1.0f, 1.0f) *
        s_state.config.max_manual_angular_rps * scale;
    mark_command_received();
    state_unlock();
    return ESP_OK;
}

esp_err_t web_control_set_speeds(uint8_t follow_speed, uint8_t follow_turn,
                                 uint8_t remote_speed)
{
    if (follow_speed > 100 || follow_turn > 100 || remote_speed > 100) {
        return ESP_ERR_INVALID_ARG;
    }
    state_lock();
    s_state.command.follow_speed_pct = follow_speed;
    s_state.command.follow_turn_pct = follow_turn;
    s_state.command.remote_speed_pct = remote_speed;
    mark_command_received();
    state_unlock();
    return ESP_OK;
}

static esp_err_t heartbeat_handler(httpd_req_t *request)
{
    web_control_heartbeat();
    return send_text(request, "OK", "text/plain");
}

static esp_err_t estop_handler(httpd_req_t *request)
{
    web_control_emergency_stop();
    return send_text(request, "ESTOP", "text/plain");
}

static esp_err_t clear_handler(httpd_req_t *request)
{
    web_control_arm();
    return send_text(request, "ARMED", "text/plain");
}

static bool get_query_value(httpd_req_t *request, const char *key,
                            char *value, size_t value_size)
{
    char query[128];
    if (httpd_req_get_url_query_str(request, query, sizeof(query)) != ESP_OK) {
        return false;
    }
    return httpd_query_key_value(query, key, value, value_size) == ESP_OK;
}

static bool get_query_percent(httpd_req_t *request, const char *key,
                              uint8_t *value)
{
    char text[8];
    if (!get_query_value(request, key, text, sizeof(text))) {
        return false;
    }
    char *end = NULL;
    const long parsed = strtol(text, &end, 10);
    if (end == text || *end != '\0' || parsed < 0 || parsed > 100) {
        return false;
    }
    *value = (uint8_t)parsed;
    return true;
}

static esp_err_t mode_handler(httpd_req_t *request)
{
    char value[16];
    if (!get_query_value(request, "value", value, sizeof(value))) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "missing mode");
    }
    web_control_mode_t mode;
    if (strcmp(value, "auto") == 0) {
        mode = WEB_CONTROL_MODE_AUTO;
    } else if (strcmp(value, "manual") == 0) {
        mode = WEB_CONTROL_MODE_MANUAL;
    } else {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "invalid mode");
    }
    return web_control_set_mode(mode) == ESP_OK
               ? send_text(request, "OK", "text/plain")
               : httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                     "invalid mode");
}

static esp_err_t command_handler(httpd_req_t *request)
{
    char linear_text[24];
    char angular_text[24];
    if (!get_query_value(request, "v", linear_text, sizeof(linear_text)) ||
        !get_query_value(request, "w", angular_text, sizeof(angular_text))) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "missing command");
    }
    char *linear_end = NULL;
    char *angular_end = NULL;
    const float linear = strtof(linear_text, &linear_end);
    const float angular = strtof(angular_text, &angular_end);
    if (linear_end == linear_text || *linear_end != '\0' ||
        angular_end == angular_text || *angular_end != '\0') {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "invalid command");
    }
    if (web_control_set_manual(linear, angular) != ESP_OK) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "manual mode required");
    }
    return send_text(request, "OK", "text/plain");
}

static esp_err_t speed_handler(httpd_req_t *request)
{
    uint8_t follow_speed;
    uint8_t follow_turn;
    uint8_t remote_speed;
    if (!get_query_percent(request, "follow", &follow_speed) ||
        !get_query_percent(request, "turn", &follow_turn) ||
        !get_query_percent(request, "remote", &remote_speed)) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "speed values must be 0..100");
    }
    web_control_set_speeds(follow_speed, follow_turn, remote_speed);
    return send_text(request, "OK", "text/plain");
}

static esp_err_t status_handler(httpd_req_t *request)
{
    web_control_command_t command;
    web_control_telemetry_t telemetry;
    web_control_get_command(&command);
    state_lock();
    telemetry = s_state.telemetry;
    state_unlock();
    char json[1024];
    const int length = snprintf(
        json, sizeof(json),
        "{\"mode\":\"%s\",\"estop\":%s,\"client\":%s,\"age_ms\":%lu,"
        "\"follow_speed_pct\":%u,\"follow_turn_pct\":%u,"
        "\"remote_speed_pct\":%u,"
        "\"state\":\"%s\",\"uwb\":%s,\"lidar\":%s,\"ultra_left\":%s,"
        "\"ultra_right\":%s,\"fsr\":%s,\"battery\":%s,\"encoder\":%s,"
        "\"target_m\":%.3f,\"bearing_rad\":%.3f,\"clearance_m\":%.3f,"
        "\"clearance_left_m\":%.3f,\"clearance_right_m\":%.3f,\"avoid_heading_rad\":%.3f,"
        "\"lidar_front_m\":%.3f,\"ultra_left_m\":%.3f,\"ultra_right_m\":%.3f,"
        "\"fsr_v\":%.3f,\"fsr_kg\":%.3f,\"fsr_raw\":%d,"
        "\"battery_v\":%.3f,\"battery_pct\":%.1f,\"battery_adc_v\":%.3f,\"battery_raw\":%d,"
        "\"measured_v\":%.3f,\"measured_w\":%.3f,"
        "\"left_us\":%d,\"right_us\":%d}",
        command.mode == WEB_CONTROL_MODE_AUTO ? "auto" : "manual",
        command.estop_latched ? "true" : "false",
        command.client_alive ? "true" : "false",
        (unsigned long)command.command_age_ms,
        (unsigned)command.follow_speed_pct,
        (unsigned)command.follow_turn_pct,
        (unsigned)command.remote_speed_pct,
        telemetry.state == NULL ? "BOOT" : telemetry.state,
        telemetry.uwb_ok ? "true" : "false",
        telemetry.lidar_ok ? "true" : "false",
        telemetry.ultrasonic_left_ok ? "true" : "false",
        telemetry.ultrasonic_right_ok ? "true" : "false",
        telemetry.fsr_ok ? "true" : "false",
        telemetry.battery_ok ? "true" : "false",
        telemetry.encoder_ok ? "true" : "false",
        telemetry.target_distance_m, telemetry.target_bearing_rad,
        telemetry.front_clearance_m, telemetry.left_clearance_m,
        telemetry.right_clearance_m, telemetry.chosen_heading_rad,
        telemetry.front_clearance_m, telemetry.left_clearance_m,
        telemetry.right_clearance_m,
        telemetry.fsr_voltage_v,
        telemetry.fsr_weight_kg, telemetry.fsr_raw,
        telemetry.battery_voltage_v, telemetry.battery_percent,
        telemetry.battery_adc_voltage_v, telemetry.battery_raw,
        telemetry.measured_linear_mps, telemetry.measured_angular_rps,
        telemetry.left_pulse_us, telemetry.right_pulse_us);
    if (length < 0 || length >= (int)sizeof(json)) {
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "status overflow");
    }
    return send_text(request, json, "application/json");
}

static esp_err_t register_uri(httpd_handle_t server, const char *uri,
                              httpd_method_t method, esp_err_t (*handler)(httpd_req_t *))
{
    const httpd_uri_t descriptor = {
        .uri = uri,
        .method = method,
        .handler = handler,
        .user_ctx = NULL,
    };
    return httpd_register_uri_handler(server, &descriptor);
}

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)base;
    (void)data;
    if (id == WIFI_EVENT_AP_STADISCONNECTED && s_state.lock != NULL) {
        state_lock();
        s_state.command.client_alive = false;
        s_state.command.manual_linear_mps = 0.0f;
        s_state.command.manual_angular_rps = 0.0f;
        state_unlock();
    }
}

static esp_err_t start_wifi(const web_control_config_t *config)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ret = nvs_flash_erase();
        if (ret == ESP_OK) {
            ret = nvs_flash_init();
        }
    }
    if (ret != ESP_OK) {
        return ret;
    }
    ret = esp_netif_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }
    if (esp_netif_create_default_wifi_ap() == NULL) {
        return ESP_FAIL;
    }
    const wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&init);
    if (ret != ESP_OK) {
        return ret;
    }
    const wifi_country_t country = {
        .cc = "CN",
        .schan = 1,
        .nchan = 13,
        .max_tx_power = 78,
        .policy = WIFI_COUNTRY_POLICY_MANUAL,
    };
    ret = esp_wifi_set_country(&country);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL);
    if (ret != ESP_OK) {
        return ret;
    }
    wifi_config_t wifi = {0};
    strlcpy((char *)wifi.ap.ssid, config->ap_ssid, sizeof(wifi.ap.ssid));
    wifi.ap.ssid_len = strlen((char *)wifi.ap.ssid);
    wifi.ap.channel = 6;
    wifi.ap.ssid_hidden = 0;
    wifi.ap.beacon_interval = 100;
    wifi.ap.max_connection = 2;
    const size_t password_length = strlen(config->ap_password);
    if (password_length >= 8) {
        strlcpy((char *)wifi.ap.password, config->ap_password,
                sizeof(wifi.ap.password));
        wifi.ap.authmode = WIFI_AUTH_WPA2_PSK;
    } else {
        wifi.ap.authmode = WIFI_AUTH_OPEN;
    }
    ret = esp_wifi_set_mode(WIFI_MODE_AP);
    if (ret == ESP_OK) {
        ret = esp_wifi_set_config(WIFI_IF_AP, &wifi);
    }
    if (ret == ESP_OK) {
        ret = esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11B |
                                                     WIFI_PROTOCOL_11G |
                                                     WIFI_PROTOCOL_11N);
    }
    if (ret == ESP_OK) {
        ret = esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20);
    }
    if (ret == ESP_OK) {
        ret = esp_wifi_start();
    }
    if (ret == ESP_OK) {
        ret = esp_wifi_set_max_tx_power(78);
    }
    if (ret == ESP_OK) {
        uint8_t primary = 0;
        wifi_second_chan_t secondary = WIFI_SECOND_CHAN_NONE;
        int8_t tx_power = 0;
        esp_wifi_get_channel(&primary, &secondary);
        esp_wifi_get_max_tx_power(&tx_power);
        ESP_LOGI(TAG, "AP radio: channel=%u bandwidth=20MHz tx=%.1fdBm visible=1",
                 primary, (double)tx_power / 4.0);
    }
    return ret;
}

esp_err_t web_control_init(const web_control_config_t *config)
{
    if (config == NULL || config->ap_ssid == NULL ||
        config->ap_password == NULL || config->command_timeout_ms == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(&s_state, 0, sizeof(s_state));
    s_state.lock = xSemaphoreCreateMutex();
    if (s_state.lock == NULL) {
        return ESP_ERR_NO_MEM;
    }
    s_state.config = *config;
    s_state.command.mode = WEB_CONTROL_MODE_AUTO;
    s_state.command.estop_latched = true;
    s_state.command.follow_speed_pct = config->default_follow_speed_pct <= 100
                                           ? config->default_follow_speed_pct
                                           : 100;
    s_state.command.follow_turn_pct = config->default_follow_turn_pct <= 100
                                          ? config->default_follow_turn_pct
                                          : 100;
    s_state.command.remote_speed_pct = config->default_remote_speed_pct <= 100
                                           ? config->default_remote_speed_pct
                                           : 100;
    esp_err_t ret = start_wifi(config);
    if (ret != ESP_OK) {
        return ret;
    }
    httpd_config_t server_config = HTTPD_DEFAULT_CONFIG();
    server_config.max_uri_handlers = 9;
    httpd_handle_t server = NULL;
    ret = httpd_start(&server, &server_config);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = register_uri(server, "/", HTTP_GET, root_handler);
    if (ret == ESP_OK) ret = register_uri(server, "/heartbeat", HTTP_POST, heartbeat_handler);
    if (ret == ESP_OK) ret = register_uri(server, "/estop", HTTP_POST, estop_handler);
    if (ret == ESP_OK) ret = register_uri(server, "/clear", HTTP_POST, clear_handler);
    if (ret == ESP_OK) ret = register_uri(server, "/mode", HTTP_POST, mode_handler);
    if (ret == ESP_OK) ret = register_uri(server, "/cmd", HTTP_POST, command_handler);
    if (ret == ESP_OK) ret = register_uri(server, "/speed", HTTP_POST, speed_handler);
    if (ret == ESP_OK) ret = register_uri(server, "/status", HTTP_GET, status_handler);
    if (ret != ESP_OK) {
        httpd_stop(server);
        return ret;
    }
    ESP_LOGI(TAG, "control AP ready: SSID=%s (credentials are not logged)",
             config->ap_ssid);
    return ESP_OK;
}

void web_control_get_command(web_control_command_t *command)
{
    if (command == NULL || s_state.lock == NULL) {
        return;
    }
    state_lock();
    const uint64_t current_us = now_us();
    const uint64_t age_us = current_us >= s_state.last_command_us
                                ? current_us - s_state.last_command_us
                                : UINT64_MAX;
    const uint64_t timeout_us = (uint64_t)s_state.config.command_timeout_ms * 1000ULL;
    if (s_state.last_command_us == 0 || age_us > timeout_us) {
        s_state.command.client_alive = false;
        s_state.command.manual_linear_mps = 0.0f;
        s_state.command.manual_angular_rps = 0.0f;
    }
    s_state.command.command_age_ms = age_us == UINT64_MAX
                                         ? UINT32_MAX
                                         : (uint32_t)(age_us / 1000ULL);
    *command = s_state.command;
    state_unlock();
}

void web_control_publish(const web_control_telemetry_t *telemetry)
{
    if (telemetry == NULL || s_state.lock == NULL) {
        return;
    }
    state_lock();
    s_state.telemetry = *telemetry;
    state_unlock();
}

void web_control_get_snapshot(web_control_command_t *command,
                              web_control_telemetry_t *telemetry)
{
    if (command != NULL) {
        web_control_get_command(command);
    }
    if (telemetry != NULL && s_state.lock != NULL) {
        state_lock();
        *telemetry = s_state.telemetry;
        state_unlock();
    }
}
