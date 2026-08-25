#ifndef WEB_DASHBOARD_H
#define WEB_DASHBOARD_H

#include "config.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

// ============================================================
// Web Dashboard Pro با پشتیبانی از همه ماژول‌ها
// ============================================================

class CANManager;
class UDSManager;
class OBD2Manager;
class SDLogger;
class BLERemote;
class RFManager;

class WebDashboard {
private:
    WebServer* server;
    WebSocketsServer* ws;
    CANManager* can;
    UDSManager* uds;
    OBD2Manager* obd2;
    SDLogger* sd;
    BLERemote* ble;
    RFManager* rf;
    
    bool initialized;
    char ap_ssid[32];
    char ap_pass[16];
    unsigned long last_broadcast_ms;
    
public:
    WebDashboard() : server(nullptr), ws(nullptr), 
                     can(nullptr), uds(nullptr),
                     obd2(nullptr), sd(nullptr),
                     ble(nullptr), rf(nullptr),
                     initialized(false), last_broadcast_ms(0) {
        strcpy(ap_ssid, WIFI_SSID);
        strcpy(ap_pass, WIFI_PASS);
    }
    
    void setCAN(CANManager* c) { can = c; }
    void setUDS(UDSManager* u) { uds = u; }
    void setOBD2(OBD2Manager* o) { obd2 = o; }
    void setSDLogger(SDLogger* s) { sd = s; }
    void setBLERemote(BLERemote* b) { ble = b; }
    void setRFManager(RFManager* r) { rf = r; }
    
    bool begin() {
        WiFi.mode(WIFI_AP);
        WiFi.softAP(ap_ssid, ap_pass);
        
        server = new WebServer(HTTP_PORT);
        ws = new WebSocketsServer(WEBSOCKET_PORT);
        
        server->on("/", HTTP_GET, [this]() { handleRoot(); });
        server->on("/api/status", HTTP_GET, [this]() { handleStatus(); });
        server->on("/api/vehicles", HTTP_GET, [this]() { handleVehicles(); });
        server->on("/api/sniffer", HTTP_GET, [this]() { handleSniffer(); });
        server->on("/api/learn", HTTP_GET, [this]() { handleLearn(); });
        server->on("/api/select", HTTP_POST, [this]() { handleSelect(); });
        server->on("/api/send", HTTP_POST, [this]() { handleSend(); });
        server->on("/api/uds", HTTP_POST, [this]() { handleUDS(); });
        server->on("/api/speed-detect", HTTP_GET, [this]() { handleSpeedDetect(); });
        server->on("/api/start-can", HTTP_POST, [this]() { handleStartCAN(); });
        server->on("/api/stop-can", HTTP_POST, [this]() { handleStopCAN(); });
        server->on("/api/obd2", HTTP_GET, [this]() { handleOBD2(); });
        server->on("/api/obd2-dtc", HTTP_GET, [this]() { handleOBD2DTC(); });
        server->on("/api/obd2-clear", HTTP_POST, [this]() { handleOBD2Clear(); });
        server->on("/api/sd", HTTP_GET, [this]() { handleSD(); });
        server->on("/api/sd-start", HTTP_POST, [this]() { handleSDStart(); });
        server->on("/api/sd-stop", HTTP_POST, [this]() { handleSDStop(); });
        server->on("/api/ble", HTTP_GET, [this]() { handleBLE(); });
        server->on("/api/rf", HTTP_GET, [this]() { handleRFStatus(); });
        server->on("/api/rf-capture", HTTP_POST, [this]() { handleRFCapture(); });
        server->on("/api/rf-transmit", HTTP_POST, [this]() { handleRFTransmit(); });
        server->on("/api/rf-rolljam", HTTP_POST, [this]() { handleRFRollJam(); });
        server->on("/api/rf-sniff", HTTP_GET, [this]() { handleRFSniff(); });
        
        server->begin();
        ws->begin();
        ws->onEvent([this](uint8_t num, WStype_t type, uint8_t* payload, size_t len) {
            if (type == WStype_TEXT) {
                handleWebSocket(num, (char*)payload);
            }
        });
        
        initialized = true;
        return true;
    }
    
    void loop() {
        if (!initialized) return;
        server->handleClient();
        ws->loop();
        if (millis() - last_broadcast_ms > 500) {
            broadcastStatus();
            last_broadcast_ms = millis();
        }
    }
    
private:
    void handleRoot() {
        String html = R"rawliteral(
<!DOCTYPE html>
<html lang="fa" dir="rtl">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>CarHack-ESP32 Pro</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { font-family: 'Segoe UI', Tahoma, sans-serif; background: #0a0a0f; color: #e0e0e0; }
  .container { max-width: 1200px; margin: 0 auto; padding: 20px; }
  h1 { color: #00ff88; text-align: center; margin-bottom: 30px; font-size: 2em; }
  h2 { color: #00ccff; margin-bottom: 15px; font-size: 1.3em; }
  h3 { color: #ffaa00; margin-bottom: 10px; font-size: 1.1em; }
  .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; }
  .grid-3 { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 20px; }
  .card { background: #141420; border: 1px solid #2a2a40; border-radius: 12px; padding: 20px; }
  .card-full { grid-column: 1 / -1; }
  .status-bar { display: flex; gap: 20px; flex-wrap: wrap; }
  .stat { background: #1a1a2e; padding: 10px 20px; border-radius: 8px; border-left: 3px solid #00ff88; }
  .stat-label { color: #888; font-size: 0.8em; }
  .stat-value { color: #fff; font-size: 1.2em; font-weight: bold; }
  select, button, input { 
    background: #1a1a2e; color: #e0e0e0; border: 1px solid #333; 
    border-radius: 8px; padding: 10px 15px; font-size: 1em; width: 100%;
    margin-bottom: 10px; cursor: pointer;
  }
  button { background: #00ff88; color: #000; font-weight: bold; border: none; }
  button:hover { background: #00cc66; }
  button.danger { background: #ff4444; }
  button.warn { background: #ffaa00; }
  button.blue { background: #0088ff; }
  .log { 
    background: #000; color: #00ff00; font-family: monospace; 
    padding: 15px; border-radius: 8px; height: 200px; overflow-y: auto;
    white-space: pre-wrap; font-size: 0.85em;
  }
  .data-grid { display: grid; grid-template-columns: auto auto auto auto; gap: 5px; font-family: monospace; }
  .data-grid .header { color: #00ccff; font-weight: bold; }
  .data-grid .cell { color: #fff; }
  .tab { overflow: hidden; border: 1px solid #333; border-radius: 8px; margin-bottom: 15px; }
  .tab button { background: #1a1a2e; float: left; border: none; width: auto; outline: none; cursor: pointer; padding: 10px 20px; transition: 0.3s; border-radius: 0; margin: 0; }
  .tab button.active { background: #00ff88; color: #000; }
  .tabcontent { display: none; padding: 15px 0; }
  .tabcontent.active { display: block; }
  .inline-group { display: flex; gap: 10px; }
  .inline-group button, .inline-group select { width: auto; flex: 1; }
  @media (max-width: 768px) { .grid, .grid-3 { grid-template-columns: 1fr; } }
</style>
</head>
<body>
<div class="container">
<h1>🚗 CarHack-ESP32 Pro v5.1</h1>

<!-- Status Bar -->
<div class="card card-full">
  <div class="status-bar" id="status-bar">
    <div class="stat"><div class="stat-label">CAN Bus</div><div class="stat-value" id="can-status">⏳</div></div>
    <div class="stat"><div class="stat-label">TX/RX</div><div class="stat-value" id="can-txrx">0/0</div></div>
    <div class="stat"><div class="stat-label">Speed</div><div class="stat-value" id="can-speed">-</div></div>
    <div class="stat"><div class="stat-label">Vehicle</div><div class="stat-value" id="vehicle-name">None</div></div>
    <div class="stat"><div class="stat-label">UDS</div><div class="stat-value" id="uds-status">🔒</div></div>
    <div class="stat"><div class="stat-label">BLE</div><div class="stat-value" id="ble-status">📴</div></div>
    <div class="stat"><div class="stat-label">SD</div><div class="stat-value" id="sd-status">💾</div></div>
    <div class="stat"><div class="stat-label">RF</div><div class="stat-value" id="rf-status">📻</div></div>
  </div>
</div>

<!-- Tabs -->
<div class="tab">
  <button class="tablinks active" onclick="openTab(event, 'can')">🔧 CAN</button>
  <button class="tablinks" onclick="openTab(event, 'obd2')">📊 OBD2</button>
  <button class="tablinks" onclick="openTab(event, 'sniffer')">📡 Sniffer</button>
  <button class="tablinks" onclick="openTab(event, 'uds')">🔑 UDS</button>
  <button class="tablinks" onclick="openTab(event, 'rf')">📻 RF</button>
  <button class="tablinks" onclick="openTab(event, 'ble')">📱 BLE</button>
  <button class="tablinks" onclick="openTab(event, 'sd')">💾 SD</button>
  <button class="tablinks" onclick="openTab(event, 'learn')">🧠 Learn</button>
</div>

<!-- ═══════════════ Tab: CAN ═══════════════ -->
<div id="can" class="tabcontent active">
  <div class="grid">
    <div class="card">
      <h2>انتخاب خودرو</h2>
      <select id="vehicle-select"><option value="-1">🔍 Auto Detect</option></select>
      <button onclick="selectVehicle()">✅ انتخاب</button>
      <button class="warn" onclick="speedDetect()">⚡ تشخیص سرعت CAN</button>
      <div class="inline-group">
        <button class="danger" onclick="startCAN()">▶️ شروع CAN</button>
        <button class="danger" onclick="stopCAN()">⏹ توقف</button>
      </div>
    </div>
    <div class="card">
      <h2>ارسال دستی CAN Frame</h2>
      <input type="text" id="can-id" placeholder="CAN ID (hex) مانند 7E0">
      <input type="text" id="can-data" placeholder="داده (hex) مانند 02 10 03">
      <button onclick="sendFrame()">📤 ارسال</button>
      <div class="log" id="can-log">CAN log...</div>
    </div>
  </div>
</div>

<!-- ═══════════════ Tab: OBD2 ═══════════════ -->
<div id="obd2" class="tabcontent">
  <div class="grid">
    <div class="card">
      <h2>📊 مقادیر زنده OBD2</h2>
      <button onclick="pollOBD2()">🔄 به‌روزرسانی</button>
      <div id="obd2-values" style="margin-top:10px; font-family:monospace;">
        برای به‌روزرسانی کلیک کنید...
      </div>
    </div>
    <div class="card">
      <h2>🚨 DTC (خطاها)</h2>
      <div class="inline-group">
        <button onclick="readDTCs()">📋 خواندن خطاها</button>
        <button class="danger" onclick="clearDTCs()">🗑️ پاک کردن</button>
      </div>
      <div id="obd2-dtc" style="margin-top:10px; font-family:monospace;">
        برای خواندن کلیک کنید...
      </div>
      <h3 style="margin-top:15px;">VIN</h3>
      <div id="obd2-vin" style="font-family:monospace; color:#00ff88;">نامشخص</div>
    </div>
  </div>
</div>

<!-- ═══════════════ Tab: Sniffer ═══════════════ -->
<div id="sniffer" class="tabcontent">
  <div class="grid">
    <div class="card">
      <h2>📡 Sniffer CAN</h2>
      <div class="inline-group">
        <button onclick="startSniff(3)">۳ ثانیه</button>
        <button onclick="startSniff(10)">۱۰ ثانیه</button>
        <button onclick="startSniff(30)">۳۰ ثانیه</button>
      </div>
      <button class="warn" onclick="clearSniffer()">🗑️ پاک کردن</button>
      <div id="sniffer-count" style="margin:10px 0;color:#00ff88;">تعداد فریم‌ها: ۰</div>
      <div class="log" id="sniffer-log">برای شروع کلیک کنید...</div>
    </div>
    <div class="card">
      <h2>فریم‌ها</h2>
      <div class="data-grid" id="sniffer-grid">
        <span class="header">CAN ID</span><span class="header">Len</span><span class="header">Data</span><span class="header">Time</span>
      </div>
    </div>
  </div>
</div>

<!-- ═══════════════ Tab: UDS ═══════════════ -->
<div id="uds" class="tabcontent">
  <div class="grid">
    <div class="card">
      <h2>🔑 جلسه UDS</h2>
      <button onclick="udsSession('extended')">🔓 Extended Session</button>
      <button onclick="udsSession('programming')">💾 Programming Session</button>
      <button onclick="udsSecurity()">🔐 Security Access (PSA)</button>
      <button onclick="udsReadVIN()">📋 Read VIN</button>
      <button class="warn" onclick="udsReset()">🔄 ECU Reset</button>
    </div>
    <div class="card">
      <h2>نتیجه</h2>
      <div class="log" id="uds-log">منتظر فرمان...</div>
    </div>
  </div>
</div>

<!-- ═══════════════ Tab: RF ═══════════════ -->
<div id="rf" class="tabcontent">
  <div class="grid">
    <div class="card">
      <h2>📻 RF CC1101</h2>
      <div id="rf-status-info" style="margin-bottom:10px;color:#888;">وضعیت: نامشخص</div>
      <div class="inline-group">
        <button onclick="rfCapture()">📥 ضبط فریم</button>
        <button onclick="rfTransmit()">📤 پخش فریم</button>
      </div>
      <button class="warn" onclick="rfRollJam()">⚡ RollJam Attack</button>
      <div class="inline-group">
        <button onclick="rfSniff(5)">Sniff 5s</button>
        <button onclick="rfSniff(10)">Sniff 10s</button>
      </div>
      <div class="log" id="rf-log" style="margin-top:10px;">برای ضبط کلیک کنید...</div>
    </div>
    <div class="card">
      <h2>فریم‌های ضبط‌شده</h2>
      <div id="rf-frames" style="font-family:monospace;">
        هیچ فریمی ضبط نشده
      </div>
    </div>
  </div>
</div>

<!-- ═══════════════ Tab: BLE ═══════════════ -->
<div id="ble" class="tabcontent">
  <div class="grid">
    <div class="card">
      <h2>📱 BLE Remote</h2>
      <div id="ble-info" style="margin-bottom:10px;color:#888;">وضعیت: ...</div>
      <button onclick="sendBLECommand('ping')">📡 Ping</button>
      <button onclick="sendBLECommand('status')">ℹ️ Status</button>
      <input type="text" id="ble-cmd" placeholder="دستور دلخواه">
      <button onclick="sendBLECommand(document.getElementById('ble-cmd').value)">📤 ارسال</button>
    </div>
    <div class="card">
      <h2>پیام‌ها</h2>
      <div class="log" id="ble-log">منتظر اتصال...</div>
    </div>
  </div>
</div>

<!-- ═══════════════ Tab: SD ═══════════════ -->
<div id="sd" class="tabcontent">
  <div class="grid">
    <div class="card">
      <h2>💾 SD Card Logger</h2>
      <div id="sd-info" style="margin-bottom:10px;color:#888;">وضعیت: ...</div>
      <div class="inline-group">
        <button onclick="startSD()">▶️ شروع Logging</button>
        <button class="danger" onclick="stopSD()">⏹ توقف</button>
      </div>
      <button onclick="refreshSD()">🔄 به‌روزرسانی</button>
    </div>
    <div class="card">
      <h2>آمار</h2>
      <div id="sd-stats" style="font-family:monospace;">
        در حال بارگذاری...
      </div>
    </div>
  </div>
</div>

<!-- ═══════════════ Tab: Auto Learn ═══════════════ -->
<div id="learn" class="tabcontent">
  <div class="grid">
    <div class="card">
      <h2>🧠 Auto CAN ID Learning</h2>
      <p style="color:#888;margin-bottom:15px;">شناسایی خودکار CAN IDهای خودرو</p>
      <div class="inline-group">
        <button onclick="startLearn(5)">۵ ثانیه</button>
        <button onclick="startLearn(15)">۱۵ ثانیه</button>
        <button onclick="startLearn(30)">۳۰ ثانیه</button>
      </div>
      <button class="danger" onclick="stopLearn()">⏹ توقف</button>
      <div id="learn-stats" style="color:#00ff88;margin:10px 0;">منتظر شروع...</div>
    </div>
    <div class="card">
      <h2>نتایج یادگیری</h2>
      <div class="log" id="learn-log">منتظر شروع یادگیری...</div>
    </div>
  </div>
</div>

</div>

<script>
let ws = new WebSocket('ws://' + location.hostname + ':81/');

ws.onmessage = function(event) {
  try {
    let data = JSON.parse(event.data);
    updateAll(data);
  } catch(e) {}
};

function updateAll(d) {
  // CAN
  if (d.status) document.getElementById('can-status').textContent = d.status;
  if (d.tx !== undefined && d.rx !== undefined)
    document.getElementById('can-txrx').textContent = d.tx + '/' + d.rx;
  if (d.speed) document.getElementById('can-speed').textContent = d.speed + ' kbps';
  if (d.vehicle) document.getElementById('vehicle-name').textContent = d.vehicle;
  if (d.uds_unlocked !== undefined)
    document.getElementById('uds-status').textContent = d.uds_unlocked ? '🔓' : '🔒';
  
  // OBD2
  if (d.obd2_ready !== undefined) {
    let el = document.getElementById('obd2-values');
    if (d.obd2_values) {
      let html = '';
      d.obd2_values.forEach(function(v) {
        html += v.name + ': <b>' + v.value + '</b> ' + v.unit + '<br>';
      });
      el.innerHTML = html;
    }
    if (d.obd2_vin) document.getElementById('obd2-vin').textContent = d.obd2_vin;
  }
  
  // Sniffer
  if (d.sniffer_count !== undefined)
    document.getElementById('sniffer-count').textContent = 'تعداد فریم‌ها: ' + d.sniffer_count;
  if (d.sniffer_frames) {
    let grid = document.getElementById('sniffer-grid');
    grid.innerHTML = '<span class="header">CAN ID</span><span class="header">Len</span><span class="header">Data</span><span class="header">Time</span>';
    d.sniffer_frames.forEach(function(f) {
      grid.innerHTML += '<span class="cell">0x' + f.id + '</span>';
      grid.innerHTML += '<span class="cell">' + f.len + '</span>';
      grid.innerHTML += '<span class="cell">' + f.data + '</span>';
      grid.innerHTML += '<span class="cell">' + (f.time || '') + '</span>';
    });
  }
  
  // BLE
  if (d.ble_connected !== undefined)
    document.getElementById('ble-info').textContent = d.ble_connected ? '✅ متصل' : '❌ قطع';
  if (d.ble_device) document.getElementById('ble-info').textContent += ' (' + d.ble_device + ')';
  
  // SD
  if (d.sd_card !== undefined) {
    let info = d.sd_card ? '💾 کارت SD: ✅' : '💾 کارت SD: ❌';
    if (d.sd_logging) info += ' | Logging: ✅ Frames: ' + (d.sd_frames || 0);
    else info += ' | Logging: ❌';
    document.getElementById('sd-info').textContent = info;
    document.getElementById('sd-stats').textContent = 
      'Session: #' + (d.sd_session || 0) + '\n' +
      'File: ' + (d.sd_file || '-') + '\n' +
      'Frames: ' + (d.sd_frames || 0);
  }
  
  // RF
  if (d.rf_present !== undefined) {
    let info = d.rf_present ? '📻 CC1101: ✅' : '📻 CC1101: ❌';
    if (d.rf_mode !== undefined) info += ' | Mode: ' + ['IDLE','RX','TX','ROLLJAM','SNIFFER'][d.rf_mode];
    if (d.rf_freq) info += ' | ' + d.rf_freq + ' MHz';
    if (d.rf_captured !== undefined) info += ' | Captured: ' + d.rf_captured;
    document.getElementById('rf-status-info').textContent = info;
  }
  
  // UDS log
  if (d.uds_result) document.getElementById('uds-log').textContent = d.uds_result;
  
  // Learn
  if (d.learn_status) document.getElementById('learn-stats').textContent = d.learn_status;
  if (d.learn_signals) document.getElementById('learn-log').textContent = d.learn_signals;
  
  // RF log
  if (d.rf_log) document.getElementById('rf-log').textContent = d.rf_log;
  
  // BLE log
  if (d.ble_log) {
    let log = document.getElementById('ble-log');
    let lines = log.innerHTML.split('\n');
    lines.push(new Date().toLocaleTimeString() + ' ' + d.ble_log);
    if (lines.length > 30) lines.shift();
    log.innerHTML = lines.join('\n');
    log.scrollTop = log.scrollHeight;
  }
  
  // CAN log
  if (d.log) {
    let log = document.getElementById('can-log');
    let lines = log.innerHTML.split('\n');
    lines.push(new Date().toLocaleTimeString() + ' ' + d.log);
    if (lines.length > 50) lines.shift();
    log.innerHTML = lines.join('\n');
    log.scrollTop = log.scrollHeight;
  }
}

function openTab(evt, tabName) {
  document.querySelectorAll('.tabcontent').forEach(function(t) { t.classList.remove('active'); });
  document.querySelectorAll('.tablinks').forEach(function(b) { b.classList.remove('active'); });
  document.getElementById(tabName).classList.add('active');
  evt.currentTarget.classList.add('active');
}

// Vehicle list
fetch('/api/vehicles').then(function(r) { return r.json(); }).then(function(data) {
  let sel = document.getElementById('vehicle-select');
  data.forEach(function(v, i) {
    let opt = document.createElement('option');
    opt.value = i;
    opt.text = v.make + ' ' + v.model + ' (' + v.year_start + '-' + v.year_end + ')';
    sel.appendChild(opt);
  });
});

// --- CAN ---
function selectVehicle() {
  let idx = document.getElementById('vehicle-select').value;
  fetch('/api/select', { method: 'POST', body: 'index=' + idx });
}
function speedDetect() {
  fetch('/api/speed-detect').then(function(r) { return r.json(); }).then(function(d) {
    updateAll({log: 'Speed: ' + d.speed + ' kbps'});
  });
}
function startCAN() { fetch('/api/start-can', { method: 'POST' }); }
function stopCAN() { fetch('/api/stop-can', { method: 'POST' }); }
function sendFrame() {
  let id = document.getElementById('can-id').value.trim();
  let data = document.getElementById('can-data').value.trim();
  if (!id || !data) { updateAll({log:'ERROR: Fill both fields'}); return; }
  fetch('/api/send', { method: 'POST', body: 'id=' + id + '&data=' + data });
}

// --- OBD2 ---
function pollOBD2() { fetch('/api/obd2').then(function(r) { return r.json(); }).then(function(d) { updateAll(d); }); }
function readDTCs() { fetch('/api/obd2-dtc').then(function(r) { return r.json(); }).then(function(d) { updateAll({log:'DTCs: '+d.count}); }); }
function clearDTCs() { fetch('/api/obd2-clear', {method:'POST'}); }

// --- Sniffer ---
function startSniff(sec) {
  updateAll({log:'Sniffing '+sec+'s...'});
  fetch('/api/sniffer?time='+sec).then(function(r) { return r.json(); }).then(function(d) { updateAll(d); });
}
function clearSniffer() {
  document.getElementById('sniffer-grid').innerHTML = '<span class="header">CAN ID</span><span class="header">Len</span><span class="header">Data</span><span class="header">Time</span>';
  document.getElementById('sniffer-count').textContent = 'تعداد فریم‌ها: ۰';
}

// --- UDS ---
function udsSession(type) {
  fetch('/api/uds', {method:'POST', body:'action=session&type='+type})
    .then(function(r) { return r.json(); }).then(function(d) { updateAll({uds_result:JSON.stringify(d,null,2)}); });
}
function udsSecurity() {
  fetch('/api/uds', {method:'POST', body:'action=security'})
    .then(function(r) { return r.json(); }).then(function(d) { updateAll({uds_result:JSON.stringify(d,null,2)}); });
}
function udsReadVIN() {
  fetch('/api/uds', {method:'POST', body:'action=vin'})
    .then(function(r) { return r.json(); }).then(function(d) { updateAll({uds_result:'VIN: '+(d.vin||'FAILED')}); });
}
function udsReset() {
  fetch('/api/uds', {method:'POST', body:'action=reset'})
    .then(function(r) { return r.json(); }).then(function(d) { updateAll({uds_result:JSON.stringify(d,null,2)}); });
}

// --- RF ---
function rfCapture() {
  fetch('/api/rf-capture', {method:'POST'})
    .then(function(r) { return r.json(); }).then(function(d) { updateAll({rf_log:'Captured: '+(d.ok?'✅':'❌')}); });
}
function rfTransmit() {
  fetch('/api/rf-transmit', {method:'POST'})
    .then(function(r) { return r.json(); }).then(function(d) { updateAll({rf_log:'Transmit: '+(d.ok?'✅':'❌')}); });
}
function rfRollJam() {
  updateAll({rf_log:'⚡ RollJam in progress...'});
  fetch('/api/rf-rolljam', {method:'POST'})
    .then(function(r) { return r.json(); }).then(function(d) { updateAll({rf_log:'RollJam: '+(d.ok?'✅ SUCCESS':'❌ FAILED')}); });
}
function rfSniff(sec) {
  fetch('/api/rf-sniff?time='+sec)
    .then(function(r) { return r.json(); }).then(function(d) { updateAll({rf_log:'Sniffed: '+d.count+' frames'}); });
}

// --- BLE ---
function sendBLECommand(cmd) {
  if (!cmd) return;
  fetch('/api/ble?cmd='+encodeURIComponent(cmd));
}

// --- SD ---
function startSD() { fetch('/api/sd-start', {method:'POST'}); }
function stopSD() { fetch('/api/sd-stop', {method:'POST'}); }
function refreshSD() { fetch('/api/sd').then(function(r) { return r.json(); }).then(function(d) { updateAll(d); }); }

// --- Learn ---
function startLearn(sec) {
  updateAll({learn_status:'⏳ Learning ('+sec+'s)...', learn_signals:'Collecting CAN frames...'});
  fetch('/api/learn?action=start&time='+sec);
}
function stopLearn() {
  fetch('/api/learn?action=stop').then(function(r) { return r.json(); }).then(function(d) {
    updateAll({learn_status:'✅ Done', learn_signals:JSON.stringify(d,null,2)});
  });
}
</script>
</body>
</html>
)rawliteral";

    server->send(200, "text/html", html);
  }

  void handleStatus() {
    String json = "{\n";
    
    if (can) {
      String canJson = can->toJSON();
      json += canJson.substring(1, canJson.length() - 1);
      if (uds || obd2 || sd || ble || rf) json += ",\n";
    }
    if (uds) {
      String udsJson = uds->toJSON();
      json += udsJson.substring(1, udsJson.length() - 1);
      if (obd2 || sd || ble || rf) json += ",\n";
    }
    if (obd2) {
      json += "\"obd2_ready\": true,\n";
      OBD2Value* vals;
      int vc = 0;
      vals = obd2->getValues(&vc);
      json += "\"obd2_values\": [\n";
      for (int i = 0; i < vc; i++) {
        if (i > 0) json += ",\n";
        json += "  {\"name\":\"" + String(vals[i].name) + "\",\"value\":" + String(vals[i].value, 1) + ",\"unit\":\"" + String(vals[i].unit) + "\"}";
      }
      json += "\n],\n";
      json += "\"obd2_vin\": \"" + String(obd2->getVIN()) + "\"\n";
      if (sd || ble || rf) json += ",\n";
    }
    if (sd) {
      String sdJson = sd->toJSON();
      json += sdJson.substring(1, sdJson.length() - 1);
      if (ble || rf) json += ",\n";
    }
    if (ble) {
      String bleJson = ble->toJSON();
      json += bleJson.substring(1, bleJson.length() - 1);
      if (rf) json += ",\n";
    }
    if (rf) {
      String rfJson = rf->toJSON();
      json += rfJson.substring(1, rfJson.length() - 1);
    }
    
    json += "\n}";
    server->send(200, "application/json", json);
  }

  void handleVehicles() {
    server->send(200, "application/json", VehicleDB::toJSON());
  }

  void handleSniffer() {
    if (!can || !can->isRunning()) {
      server->send(200, "application/json", "{\"count\":0,\"frames\":[]}");
      return;
    }
    int time = server->arg("time").toInt();
    if (time <= 0) time = 3;
    
    int count = can->sniff(time * 1000, true);
    
    String json = "{\n";
    json += "\"sniffer_count\": " + String(count) + ",\n";
    json += "\"count\": " + String(count) + ",\n";
    json += "\"sniffer_frames\": [\n";
    json += "\"frames\": [\n";
    
    CANFrameStruct frames[50];
    int n = 50;
    int actual = can->getSnifferBuffer(frames, n);
    
    for (int i = 0; i < actual; i++) {
      if (i > 0) json += ",\n";
      json += "  {\n";
      json += "    \"id\": \"0x" + String(frames[i].id, HEX) + "\",\n";
      json += "    \"len\": " + String(frames[i].len) + ",\n";
      json += "    \"data\": \"";
      for (int b = 0; b < frames[i].len; b++) {
        if (b > 0) json += " ";
        json += String(frames[i].data[b], HEX);
      }
      json += "\",\n";
      json += "    \"time\": " + String(frames[i].timestamp_ms) + "\n";
      json += "  }";
    }
    
    json += "\n]\n}";
    server->send(200, "application/json", json);
  }

  void handleLearn() {
    if (!can || !can->isRunning()) {
      server->send(400, "application/json", "{\"error\":\"CAN not running\"}");
      return;
    }
    String action = server->arg("action");
    if (action == "start") {
      int t = server->arg("time").toInt();
      if (t <= 0) t = 10;
      can->startLearning(t);
      server->send(200, "application/json", "{\"status\":\"learning\",\"time\":" + String(t) + "}");
    } else if (action == "stop") {
      can->stopLearning();
      CANLearner* l = can->getLearner();
      if (l) server->send(200, "application/json", l->toJSON());
      else server->send(200, "application/json", "{\"signals\":0}");
    } else {
      server->send(400, "text/plain", "Unknown action");
    }
  }

  void handleSelect() {
    int idx = server->arg("index").toInt();
    String msg = "OK";
    if (idx >= 0 && idx < VEHICLE_COUNT && can) {
      can->setActiveProfile(&vehicle_profiles[idx]);
      msg = String(vehicle_profiles[idx].make) + " " + String(vehicle_profiles[idx].model);
    } else if (idx == -1 && can) {
      const VehicleProfile* v = can->autoDetectVehicle();
      msg = v ? String(v->make) + " " + String(v->model) : "no match";
    }
    server->send(200, "text/plain", msg);
  }

  void handleSend() {
    if (!can || !can->isRunning()) { server->send(400, "text/plain", "CAN not running"); return; }
    String idStr = server->arg("id");
    String dataStr = server->arg("data");
    if (idStr.length() == 0 || dataStr.length() == 0) { server->send(400, "text/plain", "Missing id or data"); return; }
    
    uint32_t id = strtol(idStr.c_str(), NULL, 16);
    uint8_t data[8];
    uint8_t len = 0;
    char buf[64];
    strncpy(buf, dataStr.c_str(), sizeof(buf) - 1);
    char* token = strtok(buf, " ");
    while (token != NULL && len < 8) {
      data[len++] = strtol(token, NULL, 16);
      token = strtok(NULL, " ");
    }
    if (can->sendFrame(id, data, len)) {
      server->send(200, "text/plain", "Sent " + String(len) + " bytes");
    } else {
      server->send(500, "text/plain", "Send failed");
    }
  }

  void handleUDS() {
    if (!uds) { server->send(400, "application/json", "{\"error\":\"UDS not initialized\"}"); return; }
    String action = server->arg("action");
    
    if (action == "session") {
      String type = server->arg("type");
      uint8_t session = (type == "programming") ? 0x02 : 0x03;
      bool ok = uds->changeSession(session);
      server->send(200, "application/json", "{\"ok\":" + String(ok ? "true" : "false") + ",\"session\":\"" + type + "\"}");
    } else if (action == "security") {
      bool ok = uds->securityAccessPSA();
      server->send(200, "application/json", "{\"ok\":" + String(ok ? "true" : "false") + ",\"unlocked\":" + String(ok ? "true" : "false") + "}");
    } else if (action == "vin") {
      char vin[18];
      if (uds->readVIN(vin, sizeof(vin))) server->send(200, "application/json", "{\"vin\":\"" + String(vin) + "\"}");
      else server->send(200, "application/json", "{\"vin\":\"FAILED\"}");
    } else if (action == "reset") {
      bool ok = uds->resetECU();
      server->send(200, "application/json", "{\"ok\":" + String(ok ? "true" : "false") + "}");
    } else {
      server->send(400, "application/json", "{\"error\":\"Unknown action\"}");
    }
  }

  void handleSpeedDetect() {
    if (!can || !can->isRunning()) {
      server->send(200, "application/json", "{\"ok\":false,\"speed\":0}");
      return;
    }
    bool ok = can->autoDetectSpeed();
    server->send(200, "application/json", "{\"ok\":" + String(ok ? "true" : "false") + ",\"speed\":" + String(can->getSpeed() / 1000) + "}");
  }

  void handleStartCAN() {
    if (can && !can->isRunning()) { can->begin(500000); server->send(200, "text/plain", "CAN started"); }
    else if (can) server->send(200, "text/plain", "CAN already running");
    else server->send(400, "text/plain", "CAN not available");
  }

  void handleStopCAN() {
    if (can && can->isRunning()) { can->end(); server->send(200, "text/plain", "CAN stopped"); }
    else server->send(200, "text/plain", "CAN already stopped");
  }

  void handleOBD2() {
    if (!obd2) { server->send(200, "application/json", "{\"obd2_ready\":false}"); return; }
    obd2->pollAll();
    OBD2Value* vals;
    int count = 0;
    vals = obd2->getValues(&count);
    
    String json = "{\n";
    json += "\"obd2_ready\": true,\n";
    json += "\"obd2_vin\": \"" + String(obd2->getVIN()) + "\",\n";
    json += "\"obd2_values\": [\n";
    for (int i = 0; i < count; i++) {
      if (i > 0) json += ",\n";
      json += "  {\"name\":\"" + String(vals[i].name) + "\",\"value\":" + String(vals[i].value, 1) + ",\"unit\":\"" + String(vals[i].unit) + "\"}";
    }
    json += "\n]\n}";
    server->send(200, "application/json", json);
  }

  void handleOBD2DTC() {
    if (!obd2) { server->send(200, "application/json", "{\"count\":0}"); return; }
    int count = obd2->readDTCs();
    server->send(200, "application/json", "{\"count\":" + String(count) + ",\"ok\":true}");
  }

  void handleOBD2Clear() {
    if (!obd2) { server->send(400, "text/plain", "OBD2 not ready"); return; }
    bool ok = obd2->clearDTCs();
    server->send(200, "application/json", "{\"ok\":" + String(ok ? "true" : "false") + "}");
  }

  void handleSD() {
    if (!sd) { server->send(200, "application/json", "{\"card\":false}"); return; }
    server->send(200, "application/json", sd->toJSON());
  }

  void handleSDStart() {
    if (!sd) { server->send(400, "text/plain", "SD not initialized"); return; }
    bool ok = sd->startSession();
    server->send(200, "application/json", "{\"ok\":" + String(ok ? "true" : "false") + "}");
  }

  void handleSDStop() {
    if (!sd) { server->send(400, "text/plain", "SD not initialized"); return; }
    sd->closeSession();
    server->send(200, "application/json", "{\"ok\":true}");
  }

  void handleBLE() {
    if (!ble) { server->send(200, "application/json", "{\"initialized\":false}"); return; }
    String cmd = server->arg("cmd");
    if (cmd.length() > 0) {
      ble->send("{\"cmd\":\"" + cmd + "\"}");
    }
    server->send(200, "application/json", ble->toJSON());
  }

  void handleRFStatus() {
    if (!rf) { server->send(200, "application/json", "{\"present\":false}"); return; }
    server->send(200, "application/json", rf->toJSON());
  }

  void handleRFCapture() {
    if (!rf || !rf->isCC1101Present()) {
      server->send(200, "application/json", "{\"ok\":false,\"error\":\"RF not available\"}");
      return;
    }
    RFFrame frame;
    bool ok = rf->capture(&frame, 3000);
    String json = "{\"ok\":" + String(ok ? "true" : "false") + ",\"protocol\":" + String(frame.protocol) + ",\"len\":" + String(frame.data_len) + "}";
    server->send(200, "application/json", json);
  }

  void handleRFTransmit() {
    if (!rf || !rf->isCC1101Present()) {
      server->send(200, "application/json", "{\"ok\":false,\"error\":\"RF not available\"}");
      return;
    }
    RFFrame frames[10];
    int count = rf->getCapturedFrames(frames, 10);
    if (count == 0) {
      server->send(200, "application/json", "{\"ok\":false,\"error\":\"No captured frames\"}");
      return;
    }
    bool ok = rf->transmit(&frames[0], 3);
    server->send(200, "application/json", "{\"ok\":" + String(ok ? "true" : "false") + "}");
  }

  void handleRFRollJam() {
    if (!rf || !rf->isCC1101Present()) {
      server->send(200, "application/json", "{\"ok\":false,\"error\":\"RF not available\"}");
      return;
    }
    bool ok = rf->rollJam(433920000, 3000);
    server->send(200, "application/json", "{\"ok\":" + String(ok ? "true" : "false") + "}");
  }

  void handleRFSniff() {
    if (!rf || !rf->isCC1101Present()) {
      server->send(200, "application/json", "{\"count\":0}");
      return;
    }
    int time = server->arg("time").toInt();
    if (time <= 0) time = 5;
    int count = rf->sniff(time * 1000);
    server->send(200, "application/json", "{\"count\":" + String(count) + "}");
  }

  void handleWebSocket(uint8_t num, char* payload) {
    String cmd = String(payload);
    if (cmd == "ping") ws->sendTXT(num, "pong");
  }

  void broadcastStatus() {
    if (!ws) return;
    
    String json = "{";
    
    // CAN
    if (can) {
      json += "\"status\":\"" + String(can->isRunning() ? "running" : "stopped") + "\",";
      json += "\"speed\":" + String(can->getSpeed() / 1000) + ",";
      json += "\"tx\":" + String(can->getTxCount()) + ",";
      json += "\"rx\":" + String(can->getRxCount()) + ",";
      const VehicleProfile* vp = can->getActiveProfile();
      json += "\"vehicle\":\"" + String(vp ? vp->model : "none") + "\",";
    }
    
    // UDS
    if (uds) {
      json += "\"uds_unlocked\":" + String(uds->isSecurityUnlocked() ? "true" : "false") + ",";
    }
    
    // BLE
    if (ble) {
      json += "\"ble_connected\":" + String(ble->isConnected() ? "true" : "false") + ",";
      json += "\"ble_device\":\"" + String(BLE_DEVICE_NAME) + "\",";
    }
    
    // SD
    if (sd) {
      json += "\"sd_card\":" + String(sd->isCardPresent() ? "true" : "false") + ",";
      json += "\"sd_logging\":" + String(sd->isLogging() ? "true" : "false") + ",";
      json += "\"sd_frames\":" + String(sd->getFramesLogged()) + ",";
      json += "\"sd_session\":" + String(sd->getSessionID()) + ",";
      json += "\"sd_file\":\"" + sd->getLogFilename() + "\",";
    }
    
    // RF
    if (rf) {
      json += "\"rf_present\":" + String(rf->isCC1101Present() ? "true" : "false") + ",";
      json += "\"rf_mode\":" + String(rf->getMode()) + ",";
      json += "\"rf_freq\":" + String(rf->getFrequency(), 2) + ",";
      json += "\"rf_captured\":" + String(rf->toJSON().substring(rf->toJSON().indexOf("captured") + 9, rf->toJSON().indexOf("captured") + 11).toInt()) + ",";
    }
    
    // Learner
    if (can) {
      CANLearner* l = can->getLearner();
      if (l) {
        json += "\"learn_status\":\"" + String(l->isLearning() ? "⏳ Learning..." : "✅ Done") + "\",";
        json += "\"learn_signals\":\"" + String(l->getSignalCount()) + " signals found\"";
      }
    }
    
    json += "}";
    ws->broadcastTXT(json);
  }
};

#endif
