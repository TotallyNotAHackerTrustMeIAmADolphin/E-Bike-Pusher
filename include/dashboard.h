// dashboard.h
#ifndef DASHBOARD_H
#define DASHBOARD_H

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>E-Bike Pusher</title>
  <style>
    body { font-family: 'Segoe UI', Arial, sans-serif; background: #121212; color: #fff; text-align: center; margin: 0; padding: 10px; }
    .grid { display: flex; flex-wrap: wrap; justify-content: center; gap: 10px; margin-top: 10px; }
    .card { background: #1e1e1e; border-radius: 8px; padding: 15px; width: 135px; box-shadow: 0 4px 8px rgba(0,0,0,0.5); }
    .val { font-size: 26px; font-weight: bold; margin: 10px 0; color: #00ea8d; }
    .lbl { font-size: 11px; color: #aaa; text-transform: uppercase; letter-spacing: 1px; }
    .alert { color: #ff3b30 !important; }
    
    .panel { background: #1a1a1a; padding: 15px; border-radius: 8px; margin-top: 15px; text-align: left; }
    input { width: 120px; padding: 5px; background: #333; color: white; border: 1px solid #555; border-radius: 4px; float: right; }
    button { background: #00d2ff; color: #000; border: none; padding: 10px; border-radius: 5px; font-weight: bold; cursor: pointer; width: 100%; margin-top: 10px; }
    .danger { background: #ff3b30; color: white; }
    p { margin: 8px 0; clear: both; overflow: hidden; line-height: 24px; }
    
    textarea { width: 100%; height: 120px; background: #000; color: #0f0; border: 1px solid #333; padding: 5px; font-family: monospace; font-size: 11px; resize: none; box-sizing: border-box; }
  </style>
</head>
<body>
  <h2 style="color: #00d2ff; margin-bottom: 0;">Pusher Dashboard</h2>
  
  <div class="grid">
    <div class="card"><div class="lbl">Brake Status</div><div class="val" id="b_stat">COAST</div></div>
    <div class="card"><div class="lbl">Cadence (RPM)</div><div class="val" id="cad">0</div></div>
    <div class="card"><div class="lbl">Power (W)</div><div class="val" id="pwr">0.0</div></div>
    <div class="card"><div class="lbl">Brake Factor</div><div class="val" id="prb">1.00</div></div>
  </div>

  <div class="panel">
    <h3 style="margin-top:0; color:#aaa;">System Log</h3>
    <textarea id="syslog" readonly></textarea>
  </div>

  <div class="panel">
    <h3 style="margin-top:0; color:#aaa;">Control Settings</h3>
    <p>Brake Threshold: <input type="number" id="th"></p>
    <p>Timeout (ms): <input type="number" id="ti"></p>
    <button onclick="saveBrake()">Save Brake Settings</button>
    <hr style="border-color:#333; margin:15px 0;">
    <p>Home SSID: <input type="text" id="ssid"></p>
    <p>Home Pass: <input type="password" id="psk"></p>
    <button onclick="saveWiFi()">Save WiFi & Reboot</button>
    <hr style="border-color:#333; margin:15px 0;">
    <button class="danger" onclick="startScan()">Scan for new BLE Sensor</button>
  </div>

  <script>
    function updateTelemetry() {
      fetch('/api/data').then(res => res.json()).then(d => {
        document.getElementById("cad").innerText = d.cadence;
        document.getElementById("pwr").innerText = d.power.toFixed(1);
        document.getElementById("prb").innerText = d.probe.toFixed(2);
        
        let bs = document.getElementById("b_stat");
        if(d.isBraking) { bs.innerText = "BRAKING"; bs.classList.add("alert"); } 
        else { bs.innerText = "COAST"; bs.classList.remove("alert"); }
        
        if (document.activeElement !== document.getElementById('th')) document.getElementById("th").value = d.thresh;
        if (document.activeElement !== document.getElementById('ti')) document.getElementById("ti").value = d.timeout;
        if (document.activeElement !== document.getElementById('ssid')) document.getElementById("ssid").value = d.ssid;
        if (document.activeElement !== document.getElementById('psk')) document.getElementById("psk").value = d.psk;
        
        setTimeout(updateTelemetry, 1000); // Relaxed to 1 update per second
      }).catch(e => setTimeout(updateTelemetry, 2000));
    }
    
    function updateLog() {
      fetch('/api/log').then(res => res.text()).then(txt => {
        let logbox = document.getElementById("syslog");
        if(logbox.value !== txt) { logbox.value = txt; logbox.scrollTop = logbox.scrollHeight; }
        setTimeout(updateLog, 2000); // Update logs every 2 seconds
      }).catch(e => setTimeout(updateLog, 3000));
    }

    updateTelemetry();
    updateLog();

    function saveBrake() { fetch('/api/settings?th=' + document.getElementById("th").value + '&ti=' + document.getElementById("ti").value, { method: 'POST' }); alert("Saved!"); }
    function saveWiFi() { fetch('/api/wifi?s=' + encodeURIComponent(document.getElementById("ssid").value) + '&p=' + encodeURIComponent(document.getElementById("psk").value), { method: 'POST' }); alert("Saved. ESP32 Rebooting!"); }
    function startScan() { if(confirm("Reboot and scan for a new sensor?")) fetch('/api/scan', { method: 'POST' }); }
  </script>
</body>
</html>)rawliteral";

#endif