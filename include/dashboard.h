#ifndef DASHBOARD_H
#define DASHBOARD_H

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>E-Bike Pusher Dashboard</title>
  <style>
    body { font-family: 'Segoe UI', Arial, sans-serif; background-color: #121212; color: #ffffff; text-align: center; margin: 0; padding: 15px; }
    h2 { color: #00d2ff; margin-bottom: 5px; }
    .grid { display: flex; flex-wrap: wrap; justify-content: center; gap: 15px; margin-top: 20px; }
    .card { background-color: #1e1e1e; border-radius: 10px; padding: 15px; width: 140px; box-shadow: 0 4px 8px rgba(0,0,0,0.5); }
    .value { font-size: 26px; font-weight: bold; margin: 10px 0; color: #00ea8d; }
    .label { font-size: 12px; color: #aaaaaa; text-transform: uppercase; letter-spacing: 1px; }
    .alert { color: #ff3b30 !important; }
    
    /* Settings Section */
    .settings-panel { margin-top: 30px; background-color: #1a1a1a; padding: 20px; border-radius: 10px; display: inline-block; text-align: left; }
    input { width: 80px; padding: 5px; margin-left: 10px; background: #333; color: white; border: 1px solid #555; border-radius: 5px; }
    button { background-color: #00d2ff; color: #000; border: none; padding: 10px 15px; border-radius: 5px; font-weight: bold; cursor: pointer; margin-top: 15px; width: 100%; }
    button.danger { background-color: #ff3b30; color: white; margin-top: 10px;}
  </style>
</head>
<body>
  <h2>Pusher Trailer</h2>
  
  <div class="grid">
    <div class="card"><div class="label">Brake Status</div><div class="value" id="brake_stat">COASTING</div></div>
    <div class="card"><div class="label">Cadence (RPM)</div><div class="value" id="cadence">0</div></div>
    <div class="card"><div class="label">Motor Power (W)</div><div class="value" id="watt">0.00</div></div>
    <div class="card"><div class="label">Probe Analog</div><div class="value" id="probe">0</div></div>
  </div>

  <div class="settings-panel">
    <h3 style="margin-top:0; color:#aaa;">Settings</h3>
    <p>Brake Threshold: <input type="number" id="b_thresh"></p>
    <p>Brake Timeout: <input type="number" id="b_time"></p>
    <button onclick="saveSettings()">Save to EEPROM</button>
    <button class="danger" onclick="startScan()">Scan for BLE Sensor</button>
  </div>

  <script>
    // Fetch live data every 200ms
    setInterval(function() {
      fetch('/api/data').then(res => res.json()).then(data => {
        document.getElementById("cadence").innerText = data.cadence;
        document.getElementById("watt").innerText = data.power.toFixed(1);
        document.getElementById("probe").innerText = data.probe;
        
        let bStat = document.getElementById("brake_stat");
        if(data.isBraking) {
            bStat.innerText = "BRAKING!";
            bStat.classList.add("alert");
        } else {
            bStat.innerText = "COASTING";
            bStat.classList.remove("alert");
        }
        
        // Only update input boxes if they aren't being typed in
        if (document.activeElement !== document.getElementById('b_thresh')) document.getElementById("b_thresh").value = data.thresh;
        if (document.activeElement !== document.getElementById('b_time')) document.getElementById("b_time").value = data.timeout;
      });
    }, 200);

    function saveSettings() {
      let th = document.getElementById("b_thresh").value;
      let ti = document.getElementById("b_time").value;
      fetch(`/api/settings?th=${th}&ti=${ti}`, { method: 'POST' });
      alert("Saved to EEPROM!");
    }

    function startScan() {
      if(confirm("Motor will stop. ESP32 will reboot and scan for a new BLE Cadence Sensor. Continue?")) {
        fetch('/api/scan', { method: 'POST' });
      }
    }
  </script>
</body>
</html>)rawliteral";

#endif