#!/usr/bin/env python3
"""
operator_web_ui.py
─────────────────────────────────────────────────────────────────────────────
Minimal Flask web app that serves an operator control panel.
Buttons publish to /operator_response and /task_request_raw.

Run:
    python3 src/cognitive_amr/cognitive_amr/operator_web_ui.py

Then forward port 8080 via SSH:
    ssh -L 8080:localhost:8080 ubuntu@<aws-ip>
Open http://localhost:8080 in your browser.

This runs as a STANDALONE script (not a ROS 2 node) — it acts as a bridge
between the browser buttons and the ROS 2 network using ros2 topic pub CLI.
"""

import subprocess
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import parse_qs, urlparse
import html
import json

PORT = 8080

HTML_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>AMR Operator Panel</title>
<style>
  body {{ font-family: 'Segoe UI', sans-serif; background: #1a1a2e; color: #eee;
          display: flex; flex-direction: column; align-items: center;
          padding: 40px 20px; margin: 0; }}
  h1   {{ color: #e94560; margin-bottom: 4px; }}
  h2   {{ color: #0f3460; margin-bottom: 20px; font-weight: 400; }}
  .card {{ background: #16213e; border-radius: 12px; padding: 24px 32px;
           margin: 16px 0; width: 100%; max-width: 540px; }}
  .card h3 {{ margin-top: 0; color: #e94560; }}
  input[type=text] {{ width: 100%; padding: 10px; border-radius: 6px;
                      border: 1px solid #0f3460; background: #0f3460;
                      color: #eee; font-size: 1rem; box-sizing: border-box; }}
  button {{ padding: 12px 24px; border-radius: 8px; border: none;
            font-size: 1rem; cursor: pointer; margin: 6px 4px;
            transition: opacity 0.2s; }}
  button:hover {{ opacity: 0.82; }}
  .btn-send   {{ background: #e94560; color: white; width: 100%; }}
  .btn-rescan {{ background: #0f9b58; color: white; }}
  .btn-newloc {{ background: #0f6fa8; color: white; }}
  .btn-remap  {{ background: #9b390f; color: white; }}
  .btn-skip   {{ background: #555; color: white; }}
  .status {{ background: #0f3460; border-radius: 8px; padding: 12px;
             font-family: monospace; font-size: 0.88rem; white-space: pre-wrap;
             max-height: 120px; overflow-y: auto; margin-top: 10px; }}
  #log {{ background: #0a0a1a; border-radius: 8px; padding: 12px;
          font-family: monospace; font-size: 0.82rem; white-space: pre-wrap;
          max-height: 200px; overflow-y: auto; margin-top: 8px; color: #8effc1; }}
</style>
</head>
<body>
  <h1>Cognitive AMR</h1>
  <h2>Operator Control Panel</h2>

  <!-- Task input -->
  <div class="card">
    <h3>Submit Task</h3>
    <input type="text" id="taskInput"
           placeholder="e.g. Get me the hydraulic valve set"
           onkeydown="if(event.key==='Enter') sendTask()">
    <button class="btn-send" onclick="sendTask()">Send Task →</button>
  </div>

  <!-- Mismatch response -->
  <div class="card">
    <h3>Mismatch Response</h3>
    <p style="color:#aaa;margin:4px 0 12px">
      Published to <code>/operator_response</code>
    </p>
    <button class="btn-rescan" onclick="respond('rescan')">
      🔄 Re-scan Shelf
    </button>
    <button class="btn-remap" onclick="respond('remap')">
      🗺 Full Remap Scan
    </button>
    <button class="btn-skip" onclick="respond('skip')">
      ⏭ Skip This Task
    </button>
    <br>
    <input type="text" id="newLocInput"
           placeholder="New location: SHELF:SLOT (e.g. A3:3)"
           style="margin-top:10px">
    <button class="btn-newloc" onclick="respondNewLoc()">
      📍 Set New Location
    </button>
  </div>

  <!-- Activity log -->
  <div class="card">
    <h3>Activity Log</h3>
    <div id="log">Waiting for actions...</div>
  </div>

<script>
  function log(msg) {{
    const el = document.getElementById('log');
    const ts = new Date().toLocaleTimeString();
    el.textContent = '[' + ts + '] ' + msg + '\\n' + el.textContent;
  }}

  function sendTask() {{
    const text = document.getElementById('taskInput').value.trim();
    if (!text) return;
    fetch('/action', {{
      method: 'POST',
      headers: {{'Content-Type': 'application/x-www-form-urlencoded'}},
      body: 'type=task&value=' + encodeURIComponent(text)
    }}).then(r => r.text()).then(msg => {{
      log('TASK: ' + text);
      document.getElementById('taskInput').value = '';
    }}).catch(e => log('ERROR: ' + e));
  }}

  function respond(choice) {{
    fetch('/action', {{
      method: 'POST',
      headers: {{'Content-Type': 'application/x-www-form-urlencoded'}},
      body: 'type=response&value=' + encodeURIComponent(choice)
    }}).then(() => log('RESPONSE: ' + choice));
  }}

  function respondNewLoc() {{
    const loc = document.getElementById('newLocInput').value.trim();
    if (!loc) return;
    const choice = 'new_location:' + loc.toUpperCase();
    respond(choice);
    document.getElementById('newLocInput').value = '';
  }}
</script>
</body>
</html>
"""


def _ros2_publish(topic: str, msg_type: str, data: str):
    """Publish a single message using ros2 CLI (non-blocking)."""
    cmd = ['ros2', 'topic', 'pub', '--once', topic, msg_type,
           f'{{data: "{data}"}}']
    subprocess.Popen(cmd,
                     stdout=subprocess.DEVNULL,
                     stderr=subprocess.DEVNULL)


class Handler(BaseHTTPRequestHandler):

    def log_message(self, fmt, *args):
        pass   # suppress access log

    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.end_headers()
        self.wfile.write(HTML_PAGE.encode())

    def do_POST(self):
        length = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(length).decode()
        params = parse_qs(body)

        action_type = params.get('type', [''])[0]
        value = params.get('value', [''])[0]

        if action_type == 'task' and value:
            _ros2_publish('/task_request_raw', 'std_msgs/msg/String', value)

        elif action_type == 'response' and value:
            _ros2_publish('/operator_response', 'std_msgs/msg/String', value)

        self.send_response(200)
        self.send_header('Content-Type', 'text/plain')
        self.end_headers()
        self.wfile.write(b'ok')


def main():
    server = HTTPServer(('0.0.0.0', PORT), Handler)
    print(f"Operator panel: http://localhost:{PORT}")
    print("Forward port if on AWS: ssh -L 8080:localhost:8080 ubuntu@<ip>")
    server.serve_forever()


if __name__ == '__main__':
    main()
