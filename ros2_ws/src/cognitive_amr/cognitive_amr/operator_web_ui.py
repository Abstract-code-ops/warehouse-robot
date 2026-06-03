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
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import parse_qs, urlparse
import json
import traceback

PORT = 8080

HTML_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>AMR Operator Panel</title>
<style>
  :root {
    --bg: #f4f7fb;
    --bg-soft: radial-gradient(circle at 20% 10%, #d9e9ff 0%, transparent 45%),
               radial-gradient(circle at 85% 90%, #ffe2ec 0%, transparent 42%);
    --text: #1f2a37;
    --muted: #5a6778;
    --panel: rgba(255, 255, 255, 0.88);
    --panel-border: rgba(31, 42, 55, 0.10);
    --input-bg: #f8fbff;
    --input-border: #c8d6ea;
    --shadow: 0 14px 40px rgba(32, 48, 70, 0.12);

    --send: #da3d66;
    --rescan: #1c9a62;
    --newloc: #2079c9;
    --remap: #c45e1a;
    --skip: #6a768a;
  }

  body[data-theme="dark"] {
    --bg: #0f1524;
    --bg-soft: radial-gradient(circle at 10% 5%, #203252 0%, transparent 45%),
               radial-gradient(circle at 85% 90%, #3a1e35 0%, transparent 42%);
    --text: #e8eef9;
    --muted: #a5b1c2;
    --panel: rgba(18, 26, 41, 0.82);
    --panel-border: rgba(232, 238, 249, 0.12);
    --input-bg: #101b2f;
    --input-border: #314767;
    --shadow: 0 16px 48px rgba(0, 0, 0, 0.38);

    --send: #ef5a82;
    --rescan: #22b874;
    --newloc: #37a0ff;
    --remap: #de7b3d;
    --skip: #8895aa;
  }

  * { box-sizing: border-box; }

  body {
    font-family: "Plus Jakarta Sans", "Segoe UI", sans-serif;
    background: var(--bg-soft), var(--bg);
    color: var(--text);
    display: flex;
    flex-direction: column;
    align-items: center;
    padding: 28px 20px 40px;
    margin: 0;
    min-height: 100vh;
    transition: background 0.28s ease, color 0.28s ease;
  }

  .header {
    width: 100%;
    max-width: 760px;
    display: flex;
    align-items: flex-start;
    justify-content: space-between;
    gap: 14px;
    margin-bottom: 10px;
  }

  h1 {
    margin: 0;
    font-size: clamp(1.45rem, 2.6vw, 2.1rem);
    letter-spacing: 0.4px;
  }

  h2 {
    margin: 2px 0 0;
    color: var(--muted);
    font-weight: 500;
    font-size: 1rem;
  }

  .theme-toggle {
    background: var(--panel);
    color: var(--text);
    border: 1px solid var(--panel-border);
    border-radius: 999px;
    padding: 10px 14px;
    font-size: 0.9rem;
    cursor: pointer;
    backdrop-filter: blur(6px);
  }

  .card {
    background: var(--panel);
    border: 1px solid var(--panel-border);
    backdrop-filter: blur(6px);
    border-radius: 16px;
    padding: 22px;
    margin: 12px 0;
    width: 100%;
    max-width: 760px;
    box-shadow: var(--shadow);
  }

  .card h3 {
    margin: 0 0 12px;
    font-size: 1.06rem;
  }

  p.help {
    color: var(--muted);
    margin: 4px 0 14px;
    font-size: 0.95rem;
  }

  input[type=text] {
    width: 100%;
    padding: 12px 14px;
    border-radius: 10px;
    border: 1px solid var(--input-border);
    background: var(--input-bg);
    color: var(--text);
    font-size: 1rem;
    outline: none;
    transition: border-color 0.2s ease, box-shadow 0.2s ease;
  }

  input[type=text]:focus {
    border-color: #56a2f4;
    box-shadow: 0 0 0 3px rgba(86, 162, 244, 0.23);
  }

  .actions {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(160px, 1fr));
    gap: 10px;
    margin-top: 8px;
  }

  button {
    padding: 11px 14px;
    border-radius: 10px;
    border: none;
    font-size: 0.98rem;
    font-weight: 600;
    cursor: pointer;
    color: white;
    transition: transform 0.12s ease, filter 0.2s ease;
  }

  button:hover { filter: brightness(1.06); }
  button:active { transform: translateY(1px); }

  .btn-send { background: var(--send); width: 100%; margin-top: 10px; }
  .btn-rescan { background: var(--rescan); }
  .btn-newloc { background: var(--newloc); margin-top: 10px; width: 100%; }
  .btn-remap { background: var(--remap); }
  .btn-skip { background: var(--skip); }

  #log {
    background: var(--input-bg);
    border: 1px solid var(--input-border);
    border-radius: 10px;
    padding: 12px;
    font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
    font-size: 0.82rem;
    white-space: pre-wrap;
    max-height: 220px;
    overflow-y: auto;
    margin-top: 8px;
    color: var(--text);
  }

  .toast {
    position: fixed;
    right: 14px;
    bottom: 14px;
    background: var(--panel);
    color: var(--text);
    border: 1px solid var(--panel-border);
    border-radius: 10px;
    padding: 10px 12px;
    box-shadow: var(--shadow);
    font-size: 0.9rem;
    opacity: 0;
    transform: translateY(12px);
    transition: opacity 0.2s ease, transform 0.2s ease;
    pointer-events: none;
  }

  .toast.show {
    opacity: 1;
    transform: translateY(0);
  }

  @media (max-width: 640px) {
    body { padding: 20px 14px 34px; }
    .card { padding: 18px; }
    .header { flex-direction: column; align-items: stretch; }
    .theme-toggle { align-self: flex-end; }
  }
</style>
</head>
<body>
  <div class="header">
    <div>
      <h1>Cognitive AMR</h1>
      <h2>Operator Control Panel</h2>
    </div>
    <button class="theme-toggle" onclick="toggleTheme()" id="themeBtn">Switch Theme</button>
  </div>

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
    <p class="help">
      Published to <code>/operator_response</code>
    </p>
    <div class="actions">
      <button class="btn-rescan" onclick="respond('rescan')">
        Re-scan Shelf
      </button>
      <button class="btn-remap" onclick="respond('remap')">
        Full Remap Scan
      </button>
      <button class="btn-skip" onclick="respond('skip')">
        Skip This Task
      </button>
    </div>
    <input type="text" id="newLocInput"
           placeholder="New location: SHELF:SLOT (e.g. A3:3)"
           style="margin-top:10px">
    <button class="btn-newloc" onclick="respondNewLoc()">
      Set New Location
    </button>
  </div>

  <!-- Activity log -->
  <div class="card">
    <h3>Activity Log</h3>
    <div id="log">Waiting for actions...</div>
  </div>
  <div id="toast" class="toast" aria-live="polite"></div>

<script>
  const STORAGE_KEY = 'operator_ui_theme';

  function applyTheme(theme) {
    document.body.setAttribute('data-theme', theme);
    const btn = document.getElementById('themeBtn');
    if (btn) {
      btn.textContent = theme === 'dark' ? 'Use Light Mode' : 'Use Dark Mode';
    }
  }

  function toggleTheme() {
    const current = document.body.getAttribute('data-theme') === 'dark' ? 'dark' : 'light';
    const next = current === 'dark' ? 'light' : 'dark';
    localStorage.setItem(STORAGE_KEY, next);
    applyTheme(next);
    showToast('Theme set to ' + next + ' mode');
  }

  function initTheme() {
    const saved = localStorage.getItem(STORAGE_KEY);
    if (saved === 'light' || saved === 'dark') {
      applyTheme(saved);
      return;
    }
    const prefersDark = window.matchMedia && window.matchMedia('(prefers-color-scheme: dark)').matches;
    applyTheme(prefersDark ? 'dark' : 'light');
  }

  function showToast(msg) {
    const toast = document.getElementById('toast');
    toast.textContent = msg;
    toast.classList.add('show');
    setTimeout(() => toast.classList.remove('show'), 1800);
  }

  function log(msg) {
    const el = document.getElementById('log');
    const ts = new Date().toLocaleTimeString();
    el.textContent = '[' + ts + '] ' + msg + '\\n' + el.textContent;
  }

  function postAction(type, value) {
    return fetch('/action', {
      method: 'POST',
      headers: {'Content-Type': 'application/x-www-form-urlencoded'},
      body: 'type=' + encodeURIComponent(type) + '&value=' + encodeURIComponent(value)
    }).then(r => {
      if (!r.ok) {
        throw new Error('HTTP ' + r.status);
      }
      return r.text();
    });
  }

  function sendTask() {
    const text = document.getElementById('taskInput').value.trim();
    if (!text) return;
    postAction('task', text).then(() => {
      log('TASK: ' + text);
      document.getElementById('taskInput').value = '';
      showToast('Task sent');
    }).catch(e => {
      log('ERROR: ' + e);
      showToast('Failed to send task');
    });
  }

  function respond(choice) {
    postAction('response', choice)
      .then(() => {
        log('RESPONSE: ' + choice);
        showToast('Response sent');
      })
      .catch(e => {
        log('ERROR: ' + e);
        showToast('Failed to send response');
      });
  }

  function respondNewLoc() {
    const loc = document.getElementById('newLocInput').value.trim();
    if (!loc) return;
    const choice = 'new_location:' + loc.toUpperCase();
    respond(choice);
    document.getElementById('newLocInput').value = '';
  }

  initTheme();
</script>
</body>
</html>
"""


def _ros2_publish(topic: str, msg_type: str, data: str):
    """Publish a single message using ros2 CLI (non-blocking)."""
    payload = '{data: ' + json.dumps(data) + '}'
    cmd = ['ros2', 'topic', 'pub', '--once', topic, msg_type,
      payload]
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
      try:
        parsed = urlparse(self.path)
        if parsed.path != '/action':
          self.send_response(404)
          self.end_headers()
          return

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

      except Exception as e:
        print('Exception in do_POST:', e)
        traceback.print_exc()
        try:
          self.send_response(500)
          self.send_header('Content-Type', 'text/plain')
          self.end_headers()
          self.wfile.write(b'error')
        except Exception:
          pass


def main():
    server = HTTPServer(('0.0.0.0', PORT), Handler)
    print(f"Operator panel: http://localhost:{PORT}")
    print("Forward port if on AWS: ssh -L 8080:localhost:8080 ubuntu@<ip>")
    server.serve_forever()


if __name__ == '__main__':
    main()
