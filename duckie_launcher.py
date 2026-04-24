#!/usr/bin/env python3
"""
Duckie Launcher Proxy
Run this on your laptop alongside duckie_control.html.
It listens on localhost:8766 and:
  - Spawns dts devel run / build / stop as subprocesses
  - Streams their stdout to the GUI via /logs
  - Handles CORS so the browser can call it

Usage:
    python duckie_launcher.py
Then open duckie_control.html in your browser.
"""

import subprocess
import threading
import time
import os
import sys
import json
import socket
import platform
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

PORT = 8766
log_buffer = []
log_lock = threading.Lock()
active_proc = None
proc_lock = threading.Lock()

# On Windows, dts lives inside WSL — detect this once at startup.
IS_WINDOWS = platform.system() == "Windows"

def _dts_cmd(args: list) -> list:
    """Return the full command list to run dts, handling WSL on Windows."""
    if IS_WINDOWS:
        return ["wsl"] + args
    return args


def append_log(line: str):
    with log_lock:
        log_buffer.append(line.rstrip())
        if len(log_buffer) > 2000:
            del log_buffer[:500]


def stream_proc(proc):
    try:
        for raw in proc.stdout:
            line = raw.decode("utf-8", errors="replace").rstrip()
            append_log(line)
            print(f"[dts] {line}", flush=True)
    except Exception as e:
        append_log(f"[stream error] {e}")
    finally:
        proc.wait()
        append_log(f"[process exited with code {proc.returncode}]")


def run_dts(args: list, hostname: str):
    global active_proc
    with proc_lock:
        if active_proc and active_proc.poll() is None:
            append_log("[launcher] Killing existing process before starting new one")
            active_proc.terminate()
            time.sleep(0.5)
        cmd = _dts_cmd(args + ["-H", hostname])
        append_log(f"[launcher] Running: {' '.join(cmd)}")
        if IS_WINDOWS:
            append_log("[launcher] Windows detected — routing through WSL")
        try:
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                bufsize=0,
            )
            active_proc = proc
            t = threading.Thread(target=stream_proc, args=(proc,), daemon=True)
            t.start()
            return True, f"Started: {' '.join(cmd)}"
        except FileNotFoundError:
            if IS_WINDOWS:
                msg = f"Command not found: {cmd[0]}. Make sure WSL is installed and 'dts' is on the WSL PATH."
            else:
                msg = f"Command not found: {cmd[0]}. Is dts installed and on PATH?"
            append_log(f"[launcher ERROR] {msg}")
            return False, msg
        except Exception as e:
            msg = str(e)
            append_log(f"[launcher ERROR] {msg}")
            return False, msg


class Handler(BaseHTTPRequestHandler):

    def do_OPTIONS(self):
        self._cors()
        self.send_response(200)
        self.end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        qs = parse_qs(parsed.query)

        if parsed.path == "/logs":
            offset = int(qs.get("offset", ["0"])[0])
            with log_lock:
                lines = log_buffer[offset:]
                new_offset = len(log_buffer)
            self._json({"lines": lines, "offset": new_offset})

        elif parsed.path == "/ping":
            self._json({"ok": True, "pid": os.getpid()})

        elif parsed.path == "/health":
            self._json({
                "ok": True,
                "platform": platform.system(),
                "wsl_mode": IS_WINDOWS,
                "python": sys.version,
                "pid": os.getpid(),
            })

        else:
            self._json({"error": "not found"}, 404)

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(length) if length else b"{}"
        try:
            data = json.loads(body)
        except Exception:
            data = {}

        hostname = data.get("hostname", "nasavpns")
        parsed = urlparse(self.path)

        if parsed.path == "/dts/run":
            ok, msg = run_dts(["dts", "devel", "run", "-f"], hostname)
            self._json({"ok": ok, "message": msg})

        elif parsed.path == "/dts/build":
            ok, msg = run_dts(["dts", "devel", "build", "-f"], hostname)
            self._json({"ok": ok, "message": msg})

        elif parsed.path == "/dts/stop":
            with proc_lock:
                if active_proc and active_proc.poll() is None:
                    active_proc.terminate()
                    append_log("[launcher] Process terminated by user")
                    self._json({"ok": True, "message": "Process terminated"})
                else:
                    self._json({"ok": False, "message": "No active process"})

        else:
            self._json({"error": "not found"}, 404)

    def _cors(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def _json(self, obj, code=200):
        payload = json.dumps(obj).encode()
        self.send_response(code)
        self._cors()
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def log_message(self, fmt, *args):
        pass  # suppress default access log


def resolve_mdns(hostname: str) -> str | None:
    """Try to resolve hostname.local via mDNS."""
    for suffix in [".local", ""]:
        try:
            ip = socket.gethostbyname(f"{hostname}{suffix}")
            return ip
        except socket.gaierror:
            pass
    return None


if __name__ == "__main__":
    append_log("[launcher] Duckie Launcher Proxy started")
    append_log(f"[launcher] Listening on localhost:{PORT}")
    append_log("[launcher] Open duckie_control.html in your browser to connect.")

    # Try to resolve the bot on startup
    default_host = "nasavpns"
    append_log(f"[launcher] Attempting to resolve {default_host}.local ...")
    ip = resolve_mdns(default_host)
    if ip:
        append_log(f"[launcher] Resolved {default_host}.local → {ip}")
    else:
        append_log(f"[launcher] Could not resolve {default_host}.local — ensure bot is on same network")

    server = HTTPServer(("localhost", PORT), Handler)
    print(f"Duckie Launcher Proxy running on http://localhost:{PORT}")
    print("Open duckie_control.html in your browser.")
    print("Ctrl+C to quit.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutdown.")
