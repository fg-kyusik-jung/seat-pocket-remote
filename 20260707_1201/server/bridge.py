"""아두이노 시리얼 <-> 웹소켓 중계 서버 + CSV 로깅 + 대시보드 정적 서빙.

실행:
    python bridge.py --port COM3

브라우저:
    http://localhost:8000/ 접속 (웹소켓은 8765 포트로 자동 연결됨)
"""

import argparse
import csv
import datetime
import http.server
import json
import os
import socketserver
import sys
import threading
import time

import serial
import serial.tools.list_ports
import websockets
import websockets.sync.server

BAUD_RATE = 115200
WS_PORT = 8765
HTTP_PORT = 8000
WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "web")
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")

clients = set()
clients_lock = threading.Lock()

current_log_path = None
log_file = None
log_writer = None
log_lock = threading.Lock()


def guess_serial_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if "Arduino" in (p.description or "") or "CH340" in (p.description or ""):
            return p.device
    return ports[0].device if ports else None


def start_new_log():
    global current_log_path, log_file, log_writer
    os.makedirs(LOG_DIR, exist_ok=True)
    with log_lock:
        if log_file:
            log_file.close()
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        current_log_path = os.path.join(LOG_DIR, f"log_{timestamp}.csv")
        log_file = open(current_log_path, "w", newline="", encoding="utf-8")
        log_writer = csv.writer(log_file)
        log_writer.writerow(["wall_time", "arduino_millis", "sector_raw_values"])


def append_log(wall_time, arduino_millis, values):
    with log_lock:
        if log_writer is None:
            return
        log_writer.writerow([wall_time, arduino_millis, *values])
        log_file.flush()


def broadcast(message):
    with clients_lock:
        dead = set()
        for ws in clients:
            try:
                ws.send(message)
            except Exception:
                dead.add(ws)
        clients -= dead


def serial_reader(ser):
    while True:
        try:
            raw = ser.readline().decode("utf-8", errors="ignore").strip()
        except Exception as exc:
            print(f"[serial] 읽기 오류: {exc}")
            time.sleep(1)
            continue
        if not raw:
            continue

        parts = raw.split(",")
        if parts[0] == "T" and len(parts) >= 3:
            arduino_millis = parts[1]
            values = parts[2:]
            wall_time = datetime.datetime.now().isoformat()
            append_log(wall_time, arduino_millis, values)
            broadcast(json.dumps({
                "type": "telemetry",
                "millis": int(arduino_millis),
                "values": [int(v) for v in values],
            }))
        elif parts[0] == "READY":
            num_sectors = int(parts[1]) if len(parts) > 1 else None
            print(f"[serial] READY, NUM_SECTORS={num_sectors}")
            broadcast(json.dumps({"type": "ready", "num_sectors": num_sectors}))
        else:
            print(f"[serial] 알수없는 라인: {raw}")


def ws_handler(ser):
    def handler(websocket):
        with clients_lock:
            clients.add(websocket)
        try:
            for message in websocket:
                handle_client_command(ser, message)
        except Exception:
            pass
        finally:
            with clients_lock:
                clients.discard(websocket)
    return handler


def handle_client_command(ser, message):
    try:
        cmd = json.loads(message)
    except json.JSONDecodeError:
        print(f"[ws] JSON 파싱 실패: {message}")
        return

    kind = cmd.get("cmd")
    if kind == "valve":
        line = f"VALVE,{int(cmd['idx'])},{int(cmd['state'])}\n"
    elif kind == "pump":
        line = f"PUMP,{int(cmd['speed'])},{int(cmd['dir'])}\n"
    elif kind == "status":
        line = "STATUS\n"
    else:
        print(f"[ws] 알수없는 명령: {cmd}")
        return

    ser.write(line.encode("utf-8"))


class StaticHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=WEB_DIR, **kwargs)

    def do_GET(self):
        if self.path == "/download":
            self.send_download()
            return
        super().do_GET()

    def send_download(self):
        with log_lock:
            path = current_log_path
        if not path or not os.path.exists(path):
            self.send_error(404, "로그 파일 없음")
            return
        with open(path, "rb") as f:
            data = f.read()
        self.send_response(200)
        self.send_header("Content-Type", "text/csv")
        self.send_header("Content-Disposition", f'attachment; filename="{os.path.basename(path)}"')
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, format, *args):
        pass


def run_http_server():
    with socketserver.TCPServer(("0.0.0.0", HTTP_PORT), StaticHandler) as httpd:
        print(f"[http] 대시보드: http://localhost:{HTTP_PORT}/")
        httpd.serve_forever()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", help="시리얼 포트 (예: COM3). 생략 시 자동 탐색")
    args = parser.parse_args()

    port = args.port or guess_serial_port()
    if not port:
        print("시리얼 포트를 찾을 수 없습니다. --port COM3 형태로 지정하세요.")
        sys.exit(1)

    print(f"[serial] 연결: {port} @ {BAUD_RATE}")
    ser = serial.Serial(port, BAUD_RATE, timeout=1)
    time.sleep(2)  # 아두이노 리셋 대기

    start_new_log()
    print(f"[log] 기록 파일: {current_log_path}")

    threading.Thread(target=serial_reader, args=(ser,), daemon=True).start()
    threading.Thread(target=run_http_server, daemon=True).start()

    print(f"[ws] 웹소켓 서버: ws://localhost:{WS_PORT}/")
    with websockets.sync.server.serve(ws_handler(ser), "0.0.0.0", WS_PORT) as ws_server:
        ws_server.serve_forever()


if __name__ == "__main__":
    main()
