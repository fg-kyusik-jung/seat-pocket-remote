"""4포켓 압력센서 실시간 모니터 (pilot3.ino 시리얼 텔레메트리 파싱).

사용법:
    python tools/pocket_monitor.py [COM포트]  (기본 COM3, 115200 baud)

pilot3.ino의 printDebug() 출력(...,RAW,r0,r1,r2,r3)을 파싱해서
- 포켓별 현재 raw값 / 막대그래프
- 기준압 대비 편차(모드2 postureImbalanced()와 동일 계산) 추세그래프
를 실시간으로 보여준다. "기준 재설정" 버튼으로 현재 값을 기준압으로 다시 잡을 수 있다
(모드2 진입 시 postureBaseline을 새로 찍는 것과 동일한 개념).
"""

import re
import sys
import threading
import time
import tkinter as tk
from collections import deque

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM3"
BAUD = 115200
HISTORY_SECONDS = 30
RAW_MIN, RAW_MAX = 80, 500
POCKET_COLORS = ["#e6194b", "#3cb44b", "#4363d8", "#f58231"]

RAW_LINE_RE = re.compile(r"RAW,(\d+),(\d+),(\d+),(\d+)")


class SerialReader(threading.Thread):
    def __init__(self, port, baud, on_reading):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.on_reading = on_reading
        self.ser = None
        self.stop_flag = False

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.5)
        except serial.SerialException as e:
            self.on_reading(None, error=str(e))
            return
        time.sleep(2.0)  # 리셋 후 부트로더/펌웨어 안정화 대기
        try:
            self.ser.write(b"DEBUGPRINT,1\n")
        except serial.SerialException:
            pass
        while not self.stop_flag:
            try:
                raw = self.ser.readline().decode("utf-8", errors="ignore").strip()
            except serial.SerialException as e:
                self.on_reading(None, error=str(e))
                return
            if not raw:
                continue
            m = RAW_LINE_RE.search(raw)
            if m:
                values = tuple(int(g) for g in m.groups())
                self.on_reading(values)

    def stop(self):
        self.stop_flag = True
        if self.ser:
            try:
                self.ser.close()
            except serial.SerialException:
                pass


class MonitorApp:
    def __init__(self, root):
        self.root = root
        root.title(f"4포켓 압력 모니터 - {PORT}")

        self.baseline = None
        self.threshold = tk.IntVar(value=60)
        self.latest = [0, 0, 0, 0]
        self.history = deque()  # (t, r0, r1, r2, r3)
        self.lock = threading.Lock()

        top = tk.Frame(root)
        top.pack(fill="x", padx=8, pady=6)
        self.status_label = tk.Label(top, text="연결 대기 중...", fg="gray", anchor="w")
        self.status_label.pack(side="left", fill="x", expand=True)

        ctrl = tk.Frame(root)
        ctrl.pack(fill="x", padx=8)
        tk.Button(ctrl, text="기준 재설정", command=self.reset_baseline).pack(side="left")
        tk.Label(ctrl, text="  임계값(raw):").pack(side="left")
        tk.Entry(ctrl, textvariable=self.threshold, width=6).pack(side="left")
        tk.Label(ctrl, text="  (표시용, 실제 펌웨어 POSTURE 값과 별개)").pack(side="left")

        values_frame = tk.Frame(root)
        values_frame.pack(fill="x", padx=8, pady=8)
        self.value_labels = []
        self.bar_canvases = []
        for i in range(4):
            col = tk.Frame(values_frame, relief="groove", borderwidth=1)
            col.pack(side="left", expand=True, fill="both", padx=4)
            tk.Label(col, text=f"포켓 {i+1}", fg=POCKET_COLORS[i], font=("Segoe UI", 11, "bold")).pack()
            lbl = tk.Label(col, text="--", font=("Consolas", 20))
            lbl.pack()
            dev_lbl = tk.Label(col, text="편차 --", font=("Consolas", 10))
            dev_lbl.pack()
            bar = tk.Canvas(col, width=40, height=160, bg="white", highlightthickness=1, highlightbackground="#ccc")
            bar.pack(pady=4)
            self.value_labels.append((lbl, dev_lbl))
            self.bar_canvases.append(bar)

        self.trend_canvas = tk.Canvas(root, width=760, height=260, bg="white", highlightthickness=1, highlightbackground="#ccc")
        self.trend_canvas.pack(padx=8, pady=8)

        legend = tk.Frame(root)
        legend.pack(padx=8, pady=(0, 8), anchor="w")
        for i in range(4):
            tk.Label(legend, text=f"■ 포켓{i+1}", fg=POCKET_COLORS[i]).pack(side="left", padx=6)
        tk.Label(legend, text="   (그래프는 기준압 대비 편차 raw, 회색 밴드=임계값 이내)").pack(side="left")

        self.reader = SerialReader(PORT, BAUD, self.on_reading)
        self.reader.start()

        root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(100, self.tick)

    def on_reading(self, values, error=None):
        if error:
            self.root.after(0, lambda: self.status_label.config(text=f"시리얼 오류: {error}", fg="red"))
            return
        with self.lock:
            self.latest = list(values)
            if self.baseline is None:
                self.baseline = list(values)
            self.history.append((time.time(), *values))
            cutoff = time.time() - HISTORY_SECONDS
            while self.history and self.history[0][0] < cutoff:
                self.history.popleft()

    def reset_baseline(self):
        with self.lock:
            self.baseline = list(self.latest)

    def tick(self):
        with self.lock:
            latest = list(self.latest)
            baseline = list(self.baseline) if self.baseline else None
            history = list(self.history)

        if baseline:
            self.status_label.config(text=f"연결됨 ({PORT}) - 기준압: {baseline}", fg="black")

        thr = self.threshold.get()
        for i in range(4):
            lbl, dev_lbl = self.value_labels[i]
            lbl.config(text=str(latest[i]))
            bar = self.bar_canvases[i]
            bar.delete("all")
            frac = max(0.0, min(1.0, (latest[i] - RAW_MIN) / (RAW_MAX - RAW_MIN)))
            h = int(frac * 150)
            bar.create_rectangle(5, 155 - h, 35, 155, fill=POCKET_COLORS[i], outline="")
            if baseline:
                dev = latest[i] - baseline[i]
                over = abs(dev) > thr
                dev_lbl.config(text=f"편차 {dev:+d}", fg="red" if over else "black")

        self.draw_trend(history, baseline, thr)
        self.root.after(150, self.tick)

    def draw_trend(self, history, baseline, thr):
        c = self.trend_canvas
        c.delete("all")
        w, h = 760, 260
        if not history or not baseline:
            c.create_text(w / 2, h / 2, text="데이터 대기 중...", fill="gray")
            return

        span_raw = 250  # 편차 표시 범위 +-250raw
        def y_of(dev):
            return h / 2 - (dev / span_raw) * (h / 2 - 10)

        band_top = y_of(thr)
        band_bot = y_of(-thr)
        c.create_rectangle(0, band_top, w, band_bot, fill="#eeeeee", outline="")
        c.create_line(0, h / 2, w, h / 2, fill="#bbbbbb")

        t_now = history[-1][0]
        t_min = t_now - HISTORY_SECONDS

        def x_of(t):
            return (t - t_min) / HISTORY_SECONDS * w

        for i in range(4):
            points = []
            for row in history:
                t = row[0]
                dev = row[1 + i] - baseline[i]
                points.append((x_of(t), y_of(dev)))
            if len(points) >= 2:
                flat = [coord for pt in points for coord in pt]
                c.create_line(*flat, fill=POCKET_COLORS[i], width=2)

    def on_close(self):
        self.reader.stop()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = MonitorApp(root)
    root.mainloop()
