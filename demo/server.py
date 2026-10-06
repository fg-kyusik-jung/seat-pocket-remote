"""firmware/pilot3.ino의 UI 상태머신을 그대로 흉내내는 발표용 데모 서버.
실제 하드웨어(밸브/펌프/센서) 없이, 압력값은 단순 물리 흉내로 시뮬레이션.

실행:
    python server.py

브라우저:
    http://localhost:8001/ 접속 (웹소켓은 8766 포트로 자동 연결됨)
"""

import http.server
import json
import os
import socketserver
import sys
import threading
import time
import webbrowser

import websockets.sync.server

WS_PORT = 8766
HTTP_PORT = 8001

# PyInstaller onefile로 패키징되면 리소스(web/)는 sys._MEIPASS 임시폴더에 풀리고,
# exe 자체는 다른 위치에 있으므로 두 경로를 구분해서 잡는다.
if getattr(sys, "frozen", False):
    RESOURCE_DIR = sys._MEIPASS
    APP_DIR = os.path.dirname(sys.executable)
else:
    RESOURCE_DIR = os.path.dirname(os.path.abspath(__file__))
    APP_DIR = RESOURCE_DIR

WEB_DIR = os.path.join(RESOURCE_DIR, "web")
PRESET_FILE = os.path.join(APP_DIR, "presets.json")
TICK_S = 0.05

# ---------- 펌웨어 상수 ----------
CEILING = 500.0
FLOOR = 120.0
TOLERANCE = 5.0
LONG_PRESS_S = 1.0
DOUBLE_CLICK_WINDOW_S = 0.4
LETTER_REVEAL_S = 0.15
DOT_ANIM_INTERVAL_S = 0.4
CAL_COMPLETE_SHOW_S = 2.0
ALERT_DURATION_S = 10.0
BLINK_INTERVAL_S = 0.25
ALERT_BEEP_MAX = 4
SELECT_BLINK_S = 0.5
PRESET_DISPLAY_HOLD_S = 1.5
ADJUST_HOLD_TIMEOUT_S = 0.2
SYSTEM_ON_HOLD_S = 1.2
SYSTEM_OFF_HOLD_S = 1.2

MODE_NAMES = ["AUTO FIT", "REALTIME", "MANUAL"]
LINE0_TEXT = "Analyzing your"
LINE1_TEXT = "body pressure"
ANALYZE_TOTAL_LEN = len(LINE0_TEXT) + len(LINE1_TEXT)

# 4포켓이 실제로는 하중에 따라 다르게 반응하는 걸 흉내내기 위한 고정 배기속도 비율 (매번 같은 결과가 나오도록 고정값)
LOAD_FACTORS = [1.0, 0.82, 1.18, 0.95]
POCKET_GROUP = [0, 0, 1, 1]  # 포켓0,1=모터1 / 포켓2,3=모터2
INFLATE_RATE = 130.0   # raw/sec
SWEEP_BASE_RATE = 110.0  # raw/sec
RETURN_LERP_S = 1.5
ADJUST_RATE = 150.0    # raw/sec, 매뉴얼 홀드 중 변화 속도


def raw_to_level(raw):
    c = max(80.0, min(500.0, raw))
    lvl = int(((c - 80.0) * 10) / 420) + 1
    return max(1, min(10, lvl))


def level_to_raw_target(level):
    level = max(1, min(10, level))
    return 80.0 + (level - 1) * 42 + 21


class Sim:
    def __init__(self):
        self.lock = threading.Lock()
        self.events = []  # 1회성 이벤트(비프/차임) 큐

        self.system_on = False
        self.top_state = "SELECT"
        self.mode_select_idx = 0
        self.current_mode_idx = 0

        self.enc_pressed = False
        self.enc_press_start = 0.0
        self.long_press_fired = False
        self.click_pending = False
        self.click_pending_time = 0.0

        self.system_on_until = 0.0   # SYSTEM ON/HELLO 문구 표시 유지 시각
        self.system_off_until = 0.0  # SYSTEM OFF/GOOD BYE 문구 표시 유지 시각

        self.sensor_raw = [80.0, 80.0, 80.0, 80.0]

        self.cal_state = "IDLE"
        self.active_state = "INFLATE"
        self.cal_state_start = 0.0
        self.active_state_start = 0.0
        self.revealed_chars = 0
        self.last_letter_time = 0.0
        self.last_dot_time = 0.0
        self.dot_count = 1
        self.best_found = False
        self.best_avg = None
        self.best_target = [0.0] * 4

        self.return_targets = [0.0] * 4
        self.return_start = [0.0] * 4

        self.mode2_state = "IDLE"
        self.alert_start = 0.0
        self.last_blink = 0.0
        self.blink_on = False
        self.alert_beep_count = 0

        self.mode3_state = "IDLE"
        self.selected_pocket = 0  # 0~3 포켓, 4 북마크
        self.preset_select_idx = 0
        self.last_blink_sel = 0.0
        self.blink_sel_on = True

        self.last_adjust_cw = -10.0
        self.last_adjust_ccw = -10.0

        self.presets = self._load_presets()

        self.preset_applying = False
        self.preset_applying_idx = 0
        self.preset_display_until = 0.0
        self.preset_apply_start = 0.0
        self.preset_empty_flag = False

        self.leds = [False, False, False]

        # 하드웨어 동작 시각화용 (모터2개/포켓별 2way밸브4개)
        self.motor_running = [False, False]
        self.valve_open = [False, False, False, False]

    # ---------- 프리셋 파일 저장 (EEPROM 흉내) ----------
    def _load_presets(self):
        if os.path.exists(PRESET_FILE):
            try:
                with open(PRESET_FILE, "r", encoding="utf-8") as f:
                    data = json.load(f)
                    if isinstance(data, list) and len(data) == 3:
                        return [list(map(int, row)) for row in data]
            except Exception:
                pass
        return [[0, 0, 0, 0] for _ in range(3)]

    def _save_presets(self):
        try:
            with open(PRESET_FILE, "w", encoding="utf-8") as f:
                json.dump(self.presets, f)
        except Exception:
            pass

    # ---------- 외부 입력 ----------
    def on_press_down(self):
        now = time.time()
        with self.lock:
            self.enc_pressed = True
            self.enc_press_start = now
            self.long_press_fired = False

    def on_press_up(self):
        now = time.time()
        with self.lock:
            self.enc_pressed = False
            if not self.long_press_fired and self.system_on and not self.preset_applying:
                if self.click_pending and (now - self.click_pending_time) <= DOUBLE_CLICK_WINDOW_S:
                    self.click_pending = False
                    self._handle_double_click()
                else:
                    self.click_pending = True
                    self.click_pending_time = now

    def on_rotate(self, direction):
        with self.lock:
            if not self.system_on or self.preset_applying:
                return
            if self.top_state == "SELECT":
                if direction > 0 and self.mode_select_idx < 2:
                    self.mode_select_idx += 1
                elif direction < 0 and self.mode_select_idx > 0:
                    self.mode_select_idx -= 1
                return

            if self.current_mode_idx == 1 and self.mode2_state == "BALANCED" and direction > 0:
                self._enter_mode2_alert()
                return

            if self.current_mode_idx == 2:
                if self.mode3_state == "SELECT":
                    if direction > 0 and self.selected_pocket < 4:
                        self.selected_pocket += 1
                        self.blink_sel_on = True
                        self.last_blink_sel = time.time()
                    elif direction < 0 and self.selected_pocket > 0:
                        self.selected_pocket -= 1
                        self.blink_sel_on = True
                        self.last_blink_sel = time.time()
                elif self.mode3_state == "ADJUST":
                    now = time.time()
                    if direction > 0:
                        self.last_adjust_cw = now
                    else:
                        self.last_adjust_ccw = now
                elif self.mode3_state == "PRESET_SELECT":
                    if direction > 0 and self.preset_select_idx < 2:
                        self.preset_select_idx += 1
                        self.blink_sel_on = True
                        self.last_blink_sel = time.time()
                    elif direction < 0 and self.preset_select_idx > 0:
                        self.preset_select_idx -= 1
                        self.blink_sel_on = True
                        self.last_blink_sel = time.time()

    def on_preset_button(self, idx):
        with self.lock:
            if not self.system_on:
                return
            if self.cal_state == "ANALYZING" or self.preset_applying:
                self.events.append({"name": "log", "text": f"프리셋{idx+1}: 지금은 다른 하드웨어 동작 중"})
                return
            self._activate_preset(idx)

    # ---------- 싱글/더블클릭 동작 ----------
    def _toggle_power(self):
        self.system_on = not self.system_on
        if self.system_on:
            self.system_on_until = time.time() + SYSTEM_ON_HOLD_S
        else:
            self.system_off_until = time.time() + SYSTEM_OFF_HOLD_S
            self.leds = [False, False, False]
            self.cal_state = "IDLE"
            self.mode2_state = "IDLE"
            self.mode3_state = "IDLE"
            self.preset_applying = False
            self.click_pending = False
            self.sensor_raw = [80.0, 80.0, 80.0, 80.0]

    def _enter_mode_select(self):
        self.top_state = "SELECT"
        self.mode_select_idx = self.current_mode_idx
        self.cal_state = "IDLE"
        self.mode2_state = "IDLE"
        self.mode3_state = "IDLE"

    def _handle_single_click(self):
        if self.top_state == "SELECT":
            self.current_mode_idx = self.mode_select_idx
            self.top_state = "IN_MODE"
            if self.current_mode_idx == 0:
                self._start_calibration()
            elif self.current_mode_idx == 1:
                self._enter_mode2_balanced()
            elif self.current_mode_idx == 2:
                self._enter_mode3_select()
            return

        if self.current_mode_idx == 0 and self.cal_state == "IDLE":
            self._start_calibration()
        elif self.current_mode_idx == 1 and self.mode2_state == "IDLE":
            self._enter_mode2_balanced()
        elif self.current_mode_idx == 2:
            if self.mode3_state == "IDLE":
                self._enter_mode3_select()
            elif self.mode3_state == "SELECT":
                if self.selected_pocket == 4:
                    self._enter_preset_select()
                else:
                    self._enter_mode3_adjust()
            elif self.mode3_state == "PRESET_SELECT":
                for i in range(4):
                    self.presets[self.preset_select_idx][i] = raw_to_level(self.sensor_raw[i])
                self._save_presets()
                self.mode3_state = "PRESET_SAVED"

    def _handle_double_click(self):
        if self.top_state == "SELECT":
            return
        if self.current_mode_idx == 0:
            self._enter_mode_select()
        elif self.current_mode_idx == 1:
            self._enter_mode_select()
        elif self.current_mode_idx == 2:
            if self.mode3_state == "SELECT":
                self._enter_mode_select()
            else:
                self.mode3_state = "SELECT"
                self.blink_sel_on = True
                self.last_blink_sel = time.time()

    # ---------- 모드1: 능동형 ----------
    def _start_calibration(self):
        self.cal_state = "ANALYZING"
        self.cal_state_start = time.time()
        self.revealed_chars = 0
        self.last_letter_time = 0.0
        self.last_dot_time = 0.0
        self.dot_count = 1
        self._enter_inflate()

    def _enter_inflate(self):
        self.active_state = "INFLATE"
        self.active_state_start = time.time()
        self.best_found = False
        self.best_avg = None
        self.sensor_raw = [80.0, 80.0, 80.0, 80.0]

    def _tick_active_mode(self, dt):
        if self.active_state == "INFLATE":
            done = True
            group_active = [False, False]
            for i in range(4):
                if self.sensor_raw[i] < CEILING:
                    self.sensor_raw[i] = min(CEILING, self.sensor_raw[i] + INFLATE_RATE * dt)
                    done = False
                    self.valve_open[i] = True
                    group_active[POCKET_GROUP[i]] = True
                else:
                    self.valve_open[i] = False
            self.motor_running = group_active
            if done:
                self.active_state = "SWEEP"
                self.active_state_start = time.time()
                self.valve_open = [False] * 4
                self.motor_running = [False, False]
        elif self.active_state == "SWEEP":
            for i in range(4):
                self.sensor_raw[i] = max(FLOOR, self.sensor_raw[i] - SWEEP_BASE_RATE * LOAD_FACTORS[i] * dt)
            self.valve_open = [True, True, True, True]  # 배기 중 - 펌프는 정지, 밸브만 열림
            self.motor_running = [False, False]
            ref_low = min(self.sensor_raw)
            in_tol = all(v <= ref_low + TOLERANCE for v in self.sensor_raw)
            avg = sum(self.sensor_raw) / 4
            if in_tol and (not self.best_found or avg < self.best_avg):
                self.best_found = True
                self.best_avg = avg
                self.best_target = list(self.sensor_raw)
            if ref_low <= FLOOR:
                if not self.best_found:
                    self.best_found = True
                    self.best_target = list(self.sensor_raw)
                self.active_state = "RETURN"
                self.active_state_start = time.time()
                self.return_targets = list(self.best_target)
                self.return_start = list(self.sensor_raw)
        elif self.active_state == "RETURN":
            elapsed = time.time() - self.active_state_start
            t = min(1.0, elapsed / RETURN_LERP_S)
            group_active = [False, False]
            for i in range(4):
                self.sensor_raw[i] = self.return_start[i] + (self.return_targets[i] - self.return_start[i]) * t
                diff = self.return_targets[i] - self.return_start[i]
                self.valve_open[i] = abs(diff) > 1.0 and t < 1.0
                if diff > 1.0 and t < 1.0:
                    group_active[POCKET_GROUP[i]] = True
            self.motor_running = group_active
            if t >= 1.0:
                self.active_state = "LOCKED"
                self.valve_open = [False] * 4
                self.motor_running = [False, False]

    def _tick_calibration(self, now, dt):
        if self.cal_state != "ANALYZING":
            return
        self._tick_active_mode(dt)

        if self.revealed_chars < ANALYZE_TOTAL_LEN:
            if now - self.last_letter_time >= LETTER_REVEAL_S:
                self.last_letter_time = now
                self.revealed_chars += 1
        elif now - self.last_dot_time >= DOT_ANIM_INTERVAL_S:
            self.last_dot_time = now
            self.dot_count += 1
            if self.dot_count > 3:
                self.dot_count = 1

        if self.active_state == "LOCKED":
            self.cal_state = "COMPLETE_SHOW"
            self.cal_state_start = now
            self.events.append({"name": "chime"})
        return

    # ---------- 모드2: 실시간 ----------
    def _enter_mode2_balanced(self):
        self.mode2_state = "BALANCED"

    def _enter_mode2_alert(self):
        self.mode2_state = "ALERT"
        self.alert_start = time.time()
        self.last_blink = 0.0
        self.blink_on = False
        self.alert_beep_count = 0

    def _tick_mode2(self, now):
        if self.mode2_state != "ALERT":
            return
        if now - self.last_blink >= BLINK_INTERVAL_S:
            self.last_blink = now
            self.blink_on = not self.blink_on
            if self.blink_on and self.alert_beep_count < ALERT_BEEP_MAX:
                self.events.append({"name": "beep", "freq": 440, "ms": 150})
                self.alert_beep_count += 1
        if now - self.alert_start >= ALERT_DURATION_S:
            self.events.append({"name": "chime"})
            self._enter_mode2_balanced()

    # ---------- 모드3: 매뉴얼 ----------
    def _enter_mode3_select(self):
        self.mode3_state = "SELECT"
        self.selected_pocket = 0
        self.blink_sel_on = True
        self.last_blink_sel = time.time()

    def _enter_mode3_adjust(self):
        self.mode3_state = "ADJUST"
        self.last_adjust_cw = -10.0
        self.last_adjust_ccw = -10.0

    def _enter_preset_select(self):
        self.mode3_state = "PRESET_SELECT"
        self.preset_select_idx = 0
        self.blink_sel_on = True
        self.last_blink_sel = time.time()

    def _tick_mode3(self, now, dt):
        if self.mode3_state in ("SELECT", "PRESET_SELECT"):
            if now - self.last_blink_sel >= SELECT_BLINK_S:
                self.last_blink_sel = now
                self.blink_sel_on = not self.blink_sel_on
        elif self.mode3_state == "ADJUST":
            holding_cw = (now - self.last_adjust_cw) < ADJUST_HOLD_TIMEOUT_S
            holding_ccw = (now - self.last_adjust_ccw) < ADJUST_HOLD_TIMEOUT_S
            i = self.selected_pocket
            if holding_cw and self.sensor_raw[i] < CEILING:
                self.sensor_raw[i] = min(CEILING, self.sensor_raw[i] + ADJUST_RATE * dt)
                self.valve_open[i] = True
                self.motor_running[POCKET_GROUP[i]] = True
            elif holding_ccw and self.sensor_raw[i] > FLOOR:
                self.sensor_raw[i] = max(FLOOR, self.sensor_raw[i] - ADJUST_RATE * dt)
                self.valve_open[i] = True

    # ---------- 프리셋 적용 ----------
    def _activate_preset(self, idx):
        self.leds = [i == idx for i in range(3)]
        any_saved = any(v != 0 for v in self.presets[idx])
        if not any_saved:
            self.preset_applying = False
            self.preset_display_until = time.time() + PRESET_DISPLAY_HOLD_S
            self.preset_applying_idx = idx
            self.preset_empty_flag = True
            return
        self.preset_empty_flag = False
        self.preset_applying = True
        self.preset_applying_idx = idx
        self.return_targets = [level_to_raw_target(v) for v in self.presets[idx]]
        self.return_start = list(self.sensor_raw)
        self.preset_apply_start = time.time()
        self.click_pending = False

    def _tick_preset_apply(self, now, dt):
        if not self.preset_applying:
            return
        elapsed = now - self.preset_apply_start
        t = min(1.0, elapsed / RETURN_LERP_S)
        group_active = [False, False]
        for i in range(4):
            self.sensor_raw[i] = self.return_start[i] + (self.return_targets[i] - self.return_start[i]) * t
            diff = self.return_targets[i] - self.return_start[i]
            self.valve_open[i] = abs(diff) > 1.0 and t < 1.0
            if diff > 1.0 and t < 1.0:
                group_active[POCKET_GROUP[i]] = True
        self.motor_running = group_active
        if t >= 1.0:
            self.preset_applying = False
            self.preset_display_until = now + PRESET_DISPLAY_HOLD_S

    # ---------- 메인 틱 ----------
    def tick(self):
        now = time.time()
        dt = TICK_S
        with self.lock:
            if self.enc_pressed and not self.long_press_fired and (now - self.enc_press_start) >= LONG_PRESS_S:
                self.long_press_fired = True
                self._toggle_power()

            if self.click_pending and not self.preset_applying and (now - self.click_pending_time) > DOUBLE_CLICK_WINDOW_S:
                self.click_pending = False
                self._handle_single_click()

            if not self.system_on:
                return

            self.motor_running = [False, False]
            self.valve_open = [False, False, False, False]

            if self.preset_applying:
                self._tick_preset_apply(now, dt)
            elif now < self.preset_display_until:
                pass
            elif self.top_state == "SELECT":
                pass
            else:
                if self.cal_state == "COMPLETE_SHOW":
                    if now - self.cal_state_start >= CAL_COMPLETE_SHOW_S:
                        self.cal_state = "LOCKED"
                else:
                    self._tick_calibration(now, dt)
                self._tick_mode2(now)
                self._tick_mode3(now, dt)

    # ---------- 화면 렌더 ----------
    def _lcd_line(self, s):
        return (s + " " * 16)[:16]

    def snapshot(self):
        with self.lock:
            line0, line1 = self._render()
            return {
                "type": "state",
                "power": self.system_on,
                "lcd0": self._lcd_line(line0),
                "lcd1": self._lcd_line(line1),
                "backlight": self._backlight(),
                "leds": list(self.leds),
                "levels": [raw_to_level(v) for v in self.sensor_raw],
                "motors": list(self.motor_running),
                "valves": list(self.valve_open),
            }

    def _backlight(self):
        if not self.system_on:
            return time.time() < self.system_off_until
        if self.current_mode_idx == 1 and self.mode2_state == "ALERT":
            return self.blink_on
        return True

    def _render(self):
        if not self.system_on:
            if time.time() < self.system_off_until:
                return "SYSTEM OFF", "GOOD BYE :)"
            return "", ""
        if time.time() < self.system_on_until:
            return "SYSTEM ON", "HELLO :)"
        if self.preset_applying or time.time() < self.preset_display_until:
            idx = self.preset_applying_idx
            if self.preset_empty_flag and not self.preset_applying:
                return f"PRESET #{idx+1}", "Empty"
            if self.preset_applying:
                return f"PRESET #{idx+1}", "Applying..."
            return f"PRESET #{idx+1}", " ".join(str(v) for v in self.presets[idx])

        if self.top_state == "SELECT":
            idx = self.mode_select_idx
            if idx == 2:
                return self._mode3_default_text()
            return f"MODE {idx+1}", MODE_NAMES[idx]

        if self.current_mode_idx == 0:
            return self._mode1_text()
        elif self.current_mode_idx == 1:
            return self._mode2_text()
        else:
            return self._mode3_text()

    def _mode3_default_text(self):
        vals = [raw_to_level(v) for v in self.sensor_raw]
        return "MODE 3 - MANUAL", " ".join(str(v) for v in vals)

    def _mode1_text(self):
        if self.cal_state == "IDLE":
            return "MODE 1", MODE_NAMES[0]
        if self.cal_state == "ANALYZING":
            combined = (LINE0_TEXT + LINE1_TEXT)[: self.revealed_chars]
            line0 = combined[: len(LINE0_TEXT)]
            line1 = combined[len(LINE0_TEXT):]
            if self.revealed_chars >= ANALYZE_TOTAL_LEN:
                line1 = LINE1_TEXT + "." * self.dot_count
            return line0, line1
        if self.cal_state == "COMPLETE_SHOW":
            return "Analyze Complete", "!"
        return "Maintaining", "Optimal Pressure"

    def _mode2_text(self):
        if self.mode2_state == "IDLE":
            return "MODE 2", MODE_NAMES[1]
        if self.mode2_state == "ALERT":
            return "Sit up straight", "!!!"
        return "Well balanced :)", ""

    def _mode3_text(self):
        if self.mode3_state == "IDLE":
            return self._mode3_default_text()
        if self.mode3_state == "SELECT":
            cells = []
            for i in range(4):
                hide = (i == self.selected_pocket) and not self.blink_sel_on
                cells.append(" " if hide else str(i + 1))
            hide_bm = (self.selected_pocket == 4) and not self.blink_sel_on
            line1 = "  ".join(cells) + "  " + (" " if hide_bm else "\U0001F516")
            return "select section", line1
        if self.mode3_state == "ADJUST":
            return f"#{self.selected_pocket+1} POCKET Adj.", f"Level {raw_to_level(self.sensor_raw[self.selected_pocket])}"
        if self.mode3_state == "PRESET_SELECT":
            cells = []
            for i in range(3):
                hide = (i == self.preset_select_idx) and not self.blink_sel_on
                cells.append(" " if hide else str(i + 1))
            return "PRESET", " ".join(cells)
        return f"PRESET #{self.preset_select_idx+1}", "Saved! :)"

    def pop_events(self):
        with self.lock:
            ev = self.events
            self.events = []
            return ev


sim = Sim()
clients = set()
clients_lock = threading.Lock()


def broadcast(message):
    with clients_lock:
        dead = set()
        for ws in clients:
            try:
                ws.send(message)
            except Exception:
                dead.add(ws)
        clients.difference_update(dead)


def tick_loop():
    while True:
        sim.tick()
        broadcast(json.dumps(sim.snapshot()))
        for ev in sim.pop_events():
            broadcast(json.dumps({"type": "event", **ev}))
        time.sleep(TICK_S)


def ws_handler(websocket):
    with clients_lock:
        clients.add(websocket)
    try:
        websocket.send(json.dumps(sim.snapshot()))
        for message in websocket:
            try:
                cmd = json.loads(message)
            except json.JSONDecodeError:
                continue
            kind = cmd.get("cmd")
            if kind == "press_down":
                sim.on_press_down()
            elif kind == "press_up":
                sim.on_press_up()
            elif kind == "rotate":
                sim.on_rotate(int(cmd.get("dir", 0)))
            elif kind == "preset":
                sim.on_preset_button(int(cmd.get("idx", 0)))
    except Exception:
        pass
    finally:
        with clients_lock:
            clients.discard(websocket)


class StaticHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=WEB_DIR, **kwargs)

    def log_message(self, format, *args):
        pass


def run_http_server():
    with socketserver.TCPServer(("0.0.0.0", HTTP_PORT), StaticHandler) as httpd:
        print(f"[http] 데모 페이지: http://localhost:{HTTP_PORT}/")
        httpd.serve_forever()


def main():
    threading.Thread(target=run_http_server, daemon=True).start()
    threading.Thread(target=tick_loop, daemon=True).start()

    threading.Timer(0.5, lambda: webbrowser.open(f"http://localhost:{HTTP_PORT}/")).start()

    print(f"[ws] 웹소켓 서버: ws://localhost:{WS_PORT}/")
    print("창을 닫으려면 이 창에서 Ctrl+C를 누르세요.")
    with websockets.sync.server.serve(ws_handler, "0.0.0.0", WS_PORT) as ws_server:
        ws_server.serve_forever()


if __name__ == "__main__":
    main()
