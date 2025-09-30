#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LapTimer + MiniMap + Analog Speedometer + IMU-based G-Meter (800x480, pygame 1.9.x)
+ ZED-F9P GNSS 10 Hz 자동 설정(UBX, pyubx2)

- Lap: A→B, endpoint inset(2m), min 60s, cooldown 4s
- Map: first full lap recorded as base, live dot after
- Speedo: 0~120 km/h, analog gauge (0 at ~8 o’clock), red numbers
- G-Meter: ±1.5 g, 0.5 g rings, peak red dot below current dot layer
- Lap box: 'Last/Best/Run' with line-break style
- CSV: single file /home/pi/DJY/Laptime_data/Laptime_YYYY_MM_DD_HH_MM.csv
       type=LAP|SAMPLE rows in one file
- GNSS config: measRate=100ms(10Hz), USB에서 RMC/VTG만 ON, 저장
"""

from __future__ import annotations
import os, sys, math, time, csv, serial, datetime, collections, threading, re
from dataclasses import dataclass

# -------- optional pyubx2 for GNSS auto-config --------
try:
    from pyubx2 import UBXMessage, SET
    HAS_UBX = True
except Exception:
    HAS_UBX = False

try:
    import pygame
except Exception:
    pygame = None

# ===================== User Configs =====================
# --- GNSS ---
PORT_GNSS = "/dev/ttyGNSS"
BAUD_GNSS = 230400
AUTO_CONFIG_GNSS = True   # 시작 시 ZED-F9P를 10Hz로 설정 (pyubx2 필요)

# --- IMU (G-meter) ---
IMU_PORT = "/dev/ttyIMU"
IMU_BAUD = 115200
SER_TIMEOUT = 0.02
ACC_UNIT = "g"               # 센서가 m/s^2면 "mps2"
ACC_TRIPLE_INDEX = -3        # (ax, ay, az) 시작 인덱스(끝에서 3개)
LINE_MUST_START = "*"
G_SI = 9.80665

# IMU → 차량축 매핑 (X=횡, Y=종)
MAP_X_FROM, MAP_Y_FROM = "x", "y"
X_SIGN_INIT, Y_SIGN_INIT = +1, +1

# --- Lap finish line ---
POINT_A = (35.142685, 126.928691)  # lat, lon
POINT_B = (35.142623, 126.928930)
LAP_DIRECTION = 1                  # 1=A->B, -1=B->A, 0=both
LINE_ENDPOINT_INSET_M = 2.0
CROSS_COOLDOWN_S = 4.0
MIN_LAP_TIME_S = 60.0

# --- HUD / window ---
WIN_W, WIN_H = 800, 480
PAD_FRACTION = 0.08

# layout
MAP_RECT = (20, 20, 280, 390) 
G_SIZE     = 160
GM_POS     = (WIN_W - 20 - G_SIZE, 220)
GM_SIZE    = (G_SIZE, G_SIZE)

# lap timer
MID_X0 = MAP_RECT[0] + MAP_RECT[2] + 10
MID_X1 = GM_POS[0] - 10
LAP_RECT = (MID_X0, 20, max(140, MID_X1 - MID_X0), 170)

# speed meter
SPD_X = MAP_RECT[0] + MAP_RECT[2] + 5
SPD_Y = 200
SPD_W = (GM_POS[0] - 10) - SPD_X
SPD_H = 300
SPD_RECT = (SPD_X, SPD_Y, max(280, SPD_W), SPD_H)

# font
FONT_LAP_LABEL = 22
FONT_LAP_TIME  = 32
FONT_SPEED_NUM = 22

# Speedo
SPEEDO_MAX_KMH   = 120.0
SPEEDO_TICK_STEP = 20

# G-Meter
G_CAP      = 1.5
RING_STEP  = 0.5

# drawing thickness
LINE_W_BASE = 3
LINE_W_LIVE = 2
FINISH_W    = 3
DOT_R       = 5

# ===================== CSV (single file) =====================
SAVE_DIR = "/home/pi/DJY/Laptime_data"
os.makedirs(SAVE_DIR, exist_ok=True)
_now = datetime.datetime.now()
CSV_PATH = os.path.join(SAVE_DIR, _now.strftime("Laptime_%Y_%m_%d_%H_%M.csv"))
CSV_HEADERS = [
    "timestamp_iso","type","lap_index","lap_time_s","lap_elapsed_s",
    "lat","lon","speed_kmh","g_lat","g_lon","delta_vs_best_s"
]

# ===================== Helpers =====================
EARTH_R      = 6371000.0
KNOTS_TO_KMH = 1.852
SOG_HOLD_SEC = 1.0

def fmt_lap_time(t: float | None) -> str:
    if t is None or t < 0: return "--"
    m = int(t // 60); s = int(t % 60)
    cs = int((t - int(t)) * 100)
    return "%d:%02d.%02d" % (m, s, cs)

def latlon_to_xy_m(lat: float, lon: float, ref_lat: float, ref_lon: float):
    lat_r = math.radians(lat); lon_r = math.radians(lon)
    rlat  = math.radians(ref_lat); rlon = math.radians(ref_lon)
    x = (lon_r - rlon) * math.cos((lat_r + rlat) * 0.5) * EARTH_R
    y = (lat_r - rlat) * EARTH_R
    return x, y

# ===================== Geometry / Lap timer =====================
@dataclass
class Line2D:
    ax: float; ay: float; bx: float; by: float
    def length(self): return math.hypot(self.bx - self.ax, self.by - self.ay)
    def direction(self):
        L = self.length()
        return (0.0, 0.0) if L == 0 else ((self.bx - self.ax)/L, (self.by - self.ay)/L)
    def segment_intersection(self, p0x, p0y, p1x, p1y):
        s1x = p1x - p0x; s1y = p1y - p0y
        s2x = self.bx - self.ax; s2y = self.by - self.ay
        denom = (-s2x * s1y + s1x * s2y)
        if abs(denom) < 1e-9: return (False, 0.0, 0.0)
        s = (-s1y * (p0x - self.ax) + s1x * (p0y - self.ay)) / denom
        t = ( s2x * (p0y - self.ay) - s2y * (p0x - self.ax)) / denom
        if 0.0 <= s <= 1.0 and 0.0 <= t <= 1.0: return (True, t, s)
        return (False, t, s)

@dataclass
class LapStats:
    lap_times: list[float]
    best: float | None
    last_lap_time: float | None

class FinishLineTimer:
    def __init__(self, pA_latlon, pB_latlon, direction=1, min_lap_s=20.0, cross_cooldown_s=4.0):
        self.ref_lat = (pA_latlon[0] + pB_latlon[0]) * 0.5
        self.ref_lon = (pA_latlon[1] + pB_latlon[1]) * 0.5
        ax, ay = latlon_to_xy_m(pA_latlon[0], pA_latlon[1], self.ref_lat, self.ref_lon)
        bx, by = latlon_to_xy_m(pB_latlon[0], pB_latlon[1], self.ref_lat, self.ref_lon)
        self.line = Line2D(ax, ay, bx, by)
        self.dir = direction
        self.min_lap = min_lap_s
        self.cooldown = cross_cooldown_s
        self.inset_frac = min(0.45, LINE_ENDPOINT_INSET_M / max(self.line.length(), 1e-6))
        self.last_cross_t = 0.0
        self.prev_xy = None
        self.lap_times = []
        self.last_lap_start = None

    def _dir_ok(self, vpx, vpy):
        if self.dir == 0: return True
        lx, ly = self.line.direction()
        ncomp = (vpx * (-ly) + vpy * (lx))
        return (ncomp > 0) if self.dir == 1 else (ncomp < 0)

    def update(self, lat, lon, t_now):
        x, y = latlon_to_xy_m(lat, lon, self.ref_lat, self.ref_lon)
        crossed = False; stats = None
        if self.prev_xy is not None:
            p0x, p0y = self.prev_xy
            inter, t_seg, u_line = self.line.segment_intersection(p0x, p0y, x, y)
            if inter and (self.inset_frac <= u_line <= (1.0 - self.inset_frac)):
                vpx = x - p0x; vpy = y - p0y
                if self._dir_ok(vpx, vpy) and (t_now - self.last_cross_t) >= self.cooldown:
                    if self.last_lap_start is None:
                        self.last_lap_start = t_now
                    else:
                        lap_time = t_now - self.last_lap_start
                        if lap_time >= self.min_lap:
                            self.lap_times.append(lap_time)
                            best = min(self.lap_times)
                            stats = LapStats(self.lap_times[:], best, lap_time)
                            self.last_lap_start = t_now
                            crossed = True
                    self.last_cross_t = t_now
        self.prev_xy = (x, y)
        return crossed, stats, (x, y)

# ===================== NMEA (RMC) =====================
KNOTS_TO_KMH = 1.852
class NMEA:
    @staticmethod
    def parse_rmc(line):
        try:
            if not (line.startswith("$GPRMC") or line.startswith("$GNRMC")):
                return None
            tokens = line.split(',')
            if len(tokens) < 7: return None
            if tokens[2] != 'A': return None
            lat_raw, lat_hemi = tokens[3], tokens[4]
            lon_raw, lon_hemi = tokens[5], tokens[6]
            sog_knots = tokens[7] if tokens[7] != '' else None
            lat = NMEA._dm_to_deg(lat_raw, lat_hemi)
            lon = NMEA._dm_to_deg(lon_raw, lon_hemi)
            sog_kmh = float(sog_knots) * KNOTS_TO_KMH if sog_knots is not None else None
            return (lat, lon, sog_kmh)
        except Exception:
            return None

    @staticmethod
    def _dm_to_deg(dm, hemi):
        if not dm: return None
        dm = float(dm)
        deg = math.floor(dm / 100.0)
        minutes = dm - deg * 100.0
        val = deg + minutes / 60.0
        if hemi in ('S', 'W'): val = -val
        return val

# ===================== CSV helpers =====================
def ensure_csv(path, headers):
    if not os.path.exists(path):
        with open(path, 'w', newline='') as f:
            csv.writer(f).writerow(headers)

def append_csv_row(path, row_type, **kw):
    ts = time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime())
    def _fmt(v, fmt): return "" if v is None else (fmt % v)
    row = [
        ts, row_type, str(kw.get("lap_index") or ""),
        _fmt(kw.get("lap_time_s"), "%.3f"),
        _fmt(kw.get("lap_elapsed_s"), "%.3f"),
        _fmt(kw.get("lat"), "%.7f"),
        _fmt(kw.get("lon"), "%.7f"),
        _fmt(kw.get("speed_kmh"), "%.2f"),
        _fmt(kw.get("g_lat"), "+%.3f"),
        _fmt(kw.get("g_lon"), "+%.3f"),
        _fmt(kw.get("delta_vs_best_s"), "+%.3f"),
    ]
    with open(path, 'a', newline='') as f:
        csv.writer(f).writerow(row)

# ===================== IMU G-meter Core =====================
_num_re = re.compile(r"[-+]?\d+(?:\.\d+)?")

def _parse_numbers(line: str):
    if not line: return None
    if LINE_MUST_START and not line.startswith(LINE_MUST_START): return None
    try:
        nums = [float(x) for x in _num_re.findall(line)]
        return nums if nums else None
    except:
        return None

def _extract_accel(nums):
    if not nums or len(nums) < 3: return None
    start = ACC_TRIPLE_INDEX if ACC_TRIPLE_INDEX >= 0 else len(nums) + ACC_TRIPLE_INDEX
    if start < 0 or start + 2 >= len(nums): return None
    return nums[start], nums[start+1], nums[start+2]

def _autoscale_to_g(ax, ay, az):
    if ACC_UNIT.lower() == "mps2":
        ax /= G_SI; ay /= G_SI; az /= G_SI
    mx = max(abs(ax), abs(ay), abs(az))
    if 50 <= mx < 5000:  # mg → g
        ax *= 0.001; ay *= 0.001; az *= 0.001
    return ax, ay, az

def _clamp(v, lo, hi): 
    return lo if v < lo else (hi if v > hi else v)

class GMeterCore:
    def __init__(self):
        self.range_g = 2.0
        self.swap_xy = False
        self.x_sign  = X_SIGN_INIT
        self.y_sign  = Y_SIGN_INIT
        self.yaw_deg = 0.0
        self.X_disp = 0.0; self.Y_disp = 0.0
        self._prev_X = 0.0; self._prev_Y = 0.0
        self._X_ma = collections.deque(maxlen=5)
        self._Y_ma = collections.deque(maxlen=5)
        self.peak_lat_abs = 0.0
        self.peak_lat_g = 0.0
        self.peak_lon_g = 0.0
        self._reset_at = None
        self.input_hz = 0.0
        self._rate_win_start = time.monotonic()
        self._rate_count = 0
        self._lock = threading.Lock()

    def _map_vehicle_xy(self, axg, ayg, azg):
        vec = {"x": axg, "y": ayg, "z": azg}
        rx = vec[MAP_X_FROM]; ry = vec[MAP_Y_FROM]
        if self.swap_xy: rx, ry = ry, rx
        rx *= (1 if self.x_sign > 0 else -1)
        ry *= (1 if self.y_sign > 0 else -1)
        th = math.radians(self.yaw_deg)
        c, s = math.cos(th), math.sin(th)
        X =  c*rx + s*ry
        Y = -s*rx + c*ry
        return X, Y

    def ingest_line(self, line: str):
        nums = _parse_numbers(line)
        if not nums: self._tick_rate(); return
        acc = _extract_accel(nums)
        if not acc: self._tick_rate(); return
        ax, ay, az = _autoscale_to_g(*acc)
        X_raw, Y_raw = self._map_vehicle_xy(ax, ay, az)
        if max(abs(X_raw), abs(Y_raw), abs(az)) > 20.0:
            self._tick_rate(); return
        dX = _clamp(X_raw - self._prev_X, -0.5, 0.5)
        dY = _clamp(Y_raw - self._prev_Y, -0.5, 0.5)
        X_raw = self._prev_X + dX; Y_raw = self._prev_Y + dY
        self._prev_X, self._prev_Y = X_raw, Y_raw
        self._X_ma.append(X_raw); self._Y_ma.append(Y_raw)
        X_f = sum(self._X_ma)/len(self._X_ma); Y_f = sum(self._Y_ma)/len(self._Y_ma)
        now = time.monotonic()
        with self._lock:
            self.X_disp += 0.20 * (X_f - self.X_disp)
            self.Y_disp += 0.20 * (Y_f - self.Y_disp)
            if abs(X_f) > self.peak_lat_abs:
                self.peak_lat_abs = abs(X_f)
                self.peak_lat_g = X_f
                self.peak_lon_g = Y_f
            if abs(self.X_disp) <= 0.3:
                if self._reset_at is None: self._reset_at = now + 1.5
            else:
                self._reset_at = None
            if self._reset_at is not None and now >= self._reset_at:
                self.peak_lat_abs = 0.0
                self.peak_lat_g = 0.0
                self.peak_lon_g = 0.0
                self._reset_at = None
        self._tick_rate()

    def _tick_rate(self):
        self._rate_count += 1
        now = time.monotonic()
        if now - self._rate_win_start >= 0.5:
            self.input_hz = self._rate_count / (now - self._rate_win_start)
            self._rate_count = 0
            self._rate_win_start = now

    def get_values(self):
        with self._lock:
            return (self.X_disp, self.Y_disp, self.peak_lat_g, self.peak_lon_g, self.input_hz)

    def on_key(self, key):
        if key == pygame.K_c:
            with self._lock:
                self.peak_lat_abs = 0.0; self.peak_lat_g = 0.0; self.peak_lon_g = 0.0; self._reset_at = None
        elif key in (pygame.K_PLUS, pygame.K_EQUALS): self.range_g = min(5.0, self.range_g + 0.5)
        elif key == pygame.K_MINUS: self.range_g = max(0.5, self.range_g - 0.5)
        elif key == pygame.K_s: self.swap_xy = not self.swap_xy
        elif key == pygame.K_f: self.x_sign *= -1
        elif key == pygame.K_g: self.y_sign *= -1
        elif key == pygame.K_LEFTBRACKET:  self.yaw_deg -= 2.0
        elif key == pygame.K_RIGHTBRACKET: self.yaw_deg += 2.0

class IMUReader(threading.Thread):
    def __init__(self, core: GMeterCore):
        super().__init__(daemon=True)
        self.core = core
        self._stop = False
        self.ser = None
    def run(self):
        try:
            self.ser = serial.Serial(IMU_PORT, IMU_BAUD, timeout=SER_TIMEOUT)
            time.sleep(0.1); self.ser.reset_input_buffer()
        except Exception as e:
            print("[IMU] open fail:", e); self.ser = None
        while not self._stop:
            try:
                line = self.ser.readline().decode(errors="ignore").strip() if self.ser else ""
                self.core.ingest_line(line)
            except Exception:
                pass
        if self.ser:
            try: self.ser.close()
            except: pass
    def stop(self): self._stop = True

# ===================== GNSS 10Hz auto-config (ZED-F9P) =====================
def configure_gnss_ubx_10hz(port: str, baud: int):
    if not HAS_UBX:
        print("[GNSS] pyubx2 미설치 → 자동 설정 생략")
        return
    try:
        with serial.Serial(port, baud, timeout=2) as s:
            # 1) 측정 주기: 10 Hz → measRate=100 ms
            cfg_rate = UBXMessage("CFG", "CFG-RATE", SET,
                                  measRate=100,   # ms (10 Hz)
                                  navRate=1,
                                  timeRef=1)     # 0: UTC, 1: GPS
            s.write(cfg_rate.serialize())

            # 2) NMEA 메시지: USB에서 RMC/VTG만 ON(1), 나머지 OFF(0)
            for msgid in (0x04, 0x05):  # RMC=0x04, VTG=0x05
                cfg_msg = UBXMessage("CFG", "CFG-MSG", SET,
                                     msgClass=0xF0, msgID=msgid,
                                     rateDDC=0, rateUART1=0, rateUSB=1, rateSPI=0, rateI2C=0)
                s.write(cfg_msg.serialize())
            for msgid in (0x00, 0x01, 0x02, 0x03, 0x06, 0x07):  # GGA,GLL,GSA,GSV,GRS,GST OFF
                cfg_msg_off = UBXMessage("CFG", "CFG-MSG", SET,
                                         msgClass=0xF0, msgID=msgid,
                                         rateDDC=0, rateUART1=0, rateUSB=0, rateSPI=0, rateI2C=0)
                s.write(cfg_msg_off.serialize())

            # 3) 저장
            save = UBXMessage("CFG", "CFG-CFG", SET,
                              clearMask=b"\x00\x00\x00\x00",
                              saveMask =b"\xFF\xFF\xFF\xFF",
                              loadMask =b"\x00\x00\x00\x00",
                              deviceMask=b"\x17")  # BBR+Flash(+EEPROM옵션)
            s.write(save.serialize())
        print("[GNSS] ZED-F9P set to 10 Hz (USB: RMC/VTG only), config saved.")
    except Exception as e:
        print("[GNSS] auto-config 실패:", e)

# ===================== HUD widgets =====================
class MiniMap:
    def __init__(self, rect):
        self.rect = pygame.Rect(rect)
        self.base_path = []; self.live_path = []; self.frozen = False; self.ax_by = None
    def set_finish_line(self, line: 'Line2D'): self.ax_by = ((line.ax, line.ay), (line.bx, line.by))
    def feed_point(self, xy, recording_first_lap):
        if recording_first_lap and not self.frozen: self.base_path.append(xy)
        else:
            self.live_path.append(xy)
            if len(self.live_path) > 4000: self.live_path.pop(0)
    def freeze_base(self): self.frozen = True; self.live_path = []
    def _bounds(self, current_xy):
        pts = []; pts.extend(self.base_path); pts.extend(self.live_path)
        if self.ax_by: pts.extend(self.ax_by)
        if current_xy: pts.append(current_xy)
        if not pts: return (-10, -10, 20, 20)
        xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
        minx, maxx = min(xs), max(xs); miny, maxy = min(ys), max(ys)
        if maxx-minx < 5: minx -= 5; maxx += 5
        if maxy-miny < 5: miny -= 5; maxy += 5
        return (minx, miny, maxx-minx, maxy-miny)
    def _xy_to_screen(self, x, y, bounds):
        rx, ry, rw, rh = self.rect
        bx, by, bw, bh = bounds
        pad_x = bw * PAD_FRACTION; pad_y = bh * PAD_FRACTION
        bw += 2*pad_x; bh += 2*pad_y; bx -= pad_x; by -= pad_y
        scale = min(rw / bw, rh / bh)
        sx = rx + (x - bx) * scale; sy = ry + (bh - (y - by)) * scale
        return int(sx), int(sy)
    def draw(self, surf, current_xy, last_xy):
        pygame.draw.rect(surf, (30,30,30), self.rect)
        bounds = self._bounds(current_xy or last_xy)
        if len(self.base_path) >= 2:
            pts = [self._xy_to_screen(x,y, bounds) for (x,y) in self.base_path]
            pygame.draw.lines(surf, (200,200,200), False, pts, LINE_W_BASE)
        if len(self.live_path) >= 2:
            pts = [self._xy_to_screen(x,y, bounds) for (x,y) in self.live_path]
            pygame.draw.lines(surf, (140,140,140), False, pts, LINE_W_LIVE)
        if self.ax_by:
            (ax,ay),(bx,by) = self.ax_by
            a = self._xy_to_screen(ax,ay, bounds); b = self._xy_to_screen(bx,by, bounds)
            pygame.draw.line(surf, (255,80,80), a, b, FINISH_W)
        draw_xy = current_xy or last_xy
        if draw_xy:
            cx, cy = self._xy_to_screen(draw_xy[0], draw_xy[1], bounds)
            pygame.draw.circle(surf, (80,200,255), (cx,cy), DOT_R)

class GMeterHUD:
    """정사각 G-meter HUD
    - draw(..., range_g): 스케일을 동적으로(±range_g) 반영
    - 피크(빨강) 먼저 → 현재(하늘색) 나중에 그려서 현재 점이 위에 보이도록
    """
    def __init__(self, pos, size):
        self.rect = pygame.Rect(pos, size)
        self.R_pix = min(size) // 2 - 12

    def _clamp_dot(self, cx, cy, px, py):
        dx = px - cx
        dy = py - cy
        r = math.hypot(dx, dy)
        if r <= self.R_pix or r == 0:
            return (px, py)
        s = self.R_pix / r
        return (int(cx + dx * s), int(cy + dy * s))

    def draw(self, surf, glat, glon, peak_lat_g, peak_lon_g, range_g):
        # 패널 배경
        pygame.draw.rect(surf, (30, 30, 30), self.rect)

        x, y, w, h = self.rect
        cx = x + w // 2
        cy = y + h // 2

        # 동적 스케일 (최소 0.5 g 보장)
        cap = G_CAP

        # 동심 원(0.5 g 간격)
        steps = int(round(cap / RING_STEP))
        for i in range(1, steps + 1):
            gval = i * RING_STEP
            r = int(self.R_pix * (min(gval, cap) / cap))
            pygame.draw.circle(
                surf,
                (70, 70, 70) if gval < cap else (100, 100, 100),
                (cx, cy),
                r,
                1
            )

        # 십자선
        pygame.draw.line(surf, (80, 80, 80), (cx - self.R_pix, cy), (cx + self.R_pix, cy), 1)
        pygame.draw.line(surf, (80, 80, 80), (cx, cy - self.R_pix), (cx, cy + self.R_pix), 1)

        # ---- 피크점 ----
        pnx = max(-1.0, min(1.0, peak_lat_g / cap))
        pny = max(-1.0, min(1.0, peak_lon_g / cap))
        px = int(cx + pnx * self.R_pix)
        py = int(cy - pny * self.R_pix)
        px, py = self._clamp_dot(cx, cy, px, py)
        pygame.draw.circle(surf, (255, 64, 64), (px, py), 6)

        # ---- 현재점 ----
        nx = max(-1.0, min(1.0, glat / cap))
        ny = max(-1.0, min(1.0, glon / cap))
        sx = int(cx + nx * self.R_pix)
        sy = int(cy - ny * self.R_pix)
        sx, sy = self._clamp_dot(cx, cy, sx, sy)
        pygame.draw.circle(surf, (80, 200, 255), (sx, sy), 7)

class LapBoxHUD:
    def __init__(self, rect): self.rect = pygame.Rect(rect)
    def draw(self, surf, font_label, font_time, last_t, best_t, run_t):
        pygame.draw.rect(surf, (30,30,30), self.rect)
        x, y, w, h = self.rect; pad = 10; oy = y + pad
        lbl = font_label.render("Last:", True, (240,240,240)); surf.blit(lbl, (x+pad, oy)); oy += lbl.get_height()+4
        val = font_time.render(fmt_lap_time(last_t), True, (240,240,240)); surf.blit(val, (x+pad, oy)); oy += val.get_height()+8
        lbl = font_label.render("Best:", True, (240,240,240)); surf.blit(lbl, (x+pad, oy)); oy += lbl.get_height()+4
        val = font_time.render(fmt_lap_time(best_t if best_t is not None else -1), True, (240,240,240)); surf.blit(val, (x+pad, oy)); oy += val.get_height()+8
        lbl = font_label.render("Run :", True, (240,240,240)); surf.blit(lbl, (x+pad, oy)); oy += lbl.get_height()+4
        val = font_time.render(fmt_lap_time(run_t if run_t is not None else -1), True, (240,240,240)); surf.blit(val, (x+pad, oy))

class AnalogSpeedoHUD:
    """우측 하단 아날로그 속도계"""
    def __init__(self, rect): self.rect = pygame.Rect(rect)
    def draw(self, surf, font_num, speed_kmh):
        pygame.draw.rect(surf, (30,30,30), self.rect)
        x, y, w, h = self.rect
        r  = int(min(w, h) * 0.38)
        cx = x + int(w * 0.48)
        cy = y + int(h * 0.48)
        ang_min = math.radians(-190)
        ang_max = math.radians( +20)
        pygame.draw.circle(surf, (50,50,50), (cx, cy), r, 2)
        for v in range(0, int(SPEEDO_MAX_KMH)+1, SPEEDO_TICK_STEP):
            frac = v / SPEEDO_MAX_KMH
            ang = ang_min + (ang_max - ang_min) * frac
            sx = cx + int(math.cos(ang) * (r-6));  sy = cy + int(math.sin(ang) * (r-6))
            ex = cx + int(math.cos(ang) * (r-18)); ey = cy + int(math.sin(ang) * (r-18))
            pygame.draw.line(surf, (140,140,140), (sx, sy), (ex, ey), 2)
            tx = cx + int(math.cos(ang) * (r-34)); ty = cy + int(math.sin(ang) * (r-34))
            img = font_num.render(str(v), True, (255,80,80))
            surf.blit(img, img.get_rect(center=(tx, ty)))
        spd = max(0.0, min(SPEEDO_MAX_KMH, 0.0 if speed_kmh is None else speed_kmh))
        frac = spd / SPEEDO_MAX_KMH
        ang = ang_min + (ang_max - ang_min) * frac
        nx = cx + int(math.cos(ang) * (r-22)); ny = cy + int(math.sin(ang) * (r-22))
        pygame.draw.line(surf, (80,200,255), (cx, cy), (nx, ny), 4)
        pygame.draw.circle(surf, (200,200,200), (cx, cy), 6)
        spd_txt = "%3.0f kph" % spd
        img_spd = font_num.render(spd_txt, True, (240,240,240))
        tx = cx
        ty = cy + 40
        surf.blit(img_spd, img_spd.get_rect(center=(tx, ty)))

# ===================== Main =====================
class RunningAvg:
    def __init__(self, n=30): self.buf = collections.deque(maxlen=n)
    def add_dt(self, dt): 
        if dt>0: self.buf.append(1.0/dt)
    def fps(self): 
        return sum(self.buf)/len(self.buf) if self.buf else 0.0

def open_serial(port, baud): return serial.Serial(port, baudrate=baud, timeout=0.05)
def ensure_setup(): ensure_csv(CSV_PATH, CSV_HEADERS)

def demo_loop():
    ensure_setup()

    # GNSS 10 Hz 자동 설정
    if AUTO_CONFIG_GNSS:
        configure_gnss_ubx_10hz(PORT_GNSS, BAUD_GNSS)

    fl = FinishLineTimer(POINT_A, POINT_B, direction=LAP_DIRECTION,
                         min_lap_s=MIN_LAP_TIME_S, cross_cooldown_s=CROSS_COOLDOWN_S)
    # GNSS open
    try:
        ser_gnss = open_serial(PORT_GNSS, BAUD_GNSS)
    except Exception as e:
        print("[GNSS] open fail:", e); ser_gnss = None

    # IMU
    gcore = GMeterCore(); imu_th = IMUReader(gcore); imu_th.start()

    if pygame is None:
        print("[WARN] pygame not available"); return
    pygame.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption("LapTimer + MiniMap + Analog Speedo + IMU G-Meter (800x480)")
    try:
        font_label = pygame.font.SysFont("consolas", FONT_LAP_LABEL)
        font_time  = pygame.font.SysFont("consolas", FONT_LAP_TIME)
        font_num   = pygame.font.SysFont("consolas", FONT_SPEED_NUM)
    except Exception:
        font_label = pygame.font.Font(None, FONT_LAP_LABEL)
        font_time  = pygame.font.Font(None, FONT_LAP_TIME)
        font_num   = pygame.font.Font(None, FONT_SPEED_NUM)

    mm = MiniMap(MAP_RECT); mm.set_finish_line(fl.line)
    gmhud = GMeterHUD(GM_POS, GM_SIZE)
    laphud = LapBoxHUD(LAP_RECT)
    speedohud = AnalogSpeedoHUD(SPD_RECT)

    # state
    recording_first_lap = True
    last_print = 0.0; fpsmeter = RunningAvg(); prev_t = time.time()
    best = None; last_lap_time = None; last_rmc_time = 0.0
    last_xy_drawn = None; last_speed_kmh = 0.0; last_speed_time = 0.0

    print("Ready. GNSS RMC + IMU stream...")

    running = True
    while running:
        try:
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT: running = False
                elif ev.type == pygame.KEYDOWN:
                    if ev.key in (pygame.K_ESCAPE, pygame.K_q): running = False
                    else: gcore.on_key(ev.key)

            # GNSS read
            line = ""
            if ser_gnss:
                try: line = ser_gnss.readline().decode(errors="ignore").strip()
                except Exception: line = ""

            now = time.time(); dt = now - prev_t; prev_t = now; fpsmeter.add_dt(dt)
            current_xy = None; stats = None; sog_kmh = None; lat = lon = None

            if line:
                r = NMEA.parse_rmc(line)
                if r:
                    lat, lon, sog_kmh = r; last_rmc_time = now
                    crossed, stats, current_xy = fl.update(lat, lon, now)
                    last_xy_drawn = current_xy; mm.feed_point(current_xy, recording_first_lap)
                    if sog_kmh is not None: last_speed_kmh = sog_kmh; last_speed_time = now
                    if crossed and stats:
                        last_lap_time = stats.last_lap_time; best = stats.best
                        if recording_first_lap: recording_first_lap = False; mm.freeze_base()
                        append_csv_row(CSV_PATH, "LAP", lap_index=len(stats.lap_times),
                                       lap_time_s=last_lap_time, delta_vs_best_s=0.0)

            # hold last speed 1s
            speed_kmh = last_speed_kmh if (now - last_speed_time) <= SOG_HOLD_SEC else 0.0
            # IMU values
            Xg, Yg, peakX, peakY, inHz = gcore.get_values()
            glat = max(-G_CAP, min(G_CAP, Xg)); glon = max(-G_CAP, min(G_CAP, Yg))
            peakX = max(-G_CAP, min(G_CAP, peakX)); peakY = max(-G_CAP, min(G_CAP, peakY))

            # per-sample CSV during a lap
            if fl.last_lap_start is not None and (lat is not None) and (lon is not None):
                current_lap_index = len(fl.lap_times) + 1
                lap_elapsed = now - fl.last_lap_start
                append_csv_row(CSV_PATH, "SAMPLE", lap_index=current_lap_index,
                               lap_elapsed_s=lap_elapsed, lat=lat, lon=lon,
                               speed_kmh=max(0.0, speed_kmh), g_lat=glat, g_lon=glon)

            # ---- draw ----
            screen.fill((10,10,10))
            draw_xy = current_xy if current_xy is not None else (last_xy_drawn if (now - last_rmc_time) <= 2.0 else None)
            mm.draw(screen, draw_xy, last_xy_drawn)
            gmhud.draw(screen, glat, glon, peakX, peakY, gcore.range_g)
            run_t = (now - fl.last_lap_start) if fl.last_lap_start is not None else None
            laphud.draw(screen, font_label, font_time, last_lap_time, best, run_t)
            speedohud.draw(screen, font_num, max(0.0, speed_kmh))
            pygame.display.flip()

            if now - last_print > 1.0:
                last_print = now
                sys.stdout.write("\rfps~%.1f laps=%d imu~%.1fHz   " % (fpsmeter.fps(), len(fl.lap_times), inHz))
                sys.stdout.flush()

        except KeyboardInterrupt:
            running = False
        except Exception as e:
            print("[ERR]", e)

    # shutdown
    if pygame: pygame.quit()
    if 'ser_gnss' in locals() and ser_gnss:
        try: ser_gnss.close()
        except: pass

if __name__ == '__main__':
    demo_loop()
