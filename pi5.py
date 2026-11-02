#!/usr/bin/env python3
"""
pi5_controller.py  —  รันบน Raspberry Pi 5  ("สมอง" ของเครื่อง)   * เวอร์ชันแกนเดียว *

เครื่อง: stepper 1 ตัว (TB6600, 800 pulse/rev, 1 รอบ = 8 cm -> 100 steps = 1 cm), ระยะทำงาน 60 cm
กล้อง: Raspberry Pi HQ Camera (IMX477)

หน้าที่:
  1. ถ่ายภาพด้วย Picamera2
  2. หาตำแหน่ง (x, y) ของมาร์กเกอร์ 4 สีด้วย OpenCV (HSV threshold + centroid)
  3. เช็คความชัดของภาพ (Variance of Laplacian)
  4. ตัดสินใจตามเงื่อนไข 4 กรณี + ทำ PID แล้วสั่ง ESP32 ขยับ stepper
  5. ส่งภาพ/สถานะไปที่คอม และรับคำสั่งจากคอม

เรื่องแกนเดียว:
  มอเตอร์ขยับกล้องได้ทิศเดียว มาร์กเกอร์ในภาพจึงเลื่อนไปตาม "ทิศของแกน" อย่างเดียว
  error ของมาร์กเกอร์ (x, y) จึงถูกแยกเป็น 2 ส่วน
    - along : ส่วนที่อยู่ในแนวแกน  -> แก้ได้ด้วยมอเตอร์ (ใช้ทำ PID และตัดสินว่าตำแหน่งถูก)
    - perp  : ส่วนที่ตั้งฉากกับแกน -> มอเตอร์แก้ไม่ได้ (แจ้งเตือนเท่านั้น เช่น ชิ้นงานวางเบี้ยว)
  ทิศของแกนในภาพได้จากปุ่ม Calibrate

ติดตั้ง:
  sudo apt install -y python3-picamera2 python3-opencv python3-serial python3-numpy
"""
import json
import logging
import os
import queue
import socket
import struct
import threading
import time

import cv2
import numpy as np
import serial
from picamera2 import Picamera2

try:
    from libcamera import controls
except ImportError:
    controls = None

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger("pi5")

# ============================================================
# CONFIG  (ครั้งแรกจะสร้างไฟล์ config.json ให้ แก้ค่าในไฟล์นั้นได้เลย)
# ============================================================
CONFIG_FILE = "config.json"
DEFAULT_STATIONS = [                    # ตำแหน่งถ่ายภาพ 4 จุด: แต่ละจุดมีมาร์กเกอร์ของตัวเอง 1 สี
    {"name": "P1", "cm": 5.0,  "color": None, "ref": None},   # color/ref ได้จากปุ่ม Teach
    {"name": "P2", "cm": 15.0, "color": None, "ref": None},
    {"name": "P3", "cm": 25.0, "color": None, "ref": None},
    {"name": "P4", "cm": 35.0, "color": None, "ref": None},
]
DEFAULT_CONFIG = {
    "serial_port": "/dev/ttyUSB0",      # ESP32 (ลอง ls /dev/ttyUSB* /dev/ttyACM*)  ทดสอบ: /tmp/fake_esp32
    "baud": 115200,
    "pc_host": "192.168.1.100",         # IP ของคอม
    "pc_port": 5000,
    "resolution": [2028, 1520],         # HQ camera
    "lens_position": None,              # HQ ไม่มี autofocus -> ปล่อย None
    "home_before_run": True,
    # ---- เครื่องกล (ต้องตรงกับ esp32_stepper.ino) ----
    "steps_per_cm": 100.0,              # 800 pulse/rev ÷ 8 cm/rev
    "travel_cm": 60.0,
    # ---- ภาพ ----
    "blur_threshold": 8.0,              # เกณฑ์ความชัด สำหรับจุดที่ยังไม่มี sharp_ref (คม ~15-20, เบลอ < 5)
    "blur_ratio": 0.7,                  # จุดที่ Teach แล้ว: ผ่านถ้า sharp >= 70% ของ sharp ตอน Teach
    "sharp_roi_px": 200,                # วัดความคมในกรอบ ±200 px รอบมาร์กเกอร์ (กระดาษ 2.5 cm ≈ 230 px)
    "sharp_version": 3,                 # วิธีวัดความคม (3 = ช่อง S, ไม่ขึ้นกับแสงและสี)
    "pos_tolerance_px": 4.0,            # error ตามแนวแกนที่ยอมรับได้ (pixel)
    "perp_warn_px": 15.0,               # คลาดขวางรางเกินนี้ -> แจ้งเตือน
    "perp_max_px": 40.0,                # คลาดขวางรางเกินนี้ -> ไม่ผ่าน ไม่ถ่ายเก็บ (มอเตอร์แก้ไม่ได้)
    "settle_time_s": 0.5,               # รอให้เครื่องหยุดสั่นก่อนถ่าย
    "max_blur_retry": 3,
    "max_pid_iter": 10,
    "pid": {"kp": 0.8, "ki": 0.1, "kd": 0.05, "i_limit": 50.0},
    "max_correction_cm": 3.0,           # PID ขยับได้ไม่เกินครั้งละ 3 cm
    "calib_cm": 0.5,                    # ระยะขยับตอน Calibrate (เล็กไว้ เผื่อมาร์กเกอร์อยู่ใกล้ขอบภาพ)
    "px_per_step": None,                # ได้จาก Calibrate (อย่าแก้มือ)
    "live_fps": 5,                      # ภาพสดที่ส่งไปคอม (เฟรม/วินาที)
    "live_width": 960,                  # ความกว้างภาพสด (pixel)
    # ---- การหามาร์กเกอร์ (กระดาษสี่เหลี่ยม 4 สี) ----
    "min_area_px": 2000,                # ก้อนสีเล็กกว่านี้ไม่นับ (pixel ของภาพเต็ม)
    "max_area_px": 0,                   # ก้อนสีใหญ่กว่านี้ไม่นับ (0 = ไม่จำกัด)
    "square_only": True,                # รับเฉพาะก้อนสีที่เป็นสี่เหลี่ยม
    "max_aspect": 1.6,                  # ด้านยาว/ด้านสั้น ไม่เกินนี้
    "min_fill": 0.75,                   # พื้นที่สี / พื้นที่กรอบสี่เหลี่ยม ไม่น้อยกว่านี้
    "colors": {                         # ช่วง HSV ของแต่ละสี (OpenCV: H 0-180) — ปรับด้วยปุ่ม "สอนสี" บนคอม
        "purple": [[[125, 50, 40], [165, 255, 255]]],
        "brown":  [[[5, 80, 30], [22, 255, 140]]],
        "yellow": [[[22, 100, 120], [38, 255, 255]]],
        "orange": [[[5, 150, 150], [20, 255, 255]]],
    },
    "color_samples": {},                # สีตัวอย่างจากปุ่ม "สอนสี" (อย่าแก้มือ)
    # ---- ปรับภาพให้สเกลเท่ากันทั้งภาพ (แท็บ "กล้อง/สเกล" บนคอม) ----
    "chess_cols": 9,                    # จำนวน "มุมด้านใน" ของกระดานหมากรุก (แนวนอน)
    "chess_rows": 6,                    # จำนวน "มุมด้านใน" (แนวตั้ง)   กระดาน 10x7 ช่อง -> 9x6 มุม
    "chess_square_mm": 12.0,            # ขนาดช่องที่วัดจริงด้วยไม้บรรทัดหลังพิมพ์ (กระดาน 10x7 ช่อง = 12x8.4 cm)
    "rectify_captures": True,           # ภาพที่บันทึกตอน START ผ่านการปรับแล้ว (ถ้ามีไฟล์ rectify.npz)
    "stations": DEFAULT_STATIONS,
}


def load_config():
    if not os.path.exists(CONFIG_FILE):
        cfg = json.loads(json.dumps(DEFAULT_CONFIG))
        save_config(cfg)
        return cfg
    with open(CONFIG_FILE) as f:
        cfg = json.load(f)
    old_sharp = cfg.get("sharp_version", 1)
    for k, v in DEFAULT_CONFIG.items():
        cfg.setdefault(k, json.loads(json.dumps(v)))
    # วิธีวัดความคมเปลี่ยน -> ค่าเดิมคนละสเกล: เก็บตำแหน่ง/สีไว้ ล้างแค่ค่าความคมอ้างอิง
    if old_sharp < 3:
        log.warning("เปลี่ยนวิธีวัดความคม -> ล้าง sharp_ref (ตำแหน่ง Teach เดิมยังอยู่) ควร Teach ใหม่")
        for s in cfg["stations"]:
            s.pop("sharp_ref", None)
        cfg["blur_threshold"] = DEFAULT_CONFIG["blur_threshold"]
        cfg["sharp_version"] = 3
        cfg["_note"] = "sharp_reset"
        save_config(cfg)
    # config เก่า (แบบ 2 แกน) -> เปลี่ยนเป็นแบบแกนเดียว
    if any("cm" not in s for s in cfg["stations"]) or "px_to_steps" in cfg:
        log.warning("พบ config แบบ 2 แกนเดิม -> รีเซ็ต stations/calibrate เป็นแบบแกนเดียว")
        cfg["stations"] = json.loads(json.dumps(DEFAULT_STATIONS))
        cfg.pop("px_to_steps", None)
        cfg.pop("calib_steps", None)
        cfg.pop("max_correction_steps", None)
        cfg["px_per_step"] = None
        save_config(cfg)
    # ชุดสีเดิม (แดง/เขียว/น้ำเงิน/เหลือง) -> เปลี่ยนเป็นชุดใหม่ (ม่วง/น้ำตาล/เหลือง/ส้ม)
    if set(cfg["colors"]) != set(DEFAULT_CONFIG["colors"]):
        log.warning("เปลี่ยนชุดสีมาร์กเกอร์เป็น %s -> ต้อง Teach ใหม่", list(DEFAULT_CONFIG["colors"]))
        cfg["colors"] = json.loads(json.dumps(DEFAULT_CONFIG["colors"]))
        cfg["color_samples"] = {}
        for s in cfg["stations"]:
            s["ref"] = None
        if cfg.get("min_area_px", 0) < DEFAULT_CONFIG["min_area_px"]:
            cfg["min_area_px"] = DEFAULT_CONFIG["min_area_px"]
        save_config(cfg)
    return cfg


def save_config(cfg):
    with open(CONFIG_FILE, "w") as f:
        json.dump(cfg, f, indent=2)


# ============================================================
# VISION
# ============================================================
def blur_score(bgr):
    """ค่ายิ่งสูง = ภาพยิ่งชัด (Variance of Laplacian)"""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def color_mask(hsv, ranges):
    mask = None
    for lo, hi in ranges:
        m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    return mask


def is_square(c, cfg):
    """ก้อนสีนี้เป็นแผ่นสี่เหลี่ยมไหม (ไม่ใช่สายไฟ ขอบโต๊ะ หรือเงา)"""
    (_, _), (w, h), _ = cv2.minAreaRect(c)
    if min(w, h) < 1:
        return False
    aspect = max(w, h) / min(w, h)
    fill = cv2.contourArea(c) / (w * h)
    return aspect <= cfg["max_aspect"] and fill >= cfg["min_fill"]


def sharp_center(found, ref, shape):
    """จุดที่ใช้วัดความคม: มาร์กเกอร์ของจุดนี้ -> ตำแหน่งอ้างอิง -> มาร์กเกอร์ที่ใกล้กลางภาพ"""
    if ref:
        c = next(iter(ref))
        return found.get(c, ref[c])
    if found:
        h, w = shape[:2]
        return min(found.values(), key=lambda p: np.hypot(p[0] - w / 2, p[1] - h / 2))
    return None


def norm_sharpness(bgr):
    """
    ความคมของขอบกระดาษสี วัดบนช่อง Saturation (S) ของ HSV
      - กระดาษสีมี S สูง พื้นขาวมี S ต่ำ -> ขอบชัดทุกสี (รวมสีเหลืองที่กลืนกับพื้นขาวในภาพขาวดำ)
      - S ไม่เปลี่ยนตามความสว่างของแสง
      - หารด้วย (ความต่างของ S)^2 -> ค่าใกล้เคียงกันทุกสี  ภาพคม ~15-20  ภาพเบลอ < 5
    """
    s = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)[:, :, 1].astype(np.float32)
    s = cv2.GaussianBlur(s, (3, 3), 0)
    lo, hi = np.percentile(s, (2, 98))
    c = max(float(hi - lo), 25.0)
    return float(cv2.Laplacian(s, cv2.CV_32F).var() / (c * c) * 10000)


def to_hsv(bgr, bg_target=220.0):
    """
    แปลงเป็น HSV แล้วปรับความสว่าง (V) ให้พื้นขาวมีค่าเท่าเดิมเสมอ
    แสงเปลี่ยน (เช้า/เย็น/เปิดไฟ) -> สีที่สอนไว้ยังใช้ได้ และแยกน้ำตาล/ส้มได้ (น้ำตาล = ส้มที่มืดกว่าพื้นขาว)
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    sm = hsv[::8, ::8]
    white = sm[(sm[:, :, 1] < 40) & (sm[:, :, 2] > 50)][:, 2]
    if white.size > 50:
        k = bg_target / max(float(np.median(white)), 1.0)
        if abs(k - 1.0) > 0.02:
            hsv[:, :, 2] = np.clip(hsv[:, :, 2].astype(np.float32) * k, 0, 255).astype(np.uint8)
    return hsv


def roi_sharpness(img, center, half):
    """วัดความคมเฉพาะกรอบเล็กรอบมาร์กเกอร์ (ขอบกระดาษสีบนพื้นขาว) ไม่รวมใบลาน/สายไฟ"""
    if center is None:
        return norm_sharpness(img)
    h, w = img.shape[:2]
    x, y = int(center[0]), int(center[1])
    crop = img[max(0, y - half):min(h, y + half), max(0, x - half):min(w, x + half)]
    if crop.size == 0 or min(crop.shape[:2]) < 20:
        return norm_sharpness(img)
    return norm_sharpness(crop)


def detect_markers(bgr, cfg):
    """คืนค่า {สี: (x, y)} = จุดกึ่งกลางของแผ่นสี่เหลี่ยมสีที่ใหญ่ที่สุดที่ผ่านตัวกรอง"""
    hsv = to_hsv(bgr)
    k = max(3, int(round(bgr.shape[1] / 400)) | 1)       # kernel ปรับตามขนาดภาพ
    kernel = np.ones((k, k), np.uint8)
    min_a = cfg["min_area_px"]
    max_a = cfg.get("max_area_px", 0) or float("inf")
    found = {}
    for name, ranges in cfg["colors"].items():
        mask = color_mask(hsv, ranges)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in sorted(cnts, key=cv2.contourArea, reverse=True):
            a = cv2.contourArea(c)
            if a < min_a:
                break                      # ที่เหลือเล็กกว่านี้ทั้งหมด
            if a > max_a:
                continue
            if cfg.get("square_only", True) and not is_square(c, cfg):
                continue
            M = cv2.moments(c)
            found[name] = (M["m10"] / M["m00"], M["m01"] / M["m00"])
            break
    return found


def _hue_diff(a, b):
    """b - a บนวงล้อสี H (0-180) ผลอยู่ใน -90..+90"""
    return ((b - a) + 90) % 180 - 90


def _make_range(h_lo, h_hi, s_lo, v_lo, v_hi):
    h_lo, h_hi, s_lo, v_lo, v_hi = (int(round(x)) for x in (h_lo, h_hi, s_lo, v_lo, v_hi))
    if h_lo < 0:
        return [[[0, s_lo, v_lo], [h_hi, 255, v_hi]], [[180 + h_lo, s_lo, v_lo], [180, 255, v_hi]]]
    if h_hi > 180:
        return [[[h_lo, s_lo, v_lo], [180, 255, v_hi]], [[0, s_lo, v_lo], [h_hi - 180, 255, v_hi]]]
    return [[[h_lo, s_lo, v_lo], [h_hi, 255, v_hi]]]


def ranges_from_samples(samples, dh=8, ds=70, dv=70):
    """
    สร้างช่วง HSV ของทุกสีจากสีตัวอย่างที่สอนไว้ {ชื่อ: [h, s, v]}
    สีที่เฉดใกล้กัน (เช่น น้ำตาล/ส้ม) จะถูกแบ่งเขตกันไม่ให้ทับ:
      ความสว่างต่างกันพอ -> แบ่งด้วย V,  ไม่งั้น -> แบ่งด้วย H
    """
    out = {}
    for name, (h, s, v) in samples.items():
        h_lo, h_hi = h - dh, h + dh
        s_lo = max(30, s - ds)
        v_lo, v_hi = max(20, v - dv), min(255, v + dv)
        for other, (h2, s2, v2) in samples.items():
            if other == name:
                continue
            d = _hue_diff(h, h2)
            if abs(d) > 2 * dh:
                continue                   # เฉดห่างกันพอ ไม่ทับกัน
            if abs(v2 - v) >= 30:          # แบ่งด้วยความสว่าง
                mid = (v + v2) / 2
                if v2 > v:
                    v_hi = min(v_hi, mid)
                else:
                    v_lo = max(v_lo, mid)
            else:                          # แบ่งด้วยเฉดสี
                if d > 0:
                    h_hi = min(h_hi, h + d / 2)
                else:
                    h_lo = max(h_lo, h + d / 2)
        out[name] = _make_range(h_lo, h_hi, s_lo, v_lo, v_hi)
    return out


def hsv_range_from_sample(hsv_px, dh=8, ds=70, dv=70):
    """ช่วง HSV จากสีตัวอย่างสีเดียว"""
    return ranges_from_samples({"_": list(hsv_px)}, dh, ds, dv)["_"]


def position_error(found, ref):
    """
    error = ref - found (เฉลี่ยจากทุกสีที่เจอ) หน่วย pixel
    คืนค่า (error เป็น np.array [ex, ey] หรือ None, จำนวนมาร์กที่ใช้ได้)
    """
    if ref is None:
        return None, len(found)
    common = [c for c in ref if c in found]
    if not common:
        return None, 0
    d = np.array([np.subtract(ref[c], found[c]) for c in common], float)
    return d.mean(axis=0), len(common)


DRAW_COLOR = {"purple": (200, 0, 180), "brown": (40, 80, 140), "yellow": (0, 230, 255),
              "orange": (0, 140, 255), "red": (0, 0, 255), "green": (0, 200, 0), "blue": (255, 0, 0)}


def _label(img, txt, org, color):
    """ตัวหนังสือมีพื้นหลังดำ อ่านง่ายบนพื้นขาว"""
    font, sc, th = cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
    (tw, tht), _ = cv2.getTextSize(txt, font, sc, th)
    x = int(min(max(0, org[0]), img.shape[1] - tw - 2))
    y = int(min(max(tht + 2, org[1]), img.shape[0] - 2))
    cv2.rectangle(img, (x - 2, y - tht - 3), (x + tw + 2, y + 3), (0, 0, 0), -1)
    cv2.putText(img, txt, (x, y), font, sc, color, th, cv2.LINE_AA)


def annotate(img, found, ref, text, width=960, coord_scale=1.0, lines=()):
    """
    วาดวงกลม = มาร์กเกอร์ที่เจอ, กากบาท = ตำแหน่งตอน Teach พร้อมตัวเลข x, y (pixel ของภาพเต็ม)
    coord_scale: ตัวคูณเพื่อแสดงพิกัดเป็นของภาพเต็ม (ภาพสดถูกย่อขนาด)
    """
    s = width / img.shape[1]
    out = cv2.resize(img, None, fx=s, fy=s)
    k = s                                   # พิกัดใน img -> พิกัดบนภาพที่ย่อแล้ว
    for c, (x, y) in found.items():
        col = DRAW_COLOR.get(c, (255, 255, 255))
        p = (int(x * k), int(y * k))
        cv2.circle(out, p, 14, col, 3)
        _label(out, f"{c} ({x * coord_scale:.0f}, {y * coord_scale:.0f})", (p[0] + 18, p[1] + 30), (255, 255, 255))
    if ref:
        for c, (x, y) in ref.items():
            col = DRAW_COLOR.get(c, (255, 255, 255))
            p = (int(x / coord_scale * k), int(y / coord_scale * k))
            cv2.drawMarker(out, p, col, cv2.MARKER_CROSS, 34, 2)
            _label(out, f"ref ({x:.0f}, {y:.0f})", (p[0] + 18, p[1] - 12), (255, 255, 255))
    cv2.putText(out, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(out, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
    for i, ln in enumerate(lines):          # ข้อมูล x, y ไว้มุมล่างซ้าย (มาร์กเกอร์อยู่มุมบน)
        _label(out, ln, (10, out.shape[0] - 10 - 22 * (len(lines) - 1 - i)), (255, 255, 255))
    return out


# ============================================================
# CAMERA CALIBRATION: ทำให้ภาพมีสเกลเท่ากันทั้งภาพ (1 mm = จำนวน pixel เท่ากันทุกตำแหน่ง)
#   ขั้น 1 เลนส์ : ถ่ายกระดานหมากรุกหลายมุม -> cv2.calibrateCamera -> K, dist (แก้เส้นตรงโค้ง)
#   ขั้น 2 ระนาบ: วางกระดานราบบนพื้นงาน 1 ภาพ -> Homography -> ทุกช่องเท่ากัน + รู้ px/mm
# ============================================================
LENS_FILE = "lens.npz"
RECTIFY_FILE = "rectify.npz"


def board_points(pattern, square):
    """พิกัดจริง (mm) ของมุมด้านในกระดาน เรียงแบบเดียวกับ findChessboardCorners"""
    c, r = pattern
    g = np.zeros((c * r, 3), np.float32)
    g[:, :2] = np.mgrid[0:c, 0:r].T.reshape(-1, 2) * square
    return g


def find_chessboard(gray, pattern):
    """หามุมด้านในของกระดานหมากรุก คืน array (N, 2) หรือ None"""
    ok, corners = cv2.findChessboardCornersSB(gray, pattern, cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not ok:
        ok, corners = cv2.findChessboardCorners(
            gray, pattern, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not ok:
            return None
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                   (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.01))
    return corners.reshape(-1, 2).astype(np.float32)


def square_stats(pts, pattern):
    """ขนาดช่อง (pixel) เฉลี่ยและส่วนเบี่ยงเบน จากระยะระหว่างมุมที่อยู่ติดกัน"""
    c, r = pattern
    g = pts.reshape(r, c, 2)
    d = np.concatenate([np.linalg.norm(np.diff(g, axis=1), axis=2).ravel(),
                        np.linalg.norm(np.diff(g, axis=0), axis=2).ravel()])
    return float(d.mean()), float(d.std())


def _orient(H_mm, w, h):
    """หมุน/กลับแกนของกระดาน ให้ภาพผลลัพธ์หันทางเดียวกับภาพกล้อง (ขวา = +x, ล่าง = +y)"""
    c = np.array([[[w / 2, h / 2]], [[w / 2 + 50, h / 2]], [[w / 2, h / 2 + 50]]], np.float32)
    m = cv2.perspectiveTransform(c, H_mm).reshape(-1, 2)
    vx, vy = m[1] - m[0], m[2] - m[0]
    best, bestP = -9, None
    for P in ([[1, 0], [0, 1]], [[0, 1], [1, 0]]):
        for sx in (1, -1):
            for sy in (1, -1):
                Q = np.array(P, float) * np.array([[sx], [sy]])
                a, b = Q @ vx, Q @ vy
                score = a[0] / np.linalg.norm(a) + b[1] / np.linalg.norm(b)
                if score > best:
                    best, bestP = score, Q
    P3 = np.eye(3)
    P3[:2, :2] = bestP
    return P3 @ H_mm


def build_rectify(img, pattern, square, K=None, dist=None, ppm=None, max_side=4000):
    """
    จากภาพกระดานที่วางราบบนพื้นงาน -> H (ภาพที่แก้เลนส์แล้ว -> ภาพผลลัพธ์), ขนาดผลลัพธ์, px/mm
    """
    h, w = img.shape[:2]
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if K is not None:
        gray = cv2.undistort(gray, K, dist)
    pts = find_chessboard(gray, pattern)
    if pts is None:
        return None
    obj = board_points(pattern, square)[:, :2]
    H_mm, _ = cv2.findHomography(pts, obj)                 # pixel -> mm บนพื้นงาน
    H_mm = _orient(H_mm, w, h)
    if ppm is None:                                        # คงความละเอียดใกล้ภาพเดิม
        ppm = round(square_stats(pts, pattern)[0] / square, 2)
    corners = np.array([[[0, 0]], [[w, 0]], [[w, h]], [[0, h]]], np.float32)
    mm = cv2.perspectiveTransform(corners, H_mm).reshape(-1, 2)
    x0, y0 = mm.min(0)
    x1, y1 = mm.max(0)
    ppm = min(ppm, max_side / max(x1 - x0, y1 - y0))
    H = np.array([[ppm, 0, -x0 * ppm], [0, ppm, -y0 * ppm], [0, 0, 1]]) @ H_mm
    out = (int(np.ceil((x1 - x0) * ppm)), int(np.ceil((y1 - y0) * ppm)))
    return {"H": H, "out": out, "ppm": float(ppm), "pts": pts}


def make_maps(H, out, K=None, dist=None):
    """ตาราง remap: pixel ผลลัพธ์ -> pixel ในภาพกล้องจริง (รวมการแก้เลนส์) ทำครั้งเดียว ใช้ซ้ำได้เร็ว"""
    W, Hh = out
    u, v = np.meshgrid(np.arange(W, dtype=np.float64), np.arange(Hh, dtype=np.float64))
    p = np.linalg.inv(H) @ np.stack([u.ravel(), v.ravel(), np.ones(u.size)])
    x, y = p[0] / p[2], p[1] / p[2]                        # พิกัดในภาพที่แก้เลนส์แล้ว
    if K is not None:                                      # -> พิกัดในภาพจริงที่ยังบิดเบี้ยว
        obj = np.stack([(x - K[0, 2]) / K[0, 0], (y - K[1, 2]) / K[1, 1], np.ones_like(x)], -1)
        ip, _ = cv2.projectPoints(obj.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, dist)
        x, y = ip[:, 0, 0], ip[:, 0, 1]
    return x.reshape(Hh, W).astype(np.float32), y.reshape(Hh, W).astype(np.float32)


# ============================================================
# PID แกนเดียว (ทำงานแบบ "ต่อรอบการถ่าย" : dt = 1 รอบ)
# ============================================================
class PID:
    def __init__(self, kp, ki, kd, i_limit):
        self.kp, self.ki, self.kd, self.i_limit = kp, ki, kd, i_limit
        self.reset()

    def reset(self):
        self.i = 0.0
        self.prev = None

    def update(self, err):
        self.i = float(np.clip(self.i + err, -self.i_limit, self.i_limit))
        d = 0.0 if self.prev is None else err - self.prev
        self.prev = err
        self.terms = (self.kp * err, self.ki * self.i, self.kd * d)   # (P, I, D) ไว้แสดงผล
        return sum(self.terms)                                       # หน่วย pixel


# ============================================================
# ESP32 (Serial, ส่งเป็นบรรทัดข้อความ, หน่วย steps)
# ============================================================
class ESP32:
    def __init__(self, port, baud):
        self.cur = None                    # ตำแหน่งล่าสุด (steps) อัปเดตสดระหว่างวิ่ง
        self.moving = False
        self.ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(2.0)                    # ESP32 รีเซ็ตตอนเปิดพอร์ต
        self.ser.reset_input_buffer()
        self.cmd("PING", ("PONG",), 3)
        log.info("ESP32 connected on %s", port)

    def cmd(self, line, wait=("OK",), timeout=10.0):
        self.ser.reset_input_buffer()
        self.ser.write((line + "\n").encode())
        self.moving = line.split()[0] in ("MOVE", "MOVR", "HOME")
        try:
            t0 = time.time()
            while time.time() - t0 < timeout:
                r = self.ser.readline().decode(errors="ignore").strip()
                if not r:
                    continue
                if r.startswith("P "):     # ตำแหน่งสดระหว่างวิ่ง
                    try:
                        self.cur = int(r.split()[1])
                    except (IndexError, ValueError):
                        pass
                    continue
                if r.startswith("ERR"):
                    raise RuntimeError(f"ESP32: {r} (cmd={line})")
                if r.startswith(wait):
                    if r.startswith(("DONE", "POS")):
                        self.cur = int(r.split()[1])
                    return r
            raise TimeoutError(f"ESP32 timeout: {line}")
        finally:
            self.moving = False

    @staticmethod
    def _x(r):
        return int(r.split()[1])

    def move_abs(self, x):
        return self._x(self.cmd(f"MOVE {int(x)}", ("DONE",), 60))

    def move_rel(self, dx):
        return self._x(self.cmd(f"MOVR {int(dx)}", ("DONE",), 60))

    def home(self):
        self.cur = None                    # ระหว่าง home ไม่รู้ตำแหน่ง
        return self._x(self.cmd("HOME", ("DONE",), 60))

    def pos(self):
        return self._x(self.cmd("POS?", ("POS",), 3))

    def emergency_stop(self):
        self.ser.write(b"STOP\n")          # เรียกจาก thread อื่นได้


# ============================================================
# CAMERA
# ============================================================
class Camera:
    def __init__(self, res, lens_position=None):
        self.cam = Picamera2()
        # "RGB888" ใน Picamera2 เรียง byte เป็น BGR -> ใช้กับ OpenCV ได้ทันที
        self.cam.configure(self.cam.create_still_configuration(
            main={"size": tuple(res), "format": "RGB888"}))
        self.cam.start()
        time.sleep(1.0)
        self.lock = threading.Lock()       # ภาพสดกับการถ่ายตรวจใช้กล้องตัวเดียวกัน
        self.manual = lens_position is not None
        self.has_af = controls is not None and "AfMode" in self.cam.camera_controls
        if self.has_af:
            if self.manual:
                self.cam.set_controls({"AfMode": controls.AfModeEnum.Manual,
                                       "LensPosition": float(lens_position)})
            else:
                self.cam.set_controls({"AfMode": controls.AfModeEnum.Auto})
                self.cam.autofocus_cycle()

    def capture(self, refocus=False):
        with self.lock:
            if refocus and self.has_af and not self.manual:
                self.cam.autofocus_cycle()
            return self.cam.capture_array("main")


# ============================================================
# PC LINK (TCP: [4 byte ความยาว header][JSON header][JPEG bytes])
# ============================================================
class PCLink:
    def __init__(self, host, port):
        self.addr = (host, port)
        self.sock = None
        self.lock = threading.Lock()
        self.cmds = queue.Queue()
        self.stop_flag = threading.Event()
        self.on_stop = None
        self.live_on = True                # เปิด/ปิดภาพสดจากคอม
        threading.Thread(target=self._rx_loop, daemon=True).start()

    @staticmethod
    def _recv_exact(s, n):
        buf = b""
        while len(buf) < n:
            chunk = s.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("PC closed")
            buf += chunk
        return buf

    def _rx_loop(self):
        while True:
            try:
                s = socket.create_connection(self.addr, timeout=5)
                s.settimeout(None)
                self.sock = s
                log.info("connected to PC %s:%d", *self.addr)
                while True:
                    (n,) = struct.unpack(">I", self._recv_exact(s, 4))
                    msg = json.loads(self._recv_exact(s, n))
                    if msg.get("cmd") == "stop":
                        self.stop_flag.set()
                        if self.on_stop:
                            self.on_stop()
                    elif msg.get("cmd") == "live":     # ทำทันที แม้ระหว่าง START
                        self.live_on = bool(msg.get("on", True))
                    else:
                        self.cmds.put(msg)
            except Exception:
                self.sock = None
                time.sleep(2)

    def send(self, header, img=None, quality=90):
        data = b""
        if img is not None:
            ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
            data = buf.tobytes()
        header["img_len"] = len(data)
        h = json.dumps(header).encode()
        s = self.sock
        if s is None:
            return
        try:
            with self.lock:
                s.sendall(struct.pack(">I", len(h)) + h + data)
        except OSError:
            self.sock = None

    def log(self, text):
        log.info(text)
        self.send({"type": "log", "text": text})


# ============================================================
# MACHINE (state machine หลัก)
# ============================================================
class Machine:
    def __init__(self):
        self.cfg = load_config()
        c = self.cfg
        self.pc = PCLink(c["pc_host"], c["pc_port"])
        self.esp = ESP32(c["serial_port"], c["baud"])
        self.pc.on_stop = self.esp.emergency_stop
        self.cam = Camera(c["resolution"], c["lens_position"])
        p = c["pid"]
        self.pid = PID(p["kp"], p["ki"], p["kd"], p["i_limit"])
        self.live_ref = None               # ตำแหน่งอ้างอิงที่วาดกากบาทบนภาพสด
        self.live_name = ""
        self.live_st = None                # จุดที่กำลังดูอยู่ (ใช้เกณฑ์ความชัดของจุดนั้น)
        self.lens_shots = []               # มุมกระดานจากปุ่ม "ถ่ายภาพกระดาน"
        self.lens_size = None
        self._rect = None                  # (mapx, mapy, ppm) โหลดตอนใช้ครั้งแรก
        self._rect_loaded = False
        threading.Thread(target=self._live_loop, daemon=True).start()

    # ---------- ภาพสด: ถ่ายต่อเนื่องแล้วส่งไปคอม ----------
    def _live_loop(self):
        while True:
            t0 = time.time()
            if self.pc.sock is None or not self.pc.live_on:
                time.sleep(0.2)
                continue
            try:
                img = self.cam.capture()
                w = self.cfg["live_width"]
                s = w / img.shape[1]
                small = cv2.resize(img, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
                cfg_s = dict(self.cfg, min_area_px=self.cfg["min_area_px"] * s * s,
                             max_area_px=(self.cfg.get("max_area_px", 0) or 0) * s * s)
                found = detect_markers(small, cfg_s)
                ref = self.live_ref
                found_full = {k: (v[0] / s, v[1] / s) for k, v in found.items()}
                sv = roi_sharpness(img, sharp_center(found_full, ref, img.shape),
                                   self.roi_half())               # ค่าเดียวกับตอนถ่ายตรวจ
                # บรรทัดข้อมูล: ตำแหน่งมาร์กเกอร์ของจุดนี้ เทียบกับตอน Teach
                lines = []
                if ref:
                    c0 = next(iter(ref))
                    rx, ry = ref[c0]
                    if c0 in found_full:
                        fx_, fy_ = found_full[c0]
                        dx, dy = rx - fx_, ry - fy_
                        along, perp = self.split_err(np.array([dx, dy]))
                        ok_a = abs(along) <= self.cfg["pos_tolerance_px"]
                        ok_p = abs(perp) <= self.cfg["perp_max_px"]
                        lines = [f"{c0}: x={fx_:.0f} y={fy_:.0f}   ref: x={rx:.0f} y={ry:.0f}",
                                 f"dx={dx:+.0f} dy={dy:+.0f}  ->  along={along:+.1f}px "
                                 f"{'OK' if ok_a else 'BAD'}   perp={perp:+.1f}px {'OK' if ok_p else 'BAD'}"]
                    else:
                        lines = [f"{c0}: not found   ref: x={rx:.0f} y={ry:.0f}"]

                cur = self.esp.cur
                pos_txt = "-- cm" if cur is None else f"{self.s2cm(cur):6.2f} cm"
                state = "MOVING" if self.esp.moving else "STOP"
                thr = self.blur_thr(self.live_st)
                sharp = "OK" if sv >= thr else "BLUR"
                text = (f"LIVE {pos_txt} [{state}]  sharp={sv:.1f}/{thr:.1f} {sharp}  "
                        f"seen={'+'.join(found) or '-'}  {self.live_name}")
                hdr = {"type": "live", "text": text, "moving": self.esp.moving}
                if cur is not None:
                    hdr.update(steps=cur, cm=self.s2cm(cur))
                self.pc.send(hdr, annotate(small, found, ref, text, width=w,
                                           coord_scale=1 / s, lines=lines), 70)
            except Exception as e:
                log.warning("live: %s", e)
                time.sleep(1)
            time.sleep(max(0.0, 1.0 / self.cfg["live_fps"] - (time.time() - t0)))

    # ---------- แปลงหน่วย ----------
    def cm2s(self, cm):
        return int(round(float(cm) * self.cfg["steps_per_cm"]))

    def s2cm(self, s):
        return s / self.cfg["steps_per_cm"]

    def axis(self):
        """(ทิศของแกนในภาพแบบ unit vector, pixel ต่อ 1 step) หรือ (None, None) ถ้ายังไม่ calibrate"""
        j = self.cfg.get("px_per_step")
        if not j:
            return None, None
        j = np.array(j, float)
        n = float(np.linalg.norm(j))
        return j / n, n

    def roi_half(self):
        """ครึ่งความกว้างกรอบวัดความคม (คงที่ ไม่ขึ้นกับ Calibrate -> ค่าตอน Teach กับตอน START เทียบกันได้)"""
        return int(self.cfg["sharp_roi_px"])

    def blur_thr(self, st):
        """เกณฑ์ความชัดของจุดนี้: 70% ของค่าตอน Teach (สีเหลืองได้ค่าต่ำกว่าสีเข้มเป็นธรรมชาติ)"""
        ref = st.get("sharp_ref") if st else None
        if ref:
            return ref * self.cfg["blur_ratio"]
        return self.cfg["blur_threshold"]

    def split_err(self, err):
        """แยก error เป็น along (แก้ได้ด้วยมอเตอร์) และ perp (แก้ไม่ได้)"""
        if err is None:
            return None, None
        u, _ = self.axis()
        if u is None:                      # ยังไม่ calibrate -> ใช้ขนาดรวม
            return float(np.linalg.norm(err)), 0.0
        along = float(err @ u)
        perp = float(err[0] * u[1] - err[1] * u[0])
        return along, perp

    # ---------- ถ่าย + วิเคราะห์ ----------
    def measure(self, ref, refocus=False):
        time.sleep(self.cfg["settle_time_s"])
        img = self.cam.capture(refocus)
        found = detect_markers(img, self.cfg)
        sv = roi_sharpness(img, sharp_center(found, ref, img.shape), self.roi_half())
        err, n = position_error(found, ref)
        return img, sv, found, err, n

    # ---------- ถ่าย 1 ตำแหน่ง ตามเงื่อนไข 4 กรณี ----------
    def run_station(self, i):
        c = self.cfg
        st = c["stations"][i]
        ref, name = st["ref"], st["name"]
        tol = c["pos_tolerance_px"]
        u, pps = self.axis()
        target = st.get("color")
        if ref is None:
            self.pc.log(f"[{name}] ยังไม่ได้ Teach -> เช็คแค่ว่าเห็นมาร์กเกอร์อย่างน้อย 1 สี")

        self.live_ref, self.live_name, self.live_st = ref, f"-> {name} ({target or '?'})", st
        thr = self.blur_thr(st)
        self.esp.move_abs(self.cm2s(st["cm"]))
        self.pid.reset()
        blur_retry = pid_iter = shot = 0
        refocus = False
        warned = False
        sv = along = perp = None
        p = c["pid"]
        self.pc.send({"type": "pid", "event": "start", "station": i, "name": name,
                      "color": target, "tol": tol, "kp": p["kp"], "ki": p["ki"], "kd": p["kd"],
                      "px_per_cm": (pps or 0) * c["steps_per_cm"]})

        def rec(decision, **extra):
            """ส่งข้อมูลรอบนี้ไปแสดงในแท็บ PID บนคอม"""
            cur = self.esp.cur
            self.pc.send({"type": "pid", "event": "shot", "station": i, "name": name,
                          "shot": shot, "pid_iter": pid_iter, "err": along, "perp": perp,
                          "sharp": sv, "thr": thr, "pos_cm": None if cur is None else self.s2cm(cur),
                          "decision": decision, **extra})

        while not self.pc.stop_flag.is_set():
            img, sv, found, err, n = self.measure(ref, refocus)
            shot += 1
            refocus = False
            sharp = sv >= thr
            along, perp = self.split_err(err)
            perp_bad = (ref is not None and perp is not None
                        and abs(perp) > c["perp_max_px"])
            if ref is None:
                ok = len(found) >= 1
            else:
                ok = n >= len(ref) and abs(along) <= tol and not perp_bad

            e_txt = "-" if along is None else f"{along:+.1f}px (perp {perp:+.1f})"
            status = (f"{name}[{target or '?'}] sharp={sv:.1f}/{thr:.1f} {'OK' if sharp else 'BLUR'} "
                      f"seen={'+'.join(found) or '-'} err={e_txt} {'POS_OK' if ok else 'POS_BAD'}")
            info = {"station": i, "name": name, "sharp": sv,
                    "markers": {k: list(v) for k, v in found.items()},
                    "err": None if err is None else err.tolist(),
                    "err_along": along, "err_perp": perp, "pos_ok": ok}
            lines = []
            if ref is not None and target in found:
                fx_, fy_ = found[target]
                rx, ry = ref[target]
                lines = [f"{target}: x={fx_:.0f} y={fy_:.0f}   ref: x={rx:.0f} y={ry:.0f}",
                         f"dx={rx - fx_:+.0f} dy={ry - fy_:+.0f}  ->  along={along:+.1f}px  perp={perp:+.1f}px"]
            self.pc.send({"type": "frame", "text": status, **info},
                         annotate(img, found, ref, status, lines=lines), 80)
            log.info(status)

            # คลาดขวางราง: มอเตอร์แกนเดียวแก้ไม่ได้
            if perp_bad:
                rec("FAIL ขวางราง")
                self.pc.log(f"[{name}] ไม่ผ่าน: มาร์กคลาดขวางราง {perp:+.1f}px "
                            f"(เกิน {c['perp_max_px']:.0f}px) มอเตอร์แก้ไม่ได้ "
                            f"-> ตรวจว่ากระดาษ/ชิ้นงานถูกเลื่อนหรือไม่")
                return False
            if (ref is not None and perp is not None and not warned
                    and abs(perp) > c["perp_warn_px"]):
                self.pc.log(f"[{name}] เตือน: มาร์กคลาดขวางราง {perp:+.1f}px "
                            f"(ยังไม่เกิน {c['perp_max_px']:.0f}px)")
                warned = True

            # กรณี 1: ชัด + ตำแหน่งถูก -> เก็บภาพ ไปตำแหน่งถัดไป
            if sharp and ok:
                rec("CAPTURE")
                if c.get("rectify_captures", True) and self.px_per_mm():
                    out = self.rectify(img)                # ภาพที่บันทึก: สเกลเท่ากันทั้งภาพ
                    info["px_per_mm"] = self.px_per_mm()
                else:
                    out = img
                self.pc.send({"type": "capture", **info}, out, 95)
                return True

            # กรณี 2: ไม่ชัด + ตำแหน่งถูก -> ถ่ายใหม่ ไม่ขยับมอเตอร์
            # (รวมกรณีตำแหน่งตรงแต่เห็นมาร์กไม่ครบ / หามาร์กไม่เจอเพราะเบลอ)
            near = along is not None and abs(along) <= tol
            if ok or near or (err is None and not sharp):
                blur_retry += 1
                if blur_retry > c["max_blur_retry"]:
                    rec("FAIL เบลอ")
                    self.pc.log(f"[{name}] ถ่ายซ้ำครบ {c['max_blur_retry']} ครั้งแล้วยังไม่ผ่าน")
                    return False
                rec("ถ่ายซ้ำ (เบลอ)")
                refocus = True
                continue

            if err is None:
                rec("FAIL ไม่เจอมาร์ก")
                self.pc.log(f"[{name}] ภาพชัดแต่ไม่เจอมาร์กเกอร์สี {target} "
                            f"(เห็น: {', '.join(found) or 'ไม่มี'}) -> ตรวจตำแหน่ง/สอนสีใหม่")
                return False
            if u is None:
                rec("FAIL ยังไม่ Calibrate")
                self.pc.log(f"[{name}] ตำแหน่งคลาด แต่ยังไม่ได้ Calibrate -> กด Calibrate ก่อน")
                return False

            # กรณี 3 และ 4: ตำแหน่งผิด -> PID ขยับก่อน แล้วถ่ายใหม่
            pid_iter += 1
            if pid_iter > c["max_pid_iter"]:
                rec("FAIL PID ไม่ลู่เข้า")
                self.pc.log(f"[{name}] PID ไม่ลู่เข้าใน {c['max_pid_iter']} รอบ")
                return False
            corr_px = self.pid.update(along)
            lim = self.cm2s(c["max_correction_cm"])
            steps = int(np.clip(round(corr_px / pps), -lim, lim))
            if steps == 0:                 # ต้องขยับอย่างน้อย 1 step
                steps = 1 if corr_px > 0 else -1
            P_, I_, D_ = self.pid.terms
            rec("PID ขยับ" + ("" if sharp else " (+เบลอ)"), p=P_, i=I_, d=D_, out=corr_px,
                steps=steps, move_cm=self.s2cm(steps))
            self.pc.log(f"[{name}] PID#{pid_iter} err={along:+.1f}px -> move {steps:+d} steps "
                        f"({self.s2cm(steps):+.2f} cm)")
            self.esp.move_rel(steps)
            refocus = not sharp            # กรณี 4: เคยเบลอ -> โฟกัสใหม่หลังขยับ
        return False

    def run_all(self):
        self.pc.stop_flag.clear()
        if self.cfg["home_before_run"]:
            self.esp.home()
        results = []
        for i in range(len(self.cfg["stations"])):
            if self.pc.stop_flag.is_set():
                self.pc.log("STOP โดยผู้ใช้")
                break
            results.append(self.run_station(i))
        self.pc.send({"type": "done", "results": results})

    # ---------- calibrate: หาว่าขยับ 1 step แล้วมาร์กในภาพเลื่อนไปทางไหน กี่ pixel ----------
    def calibrate(self, station=0):
        c = self.cfg
        st = c["stations"][station]
        start = self.cm2s(st["cm"])
        S = self.cm2s(c["calib_cm"])
        if start + S > self.cm2s(c["travel_cm"]):
            S = -S                         # ใกล้ปลายราง -> calibrate ถอยหลังแทน
        self.esp.move_abs(start)
        f0 = self.measure(None)[2]
        self.esp.move_rel(S)
        f1 = self.measure(None)[2]
        self.esp.move_rel(-S)
        common = [k for k in f0 if k in f1]
        if not common and start - S >= 0:
            # มาร์กเกอร์อยู่ใกล้ขอบภาพแล้วหลุดออกไป -> ลองขยับอีกทิศ
            self.pc.log("calibrate: มาร์กเกอร์หลุดขอบภาพ -> ลองขยับอีกทิศ")
            S = -S
            self.esp.move_rel(S)
            f1 = self.measure(None)[2]
            self.esp.move_rel(-S)
            common = [k for k in f0 if k in f1]
        if not common:
            raise RuntimeError("calibrate: ไม่เห็นมาร์กเกอร์สีเดียวกันทั้งก่อนและหลังขยับ "
                               "-> Teach P1 ก่อน และให้มาร์กเกอร์อยู่กลางภาพ")
        j = np.mean([np.subtract(f1[k], f0[k]) for k in common], axis=0) / S
        pps = float(np.linalg.norm(j))
        if pps * abs(S) < 5:
            raise RuntimeError("calibrate: มาร์กเกอร์แทบไม่ขยับในภาพ -> มอเตอร์หมุนจริงไหม?")
        c["px_per_step"] = j.tolist()
        save_config(c)
        u = j / pps
        self.pc.log(f"calibrate OK: 1 step = {pps:.2f} px  (1 cm = {pps * c['steps_per_cm']:.0f} px)  "
                    f"ทิศแกนในภาพ = ({u[0]:+.2f}, {u[1]:+.2f})")
        if c["pos_tolerance_px"] < 0.6 * pps:
            self.pc.log(f"เตือน: pos_tolerance_px ({c['pos_tolerance_px']}) เล็กกว่าระยะ 1 step "
                        f"-> ควรตั้งอย่างน้อย {0.6 * pps:.1f}")

    # ---------- teach: บันทึกตำแหน่งปัจจุบันเป็นตำแหน่งอ้างอิง ----------
    def teach(self, i):
        st = self.cfg["stations"][i]
        st["cm"] = round(self.s2cm(self.esp.pos()), 2)
        # มาร์กเกอร์ของจุดนี้: ใกล้ "ตำแหน่งมาร์กเกอร์ของจุดอื่นที่ Teach แล้ว" ที่สุด
        # (ทุกจุดวางมาร์กเกอร์ไว้มุมเดียวกันของภาพ)  ถ้ายังไม่มีจุดไหน Teach -> ใกล้กลางภาพที่สุด
        anchor = None
        for s in self.cfg["stations"]:
            if s is not st and s.get("ref"):
                anchor = next(iter(s["ref"].values()))
                break
        # ถ่าย 3 ภาพ เก็บภาพที่คมที่สุด (กันภาพเบลอจากการสั่นมาเป็นค่าอ้างอิง)
        best = None
        for _ in range(3):
            img, _, found, _, _ = self.measure(None, refocus=best is None)
            if not found:
                continue
            h, w = img.shape[:2]
            ax, ay = anchor if anchor else (w / 2, h / 2)
            color = min(found, key=lambda k: np.hypot(found[k][0] - ax, found[k][1] - ay))
            sv = roi_sharpness(img, found[color], self.roi_half())
            if best is None or sv > best[1]:
                best = (img, sv, found, color)
        if best is None:
            self.pc.log(f"Teach {st['name']} ไม่สำเร็จ: ไม่เห็นมาร์กเกอร์เลย -> สอนสีก่อน")
            return
        img, sv, found, color = best
        x, y = found[color]
        st["color"] = color
        st["ref"] = {color: [round(x, 1), round(y, 1)]}
        st["sharp_ref"] = round(sv, 2)             # ความคมตอน Teach = ค่าอ้างอิงของจุดนี้
        save_config(self.cfg)
        dup = [s["name"] for s in self.cfg["stations"] if s is not st and s.get("color") == color]
        if dup:
            self.pc.log(f"เตือน: สี {color} ถูกใช้กับ {', '.join(dup)} แล้ว -> แต่ละจุดควรใช้คนละสี")
        self.live_ref, self.live_name, self.live_st = st["ref"], f"{st['name']} ({color})", st
        thr = self.blur_thr(st)
        text = f"TEACH {st['name']} = {st['cm']:.2f} cm  color={color}  sharp={sv:.1f}"   # OpenCV เขียนไทยไม่ได้
        self.pc.send({"type": "frame", "text": text}, annotate(img, found, st["ref"], text), 80)
        self.pc.log(f"TEACH {st['name']} = {st['cm']:.2f} cm  สี {color}  sharp={sv:.1f} "
                    f"-> เกณฑ์ผ่านของจุดนี้ = {thr:.1f}")

    def preview(self):
        img, sv, found, _, n = self.measure(None)
        text = f"PREVIEW sharp={sv:.0f} seen={'+'.join(found) or '-'}"
        self.pc.send({"type": "frame", "text": text,
                      "markers": {k: list(v) for k, v in found.items()}},
                     annotate(img, found, None, text), 80)

    # ---------- สอนสี: จิ้มที่กระดาษสีบนภาพ แล้วตั้งช่วง HSV ของสีนั้นให้อัตโนมัติ ----------
    def pick_color(self, color, fx, fy):
        if color not in self.cfg["colors"]:
            raise ValueError(f"ไม่มีสี {color} (มี {list(self.cfg['colors'])})")
        img = self.cam.capture()
        h, w = img.shape[:2]
        x, y = int(fx * w), int(fy * h)
        r = max(5, w // 200)                       # จุดตัวอย่างขนาด ~2r x 2r pixel
        patch = img[max(0, y - r):y + r, max(0, x - r):x + r]
        if patch.size == 0:
            raise ValueError("จุดที่คลิกอยู่นอกภาพ")
        hsv_full = to_hsv(img)                     # ปรับแสงแบบเดียวกับตอนหามาร์กเกอร์
        patch = hsv_full[max(0, y - r):y + r, max(0, x - r):x + r]
        hsv_px = np.median(patch.reshape(-1, 3), axis=0)
        hsv_txt = f"HSV = ({hsv_px[0]:.0f}, {hsv_px[1]:.0f}, {hsv_px[2]:.0f})"
        if hsv_px[1] < 40:
            self.pc.log(f"สอนสี {color} ไม่สำเร็จ: จุดที่คลิกเป็นสีขาว/เทา/ดำ ({hsv_txt}) "
                        f"-> คลิกให้โดนกลางกระดาษสี")
            return
        samples = dict(self.cfg.get("color_samples") or {})
        samples[color] = [round(float(x), 1) for x in hsv_px]
        old_colors = json.loads(json.dumps(self.cfg["colors"]))
        self.cfg["colors"].update(ranges_from_samples(samples))   # สร้างช่วงใหม่ทุกสีที่สอนแล้ว
        found = detect_markers(img, self.cfg)
        if color not in found:
            self.cfg["colors"] = old_colors           # ไม่เจอเป็นแผ่นสี่เหลี่ยม -> ใช้ค่าเดิม
            self.pc.log(f"สอนสี {color} ไม่สำเร็จ: {hsv_txt} แต่ไม่เจอเป็นแผ่นสี่เหลี่ยม "
                        f"-> ใช้ค่าเดิม (ลองคลิกกลางแผ่น หรือปรับแสง/min_area_px)")
            return
        self.cfg["color_samples"] = samples
        save_config(self.cfg)
        x0, y0 = found[color]
        self.pc.log(f"สอนสี {color} สำเร็จ: {hsv_txt} -> เจอที่ ({x0:.0f}, {y0:.0f})   "
                    f"ตอนนี้เห็น: {', '.join(found)}")

    # ---------- ปรับภาพให้สเกลเท่ากันทั้งภาพ ----------
    def calib_msg(self, text):
        self.pc.log(text)
        self.pc.send({"type": "calib", "text": text})

    def pattern(self):
        return (int(self.cfg["chess_cols"]), int(self.cfg["chess_rows"]))

    def load_lens(self):
        if os.path.exists(LENS_FILE):
            d = np.load(LENS_FILE)
            return d["K"], d["dist"]
        return None, None

    def lens_shot(self):
        img = self.cam.capture()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        pts = find_chessboard(gray, self.pattern())
        vis = img.copy()
        if pts is None:
            self.calib_msg(f"ไม่เจอกระดาน {self.pattern()[0]}x{self.pattern()[1]} มุม "
                           "-> ให้เห็นกระดานเต็มแผ่น ไม่มีเงา/แสงสะท้อน")
        else:
            self.lens_shots.append(pts)
            self.lens_size = (img.shape[1], img.shape[0])
            cv2.drawChessboardCorners(vis, self.pattern(), pts.reshape(-1, 1, 2), True)
            n = len(self.lens_shots)
            self.calib_msg(f"ภาพกระดานที่ {n} ใช้ได้ "
                           f"({'ถ่ายต่อให้ได้ 12-20 ภาพ' if n < 12 else 'พอแล้ว กด คำนวณเลนส์ ได้'})")
        self.pc.send({"type": "frame", "text": f"LENS SHOT {len(self.lens_shots)}"},
                     annotate(vis, {}, None, f"LENS SHOT {len(self.lens_shots)}"), 80)

    def lens_solve(self):
        n = len(self.lens_shots)
        if n < 8:
            self.calib_msg(f"มีภาพกระดาน {n} ภาพ ต้องอย่างน้อย 8 (แนะนำ 12-20)")
            return
        objp = board_points(self.pattern(), float(self.cfg["chess_square_mm"]))
        rms, K, dist, _, _ = cv2.calibrateCamera(
            [objp] * n, [p.reshape(-1, 1, 2) for p in self.lens_shots], self.lens_size, None, None)
        np.savez(LENS_FILE, K=K, dist=dist, rms=rms, size=np.array(self.lens_size))
        q = "ดีมาก" if rms < 0.5 else "ใช้ได้" if rms < 1.0 else "ไม่ดี -> ล้างแล้วถ่ายใหม่"
        self.calib_msg(f"คำนวณเลนส์เสร็จ: ค่าคลาด (RMS) = {rms:.3f} px ({q})  "
                       f"k1={dist.ravel()[0]:+.3f}  -> ต่อไป: วางกระดานราบบนพื้นงานแล้วกด ตั้งสเกล")

    def lens_reset(self):
        self.lens_shots = []
        self.calib_msg("ล้างภาพกระดานแล้ว")

    def plane_calib(self):
        K, dist = self.load_lens()
        img = self.cam.capture()
        sq = float(self.cfg["chess_square_mm"])
        r = build_rectify(img, self.pattern(), sq, K, dist)
        if r is None:
            self.calib_msg("ตั้งสเกลไม่สำเร็จ: ไม่เจอกระดาน -> วางกระดานราบบนพื้นงาน ให้เห็นเต็มแผ่น")
            return
        np.savez(RECTIFY_FILE, H=r["H"], out=np.array(r["out"]), ppm=r["ppm"],
                 K=K if K is not None else np.zeros(0), dist=dist if dist is not None else np.zeros(0))
        self._rect_loaded = False
        out = self.rectify(img)
        # ตรวจผล: หากระดานในภาพที่ปรับแล้ว ช่องต้องเท่ากันทุกช่อง
        pts2 = find_chessboard(cv2.cvtColor(out, cv2.COLOR_BGR2GRAY), self.pattern())
        if pts2 is not None:
            m, s = square_stats(pts2, self.pattern())
            chk = f"ตรวจผล: ช่องตาราง = {m:.2f} ± {s:.2f} px = {m / r['ppm']:.2f} mm (จริง {sq:g} mm)"
        else:
            chk = "ตรวจผล: หากระดานในภาพที่ปรับแล้วไม่เจอ"
        lens_txt = "แก้เลนส์ + มุมกล้อง" if K is not None else "แก้มุมกล้อง (ยังไม่ได้แก้เลนส์)"
        self.calib_msg(f"ตั้งสเกลเสร็จ ({lens_txt}): 1 mm = {r['ppm']:.2f} px  "
                       f"ภาพผลลัพธ์ {r['out'][0]}x{r['out'][1]}  |  {chk}")
        vis = out.copy()
        step = r["ppm"] * 10                                   # เส้นตารางทุก 10 mm
        for x in np.arange(0, vis.shape[1], step):
            cv2.line(vis, (int(x), 0), (int(x), vis.shape[0]), (0, 200, 0), 1)
        for y in np.arange(0, vis.shape[0], step):
            cv2.line(vis, (0, int(y)), (vis.shape[1], int(y)), (0, 200, 0), 1)
        self.pc.send({"type": "frame", "text": "RECTIFIED (เส้นเขียวทุก 10 mm)"},
                     annotate(vis, {}, None, f"RECTIFIED  1 mm = {r['ppm']:.2f} px  (grid 10 mm)"), 85)

    def _load_rect(self):
        if self._rect_loaded:
            return
        self._rect_loaded = True
        self._rect = None
        if os.path.exists(RECTIFY_FILE):
            d = np.load(RECTIFY_FILE)
            K = d["K"] if d["K"].size else None
            dist = d["dist"] if d["dist"].size else None
            mx, my = make_maps(d["H"], tuple(int(v) for v in d["out"]), K, dist)
            self._rect = (mx, my, float(d["ppm"]))

    def rectify(self, img):
        """ภาพกล้อง -> ภาพที่แก้เลนส์และมุมกล้องแล้ว (สเกลเท่ากันทั้งภาพ)  ไม่มีไฟล์ -> คืนภาพเดิม"""
        self._load_rect()
        if self._rect is None:
            return img
        mx, my, _ = self._rect
        return cv2.remap(img, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    def px_per_mm(self):
        self._load_rect()
        return self._rect[2] if self._rect else None

    def send_pos(self):
        p = self.esp.pos()
        self.pc.send({"type": "pos", "steps": p, "cm": self.s2cm(p)})

    # ---------- รับคำสั่งจากคอม ----------
    def loop(self):
        self.pc.log("Pi5 ready")
        if self.cfg.pop("_note", None) == "sharp_reset":
            save_config(self.cfg)
            time.sleep(1.0)
            self.pc.log("เปลี่ยนวิธีวัดความคมแล้ว: ตำแหน่ง/สีที่ Teach ไว้ยังอยู่ "
                        "-> แนะนำ Goto + Teach P1-P4 ใหม่ 1 รอบ เพื่อตั้งเกณฑ์ความคมของแต่ละจุด")
        while True:
            msg = self.pc.cmds.get()
            cmd = msg.get("cmd")
            try:
                if cmd == "home":
                    self.live_ref, self.live_name, self.live_st = None, "", None
                    self.esp.home()
                elif cmd == "start":
                    self.run_all()
                elif cmd == "calibrate":
                    self.calibrate(int(msg.get("station", 0)))
                elif cmd == "teach":
                    self.teach(int(msg["station"]))
                elif cmd == "goto":
                    st = self.cfg["stations"][int(msg["station"])]
                    self.live_ref, self.live_name, self.live_st = (
                        st["ref"], f"-> {st['name']} ({st.get('color') or '?'})", st)
                    self.esp.move_abs(self.cm2s(st["cm"]))
                elif cmd == "jog":
                    self.esp.move_rel(self.cm2s(msg["dcm"]))
                elif cmd == "preview":
                    self.preview()
                elif cmd == "pick_color":
                    self.pick_color(msg["color"], float(msg["fx"]), float(msg["fy"]))
                elif cmd == "lens_shot":
                    self.lens_shot()
                elif cmd == "lens_solve":
                    self.lens_solve()
                elif cmd == "lens_reset":
                    self.lens_reset()
                elif cmd == "plane_calib":
                    self.plane_calib()
                elif cmd == "set":
                    self.cfg.update(msg["values"])
                    save_config(self.cfg)
                    self.pc.log(f"config updated: {list(msg['values'])}")
                if self.esp.cur is not None:
                    self.send_pos()
            except Exception as e:
                self.pc.log(f"ERROR {cmd}: {e}")


if __name__ == "__main__":
    Machine().loop()