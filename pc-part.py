#!/usr/bin/env python3
"""
pc_station.py  —  รันบนคอม  ("หน้าจอควบคุม + ที่เก็บภาพ")

หน้าที่:
  - เปิด TCP server ให้ Pi5 ต่อเข้ามา
  - แสดงภาพ live พร้อมจุดมาร์กเกอร์/ค่าความชัด/error
  - ปุ่มสั่งงาน: Home, Calibrate, Start, STOP, Preview, Jog, Goto, Teach
  - บันทึกภาพสุดท้ายของแต่ละตำแหน่ง + log ผลเป็น CSV
  - จุดต่อยอด: process_results() สำหรับประมวลผลภาพทั้ง 4 ภาพหลังถ่ายเสร็จ

ติดตั้ง:
  pip install numpy opencv-python pillow
"""
import csv
import json
import os
import queue
import socket
import struct
import threading
import time
import tkinter as tk
from tkinter import ttk

import cv2
import numpy as np
from PIL import Image, ImageTk

HOST, PORT = "0.0.0.0", 5000
SAVE_DIR = "captures"
STATIONS = ["P1", "P2", "P3", "P4"]


# ============================================================
# TCP SERVER (โปรโตคอลเดียวกับ Pi: [4 byte len][JSON header][JPEG])
# ============================================================
class Server:
    def __init__(self, inbox):
        self.inbox = inbox
        self.conn = None
        self.lock = threading.Lock()
        threading.Thread(target=self._serve, daemon=True).start()

    @staticmethod
    def _recv_exact(s, n):
        buf = b""
        while len(buf) < n:
            chunk = s.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("closed")
            buf += chunk
        return buf

    def _serve(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((HOST, PORT))
        srv.listen(1)
        while True:
            conn, addr = srv.accept()
            self.conn = conn
            self.inbox.put(({"type": "log", "text": f"Pi connected from {addr[0]}"}, None, b""))
            try:
                while True:
                    (n,) = struct.unpack(">I", self._recv_exact(conn, 4))
                    hdr = json.loads(self._recv_exact(conn, n))
                    data, img = b"", None
                    if hdr.get("img_len", 0) > 0:
                        data = self._recv_exact(conn, hdr["img_len"])
                        img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                    self.inbox.put((hdr, img, data))
            except Exception as e:
                self.conn = None
                self.inbox.put(({"type": "log", "text": f"Pi disconnected ({e})"}, None, b""))

    def send(self, msg):
        c = self.conn
        if c is None:
            return False
        h = json.dumps(msg).encode()
        with self.lock:
            c.sendall(struct.pack(">I", len(h)) + h)
        return True


# ============================================================
# ประมวลผลหลังถ่ายครบ (ใส่งานของโปรเจกต์ตรงนี้)
# ============================================================
def process_results(folder):
    """
    ตัวอย่าง: อ่านภาพทั้ง 4 ภาพมาทำต่อ เช่น ตรวจชิ้นงาน วัดขนาด ต่อภาพ (stitch) ฯลฯ
    """
    files = sorted(f for f in os.listdir(folder) if f.endswith(".jpg"))
    return f"มีภาพ {len(files)} ภาพใน {folder}"


# ============================================================
# GUI
# ============================================================
class App:
    def __init__(self, root):
        self.root = root
        root.title("Camera Gantry — PC Station")
        self.inbox = queue.Queue()
        self.server = Server(self.inbox)
        self.session_dir = None
        self.tkimg = None

        # ภาพ
        self.blank = tk.PhotoImage(width=960, height=540)   # ภาพว่างขนาด 960x540 pixel
        self.img_label = tk.Label(root, bg="black", image=self.blank, bd=0, highlightthickness=0)
        self.img_label.grid(row=0, column=0, padx=6, pady=(6, 0), sticky="n")
        self.img_label.bind("<Button-1>", self.on_image_click)
        self.pick_armed = False

        # แผงควบคุม
        panel = ttk.Frame(root)
        panel.grid(row=0, column=1, rowspan=2, sticky="n", padx=6, pady=6)
        for txt, cmd in [("Home", "home"), ("Calibrate", "calibrate"),
                         ("Preview", "preview"), ("START", "start")]:
            ttk.Button(panel, text=txt, command=lambda c=cmd: self.send({"cmd": c})).pack(fill="x", pady=2)
        tk.Button(panel, text="STOP", bg="#c62828", fg="white",
                  command=lambda: self.send({"cmd": "stop"})).pack(fill="x", pady=6)

        # station
        ttk.Label(panel, text="Station").pack(anchor="w", pady=(10, 0))
        self.station = ttk.Combobox(panel, values=STATIONS, state="readonly", width=8)
        self.station.current(0)
        self.station.pack(fill="x")
        ttk.Button(panel, text="Goto", command=lambda: self.send(
            {"cmd": "goto", "station": self.station.current()})).pack(fill="x", pady=2)
        ttk.Button(panel, text="Teach (บันทึกจุดนี้)", command=lambda: self.send(
            {"cmd": "teach", "station": self.station.current()})).pack(fill="x", pady=2)

        # jog (cm)
        ttk.Label(panel, text="Jog (cm)").pack(anchor="w", pady=(10, 0))
        self.jog = tk.DoubleVar(value=1.0)
        ttk.Entry(panel, textvariable=self.jog, width=8).pack(fill="x")
        jf = ttk.Frame(panel)
        jf.pack(pady=4)
        ttk.Button(jf, text="◀ เข้าหา Home", width=12, command=lambda: self.do_jog(-1)).grid(row=0, column=0)
        ttk.Button(jf, text="ออกจาก Home ▶", width=12, command=lambda: self.do_jog(1)).grid(row=0, column=1)

        # สถานะแต่ละ station
        self.st_labels = []
        sf = ttk.Frame(panel)
        sf.pack(pady=10, fill="x")
        for i, s in enumerate(STATIONS):
            l = tk.Label(sf, text=s, width=5, bg="#9e9e9e", fg="white")
            l.grid(row=0, column=i, padx=1)
            self.st_labels.append(l)
        self.pos_var = tk.StringVar(value="pos: -")
        ttk.Label(panel, textvariable=self.pos_var, font=("Consolas", 12, "bold")).pack(anchor="w")

        # สอนสี: เลือกสี -> กดปุ่ม -> คลิกกลางกระดาษสีนั้นบนภาพสด
        ttk.Label(panel, text="สอนสีมาร์กเกอร์").pack(anchor="w", pady=(10, 0))
        cf = ttk.Frame(panel)
        cf.pack(fill="x")
        self.color_names = {"ม่วง (purple)": "purple", "น้ำตาล (brown)": "brown",
                            "เหลือง (yellow)": "yellow", "ส้ม (orange)": "orange"}
        self.color_box = ttk.Combobox(cf, values=list(self.color_names), state="readonly", width=14)
        self.color_box.current(0)
        self.color_box.pack(side="left")
        self.pick_btn = ttk.Button(cf, text="คลิกบนภาพ", command=self.arm_pick)
        self.pick_btn.pack(side="left", padx=4)

        # เปิด/ปิดภาพสด
        self.live_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(panel, text="Live view (ภาพสด)", variable=self.live_var,
                        command=lambda: self.send({"cmd": "live", "on": self.live_var.get()})
                        ).pack(anchor="w", pady=(8, 0))

        # ภาพล่าสุดที่ถ่ายตรวจ (ภาพที่ใช้ตัดสินเงื่อนไข 4 กรณี)
        ttk.Label(panel, text="ภาพที่ถ่ายตรวจล่าสุด").pack(anchor="w", pady=(10, 0))
        self.thumb_blank = tk.PhotoImage(width=320, height=240)
        self.thumb_label = tk.Label(panel, bg="black", image=self.thumb_blank)
        self.thumb_label.pack()
        self.thumb_text = tk.StringVar(value="")
        ttk.Label(panel, textvariable=self.thumb_text, wraplength=320).pack(anchor="w")
        self.tkthumb = None

        # แท็บด้านล่าง: Log / PID
        self.tabs = ttk.Notebook(root)
        self.tabs.grid(row=1, column=0, padx=6, pady=6, sticky="nwe")      # ใต้ภาพ ไม่ให้หน้าต่างสูงเกินจอ
        self.log_box = tk.Text(self.tabs, height=12, width=120)
        self.tabs.add(self.log_box, text="  Log  ")
        self._build_pid_tab()
        self._build_calib_tab()

        root.after(50, self.poll)

    # ---------- แท็บ PID: ตารางค่าทุกรอบ + กราฟ error ----------
    PID_COLS = [("name", "จุด", 32), ("shot", "รอบ", 32), ("err", "err px", 52),
                ("err_mm", "err mm", 52), ("perp", "ขวาง px", 54), ("p", "P", 46),
                ("i", "I", 42), ("d", "D", 42), ("out", "out px", 50),
                ("steps", "step", 40), ("move_cm", "ขยับ cm", 54),
                ("pos", "pos cm", 52), ("sharp", "sharp", 84), ("decision", "ผล", 92)]
    SERIES_COLOR = {"orange": "#ff8c00", "yellow": "#d4b000", "brown": "#8b4513",
                    "purple": "#8a2be2", None: "#1e88e5"}

    def _build_pid_tab(self):
        f = ttk.Frame(self.tabs)
        self.tabs.add(f, text="  PID  ")
        left = ttk.Frame(f)
        left.pack(side="left", fill="both", expand=True)
        self.pid_info = tk.StringVar(value="กด START แล้วค่าจะขึ้นที่นี่ทุกครั้งที่ถ่ายตรวจ")
        ttk.Label(left, textvariable=self.pid_info).pack(anchor="w")
        cols = [c[0] for c in self.PID_COLS]
        self.pid_tree = ttk.Treeview(left, columns=cols, show="headings", height=9)
        for key, title, w in self.PID_COLS:
            self.pid_tree.heading(key, text=title)
            self.pid_tree.column(key, width=w, minwidth=w, stretch=False,
                                 anchor="e" if key not in ("name", "decision") else "w")
        sb = ttk.Scrollbar(left, orient="vertical", command=self.pid_tree.yview)
        self.pid_tree.configure(yscrollcommand=sb.set)
        self.pid_tree.pack(side="left", fill="both", expand=True)
        sb.pack(side="left", fill="y")
        self.pid_tree.tag_configure("ok", foreground="#2e7d32")
        self.pid_tree.tag_configure("bad", foreground="#c62828")
        self.pid_tree.tag_configure("move", foreground="#1565c0")
        self.chart = tk.Canvas(f, width=250, height=210, bg="white", highlightthickness=1,
                               highlightbackground="#bbb")
        self.chart.pack(side="left", padx=6)
        self.pid_series = {}               # ชื่อจุด -> {"color":.., "pts":[(shot, err)]}
        self.pid_tol = 4.0
        self.px_per_cm = 0.0
        self._draw_chart()

    # ---------- แท็บ กล้อง/สเกล: ทำให้ภาพสเกลเท่ากันทั้งภาพ ----------
    def _build_calib_tab(self):
        f = ttk.Frame(self.tabs, padding=6)
        self.tabs.add(f, text="  กล้อง/สเกล  ")
        steps = (
            "ใช้กระดานหมากรุก 10x7 ช่อง (มุมด้านใน 9x6) พิมพ์ขนาดจริง แล้ววัดขนาดช่องด้วยไม้บรรทัด "
            "ใส่ใน config.json -> chess_square_mm\n"
            "ขั้น 1 แก้เลนส์: ถือ/วางกระดานในภาพสด ให้เห็นเต็มแผ่น กด [ถ่ายภาพกระดาน] 12-20 ครั้ง "
            "โดยย้ายไปทั้ง 4 มุม ขอบ และกลางภาพ เอียงบ้าง แล้วกด [คำนวณเลนส์]\n"
            "ขั้น 2 ตั้งสเกล: วางกระดานราบบนพื้นงาน (ระดับเดียวกับใบลาน) ขอบขนานกับราง แล้วกด [ตั้งสเกล]\n"
            "เสร็จแล้วภาพที่บันทึกตอน START จะถูกปรับให้ 1 mm = จำนวน pixel เท่ากันทั้งภาพ")
        ttk.Label(f, text=steps, wraplength=900, justify="left").pack(anchor="w")
        bf = ttk.Frame(f)
        bf.pack(anchor="w", pady=8)
        for txt, cmd in [("ขั้น 1: ถ่ายภาพกระดาน", "lens_shot"), ("คำนวณเลนส์", "lens_solve"),
                         ("ล้างภาพกระดาน", "lens_reset"), ("ขั้น 2: ตั้งสเกล", "plane_calib")]:
            ttk.Button(bf, text=txt, command=lambda c=cmd: self.send({"cmd": c})).pack(side="left", padx=3)
        self.calib_var = tk.StringVar(value="ผลจะแสดงที่นี่ (และภาพในช่อง 'ภาพที่ถ่ายตรวจล่าสุด')")
        ttk.Label(f, textvariable=self.calib_var, wraplength=900, justify="left",
                  font=("Tahoma", 10, "bold")).pack(anchor="w", pady=4)

    def _reset_pid(self):
        self.pid_tree.delete(*self.pid_tree.get_children())
        self.pid_series = {}
        self._draw_chart()

    def _draw_chart(self):
        cv = self.chart
        cv.delete("all")
        W, H, L, R, T, B = 250, 210, 34, 8, 30, 30
        cv.create_text(W / 2, 9, text="error ตามราง (px) แต่ละรอบ", font=("Tahoma", 8))
        pts = [p for s in self.pid_series.values() for p in s["pts"]]
        ymax = max([10.0, self.pid_tol * 2] + [abs(e) for _, e in pts]) * 1.1
        xmax = max([5] + [x for x, _ in pts])
        X = lambda x: L + (x - 1) / max(1, xmax - 1) * (W - L - R)
        Y = lambda y: T + (ymax - y) / (2 * ymax) * (H - T - B)
        cv.create_rectangle(L, Y(self.pid_tol), W - R, Y(-self.pid_tol), fill="#e8f5e9", outline="")
        cv.create_line(L, Y(0), W - R, Y(0), fill="#999", dash=(2, 2))
        cv.create_line(L, T, L, H - B, fill="#555")
        cv.create_line(L, H - B, W - R, H - B, fill="#555")
        for v in (ymax / 1.1, 0, -ymax / 1.1):
            cv.create_text(L - 4, Y(v), text=f"{v:.0f}", anchor="e", font=("Tahoma", 7))
        for x in range(1, xmax + 1):
            cv.create_text(X(x), H - B + 9, text=str(x), font=("Tahoma", 7))
        cv.create_text(W - R, H - 6, text=f"แถบเขียว = ผ่าน (±{self.pid_tol:g}px)", anchor="e",
                       font=("Tahoma", 7), fill="#2e7d32")
        lx = L + 4
        for name, s in self.pid_series.items():
            xy = [(X(x), Y(e)) for x, e in s["pts"]]
            if len(xy) > 1:
                cv.create_line(*[c for p in xy for c in p], fill=s["color"], width=2)
            for px, py in xy:
                cv.create_oval(px - 3, py - 3, px + 3, py + 3, fill=s["color"], outline="")
            cv.create_text(lx, 22, text=name, fill=s["color"], anchor="w", font=("Tahoma", 8, "bold"))
            lx += 28

    def on_pid(self, h):
        f = lambda v, d=1: "" if v is None else f"{v:+.{d}f}"
        if h.get("event") == "start":
            self.pid_tol = h.get("tol", 4.0)
            self.px_per_cm = h.get("px_per_cm") or 0.0
            self.pid_series[h["name"]] = {"color": self.SERIES_COLOR.get(h.get("color"), "#1e88e5"),
                                          "pts": []}
            self.pid_info.set(f"{h['name']} ({h.get('color') or '?'})   Kp={h['kp']}  Ki={h['ki']}  "
                              f"Kd={h['kd']}   เกณฑ์ ±{self.pid_tol:g}px   1 cm = {self.px_per_cm:.0f}px")
            if h["station"] == 0:
                self.tabs.select(1)        # เริ่ม START -> เปิดแท็บ PID ให้เลย
            self._draw_chart()
            return
        err = h.get("err")
        err_mm = (err / self.px_per_cm * 10) if (err is not None and self.px_per_cm) else None
        dec = h.get("decision", "")
        sharp = "" if h.get("sharp") is None else f"{h['sharp']:.1f}/{h['thr']:.1f}"
        row = [h["name"], h["shot"], f(err), f(err_mm, 2), f(h.get("perp")), f(h.get("p")),
               f(h.get("i")), f(h.get("d")), f(h.get("out")),
               "" if h.get("steps") is None else f"{h['steps']:+d}", f(h.get("move_cm"), 2),
               "" if h.get("pos_cm") is None else f"{h['pos_cm']:.2f}", sharp, dec]
        tag = "ok" if dec == "CAPTURE" else "bad" if dec.startswith("FAIL") else \
              "move" if dec.startswith("PID") else ""
        iid = self.pid_tree.insert("", "end", values=row, tags=(tag,))
        self.pid_tree.see(iid)
        if err is not None and h["name"] in self.pid_series:
            self.pid_series[h["name"]]["pts"].append((h["shot"], err))
            self._draw_chart()
        if self.session_dir:                # บันทึกลงไฟล์ pid.csv ด้วย
            path = os.path.join(self.session_dir, "pid.csv")
            new = not os.path.exists(path)
            with open(path, "a", newline="", encoding="utf-8-sig") as fp:
                w = csv.writer(fp)
                if new:
                    w.writerow([c[1] for c in self.PID_COLS])
                w.writerow(row)

    def send(self, msg):
        if msg.get("cmd") == "start":
            self.new_session()
        if not self.server.send(msg):
            self.log("ยังไม่มี Pi เชื่อมต่อ")

    def arm_pick(self):
        self.pick_armed = True
        self.img_label.config(cursor="crosshair")
        self.log(f"คลิกที่กลางกระดาษสี {self.color_box.get()} บนภาพสด")

    def on_image_click(self, event):
        if not self.pick_armed or self.tkimg is None:
            return
        self.pick_armed = False
        self.img_label.config(cursor="")
        w, h = self.tkimg.width(), self.tkimg.height()
        if not (0 <= event.x < w and 0 <= event.y < h):
            self.log("คลิกนอกภาพ ลองใหม่")
            return
        color = self.color_names[self.color_box.get()]
        self.send({"cmd": "pick_color", "color": color, "fx": event.x / w, "fy": event.y / h})

    def do_jog(self, sign):
        try:
            cm = float(self.jog.get())
        except (tk.TclError, ValueError):
            self.log("ใส่ระยะ Jog เป็นตัวเลข (cm)")
            return
        self.send({"cmd": "jog", "dcm": sign * cm})

    def new_session(self):
        self.session_dir = os.path.join(SAVE_DIR, time.strftime("%Y%m%d_%H%M%S"))
        os.makedirs(self.session_dir, exist_ok=True)
        for l in self.st_labels:
            l.config(bg="#9e9e9e")
        self._reset_pid()

    def log(self, text):
        self.log_box.insert("end", time.strftime("%H:%M:%S ") + text + "\n")
        self.log_box.see("end")

    @staticmethod
    def _to_tk(img, max_w, max_h):
        h, w = img.shape[:2]
        s = min(max_w / w, max_h / h)
        img = cv2.resize(img, (int(w * s), int(h * s)), interpolation=cv2.INTER_AREA)
        return ImageTk.PhotoImage(Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB)))

    def show(self, img):
        """ช่องใหญ่: ภาพสด"""
        self.tkimg = self._to_tk(img, 960, 540)
        self.img_label.config(image=self.tkimg)

    def show_thumb(self, img, text=""):
        """ช่องเล็ก: ภาพที่ถ่ายตรวจล่าสุด"""
        self.tkthumb = self._to_tk(img, 320, 240)
        self.thumb_label.config(image=self.tkthumb)
        self.thumb_text.set(text)
        if not self.live_var.get():        # ปิดภาพสดอยู่ -> แสดงภาพตรวจในช่องใหญ่ด้วย
            self.show(img)

    def poll(self):
        latest_live = None
        while not self.inbox.empty():
            hdr, img, data = self.inbox.get()
            t = hdr.get("type")
            if t == "live":
                latest_live = (hdr, img)       # แสดงแค่เฟรมล่าสุด ไม่ให้ภาพค้าง/ดีเลย์
            elif t == "log":
                self.log(hdr["text"])
            elif t == "pid":
                self.on_pid(hdr)
            elif t == "calib":
                self.calib_var.set(hdr["text"])
            elif t == "frame":
                if img is not None:
                    self.show_thumb(img, hdr.get("text", ""))
                if "station" in hdr:
                    self.st_labels[hdr["station"]].config(bg="#f9a825")   # กำลังทำ
            elif t == "capture":
                self.save_capture(hdr, img, data)
            elif t == "pos":
                self.pos_var.set(f"pos: {hdr['cm']:.2f} cm  ({hdr['steps']} steps)")
            elif t == "done":
                res = hdr["results"]
                for i, ok in enumerate(res):
                    if not ok:
                        self.st_labels[i].config(bg="#c62828")
                self.log(f"เสร็จ: ผ่าน {sum(res)}/{len(res)} ตำแหน่ง")
                if self.session_dir:
                    self.log(process_results(self.session_dir))
        if latest_live is not None:
            hdr, img = latest_live
            if img is not None and self.live_var.get():
                self.show(img)
            if "cm" in hdr:
                state = "  (กำลังเคลื่อนที่)" if hdr.get("moving") else ""
                self.pos_var.set(f"pos: {hdr['cm']:.2f} cm{state}")
        self.root.after(30, self.poll)

    def save_capture(self, hdr, img, data):
        if self.session_dir is None:
            self.new_session()
        i, name = hdr["station"], hdr["name"]
        path = os.path.join(self.session_dir, f"{name}.jpg")
        with open(path, "wb") as f:
            f.write(data)                      # เก็บ JPEG ต้นฉบับ ไม่บีบอัดซ้ำ
        csv_path = os.path.join(self.session_dir, "result.csv")
        new = not os.path.exists(csv_path)
        with open(csv_path, "a", newline="") as f:
            w = csv.writer(f)
            if new:
                w.writerow(["station", "sharpness", "err_along_px", "err_perp_px", "px_per_mm", "markers_json"])
            w.writerow([name, round(hdr["sharp"], 1), hdr.get("err_along"), hdr.get("err_perp"),
                        hdr.get("px_per_mm", ""), json.dumps(hdr["markers"])])
        self.st_labels[i].config(bg="#2e7d32")
        self.log(f"บันทึก {path}")
        if img is not None:
            self.show_thumb(img, f"บันทึก {name} แล้ว")


if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()