#!/usr/bin/env python3
"""
Robotic Arm — Standalone Serial Dashboard
==========================================
Controls the arm directly over USB serial using the S-Protocol firmware.

Protocol Reference (from robotic_arm_firmware.ino):
  • S:<w>,<s>,<e>,<roll>,<pitch>,<grip>  — Move all 6 joints
  • HOME                                  — Firmware moves to boot home pose
  • PING                                  — Firmware replies PONG:<version>

Joint limits match firmware exactly:
  Waist       0–180  | home 90
  Shoulder   30–150  | home 90
  Elbow      20–160  | home 90  (firmware inverts elbow internally)
  Wrist Roll  0–180  | home 90
  Wrist Pitch 0–180  | home 90
  Gripper    20–160  | home 30
"""

import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import serial
import serial.tools.list_ports

# ──────────────────────────────────────────────────────────────────────────────
#  JOINT CONFIGURATION
#  Exactly mirrors the firmware Joint array; keep in sync with the .ino file.
# ──────────────────────────────────────────────────────────────────────────────
# (Label, key_increase, key_decrease, min°, max°, home°)
JOINTS = [
    ("Waist",       "q", "a",   0, 180,  90),
    ("Shoulder",    "w", "s",  30, 150,  90),
    ("Elbow",       "e", "d",  20, 160,  90),
    ("Wrist Roll",  "r", "f",   0, 180,  90),
    ("Wrist Pitch", "t", "g",   0, 180,  90),
    ("Gripper",     "y", "h",  20, 160,  30),
]

# ──────────────────────────────────────────────────────────────────────────────
#  COMMUNICATION
# ──────────────────────────────────────────────────────────────────────────────
BAUD_RATE      = 115200
SERIAL_TIMEOUT = 0.05   # seconds — non-blocking readline

# ──────────────────────────────────────────────────────────────────────────────
#  COLOUR PALETTE
# ──────────────────────────────────────────────────────────────────────────────
C = {
    "bg":          "#1a1f2e",
    "panel":       "#242938",
    "border":      "#2f3650",
    "text":        "#e8eaf0",
    "muted":       "#7f8fa4",
    "blue":        "#3b82f6",
    "green":       "#10b981",
    "red":         "#ef4444",
    "yellow":      "#f59e0b",
    "purple":      "#8b5cf6",
}


# ──────────────────────────────────────────────────────────────────────────────
#  DASHBOARD
# ──────────────────────────────────────────────────────────────────────────────
class Dashboard(tk.Tk):
    """Standalone GUI for direct serial control of the robotic arm."""

    def __init__(self) -> None:
        super().__init__()
        self.title("Robotic Arm — Serial Dashboard")
        self.geometry("520x860")
        self.configure(bg=C["bg"])
        self.resizable(False, False)

        # ── State ─────────────────────────────────────────────────────────
        self._ser:       serial.Serial | None = None
        self._sliders:   dict[int, tk.IntVar] = {}
        self._rx_job:    str | None           = None   # after() job id
        self._waypoints: list[list[int]]      = []
        self._playing:   bool                 = False
        self._delay_ms:  int                  = 400
        self._rx_log:    list[str]            = []     # last 5 RX lines

        self._build_ttk_styles()
        self._build_ui()
        self._bind_keyboard()
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ── Styles ────────────────────────────────────────────────────────────────
    def _build_ttk_styles(self) -> None:
        s = ttk.Style(self)
        s.theme_use("clam")
        s.configure("Panel.TLabelframe",
                    background=C["panel"], bordercolor=C["border"],
                    borderwidth=1, relief="solid")
        s.configure("Panel.TLabelframe.Label",
                    background=C["panel"], foreground=C["muted"],
                    font=("Segoe UI", 10, "bold"))

    # ── Full UI ───────────────────────────────────────────────────────────────
    def _build_ui(self) -> None:
        # 1. Header
        hdr = tk.Frame(self, bg=C["blue"], pady=12)
        hdr.pack(fill="x")
        tk.Label(hdr, text="🦾  ROBOTIC ARM DASHBOARD",
                 bg=C["blue"], fg="white",
                 font=("Segoe UI", 17, "bold")).pack()
        tk.Label(hdr, text="S-Protocol  •  Direct USB Serial Control",
                 bg=C["blue"], fg="#bfdbfe",
                 font=("Segoe UI", 9)).pack()

        # 2. Connection panel
        self._build_connection_panel()

        # 3. Sliders
        self._build_slider_panel()

        # 4. Quick commands
        self._build_command_panel()

        # 5. Sequence memory
        self._build_sequence_panel()

        # 6. RX Log
        self._build_rx_log()

    # ── Section: Connection ────────────────────────────────────────────────────
    def _build_connection_panel(self) -> None:
        frm = ttk.LabelFrame(self, text=" 🔌  Hardware Connection",
                             style="Panel.TLabelframe", padding=10)
        frm.pack(fill="x", padx=16, pady=(14, 4))

        # Row 1: port selector
        row1 = tk.Frame(frm, bg=C["panel"])
        row1.pack(fill="x")

        tk.Label(row1, text="Port:", bg=C["panel"], fg=C["muted"],
                 font=("Segoe UI", 9)).pack(side="left")

        self._port_var = tk.StringVar()
        self._port_menu = ttk.Combobox(row1, textvariable=self._port_var,
                                       state="readonly", width=13)
        self._port_menu.pack(side="left", padx=8)

        tk.Button(row1, text="↺ Refresh", bg=C["panel"], fg=C["muted"],
                  relief="flat", font=("Segoe UI", 8),
                  command=self._refresh_ports).pack(side="left")

        self._btn_connect = tk.Button(
            row1, text="Connect", bg=C["green"], fg="white",
            relief="flat", font=("Segoe UI", 9, "bold"), padx=12,
            command=self._toggle_connection)
        self._btn_connect.pack(side="right")

        self._refresh_ports()

        # Row 2: status line
        self._status_var = tk.StringVar(value="OFFLINE")
        self._status_lbl = tk.Label(frm, textvariable=self._status_var,
                                    bg=C["panel"], fg=C["red"],
                                    font=("Consolas", 11, "bold"))
        self._status_lbl.pack(pady=(6, 0))

        # Row 3: last TX
        self._tx_var = tk.StringVar(value="TX: —")
        tk.Label(frm, textvariable=self._tx_var,
                 bg=C["panel"], fg=C["muted"],
                 font=("Consolas", 8)).pack()

    # ── Section: Sliders ────────────────────────────────────────────────────
    def _build_slider_panel(self) -> None:
        frm = ttk.LabelFrame(self, text=" 🕹️  Live Teleoperation",
                             style="Panel.TLabelframe", padding=12)
        frm.pack(fill="x", padx=16, pady=8)

        for idx, (label, key_up, key_dn, lo, hi, home) in enumerate(JOINTS):
            row = tk.Frame(frm, bg=C["panel"])
            row.pack(fill="x", pady=4)

            # Joint label
            tk.Label(row, text=label, width=11, anchor="w",
                     bg=C["panel"], fg=C["text"],
                     font=("Segoe UI", 9, "bold")).pack(side="left")

            # Key hint
            tk.Label(row, text=f"[{key_dn.upper()}]",
                     bg=C["panel"], fg=C["muted"],
                     font=("Segoe UI", 8)).pack(side="left")

            var = tk.IntVar(value=home)
            self._sliders[idx] = var

            scale = tk.Scale(
                row, from_=lo, to=hi, variable=var,
                orient="horizontal", length=220,
                bg=C["panel"], fg=C["text"], troughcolor=C["border"],
                highlightthickness=0, showvalue=True,
                command=lambda _, i=idx: self._push())
            scale.pack(side="left", padx=6)

            tk.Label(row, text=f"[{key_up.upper()}]",
                     bg=C["panel"], fg=C["muted"],
                     font=("Segoe UI", 8)).pack(side="left")

    # ── Section: Quick Commands ─────────────────────────────────────────────
    def _build_command_panel(self) -> None:
        frm = ttk.LabelFrame(self, text=" ⚡  Firmware Commands",
                             style="Panel.TLabelframe", padding=10)
        frm.pack(fill="x", padx=16, pady=4)

        grid = tk.Frame(frm, bg=C["panel"])
        grid.pack(fill="x")

        # (label, bg, command, row, col)
        cmds = [
            ("🏠  Home",  C["blue"],   self._cmd_home,    0, 0),
            ("📡  Ping",  C["purple"],  self._cmd_ping,    0, 1),
        ]
        for text, bg, func, r, c in cmds:
            tk.Button(grid, text=text, bg=bg, fg="white",
                      relief="flat", font=("Segoe UI", 10, "bold"),
                      pady=6, command=func
                      ).grid(row=r, column=c, padx=5, pady=4, sticky="ew")

        grid.columnconfigure(0, weight=1)
        grid.columnconfigure(1, weight=1)

        tk.Label(frm, text=(
            "Home  → sends 'HOME' command to firmware\n"
            "Ping  → sends 'PING', firmware replies PONG:<version>"
        ), bg=C["panel"], fg=C["muted"], font=("Segoe UI", 7),
           justify="left").pack(anchor="w", pady=(4, 0))

    # ── Section: Sequence Memory ────────────────────────────────────────────
    def _build_sequence_panel(self) -> None:
        frm = ttk.LabelFrame(self, text=" 💾  Waypoint Sequence",
                             style="Panel.TLabelframe", padding=10)
        frm.pack(fill="x", padx=16, pady=4)

        grid = tk.Frame(frm, bg=C["panel"])
        grid.pack(fill="x")

        seqs = [
            ("💾  Save Point",    C["yellow"], "black",  self._save_waypoint, 0, 0),
            ("▶  Run",           C["green"],  "white",  self._play,          0, 1),
            ("⏹  Stop",          C["red"],    "white",  self._stop,          1, 0),
            ("🗑  Clear",         C["panel"],  C["muted"], self._clear,       1, 1),
            ("🐇  Faster (−50ms)", C["panel"], C["muted"], lambda: self._adj_delay(-50), 2, 0),
            ("🐢  Slower (+50ms)", C["panel"], C["muted"], lambda: self._adj_delay(+50), 2, 1),
        ]
        for text, bg, fg, func, r, c in seqs:
            tk.Button(grid, text=text, bg=bg, fg=fg,
                      relief="flat", font=("Segoe UI", 9, "bold"),
                      pady=5, command=func
                      ).grid(row=r, column=c, padx=5, pady=3, sticky="ew")

        grid.columnconfigure(0, weight=1)
        grid.columnconfigure(1, weight=1)

        self._seq_lbl = tk.Label(
            frm, text="Waypoints: 0  |  Delay: 400 ms",
            bg=C["panel"], fg=C["muted"], font=("Segoe UI", 8))
        self._seq_lbl.pack(pady=(6, 0))

        tk.Label(frm, text=(
            "Keys:  V = save  •  B = run  •  P = stop"
        ), bg=C["panel"], fg=C["muted"], font=("Segoe UI", 7)).pack()

    # ── Section: RX Log ────────────────────────────────────────────────────
    def _build_rx_log(self) -> None:
        frm = ttk.LabelFrame(self, text=" 📟  Firmware Responses (last 5)",
                             style="Panel.TLabelframe", padding=8)
        frm.pack(fill="x", padx=16, pady=(4, 12))

        self._log_text = tk.Text(
            frm, height=5, bg=C["bg"], fg=C["green"],
            font=("Consolas", 8), relief="flat",
            state="disabled", wrap="word")
        self._log_text.pack(fill="x")

    # ── Keyboard bindings ─────────────────────────────────────────────────────
    def _bind_keyboard(self) -> None:
        for idx, (_, key_up, key_dn, lo, hi, _) in enumerate(JOINTS):
            var = self._sliders[idx]
            self.bind(f"<KeyPress-{key_up}>",
                      lambda e, v=var, mx=hi:  self._kb(v, +2, mx))
            self.bind(f"<KeyPress-{key_dn}>",
                      lambda e, v=var, mn=lo:  self._kb(v, -2, mn))
        self.bind("<KeyPress-v>", lambda e: self._save_waypoint())
        self.bind("<KeyPress-b>", lambda e: self._play())
        self.bind("<KeyPress-p>", lambda e: self._stop())

    # ── Connection helpers ────────────────────────────────────────────────────
    def _refresh_ports(self) -> None:
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self._port_menu["values"] = ports or ["(no ports found)"]
        if ports:
            self._port_menu.current(0)

    def _toggle_connection(self) -> None:
        if self._ser and self._ser.is_open:
            self._disconnect()
        else:
            self._connect()

    def _connect(self) -> None:
        port = self._port_var.get()
        if not port or "(no ports" in port:
            messagebox.showwarning("Port Missing", "Select a valid serial port.")
            return
        self._btn_connect.config(state="disabled")
        self._set_status("CONNECTING...", C["yellow"])
        threading.Thread(target=self._connect_worker,
                         args=(port,), daemon=True).start()

    def _connect_worker(self, port: str) -> None:
        try:
            ser = serial.Serial(port, BAUD_RATE, timeout=SERIAL_TIMEOUT)
            time.sleep(2)                       # wait for Arduino reset
            ser.reset_input_buffer()
            self.after(0, self._on_connected, ser, port)
        except Exception as exc:
            self.after(0, self._on_connect_fail, str(exc))

    def _on_connected(self, ser: serial.Serial, port: str) -> None:
        self._ser = ser
        self._btn_connect.config(text="Disconnect", bg=C["red"], state="normal")
        self._set_status(f"ONLINE  ⟷  {port}", C["green"])
        self._start_rx_poll()
        # Go to home position via slider reset (also sends S: command)
        self._cmd_home()

    def _on_connect_fail(self, msg: str) -> None:
        self._btn_connect.config(state="normal")
        self._set_status("CONNECTION FAILED", C["red"])
        messagebox.showerror("Serial Error", msg)

    def _disconnect(self) -> None:
        self._stop_rx_poll()
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None
        self._btn_connect.config(text="Connect", bg=C["green"], state="normal")
        self._set_status("OFFLINE", C["red"])
        self._tx_var.set("TX: —")

    # ── Serial send helpers ───────────────────────────────────────────────────
    def _push(self) -> None:
        """Send current slider values as S-Protocol command."""
        if not self._ser or not self._ser.is_open:
            return
        angles = [str(self._sliders[i].get()) for i in range(len(JOINTS))]
        cmd = f"S:{','.join(angles)}\n"
        self._send_raw(cmd)

    def _send_raw(self, cmd: str) -> None:
        """Low-level serial write with error recovery."""
        if not self._ser or not self._ser.is_open:
            return
        try:
            self._ser.write(cmd.encode("utf-8"))
            self._tx_var.set(f"TX: {cmd.strip()}")
        except Exception as exc:
            self._set_status("TX ERROR", C["red"])
            self._log(f"TX failed: {exc}")

    # ── Firmware Commands ─────────────────────────────────────────────────────
    def _cmd_home(self) -> None:
        """Reset sliders to home positions AND send HOME command to firmware."""
        self._playing = False
        for idx, (*_, home) in enumerate(JOINTS):
            self._sliders[idx].set(home)
        # Send firmware HOME command (firmware moves servos if needed)
        self._send_raw("HOME\n")

    def _cmd_ping(self) -> None:
        """Send PING — firmware responds with PONG:<version>."""
        self._send_raw("PING\n")

    # ── Keyboard movement ─────────────────────────────────────────────────────
    def _kb(self, var: tk.IntVar, delta: int, limit: int) -> None:
        v = var.get()
        if delta > 0 and v < limit:
            var.set(v + delta)
            self._push()
        elif delta < 0 and v > limit:
            var.set(v + delta)
            self._push()

    # ── Sequence engine ───────────────────────────────────────────────────────
    def _save_waypoint(self) -> None:
        wp = [self._sliders[i].get() for i in range(len(JOINTS))]
        self._waypoints.append(wp)
        self._update_seq_label()

    def _clear(self) -> None:
        self._playing = False
        self._waypoints = []
        self._update_seq_label()

    def _adj_delay(self, delta: int) -> None:
        self._delay_ms = max(50, min(3000, self._delay_ms + delta))
        self._update_seq_label()

    def _update_seq_label(self) -> None:
        self._seq_lbl.config(
            text=f"Waypoints: {len(self._waypoints)}  |  Delay: {self._delay_ms} ms")

    def _stop(self) -> None:
        self._playing = False

    def _play(self) -> None:
        if not self._waypoints or self._playing:
            return
        self._playing = True
        threading.Thread(target=self._play_worker, daemon=True).start()

    def _play_worker(self) -> None:
        for wp in self._waypoints:
            if not self._playing:
                break
            self.after(0, self._apply_wp, wp)
            time.sleep(self._delay_ms / 1000.0)
        self.after(0, setattr, self, "_playing", False)

    def _apply_wp(self, wp: list[int]) -> None:
        for i, val in enumerate(wp):
            self._sliders[i].set(val)
        self._push()

    # ── RX polling (non-blocking, Tk event loop friendly) ────────────────────
    def _start_rx_poll(self) -> None:
        if self._rx_job is None:
            self._poll()

    def _stop_rx_poll(self) -> None:
        if self._rx_job is not None:
            self.after_cancel(self._rx_job)
            self._rx_job = None

    def _poll(self) -> None:
        if self._ser and self._ser.is_open:
            try:
                while self._ser.in_waiting > 0:
                    line = self._ser.readline().decode("utf-8", errors="ignore").strip()
                    if line:
                        self._log(line)
            except Exception:
                pass
            self._rx_job = self.after(100, self._poll)
        else:
            self._rx_job = None

    def _log(self, text: str) -> None:
        """Append a line to the RX log (max 5 lines shown)."""
        self._rx_log.append(text)
        if len(self._rx_log) > 5:
            self._rx_log = self._rx_log[-5:]
        self._log_text.config(state="normal")
        self._log_text.delete("1.0", "end")
        self._log_text.insert("end", "\n".join(self._rx_log))
        self._log_text.config(state="disabled")

    # ── Misc ──────────────────────────────────────────────────────────────────
    def _set_status(self, msg: str, color: str) -> None:
        self._status_var.set(msg)
        self._status_lbl.config(fg=color)

    def _on_close(self) -> None:
        self._playing = False
        self._stop_rx_poll()
        if self._ser and self._ser.is_open:
            self._ser.close()
        self.destroy()


# ──────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    app = Dashboard()
    app.mainloop()
