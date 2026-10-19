#!/usr/bin/env python3
"""
Serial Terminal - lightweight, cross-platform (Windows / Ubuntu) serial monitor.

Requirements:
    pip install pyserial
    (Ubuntu also needs Tk:  sudo apt install python3-tk
     and serial permission: sudo usermod -aG dialout $USER  -> log out/in)

Features:
    * Config tab: port + serial settings (baud, data bits, parity, stop bits, flow control)
    * Connect / Disconnect button (app stays open)
    * ANSI color support (ESP-IDF style logging colors)
    * Line-based display: a line is complete only when "\r\n" is received (any OS)
    * Live text filter (applied to complete lines)
    * Configurable macro buttons (edit/label next to each button; text sent + line ending)
    * RTS / DTR toggles and CTS / DSR / RI / CD indicators
    * Send box at the bottom (text sent + "\r\n")
    * Dark theme
"""

import json
import math
import queue
import re
import sys
import threading
import tkinter as tk
from collections import deque
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    raise SystemExit("pyserial is required:  pip install pyserial")

APP_TITLE = "Serial Terminal"
CONFIG_PATH = Path.home() / ".serial_terminal.json"
LINE_END = "\r\n"   # default TX terminator; selectable on the Config tab
TX_ENDINGS = {
    "CR+LF (\\r\\n)": "\r\n",
    "LF only (\\n)": "\n",
    "CR only (\\r)": "\r",
    "None": "",
}
MAX_STORED_LINES = 100_000   # lines kept in memory for re-filtering / saving
MAX_DISPLAY_LINES = 5_000    # lines kept in the text widget
NUM_MACROS = 8
NUM_HIGHLIGHTS = 4
HL_DEFAULT_COLOR = "#ff00ff"   # magenta

# ---------------------------------------------------------------- theme
BG = "#1e1e1e"
BG_ALT = "#252526"
BG_INPUT = "#2d2d30"
FG = "#d4d4d4"
FG_DIM = "#808080"
ACCENT = "#0e639c"
BORDER = "#3c3c3c"
TX_COLOR = "#4fc1ff"
SYS_COLOR = "#c586c0"

# ---------------------------------------------------------------- ANSI
ANSI_BASIC = ["#000000", "#cd3131", "#0dbc79", "#e5e510",
              "#2472c8", "#bc3fbc", "#11a8cd", "#e5e5e5"]
ANSI_BRIGHT = ["#666666", "#f14c4c", "#23d18b", "#f5f543",
               "#3b8eea", "#d670d6", "#29b8db", "#ffffff"]
CSI_RE = re.compile(r"\x1b\[([0-9;?]*)([A-Za-z])")


def xterm256(n):
    if n < 8:
        return ANSI_BASIC[n]
    if n < 16:
        return ANSI_BRIGHT[n - 8]
    if n < 232:
        n -= 16
        r, g, b = n // 36, (n // 6) % 6, n % 6
        conv = lambda v: 0 if v == 0 else 55 + v * 40
        return "#%02x%02x%02x" % (conv(r), conv(g), conv(b))
    v = 8 + (n - 232) * 10
    return "#%02x%02x%02x" % (v, v, v)


def parse_ansi(line):
    """Return list of (text, fg_color_or_None, bold) segments."""
    segs = []
    fg, bold = None, False
    pos = 0
    for m in CSI_RE.finditer(line):
        if m.start() > pos:
            segs.append((line[pos:m.start()], fg, bold))
        pos = m.end()
        if m.group(2) != "m":
            continue  # ignore non-color escape sequences
        codes = [int(c) if c.isdigit() else 0 for c in m.group(1).split(";")] \
            if m.group(1) else [0]
        i = 0
        while i < len(codes):
            c = codes[i]
            if c == 0:
                fg, bold = None, False
            elif c == 1:
                bold = True
            elif c == 22:
                bold = False
            elif 30 <= c <= 37:
                fg = ANSI_BASIC[c - 30]
            elif 90 <= c <= 97:
                fg = ANSI_BRIGHT[c - 90]
            elif c == 39:
                fg = None
            elif c == 38 and i + 2 < len(codes) and codes[i + 1] == 5:
                fg = xterm256(min(codes[i + 2], 255))
                i += 2
            elif c == 38 and i + 4 < len(codes) and codes[i + 1] == 2:
                fg = "#%02x%02x%02x" % tuple(min(v, 255) for v in codes[i + 2:i + 5])
                i += 4
            i += 1
    if pos < len(line):
        segs.append((line[pos:], fg, bold))
    return segs


def strip_ansi(line):
    return CSI_RE.sub("", line)


# ---------------------------------------------------------------- reader thread
class SerialReader(threading.Thread):
    """Reads bytes and emits only COMPLETE lines terminated by CRLF."""

    def __init__(self, ser, out_queue):
        super().__init__(daemon=True)
        self.ser = ser
        self.q = out_queue
        self.stop_evt = threading.Event()

    def run(self):
        buf = bytearray()
        try:
            while not self.stop_evt.is_set():
                data = self.ser.read(max(1, self.ser.in_waiting))
                if not data:
                    continue
                buf.extend(data)
                while True:
                    idx = buf.find(b"\r\n")
                    if idx < 0:
                        break
                    line = bytes(buf[:idx])
                    del buf[:idx + 2]
                    self.q.put(("line", line.decode("utf-8", errors="replace")))
                if len(buf) > 65536:  # safety valve: no terminator for a long time
                    self.q.put(("line", bytes(buf).decode("utf-8", errors="replace")))
                    buf.clear()
        except (serial.SerialException, OSError) as e:
            if not self.stop_evt.is_set():
                self.q.put(("error", str(e)))

    def stop(self):
        self.stop_evt.set()


# ---------------------------------------------------------------- application
class SerialApp(tk.Tk):
    PARITY = {"None": serial.PARITY_NONE, "Even": serial.PARITY_EVEN,
              "Odd": serial.PARITY_ODD, "Mark": serial.PARITY_MARK,
              "Space": serial.PARITY_SPACE}
    STOPBITS = {"1": serial.STOPBITS_ONE, "1.5": serial.STOPBITS_ONE_POINT_FIVE,
                "2": serial.STOPBITS_TWO}
    FLOW = ["None", "RTS/CTS", "DSR/DTR", "XON/XOFF"]
    BAUDS = ["1200", "2400", "4800", "9600", "19200", "38400", "57600",
             "115200", "230400", "460800", "921600", "1500000", "2000000"]

    def __init__(self):
        super().__init__()
        self.title(APP_TITLE)
        self.geometry("1000x720")
        self.minsize(760, 500)

        self.ser = None
        self.reader = None
        self.rx_queue = queue.Queue()
        self.lines = deque(maxlen=MAX_STORED_LINES)   # (kind, text)
        self.fg_tags = set()
        self.history, self.hist_idx = [], 0
        self._filter_job = None

        self.cfg = self._load_config()
        self._build_style()
        self._build_vars()
        self._build_ui()
        self.refresh_ports()
        self._update_connection_ui()

        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(30, self._poll_queue)
        self.after(250, self._poll_modem_lines)

    # ------------------------------------------------------------ config file
    def _load_config(self):
        defaults = {
            "port": "", "baud": "115200", "bytesize": "8", "parity": "None",
            "stopbits": "1", "flow": "None", "line_ending": "CR+LF (\\r\\n)",
            "signals_on_connect": False, "echo": True, "autoscroll": True,
            "macros": [{"label": "", "text": ""} for _ in range(NUM_MACROS)],
            "highlights": [{"enabled": False, "text": "", "color": HL_DEFAULT_COLOR}
                           for _ in range(NUM_HIGHLIGHTS)],
        }
        try:
            data = json.loads(CONFIG_PATH.read_text())
            defaults.update({k: v for k, v in data.items() if k in defaults})
        except Exception:
            pass
        macros = defaults["macros"][:NUM_MACROS]
        macros += [{"label": "", "text": ""}] * (NUM_MACROS - len(macros))
        defaults["macros"] = macros
        hls = [h for h in defaults["highlights"] if isinstance(h, dict)][:NUM_HIGHLIGHTS]
        hls += [{"enabled": False, "text": "", "color": HL_DEFAULT_COLOR}] * (
            NUM_HIGHLIGHTS - len(hls))
        defaults["highlights"] = [
            {"enabled": bool(h.get("enabled", False)), "text": str(h.get("text", "")),
             "color": str(h.get("color") or HL_DEFAULT_COLOR)} for h in hls]
        return defaults

    def _save_config(self):
        self.cfg.update({
            "port": self.port_var.get(), "baud": self.baud_var.get(),
            "bytesize": self.bytesize_var.get(), "parity": self.parity_var.get(),
            "stopbits": self.stopbits_var.get(), "flow": self.flow_var.get(),
            "line_ending": self.ending_var.get(),
            "signals_on_connect": self.sig_on_connect_var.get(),
            "echo": self.echo_var.get(), "autoscroll": self.autoscroll_var.get(),
            "macros": [{"label": l.get().strip(), "text": t.get()}
                       for l, t in self.macro_vars],
            "highlights": [{"enabled": en.get(), "text": tx.get(),
                            "color": self.hl_colors[i]}
                           for i, (en, tx) in enumerate(self.hl_vars)],
        })
        try:
            CONFIG_PATH.write_text(json.dumps(self.cfg, indent=2))
        except OSError:
            pass

    # ------------------------------------------------------------ style
    def _build_style(self):
        self.configure(bg=BG)
        s = ttk.Style(self)
        s.theme_use("clam")
        s.configure(".", background=BG, foreground=FG, fieldbackground=BG_INPUT,
                    bordercolor=BORDER, lightcolor=BG, darkcolor=BG,
                    troughcolor=BG_ALT, focuscolor=BG, insertcolor=FG)
        s.configure("TFrame", background=BG)
        s.configure("TLabel", background=BG, foreground=FG)
        s.configure("TLabelframe", background=BG, bordercolor=BORDER)
        s.configure("TLabelframe.Label", background=BG, foreground=FG_DIM)
        s.configure("TButton", background=BG_INPUT, foreground=FG, padding=(10, 4),
                    bordercolor=BORDER)
        s.map("TButton", background=[("active", "#3e3e42"), ("disabled", BG_ALT)],
              foreground=[("disabled", FG_DIM)])
        s.configure("Accent.TButton", background=ACCENT, foreground="#ffffff")
        s.map("Accent.TButton", background=[("active", "#1177bb"), ("disabled", BG_ALT)])
        s.configure("TEntry", fieldbackground=BG_INPUT, foreground=FG, insertcolor=FG)
        s.configure("TCombobox", fieldbackground=BG_INPUT, background=BG_INPUT,
                    foreground=FG, arrowcolor=FG, insertcolor=FG)
        s.map("TCombobox", fieldbackground=[("readonly", BG_INPUT)],
              foreground=[("readonly", FG)], selectbackground=[("readonly", BG_INPUT)],
              selectforeground=[("readonly", FG)])
        s.configure("TCheckbutton", background=BG, foreground=FG)
        s.map("TCheckbutton", background=[("active", BG)], indicatorcolor=[
            ("selected", ACCENT), ("!selected", BG_INPUT)])
        s.configure("Toggle.TCheckbutton", background=BG_INPUT, padding=(10, 4))
        s.map("Toggle.TCheckbutton", background=[("selected", "#2e7d32"),
                                                  ("active", "#3e3e42")])
        s.configure("TNotebook", background=BG, bordercolor=BORDER)
        s.configure("TNotebook.Tab", background=BG_ALT, foreground=FG_DIM,
                    padding=(16, 6))
        s.map("TNotebook.Tab", background=[("selected", BG)],
              foreground=[("selected", FG)])
        s.configure("Vertical.TScrollbar", background=BG_INPUT, arrowcolor=FG,
                    bordercolor=BG, troughcolor=BG_ALT)
        # dropdown list colors for comboboxes
        self.option_add("*TCombobox*Listbox.background", BG_INPUT)
        self.option_add("*TCombobox*Listbox.foreground", FG)
        self.option_add("*TCombobox*Listbox.selectBackground", ACCENT)
        self.option_add("*TCombobox*Listbox.selectForeground", "#ffffff")

    # ------------------------------------------------------------ vars
    def _build_vars(self):
        c = self.cfg
        self.port_var = tk.StringVar(value=c["port"])
        self.baud_var = tk.StringVar(value=c["baud"])
        self.bytesize_var = tk.StringVar(value=c["bytesize"])
        self.parity_var = tk.StringVar(value=c["parity"])
        self.stopbits_var = tk.StringVar(value=c["stopbits"])
        self.flow_var = tk.StringVar(value=c["flow"])
        self.ending_var = tk.StringVar(value=c["line_ending"])
        self.sig_on_connect_var = tk.BooleanVar(value=c["signals_on_connect"])
        self.echo_var = tk.BooleanVar(value=c["echo"])
        self.autoscroll_var = tk.BooleanVar(value=c["autoscroll"])
        self.filter_var = tk.StringVar()
        self.case_var = tk.BooleanVar(value=False)
        self.rts_var = tk.BooleanVar(value=False)
        self.dtr_var = tk.BooleanVar(value=False)
        self.status_var = tk.StringVar(value="Disconnected")
        self.send_var = tk.StringVar()
        self.hl_vars = [(tk.BooleanVar(value=h["enabled"]), tk.StringVar(value=h["text"]))
                        for h in c["highlights"]]
        self.hl_colors = [h["color"] for h in c["highlights"]]
        self.hl_btns = []
        self._hl_job = None
        self.macro_vars = [(tk.StringVar(value=m["label"]), tk.StringVar(value=m["text"]))
                           for m in c["macros"]]

    # ------------------------------------------------------------ UI
    def _build_ui(self):
        # Packed with side="bottom": first widget is at the very bottom.
        # Order from bottom to top: Highlights, Macros, Send.
        hf = ttk.LabelFrame(self, text="Highlights", padding=6)
        hf.pack(side="bottom", fill="x", padx=8, pady=(0, 8))
        for i, (en, tx) in enumerate(self.hl_vars):
            slot = ttk.Frame(hf)
            slot.grid(row=0, column=i, sticky="ew", padx=4)
            hf.columnconfigure(i, weight=1, uniform="hl")
            ttk.Checkbutton(slot, text=str(i + 1), variable=en,
                            command=self._schedule_rehighlight).pack(side="left")
            btn = tk.Button(slot, width=1, height=1, relief="flat", bd=0,
                            padx=0, pady=0, cursor="hand2",
                            bg=self.hl_colors[i], activebackground=self.hl_colors[i],
                            highlightthickness=1, highlightbackground=BORDER,
                            command=lambda i=i: self._pick_highlight_color(i))
            btn.pack(side="right", padx=(4, 0))
            self.hl_btns.append(btn)
            ttk.Entry(slot, textvariable=tx, width=10).pack(
                side="left", fill="x", expand=True)
            tx.trace_add("write", lambda *a: self._schedule_rehighlight())

        mf = ttk.LabelFrame(self, text="Macros", padding=6)
        mf.pack(side="bottom", fill="x", padx=8, pady=(0, 2))
        self.macro_buttons = []
        self.macro_name_lbls = []
        self.macro_preview_lbls = []
        preview_font = (self._mono_font()[0], 8)
        for i in range(NUM_MACROS):
            cell = ttk.Frame(mf)
            cell.grid(row=0, column=i, sticky="nsew", padx=2)
            mf.columnconfigure(i, weight=1, uniform="macro")

            side = ttk.Frame(cell)
            side.pack(side="right", padx=(3, 0))
            self._icon_button(side, "gear", "Edit text to send",
                              lambda i=i: self._edit_macro_text(i)).pack(pady=(0, 3))
            self._icon_button(side, "tag", "Edit button label",
                              lambda i=i: self._edit_macro_label(i)).pack()

            send = tk.Frame(cell, bg=BG_INPUT, highlightthickness=1,
                            highlightbackground=BORDER, cursor="hand2")
            send.pack(side="left", fill="both", expand=True)
            name = tk.Label(send, bg=BG_INPUT, fg=FG, anchor="w")
            name.pack(fill="x", padx=6, pady=(4, 0))
            preview = tk.Label(send, bg=BG_INPUT, fg=FG_DIM, anchor="w",
                               font=preview_font)
            preview.pack(fill="x", padx=6, pady=(0, 4))
            for w in (send, name, preview):
                w.bind("<Button-1>", lambda e, i=i: self.send_macro(i))
                w.bind("<Enter>", lambda e, i=i: self._macro_hover(i, True))
                w.bind("<Leave>", lambda e, i=i: self._macro_hover(i, False))
            self.macro_buttons.append(send)
            self.macro_name_lbls.append(name)
            self.macro_preview_lbls.append(preview)
        self._refresh_macro_buttons()

        bottom = ttk.Frame(self, padding=(8, 4, 8, 6))
        bottom.pack(side="bottom", fill="x")
        ttk.Label(bottom, text="Send:").pack(side="left")
        self.send_entry = ttk.Entry(bottom, textvariable=self.send_var)
        self.send_entry.pack(side="left", fill="x", expand=True, padx=6)
        self.send_entry.bind("<Return>", lambda e: self.send_entry_text())
        self.send_entry.bind("<Up>", self._hist_up)
        self.send_entry.bind("<Down>", self._hist_down)
        self.send_btn = ttk.Button(bottom, text="Send", style="Accent.TButton",
                                   command=self.send_entry_text)
        self.send_btn.pack(side="left")

        self.nb = ttk.Notebook(self)
        self.nb.pack(fill="both", expand=True, padx=8, pady=(8, 0))
        term = ttk.Frame(self.nb, padding=6)
        conf = ttk.Frame(self.nb, padding=12)
        self.nb.add(term, text="Terminal")
        self.nb.add(conf, text="Config")
        self._build_terminal_tab(term)
        self._build_config_tab(conf)

    def _build_terminal_tab(self, parent):
        # Row 1: connect + status + filter
        top = ttk.Frame(parent)
        top.pack(fill="x")
        self.connect_btn = ttk.Button(top, text="Connect", style="Accent.TButton",
                                      width=12, command=self.toggle_connection)
        self.connect_btn.pack(side="left")
        ttk.Checkbutton(top, text="Set RTS/DTR on connect",
                        variable=self.sig_on_connect_var).pack(side="left", padx=(10, 0))
        ttk.Label(top, textvariable=self.status_var, foreground=FG_DIM).pack(
            side="left", padx=10)

        ttk.Button(top, text="✕", width=3, command=lambda: self.filter_var.set("")
                   ).pack(side="right")
        ttk.Checkbutton(top, text="Match case", variable=self.case_var,
                        command=self._schedule_refilter).pack(side="right", padx=6)
        fe = ttk.Entry(top, textvariable=self.filter_var, width=28)
        fe.pack(side="right", padx=4)
        ttk.Label(top, text="Filter:").pack(side="right")
        self.filter_var.trace_add("write", lambda *a: self._schedule_refilter())

        # Row 2: hardware signals + log controls
        sf = ttk.Frame(parent)
        sf.pack(fill="x", pady=(8, 4))
        self.rts_btn = ttk.Checkbutton(sf, text="RTS", style="Toggle.TCheckbutton",
                                       variable=self.rts_var, command=self._apply_rts)
        self.rts_btn.pack(side="left")
        self.dtr_btn = ttk.Checkbutton(sf, text="DTR", style="Toggle.TCheckbutton",
                                       variable=self.dtr_var, command=self._apply_dtr)
        self.dtr_btn.pack(side="left", padx=6)
        self.ind = {}
        for name in ("CTS", "DSR", "RI", "CD"):
            lbl = tk.Label(sf, text=f" {name} ", bg=BG_ALT, fg=FG_DIM,
                           font=("TkDefaultFont", 9, "bold"))
            lbl.pack(side="left", padx=2)
            self.ind[name] = lbl

        ttk.Button(sf, text="Save log…", command=self.save_log).pack(side="right")
        ttk.Button(sf, text="Clear", command=self.clear_log).pack(side="right", padx=6)
        ttk.Checkbutton(sf, text="Autoscroll", variable=self.autoscroll_var
                        ).pack(side="right", padx=6)
        ttk.Checkbutton(sf, text="Echo sent", variable=self.echo_var
                        ).pack(side="right", padx=6)

        # Log view
        lf = ttk.Frame(parent)
        lf.pack(fill="both", expand=True)
        self.text = tk.Text(lf, bg=BG, fg=FG, insertbackground=FG, wrap="none",
                            relief="flat", highlightthickness=1,
                            highlightbackground=BORDER, highlightcolor=BORDER,
                            selectbackground="#264f78", selectforeground="#ffffff",
                            font=self._mono_font(), state="disabled", undo=False)
        vs = ttk.Scrollbar(lf, orient="vertical", command=self.text.yview)
        hs = ttk.Scrollbar(lf, orient="horizontal", command=self.text.xview)
        self.text.configure(yscrollcommand=vs.set, xscrollcommand=hs.set)
        vs.pack(side="right", fill="y")
        hs.pack(side="bottom", fill="x")
        self.text.pack(side="left", fill="both", expand=True)
        bold_font = (self._mono_font()[0], self._mono_font()[1], "bold")
        self.text.tag_configure("bold", font=bold_font)
        self.text.tag_configure("tx", foreground=TX_COLOR)
        self.text.tag_configure("sys", foreground=SYS_COLOR)
        for i, color in enumerate(self.hl_colors):
            self.text.tag_configure(f"hl{i}", background=color)

    def _build_config_tab(self, parent):
        sp = ttk.LabelFrame(parent, text="Serial port", padding=12)
        sp.pack(fill="x")
        sp.columnconfigure(1, weight=1)

        ttk.Label(sp, text="Port").grid(row=0, column=0, sticky="w", pady=4)
        self.port_combo = ttk.Combobox(sp, textvariable=self.port_var)
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=8)
        ttk.Button(sp, text="Refresh", command=self.refresh_ports).grid(row=0, column=2)

        rows = [
            ("Baud rate", self.baud_var, self.BAUDS, False),
            ("Data bits", self.bytesize_var, ["5", "6", "7", "8"], True),
            ("Parity", self.parity_var, list(self.PARITY), True),
            ("Stop bits", self.stopbits_var, list(self.STOPBITS), True),
            ("Flow control", self.flow_var, self.FLOW, True),
            ("Send line ending", self.ending_var, list(TX_ENDINGS), True),
        ]
        for r, (label, var, values, ro) in enumerate(rows, start=1):
            ttk.Label(sp, text=label).grid(row=r, column=0, sticky="w", pady=4)
            cb = ttk.Combobox(sp, textvariable=var, values=values,
                              state="readonly" if ro else "normal", width=18)
            cb.grid(row=r, column=1, sticky="w", padx=8)
        self.settings_note = ttk.Label(
            sp, text="Port settings apply the next time you connect; "
                     "the send line ending applies immediately.",
            foreground=FG_DIM)
        self.settings_note.grid(row=7, column=0, columnspan=3, sticky="w", pady=(8, 0))

        ttk.Button(parent, text="Save settings", command=self._save_and_note
                   ).pack(anchor="e", pady=12)

    def _save_and_note(self):
        self._save_config()
        self.settings_note.configure(text=f"Saved to {CONFIG_PATH}")

    @staticmethod
    def _mono_font():
        import tkinter.font as tkfont
        fams = set(tkfont.families())
        for f in ("Cascadia Mono", "Consolas", "DejaVu Sans Mono", "Ubuntu Mono",
                  "Liberation Mono", "Courier New"):
            if f in fams:
                return (f, 10)
        return ("TkFixedFont", 10)

    # ------------------------------------------------------------ ports
    def refresh_ports(self):
        ports = sorted(list_ports.comports(), key=lambda p: p.device)
        values = [f"{p.device} - {p.description}" if p.description and
                  p.description != "n/a" else p.device for p in ports]
        self.port_combo["values"] = values
        cur = self.port_var.get()
        if not cur and values:
            self.port_var.set(values[0])
        elif cur:
            # re-sync a saved plain device name with its description if present
            dev = cur.split(" - ")[0].strip()
            for v in values:
                if v.split(" - ")[0].strip() == dev:
                    self.port_var.set(v)
                    break

    def _selected_port(self):
        return self.port_var.get().split(" - ")[0].strip()

    # ------------------------------------------------------------ connection
    @property
    def connected(self):
        return self.ser is not None and self.ser.is_open

    def toggle_connection(self):
        if self.connected:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        port = self._selected_port()
        if not port:
            messagebox.showwarning(APP_TITLE, "Select a serial port on the Config tab.")
            self.nb.select(1)
            return
        try:
            baud = int(self.baud_var.get())
            flow = self.flow_var.get()
            ser = serial.Serial()
            ser.port = port
            ser.baudrate = baud
            ser.bytesize = int(self.bytesize_var.get())
            ser.parity = self.PARITY[self.parity_var.get()]
            ser.stopbits = self.STOPBITS[self.stopbits_var.get()]
            ser.timeout = 0.1
            ser.write_timeout = 1
            ser.rtscts = flow == "RTS/CTS"
            ser.dsrdtr = flow == "DSR/DTR"
            ser.xonxoff = flow == "XON/XOFF"
            apply_sig = self.sig_on_connect_var.get()
            is_win = sys.platform.startswith("win")
            # Windows applies RTS/DTR through the DCB at open time, so presetting
            # them before open() is the cleanest option there.
            if apply_sig and is_win:
                if not ser.rtscts:
                    ser.rts = self.rts_var.get()
                if not ser.dsrdtr:
                    ser.dtr = self.dtr_var.get()
            ser.open()
            if apply_sig and not is_win:
                self._apply_signals_ordered(ser)
            elif not apply_sig:
                self._sync_signal_buttons(ser)
        except (serial.SerialException, ValueError, OSError) as e:
            messagebox.showerror(APP_TITLE, f"Could not open {port}:\n{e}")
            return

        self.ser = ser
        self.reader = SerialReader(ser, self.rx_queue)
        self.reader.start()
        self._save_config()
        self._sys(f"Connected to {port} @ {baud} "
                  f"{self.bytesize_var.get()}{self.parity_var.get()[0]}"
                  f"{self.stopbits_var.get()}, flow: {self.flow_var.get()}")
        self.status_var.set(f"Connected: {port} @ {baud}")
        self._update_connection_ui()

    def _apply_signals_ordered(self, ser):
        """Apply the RTS/DTR button states after open (POSIX).

        The OS asserts both lines when a port is opened. Writing DTR before RTS
        would pass through DTR=0/RTS=1, which resets ESP32/ESP8266 auto-reset
        circuits. Lowering RTS first (or raising DTR first) avoids that.
        """
        steps = []
        if not ser.dsrdtr:
            steps.append(("dtr", self.dtr_var.get()))
        if not ser.rtscts:
            steps.append(("rts", self.rts_var.get()))
        if not self.rts_var.get():
            steps.reverse()
        for attr, val in steps:
            setattr(ser, attr, val)

    def _sync_signal_buttons(self, ser):
        """Leave the lines as the OS/driver set them; show that state on the buttons."""
        try:
            if not ser.rtscts:
                self.rts_var.set(bool(ser.rts))
            if not ser.dsrdtr:
                self.dtr_var.set(bool(ser.dtr))
        except (serial.SerialException, OSError):
            pass

    def disconnect(self, reason=None):
        if self.reader:
            self.reader.stop()
            self.reader.join(timeout=1.0)
            self.reader = None
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.status_var.set("Disconnected")
        self._sys(reason or "Disconnected")
        self._update_connection_ui()

    def _update_connection_ui(self):
        on = self.connected
        self.connect_btn.configure(text="Disconnect" if on else "Connect")
        state = "normal" if on else "disabled"
        self.send_btn.configure(state=state)
        for i, send in enumerate(self.macro_buttons):
            send.configure(cursor="hand2" if on else "arrow")
            fg = FG if on else FG_DIM
            self.macro_name_lbls[i].configure(fg=fg)
            self._macro_hover(i, False)
        flow = self.flow_var.get() if not on else (
            "RTS/CTS" if self.ser.rtscts else "DSR/DTR" if self.ser.dsrdtr else "")
        self.rts_btn.configure(state="disabled" if flow == "RTS/CTS" else "normal")
        self.dtr_btn.configure(state="disabled" if flow == "DSR/DTR" else "normal")
        if not on:
            for l in self.ind.values():
                l.configure(bg=BG_ALT, fg=FG_DIM)

    def _apply_rts(self):
        if self.connected and not self.ser.rtscts:
            try:
                self.ser.rts = self.rts_var.get()
            except (serial.SerialException, OSError) as e:
                self._sys(f"RTS error: {e}")

    def _apply_dtr(self):
        if self.connected and not self.ser.dsrdtr:
            try:
                self.ser.dtr = self.dtr_var.get()
            except (serial.SerialException, OSError) as e:
                self._sys(f"DTR error: {e}")

    def _poll_modem_lines(self):
        if self.connected:
            try:
                states = {"CTS": self.ser.cts, "DSR": self.ser.dsr,
                          "RI": self.ser.ri, "CD": self.ser.cd}
                for name, v in states.items():
                    self.ind[name].configure(bg="#2e7d32" if v else BG_ALT,
                                             fg="#ffffff" if v else FG_DIM)
            except (serial.SerialException, OSError):
                pass
        self.after(250, self._poll_modem_lines)

    # ------------------------------------------------------------ sending
    def _write_line(self, text):
        if not self.connected:
            return False
        try:
            ending = TX_ENDINGS.get(self.ending_var.get(), LINE_END)
            self.ser.write((text + ending).encode("utf-8", errors="replace"))
        except (serial.SerialException, OSError) as e:
            self.disconnect(f"Write failed: {e}")
            return False
        if self.echo_var.get():
            self._add_line("tx", text)
        return True

    def send_entry_text(self):
        text = self.send_var.get()
        if self._write_line(text):
            if text and (not self.history or self.history[-1] != text):
                self.history.append(text)
            self.hist_idx = len(self.history)
            self.send_var.set("")

    def send_macro(self, i):
        if not self.connected:
            return
        text = self.macro_vars[i][1].get()
        if not text:
            self._sys(f"Macro {i + 1} has no text configured (click the gear icon).")
            return
        self._write_line(text)

    def _icon_button(self, parent, kind, tooltip, command):
        size = 16
        cv = tk.Canvas(parent, width=size, height=size, bg=BG,
                       highlightthickness=0, cursor="hand2", borderwidth=0)
        self._draw_icon(cv, kind, size, FG_DIM, BG)
        tip = {"win": None}

        def redraw(color, bg):
            cv.configure(bg=bg)
            self._draw_icon(cv, kind, size, color, bg)

        def show_tip():
            if tip["win"] is not None:
                return
            w = tk.Toplevel(cv)
            w.wm_overrideredirect(True)
            w.configure(bg=BORDER)
            tk.Label(w, text=tooltip, bg=BG_ALT, fg=FG, padx=6, pady=2).pack()
            w.geometry(f"+{cv.winfo_rootx()}+{cv.winfo_rooty() + size + 6}")
            tip["win"] = w

        def hide_tip():
            if tip["win"] is not None:
                tip["win"].destroy()
                tip["win"] = None

        def enter(_e):
            redraw(FG, "#3e3e42")
            show_tip()

        def leave(_e):
            redraw(FG_DIM, BG)
            hide_tip()

        def click(_e):
            hide_tip()
            # Open after this click finishes so the same mouse-up cannot
            # dismiss the dialog before the user types anything.
            self.after_idle(command)

        cv.bind("<Button-1>", click)
        cv.bind("<Enter>", enter)
        cv.bind("<Leave>", leave)
        return cv

    @staticmethod
    def _draw_icon(cv, kind, size, color, bg):
        cv.delete("all")
        cx = cy = size / 2.0
        if kind == "gear":
            teeth, r_tip, r_root, r_hole = 6, size * 0.46, size * 0.30, size * 0.10
            pts = []
            da = 2 * math.pi / teeth
            for t in range(teeth):
                a = t * da
                for ang, r in (
                    (a - da * 0.18, r_root),
                    (a - da * 0.10, r_tip),
                    (a + da * 0.10, r_tip),
                    (a + da * 0.18, r_root),
                ):
                    pts.extend((cx + r * math.cos(ang), cy + r * math.sin(ang)))
            cv.create_polygon(pts, fill=color, outline=color, smooth=False)
            cv.create_oval(cx - r_hole, cy - r_hole, cx + r_hole, cy + r_hole,
                           fill=bg, outline=bg)
        else:  # tag
            pts = (2, 3, 10, 3, 14, 8, 10, 13, 2, 13)
            cv.create_polygon(pts, fill=color, outline=color)
            cv.create_oval(4, 6.5, 7.2, 9.7, fill=bg, outline=bg)

    def _macro_hover(self, i, on):
        if not getattr(self, "macro_buttons", None) or i >= len(self.macro_buttons):
            return
        send = self.macro_buttons[i]
        if not on:
            try:
                x = send.winfo_pointerx() - send.winfo_rootx()
                y = send.winfo_pointery() - send.winfo_rooty()
                if 0 <= x < send.winfo_width() and 0 <= y < send.winfo_height():
                    return
            except tk.TclError:
                pass
        bg = "#3e3e42" if on and self.connected else BG_INPUT
        send.configure(bg=bg)
        self.macro_name_lbls[i].configure(bg=bg)
        self.macro_preview_lbls[i].configure(bg=bg)

    def _ask_string(self, title, prompt, initial=""):
        win = tk.Toplevel(self)
        win.title(title)
        win.configure(bg=BG)
        win.transient(self)
        win.resizable(True, False)
        ttk.Label(win, text=prompt).pack(padx=12, pady=(12, 4), anchor="w")
        var = tk.StringVar(value=initial)
        ent = ttk.Entry(win, textvariable=var, width=48)
        ent.pack(fill="x", padx=12)
        result = {"value": None}

        def ok(_e=None):
            result["value"] = var.get()
            win.destroy()
            return "break"

        def cancel(_e=None):
            win.destroy()
            return "break"

        bf = ttk.Frame(win)
        bf.pack(pady=12)
        ttk.Button(bf, text="OK", style="Accent.TButton", command=ok).pack(
            side="left", padx=4)
        ttk.Button(bf, text="Cancel", command=cancel).pack(side="left")
        ent.bind("<Return>", ok)
        win.bind("<Return>", ok)
        win.bind("<Escape>", cancel)
        win.protocol("WM_DELETE_WINDOW", cancel)
        win.update_idletasks()
        px = self.winfo_rootx() + max(0, (self.winfo_width() - win.winfo_reqwidth()) // 2)
        py = self.winfo_rooty() + max(0, (self.winfo_height() - win.winfo_reqheight()) // 3)
        win.geometry(f"+{px}+{py}")
        ent.focus_set()
        ent.icursor("end")
        try:
            win.grab_set()
        except tk.TclError:
            pass
        win.wait_window()
        return result["value"]

    def _edit_macro_text(self, i):
        value = self._ask_string(
            f"Edit macro {i + 1}",
            "Text to send (the selected line ending is appended):",
            self.macro_vars[i][1].get())
        if value is None:
            return
        self.macro_vars[i][1].set(value)
        self._refresh_macro_buttons()
        self._save_config()

    def _edit_macro_label(self, i):
        value = self._ask_string(
            f"Label macro {i + 1}",
            "Button label:",
            self.macro_vars[i][0].get())
        if value is None:
            return
        self.macro_vars[i][0].set(value)
        self._refresh_macro_buttons()
        self._save_config()

    @staticmethod
    def _macro_preview_text(text):
        shown = text.replace("\r", "\\r").replace("\n", "\\n")
        if len(shown) > 20:
            shown = shown[:19] + "…"
        return f"`{shown}`"

    def _refresh_macro_buttons(self):
        names = getattr(self, "macro_name_lbls", [])
        previews = getattr(self, "macro_preview_lbls", [])
        for i, name in enumerate(names):
            label = self.macro_vars[i][0].get().strip() or f"Macro {i + 1}"
            name.configure(text=label)
            previews[i].configure(text=self._macro_preview_text(
                self.macro_vars[i][1].get()))

    def _hist_up(self, _e):
        if self.history and self.hist_idx > 0:
            self.hist_idx -= 1
            self.send_var.set(self.history[self.hist_idx])
            self.send_entry.icursor("end")
        return "break"

    def _hist_down(self, _e):
        if self.hist_idx < len(self.history) - 1:
            self.hist_idx += 1
            self.send_var.set(self.history[self.hist_idx])
        else:
            self.hist_idx = len(self.history)
            self.send_var.set("")
        self.send_entry.icursor("end")
        return "break"

    # ------------------------------------------------------------ log handling
    def _sys(self, msg):
        self._add_line("sys", msg)

    def _add_line(self, kind, text):
        self.lines.append((kind, text))
        if self._matches(kind, text):
            self._insert([(kind, text)])

    def _poll_queue(self):
        batch, err = [], None
        try:
            for _ in range(2000):
                kind, payload = self.rx_queue.get_nowait()
                if kind == "line":
                    self.lines.append(("rx", payload))
                    if self._matches("rx", payload):
                        batch.append(("rx", payload))
                elif kind == "error":
                    err = payload
        except queue.Empty:
            pass
        if batch:
            self._insert(batch)
        if err and self.connected:
            self.disconnect(f"Connection lost: {err}")
        self.after(30, self._poll_queue)

    def _matches(self, kind, text):
        if kind == "sys":
            return True
        needle = self.filter_var.get()
        if not needle:
            return True
        hay = strip_ansi(text)
        if not self.case_var.get():
            return needle.lower() in hay.lower()
        return needle in hay

    def _fg_tag(self, color):
        name = f"fg_{color}"
        if name not in self.fg_tags:
            self.text.tag_configure(name, foreground=color)
            self.fg_tags.add(name)
        return name

    def _insert(self, items):
        t = self.text
        t.configure(state="normal")
        pats = self._active_highlights()
        for kind, text in items:
            line_no = int(t.index("end-1c").split(".")[0])
            if kind == "rx":
                disp = []
                for seg, fg, bold in parse_ansi(text.replace("\r", "")):
                    tags = []
                    if fg:
                        tags.append(self._fg_tag(fg))
                    if bold:
                        tags.append("bold")
                    t.insert("end", seg, tuple(tags))
                    disp.append(seg)
                disp = "".join(disp)
            elif kind == "tx":
                disp = "TX> " + text
                t.insert("end", disp, "tx")
            else:
                disp = "-- " + text + " --"
                t.insert("end", disp, "sys")
            t.insert("end", "\n")
            if pats:
                self._tag_highlights(line_no, disp, pats)
        total = int(t.index("end-1c").split(".")[0])
        if total > MAX_DISPLAY_LINES:
            t.delete("1.0", f"{total - MAX_DISPLAY_LINES}.0")
        t.configure(state="disabled")
        if self.autoscroll_var.get():
            t.see("end")

    # ------------------------------------------------------------ highlights
    def _active_highlights(self):
        """[(index, compiled case-insensitive regex)] for enabled, non-empty slots."""
        pats = []
        for i, (en, tx) in enumerate(self.hl_vars):
            word = tx.get()
            if en.get() and word:
                pats.append((i, re.compile(re.escape(word), re.IGNORECASE)))
        return pats

    def _tag_highlights(self, line_no, disp, pats):
        t = self.text
        for k, sub in enumerate(disp.split("\n")):
            for i, rx in pats:
                for m in rx.finditer(sub):
                    t.tag_add(f"hl{i}", f"{line_no + k}.{m.start()}",
                              f"{line_no + k}.{m.end()}")

    def _schedule_rehighlight(self):
        if self._hl_job:
            self.after_cancel(self._hl_job)
        self._hl_job = self.after(150, self._rehighlight)

    def _rehighlight(self):
        """Re-scan everything currently shown (enable/disable or text changed)."""
        self._hl_job = None
        t = self.text
        for i in range(NUM_HIGHLIGHTS):
            t.tag_remove(f"hl{i}", "1.0", "end")
        pats = self._active_highlights()
        if not pats:
            return
        for n, line in enumerate(t.get("1.0", "end-1c").split("\n"), start=1):
            self._tag_highlights(n, line, pats)

    def _pick_highlight_color(self, i):
        from tkinter import colorchooser
        _rgb, hexcolor = colorchooser.askcolor(
            color=self.hl_colors[i], parent=self, title=f"Highlight {i + 1} color")
        if not hexcolor:
            return
        self.hl_colors[i] = hexcolor
        self.hl_btns[i].configure(bg=hexcolor, activebackground=hexcolor)
        self.text.tag_configure(f"hl{i}", background=hexcolor)   # updates live
        self._save_config()

    def _schedule_refilter(self):
        if self._filter_job:
            self.after_cancel(self._filter_job)
        self._filter_job = self.after(200, self._refilter)

    def _refilter(self):
        self._filter_job = None
        self.text.configure(state="normal")
        self.text.delete("1.0", "end")
        self.text.configure(state="disabled")
        matched = deque((it for it in self.lines if self._matches(*it)),
                        maxlen=MAX_DISPLAY_LINES)
        self._insert(list(matched))

    def clear_log(self):
        self.lines.clear()
        self.text.configure(state="normal")
        self.text.delete("1.0", "end")
        self.text.configure(state="disabled")

    def save_log(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".log",
            filetypes=[("Log files", "*.log"), ("Text files", "*.txt"), ("All", "*.*")])
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8", newline="\n") as f:
                for kind, text in self.lines:
                    prefix = "TX> " if kind == "tx" else "-- " if kind == "sys" else ""
                    f.write(prefix + strip_ansi(text) + "\n")
        except OSError as e:
            messagebox.showerror(APP_TITLE, f"Could not save log:\n{e}")

    # ------------------------------------------------------------ shutdown
    def on_close(self):
        self._save_config()
        if self.connected:
            self.disconnect()
        self.destroy()


if __name__ == "__main__":
    SerialApp().mainloop()