"""
RSI TestServer - Python equivalent of KUKA's RSI TestServer.exe

Simulates the robot side of an RSI connection. Listens on UDP for <Sen> XML
from a client, and responds with <Rob> XML containing the current robot state.

Usage:
    python test_server.py
    python test_server.py --config path/to/RSI_EthernetConfig.xml
"""

import socket
import threading
import tkinter as tk
from tkinter import messagebox
import xml.etree.ElementTree as ET
import argparse
import copy
import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))
from RSIPI.config_parser import ConfigParser
from RSIPI.xml_handler import XMLGenerator

# --- Theme colours ---
BG_DARK = "#1e1e2e"
BG_MID = "#282840"
BG_CARD = "#313150"
BG_INPUT = "#3b3b5c"
FG = "#cdd6f4"
FG_DIM = "#7f849c"
FG_BRIGHT = "#ffffff"
ACCENT = "#f5a623"
ACCENT_HOVER = "#f7bc5e"
GREEN = "#a6e3a1"
RED = "#f38ba8"
BLUE = "#89b4fa"
BORDER = "#45475a"
AXIS_COLOURS = {
    "X": "#f38ba8", "Y": "#a6e3a1", "Z": "#89b4fa",
    "A": "#fab387", "B": "#cba6f7", "C": "#94e2d5",
}


class RSITestServer:
    """UDP server simulating a KUKA robot controller for RSI testing."""

    def __init__(self, config_file: str):
        self.config = ConfigParser(config_file)
        self.network_settings = self.config.get_network_settings()
        self.port = self.network_settings["port"]

        self.robot_state = copy.deepcopy(self.config.send_variables)
        self.receive_template = copy.deepcopy(self.config.receive_variables)

        self.ipoc = 0
        self.correction_step = 0.01
        self.running = False
        self.receive_only = False
        self.udp_socket = None
        self.listen_thread = None
        self.client_address = None
        self.last_received_xml = ""
        self.last_sent_xml = ""
        self.packet_count = 0
        self.lock = threading.Lock()

    def start(self):
        if self.running:
            return
        try:
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.udp_socket.bind(("0.0.0.0", self.port))
            self.udp_socket.settimeout(0.5)
            self.running = True
            self.packet_count = 0
            self.listen_thread = threading.Thread(target=self._listen_loop, daemon=True)
            self.listen_thread.start()
            return True
        except OSError as e:
            self.running = False
            return str(e)

    def stop(self):
        self.running = False
        if self.listen_thread:
            self.listen_thread.join(timeout=2)
        if self.udp_socket:
            self.udp_socket.close()
            self.udp_socket = None
        self.client_address = None

    def _listen_loop(self):
        while self.running:
            try:
                data, addr = self.udp_socket.recvfrom(4096)
                self.client_address = addr
                xml_string = data.decode("utf-8", errors="replace")

                with self.lock:
                    self.last_received_xml = xml_string
                    self._process_received(xml_string)
                    self.packet_count += 1

                    if not self.receive_only:
                        response = self._generate_response()
                        self.last_sent_xml = response
                        self.udp_socket.sendto(response.encode("utf-8"), addr)

            except socket.timeout:
                continue
            except OSError:
                break
            except Exception as e:
                if self.running:
                    print(f"[TestServer] Error: {e}")

    def _process_received(self, xml_string: str):
        try:
            root = ET.fromstring(xml_string)
            ipoc_elem = root.find("IPOC")
            if ipoc_elem is not None and ipoc_elem.text:
                self.ipoc = int(ipoc_elem.text.strip())
        except ET.ParseError:
            self.ipoc = self._extract_ipoc_string(xml_string)

    def _extract_ipoc_string(self, xml_string: str) -> int:
        end_tag = "</IPOC>"
        start_tag = "<IPOC>"
        end_idx = xml_string.find(end_tag)
        if end_idx == -1:
            return self.ipoc
        start_idx = xml_string.find(start_tag)
        if start_idx == -1:
            return self.ipoc
        try:
            return int(xml_string[start_idx + len(start_tag):end_idx].strip())
        except ValueError:
            return self.ipoc

    def _generate_response(self) -> str:
        state = copy.deepcopy(self.robot_state)
        state["IPOC"] = self.ipoc
        return XMLGenerator.generate_receive_xml(state)

    def adjust_position(self, axis: str, delta: float):
        with self.lock:
            if "RIst" in self.robot_state and axis in self.robot_state["RIst"]:
                self.robot_state["RIst"][axis] += delta

    def reset_positions(self):
        with self.lock:
            if "RIst" in self.robot_state:
                for axis in self.robot_state["RIst"]:
                    self.robot_state["RIst"][axis] = 0.0


class TestServerGUI:
    """Dark-themed tkinter GUI for the RSI TestServer."""

    def __init__(self, config_file: str):
        self.server = RSITestServer(config_file)
        self._prev_packet_count = 0
        self._prev_time = time.time()
        self._packets_per_sec = 0.0
        self.recv_text: tk.Text = None  # type: ignore[assignment]
        self.sent_text: tk.Text = None  # type: ignore[assignment]

        self.root = tk.Tk()
        self.root.title("RSI TestServer")
        self.root.configure(bg=BG_DARK)
        self.root.geometry("1100x750")
        self.root.resizable(True, True)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._build_ui()
        self._update_display()

    # ── helpers ──────────────────────────────────────────────────────

    def _card(self, parent, **kw):
        f = tk.Frame(parent, bg=BG_CARD, highlightbackground=BORDER,
                     highlightthickness=1, **kw)
        return f

    def _label(self, parent, text="", font=("Segoe UI", 10), fg=FG, **kw):
        return tk.Label(parent, text=text, font=font, fg=fg, bg=parent["bg"], **kw)

    def _btn(self, parent, text, command, width=4, accent=False, **kw):
        bg = ACCENT if accent else BG_INPUT
        fg_c = BG_DARK if accent else FG
        hover = ACCENT_HOVER if accent else "#4e4e7a"
        b = tk.Button(parent, text=text, command=command, width=width,
                      font=("Segoe UI", 10, "bold"), fg=fg_c, bg=bg,
                      activeforeground=fg_c, activebackground=hover,
                      relief="flat", cursor="hand2", bd=0,
                      highlightthickness=0, **kw)
        b.bind("<Enter>", lambda e, b=b, h=hover: b.config(bg=h))
        b.bind("<Leave>", lambda e, b=b, n=bg: b.config(bg=n))
        return b

    # ── layout ───────────────────────────────────────────────────────

    def _build_ui(self):
        # ── Header ──
        header = tk.Frame(self.root, bg=BG_MID, height=56)
        header.grid(row=0, column=0, sticky="ew")
        header.grid_columnconfigure(1, weight=1)

        self._label(header, "RSI TestServer",
                    font=("Segoe UI", 16, "bold"), fg=ACCENT).grid(
            row=0, column=0, padx=16, pady=10, sticky="w")

        self._label(header, f"Port {self.server.port}",
                    font=("Segoe UI Semibold", 11), fg=FG_DIM).grid(
            row=0, column=1, sticky="w")

        self.status_dot = tk.Canvas(header, width=14, height=14,
                                    bg=BG_MID, highlightthickness=0)
        self.status_dot.create_oval(2, 2, 12, 12, fill=RED, outline="", tags="dot")
        self.status_dot.grid(row=0, column=2, padx=(0, 4))

        self.status_label = self._label(header, "Offline",
                                        font=("Segoe UI Semibold", 11), fg=RED)
        self.status_label.grid(row=0, column=3, padx=(0, 16))

        # ── Body ──
        body = tk.Frame(self.root, bg=BG_DARK)
        body.grid(row=1, column=0, sticky="nsew", padx=12, pady=8)
        body.grid_columnconfigure(1, weight=1)

        # ── Left column: controls + jog ──
        left = tk.Frame(body, bg=BG_DARK)
        left.grid(row=0, column=0, sticky="n", padx=(0, 8))

        # Controls card
        ctrl = self._card(left)
        ctrl.grid(row=0, column=0, sticky="ew", pady=(0, 8))

        btn_row = tk.Frame(ctrl, bg=BG_CARD)
        btn_row.grid(row=0, column=0, padx=10, pady=10, sticky="ew")

        self.start_btn = self._btn(btn_row, "Start", self._start,
                                   width=8, accent=True)
        self.start_btn.grid(row=0, column=0, padx=(0, 4))

        self.stop_btn = self._btn(btn_row, "Stop", self._stop, width=8)
        self.stop_btn.config(state="disabled")
        self.stop_btn.grid(row=0, column=1, padx=(0, 12))

        self.mode_var = tk.StringVar(value="Send + Recv")
        mode_btn = tk.Menubutton(btn_row, textvariable=self.mode_var,
                                 font=("Segoe UI", 9), fg=FG, bg=BG_INPUT,
                                 activeforeground=FG, activebackground="#4e4e7a",
                                 relief="flat", highlightthickness=0,
                                 indicatoron=True, cursor="hand2", width=12)
        mode_menu = tk.Menu(mode_btn, tearoff=0, bg=BG_INPUT, fg=FG,
                            activebackground=ACCENT, activeforeground=BG_DARK)
        mode_menu.add_command(label="Send + Recv",
                              command=lambda: self._set_mode("Send + Recv"))
        mode_menu.add_command(label="Recv only",
                              command=lambda: self._set_mode("Recv only"))
        mode_btn["menu"] = mode_menu
        mode_btn.grid(row=0, column=2, padx=(0, 8))

        self._label(btn_row, "Step:", font=("Segoe UI", 9), fg=FG_DIM).grid(
            row=0, column=3)
        self.step_var = tk.StringVar(value="0.01")
        step_e = tk.Entry(btn_row, textvariable=self.step_var, width=7,
                          font=("Consolas", 10), fg=ACCENT, bg=BG_INPUT,
                          insertbackground=ACCENT, relief="flat",
                          highlightthickness=1, highlightcolor=ACCENT,
                          highlightbackground=BORDER)
        step_e.grid(row=0, column=4, padx=4)
        step_e.bind("<Return>", self._step_changed)
        step_e.bind("<FocusOut>", self._step_changed)

        # Jog card
        jog = self._card(left)
        jog.grid(row=1, column=0, sticky="ew")

        jog_title = tk.Frame(jog, bg=BG_CARD)
        jog_title.grid(row=0, column=0, columnspan=4, sticky="ew", padx=10,
                       pady=(8, 4))
        self._label(jog_title, "Jog Control",
                    font=("Segoe UI Semibold", 11), fg=FG_BRIGHT).pack(
            side="left")

        axes = ["X", "Y", "Z", "A", "B", "C"]
        self.pos_labels = {}

        for i, axis in enumerate(axes):
            row_frame = tk.Frame(jog, bg=BG_CARD)
            row_frame.grid(row=i + 1, column=0, sticky="ew", padx=10, pady=2)
            row_frame.grid_columnconfigure(1, weight=1)

            colour = AXIS_COLOURS[axis]

            # Colour pip
            pip = tk.Canvas(row_frame, width=8, height=8,
                            bg=BG_CARD, highlightthickness=0)
            pip.create_oval(1, 1, 7, 7, fill=colour, outline="")
            pip.grid(row=0, column=0, padx=(0, 6))

            self._label(row_frame, f"{axis}",
                        font=("Segoe UI Semibold", 11), fg=colour).grid(
                row=0, column=1, sticky="w")

            self.pos_labels[axis] = self._label(
                row_frame, "0.0000",
                font=("Consolas", 12), fg=FG_BRIGHT)
            self.pos_labels[axis].grid(row=0, column=2, padx=8)

            self._btn(row_frame, "+",
                      lambda a=axis: self._adjust(a, 1), width=3).grid(
                row=0, column=3, padx=1)
            self._btn(row_frame, "-",
                      lambda a=axis: self._adjust(a, -1), width=3).grid(
                row=0, column=4, padx=1)

        reset_row = tk.Frame(jog, bg=BG_CARD)
        reset_row.grid(row=len(axes) + 1, column=0, sticky="ew",
                       padx=10, pady=(6, 10))
        self._btn(reset_row, "Reset All", self._reset,
                  width=36, accent=False).pack(fill="x")

        # ── Right column: state + xml ──
        right = tk.Frame(body, bg=BG_DARK)
        right.grid(row=0, column=1, sticky="nsew")
        right.grid_rowconfigure(1, weight=1)

        # Info bar
        info = self._card(right)
        info.grid(row=0, column=0, sticky="ew", pady=(0, 8))

        info_inner = tk.Frame(info, bg=BG_CARD)
        info_inner.grid(padx=10, pady=8, sticky="ew")
        info.grid_columnconfigure(0, weight=1)
        info_inner.grid_columnconfigure(1, weight=1)
        info_inner.grid_columnconfigure(3, weight=1)

        self._label(info_inner, "IPOC", font=("Segoe UI", 9), fg=FG_DIM).grid(
            row=0, column=0, sticky="w")
        self.ipoc_val = self._label(info_inner, "0",
                                    font=("Consolas", 12, "bold"), fg=BLUE)
        self.ipoc_val.grid(row=1, column=0, sticky="w")

        self._label(info_inner, "Client", font=("Segoe UI", 9), fg=FG_DIM).grid(
            row=0, column=1, sticky="w", padx=(20, 0))
        self.client_val = self._label(info_inner, "--",
                                      font=("Consolas", 11), fg=FG)
        self.client_val.grid(row=1, column=1, sticky="w", padx=(20, 0))

        self._label(info_inner, "Packets/s", font=("Segoe UI", 9), fg=FG_DIM).grid(
            row=0, column=2, sticky="w", padx=(20, 0))
        self.pps_val = self._label(info_inner, "0",
                                   font=("Consolas", 12, "bold"), fg=GREEN)
        self.pps_val.grid(row=1, column=2, sticky="w", padx=(20, 0))

        # Robot state card — grid layout
        state_card = self._card(right)
        state_card.grid(row=1, column=0, sticky="nsew", pady=(0, 8))

        state_hdr = tk.Frame(state_card, bg=BG_CARD)
        state_hdr.grid(row=0, column=0, sticky="ew", padx=10, pady=(8, 4))
        self._label(state_hdr, "Robot State",
                    font=("Segoe UI Semibold", 11), fg=FG_BRIGHT).pack(
            side="left")

        # Scrollable grid area
        state_outer = tk.Frame(state_card, bg=BG_MID)
        state_outer.grid(row=1, column=0, sticky="nsew", padx=8, pady=(0, 8))
        state_card.grid_rowconfigure(1, weight=1)

        state_canvas = tk.Canvas(state_outer, bg=BG_MID, highlightthickness=0,
                                 height=280)
        state_canvas.pack(side="left", fill="both", expand=True)
        state_scroll = tk.Scrollbar(state_outer, orient="vertical",
                                    command=state_canvas.yview,
                                    bg=BG_MID, troughcolor=BG_MID)
        state_scroll.pack(side="right", fill="y")
        state_canvas.configure(yscrollcommand=state_scroll.set)

        self._state_grid = tk.Frame(state_canvas, bg=BG_MID)
        state_canvas.create_window((0, 0), window=self._state_grid, anchor="nw")
        self._state_canvas = state_canvas

        # Build grid from robot_state
        self._state_cells = {}  # (group, subkey) -> Label
        self._build_state_grid()

        # XML card
        xml_card = self._card(right)
        xml_card.grid(row=2, column=0, sticky="nsew")

        xml_hdr = tk.Frame(xml_card, bg=BG_CARD)
        xml_hdr.grid(row=0, column=0, columnspan=2, sticky="ew", padx=10,
                     pady=(8, 4))
        self._label(xml_hdr, "XML Traffic",
                    font=("Segoe UI Semibold", 11), fg=FG_BRIGHT).pack(
            side="left")

        xml_card.grid_columnconfigure(1, weight=1)
        for i, (lbl, attr) in enumerate([("RX", "recv_text"), ("TX", "sent_text")]):
            tag_bg = GREEN if lbl == "RX" else BLUE
            tag = tk.Label(xml_card, text=f" {lbl} ", font=("Consolas", 9, "bold"),
                           fg=BG_DARK, bg=tag_bg)
            tag.grid(row=i + 1, column=0, sticky="nw", padx=(10, 4), pady=2)
            t = tk.Text(xml_card, width=60, height=6, font=("Consolas", 9),
                        fg=FG_DIM, bg=BG_MID, relief="flat", bd=0,
                        padx=6, pady=4, wrap="word", state="disabled")
            t.grid(row=i + 1, column=1, sticky="nsew", padx=(0, 8), pady=2)
            setattr(self, attr, t)

        # bottom pad
        tk.Frame(xml_card, bg=BG_CARD, height=8).grid(
            row=3, column=0, columnspan=2)

    # ── state grid builder ────────────────────────────────────────────

    def _build_state_grid(self):
        """Build the header row and value cells from robot_state structure."""
        grid = self._state_grid
        row = 0

        for key, value in self.server.robot_state.items():
            if isinstance(value, dict):
                # Group header
                self._label(grid, key, font=("Segoe UI Semibold", 9),
                            fg=ACCENT).grid(row=row, column=0, sticky="w",
                                            padx=(8, 4), pady=(6, 0))
                row += 1

                # Sub-key headers + value cells in columns
                subkeys = list(value.keys())
                # Chunk into rows of 6 for wide dicts like Tech
                chunk_size = 6
                for chunk_start in range(0, len(subkeys), chunk_size):
                    chunk = subkeys[chunk_start:chunk_start + chunk_size]
                    for col, sk in enumerate(chunk):
                        # Header
                        self._label(grid, sk, font=("Consolas", 8),
                                    fg=FG_DIM).grid(
                            row=row, column=col + 1, padx=4, sticky="e")
                    row += 1
                    for col, sk in enumerate(chunk):
                        # Value cell
                        cell = self._label(grid, "0.0000",
                                           font=("Consolas", 10), fg=FG_BRIGHT)
                        cell.grid(row=row, column=col + 1, padx=4, sticky="e")
                        self._state_cells[(key, sk)] = cell
                    row += 1
            else:
                # Scalar value
                self._label(grid, key, font=("Segoe UI Semibold", 9),
                            fg=ACCENT).grid(row=row, column=0, sticky="w",
                                            padx=(8, 4), pady=(6, 0))
                cell = self._label(grid, str(value),
                                   font=("Consolas", 10), fg=FG_BRIGHT)
                cell.grid(row=row, column=1, padx=4, sticky="e")
                self._state_cells[(key, None)] = cell
                row += 1

        # Update scroll region after grid is built
        grid.update_idletasks()
        self._state_canvas.configure(scrollregion=self._state_canvas.bbox("all"))

    def _update_state_grid(self):
        """Update all cell values from current robot_state."""
        for key, value in self.server.robot_state.items():
            if isinstance(value, dict):
                for sk, sv in value.items():
                    cell = self._state_cells.get((key, sk))
                    if cell:
                        try:
                            cell.config(text=f"{float(sv):>8.4f}")
                        except (ValueError, TypeError):
                            cell.config(text=str(sv))
            else:
                cell = self._state_cells.get((key, None))
                if cell:
                    try:
                        cell.config(text=f"{float(value):>8.4f}")
                    except (ValueError, TypeError):
                        cell.config(text=str(value))

    # ── actions ──────────────────────────────────────────────────────

    def _start(self):
        result = self.server.start()
        if result is True:
            self.status_label.config(text="Online", fg=GREEN)
            self.status_dot.itemconfig("dot", fill=GREEN)
            self.start_btn.config(state="disabled", bg=BG_INPUT, cursor="arrow")
            self.stop_btn.config(state="normal", cursor="hand2")
            self._prev_packet_count = 0
            self._prev_time = time.time()
        else:
            messagebox.showerror("Error",
                                 f"Port {self.server.port} is already in use.\n{result}")

    def _stop(self):
        self.server.stop()
        self.status_label.config(text="Offline", fg=RED)
        self.status_dot.itemconfig("dot", fill=RED)
        self.client_val.config(text="--")
        self.pps_val.config(text="0")
        self.start_btn.config(state="normal", bg=ACCENT, cursor="hand2")
        self.stop_btn.config(state="disabled", cursor="arrow")

    def _set_mode(self, mode):
        self.mode_var.set(mode)
        self.server.receive_only = (mode == "Recv only")

    def _step_changed(self, event=None):
        try:
            self.server.correction_step = float(self.step_var.get())
        except ValueError:
            self.step_var.set(str(self.server.correction_step))

    def _adjust(self, axis: str, direction: int):
        self.server.adjust_position(axis, direction * self.server.correction_step)

    def _reset(self):
        self.server.reset_positions()

    # ── display loop ─────────────────────────────────────────────────

    def _update_display(self):
        with self.server.lock:
            # Position values
            rist = self.server.robot_state.get("RIst", {})
            for axis, label in self.pos_labels.items():
                val = rist.get(axis, 0.0)
                label.config(text=f"{val:+.4f}")

            # IPOC
            self.ipoc_val.config(text=str(self.server.ipoc))

            # Client
            if self.server.client_address:
                self.client_val.config(
                    text=f"{self.server.client_address[0]}:{self.server.client_address[1]}")

            # Packets/sec
            now = time.time()
            dt = now - self._prev_time
            if dt >= 1.0:
                delta = self.server.packet_count - self._prev_packet_count
                self._packets_per_sec = delta / dt
                self._prev_packet_count = self.server.packet_count
                self._prev_time = now
            self.pps_val.config(text=f"{self._packets_per_sec:.0f}")

            # Robot state grid
            self._update_state_grid()

            # XML traffic (pretty-printed)
            self._set_xml(self.recv_text, self.server.last_received_xml)
            self._set_xml(self.sent_text, self.server.last_sent_xml)

        self.root.after(100, self._update_display)

    @staticmethod
    def _set_text(widget, text):
        widget.config(state="normal")
        widget.delete("1.0", "end")
        widget.insert("1.0", text)
        widget.config(state="disabled")

    @staticmethod
    def _set_xml(widget, xml_string):
        """Pretty-print XML into a text widget."""
        formatted = xml_string
        if xml_string.strip():
            try:
                root = ET.fromstring(xml_string)
                ET.indent(root)
                formatted = ET.tostring(root, encoding="unicode")
            except ET.ParseError:
                pass
        widget.config(state="normal")
        widget.delete("1.0", "end")
        widget.insert("1.0", formatted)
        widget.config(state="disabled")

    def _on_close(self):
        self.server.stop()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    parser = argparse.ArgumentParser(description="RSI TestServer - KUKA Robot Simulator")
    parser.add_argument(
        "--config", type=str, default="RSI_EthernetConfig.xml",
        help="Path to RSI_EthernetConfig.xml"
    )
    args = parser.parse_args()

    if not os.path.exists(args.config):
        print(f"Error: Config file not found: {args.config}")
        sys.exit(1)

    app = TestServerGUI(args.config)
    app.run()


if __name__ == "__main__":
    main()
