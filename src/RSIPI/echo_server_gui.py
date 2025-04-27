import tkinter as tk
from tkinter import ttk, filedialog
import threading
import time
from src.RSIPI.rsi_echo_server import EchoServer
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import os
import math


class EchoServerGUI:
    """
    Graphical interface for running and visualising the RSI Echo Server.
    Provides live feedback of robot TCP position and joint states, along with XML message logs.
    """

    def __init__(self, master):
        """
        Initialises the GUI, default values, and layout.

        Args:
            master (tk.Tk): Root tkinter window.
        """
        self.master = master
        self.master.title("RSI Echo Server GUI")
        self.master.geometry("1300x800")

        # Configurable input variables
        self.config_file = tk.StringVar(value="RSI_EthernetConfig.xml")
        self.mode = tk.StringVar(value="relative")
        self.delay = tk.IntVar(value=4)
        self.show_robot = tk.BooleanVar(value=True)

        # Internal state
        self.server = None
        self.visual_thread = None
        self.running = False
        self.trace = []

        self.create_widgets()

    def create_widgets(self):
        """Create and layout all UI elements including buttons, entry fields, and plots."""
        frame = ttk.Frame(self.master)
        frame.pack(pady=10)

        # Config file input
        ttk.Label(frame, text="RSI Config File:").grid(row=0, column=0, padx=5)
        ttk.Entry(frame, textvariable=self.config_file, width=50).grid(row=0, column=1, padx=5)
        ttk.Button(frame, text="Browse", command=self.browse_file).grid(row=0, column=2)

        # Mode selection
        ttk.Label(frame, text="Mode:").grid(row=1, column=0, padx=5)
        ttk.Combobox(frame, textvariable=self.mode, values=["relative", "absolute"], width=10).grid(row=1, column=1, sticky='w')

        # Delay input
        ttk.Label(frame, text="Delay (ms):").grid(row=2, column=0, padx=5)
        ttk.Entry(frame, textvariable=self.delay, width=10).grid(row=2, column=1, sticky='w')

        # Show/hide robot checkbox
        ttk.Checkbutton(frame, text="Show Robot Stick Figure", variable=self.show_robot).grid(row=3, column=0, sticky='w')

        # Start/Stop buttons
        ttk.Button(frame, text="Start Server", command=self.start_server).grid(row=4, column=0, pady=10)
        ttk.Button(frame, text="Stop Server", command=self.stop_server).grid(row=4, column=1, pady=10)

        # Status label
        self.status_label = ttk.Label(frame, text="Status: Idle")
        self.status_label.grid(row=5, column=0, columnspan=3)

        # 3D Plot setup
        self.figure = plt.Figure(figsize=(6, 5))
        self.ax = self.figure.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(self.figure, master=self.master)
        self.canvas.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH, expand=1)

        # XML message display
        right_frame = ttk.Frame(self.master)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=1)

        ttk.Label(right_frame, text="📤 Sent Message").pack()
        self.sent_box = tk.Text(right_frame, height=15, width=70)
        self.sent_box.pack(pady=5)

        ttk.Label(right_frame, text="📩 Received Message").pack()
        self.received_box = tk.Text(right_frame, height=15, width=70)
        self.received_box.pack(pady=5)

    def browse_file(self):
        """Open a file dialog to select a new RSI config file."""
        filename = filedialog.askopenfilename(filetypes=[("XML Files", "*.xml")])
        if filename:
            self.config_file.set(filename)

    def start_server(self):
        """
        Starts the Echo Server in a background thread and begins the visual update loop.
        Validates the existence of the config file first.
        """
        if not os.path.exists(self.config_file.get()):
            self.status_label.config(text="❌ Config file not found.")
            return

        self.server = EchoServer(
            config_file=self.config_file.get(),
            delay_ms=self.delay.get(),
            mode=self.mode.get()
        )
        self.server.start()
        self.running = True
        self.status_label.config(text=f"✅ Server running in {self.mode.get().upper()} mode.")
        self.visual_thread = threading.Thread(target=self.update_visualisation, daemon=True)
        self.visual_thread.start()

    def stop_server(self):
        """Stops the Echo Server and ends the visual update thread."""
        if self.server:
            self.server.stop()
            self.status_label.config(text="😕 Server stopped.")
        self.running = False

    def update_visualisation(self):
        """
        Continuously updates the 3D plot and message windows with live robot TCP and joint data.
        Also displays simplified robot kinematics as a stick figure if enabled.
        """
        while self.running:
            try:
                pos = self.server.state.get("RIst", {})
                joints = self.server.state.get("AIPos", {})
                x = pos.get("X", 0)
                y = pos.get("Y", 0)
                z = pos.get("Z", 0)

                # Track TCP trace history (max 300 points)
                self.trace.append((x, y, z))
                if len(self.trace) > 300:
                    self.trace.pop(0)

                self.ax.clear()
                self.ax.set_title("3D Robot Movement Trace")
                self.ax.set_xlabel("X")
                self.ax.set_ylabel("Y")
                self.ax.set_zlabel("Z")

                # Draw shaded base plane
                floor_x, floor_y = np.meshgrid(np.linspace(-200, 200, 2), np.linspace(-200, 200, 2))
                floor_z = np.zeros_like(floor_x)
                self.ax.plot_surface(floor_x, floor_y, floor_z, alpha=0.2, color='gray')

                # Draw TCP trajectory
                xs = [pt[0] for pt in self.trace]
                ys = [pt[1] for pt in self.trace]
                zs = [pt[2] for pt in self.trace]
                self.ax.plot(xs, ys, zs, label="TCP Path", color='blue')

                # Draw robot as stick figure if enabled
                if self.show_robot.get():
                    link_lengths = [100, 80, 60, 40, 20, 10]
                    angles = [math.radians(joints.get(f"A{i+1}", 0)) for i in range(6)]

                    x0, y0, z0 = 0, 0, 0
                    x_points = [x0]
                    y_points = [y0]
                    z_points = [z0]

                    for i in range(6):
                        dx = link_lengths[i] * math.cos(angles[i])
                        dy = link_lengths[i] * math.sin(angles[i])
                        dz = 0 if i < 3 else link_lengths[i] * math.sin(angles[i])
                        x0 += dx
                        y0 += dy
                        z0 += dz
                        x_points.append(x0)
                        y_points.append(y0)
                        z_points.append(z0)

                    self.ax.plot(x_points, y_points, z_points, label="Robot Arm", color='red', marker='o')

                self.ax.legend()
                self.canvas.draw()

                # Update message displays
                self.received_box.delete("1.0", tk.END)
                self.received_box.insert(tk.END, self.server.last_received.strip() if hasattr(self.server, 'last_received') else "")

                self.sent_box.delete("1.0", tk.END)
                self.sent_box.insert(tk.END, self.server.generate_message().strip())

                time.sleep(0.2)
            except Exception as e:
                print(f"[Visualisation Error] {e}")


if __name__ == "__main__":
    root = tk.Tk()
    app = EchoServerGUI(root)
    root.mainloop()
