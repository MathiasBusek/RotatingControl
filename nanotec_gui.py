"""Connect to a Nanotec controller over USB/Modbus RTU using NanoLib.

Based on Nanotec's NanoLib example (CC BY 4.0), Nanotec GmbH & Co. KG.
"""
import csv
import queue
import sys
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import ttk, messagebox

from nanotec_nanolib import Nanolib as nl

ENCODER_COUNTS_PER_MOTOR_REVOLUTION = 16384
LOG_INTERVAL_SECONDS = 300


class DailyCycleLogger:
    """Append local-time entries to separate daily CSV files."""
    def __init__(self, directory):
        self.directory = Path(directory)

    def write(self, timestamp, event, speeds, encoder):
        self.directory.mkdir(parents=True, exist_ok=True)
        path = self.directory / f"rotation_{timestamp:%Y-%m-%d}.csv"
        needs_header = not path.exists() or path.stat().st_size == 0
        with path.open("a", encoding="utf-8", newline="") as log:
            writer = csv.writer(log)
            if needs_header:
                writer.writerow(("timestamp", "event", "requested_speed1_rpm", "requested_speed2_rpm", "encoder_counts"))
            writer.writerow((timestamp.isoformat(timespec="seconds"), event,
                             f"{speeds[0] / 10:.1f}", f"{speeds[1] / 10:.1f}", encoder))
        return path


def checked(result, operation):
    if result.hasError():
        raise RuntimeError(f"{operation}: {result.getError()}")
    return result.getResult() if hasattr(result, "getResult") else None


class ScanCallback(nl.NlcScanBusCallback):
    def callback(self, info, devices_found, data):
        return nl.ResultVoid()


class Connection:
    def __init__(self):
        self.api = nl.getNanoLibAccessor()
        self.callback = ScanCallback()
        self.buses = []
        self.devices = []
        self.bus = None
        self.handle = None
        self.connected = False

    def find_ports(self):
        buses = checked(self.api.listAvailableBusHardware(), "Find interfaces")
        supported = (nl.BUS_HARDWARE_ID_PROTOCOL_MODBUS_RTU, nl.BUS_HARDWARE_ID_PROTOCOL_MODBUS_VCP, nl.BUS_HARDWARE_ID_PROTOCOL_MSC)
        self.buses = [b for b in buses if b.getProtocol() in supported]
        return [f"{b.getName()}  |  {b.getHardwareSpecifier()}  |  {b.getProtocol()}" for b in self.buses]

    def open(self, index, baud, parity):
        bus = self.buses[index]
        options = nl.BusHardwareOptions()
        if bus.getProtocol() == nl.BUS_HARDWARE_ID_PROTOCOL_MODBUS_RTU:
            options.addOption(nl.Serial().BAUD_RATE_OPTIONS_NAME, getattr(nl.SerialBaudRate(), 'BAUD_RATE_' + baud))
            options.addOption(nl.Serial().PARITY_OPTIONS_NAME, getattr(nl.SerialParity(), parity.upper()))
        checked(self.api.openBusHardwareWithProtocol(bus, options), "Open interface")
        self.bus = bus

    def scan(self):
        self.devices = []
        devices = checked(self.api.scanDevices(self.bus, self.callback), "Scan devices")
        self.devices = list(devices)
        return [f"ID {d.getDeviceId()}  |  {d.getDescription()}" for d in self.devices]

    def auto_connect(self):
        """Choose a unique Nanotec VCP bus and a unique C5-E-2-11."""
        ports = self.find_ports()
        vcp = [(i, b) for i, b in enumerate(self.buses)
               if b.getProtocol() == nl.BUS_HARDWARE_ID_PROTOCOL_MODBUS_VCP]
        direct = [(i, b) for i, b in vcp
                  if 'nanotec' in (b.getName() + ' ' + b.getBusHardware()).lower()]
        if len(direct) == 1:
            index = direct[0][0]
        elif len(direct) > 1:
            raise RuntimeError("Multiple Nanotec VCP interfaces found; select one manually.")
        elif len(vcp) == 1:
            index = vcp[0][0]
        else:
            raise RuntimeError("No unique Nanotec VCP interface found; use the Connection tab.")
        try:
            self.open(index, '19200', 'Even')
            devices = self.scan()
            matches = [i for i, d in enumerate(self.devices)
                       if 'c5-e-2-11' in d.getDescription().lower()]
            if len(matches) != 1:
                raise RuntimeError(f"Expected one C5-E-2-11, found {len(matches)}. Select the controller manually.")
            device_index = matches[0]
            name = self.connect(device_index)
            try:
                speeds, speed_error = self.read_cycle_speeds(), None
            except Exception as exc:
                speeds, speed_error = None, str(exc)
            return ports, index, devices, device_index, name, speeds, speed_error
        except Exception as exc:
            try:
                self.close()
            except Exception as cleanup:
                raise RuntimeError(f"{exc}; cleanup failed: {cleanup}") from exc
            raise

    def connect(self, index):
        handle = checked(self.api.addDevice(self.devices[index]), "Add device")
        self.handle = handle
        try:
            checked(self.api.connectDevice(handle), "Connect device")
        except Exception:
            try:
                checked(self.api.removeDevice(handle), "Remove device")
                self.handle = None
            except Exception as cleanup_error:
                raise RuntimeError(f"Connection failed and cleanup failed: {cleanup_error}")
            raise
        self.connected = True
        result = self.api.getDeviceName(handle)
        return str(result.getResult()) if not result.hasError() else ""

    def read_feedback(self):
        if not self.connected:
            raise RuntimeError("Controller is not connected.")
        return (self.read_encoder(),
                checked(self.api.readNumber(self.handle, nl.OdIndex(0x606C, 0)), "Read actual velocity"))

    def read_encoder(self):
        if not self.connected:
            raise RuntimeError("Controller is not connected.")
        return checked(self.api.readNumber(self.handle, nl.OdIndex(0x6063, 0)), "Read encoder position")

    def nanoj_status(self):
        status = checked(self.api.readNumber(self.handle, nl.OdIndex(0x2301, 0)), "Read NanoJ status")
        error = checked(self.api.readNumber(self.handle, nl.OdIndex(0x2302, 0)), "Read NanoJ error")
        return status, error

    def read_cycle_speeds(self):
        if not self.connected:
            raise RuntimeError("Controller is not connected.")
        return tuple(checked(self.api.readNumber(self.handle, nl.OdIndex(0x2400, sub)),
                             f"Read Position {sub} speed") for sub in (1, 2))

    def set_cycle_speeds(self, speed1, speed2):
        if not self.connected:
            raise RuntimeError("Controller is not connected.")
        if any(not isinstance(v, int) or isinstance(v, bool) or not 1 <= v <= 100 for v in (speed1, speed2)):
            raise ValueError("Both speeds must be whole numbers from 1 to 100.")
        try:
            for sub, value in ((1, speed1), (2, speed2)):
                checked(self.api.writeNumber(self.handle, value, nl.OdIndex(0x2400, sub), 32), f"Write Position {sub} speed")
            actual = self.read_cycle_speeds()
            if actual != (speed1, speed2):
                raise RuntimeError(f"Readback returned {actual}.")
        except Exception as exc:
            raise RuntimeError(f"Could not set both speeds: {exc} One or both settings may have changed; check before continuing.") from exc

    def command_cycle(self, start):
        if not self.connected:
            raise RuntimeError("Controller is not connected.")
        state, error = self.nanoj_status()
        if not state & 1 or state & 4 or error:
            raise RuntimeError(f"NanoJ must be running without an error (status {state}, error {error}).")
        index = nl.OdIndex(0x2400, 3 if start else 4)
        old = checked(self.api.readNumber(self.handle, index), "Read cycle command counter")
        new = old + 1 if old < 2147483647 else 0
        try:
            checked(self.api.writeNumber(self.handle, new, index, 32), "Write cycle command")
        except Exception as exc:
            raise RuntimeError(f"Cycle command failed or its delivery is unknown: {exc}. Check the platform before retrying.") from exc

    def set_nanoj_running(self, running):
        if not self.connected:
            raise RuntimeError("Controller is not connected.")
        index = nl.OdIndex(0x2300, 0)
        control = checked(self.api.readNumber(self.handle, index), "Read NanoJ control")
        if running:
            error = checked(self.api.readNumber(self.handle, nl.OdIndex(0x2302, 0)), "Read NanoJ error")
            if error:
                raise RuntimeError(f"NanoJ error code {error}; program was not started.")
        new_control = (control | 1) if running else (control & ~1)
        checked(self.api.writeNumber(self.handle, new_control, index, 32), "Write NanoJ control")
        time.sleep(0.25)
        return self.nanoj_status()

    def close(self):
        errors = []
        if self.handle is not None:
            if self.connected:
                try:
                    checked(self.api.disconnectDevice(self.handle), "Disconnect device")
                    self.connected = False
                except Exception as exc:
                    errors.append(str(exc))
            if not self.connected:
                try:
                    checked(self.api.removeDevice(self.handle), "Remove device")
                    self.handle = None
                except Exception as exc:
                    errors.append(str(exc))
        if self.bus is not None and self.handle is None:
            try:
                checked(self.api.closeBusHardware(self.bus), "Close interface")
                self.bus = None
                self.devices = []
            except Exception as exc:
                errors.append(str(exc))
        if errors:
            raise RuntimeError("\n".join(errors))


class App(tk.Tk):
    def __init__(self, fullscreen=False, autoconnect=False):
        super().__init__()
        self.title("Nanotec motor connection")
        self.conn = Connection()
        self.results = queue.Queue()
        self.busy = False
        self.active_quiet = False
        self.pending_action = None
        self.closing = False
        self.feedback_timer = None
        self.poll_timer = None
        self.runtime_timer = None
        self.feedback_active = False
        self.logger = DailyCycleLogger(Path(__file__).resolve().parent / "logs")
        self.runtime_started = None
        self.runtime_stopped = None
        self.cycle_tracking = False
        self.next_log_due = None
        self.applied_speeds = None
        self.velocity_samples = deque(maxlen=10)
        self.angle_zero = None
        self.last_encoder = None
        self.unwrapped_counts = 0
        self.port = tk.StringVar()
        self.device = tk.StringVar()
        self.baud = tk.StringVar(value="19200")
        self.parity = tk.StringVar(value="Even")
        self.status = tk.StringVar(value="Find interfaces to begin.")
        self.encoder = tk.StringVar(value="—")
        self.angle = tk.StringVar(value="—")
        self.velocity = tk.StringVar(value="—")
        self.nanoj_state = tk.StringVar(value="Unknown — connect to read status.")
        self.runtime_display = tk.StringVar(value="Runtime 00:00:00")
        self.feedback_status = tk.StringVar(value="Connect to start live readings.")
        self.speed1 = tk.StringVar(value="")
        self.speed2 = tk.StringVar(value="")
        self.symmetric = tk.BooleanVar(value=False)
        # NanoJ does not expose cycle state; None means it has not been confirmed.
        self.cycle_request = None
        self.boundary_stop_pending = False
        self.boundary_stop_anchor = None
        self.boundary_stop_target = None
        self.boundary_stop_direction = None
        self.boundary_stop_armed_at = None
        self.speed1_rpm = tk.StringVar(value="—")
        self.speed2_rpm = tk.StringVar(value="—")
        self.speed1.trace_add("write", self.on_speed1_changed)
        self.speed2.trace_add("write", lambda *_: self.show_speed_rpm(self.speed2, self.speed2_rpm))
        # One status message appears directly below the cycle controls and on
        # the connection screen, so important command feedback is not hidden.
        self.cycle_status = self.status

        # The control screen fits a landscape 800 x 480 display. The Connection
        # tab keeps infrequently used settings away from the motion controls.
        screen_width, screen_height = self.winfo_screenwidth(), self.winfo_screenheight()
        self.ui_scale = max(0.75, min(screen_width / 800, screen_height / 480)) if fullscreen else 1.0
        self.geometry(f"{screen_width}x{screen_height}" if fullscreen else "800x480")
        self.minsize(640, 420)
        self.configure(bg="#111b29")
        self.tk.call("tk", "scaling", self.ui_scale)
        px = lambda value: round(value * self.ui_scale)
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("TFrame", background="#111b29")
        style.configure("TLabel", background="#111b29", foreground="#e8f0f8", font=("Segoe UI", 13))
        style.configure("Heading.TLabel", font=("Segoe UI", 18, "bold"), foreground="#f4f8fc")
        style.configure("Muted.TLabel", foreground="#9aafc3", font=("Segoe UI", 12))
        style.configure("Value.TLabel", foreground="#ffffff", font=("Segoe UI", 22, "bold"))
        style.configure("Speed.TLabel", foreground="#54d1c6", font=("Segoe UI", 19, "bold"))
        style.configure("TButton", padding=(px(12), px(10)), font=("Segoe UI", 12, "bold"))
        style.configure("Start.TButton", background="#1b8f78", foreground="white")
        style.map("Start.TButton", background=[("disabled", "#435661"), ("active", "#25aa8e")])
        style.configure("Stop.TButton", background="#a8404f", foreground="white")
        style.map("Stop.TButton", background=[("disabled", "#435661"), ("active", "#c45463")])
        style.configure("CycleActive.TButton", background="#a8404f", foreground="white", relief="sunken")
        style.map("CycleActive.TButton", background=[("disabled", "#435661"), ("active", "#c45463")])
        style.configure("Symmetric.Toolbutton", padding=(px(12), px(10)), font=("Segoe UI", 12, "bold"))
        style.map("Symmetric.Toolbutton", background=[("selected", "#1b8f78"), ("active", "#526d7d")],
                  foreground=[("selected", "white")], relief=[("selected", "sunken")])
        style.configure("TNotebook", background="#111b29", borderwidth=0)
        style.configure("TNotebook.Tab", padding=(px(18), px(7)), font=("Segoe UI", 12, "bold"))
        style.map("TNotebook.Tab", background=[("selected", "#294256")], foreground=[("selected", "white")])

        frame = ttk.Frame(self, padding=(px(12), px(7)))
        frame.pack(fill="both", expand=True)
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(1, weight=1)
        header = ttk.Frame(frame)
        header.grid(row=0, column=0, sticky="ew", pady=(0, 4))
        header.columnconfigure(0, weight=1)
        ttk.Label(header, text="ROTATION CONTROL", style="Heading.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(header, textvariable=self.nanoj_state, style="Muted.TLabel").grid(row=0, column=1, sticky="e")
        self.runtime_label = ttk.Label(header, textvariable=self.runtime_display, style="Muted.TLabel")
        self.runtime_label.grid(row=0, column=2, sticky="e", padx=(px(12), 0))
        self.exit_button = ttk.Button(header, text="Exit", command=self.exit_app)
        self.exit_button.grid(row=0, column=3, padx=(12, 0))

        self.pages = ttk.Notebook(frame)
        self.pages.grid(row=1, column=0, sticky="nsew")
        control = ttk.Frame(self.pages, padding=(px(8), px(7)))
        connection = ttk.Frame(self.pages, padding=(px(12), px(8)))
        self.pages.add(control, text="  CONTROL  ")
        self.pages.add(connection, text="  CONNECTION  ")
        self.pages.select(connection)

        control.columnconfigure(0, weight=1)
        readings = ttk.Frame(control)
        readings.grid(row=0, column=0, sticky="ew", pady=(0, 6))
        for column in range(3): readings.columnconfigure(column, weight=1)
        for column, label, variable in ((0, "MOTOR ANGLE  ·  °", self.angle),
                                        (1, "AVERAGE SPEED  ·  RPM", self.velocity),
                                        (2, "ENCODER  ·  COUNTS", self.encoder)):
            ttk.Label(readings, text=label, style="Muted.TLabel").grid(row=0, column=column, sticky="w")
            ttk.Label(readings, textvariable=variable, style="Value.TLabel").grid(row=1, column=column, sticky="w")

        speeds = ttk.Frame(control)
        speeds.grid(row=1, column=0, sticky="ew")
        speeds.columnconfigure(0, weight=1, uniform="speed")
        speeds.columnconfigure(1, weight=1, uniform="speed")
        self.slider_values = [tk.IntVar(value=80), tk.IntVar(value=5)]
        self.speed1_box = self.make_speed_slider(speeds, 0, "SPEED 1 · RPM", self.speed1, self.speed1_rpm, self.slider_values[0])
        self.speed2_box = self.make_speed_slider(speeds, 1, "SPEED 2 · RPM", self.speed2, self.speed2_rpm, self.slider_values[1])

        actions = ttk.Frame(control)
        actions.grid(row=2, column=0, sticky="ew", pady=(8, 2))
        for column in range(4): actions.columnconfigure(column, weight=1, uniform="action")
        self.cycle_button = ttk.Button(actions, text="■ IMMEDIATE STOP", style="CycleActive.TButton", command=self.toggle_cycle)
        self.cycle_button.grid(row=0, column=0, sticky="ew", padx=(0, 6))
        self.boundary_stop_button = ttk.Button(actions, text="STOP CYCLE", command=self.stop_at_cycle_end)
        self.boundary_stop_button.grid(row=0, column=1, sticky="ew", padx=3)
        self.symmetric_button = ttk.Checkbutton(actions, text="SYMMETRIC", style="Symmetric.Toolbutton", variable=self.symmetric, command=self.toggle_symmetric)
        self.symmetric_button.grid(row=0, column=2, sticky="ew", padx=3)
        self.apply_speeds_button = ttk.Button(actions, text="APPLY SPEEDS", command=self.apply_speeds)
        self.apply_speeds_button.grid(row=0, column=3, sticky="ew", padx=(6, 0))
        status_bar = ttk.Frame(control)
        status_bar.grid(row=3, column=0, sticky="ew", pady=(px(8), 0))
        status_bar.columnconfigure(1, weight=1)
        ttk.Label(status_bar, text="Status:").grid(row=0, column=0, sticky="nw", padx=(0, px(8)))
        ttk.Label(status_bar, textvariable=self.status, wraplength=px(670)).grid(row=0, column=1, sticky="w")

        connection.columnconfigure(1, weight=1)
        ttk.Label(connection, text="Interface").grid(row=0, column=0, sticky="w", padx=(0, 8), pady=4)
        self.ports = ttk.Combobox(connection, textvariable=self.port, state="readonly", font=("Segoe UI", 12))
        self.ports.grid(row=0, column=1, sticky="ew", pady=4)
        self.find_button = ttk.Button(connection, text="Find interfaces", command=self.find_ports)
        self.find_button.grid(row=0, column=2, padx=(8, 0))
        ttk.Label(connection, text="Baud / parity").grid(row=1, column=0, sticky="w", pady=4)
        settings = ttk.Frame(connection)
        settings.grid(row=1, column=1, sticky="w")
        self.baud_box = ttk.Combobox(settings, textvariable=self.baud, width=10, state="readonly", font=("Segoe UI", 12), values=("7200", "9600", "14400", "19200", "38400", "56000", "57600", "115200", "128000", "256000"))
        self.baud_box.pack(side="left")
        self.parity_box = ttk.Combobox(settings, textvariable=self.parity, width=8, state="readonly", font=("Segoe UI", 12), values=("Even", "None", "Odd", "Mark", "Space"))
        self.parity_box.pack(side="left", padx=8)
        self.open_button = ttk.Button(connection, text="Open interface", command=self.open_port)
        self.open_button.grid(row=1, column=2, sticky="ew", padx=(8, 0))
        ttk.Label(connection, text="Controller").grid(row=2, column=0, sticky="w", pady=4)
        self.device_box = ttk.Combobox(connection, textvariable=self.device, state="readonly", font=("Segoe UI", 12))
        self.device_box.grid(row=2, column=1, sticky="ew")
        self.scan_button = ttk.Button(connection, text="Scan devices", command=self.scan)
        self.scan_button.grid(row=2, column=2, padx=(8, 0))
        controls = ttk.Frame(connection)
        controls.grid(row=3, column=0, columnspan=3, sticky="w", pady=(8, 4))
        self.connect_button = ttk.Button(controls, text="Connect", style="Start.TButton", command=self.connect)
        self.connect_button.pack(side="left", padx=(0, 8))
        self.close_button = ttk.Button(controls, text="Disconnect", command=self.close_connection)
        self.close_button.pack(side="left", padx=(0, 8))
        self.retry_button = ttk.Button(controls, text="Resume readings", command=self.start_feedback)
        self.retry_button.pack(side="left")
        nanoj = ttk.Frame(connection)
        nanoj.grid(row=4, column=0, columnspan=3, sticky="w", pady=(8, 4))
        self.nanoj_start = ttk.Button(nanoj, text="Start NanoJ", command=lambda: self.control_nanoj(True))
        self.nanoj_start.pack(side="left", padx=(0, 8))
        self.nanoj_stop = ttk.Button(nanoj, text="Stop NanoJ", command=lambda: self.control_nanoj(False))
        self.nanoj_stop.pack(side="left")
        ttk.Label(connection, text="Stopping NanoJ may not stop the motor. Starting it resets its variables.", style="Muted.TLabel", wraplength=px(710)).grid(row=5, column=0, columnspan=3, sticky="w")
        ttk.Label(connection, textvariable=self.feedback_status, style="Muted.TLabel", wraplength=px(710)).grid(row=6, column=0, columnspan=3, sticky="w", pady=(3, 0))
        ttk.Label(connection, textvariable=self.status, wraplength=px(710)).grid(row=7, column=0, columnspan=3, sticky="w", pady=(px(6), 0))
        self.ports.bind("<<ComboboxSelected>>", lambda e: self.update_controls())
        self.device_box.bind("<<ComboboxSelected>>", lambda e: self.update_controls())
        self.protocol("WM_DELETE_WINDOW", self.exit_app)
        self.update_controls()
        self.poll_timer = self.after(100, self.poll)
        self.runtime_timer = self.after(1000, self.update_runtime_clock)
        if autoconnect:
            self.after(250, self.auto_connect)

    def show_speed_rpm(self, raw, displayed):
        try:
            displayed.set(f"{int(raw.get()) / 10:.1f}")
        except ValueError:
            displayed.set("—")

    def on_speed1_changed(self, *_):
        self.show_speed_rpm(self.speed1, self.speed1_rpm)
        if self.symmetric.get():
            self.speed2.set(self.speed1.get())

    def toggle_symmetric(self):
        left = self.speed1_box.master
        right = self.speed2_box.master
        if self.symmetric.get():
            self.speed2.set(self.speed1.get())
            right.grid_remove()
            left.grid_configure(columnspan=2)
        else:
            left.grid_configure(columnspan=1)
            right.grid()
            try:
                value = int(self.speed2.get())
                if 1 <= value <= 100:
                    self.speed2_box.set(value)
            except ValueError:
                pass

    def make_speed_slider(self, parent, column, title, value, display, slider_value):
        panel = ttk.Frame(parent, padding=tuple(round(n * self.ui_scale) for n in (5, 2, 12, 2)))
        panel.grid(row=0, column=column, sticky="ew")
        panel.columnconfigure(0, weight=1)
        ttk.Label(panel, text=title, style="Muted.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(panel, textvariable=display, style="Speed.TLabel").grid(row=0, column=1, sticky="e")
        slider = tk.Scale(panel, from_=1, to=100, orient="horizontal", resolution=1,
                          variable=slider_value, showvalue=False, sliderlength=round(44 * self.ui_scale),
                          width=round(32 * self.ui_scale), borderwidth=0, highlightthickness=0,
                          bg="#111b29", troughcolor="#294256", fg="#e8f0f8",
                          activebackground="#54d1c6", command=lambda number: value.set(str(round(float(number)))))
        slider.grid(row=1, column=0, columnspan=2, sticky="ew")
        ttk.Label(panel, text="0.1", style="Muted.TLabel").grid(row=2, column=0, sticky="w")
        ttk.Label(panel, text="10.0", style="Muted.TLabel").grid(row=2, column=1, sticky="e")
        return slider


    def update_controls(self):
        idle = not self.busy and not self.closing
        opened = self.conn.bus is not None
        connected = self.conn.handle is not None
        def state(widget, enabled): widget.configure(state="normal" if enabled else "disabled")
        state(self.find_button, idle and not opened)
        state(self.open_button, idle and not opened and self.ports.current() >= 0)
        state(self.scan_button, idle and opened and not connected)
        state(self.connect_button, idle and opened and not connected and self.device_box.current() >= 0)
        state(self.close_button, idle and (opened or connected))
        state(self.retry_button, idle and self.conn.connected and not self.feedback_active)
        state(self.nanoj_start, idle and self.conn.connected)
        state(self.nanoj_stop, idle and self.conn.connected)
        for widget in (self.apply_speeds_button, self.cycle_button, self.symmetric_button):
            state(widget, idle and self.conn.connected)
        state(self.boundary_stop_button, idle and self.conn.connected and self.feedback_active and self.angle_zero is not None and not self.boundary_stop_pending)
        for widget in (self.speed1_box, self.speed2_box):
            state(widget, self.conn.connected and not self.closing)
        for widget in (self.ports, self.baud_box, self.parity_box):
            widget.configure(state="readonly" if idle and not opened else "disabled")
        self.device_box.configure(state="readonly" if idle and opened and not connected else "disabled")

    def run(self, label, work, done, quiet=False, on_error=None):
        if self.busy:
            if self.active_quiet and not quiet and self.pending_action is None and not self.closing:
                # A controller command must wait for the current read; NanoLib
                # operations must not run concurrently on the same connection.
                self.pending_action = (label, work, done, on_error)
                self.status.set("Waiting for the current reading…")
                self.update_controls()
            return
        self.busy = True
        self.active_quiet = quiet
        if not quiet:
            self.status.set(label)
            self.update_controls()
        def worker():
            try: self.results.put((done, work(), None, quiet, on_error))
            except Exception as exc: self.results.put((done, None, str(exc), quiet, on_error))
        threading.Thread(target=worker, daemon=True).start()

    def poll(self):
        self.poll_timer = None
        try:
            done, result, error, quiet, on_error = self.results.get_nowait()
        except queue.Empty:
            pass
        else:
            self.busy = False
            self.active_quiet = False
            if error:
                if on_error is not None:
                    on_error()
                if quiet:
                    self.feedback_active = False
                    self.stop_runtime_tracking()
                    self.clear_feedback()
                    self.boundary_stop_pending = False
                    self.boundary_stop_anchor = None
                    self.boundary_stop_target = None
                    self.boundary_stop_direction = None
                    self.boundary_stop_armed_at = None
                    self.set_cycle_request(None)
                    self.nanoj_state.set("NanoJ status unavailable.")
                    self.feedback_status.set(f"Read failed: {error}. Select Resume readings to retry.")
                    self.status.set("Feedback lost; cycle status unknown. Check motion and reconnect.")
                else:
                    self.status.set(error)
                    if self.nanoj_state.get() == "Sending NanoJ command…":
                        self.nanoj_state.set("NanoJ command failed; status not verified.")
                    messagebox.showerror("NanoLib", error, parent=self)
            else:
                done(result)
            if not self.winfo_exists():
                return
            # A feedback callback may have started the zero-crossing stop.
            # Let it finish before dispatching a user action queued during the read.
            if self.busy:
                self.poll_timer = self.after(50, self.poll)
                return
            if self.closing:
                if self.pending_action is not None:
                    label, work, callback, failure = self.pending_action
                    self.pending_action = None
                    self.run(label, work, callback, on_error=failure)
                    self.poll_timer = self.after(50, self.poll)
                    return
                self.closing = False
                self.exit_app()
                return
            if self.pending_action is not None:
                label, work, callback, failure = self.pending_action
                self.pending_action = None
                self.run(label, work, callback, on_error=failure)
            elif not quiet or error:
                self.update_controls()
        self.poll_timer = self.after(50, self.poll)

    def destroy(self):
        if self.poll_timer is not None:
            self.after_cancel(self.poll_timer)
            self.poll_timer = None
        if self.runtime_timer is not None:
            self.after_cancel(self.runtime_timer)
            self.runtime_timer = None
        super().destroy()

    def update_runtime_clock(self):
        if self.runtime_started is not None:
            end = time.monotonic() if self.cycle_tracking else self.runtime_stopped
            if end is not None:
                elapsed = max(0, int(end - self.runtime_started))
                hours, remainder = divmod(elapsed, 3600)
                minutes, seconds = divmod(remainder, 60)
                self.set_if_changed(self.runtime_display, f"Runtime {hours:02}:{minutes:02}:{seconds:02}")
        self.runtime_timer = self.after(1000, self.update_runtime_clock)

    def stop_runtime_tracking(self):
        if self.cycle_tracking:
            self.runtime_stopped = time.monotonic()
            self.cycle_tracking = False
            self.next_log_due = None

    def record_cycle_event(self, event, encoder, timestamp=None):
        if self.applied_speeds is None:
            self.status.set("Logging failed: speed settings are unavailable.")
            return
        try:
            self.logger.write(timestamp or datetime.now().astimezone(), event, self.applied_speeds, encoder)
        except (OSError, ValueError) as exc:
            self.status.set(f"Logging failed: {exc}")

    def find_ports(self):
        self.ports['values'] = ()
        self.port.set("")
        def done(ports):
            self.ports['values'] = ports
            if ports: self.ports.current(0)
            self.status.set(f"Found {len(ports)} interface(s). Select the Nanotec VCP interface." if ports else "No compatible interfaces found. Check USB connection and drivers.")
        self.run("Finding interfaces…", self.conn.find_ports, done)

    def open_port(self):
        index, baud, parity = self.ports.current(), self.baud.get(), self.parity.get()
        self.run("Opening interface…", lambda: self.conn.open(index, baud, parity), lambda _: self.status.set("Interface open. Scan for controllers."))

    def scan(self):
        self.device_box['values'] = ()
        self.device.set("")
        def done(devices):
            self.device_box['values'] = devices
            if devices: self.device_box.current(0)
            self.status.set(f"Found {len(devices)} controller(s)." if devices else "No controllers found. Check power, wiring and serial settings.")
        self.run("Scanning for controllers…", self.conn.scan, done)

    def connect(self):
        index = self.device_box.current()
        def work():
            name = self.conn.connect(index)
            try:
                speeds, speed_error = self.conn.read_cycle_speeds(), None
            except Exception as exc:
                speeds, speed_error = None, str(exc)
            return name, speeds, speed_error
        def done(result):
            self.connected_ui(*result)
        self.run("Connecting…", work, done)

    def connected_ui(self, name, speeds, error):
            self.boundary_stop_pending = False
            self.boundary_stop_anchor = None
            self.boundary_stop_target = None
            self.boundary_stop_direction = None
            self.boundary_stop_armed_at = None
            if error:
                self.speed1.set("")
                self.speed2.set("")
                status_message = f"Speed read failed: {error}. Cycle state unknown; stop before starting."
            else:
                speed1, speed2 = speeds
                speed1 = speed1 if speed1 > 0 else 80
                speed2 = speed2 if speed2 > 0 else 5
                self.speed1.set(str(speed1))
                self.speed2.set(str(speed1 if self.symmetric.get() else speed2))
                if self.symmetric.get():
                    speed2 = speed1
                for slider, speed in zip((self.speed1_box, self.speed2_box), (speed1, speed2)):
                    if 1 <= speed <= 100:
                        slider.set(speed)
                if not (1 <= speed1 <= 100 and 1 <= speed2 <= 100):
                    status_message = "Speed outside 0.1–10.0 rpm; adjust sliders. Cycle state unknown."
                else:
                    status_message = "Cycle state unknown; stop before starting."
            self.pages.select(0)
            self.set_cycle_request(None)
            self.status.set(status_message)
            self.start_feedback()

    def auto_connect(self):
        if self.busy or self.conn.connected or self.closing:
            return
        def work():
            return self.conn.auto_connect()
        def done(result):
            ports, port_index, devices, device_index, name, speeds, error = result
            self.ports['values'] = ports
            self.ports.current(port_index)
            self.device_box['values'] = devices
            self.device_box.current(device_index)
            self.connected_ui(name, speeds, error)
        self.run("Connecting to the controller…", work, done)

    def speed_values(self):
        try:
            speed1 = int(self.speed1.get())
            values = (speed1, speed1 if self.symmetric.get() else int(self.speed2.get()))
            if not all(1 <= value <= 100 for value in values):
                raise ValueError
            return values
        except ValueError:
            messagebox.showerror("Invalid speed", "Set both sliders between 0.1 and 10.0 rpm.", parent=self)
            return None

    def apply_speeds(self):
        values = self.speed_values()
        if values is None:
            return
        def done(_):
            self.applied_speeds = values
            self.status.set("Speeds confirmed; they apply at the next segment.")
        self.run("Setting speeds…", lambda: self.conn.set_cycle_speeds(*values), done)

    def start_cycle(self):
        values = self.speed_values()
        if values is None:
            return
        self.angle_zero = None
        self.last_encoder = None
        self.boundary_stop_pending = False
        self.angle.set("—")
        def work():
            self.conn.set_cycle_speeds(*values)
            zero_count = self.conn.read_encoder()
            self.conn.command_cycle(True)
            return zero_count, time.monotonic(), datetime.now().astimezone()
        def done(result):
            zero_count, requested_at, wall_time = result
            self.angle_zero = zero_count
            self.last_encoder = zero_count
            self.unwrapped_counts = 0
            self.applied_speeds = values
            self.runtime_started = requested_at
            self.runtime_stopped = None
            self.cycle_tracking = True
            self.next_log_due = requested_at + LOG_INTERVAL_SECONDS
            self.runtime_display.set("Runtime 00:00:00")
            self.angle.set("0.00")
            self.set_cycle_request("started")
            self.status.set("Start requested; no confirmation from NanoJ. Check motion.")
            self.record_cycle_event("start_requested", zero_count, wall_time)
        self.run("Sending cycle start…", work, done, on_error=self.cycle_request_failed)

    def stop_cycle(self):
        self.boundary_stop_pending = False
        self.boundary_stop_anchor = None
        self.boundary_stop_target = None
        self.boundary_stop_direction = None
        self.boundary_stop_armed_at = None
        def done(_):
            self.stop_runtime_tracking()
            self.set_cycle_request("stopped")
            self.status.set("Stop requested; no confirmation from NanoJ. Check motion.")
        self.run("Sending cycle stop…", lambda: self.conn.command_cycle(False), done, on_error=self.cycle_request_failed)

    def set_cycle_request(self, state):
        self.cycle_request = state
        if state == "stopped":
            self.cycle_button.configure(text="▶ START", style="Start.TButton")
        else:
            self.cycle_button.configure(text="■ IMMEDIATE STOP", style="CycleActive.TButton")

    def stop_at_cycle_end(self):
        if self.boundary_stop_pending or not self.feedback_active or self.angle_zero is None or self.last_encoder is None:
            return
        self.boundary_stop_pending = True
        self.boundary_stop_anchor = self.unwrapped_counts
        self.boundary_stop_target = None
        self.boundary_stop_direction = None
        self.boundary_stop_armed_at = time.monotonic()
        self.status.set("Stop armed: software will request stop after the next zero crossing.")
        if self.feedback_timer is not None:
            self.after_cancel(self.feedback_timer)
            self.feedback_timer = None
        # An in-flight read will schedule its own next sample on completion.
        if not (self.busy and self.active_quiet):
            self.schedule_feedback(0)
        self.update_controls()

    def cycle_request_failed(self):
        self.stop_runtime_tracking()
        self.set_cycle_request(None)
        self.cycle_status.set("Cycle command failed or delivery is unknown. Check motion before retrying.")

    def toggle_cycle(self):
        if self.cycle_request == "stopped":
            self.start_cycle()
        else:
            self.stop_cycle()

    def clear_feedback(self):
        self.encoder.set("—")
        self.angle.set("—")
        self.angle_zero = None
        self.last_encoder = None
        self.unwrapped_counts = 0
        self.boundary_stop_pending = False
        self.boundary_stop_anchor = None
        self.boundary_stop_target = None
        self.boundary_stop_direction = None
        self.boundary_stop_armed_at = None
        self.velocity.set("—")
        self.velocity_samples.clear()

    def stop_feedback(self):
        self.stop_runtime_tracking()
        self.feedback_active = False
        if self.feedback_timer is not None:
            self.after_cancel(self.feedback_timer)
            self.feedback_timer = None
        self.clear_feedback()
        self.feedback_status.set("Readings stopped.")
        self.nanoj_state.set("NanoJ status unavailable.")

    def start_feedback(self):
        if not self.conn.connected or self.feedback_active or self.closing:
            return
        self.feedback_active = True
        self.feedback_status.set("Reading…")
        self.schedule_feedback(0)
        self.update_controls()

    def schedule_feedback(self, delay=None):
        if self.feedback_active and not self.closing:
            if delay is None:
                delay = 100 if self.boundary_stop_pending else 500
            self.feedback_timer = self.after(delay, self.read_feedback)

    def read_feedback(self):
        self.feedback_timer = None
        if not self.feedback_active or not self.conn.connected or self.closing:
            return
        if self.busy:
            self.schedule_feedback(100)
            return
        def work():
            values = self.conn.read_feedback()
            sampled_at = time.monotonic()
            try:
                state = self.conn.nanoj_status()
            except Exception as exc:
                state = (None, str(exc))
            return values, state, sampled_at
        def done(result):
            if self.feedback_active and not self.closing:
                values, state, sampled_at = result
                encoder, velocity = values
                self.set_if_changed(self.encoder, str(encoder))
                crossed_zero = self.update_encoder_phase(encoder, sampled_at)
                if self.angle_zero is not None:
                    angle = ((encoder - self.angle_zero) % ENCODER_COUNTS_PER_MOTOR_REVOLUTION) * 360 / ENCODER_COUNTS_PER_MOTOR_REVOLUTION
                    self.set_if_changed(self.angle, f"{angle:.2f}")
                self.velocity_samples.append(velocity)
                self.set_if_changed(self.velocity, f"{sum(self.velocity_samples) / len(self.velocity_samples) / 10:.1f}")
                self.show_nanoj_status(state)
                if self.cycle_tracking and self.next_log_due is not None and sampled_at >= self.next_log_due:
                    self.next_log_due = sampled_at + LOG_INTERVAL_SECONDS
                    self.record_cycle_event("periodic_unconfirmed", encoder)
                if crossed_zero and self.boundary_stop_pending:
                    self.stop_cycle()
                self.set_if_changed(self.feedback_status, "Live · velocity averaged over up to 10 samples.")
                self.schedule_feedback()
        self.run("", work, done, quiet=True)

    def update_encoder_phase(self, encoder, sampled_at=None):
        if self.angle_zero is None or self.last_encoder is None:
            return False
        # 0x6063 is a signed 32-bit counter. Unwrap its rollover between reads.
        delta = ((encoder - self.last_encoder + (1 << 31)) % (1 << 32)) - (1 << 31)
        previous = self.unwrapped_counts
        self.unwrapped_counts += delta
        self.last_encoder = encoder
        if (self.boundary_stop_pending and sampled_at is not None and
                self.boundary_stop_armed_at is not None and sampled_at < self.boundary_stop_armed_at):
            # This read completed before the button was pressed; its result is
            # only the new starting point, never a post-request crossing.
            self.boundary_stop_anchor = self.unwrapped_counts
            self.boundary_stop_target = None
            self.boundary_stop_direction = None
            return False
        if not self.boundary_stop_pending or delta == 0:
            return False
        direction = 1 if delta > 0 else -1
        if direction != self.boundary_stop_direction:
            reference = self.boundary_stop_anchor if self.boundary_stop_direction is None else previous
            count = ENCODER_COUNTS_PER_MOTOR_REVOLUTION
            self.boundary_stop_target = ((reference // count) + 1) * count if direction > 0 else ((reference - 1) // count) * count
            self.boundary_stop_direction = direction
        return (self.unwrapped_counts >= self.boundary_stop_target if direction > 0
                else self.unwrapped_counts <= self.boundary_stop_target)

    @staticmethod
    def set_if_changed(variable, value):
        if variable.get() != value:
            variable.set(value)

    def show_nanoj_status(self, state):
        status, error = state
        if (status is None or not (status & 1) or status & 4 or error) and self.cycle_request is not None:
            self.set_cycle_request(None)
            self.cycle_status.set("Cycle state unknown. Check NanoJ and the platform before starting.")
        if status is None or not (status & 1) or status & 4 or error:
            self.stop_runtime_tracking()
            was_pending = self.boundary_stop_pending
            self.boundary_stop_pending = False
            if was_pending:
                self.update_controls()
                self.status.set("Stop-at-zero cancelled: NanoJ status unavailable. Check motion.")
        if status is None:
            self.set_if_changed(self.nanoj_state, f"NanoJ status unavailable: {error}")
        elif status & 4 or error:
            self.set_if_changed(self.nanoj_state, f"NanoJ error: status {status}, code {error}")
        else:
            self.set_if_changed(self.nanoj_state, "NanoJ: " + ("running" if status & 1 else "stopped"))

    def control_nanoj(self, running):
        self.nanoj_state.set("Sending NanoJ command…")
        def done(state):
            self.boundary_stop_pending = False
            self.set_cycle_request(None)
            self.show_nanoj_status(state)
            status, error = state
            if error or bool(status & 1) != running:
                self.status.set("NanoJ command sent; check status. Cycle state unknown.")
            else:
                self.status.set("NanoJ " + ("started" if running else "stopped") + "; cycle state unknown.")
        self.run("Sending NanoJ command…", lambda: self.conn.set_nanoj_running(running), done)

    def close_connection(self):
        self.stop_feedback()
        self.boundary_stop_pending = False
        self.set_cycle_request(None)
        self.speed1.set("")
        self.speed2.set("")
        self.cycle_status.set("Disconnected.")
        def done(_):
            self.device_box['values'] = ()
            self.device.set("")
            self.status.set("Disconnected; interface closed.")
        self.run("Closing connection…", self.conn.close, done)

    def exit_app(self):
        self.stop_feedback()
        if self.busy:
            # Keep an already-requested cycle stop; cancel other queued actions.
            if self.pending_action is not None and self.pending_action[0] != "Sending cycle stop…":
                self.pending_action = None
            self.closing = True
            self.status.set("Waiting for the current operation before closing…")
        elif self.conn.bus is not None or self.conn.handle is not None:
            self.run("Closing connection…", self.conn.close, lambda _: self.destroy())
        else:
            self.destroy()


if __name__ == "__main__":
    fullscreen = "--fullscreen" in sys.argv[1:] or (sys.platform.startswith("linux") and "--windowed" not in sys.argv[1:])
    app = App(fullscreen=fullscreen, autoconnect=True)
    if fullscreen:
        app.attributes("-fullscreen", True)
    app.mainloop()
