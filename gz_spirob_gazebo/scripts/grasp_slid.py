#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk

try:
    from gz.msgs.double_pb2 import Double
    from gz.transport import Node
except Exception:
    from gz.msgs11.double_pb2 import Double
    from gz.transport14 import Node


FORCE_MIN = 0.0
FORCE_MAX = 1500.0
FORCE_STEP = 5.0
TICK_MS = 100


class TendonGui:
    def __init__(self):
        self.model_name = "arm"
        self.tendon1_name = "tendon1"
        self.tendon2_name = "tendon2"
        self.tendon3_name = "tendon3"

        self.node = Node()
        self.tendon1_topic = (
            f"/model/{self.model_name}/tendon/{self.tendon1_name}/cmd_force"
        )
        self.tendon2_topic = (
            f"/model/{self.model_name}/tendon/{self.tendon2_name}/cmd_force"
        )
        self.tendon3_topic = (
            f"/model/{self.model_name}/tendon/{self.tendon3_name}/cmd_force"
        )
        self.tendon1_pub = self.node.advertise(self.tendon1_topic, Double)
        self.tendon2_pub = self.node.advertise(self.tendon2_topic, Double)
        self.tendon3_pub = self.node.advertise(self.tendon3_topic, Double)

        self.root = tk.Tk()
        self.root.title("SpiRob Tendon Control")
        self.root.geometry("520x280")
        self.root.resizable(False, False)

        self.force_1_var = tk.DoubleVar(value=0.0)
        self.force_2_var = tk.DoubleVar(value=0.0)
        self.force_3_var = tk.DoubleVar(value=0.0)
        self.value_1_var = tk.StringVar(value="0.00 N")
        self.value_2_var = tk.StringVar(value="0.00 N")
        self.value_3_var = tk.StringVar(value="0.00 N")
        self.stream_enabled = tk.BooleanVar(value=False)
        self.status_var = tk.StringVar(value="Ready")

        self.force_1_var.trace_add(
            "write", lambda *_: self._update_label(self.force_1_var, self.value_1_var)
        )
        self.force_2_var.trace_add(
            "write", lambda *_: self._update_label(self.force_2_var, self.value_2_var)
        )
        self.force_3_var.trace_add(
            "write", lambda *_: self._update_label(self.force_3_var, self.value_3_var)
        )

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self):
        frame = ttk.Frame(self.root, padding=12)
        frame.pack(fill=tk.BOTH, expand=True)

        ttk.Label(
            frame,
            text=f"Tendon force ({FORCE_MIN:.0f}-{FORCE_MAX:.0f} N)",
            font=("TkDefaultFont", 11, "bold"),
        ).pack(anchor=tk.W, pady=(0, 8))

        self._add_tendon_controls(
            frame, "Tendon 1 (left / F1)", self.force_1_var, self.value_1_var
        )
        self._add_tendon_controls(
            frame, "Tendon 2 (right / F2)", self.force_2_var, self.value_2_var
        )
        self._add_tendon_controls(
            frame, "Tendon 3 (mid / F3)", self.force_3_var, self.value_3_var
        )

        button_row = ttk.Frame(frame)
        button_row.pack(fill=tk.X, pady=(8, 6))

        ttk.Button(button_row, text="Reset", command=self.zero_forces).pack(
            side=tk.LEFT, padx=(0, 4)
        )
        ttk.Button(button_row, text="Send Once", command=self.publish_now).pack(
            side=tk.LEFT
        )
        ttk.Checkbutton(
            button_row,
            text="Stream at 10 Hz",
            variable=self.stream_enabled,
            command=self._handle_stream_toggle,
        ).pack(side=tk.LEFT, padx=(12, 0))

        ttk.Label(frame, textvariable=self.status_var).pack(anchor=tk.W, pady=(6, 0))

    def _add_tendon_controls(self, parent, label, force_var, value_var):
        header = ttk.Frame(parent)
        header.pack(fill=tk.X)
        ttk.Label(header, text=label).pack(side=tk.LEFT)
        ttk.Label(header, textvariable=value_var).pack(side=tk.RIGHT)

        row = ttk.Frame(parent)
        row.pack(fill=tk.X, pady=(0, 6))

        ttk.Scale(
            row,
            from_=FORCE_MIN,
            to=FORCE_MAX,
            variable=force_var,
            orient=tk.HORIZONTAL,
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 8))

        ttk.Spinbox(
            row,
            from_=FORCE_MIN,
            to=FORCE_MAX,
            increment=FORCE_STEP,
            width=8,
            textvariable=force_var,
        ).pack(side=tk.RIGHT)

    def _update_label(self, force_var, value_var):
        value_var.set(f"{self._read(force_var):.2f} N")

    def _read(self, force_var):
        try:
            value = float(force_var.get())
        except (tk.TclError, ValueError):
            return 0.0
        if value != value:
            return 0.0
        return max(FORCE_MIN, min(FORCE_MAX, value))

    def _publish(self, force_1, force_2, force_3, label):
        cmd1 = Double()
        cmd1.data = float(force_1)
        cmd2 = Double()
        cmd2.data = float(force_2)
        cmd3 = Double()
        cmd3.data = float(force_3)

        self.tendon1_pub.publish(cmd1)
        self.tendon2_pub.publish(cmd2)
        self.tendon3_pub.publish(cmd3)

        self.status_var.set(
            f"{label}: F1={cmd1.data:.1f} N, F2={cmd2.data:.1f}, F3={cmd3.data:.1f} N"
        )

    def publish_now(self):
        self._publish(
            self._read(self.force_1_var),
            self._read(self.force_2_var),
            self._read(self.force_3_var),
            "Sent"
        )

    def _handle_stream_toggle(self):
        if self.stream_enabled.get():
            self.status_var.set("Streaming at 10 Hz")
            self._stream_loop()
        else:
            self.status_var.set("Streaming stopped")

    def _stream_loop(self):
        if not self.stream_enabled.get():
            return
        self._publish(
            self._read(self.force_1_var),
            self._read(self.force_2_var),
            self._read(self.force_3_var),
            "Streaming"
        )
        self.root.after(TICK_MS, self._stream_loop)

    def zero_forces(self):
        self.stream_enabled.set(False)
        self.force_1_var.set(0.0)
        self.force_2_var.set(0.0)
        self.force_3_var.set(0.0)
        self._publish(0.0, 0.0, 0.0, "Reset")

    def on_close(self):
        try:
            self.stream_enabled.set(False)
            self._publish(0.0, 0.0, 0.0, "Reset")
        finally:
            self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    app = TendonGui()
    app.run()


if __name__ == "__main__":
    main()
