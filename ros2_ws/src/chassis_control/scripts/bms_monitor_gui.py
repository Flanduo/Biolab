#!/usr/bin/env python3
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class BmsGuiNode(Node):
    def __init__(self):
        super().__init__("bms_monitor_gui")
        self._lock = threading.Lock()

        self.online = False
        self.pack_voltage_v = 0.0
        self.pack_current_a = 0.0
        self.soc = 0
        self.alarm_flags = 0
        self.cell_mv = []
        self.temp1_c = 0.0
        self.temp2_c = 0.0
        self.pack_power_w = 0.0

        self.create_subscription(Float64MultiArray, "/bms/full_state", self._on_full_state, 10)

    def _on_full_state(self, msg: Float64MultiArray):
        data = list(msg.data)
        if len(data) < 9:
            return
        with self._lock:
            # [online, voltage_v, current_a, soc, power_w, temp1_c, temp2_c, alarm, cell_count, cells...]
            self.online = bool(int(data[0]))
            self.pack_voltage_v = float(data[1])
            self.pack_current_a = float(data[2])
            self.soc = int(data[3])
            self.pack_power_w = float(data[4])
            self.temp1_c = float(data[5])
            self.temp2_c = float(data[6])
            self.alarm_flags = int(data[7])
            cell_count = max(0, int(data[8]))
            self.cell_mv = [float(v) for v in data[9:9 + cell_count]]

    def snapshot(self):
        with self._lock:
            return {
                "online": self.online,
                "pack_voltage_v": self.pack_voltage_v,
                "pack_current_a": self.pack_current_a,
                "soc": self.soc,
                "alarm_flags": self.alarm_flags,
                "pack_power_w": self.pack_power_w,
                "temp1_c": self.temp1_c,
                "temp2_c": self.temp2_c,
                "cell_mv": list(self.cell_mv),
            }


class BmsMonitorApp:
    def __init__(self, node: BmsGuiNode):
        self.node = node
        self.root = tk.Tk()
        self.root.title("电池监控")
        self.root.geometry("420x220")

        frm = ttk.Frame(self.root, padding=12)
        frm.pack(fill=tk.BOTH, expand=True)

        self.online_var = tk.StringVar(value="离线")
        self.volt_var = tk.StringVar(value="0.00 V")
        self.curr_var = tk.StringVar(value="0.00 A")
        self.soc_var = tk.StringVar(value="0 %")
        self.power_var = tk.StringVar(value="0.0 W")
        self.temp1_var = tk.StringVar(value="0.0 ℃")
        self.temp2_var = tk.StringVar(value="0.0 ℃")
        self.temp_var = tk.StringVar(value="0.0 ℃")
        self.alarm_var = tk.StringVar(value="0x00000000")
        self.cell_stat_var = tk.StringVar(value="单体: 无数据")

        ttk.Label(frm, text="电池状态", font=("Arial", 16, "bold")).pack(anchor=tk.W, pady=(0, 8))
        ttk.Label(frm, textvariable=self.online_var, font=("Arial", 14, "bold")).pack(anchor=tk.W, pady=(0, 10))

        grid = ttk.Frame(frm)
        grid.pack(fill=tk.X)
        self._row(grid, 0, "总电压", self.volt_var)
        self._row(grid, 1, "电量(SOC)", self.soc_var)
        self._row(grid, 2, "温度", self.temp_var)

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._refresh()

    @staticmethod
    def _row(parent, row_idx, key, val_var):
        ttk.Label(parent, text=key + ":", width=14).grid(row=row_idx, column=0, sticky=tk.W, padx=(0, 6), pady=2)
        ttk.Label(parent, textvariable=val_var, width=28).grid(row=row_idx, column=1, sticky=tk.W, pady=2)

    def _refresh(self):
        snap = self.node.snapshot()
        self.online_var.set("在线" if snap["online"] else "离线")
        self.volt_var.set(f'{snap["pack_voltage_v"]:.2f} V')
        self.soc_var.set(f'{snap["soc"]} %')
        temp = snap["temp1_c"] if snap["temp1_c"] != 0.0 else snap["temp2_c"]
        self.temp_var.set(f"{temp:.1f} ℃")

        self.root.after(300, self._refresh)

    def _on_close(self):
        self.root.quit()

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    node = BmsGuiNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    app = BmsMonitorApp(node)
    try:
        app.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
