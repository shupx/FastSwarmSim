#!/usr/bin/env python3

import signal
import sys
import threading
import time

from PyQt5.QtCore import QObject, Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication,
    QDial,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QProgressBar,
    QVBoxLayout,
    QWidget,
)

import rclpy
from rclpy.context import Context
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from fss_time_interfaces.msg import SimClockStatus
from fss_time_interfaces.srv import SimClockControl
from rosgraph_msgs.msg import Clock


def parse_initial_rtf(argv):
    ros_arg_prefixes = ("--ros-args", "-r", "--remap", "-p", "--param", "--params-file", "__node", "__ns")
    index = 1
    while index < len(argv):
        arg = argv[index]
        if arg == "--":
            index += 1
            continue
        if arg == "--ros-args":
            index += 1
            while index < len(argv) and argv[index] != "--":
                current = argv[index]
                if current in ("-r", "--remap", "-p", "--param", "--params-file"):
                    index += 2
                else:
                    index += 1
            continue
        if arg.startswith(ros_arg_prefixes):
            index += 1
            continue
        try:
            return float(arg)
        except ValueError:
            index += 1
    return 1.0


class BrokerBridge(QObject):
    status_signal = pyqtSignal(dict)
    clock_signal = pyqtSignal(float)
    progress_signal = pyqtSignal(dict)
    message_signal = pyqtSignal(str)

    def __init__(self, initial_rtf: float):
        super().__init__()
        self.expected_rtf = max(0.0, float(initial_rtf))
        self.running = False
        self._last_clock_time = 0.0
        self._last_speed_sim_time = 0.0
        self._last_speed_wall_time = time.monotonic()
        self._lock = threading.Lock()
        self._shutdown = threading.Event()
        self._ros_ready = threading.Event()
        self._context = Context()
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._progress_thread = threading.Thread(target=self._progress_loop, daemon=True)
        self._spin_thread.start()
        self._progress_thread.start()

    def _spin(self):
        rclpy.init(args=None, context=self._context)
        self.node = Node("fss_sim_time_broker_ui", context=self._context)
        self.status_sub = self.node.create_subscription(
            SimClockStatus, "/fss/sim_clock_status", self._on_status, 10
        )
        self.clock_sub = self.node.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.control_client = self.node.create_client(SimClockControl, "/fss/clock_control")
        self.executor = SingleThreadedExecutor(context=self._context)
        self.executor.add_node(self.node)
        self._ros_ready.set()
        try:
            while not self._shutdown.is_set() and self._context.ok():
                self.executor.spin_once(timeout_sec=0.1)
        except ExternalShutdownException:
            pass
        finally:
            self._ros_ready.clear()
            if hasattr(self, "executor") and hasattr(self, "node"):
                try:
                    self.executor.remove_node(self.node)
                except Exception:
                    pass
            if hasattr(self, "node"):
                try:
                    self.node.destroy_node()
                except Exception:
                    pass
            if self._context.ok():
                try:
                    rclpy.shutdown(context=self._context)
                except Exception:
                    pass

    def shutdown(self):
        self._shutdown.set()
        if self._context.ok():
            try:
                self._context.shutdown()
            except Exception:
                pass
        if self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)
        if self._progress_thread.is_alive():
            self._progress_thread.join(timeout=1.0)

    def _on_status(self, msg: SimClockStatus):
        sim_time = float(msg.sim_time.sec) + float(msg.sim_time.nanosec) * 1e-9
        self.running = bool(msg.running)
        self.expected_rtf = max(0.0, float(msg.max_real_time_factor))
        self.status_signal.emit(
            {
                "sim_time": sim_time,
                "running": self.running,
                "max_real_time_factor": self.expected_rtf,
                "participant_count": int(msg.participant_count),
                "regulator_active": bool(msg.regulator_active),
            }
        )

    def _on_clock(self, msg: Clock):
        sim_time = float(msg.clock.sec) + float(msg.clock.nanosec) * 1e-9
        with self._lock:
            self._last_clock_time = sim_time
        self.clock_signal.emit(sim_time)

    def _progress_loop(self):
        while not self._shutdown.is_set():
            time.sleep(0.2)
            with self._lock:
                current_sim_time = self._last_clock_time
            current_wall_time = time.monotonic()
            wall_dt = max(current_wall_time - self._last_speed_wall_time, 1e-6)
            actual_rtf = max(0.0, (current_sim_time - self._last_speed_sim_time) / wall_dt)
            if self.expected_rtf <= 0.0:
                text = "{:.2f}x / unbounded".format(actual_rtf)
                value = 100 if actual_rtf > 0.0 else 0
            else:
                text = "{:.2f}x / {:.2f}x".format(actual_rtf, self.expected_rtf)
                value = int(max(0.0, min(actual_rtf / self.expected_rtf, 1.0)) * 100.0)
            self.progress_signal.emit(
                {"actual_rtf": actual_rtf, "value": value, "text": text}
            )
            self._last_speed_wall_time = current_wall_time
            self._last_speed_sim_time = current_sim_time

    def request_control(self, running: bool, max_real_time_factor: float):
        if not self._ros_ready.wait(timeout=2.0):
            self.message_signal.emit("ROS 2 node not ready")
            return
        if not self.control_client.wait_for_service(timeout_sec=0.2):
            self.message_signal.emit("/fss/clock_control unavailable")
            return
        request = SimClockControl.Request()
        request.running = bool(running)
        request.max_real_time_factor = max(0.0, float(max_real_time_factor))
        future = self.control_client.call_async(request)
        future.add_done_callback(self._on_control_response)

    def _on_control_response(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.message_signal.emit("control failed: {}".format(exc))
            return
        if response.success:
            self.message_signal.emit(response.message or "broker updated")
        else:
            self.message_signal.emit(response.message or "broker rejected request")


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Sim Time Broker")
        self._build_ui()

    def _build_ui(self):
        central = QWidget(self)
        root_layout = QVBoxLayout(central)

        status_group = QGroupBox("Broker Status", central)
        status_layout = QGridLayout(status_group)
        self.label_time_value = self._add_status_row(status_layout, 0, "Sim time", "0 s", True)
        self.label_running_value = self._add_status_row(status_layout, 1, "State", "paused")
        self.label_rtf_value = self._add_status_row(status_layout, 2, "Max RTF", "1.00x")
        self.label_participants_value = self._add_status_row(status_layout, 3, "Participants", "0")
        self.label_regulator_value = self._add_status_row(status_layout, 4, "Regulator", "idle")
        root_layout.addWidget(status_group)

        control_group = QGroupBox("Control", central)
        control_layout = QVBoxLayout(control_group)
        button_layout = QHBoxLayout()
        self.start_button = QPushButton("Start", control_group)
        self.pause_button = QPushButton("Pause", control_group)
        self.unbounded_button = QPushButton("Unbounded", control_group)
        button_layout.addWidget(self.start_button)
        button_layout.addWidget(self.pause_button)
        button_layout.addWidget(self.unbounded_button)
        control_layout.addLayout(button_layout)

        target_layout = QHBoxLayout()
        target_layout.addWidget(QLabel("Target RTF", control_group))
        self.label_target_rtf_value = QLabel("1.00x", control_group)
        self.label_target_rtf_value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        target_layout.addWidget(self.label_target_rtf_value)
        control_layout.addLayout(target_layout)

        self.speed_dial = QDial(control_group)
        self.speed_dial.setMinimum(0)
        self.speed_dial.setMaximum(49)
        self.speed_dial.setSingleStep(1)
        self.speed_dial.setPageStep(5)
        self.speed_dial.setNotchesVisible(True)
        control_layout.addWidget(self.speed_dial)

        scale_layout = QHBoxLayout()
        for text in ["0.1x", "1x", "10x", "100x"]:
            label = QLabel(text, control_group)
            label.setAlignment(Qt.AlignCenter)
            scale_layout.addWidget(label)
        control_layout.addLayout(scale_layout)
        root_layout.addWidget(control_group)

        observed_group = QGroupBox("Observed Clock", central)
        observed_layout = QVBoxLayout(observed_group)
        actual_layout = QHBoxLayout()
        actual_layout.addWidget(QLabel("Measured RTF", observed_group))
        self.label_actual_rtf_value = QLabel("0.00x", observed_group)
        self.label_actual_rtf_value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        actual_layout.addWidget(self.label_actual_rtf_value)
        observed_layout.addLayout(actual_layout)

        self.speed_bar = QProgressBar(observed_group)
        self.speed_bar.setRange(0, 100)
        self.speed_bar.setValue(0)
        observed_layout.addWidget(self.speed_bar)
        root_layout.addWidget(observed_group)

        self.setCentralWidget(central)
        self.statusBar().showMessage("waiting for broker status")
        self.resize(320, 380)

    def _add_status_row(
        self, layout: QGridLayout, row: int, title: str, value: str, emphasize: bool = False
    ) -> QLabel:
        title_label = QLabel(title, self)
        value_label = QLabel(value, self)
        value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        if emphasize:
            font = value_label.font()
            font.setPointSize(14)
            font.setBold(True)
            value_label.setFont(font)
        layout.addWidget(title_label, row, 0, 1, 1)
        layout.addWidget(value_label, row, 1, 1, 1)
        return value_label

    def set_sim_time(self, sim_time: float):
        self.label_time_value.setText("{:.4f} s".format(sim_time))

    def set_status(self, status: dict):
        self.set_sim_time(status["sim_time"])
        self.label_running_value.setText("running" if status["running"] else "paused")
        self.label_rtf_value.setText(format_rtf(status["max_real_time_factor"]))
        self.label_participants_value.setText(str(status["participant_count"]))
        self.label_regulator_value.setText("active" if status["regulator_active"] else "idle")

    def set_target_rtf(self, text: str):
        self.label_target_rtf_value.setText(text)

    def set_dial_value(self, value: int):
        self.speed_dial.blockSignals(True)
        self.speed_dial.setValue(value)
        self.speed_dial.blockSignals(False)

    def set_progress(self, progress: dict):
        self.label_actual_rtf_value.setText("{:.2f}x".format(progress["actual_rtf"]))
        self.speed_bar.setValue(progress["value"])
        self.speed_bar.setFormat(progress["text"])


def format_rtf(speed: float) -> str:
    return "unbounded" if speed <= 0.0 else "{:.2f}x".format(speed)


def dial_to_rtf(dial_value: int) -> float:
    if dial_value < 10:
        return 0.1 + dial_value / 10.0 * (1.0 - 0.1)
    if dial_value < 20:
        return 1.0 + (dial_value - 10) / 10.0
    if dial_value < 30:
        return 2.0 + (dial_value - 20) / 10.0 * 8.0
    if dial_value < 40:
        return 10.0 + (dial_value - 30)
    return 20.0 + (dial_value - 40) / 9.0 * 80.0


def rtf_to_dial(speed: float) -> int:
    if speed <= 0.0:
        return 0
    if speed < 1.0:
        return max(0, min(9, int((speed - 0.1) / 0.9 * 10.0)))
    if speed < 2.0:
        return int(10 + (speed - 1.0) * 10.0)
    if speed < 10.0:
        return int(20 + (speed - 2.0) / 8.0 * 10.0)
    if speed < 20.0:
        return int(30 + (speed - 10.0))
    if speed < 100.0:
        return int(40 + (speed - 20.0) / 80.0 * 9.0)
    return 49


class SimTimeBrokerUi(QObject):
    def __init__(self, initial_rtf: float):
        super().__init__()
        self.app = QApplication(sys.argv)
        self.window = MainWindow()
        self.bridge = BrokerBridge(initial_rtf)
        self.window.set_dial_value(rtf_to_dial(initial_rtf))
        self.window.set_target_rtf(format_rtf(initial_rtf))

        self.window.start_button.clicked.connect(self._start)
        self.window.pause_button.clicked.connect(self._pause)
        self.window.unbounded_button.clicked.connect(self._set_unbounded)
        self.window.speed_dial.valueChanged.connect(self._dial_changed)

        self.bridge.status_signal.connect(self._status_updated)
        self.bridge.clock_signal.connect(self.window.set_sim_time)
        self.bridge.progress_signal.connect(self.window.set_progress)
        self.bridge.message_signal.connect(lambda text: self.window.statusBar().showMessage(text, 3000))

    def run(self):
        signal.signal(signal.SIGINT, self._handle_signal)
        self.window.show()
        exit_code = self.app.exec_()
        self.bridge.shutdown()
        sys.exit(exit_code)

    def _handle_signal(self, *_args):
        self.bridge.shutdown()
        self.app.quit()

    def _status_updated(self, status: dict):
        self.window.set_status(status)
        self.window.set_dial_value(rtf_to_dial(status["max_real_time_factor"]))
        self.window.set_target_rtf(format_rtf(status["max_real_time_factor"]))

    def _start(self):
        self.bridge.request_control(True, self.bridge.expected_rtf)

    def _pause(self):
        self.bridge.request_control(False, self.bridge.expected_rtf)

    def _set_unbounded(self):
        self.window.set_target_rtf("unbounded")
        self.bridge.request_control(self.bridge.running, 0.0)

    def _dial_changed(self, dial_value: int):
        target_rtf = dial_to_rtf(dial_value)
        self.window.set_target_rtf(format_rtf(target_rtf))
        self.bridge.request_control(self.bridge.running, target_rtf)


if __name__ == "__main__":
    initial_rtf = parse_initial_rtf(sys.argv)
    SimTimeBrokerUi(initial_rtf).run()
