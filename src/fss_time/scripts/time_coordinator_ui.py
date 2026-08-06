#!/usr/bin/env python3

import signal
import os
import sys
import threading

from PyQt5 import uic
from PyQt5.QtCore import QObject, Qt, pyqtSignal
from PyQt5.QtGui import QDoubleValidator, QIcon, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QLayout,
    QMainWindow,
    QSizePolicy,
)

import rclpy
from rclpy.context import Context
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from fss_time_interfaces.msg import SimClockStatus
from fss_time_interfaces.srv import SimClockControl
from rosgraph_msgs.msg import Clock

RTF_SLIDER_MIN = 0
RTF_SLIDER_ONE_X = 100
RTF_SLIDER_TEN_X = 200
RTF_SLIDER_MAX = 300


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


class CoordinatorBridge(QObject):
    status_signal = pyqtSignal(dict)
    clock_signal = pyqtSignal(float)
    progress_signal = pyqtSignal(dict)
    message_signal = pyqtSignal(str)
    namespace_signal = pyqtSignal(str)

    def __init__(self, initial_rtf: float):
        super().__init__()
        self.expected_rtf = max(0.0, float(initial_rtf))
        self.operation_limited_rtf = 0.0
        self.running = False
        self._shutdown = threading.Event()
        self._ros_ready = threading.Event()
        self._context = Context()
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

    def _spin(self):
        rclpy.init(args=None, context=self._context)
        self.node = Node("fss_time_coordinator_ui", context=self._context)
        self.status_sub = self.node.create_subscription(
            SimClockStatus, "fss/sim_clock_status", self._on_status, 10
        )
        self.clock_sub = self.node.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.control_client = self.node.create_client(SimClockControl, "fss/clock_control")
        self.executor = SingleThreadedExecutor(context=self._context)
        self.executor.add_node(self.node)
        self._ros_ready.set()
        self.namespace_signal.emit(self.node.get_namespace())
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
            self._spin_thread.join(timeout=0.5)

    def namespace(self) -> str:
        if not self._ros_ready.is_set():
            return ""
        return self.node.get_namespace()

    def _on_status(self, msg: SimClockStatus):
        sim_time = float(msg.sim_time.sec) + float(msg.sim_time.nanosec) * 1e-9
        self.running = bool(msg.running)
        self.expected_rtf = max(0.0, float(msg.max_real_time_factor))
        observed_rtf = max(0.0, float(msg.observed_real_time_factor))
        speed_regulator_step_ns = int(msg.speed_regulator_step_ns)
        min_operation_walltime = int(msg.min_operation_walltime)
        if min_operation_walltime > 0:
            self.operation_limited_rtf = max(
                0.0, float(speed_regulator_step_ns) / float(min_operation_walltime))
        else:
            self.operation_limited_rtf = 0.0
        self.status_signal.emit(
            {
                "sim_time": sim_time,
                "running": self.running,
                "max_real_time_factor": self.expected_rtf,
                "observed_real_time_factor": observed_rtf,
                "operation_limited_rtf": self.operation_limited_rtf,
                "participant_count": int(msg.participant_count),
                "new_request_participant_count": int(msg.new_request_participant_count),
                "regulator_active": bool(msg.regulator_active),
                "debug_msg": str(msg.debug_msg),
                "speed_regulator_step_ns": speed_regulator_step_ns,
                "min_operation_walltime": min_operation_walltime,
            }
        )
        text = "{:.2f}x / {:.2f}x".format(observed_rtf, self.expected_rtf)
        if self.expected_rtf <= 0.0:
            value = 0
        else:
            value = int(max(0.0, min(observed_rtf / self.expected_rtf, 1.0)) * 100.0)
        self.progress_signal.emit(
            {"actual_rtf": observed_rtf, "value": value, "text": text}
        )

    def _on_clock(self, msg: Clock):
        sim_time = float(msg.clock.sec) + float(msg.clock.nanosec) * 1e-9
        self.clock_signal.emit(sim_time)

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
            self.message_signal.emit(response.message or "coordinator updated")
        else:
            self.message_signal.emit(response.message or "coordinator rejected request")


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self._time_display_seconds = False
        self._last_sim_time = 0.0
        self._slider_max_rtf = 10.0
        ui_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ui", "time_coordinator.ui")
        uic.loadUi(ui_path, self)
        self._configure_compact_layout()
        validator = QDoubleValidator(self)
        validator.setNotation(QDoubleValidator.StandardNotation)
        self.target_rtf_input.setValidator(validator)
        self.target_rtf_input.setAlignment(Qt.AlignRight)
        self.target_rtf_input.setFixedWidth(72)
        self.target_rtf_input.setText("{:.2f}".format(1.0))
        self.label_time_value.mousePressEvent = self._toggle_time_display
        self.speed_dial.mousePressEvent = self._slider_mouse_press
        self.speed_dial.mouseMoveEvent = self._slider_mouse_move
        self._status_detail_widgets = [
            self.label_running_title,
            self.label_running_value,
            self.label_rtf_title,
            self.label_rtf_value,
            self.label_participants_title,
            self.label_participants_value,
            self.label_new_requests_title,
            self.label_new_requests_value,
            self.label_regulator_title,
            self.label_regulator_value,
            self.label_debug_title,
            self.debug_msg_text,
        ]
        self.status_toggle_button.toggled.connect(self.set_status_details_visible)
        self.set_status_details_visible(False)
        self.statusBar().showMessage("waiting for coordinator status")

    def _configure_compact_layout(self):
        for layout in (
            self.root_layout,
            self.status_layout,
            self.control_layout,
            self.observed_layout,
        ):
            layout.setSizeConstraint(QLayout.SetMinimumSize)
        for group in (self.status_group, self.control_group, self.observed_group):
            group.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Minimum)
        self.centralwidget.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Minimum)

    def set_sim_time(self, sim_time: float):
        self._last_sim_time = sim_time
        if self._time_display_seconds:
            self.label_time_value.setText("{:.4f} s".format(sim_time))
        else:
            self.label_time_value.setText(format_sim_time(sim_time))

    def _toggle_time_display(self, _event):
        self._time_display_seconds = not self._time_display_seconds
        self.set_sim_time(self._last_sim_time)

    def set_status_details_visible(self, visible: bool):
        for widget in self._status_detail_widgets:
            widget.setVisible(visible)
        self.status_toggle_button.setArrowType(Qt.DownArrow if visible else Qt.RightArrow)
        self.status_toggle_button.setText("Hide details" if visible else "Show details")
        self.status_toggle_button.setToolTip(
            "Hide coordinator status details" if visible else "Show coordinator status details")
        self.status_group.adjustSize()
        self.centralwidget.adjustSize()
        self.adjustSize()

    def _slider_mouse_press(self, event):
        if event.button() == Qt.LeftButton:
            self.speed_dial.setValue(self._slider_value_from_position(event.pos().x()))
            event.accept()
            return
        event.ignore()

    def _slider_mouse_move(self, event):
        if event.buttons() & Qt.LeftButton:
            self.speed_dial.setValue(self._slider_value_from_position(event.pos().x()))
            event.accept()
            return
        event.ignore()

    def _slider_value_from_position(self, x: int) -> int:
        width = max(1, self.speed_dial.width())
        ratio = max(0.0, min(1.0, float(x) / float(width)))
        return int(round(
            self.speed_dial.minimum() +
            ratio * (self.speed_dial.maximum() - self.speed_dial.minimum())))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.position_scale_labels()

    def showEvent(self, event):
        super().showEvent(event)
        self.position_scale_labels()

    def position_scale_labels(self):
        for label, slider_value in (
            (self.label_scale_01, RTF_SLIDER_MIN),
            (self.label_scale_1, RTF_SLIDER_ONE_X),
            (self.label_scale_5, RTF_SLIDER_TEN_X),
            (self.label_scale_10, RTF_SLIDER_MAX),
        ):
            label.adjustSize()
            center_x = self._slider_position_x(slider_value)
            if slider_value == RTF_SLIDER_MIN:
                x = 0
            elif slider_value == RTF_SLIDER_MAX:
                x = self.scale_widget.width() - label.width()
            else:
                x = int(round(center_x - label.width() / 2.0))
                x = max(0, min(x, self.scale_widget.width() - label.width()))
            label.move(x, 0)

    def _slider_position_x(self, slider_value: int) -> float:
        value_range = max(1, self.speed_dial.maximum() - self.speed_dial.minimum())
        value_offset = max(
            self.speed_dial.minimum(),
            min(self.speed_dial.maximum(), slider_value),
        ) - self.speed_dial.minimum()
        return self.scale_widget.width() * value_offset / value_range

    def set_status(self, status: dict):
        self.set_sim_time(status["sim_time"])
        self.label_running_value.setText("running" if status["running"] else "paused")
        self.label_rtf_value.setText(format_rtf(status["max_real_time_factor"]))
        self.label_participants_value.setText(str(status["participant_count"]))
        self.label_new_requests_value.setText(str(status["new_request_participant_count"]))
        self.label_regulator_value.setText("active" if status["regulator_active"] else "idle")
        self.set_slider_max_rtf(status.get("operation_limited_rtf", 10.0))
        debug_msg = status.get("debug_msg", "")
        if self.debug_msg_text.toPlainText() != debug_msg:
            self.debug_msg_text.setPlainText(debug_msg)

    def set_target_rtf(self, speed: float):
        if self.target_rtf_input.hasFocus():
            return
        self.target_rtf_input.setText("{:.2f}".format(max(0.0, float(speed))))

    def set_slider_max_rtf(self, speed: float):
        self._slider_max_rtf = max(10.0, float(speed))
        self.label_scale_10.setText(format_whole_rtf(self._slider_max_rtf))
        self.position_scale_labels()

    def slider_max_rtf(self) -> float:
        return self._slider_max_rtf

    def set_dial_value(self, value: int):
        self.speed_dial.blockSignals(True)
        self.speed_dial.setValue(value)
        self.speed_dial.blockSignals(False)

    def set_progress(self, progress: dict):
        self.label_actual_rtf_value.setText("{:.2f}x".format(progress["actual_rtf"]))
        self.speed_bar.setValue(progress["value"])
        self.speed_bar.setFormat(progress["text"])


def format_rtf(speed: float) -> str:
    return "{:.2f}x".format(max(0.0, float(speed)))


def format_whole_rtf(speed: float) -> str:
    return "{:.0f}x".format(max(0.0, float(speed)))


def format_sim_time(sim_time: float) -> str:
    total_seconds = max(0.0, float(sim_time))
    hours = int(total_seconds // 3600)
    minutes = int((total_seconds % 3600) // 60)
    seconds = total_seconds - hours * 3600 - minutes * 60
    return "{:02d}:{:02d}:{:07.4f}".format(hours, minutes, seconds)


def dial_to_rtf(dial_value: int, slider_max_rtf: float) -> float:
    value = max(RTF_SLIDER_MIN, min(RTF_SLIDER_MAX, int(dial_value)))
    if value <= RTF_SLIDER_ONE_X:
        return 0.1 + value / RTF_SLIDER_ONE_X * 0.9
    if value <= RTF_SLIDER_TEN_X:
        return 1.0 + (value - RTF_SLIDER_ONE_X) / (RTF_SLIDER_TEN_X - RTF_SLIDER_ONE_X) * 9.0
    max_rtf = max(10.0, float(slider_max_rtf))
    return 10.0 + (value - RTF_SLIDER_TEN_X) / (RTF_SLIDER_MAX - RTF_SLIDER_TEN_X) * (
        max_rtf - 10.0)


def rtf_to_dial(speed: float, slider_max_rtf: float) -> int:
    if speed <= 0.0:
        return RTF_SLIDER_MIN
    max_rtf = max(10.0, float(slider_max_rtf))
    bounded_speed = max(0.1, min(max_rtf, float(speed)))
    if bounded_speed <= 1.0:
        return int(round((bounded_speed - 0.1) / 0.9 * RTF_SLIDER_ONE_X))
    if bounded_speed <= 10.0:
        return RTF_SLIDER_ONE_X + int(round((bounded_speed - 1.0) / 9.0 * (
            RTF_SLIDER_TEN_X - RTF_SLIDER_ONE_X)))
    return RTF_SLIDER_TEN_X + int(round((bounded_speed - 10.0) / (max_rtf - 10.0) * (
        RTF_SLIDER_MAX - RTF_SLIDER_TEN_X)))


class TimeCoordinatorUi(QObject):
    def __init__(self, initial_rtf: float):
        super().__init__()
        self.app = QApplication([sys.argv[0]])
        self.window = MainWindow()
        self._window_title = self.window.windowTitle()
        
        ### Set logo icon
        workspace = os.path.dirname(os.path.abspath(__file__)) # absolute path of this folder 
        icon = QIcon() # 地面站logo
        print(workspace)
        icon.addPixmap(QPixmap(workspace+"/ui/clock.png"), QIcon.Normal, QIcon.Off)
        self.window.setWindowIcon(icon)
        
        self.bridge = CoordinatorBridge(initial_rtf)
        self.window.set_dial_value(rtf_to_dial(initial_rtf, self.window.slider_max_rtf()))
        self.window.set_target_rtf(initial_rtf)

        self.window.start_button.clicked.connect(self._start)
        self.window.pause_button.clicked.connect(self._pause)
        self.window.target_rtf_input.editingFinished.connect(self._manual_rtf_changed)
        self.window.speed_dial.valueChanged.connect(self._dial_changed)
        self._connect_scale_labels()

        self.bridge.status_signal.connect(self._status_updated)
        self.bridge.clock_signal.connect(self.window.set_sim_time)
        self.bridge.progress_signal.connect(self.window.set_progress)
        self.bridge.message_signal.connect(lambda text: self.window.statusBar().showMessage(text, 3000))
        self.bridge.namespace_signal.connect(self._set_namespace)
        namespace = self.bridge.namespace()
        if namespace:
            self._set_namespace(namespace)

    def run(self):
        signal.signal(signal.SIGINT, self._handle_signal)
        self.window.show()
        exit_code = self.app.exec_()
        self.bridge.shutdown()
        sys.exit(exit_code)

    def _handle_signal(self, *_args):
        self.app.quit()

    def _set_namespace(self, namespace: str):
        if namespace == "/":
            self.window.setWindowTitle(self._window_title)
            return
        self.window.setWindowTitle("{} - {}".format(namespace, self._window_title))

    def _status_updated(self, status: dict):
        self.window.set_status(status)
        self.window.set_dial_value(rtf_to_dial(
            status["max_real_time_factor"],
            self.window.slider_max_rtf()))
        self.window.set_target_rtf(status["max_real_time_factor"])

    def _start(self):
        self.bridge.request_control(True, self.bridge.expected_rtf)

    def _pause(self):
        self.bridge.request_control(False, self.bridge.expected_rtf)

    def _dial_changed(self, dial_value: int):
        target_rtf = dial_to_rtf(dial_value, self.window.slider_max_rtf())
        self.window.set_target_rtf(target_rtf)
        self.bridge.request_control(self.bridge.running, target_rtf)

    def _manual_rtf_changed(self):
        try:
            target_rtf = float(self.window.target_rtf_input.text())
        except ValueError:
            target_rtf = self.bridge.expected_rtf
        target_rtf = max(0.0, target_rtf)
        self.window.target_rtf_input.setText("{:.2f}".format(target_rtf))
        self._set_rtf(target_rtf)

    def _connect_scale_labels(self):
        for label, target_rtf in (
            (self.window.label_scale_01, 0.1),
            (self.window.label_scale_1, 1.0),
            (self.window.label_scale_5, 10.0),
            (self.window.label_scale_10, None),
        ):
            label.setCursor(Qt.PointingHandCursor)
            label.mousePressEvent = lambda _event, value=target_rtf: self._set_scale_label_rtf(value)

    def _set_scale_label_rtf(self, target_rtf):
        if target_rtf is None:
            target_rtf = self.window.slider_max_rtf()
        self._set_rtf(target_rtf)

    def _set_rtf(self, target_rtf: float):
        target_rtf = max(0.0, float(target_rtf))
        self.window.set_dial_value(rtf_to_dial(target_rtf, self.window.slider_max_rtf()))
        self.window.set_target_rtf(target_rtf)
        self.bridge.request_control(self.bridge.running, target_rtf)


if __name__ == "__main__":
    initial_rtf = parse_initial_rtf(sys.argv)
    TimeCoordinatorUi(initial_rtf).run()
