# jaka_driver.py
# Encapsulamento fino da SDK JAKA. Esta camada sabe falar com jkrc; não sabe nada de Flask/HTML.

import os
import sys
from typing import Any, Optional

SDK_DIR = r"C:\jakaAPI_V2.1.7stable\SDK2.1.7\Windows\python3\x64"

try:
    sys.path.insert(0, SDK_DIR)
    os.add_dll_directory(SDK_DIR)
    import jkrc  # type: ignore
    SDK_AVAILABLE = True
    SDK_ERROR = None
except Exception as e:  # pragma: no cover - depende do PC com SDK
    jkrc = None
    SDK_AVAILABLE = False
    SDK_ERROR = e


class JakaDriver:
    def __init__(self):
        self.robot = None
        self.ip: Optional[str] = None

    @staticmethod
    def sdk_available() -> bool:
        return SDK_AVAILABLE

    @staticmethod
    def sdk_error() -> str:
        return "" if SDK_ERROR is None else str(SDK_ERROR)

    @staticmethod
    def ret_ok(ret: Any) -> bool:
        if ret == 0 or ret is None:
            return True
        if isinstance(ret, (list, tuple)) and len(ret) > 0 and ret[0] == 0:
            return True
        return False

    def connect(self, ip: str) -> bool:
        if not SDK_AVAILABLE:
            return False
        self.robot = jkrc.RC(ip)
        login_ret = self.robot.login()
        if not self.ret_ok(login_ret):
            print(f"[JAKA] login() falhou: {login_ret}")
            self.robot = None
            return False
        try:
            self.robot.power_on()
        except Exception as e:
            print(f"[JAKA] power_on() falhou/ignorado: {e}")
        try:
            self.robot.enable_robot()
        except Exception as e:
            print(f"[JAKA] enable_robot() falhou/ignorado: {e}")
        try:
            if hasattr(self.robot, "set_status_data_update_time_interval"):
                self.robot.set_status_data_update_time_interval(100)
        except Exception as e:
            print(f"[JAKA] set_status_data_update_time_interval() falhou/ignorado: {e}")
        self.ip = ip
        return True

    def has_robot(self) -> bool:
        return self.robot is not None

    def get_tcp_position(self):
        return self.robot.get_tcp_position()

    def get_robot_status(self):
        return self.robot.get_robot_status()

    def is_in_pos(self):
        if hasattr(self.robot, "is_in_pos"):
            return self.robot.is_in_pos()
        return None

    def linear_move(self, pose, move_mode: int, is_block: bool, speed: float):
        return self.robot.linear_move(pose, move_mode, is_block, speed)

    def circular_move(self, end_pose, mid_pose, move_mode: int, is_block: bool, speed: float, acc: float, tol: float):
        return self.robot.circular_move(end_pose, mid_pose, move_mode, is_block, speed, acc, tol)

    def set_digital_output(self, io_type: int, index: int, value: bool):
        return self.robot.set_digital_output(io_type, index, value)

    def get_digital_output(self, io_type: int, index: int):
        if hasattr(self.robot, "get_digital_output"):
            return self.robot.get_digital_output(io_type, index)
        return None

    def jog(self, eixo: int, jog_mode: int, coord: int, vel: float, acc: float):
        return self.robot.jog(eixo, jog_mode, coord, vel, acc)

    def jog_stop(self, eixo: int):
        return self.robot.jog_stop(eixo)

    def motion_abort(self):
        if hasattr(self.robot, "motion_abort"):
            return self.robot.motion_abort()
        if hasattr(self.robot, "stop_move"):
            return self.robot.stop_move()
        return None
