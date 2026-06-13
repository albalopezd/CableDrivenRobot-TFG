#!/usr/bin/env python3
# Copyright 2026 Alba López del Águila
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

JOINTS = ["cable_1_joint", "cable_2_joint", "cable_3_joint"]
KP      = 5.0   # ganancia proporcional (m/s por metro de error)
MAX_VEL = 1.0   # velocidad máxima (m/s)
MIN_VEL         = 0.15  # velocidad mínima tensando (speed ~150)
MIN_VEL_RELEASE = 0.185  # velocidad mínima soltando — más fuerza contra rozamiento y peso (speed ~185)
DEADBAND = 0.001  # error < 1mm → parar (evita oscilación en reposo)


class HardwareController(Node):
    def __init__(self):
        super().__init__("hardware_controller")

        self._desired = [0.0, 0.0, 0.0]
        self._current = [0.0, 0.0, 0.0]

        self.create_subscription(JointState, "manual_cable_joint_commands", self._desired_cb, 10)
        self.create_subscription(JointState, "/joint_states", self._current_cb, 10)
        self._cmd_pub = self.create_publisher(Float64MultiArray, 
                                              "/cable_velocity_controller/commands", 10)
        self.create_timer(0.1, self._timer_loop)

    def _desired_cb(self, msg: JointState):
        for i, name in enumerate(JOINTS):
            if name in msg.name:
                idx = msg.name.index(name)
                self._desired[i] = max(-0.015, msg.position[idx])

    def _current_cb(self, msg: JointState):
        for i, name in enumerate(JOINTS):
            if name in msg.name:
                idx = msg.name.index(name)
                self._current[i] = msg.position[idx]

    def _timer_loop(self):
        cmd = Float64MultiArray()
        vels = []
        for i in range(3):
            error = self._desired[i] - self._current[i]
            if abs(error) < DEADBAND:
                vels.append(0.0)
            else:
                vel = KP * error
                vel = max(-MAX_VEL, min(MAX_VEL, vel))
                min_v = MIN_VEL_RELEASE if vel < 0 else MIN_VEL
                if abs(vel) < min_v:
                    vel = min_v * (1.0 if vel > 0 else -1.0)
                vels.append(float(vel))
        cmd.data = vels
        self._cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = HardwareController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
