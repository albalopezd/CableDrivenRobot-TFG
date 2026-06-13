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
from geometry_msgs.msg import PoseStamped
import csv, signal, sys
from datetime import datetime

CABLES = ["cable_1_joint", "cable_2_joint", "cable_3_joint"]


class DataLogger(Node):
    def __init__(self):
        super().__init__("data_logger")

        self._desired = {c: 0.0 for c in CABLES}
        self._actual = {c: 0.0 for c in CABLES}
        self._pose = {"x": 0.0, "y": 0.0, "z": 0.0}
        self._rows = []
        self._t0 = None

        self.create_subscription(JointState, "manual_cable_joint_commands", self._desired_cb, 10)
        self.create_subscription(JointState, "/joint_states", self._actual_cb, 10)
        self.create_subscription(PoseStamped, "estimated_pose", self._pose_cb, 10)
        self.create_timer(0.1, self._log)


    def _desired_cb(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in self._desired:
                self._desired[name] = msg.position[i]

    def _actual_cb(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in self._actual:
                self._actual[name] = msg.position[i]

    def _pose_cb(self, msg: PoseStamped):
        self._pose["x"] = msg.pose.position.x
        self._pose["y"] = msg.pose.position.y
        self._pose["z"] = msg.pose.position.z

    def _log(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self._t0 is None:
            self._t0 = now
        t = now - self._t0

        self._rows.append({
            "t": round(t, 3),
            "des_c1": self._desired["cable_1_joint"],
            "des_c2": self._desired["cable_2_joint"],
            "des_c3": self._desired["cable_3_joint"],
            "act_c1": self._actual["cable_1_joint"],
            "act_c2": self._actual["cable_2_joint"],
            "act_c3": self._actual["cable_3_joint"],
            "err_c1": self._desired["cable_1_joint"] - self._actual["cable_1_joint"],
            "err_c2": self._desired["cable_2_joint"] - self._actual["cable_2_joint"],
            "err_c3": self._desired["cable_3_joint"] - self._actual["cable_3_joint"],
            "pose_x": self._pose["x"],
            "pose_y": self._pose["y"],
            "pose_z": self._pose["z"],
        })

    def save(self):
        if not self._rows:
            self.get_logger().warn("Sin datos, no se guarda nada")
            return
        fname = f"cable_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        with open(fname, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=self._rows[0].keys())
            writer.writeheader()
            writer.writerows(self._rows)


def main():
    rclpy.init()
    node = DataLogger()

    def shutdown(sig, frame):
        node.save()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
