#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from lidar_auto_docking_messages.msg import Initdock
import math
import json
import threading
import sys
import select
from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class DockPoseSubscriber(Node):
    def __init__(self):
        super().__init__("dock_subscriber")
        self.subscription = self.create_subscription(
            Initdock, "init_dock", self.listener_callback, 10
        )

        self.declare_parameter("load_file_path", "dock_pose.json")
        self.dock_file_path = (
            self.get_parameter("load_file_path").get_parameter_value().string_value
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._output_timer = self.create_timer(0.5, self.on_timer)

        self.x_pos = 0.0
        self.y_pos = 0.0
        self.z_pos = 0.0
        self.w_pos = 0.0

        self.bot_x = 0.0
        self.bot_y = 0.0
        self.bot_z = 0.0
        self.bot_w = 0.0

        # uruchamiamy wątek do obsługi wejścia z klawiatury
        self.stop_event = threading.Event()
        threading.Thread(target=self.keyboard_listener, daemon=True).start()

    async def on_timer(self):
        from_frame = "base_link"
        to_frame = "map"
        when = rclpy.time.Time()
        print(f"\r[INFO] trying to get transform for dock...", end="", flush=True)
        try:
            self.robot_pose = await self._tf_buffer.lookup_transform_async(
                to_frame, from_frame, when
            )
            self.bot_x = self.robot_pose.transform.translation.x
            self.bot_y = self.robot_pose.transform.translation.y
            self.bot_z = self.robot_pose.transform.rotation.z
            self.bot_w = self.robot_pose.transform.rotation.w

            dock_x_diff = abs(self.bot_x - self.x_pos)
            dock_y_diff = abs(self.bot_y - self.y_pos)
            dist_to_dock = math.sqrt(dock_x_diff**2 + dock_y_diff**2)

            print(f"\r[INFO] Distance to dock: {dist_to_dock:.3f} m", end="", flush=True)

        except LookupException as e:
            self.get_logger().warn(f"Failed to get transform: {e!r}")

    def save_dock_callback(self):
        output_dict = {
            "x": self.x_pos,
            "y": self.y_pos,
            "z": self.z_pos,
            "w": self.w_pos,
            "bx": self.bot_x,
            "by": self.bot_y,
            "bz": self.bot_z,
            "bw": self.bot_w,
        }
        with open(self.dock_file_path, "w") as outfile:
            json.dump(output_dict, outfile, indent=2)

        print("\n[SAVED] Dock and robot coordinates have been saved!")

    def listener_callback(self, msg):
        self.x_pos = msg.x
        self.y_pos = msg.y
        self.z_pos = msg.z
        self.w_pos = msg.w

    def keyboard_listener(self):
        print("\n[CTRL+C aby wyjść, naciśnij 's' + Enter aby zapisać dock i robot pose]\n")
        while not self.stop_event.is_set():
            # sprawdzamy czy coś jest na stdin
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline().strip()
                if line == "s":
                    self.save_dock_callback()


def main(args=None):
    rclpy.init(args=args)
    node = DockPoseSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[EXIT] Zatrzymano przez użytkownika.")
    finally:
        node.stop_event.set()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
