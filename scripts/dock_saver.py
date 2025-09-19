#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from lidar_auto_docking_messages.msg import Initdock
from std_msgs.msg import Empty
import math
import json
from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_srvs.srv import Trigger

# this is simple dock saver instead of gui app
# call: ros2 service call /save_dock_pose std_srvs/srv/Trigger "{}"
# to save pose into file

class DockPoseSubscriber(Node):
    def __init__(self):
        super().__init__("dock_subscriber")

        # Subscribe dock pose
        self.subscription = self.create_subscription(
            Initdock, "init_dock", self.listener_callback, 10
        )

        # Service to save dock pose
        self.srv = self.create_service(Trigger, "save_dock_pose", self.handle_save)

        # File to save pose
        self.declare_parameter("load_file_path", "dock_pose.json")
        self.dock_file_path = (
            self.get_parameter("load_file_path").get_parameter_value().string_value
        )

        # TF listener
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Params
        self.x_pos = None
        self.y_pos = None
        self.z_pos = None
        self.w_pos = None
        self.bot_x = None
        self.bot_y = None
        self.bot_z = None
        self.bot_w = None

        self.get_logger().info("Dock_Saver ready. Call /save_dock_pose service to save dock pose into file.")

    def listener_callback(self, msg):
        self.x_pos = msg.x
        self.y_pos = msg.y
        self.z_pos = msg.z
        self.w_pos = msg.w
        self.get_logger().info(
            f"[DOCK UPDATE] Dock pose received: ({self.x_pos:.2f}, {self.y_pos:.2f})"
        )

    def handle_save(self, request, response):
        from_frame = "base_link"
        to_frame = "map"

        try:
            tf = self._tf_buffer.lookup_transform(to_frame, from_frame, rclpy.time.Time())

            self.bot_x = tf.transform.translation.x
            self.bot_y = tf.transform.translation.y
            self.bot_z = tf.transform.rotation.z
            self.bot_w = tf.transform.rotation.w

            if self.x_pos is None or self.y_pos is None:
                response.success = False
                response.message = "Dock pose not received yet!"
                return response

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

            response.success = True
            response.message = f"Dock pose saved to {self.dock_file_path}"
            self.get_logger().info(f"[SAVED] {response.message}")

        except LookupException as e:
            response.success = False
            response.message = f"TF error: {e}"
            self.get_logger().error(response.message)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = DockPoseSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Dock saver stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
