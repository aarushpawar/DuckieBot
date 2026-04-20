#!/usr/bin/env python3

# Update NODE_VERSION on every change (see CLAUDE.md § Versioning)
NODE_VERSION = "1.1.0"

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped
from std_msgs.msg import String


class ShapeDriverNode(DTROS):
    def __init__(self, node_name):
        super(ShapeDriverNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        self.veh = os.environ.get("VEHICLE_NAME", "duckiebot")

        self.pub_car_cmd = rospy.Publisher(
            f"/{self.veh}/kinematics_node/car_cmd",
            Twist2DStamped,
            queue_size=1
        )

        rospy.Subscriber(
            f"/{self.veh}/shape_driver_node/command",
            String,
            self.cb_command
        )

        self.current_shape = "rectangle"
        self.log(f"Shape Driver v{NODE_VERSION} ready. Running rectangle. Publish to ~command to change: circle, rectangle, triangle, stop")

    def cb_command(self, msg):
        cmd = msg.data.strip().lower()
        if cmd in ["circle", "rectangle", "triangle", "stop"]:
            self.current_shape = cmd
            self.log(f"Switching to: {cmd}")
        else:
            self.log(f"Unknown command: {cmd}")

    def publish_cmd(self, v, omega):
        msg = Twist2DStamped()
        msg.header.stamp = rospy.Time.now()
        msg.v = v
        msg.omega = omega
        self.pub_car_cmd.publish(msg)

    def drive_circle(self):
        self.publish_cmd(0.2, 2.0)
        rospy.sleep(0.1)

    def drive_rectangle(self):
        for _ in range(4):
            if self.current_shape != "rectangle":
                return
            t_end = rospy.Time.now() + rospy.Duration(2.0)
            while rospy.Time.now() < t_end and self.current_shape == "rectangle":
                self.publish_cmd(0.2, 0.0)
                rospy.sleep(0.1)
            t_end = rospy.Time.now() + rospy.Duration(1.2)
            while rospy.Time.now() < t_end and self.current_shape == "rectangle":
                self.publish_cmd(0.0, 3.0)
                rospy.sleep(0.1)

    def drive_triangle(self):
        for _ in range(3):
            if self.current_shape != "triangle":
                return
            t_end = rospy.Time.now() + rospy.Duration(2.0)
            while rospy.Time.now() < t_end and self.current_shape == "triangle":
                self.publish_cmd(0.2, 0.0)
                rospy.sleep(0.1)
            t_end = rospy.Time.now() + rospy.Duration(1.6)
            while rospy.Time.now() < t_end and self.current_shape == "triangle":
                self.publish_cmd(0.0, 3.0)
                rospy.sleep(0.1)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.current_shape == "circle":
                self.drive_circle()
            elif self.current_shape == "rectangle":
                self.drive_rectangle()
            elif self.current_shape == "triangle":
                self.drive_triangle()
            else:
                self.publish_cmd(0.0, 0.0)
            rate.sleep()

    def on_shutdown(self):
        self.publish_cmd(0.0, 0.0)
        super(ShapeDriverNode, self).on_shutdown()


if __name__ == '__main__':
    node = ShapeDriverNode(node_name='shape_driver_node')
    node.run()
