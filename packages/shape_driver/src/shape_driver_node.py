#!/usr/bin/env python3

import os
import rospy
import time
import threading
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped

class ShapeDriverNode(DTROS):
    def __init__(self, node_name):
        super(ShapeDriverNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        self.veh = os.environ.get("VEHICLE_NAME", "duckiebot")
        
        # Publisher for car commands (v, omega)
        self.pub_car_cmd = rospy.Publisher(
            f"/{self.veh}/joy_mapper_node/car_cmd", 
            Twist2DStamped, 
            queue_size=1
        )

        self.current_shape = "stop"
        self.stop_requested = False
        
        # Start input thread
        self.input_thread = threading.Thread(target=self.read_console)
        self.input_thread.daemon = True
        self.input_thread.start()

        self.log("Shape Driver Initialized. Commands: circle, rectangle, triangle, stop")

    def read_console(self):
        while not rospy.is_shutdown():
            cmd = input("\nEnter shape (circle, rectangle, triangle, stop): ").strip().lower()
            if cmd in ["circle", "rectangle", "triangle", "stop"]:
                self.current_shape = cmd
                self.log(f"Switching to: {cmd}")
            else:
                print("Invalid command.")

    def publish_cmd(self, v, omega):
        msg = Twist2DStamped()
        msg.v = v
        msg.omega = omega
        self.pub_car_cmd.publish(msg)

    def drive_circle(self):
        self.log("Driving Circle...")
        self.publish_cmd(0.2, 2.0)
        time.sleep(0.1)

    def drive_rectangle(self):
        self.log("Driving Rectangle...")
        for _ in range(4):
            if self.current_shape != "rectangle": break
            # Forward
            t_end = time.time() + 2.0
            while time.time() < t_end and self.current_shape == "rectangle":
                self.publish_cmd(0.2, 0.0)
                time.sleep(0.1)
            # Turn 90 deg
            t_end = time.time() + 1.2 # Adjust for ~90 deg
            while time.time() < t_end and self.current_shape == "rectangle":
                self.publish_cmd(0.0, 3.0)
                time.sleep(0.1)

    def drive_triangle(self):
        self.log("Driving Triangle...")
        for _ in range(3):
            if self.current_shape != "triangle": break
            # Forward
            t_end = time.time() + 2.0
            while time.time() < t_end and self.current_shape == "triangle":
                self.publish_cmd(0.2, 0.0)
                time.sleep(0.1)
            # Turn 120 deg
            t_end = time.time() + 1.6 # Adjust for ~120 deg
            while time.time() < t_end and self.current_shape == "triangle":
                self.publish_cmd(0.0, 3.0)
                time.sleep(0.1)

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
