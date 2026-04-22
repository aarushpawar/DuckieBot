#!/usr/bin/env python3

import os
import rospy
import cv2
import numpy as np
import yaml
from duckietown.dtros import DTROS, NodeType, DTParam, ParamType
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped

class LaneFollowerNode(DTROS):

    def __init__(self, node_name):
        super(LaneFollowerNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)

        self.veh = os.environ.get("VEHICLE_NAME", "duckiebot")

        self.v_base = DTParam("~v_base", param_type=ParamType.FLOAT, default=0.15)
        self.k_p = DTParam("~k_p", param_type=ParamType.FLOAT, default=1.2)
        self.k_repel = DTParam("~k_repel", param_type=ParamType.FLOAT, default=0.5)

        self.yellow_low = np.array([20, 100, 100])
        self.yellow_high = np.array([35, 255, 255])
        self.white_low = np.array([0, 0, 180])
        self.white_high = np.array([180, 50, 255])

        self.roi_top = 0.6
        self.roi_bot = 0.9

        self.kin = self.load_kinematics()

        self.pub_wheels = rospy.Publisher(
            f"/{self.veh}/wheels_driver_node/wheels_cmd",
            WheelsCmdStamped,
            queue_size=1
        )
        self.sub_image = rospy.Subscriber(
            f"/{self.veh}/camera_node/image/compressed",
            CompressedImage,
            self.cb_image,
            queue_size=1,
            buff_size=2**24
        )

        self.log("Lane Follower Node Initialized.")

    def load_kinematics(self):
        kin_path = f"/data/config/calibrations/kinematics/{self.veh}.yaml"
        defaults = {"gain": 1.0, "trim": 0.0, "baseline": 0.1, "radius": 0.0318, "k": 27.0, "limit": 1.0}
        if os.path.exists(kin_path):
            with open(kin_path, 'r') as f:
                data = yaml.safe_load(f)
                defaults.update(data)
        return defaults

    def steer(self, v, omega):
        k = self.kin['k']
        r = self.kin['radius']
        b = self.kin['baseline']
        g = self.kin['gain']
        t = self.kin['trim']

        v_l = (v - 0.5 * omega * b) / (r * k) * (g - t)
        v_r = (v + 0.5 * omega * b) / (r * k) * (g + t)

        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.Time.now()
        msg.vel_left = np.clip(v_l, -self.kin['limit'], self.kin['limit'])
        msg.vel_right = np.clip(v_r, -self.kin['limit'], self.kin['limit'])
        self.pub_wheels.publish(msg)

    def get_centroid(self, mask):
        pts = cv2.findNonZero(mask)
        if pts is None:
            return None
        return float(np.mean(pts, axis=0)[0][0]) / mask.shape[1]

    def cb_image(self, msg):
        if not self.is_active:
            return

        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        img = cv2.resize(img, (160, 120))
        h = 120

        y1, y2 = int(h * self.roi_top), int(h * self.roi_bot)
        roi = img[y1:y2, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        mask_y = cv2.inRange(hsv, self.yellow_low, self.yellow_high)
        mask_w = cv2.inRange(hsv, self.white_low, self.white_high)

        y_center = self.get_centroid(mask_y)
        w_center = self.get_centroid(mask_w)

        if y_center is not None:
            # Steer to keep yellow centerline at image center
            error = 0.5 - y_center
            omega = error * self.k_p.value * 5.0

            # Repel from white edge if detected
            if w_center is not None:
                if w_center < 0.3:
                    omega -= self.k_repel.value
                elif w_center > 0.7:
                    omega += self.k_repel.value

            v = max(0.0, self.v_base.value * (1.0 - abs(omega) * 0.2))
            rospy.logdebug("FOLLOW_YELLOW omega=%.2f v=%.2f", omega, v)

        elif w_center is not None:
            # White is the right edge: if it's left of center we've drifted left, steer right
            error = w_center - 0.5
            omega = error * self.k_p.value * 3.0
            v = max(0.0, self.v_base.value * 0.5)
            rospy.logdebug("WHITE_RECOVERY omega=%.2f", omega)

        else:
            omega = 0.0
            v = 0.0
            rospy.logdebug("LOST: no lane markings visible")

        self.steer(v, omega)

    def on_shutdown(self):
        self.steer(0, 0)
        super(LaneFollowerNode, self).on_shutdown()

if __name__ == '__main__':
    node = LaneFollowerNode(node_name='lane_follower_node')
    rospy.spin()
