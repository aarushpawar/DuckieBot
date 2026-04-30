#!/usr/bin/env python3
"""
simple_lane_follow.py - A simplified, robust lane follower for Duckiebot DB21J.
Focuses on the yellow line (attraction) and uses white lines for repulsion/direction.
"""

import rospy
import cv2
import numpy as np
import yaml
import os
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped

# --- CONFIGURATION ---
HOSTNAME = os.environ.get("VEHICLE_NAME", "nasavpns")
KIN_PATH = f"/data/config/calibrations/kinematics/{HOSTNAME}.yaml"

# HSV Thresholds (Tweak if lighting is very different)
YELLOW_LOW  = np.array([20, 100, 100])
YELLOW_HIGH = np.array([35, 255, 255])
WHITE_LOW   = np.array([0,   0,  180])
WHITE_HIGH  = np.array([179, 50, 255])   # H max is 179 in OpenCV HSV

# Minimum pixel counts to trust a detection
MIN_YELLOW_PX = 50
MIN_WHITE_PX  = 80

# Control Params
V_BASE   = 0.15    # Base forward velocity
K_P      = 1.2     # Proportional gain for yellow centering
K_REPEL  = 0.5     # Repulsion strength from white lines
OMEGA_MAX = 4.0    # Clip omega to keep wheel duty cycles physically meaningful
ROI_TOP  = 0.6     # Start of steering band (fraction of height)
ROI_BOT  = 0.9     # End of steering band


class SimpleLaneFollower:
    def __init__(self):
        rospy.init_node('simple_lane_follow', anonymous=True)
        self.kin = self.load_kinematics()

        # Publishers & Subscribers
        self.pub = rospy.Publisher(
            f"/{HOSTNAME}/wheels_driver_node/wheels_cmd",
            WheelsCmdStamped,
            queue_size=1,
        )
        self.sub = rospy.Subscriber(
            f"/{HOSTNAME}/camera_node/image/compressed",
            CompressedImage,
            self.cb_image,
            queue_size=1,
            buff_size=2**24,
        )

        rospy.loginfo("Simple Lane Follower Initialized. Waiting for images...")

    def load_kinematics(self):
        defaults = {
            "gain": 1.0, "trim": 0.0, "baseline": 0.1,
            "radius": 0.0318, "k": 27.0, "limit": 1.0,
        }
        if os.path.exists(KIN_PATH):
            with open(KIN_PATH, 'r') as f:
                data = yaml.safe_load(f)
                if data:
                    defaults.update(data)
        return defaults

    def steer(self, v, omega):
        """Convert v, omega to wheel commands using Duckietown kinematics."""
        k = self.kin['k']
        r = self.kin['radius']
        b = self.kin['baseline']
        g = self.kin['gain']
        t = self.kin['trim']

        v_l = (v - 0.5 * omega * b) / (r * k)
        v_r = (v + 0.5 * omega * b) / (r * k)

        v_l *= (g - t)
        v_r *= (g + t)

        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.Time.now()
        msg.vel_left  = float(np.clip(v_l, -self.kin['limit'], self.kin['limit']))
        msg.vel_right = float(np.clip(v_r, -self.kin['limit'], self.kin['limit']))
        self.pub.publish(msg)

    def get_centroid(self, mask, min_px):
        """Return normalised x centroid [0,1] or None if fewer than min_px pixels."""
        pts = cv2.findNonZero(mask)
        if pts is None or len(pts) < min_px:
            return None
        avg = np.mean(pts, axis=0)[0]
        return float(avg[0]) / mask.shape[1]

    def cb_image(self, msg):
        # Decode and resize
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is None:
            return
        img = cv2.resize(img, (160, 120))
        h, w = 120, 160

        # Define ROI
        y1, y2 = int(h * ROI_TOP), int(h * ROI_BOT)
        roi = img[y1:y2, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Masks
        mask_y = cv2.inRange(hsv, YELLOW_LOW, YELLOW_HIGH)
        mask_w = cv2.inRange(hsv, WHITE_LOW,  WHITE_HIGH)

        y_center = self.get_centroid(mask_y, MIN_YELLOW_PX)
        w_center = self.get_centroid(mask_w, MIN_WHITE_PX)

        omega = 0.0
        case  = "LOST"

        if y_center is not None:
            # Follow yellow: positive error means yellow is left of target (0.5)
            error = 0.5 - y_center
            omega = error * K_P * 5.0
            case  = "FOLLOW_YELLOW"

            # Repulsion from white when it's on the wrong side
            if w_center is not None:
                if w_center < 0.3:    # white on the left — drift into oncoming
                    omega -= K_REPEL
                elif w_center > 0.7:  # white on the right — normal position, slight nudge
                    omega += K_REPEL * 0.3

        elif w_center is not None:
            # Recovery using centroid position only (no slope — noisy in shallow ROI)
            if w_center < 0.5:
                omega = -1.5   # white on left → turn right
            else:
                omega = 1.5    # white on right → turn left
            case = "WHITE_RECOVERY"

        # Clip omega to keep wheel duty cycles in a physically meaningful range
        omega = float(np.clip(omega, -OMEGA_MAX, OMEGA_MAX))

        # Adaptive speed: slow down in turns; floor at 0.05 to avoid backwards motion
        if case == "LOST":
            v = 0.0
        else:
            v = max(0.05, V_BASE * (1.0 - abs(omega) * 0.2))

        self.steer(v, omega)
        self.draw_telemetry(case, y_center, w_center, omega)

    def draw_telemetry(self, case, y, w, omega):
        width = 40
        bar = [" "] * width
        bar[width // 2] = "|"

        if y is not None:
            bar[int(y * (width - 1))] = "Y"
        if w is not None:
            bar[int(w * (width - 1))] = "W"

        print(f"\r[{''.join(bar)}] Case: {case:15} Omega: {omega:+.2f}", end="")


if __name__ == '__main__':
    try:
        follower = SimpleLaneFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
