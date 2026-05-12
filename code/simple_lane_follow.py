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
from apriltag_ros.msg import AprilTagDetectionArray
from sensor_msgs.msg import Range

# --- CONFIGURATION ---
HOSTNAME = "nasavpns"
KIN_PATH = f"/data/config/calibrations/kinematics/{HOSTNAME}.yaml"

# HSV Thresholds (Tweak if lighting is very different)
YELLOW_LOW = np.array([20, 100, 100])
YELLOW_HIGH = np.array([35, 255, 255])
WHITE_LOW = np.array([0, 0, 180])
WHITE_HIGH = np.array([180, 50, 255])

TOF_DUCKIE_DIST = 0.25  # meters; tune 0.20–0.35
DUCKIE_STOP_DURATION = 1.5
DUCKIE_COOLDOWN = 3.0

STOP_TAG_ID = 25
STOP_DURATION = 2.0  # seconds stopped at sign
TAG_COOLDOWN = 5.0  # ignore same stop sign briefly after stopping

# Control Params
V_BASE = 0.15  # Base forward velocity
K_P = 1.2  # Proportional gain for yellow centering
K_REPEL = 0.5  # Repulsion strength from white lines
ROI_TOP = 0.6  # Start of steering band (fraction of height)
ROI_BOT = 0.9  # End of steering band


class SimpleLaneFollower:
    def __init__(self):
        rospy.init_node('simple_lane_follow', anonymous=True)
        self.kin = self.load_kinematics()

        # Publishers & Subscribers
        self.pub = rospy.Publisher(f"/{HOSTNAME}/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1)
        self.sub = rospy.Subscriber(f"/{HOSTNAME}/camera_node/image/compressed", CompressedImage, self.cb_image,
                                    queue_size=1, buff_size=2 ** 24)

        self.tag_sub = rospy.Subscriber(f"/{HOSTNAME}/apriltag_detector_node/detections", AprilTagDetectionArray,
                                        self.cb_tags, queue_size=1)

        self.last_error = 0
        self.stop_until = rospy.Time(0)
        self.last_tag_stop_time = rospy.Time(0)

        self.tof_range = None
        self.duckie_stop_until = rospy.Time(0)

        self.tof_sub = rospy.Subscriber(
            f"/{HOSTNAME}/front_center_tof_driver_node/range",
            Range,
            self.cb_tof,
            queue_size=1
        )
        self.last_duckie_stop_time = rospy.Time(0)
        rospy.loginfo("Simple Lane Follower Initialized. Waiting for images...")

    def load_kinematics(self):
        defaults = {"gain": 1.0, "trim": 0.0, "baseline": 0.1, "radius": 0.0318, "k": 27.0, "limit": 1.0}
        if os.path.exists(KIN_PATH):
            with open(KIN_PATH, 'r') as f:
                data = yaml.safe_load(f)
                defaults.update(data)
        return defaults

    def steer(self, v, omega):
        """Convert v, omega to wheel commands using Duckietown kinematics."""
        # Simplified version of the standard kinematics node logic
        k = self.kin['k']
        r = self.kin['radius']
        b = self.kin['baseline']
        g = self.kin['gain']
        t = self.kin['trim']

        # omega_l = (v - 0.5 * omega * b) / r
        # omega_r = (v + 0.5 * omega * b) / r
        # Duty cycles (approximate)
        v_l = (v - 0.5 * omega * b) / (r * k)
        v_r = (v + 0.5 * omega * b) / (r * k)

        # Apply gain and trim
        v_l *= (g - t)
        v_r *= (g + t)

        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.Time.now()
        msg.vel_left = np.clip(v_l, -self.kin['limit'], self.kin['limit'])
        msg.vel_right = np.clip(v_r, -self.kin['limit'], self.kin['limit'])
        self.pub.publish(msg)

    def get_line_stats(self, mask):
        """Returns centroid_x (0-1) and slope if pixels exist, else (None, None)."""
        pts = cv2.findNonZero(mask)
        if pts is None:
            return None, None

        # Centroid
        avg = np.mean(pts, axis=0)[0]
        centroid_x = avg[0] / mask.shape[1]

        # Slope (Regression)
        if len(pts) > 10:
            # x = my + c (since lines are mostly vertical)
            [vx, vy, x, y] = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)
            slope = vx / vy if vy != 0 else 999
            return centroid_x, float(slope)

        return centroid_x, 0.0

    def cb_tags(self, msg):
        now = rospy.Time.now()

        # Already stopping or still in cooldown
        if now < self.stop_until:
            return

        if (now - self.last_tag_stop_time).to_sec() < TAG_COOLDOWN:
            return

        for detection in msg.detections:
            if detection.id[0] == STOP_TAG_ID:
                self.stop_until = now + rospy.Duration(STOP_DURATION)
                self.last_tag_stop_time = now
                break

    def cb_tof(self, msg):
        self.tof_range = msg.range

    def cb_image(self, msg):
        # Decode and Resize
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        h, w = img.shape[:2]
        img = cv2.resize(img, (160, 120))
        h, w = 120, 160

        # Define ROI
        y1, y2 = int(h * ROI_TOP), int(h * ROI_BOT)
        roi = img[y1:y2, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Masks
        mask_y = cv2.inRange(hsv, YELLOW_LOW, YELLOW_HIGH)
        mask_w = cv2.inRange(hsv, WHITE_LOW, WHITE_HIGH)

        # Get stats
        y_center, y_slope = self.get_line_stats(mask_y)
        w_center, w_slope = self.get_line_stats(mask_w)

        now = rospy.Time.now()

        if self.tof_range is not None:
            if self.tof_range < TOF_DUCKIE_DIST:
                if (now - self.last_duckie_stop_time).to_sec() > DUCKIE_COOLDOWN:
                    self.duckie_stop_until = now + rospy.Duration(DUCKIE_STOP_DURATION)
                    self.last_duckie_stop_time = now

        omega = 0
        case = "LOST"

        if y_center is not None:
            # CASE 1: Follow Yellow
            error = 0.5 - y_center  # Positive if yellow is to the left
            omega = error * K_P * 5.0
            case = "FOLLOW_YELLOW"

            # Repulsion from white if it's too close
            if w_center is not None:
                if w_center < 0.3:  # White on left
                    omega -= K_REPEL
                elif w_center > 0.7:  # White on right
                    omega += K_REPEL

        elif w_center is not None:
            # CASE 2: White line recovery
            # Use slope to decide turn direction
            # If slope > 0, line tilts right (top is right of bottom)
            if w_slope > 0:
                omega = -1.5  # Turn right
            else:
                omega = 1.5  # Turn left
            case = "WHITE_RECOVERY"

        else:
            # CASE 3: Blind
            omega = 0
            case = "STOPPED"

        # Adaptive speed: slow down in turns
        v = V_BASE * (1.0 - abs(omega) * 0.2)
        if case == "STOPPED":
            v = 0

        if rospy.Time.now() < self.stop_until:
            v = 0
            omega = 0
            case = "STOP_TAG_TEMP"

        if now < self.duckie_stop_until:
            v = 0
            omega = 0
            case = "STOP_TOF_OBJECT"

        self.steer(v, omega)
        self.draw_telemetry(case, y_center, w_center, omega)

    def draw_telemetry(self, case, y, w, omega):
        # Create an ASCII bar
        width = 40
        bar = [" "] * width
        bar[width // 2] = "|"

        if y is not None:
            idx = int(y * (width - 1))
            bar[idx] = "Y"
        if w is not None:
            idx = int(w * (width - 1))
            bar[idx] = "W"

        bar_str = "".join(bar)
        print(f"\r[{bar_str}] Case: {case:15} Omega: {omega:+.2f}", end="")


if __name__ == '__main__':
    try:
        follower = SimpleLaneFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
