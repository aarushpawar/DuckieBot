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
        # Initialize the DTROS parent class
        super(LaneFollowerNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        
        # Get vehicle name
        self.veh = os.environ.get("VEHICLE_NAME", "duckiebot")
        
        # Parameters
        self.v_base = DTParam("~v_base", param_type=ParamType.FLOAT, default=0.15)
        self.k_p = DTParam("~k_p", param_type=ParamType.FLOAT, default=1.2)
        self.k_repel = DTParam("~k_repel", param_type=ParamType.FLOAT, default=0.5)
        
        # HSV Thresholds
        self.yellow_low = np.array([20, 100, 100])
        self.yellow_high = np.array([35, 255, 255])
        self.white_low = np.array([0, 0, 180])
        self.white_high = np.array([180, 50, 255])
        
        # ROI Params
        self.roi_top = 0.6
        self.roi_bot = 0.9

        # Load kinematics
        self.kin = self.load_kinematics()

        # Publishers & Subscribers
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
        msg.vel_left = np.clip(v_l, -self.kin['limit'], self.kin['limit'])
        msg.vel_right = np.clip(v_r, -self.kin['limit'], self.kin['limit'])
        self.pub_wheels.publish(msg)

    def get_line_stats(self, mask):
        pts = cv2.findNonZero(mask)
        if pts is None:
            return None, None
        
        avg = np.mean(pts, axis=0)[0]
        centroid_x = avg[0] / mask.shape[1]
        
        if len(pts) > 10:
            [vx, vy, x, y] = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)
            slope = vx / vy if vy != 0 else 999
            return centroid_x, float(slope)
        
        return centroid_x, 0.0

    def cb_image(self, msg):
        if not self.is_active:
            return

        # Decode and Resize
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        h, w = img.shape[:2]
        img = cv2.resize(img, (160, 120))
        h, w = 120, 160
        
        # Define ROI
        y1, y2 = int(h * self.roi_top), int(h * self.roi_bot)
        roi = img[y1:y2, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Masks
        mask_y = cv2.inRange(hsv, self.yellow_low, self.yellow_high)
        mask_w = cv2.inRange(hsv, self.white_low, self.white_high)
        
        # Get stats
        y_center, y_slope = self.get_line_stats(mask_y)
        w_center, w_slope = self.get_line_stats(mask_w)
        
        omega = 0
        case = "LOST"
        
        if y_center is not None:
            error = 0.5 - y_center
            omega = error * self.k_p.value * 5.0 
            case = "FOLLOW_YELLOW"
            
            if w_center is not None:
                if w_center < 0.3:
                    omega -= self.k_repel.value
                elif w_center > 0.7:
                    omega += self.k_repel.value

        elif w_center is not None:
            if w_slope > 0:
                omega = -1.5
            else:
                omega = 1.5
            case = "WHITE_RECOVERY"
        else:
            omega = 0
            case = "STOPPED"

        v = self.v_base.value * (1.0 - abs(omega) * 0.2)
        if case == "STOPPED": v = 0
        
        self.steer(v, omega)

    def on_shutdown(self):
        self.steer(0, 0)
        super(LaneFollowerNode, self).on_shutdown()

if __name__ == '__main__':
    node = LaneFollowerNode(node_name='lane_follower_node')
    rospy.spin()
