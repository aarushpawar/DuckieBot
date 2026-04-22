#!/usr/bin/env python3

import os
import io
import json
import time
import threading
import rospy
import cv2
import numpy as np
import yaml
from duckietown.dtros import DTROS, NodeType, DTParam, ParamType
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import Bool

try:
    from flask import Flask, Response, jsonify, request
    from flask_cors import CORS
    FLASK_AVAILABLE = True
except ImportError:
    FLASK_AVAILABLE = False


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

        # Runtime state
        self._lane_active = True
        self._last_omega = 0.0
        self._last_v = 0.0
        self._last_y_center = None
        self._last_w_center = None
        self._last_state = "INIT"
        self._frame_count = 0
        self._fps = 0.0
        self._fps_ts = time.time()
        self._debug_jpeg = None
        self._debug_lock = threading.Lock()
        self._stats_lock = threading.Lock()

        self.pub_wheels = rospy.Publisher(
            f"/{self.veh}/wheels_driver_node/wheels_cmd",
            WheelsCmdStamped,
            queue_size=1
        )
        self.pub_debug = rospy.Publisher(
            f"/{self.veh}/lane_follower_node/debug/image/compressed",
            CompressedImage,
            queue_size=1
        )
        self.sub_image = rospy.Subscriber(
            f"/{self.veh}/camera_node/image/compressed",
            CompressedImage,
            self.cb_image,
            queue_size=1,
            buff_size=2**24
        )
        self.sub_enable = rospy.Subscriber(
            f"/{self.veh}/lane_follower_node/enable",
            Bool,
            self.cb_enable,
            queue_size=1
        )

        if FLASK_AVAILABLE:
            self._start_flask()
        else:
            self.log("Flask not available — HTTP bridge disabled. Install flask flask-cors.")

        self.log("Lane Follower Node Initialized.")

    def _start_flask(self):
        app = Flask(__name__)
        CORS(app)
        node = self

        @app.route("/status")
        def status():
            with node._stats_lock:
                kin = node.kin.copy()
                return jsonify({
                    "active": node._lane_active,
                    "veh": node.veh,
                    "state": node._last_state,
                    "omega": round(node._last_omega, 4),
                    "v": round(node._last_v, 4),
                    "y_center": round(node._last_y_center, 4) if node._last_y_center is not None else None,
                    "w_center": round(node._last_w_center, 4) if node._last_w_center is not None else None,
                    "fps": round(node._fps, 1),
                    "frame_count": node._frame_count,
                    "kinematics": {k: round(float(v), 5) for k, v in kin.items()},
                    "params": {
                        "v_base": round(node.v_base.value, 4),
                        "k_p": round(node.k_p.value, 4),
                        "k_repel": round(node.k_repel.value, 4),
                    }
                })

        @app.route("/start", methods=["POST"])
        def start():
            node._lane_active = True
            node.log("Lane following ENABLED via HTTP bridge")
            return jsonify({"ok": True, "active": True})

        @app.route("/stop", methods=["POST"])
        def stop():
            node._lane_active = False
            node.steer(0, 0)
            node.log("Lane following DISABLED via HTTP bridge")
            return jsonify({"ok": True, "active": False})

        @app.route("/debug_image.jpg")
        def debug_image():
            with node._debug_lock:
                data = node._debug_jpeg
            if data is None:
                return Response("No frame yet", status=503)
            return Response(data, mimetype="image/jpeg",
                            headers={"Cache-Control": "no-cache, no-store"})

        @app.route("/debug_stream")
        def debug_stream():
            def generate():
                last = None
                while True:
                    with node._debug_lock:
                        data = node._debug_jpeg
                    if data is not None and data is not last:
                        last = data
                        yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + data + b"\r\n")
                    time.sleep(0.05)
            return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")

        t = threading.Thread(
            target=lambda: app.run(host="0.0.0.0", port=8765, debug=False, use_reloader=False),
            daemon=True
        )
        t.start()
        self.log("Flask HTTP bridge running on :8765")

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

    def _build_debug_frame(self, roi, mask_y, mask_w, y_center, w_center, v, omega, state):
        vis = roi.copy()
        h, w = vis.shape[:2]

        # Yellow mask overlay (semi-transparent amber)
        y_overlay = np.zeros_like(vis)
        y_overlay[mask_y > 0] = [0, 200, 255]
        vis = cv2.addWeighted(vis, 1.0, y_overlay, 0.5, 0)

        # White mask overlay (semi-transparent blue-white)
        w_overlay = np.zeros_like(vis)
        w_overlay[mask_w > 0] = [255, 200, 100]
        vis = cv2.addWeighted(vis, 1.0, w_overlay, 0.4, 0)

        # Yellow centroid line
        if y_center is not None:
            cx = int(y_center * w)
            cv2.line(vis, (cx, 0), (cx, h), (0, 220, 255), 2)
            cv2.circle(vis, (cx, h // 2), 4, (0, 220, 255), -1)

        # White centroid line
        if w_center is not None:
            cx = int(w_center * w)
            cv2.line(vis, (cx, 0), (cx, h), (255, 200, 80), 1)
            cv2.circle(vis, (cx, h // 2), 3, (255, 200, 80), -1)

        # Center reference line
        cv2.line(vis, (w // 2, 0), (w // 2, h), (80, 80, 80), 1)

        # State label
        color = (0, 220, 80) if state == "FOLLOW_YELLOW" else (80, 180, 255) if state == "WHITE_RECOVERY" else (60, 60, 200)
        cv2.putText(vis, state, (3, h - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1, cv2.LINE_AA)
        cv2.putText(vis, f"v={v:.2f} w={omega:.2f}", (3, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (200, 200, 200), 1, cv2.LINE_AA)

        return vis

    def cb_enable(self, msg):
        self._lane_active = msg.data
        self.log(f"Lane following {'ENABLED' if msg.data else 'DISABLED'} via ROS topic")

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
            error = 0.5 - y_center
            omega = error * self.k_p.value * 5.0
            if w_center is not None:
                if w_center < 0.3:
                    omega -= self.k_repel.value
                elif w_center > 0.7:
                    omega += self.k_repel.value
            v = max(0.0, self.v_base.value * (1.0 - abs(omega) * 0.2))
            state = "FOLLOW_YELLOW"
            rospy.logdebug("FOLLOW_YELLOW omega=%.2f v=%.2f", omega, v)
        elif w_center is not None:
            error = w_center - 0.5
            omega = error * self.k_p.value * 3.0
            v = max(0.0, self.v_base.value * 0.5)
            state = "WHITE_RECOVERY"
            rospy.logdebug("WHITE_RECOVERY omega=%.2f", omega)
        else:
            omega = 0.0
            v = 0.0
            state = "LOST"
            rospy.logdebug("LOST: no lane markings visible")

        if self._lane_active:
            self.steer(v, omega)

        # Update stats
        self._frame_count += 1
        now = time.time()
        elapsed = now - self._fps_ts
        if elapsed >= 1.0:
            self._fps = self._frame_count / elapsed if elapsed > 0 else 0.0
            self._frame_count = 0
            self._fps_ts = now

        with self._stats_lock:
            self._last_omega = omega
            self._last_v = v
            self._last_y_center = y_center
            self._last_w_center = w_center
            self._last_state = state

        # Build and cache debug frame
        debug_vis = self._build_debug_frame(roi, mask_y, mask_w, y_center, w_center, v, omega, state)
        debug_vis_up = cv2.resize(debug_vis, (320, 180), interpolation=cv2.INTER_NEAREST)
        ok, buf = cv2.imencode(".jpg", debug_vis_up, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if ok:
            jpeg_bytes = buf.tobytes()
            with self._debug_lock:
                self._debug_jpeg = jpeg_bytes

            # Also publish on ROS topic
            out_msg = CompressedImage()
            out_msg.header.stamp = rospy.Time.now()
            out_msg.format = "jpeg"
            out_msg.data = jpeg_bytes
            self.pub_debug.publish(out_msg)

    def on_shutdown(self):
        self.steer(0, 0)
        super(LaneFollowerNode, self).on_shutdown()


if __name__ == '__main__':
    node = LaneFollowerNode(node_name='lane_follower_node')
    rospy.spin()
