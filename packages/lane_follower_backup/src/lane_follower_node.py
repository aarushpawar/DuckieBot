#!/usr/bin/env python3

import os
import time
import threading
import rospy
import cv2
import numpy as np
import yaml
from duckietown.dtros import DTROS, NodeType, DTParam, ParamType
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import Twist2DStamped
from std_msgs.msg import Bool

try:
    from flask import Flask, Response, jsonify
    from flask_cors import CORS
    FLASK_AVAILABLE = True
except ImportError:
    FLASK_AVAILABLE = False

# Minimum pixels required to trust a color centroid
MIN_YELLOW_PX = 50
MIN_WHITE_PX  = 80

# Morphological kernel for noise removal
_KERNEL = np.ones((3, 3), np.uint8)


class LaneFollowerNode(DTROS):

    def __init__(self, node_name):
        super(LaneFollowerNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)

        self.veh = os.environ.get("VEHICLE_NAME", "duckiebot")

        self.v_base   = DTParam("~v_base",   param_type=ParamType.FLOAT, default=0.20)
        self.k_p      = DTParam("~k_p",      param_type=ParamType.FLOAT, default=3.0)
        self.k_repel  = DTParam("~k_repel",  param_type=ParamType.FLOAT, default=0.6)

        # HSV colour ranges
        self.yellow_low  = np.array([20, 100, 100])
        self.yellow_high = np.array([35, 255, 255])
        # Tightened: S>40 avoids grabbing white-ish overhead lights
        self.white_low   = np.array([0,  0,  180])
        self.white_high  = np.array([180, 40, 255])

        self.roi_top = 0.55
        self.roi_bot = 0.90

        # Runtime state
        self._lane_active  = True
        self._last_omega   = 0.0
        self._last_v       = 0.0
        self._last_y_ctr   = None
        self._last_w_ctr   = None
        self._last_state   = "INIT"
        self._frame_count  = 0
        self._fps          = 0.0
        self._fps_ts       = time.time()
        self._debug_jpeg   = None
        self._debug_lock   = threading.Lock()
        self._stats_lock   = threading.Lock()

        # Use same topic/type as the working shape_driver
        self.pub_cmd = rospy.Publisher(
            f"/{self.veh}/car_cmd_switch_node/cmd",
            Twist2DStamped,
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
            self.log("Flask not available — HTTP bridge disabled.")

        self.log("Lane Follower Node initialized.")

    # ------------------------------------------------------------------
    # Control
    # ------------------------------------------------------------------

    def publish_cmd(self, v, omega):
        msg = Twist2DStamped()
        msg.header.stamp = rospy.Time.now()
        msg.v = float(v)
        msg.omega = float(omega)
        self.pub_cmd.publish(msg)

    # ------------------------------------------------------------------
    # Vision helpers
    # ------------------------------------------------------------------

    def get_centroid(self, mask, min_px):
        """Return normalised x centroid [0,1] or None if fewer than min_px pixels."""
        count = cv2.countNonZero(mask)
        if count < min_px:
            return None
        pts = cv2.findNonZero(mask)
        return float(np.mean(pts, axis=0)[0][0]) / mask.shape[1]

    # ------------------------------------------------------------------
    # State logic
    # ------------------------------------------------------------------

    def compute_cmd(self, y_ctr, w_ctr):
        """
        Standard Duckietown right-lane geometry:
          - Yellow divider should sit LEFT of centre (~0.35 in camera frame)
          - White edge should be to the RIGHT (w_ctr > 0.5)

        Returns (v, omega, state_str).
        Positive omega = turn left (CCW).  Negative omega = turn right (CW).
        """
        v_base   = self.v_base.value
        k_p      = self.k_p.value
        k_repel  = self.k_repel.value

        if y_ctr is not None:
            # --- PRIMARY: follow yellow, keep it at ~35% from left ---
            error = 0.35 - y_ctr          # positive → yellow too far right → turn right
            omega = -k_p * error          # note sign: positive error → steer right (neg omega)

            # White repulsion: white on the LEFT means we've drifted into oncoming lane
            if w_ctr is not None:
                if w_ctr < 0.35:          # white dangerously on the left → turn right hard
                    omega -= k_repel
                elif w_ctr > 0.75:        # white on the far right, we're fine, slight left nudge
                    omega += k_repel * 0.3

            # Slow down proportionally to turn sharpness; never fully stop
            v = max(v_base * 0.4, v_base * (1.0 - abs(omega) * 0.15))
            return v, omega, "FOLLOW_YELLOW"

        if w_ctr is not None:
            # --- RECOVERY: only white visible ---
            if w_ctr < 0.5:
                # White is on the left → we've crossed the yellow line or are off-track left
                # Turn right to get back into right lane
                omega = -1.8
            else:
                # White is on the right where it belongs; lost yellow, crawl forward
                omega = -0.3              # slight right bias to find yellow
            v = v_base * 0.4
            return v, omega, "WHITE_RECOVERY"

        # --- LOST: no markings at all → slow right turn to search ---
        # For a 4-right-turns track this is almost always the right recovery
        return v_base * 0.3, -1.2, "LOST"

    # ------------------------------------------------------------------
    # Image callback
    # ------------------------------------------------------------------

    def cb_image(self, msg):
        if not self.switch:
            return

        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is None:
            return
        img = cv2.resize(img, (160, 120))
        h = 120

        y1, y2 = int(h * self.roi_top), int(h * self.roi_bot)
        roi = img[y1:y2, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Masks with morphological open to kill noise
        mask_y = cv2.morphologyEx(
            cv2.inRange(hsv, self.yellow_low, self.yellow_high), cv2.MORPH_OPEN, _KERNEL)
        mask_w = cv2.morphologyEx(
            cv2.inRange(hsv, self.white_low, self.white_high), cv2.MORPH_OPEN, _KERNEL)

        y_ctr = self.get_centroid(mask_y, MIN_YELLOW_PX)
        w_ctr = self.get_centroid(mask_w, MIN_WHITE_PX)

        v, omega, state = self.compute_cmd(y_ctr, w_ctr)

        if self._lane_active:
            self.publish_cmd(v, omega)

        # --- FPS tracking ---
        self._frame_count += 1
        now = time.time()
        elapsed = now - self._fps_ts
        if elapsed >= 1.0:
            self._fps = self._frame_count / elapsed
            self._frame_count = 0
            self._fps_ts = now

        with self._stats_lock:
            self._last_omega = omega
            self._last_v     = v
            self._last_y_ctr = y_ctr
            self._last_w_ctr = w_ctr
            self._last_state = state

        rospy.logdebug("%s v=%.2f omega=%.2f y=%.2f w=%s",
                       state, v, omega,
                       y_ctr if y_ctr is not None else -1,
                       f"{w_ctr:.2f}" if w_ctr is not None else "none")

        # --- Debug image ---
        debug_vis = self._build_debug_frame(roi, mask_y, mask_w, y_ctr, w_ctr, v, omega, state)
        debug_up  = cv2.resize(debug_vis, (320, 180), interpolation=cv2.INTER_NEAREST)
        ok, buf   = cv2.imencode(".jpg", debug_up, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
        if ok:
            jpeg = buf.tobytes()
            with self._debug_lock:
                self._debug_jpeg = jpeg
            out_msg = CompressedImage()
            out_msg.header.stamp = rospy.Time.now()
            out_msg.format = "jpeg"
            out_msg.data   = jpeg
            self.pub_debug.publish(out_msg)

    # ------------------------------------------------------------------
    # Debug visualisation
    # ------------------------------------------------------------------

    def _build_debug_frame(self, roi, mask_y, mask_w, y_ctr, w_ctr, v, omega, state):
        vis = roi.copy()
        h, w = vis.shape[:2]

        y_ov = np.zeros_like(vis); y_ov[mask_y > 0] = [0, 200, 255]
        vis  = cv2.addWeighted(vis, 1.0, y_ov, 0.5, 0)
        w_ov = np.zeros_like(vis); w_ov[mask_w > 0] = [255, 200, 100]
        vis  = cv2.addWeighted(vis, 1.0, w_ov, 0.4, 0)

        if y_ctr is not None:
            cx = int(y_ctr * w)
            cv2.line(vis, (cx, 0), (cx, h), (0, 220, 255), 2)
        if w_ctr is not None:
            cx = int(w_ctr * w)
            cv2.line(vis, (cx, 0), (cx, h), (255, 200, 80), 1)

        # Target line (35% from left)
        cv2.line(vis, (int(0.35 * w), 0), (int(0.35 * w), h), (0, 255, 0), 1)
        cv2.line(vis, (w // 2, 0), (w // 2, h), (60, 60, 60), 1)

        color = (0, 220, 80) if state == "FOLLOW_YELLOW" else \
                (80, 180, 255) if state == "WHITE_RECOVERY" else (60, 60, 200)
        cv2.putText(vis, state, (3, h - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1, cv2.LINE_AA)
        cv2.putText(vis, f"v={v:.2f} w={omega:.2f}", (3, 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (200, 200, 200), 1, cv2.LINE_AA)
        return vis

    # ------------------------------------------------------------------
    # Enable / disable
    # ------------------------------------------------------------------

    def cb_enable(self, msg):
        self._lane_active = msg.data
        self.log(f"Lane following {'ENABLED' if msg.data else 'DISABLED'} via ROS topic")

    # ------------------------------------------------------------------
    # Flask HTTP bridge
    # ------------------------------------------------------------------

    def _start_flask(self):
        app  = Flask(__name__)
        CORS(app)
        node = self

        @app.route("/status")
        def status():
            with node._stats_lock:
                return jsonify({
                    "active":   node._lane_active,
                    "veh":      node.veh,
                    "state":    node._last_state,
                    "omega":    round(node._last_omega, 4),
                    "v":        round(node._last_v, 4),
                    "y_center": round(node._last_y_ctr, 4) if node._last_y_ctr is not None else None,
                    "w_center": round(node._last_w_ctr, 4) if node._last_w_ctr is not None else None,
                    "fps":      round(node._fps, 1),
                    "params": {
                        "v_base":  round(node.v_base.value, 4),
                        "k_p":     round(node.k_p.value, 4),
                        "k_repel": round(node.k_repel.value, 4),
                    }
                })

        @app.route("/start", methods=["POST"])
        def start():
            node._lane_active = True
            return jsonify({"ok": True, "active": True})

        @app.route("/stop", methods=["POST"])
        def stop():
            node._lane_active = False
            node.publish_cmd(0, 0)
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
                        yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + data + b"\r\n"
                    time.sleep(0.05)
            return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")

        t = threading.Thread(
            target=lambda: app.run(host="0.0.0.0", port=8765, debug=False, use_reloader=False),
            daemon=True
        )
        t.start()
        self.log("Flask HTTP bridge running on :8765")

    def on_shutdown(self):
        self.publish_cmd(0, 0)
        super(LaneFollowerNode, self).on_shutdown()


if __name__ == '__main__':
    node = LaneFollowerNode(node_name='lane_follower_node')
    rospy.spin()
