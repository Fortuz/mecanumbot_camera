#!/usr/bin/env python3
import rclpy, cv2, numpy as np
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge

class BallTrackerRGB(Node):
    def __init__(self):
        super().__init__('ball_tracker_rgb')

        # --- Parameters (with more tunables) ---
        self.declare_parameters('', [
            ('image_topic', '/camera/image_raw'),
            ('camera_info_topic', '/camera/camera_info'),
            # Looser S/V defaults to avoid the "only lower half" issue under highlights
            ('hsv_lower', [25, 120, 120]),       # [H,S,V] OpenCV: H in [0,179]
            ('hsv_upper', [45, 255, 255]),
            ('min_area_px', 150),
            # New: optional circularity filter (0 disables). 0.75–0.85 is typical.
            ('min_circularity', 0.0),
            # New: morphology controls
            ('morph_kernel', 5),               # odd int; 0 or 1 disables closing
            ('morph_iters', 1),
            # New: median blur kernel (odd, 0/1 disables)
            ('median_ksize', 5),
            # New: use enclosing circle for bbox
            ('use_enclosing_circle', True),
            ('det_ball_topic', '/detections/ball'),
            ('debug_image_topic', '/camera/image_debug'),
            ('mask_image_topic', '/camera/image_mask'),
        ])

        vals = [p.value for p in self.get_parameters([
            'image_topic',
            'camera_info_topic',
            'hsv_lower',
            'hsv_upper',
            'min_area_px',
            'min_circularity',
            'morph_kernel',
            'morph_iters',
            'median_ksize',
            'use_enclosing_circle',
            'det_ball_topic',
            'debug_image_topic',
            'mask_image_topic',
        ])]

        (self.image_topic, info_topic, hsv_l, hsv_u, self.min_area,
         self.min_circularity, self.morph_kernel, self.morph_iters,
         self.median_ksize, self.use_enclosing_circle,
         det_topic, dbg_topic, mask_topic) = vals

        # Internal (numpy) copies of HSV thresholds
        self.hsv_lower = np.array(hsv_l, dtype=np.uint8)
        self.hsv_upper = np.array(hsv_u, dtype=np.uint8)

        self.bridge = CvBridge()
        self.create_subscription(CameraInfo, info_topic, self.on_info, 10)
        self.create_subscription(Image, self.image_topic, self.on_image, 10)
        self.pub_det  = self.create_publisher(Detection2DArray, det_topic, 10)
        self.pub_dbg  = self.create_publisher(Image, dbg_topic, 10)
        self.pub_mask = self.create_publisher(Image, mask_topic, 10)

        # React to live param changes (ros2 param set ...)
        self.add_on_set_parameters_callback(self._on_param_change)

        self.get_logger().info(
            f'Ball tracker: debug={dbg_topic} mask={mask_topic} '
            f'HSV lower={self.hsv_lower.tolist()} upper={self.hsv_upper.tolist()} '
            f'min_area={self.min_area}, min_circ={self.min_circularity}, '
            f'closing k={self.morph_kernel}x{self.morph_kernel} iters={self.morph_iters}, '
            f'median_ksize={self.median_ksize}, enclosing_circle={self.use_enclosing_circle}'
        )

    # --- Live parameter update handler ---
    def _on_param_change(self, params):
        try:
            for p in params:
                if p.name == 'hsv_lower' and p.type_ == Parameter.Type.INTEGER_ARRAY:
                    arr = np.array(p.value, dtype=np.uint8)
                    if arr.shape == (3,):
                        self.hsv_lower = arr
                elif p.name == 'hsv_upper' and p.type_ == Parameter.Type.INTEGER_ARRAY:
                    arr = np.array(p.value, dtype=np.uint8)
                    if arr.shape == (3,):
                        self.hsv_upper = arr
                elif p.name == 'min_area_px':
                    self.min_area = int(p.value)
                elif p.name == 'min_circularity':
                    self.min_circularity = float(p.value)
                elif p.name == 'morph_kernel':
                    self.morph_kernel = int(p.value)
                elif p.name == 'morph_iters':
                    self.morph_iters = int(p.value)
                elif p.name == 'median_ksize':
                    self.median_ksize = int(p.value)
                elif p.name == 'use_enclosing_circle':
                    self.use_enclosing_circle = bool(p.value)
            return SetParametersResult(successful=True)
        except Exception as e:
            self.get_logger().error(f'Param update error: {e}')
            return SetParametersResult(successful=False, reason=str(e))

    def on_info(self, msg: CameraInfo):
        pass  # reserved for intrinsics/TF usage later

    def on_image(self, img_msg: Image):
        # Ensure we convert correctly to BGR for OpenCV
        img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Threshold
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

        # Optional median blur
        if isinstance(self.median_ksize, int) and self.median_ksize >= 3 and self.median_ksize % 2 == 1:
            mask = cv2.medianBlur(mask, self.median_ksize)

        # Morphological closing to fill highlight gaps on top of the ball
        if isinstance(self.morph_kernel, int) and self.morph_kernel >= 3 and self.morph_kernel % 2 == 1:
            kernel = np.ones((self.morph_kernel, self.morph_kernel), np.uint8)
            iters = max(1, int(self.morph_iters))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=iters)

        # Publish binary mask (for Foxglove tuning)
        mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        mask_msg.header = img_msg.header
        self.pub_mask.publish(mask_msg)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        dets = Detection2DArray()
        dets.header = img_msg.header

        for c in contours:
            area = float(cv2.contourArea(c))
            if area < float(self.min_area):
                continue

            # Optional circularity filter (helps reject elongated blobs)
            if self.min_circularity and self.min_circularity > 0.0:
                perim = cv2.arcLength(c, True)
                if perim <= 1e-6:
                    continue
                circularity = 4.0 * np.pi * area / (perim * perim)
                if circularity < float(self.min_circularity):
                    continue

            # Bounding geometry: either rect or enclosing circle
            if self.use_enclosing_circle:
                (xc, yc), r = cv2.minEnclosingCircle(c)
                x = int(xc - r)
                y = int(yc - r)
                w = int(2 * r)
                h = int(2 * r)
                cx, cy = float(xc), float(yc)
            else:
                x, y, w, h = cv2.boundingRect(c)
                cx, cy = x + w / 2.0, y + h / 2.0

            # Build Detection2D message
            d = Detection2D()
            d.header = img_msg.header
            # NOTE: Some versions of vision_msgs use Pose2D for bbox.center (x, y, theta),
            # others have center.position.{x,y}. Keep your original accessors to avoid breaking your stack.
            try:
                # If center is Pose2D:
                d.bbox.center.x = float(cx)
                d.bbox.center.y = float(cy)
                d.bbox.center.theta = 0.0
            except AttributeError:
                # Fallback for center.position.{x,y}
                d.bbox.center.position.x = float(cx)
                d.bbox.center.position.y = float(cy)
                d.bbox.center.theta = 0.0

            d.bbox.size_x = float(w)
            d.bbox.size_y = float(h)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = "ball"
            hyp.hypothesis.score = 1.0
            d.results.append(hyp)
            dets.detections.append(d)

            # Debug drawing
            if self.use_enclosing_circle:
                cv2.circle(img, (int(cx), int(cy)), int(max(w, h) // 2), (0, 255, 0), 2)
            else:
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(img, (int(cx), int(cy)), 3, (0, 0, 255), -1)

        self.pub_det.publish(dets)

        dbg_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        dbg_msg.header = img_msg.header
        self.pub_dbg.publish(dbg_msg)

def main():
    rclpy.init()
    rclpy.spin(BallTrackerRGB())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
