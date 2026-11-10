"""YOLO object detection on a ROS 2 image topic (tutorial 10).

Subscribes:  /camera/camera/color/image_raw   sensor_msgs/Image
Publishes:   /detections                      vision_msgs/Detection2DArray  (class name + score + box)
             /detections/image                sensor_msgs/Image             (image with boxes drawn)

The detector uses Ultralytics YOLO (https://docs.ultralytics.com, AGPL-3.0) on the CPU. To keep the CPU load
low it only processes the newest image, at most `max_rate` times per second.
"""

import os
import time

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class YoloDetector(Node):
    def __init__(self):
        super().__init__("yolo_detector")
        self.declare_parameter("model", "yolo11n.pt")  # downloaded on first use (~6 MB)
        self.declare_parameter("confidence", 0.35)
        self.declare_parameter("max_rate", 5.0)  # Hz
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.confidence = self.get_parameter("confidence").value
        model_name = self.get_parameter("model").value

        # Weights are cached outside the repository (they are not part of it - AGPL-3.0).
        cache = os.path.expanduser("~/.cache/lime_yolo")
        os.makedirs(cache, exist_ok=True)
        os.chdir(cache)
        from ultralytics import YOLO  # imported here: it is slow to import

        self.model = YOLO(model_name)
        self.get_logger().info(f"loaded {model_name}: {len(self.model.names)} classes")

        self.bridge = CvBridge()
        self.latest = None
        topic = self.get_parameter("image_topic").value
        self.create_subscription(Image, topic, self.on_image, 1)  # queue of 1: keep only the newest image
        self.det_pub = self.create_publisher(Detection2DArray, "/detections", 10)
        self.img_pub = self.create_publisher(Image, "/detections/image", 1)
        # Wall-clock timer: detection speed is limited by the CPU, not by simulation time.
        self.create_timer(1.0 / self.get_parameter("max_rate").value, self.detect, clock=rclpy.clock.Clock())
        self.last_log = 0.0
        self.get_logger().info(f"waiting for images on {topic}")

    def on_image(self, msg: Image):
        self.latest = msg

    def detect(self):
        msg, self.latest = self.latest, None
        if msg is None:
            return
        rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        result = self.model.predict(rgb[:, :, ::-1], conf=self.confidence, verbose=False)[0]  # YOLO wants BGR

        out = Detection2DArray(header=msg.header)
        for box in result.boxes:
            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
            det = Detection2D(header=msg.header)
            det.bbox.center.position.x = (x1 + x2) / 2.0
            det.bbox.center.position.y = (y1 + y2) / 2.0
            det.bbox.size_x = x2 - x1
            det.bbox.size_y = y2 - y1
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = self.model.names[int(box.cls)]
            hyp.hypothesis.score = float(box.conf)
            det.results.append(hyp)
            out.detections.append(det)
        self.det_pub.publish(out)

        annotated = result.plot()[:, :, ::-1]  # BGR -> RGB
        img = self.bridge.cv2_to_imgmsg(annotated.copy(), encoding="rgb8")
        img.header = msg.header
        self.img_pub.publish(img)

        if time.time() - self.last_log > 2.0:
            found = ", ".join(f"{d.results[0].hypothesis.class_id} {d.results[0].hypothesis.score:.2f}" for d in out.detections)
            self.get_logger().info(f"{len(out.detections)} objects: {found or '-'}")
            self.last_log = time.time()


def main():
    rclpy.init()
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
