#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from std_msgs.msg import Float32, Bool
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import cv2
import torch
import math
import time
import queue
import threading


class DetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_depth_fusion')

        # Declare parameters
        self.declare_parameter("model_path", "/home/rokey/turtlebot4_ws/model/best.pt")
        self.declare_parameter("rgb_topic", "oakd/rgb/image_raw/compressed")
        self.declare_parameter("depth_topic", "oakd/stereo/image_raw")
        self.declare_parameter("cam_info_topic", "oakd/rgb/camera_info")

        model_path = self.get_parameter("model_path").value
        rgb_topic = self.get_parameter("rgb_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        cam_info_topic = self.get_parameter("cam_info_topic").value

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.bridge = CvBridge()

        # ===============================
        # YOLO MODEL
        # ===============================
        self.get_logger().info(f"Loading YOLO model: {model_path}")
        self.model = YOLO(model_path)
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(self.device)
        self.model.overrides["imgsz"] = 416
        self.model.overrides["half"] = (self.device == "cuda")
        self.get_logger().info(f"YOLO device = {self.device}")

        # =================================
        # FPS Limit Variables
        # =================================
        self.max_fps = 10.0       # Adjust this value to change YOLO inference FPS
        self.last_process_time = 0.0
        
        # Frame queue (to pass frames to the YOLO thread)
        self.rgb_queue = queue.Queue(maxsize=3)
        self.depth_image = None
        
        # Camera parameters
        self.camera_info = None
        self.fx = self.fy = self.cx = self.cy = None
        
        # Publishers
        self.x_pub = self.create_publisher(Float32, 'detected_x', 10)
        self.angle_pub = self.create_publisher(Float32, 'detected_angle', 10)
        self.dist_pub = self.create_publisher(Float32, 'detected_distance', 10)
        self.detected_pub = self.create_publisher(Bool, 'object_detected', 10)

        self.debug_image_pub = self.create_publisher(Image, 'yolo_annotated_img', 10)

        # Subscribers
        self.rgb_count = 0
        self.depth_count = 0
        self.create_subscription(CompressedImage, rgb_topic, self.rgb_callback, qos_profile)
        self.create_subscription(Image, depth_topic, self.depth_callback, qos_profile)
        self.create_subscription(CameraInfo, cam_info_topic, self.camera_info_callback, qos_profile)

        # ===============================
        # Start YOLO Thread
        # ===============================
        self.proc_thread = threading.Thread(target=self.process_thread_fn, daemon=True)
        self.proc_thread.start()
        self.get_logger().info("DetectionNode with FPS-limit initialized")

    # ========================================================
    # RGB Callback: Pass frames to YOLO thread
    # ========================================================
    def rgb_callback(self, msg: CompressedImage):
        self.rgb_count += 1
        if self.rgb_count % 30 == 0:
            self.get_logger().info(f"[DEBUG] RGB frames received: {self.rgb_count}")
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is not None:
            if not self.rgb_queue.full():
                self.rgb_queue.put(img)

    def depth_callback(self, msg: Image):
        self.depth_count += 1
        if self.depth_count % 30 == 0:
            self.get_logger().info(f"[DEBUG] Depth frames received: {self.depth_count}")
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    # ========================================================
    # YOLO Inference Thread (FPS control)
    # ========================================================
    def process_thread_fn(self):
        while rclpy.ok():
            # Apply FPS limit
            now = time.time()
            if now - self.last_process_time < (1.0 / self.max_fps):
                time.sleep(0.002)
                continue
            self.last_process_time = now
            
            # Fetch frame from queue
            try:
                frame = self.rgb_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            self.run_yolo(frame)

    # ========================================================
    # YOLO Inference + Depth fusion
    # ========================================================
    def run_yolo(self, rgb):
        if self.depth_image is None:
            return
        depth = self.depth_image
        
        # Adjust depth size to match RGB
        if depth.shape[:2] != rgb.shape[:2]:
            depth = cv2.resize(depth, (rgb.shape[1], rgb.shape[0]))
            
        # YOLO Inference
        results = self.model.predict(rgb, imgsz=416, conf=0.5, verbose=False)[0]
        detected_flag = False
        h, w = depth.shape[:2]
        
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cls = int(box.cls[0])
            label = self.model.names[cls]
            
            # Draw bounding boxes for all classes
            cv2.rectangle(rgb, (x1, y1), (x2, y2), (0,255,0), 2)
            
            # Draw center point
            cx_px = int((x1 + x2) / 2)
            cy_px = int((y1 + y2) / 2)
            cv2.circle(rgb, (cx_px, cy_px), 4, (0,0,255), -1)
            
            # Only publish distance/angle for 'redcar'
            if label == "redcar":
                detected_flag = True
                
                # Get depth value
                distance_m = self.get_depth_at(depth, cx_px, cy_px)
                if math.isnan(distance_m) or distance_m <= 0:
                    continue
                    
                # Calculate angle based on camera intrinsic parameters
                if self.fx:
                    X = distance_m * ((cx_px - self.cx) / self.fx)
                    Z = distance_m
                    angle_deg = math.degrees(math.atan2(X, Z))
                    x_offset = X
                else:
                    # Fallback FOV-based calculation
                    FOV = 70.0
                    img_center = w / 2
                    angle_deg = (cx_px - img_center) * (FOV / w)
                    x_offset = distance_m * math.sin(math.radians(angle_deg))
                
                # Publish data for 'redcar'
                self.x_pub.publish(Float32(data=x_offset))
                self.angle_pub.publish(Float32(data=angle_deg))
                self.dist_pub.publish(Float32(data=distance_m))
                
                # Display distance and angle for 'redcar'
                cv2.putText(rgb, f"{label} {distance_m:.2f}m {angle_deg:.1f}deg",
                            (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            else:
                # Display only the label for other classes (e.g., whitecar)
                cv2.putText(rgb, f"{label}",
                            (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

        # Publish detection status
        self.detected_pub.publish(Bool(data=detected_flag))
        try:
            img_msg = self.bridge.cv2_to_imgmsg(rgb, encoding='bgr8')
            self.debug_image_pub.publish(img_msg)

        except Exception as e:
            self.get_logger().warn(f"[DEBUG] Failed to publish debug image: {e}")

    # ========================================================
    # Read depth pixel value
    # ========================================================
    def get_depth_at(self, depth_img, cx, cy):
        h, w = depth_img.shape[:2]
        if cx < 1 or cy < 1 or cx >= w-1 or cy >= h-1:
            return float('nan')
            
        # Median filter on a 3x3 patch to handle noise
        patch = depth_img[cy-1:cy+2, cx-1:cx+2].astype(np.float32)
        patch = patch[patch > 0]
        if patch.size == 0:
            return float('nan')
        depth_mm = np.median(patch)
        return float(depth_mm) / 1000.0


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
