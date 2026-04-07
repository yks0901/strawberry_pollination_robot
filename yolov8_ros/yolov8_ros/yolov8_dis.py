import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from bumblebee_interfaces.msg import ObjectData
import time
import math
from std_msgs.msg import Bool

class RealSenseYOLOv8Processor(Node):
    def __init__(self):
        """Initialize ROS2 node and YOLOv8 model."""
        super().__init__("realsense_yolov8_processor")

        # ROS2 Publisher
        self.data_pub = self.create_publisher(ObjectData, "/object_data", 10)
        self.mani_busy = False
        # Configuration parameters
        self.WIDTH = 1280
        self.HEIGHT = 720
        self.CONFIDENCE_THRESHOLD = 0.7
        self.MODEL_PATH = "/home/ubics/bumblebee_ws/src/yolov8_ros/weight/best0305.pt"

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Initialize image storage
        self.color_image = None
        self.depth_image = None
        self.depth_scale = 0.001

        # Initialize YOLOv8
        try:
            self.model = YOLO(self.MODEL_PATH).to("cuda")   #self.model = YOLO(self.MODEL_PATH)
            self.get_logger().info("YOLOv8 model loaded successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLOv8 model: {str(e)}")
            sys.exit(1)

        # Subscribe to camera topics
        self.setup_subscribers()

    def setup_subscribers(self):
        """Set up ROS2 subscribers for camera topics."""
        self.create_subscription(Image, "/camera/camera/color/image_raw", self.color_callback, 10)
        self.create_subscription(Image, "/camera/camera/aligned_depth_to_color/image_raw", self.depth_callback, 10)
        self.create_subscription(Bool,'/manipulator_busy', self.manipulator_busy_callback,10)
        self.get_logger().info("Camera subscribers initialized")

    def color_callback(self, msg):
        """Callback for color image."""
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting color image: {str(e)}")
    def manipulator_busy_callback(self, msg):
        self.mani_busy = msg.data

    def depth_callback(self, msg):
        """Callback for depth image."""
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            self.process_frame()
        except Exception as e:
            self.get_logger().error(f"Error converting depth image: {str(e)}")

    def get_3d_point(self, pixel_x: int, pixel_y: int, depth_image: np.ndarray):
        """Get 3D point from pixel coordinates with lens distortion correction."""
        
        try:
            if 0 <= pixel_x < self.WIDTH and 0 <= pixel_y < self.HEIGHT:
                depth = depth_image[int(round(pixel_y)), int(round(pixel_x))]

                if depth > 0:
                    w, h = self.WIDTH, self.HEIGHT


                      # Original Camera Intrinsics 내부 파라미터 2개 중에 하나
                    """
                    camera_matrix = np.array([[608.0451338, 0., 326.10849846],
                                            [0., 609.54897882, 242.85667966],
                                            [0., 0., 1.]])
                    # Distortion Coefficients
                    dist_coeffs = np.array([[0.03321855, 0.52401055, 0.00300431, -0.00243685, -1.97305159]])
                    """
                    """
                    camera_matrix = np.array([[603.15968582, 0.,325.55236225],
                                                  [0., 604.79247895, 243.50523507],
                                                  [0., 0., 1.]])

                    dist_coeffs = np.array([[0.118687110,  0.0180018882,  0.0027289081, -0.00013531318, -0.983756034]])
                    """    
                    """
                    camera_matrix = np.array([[905.5031651, 0.,646.75313387],
                                                  [0., 908.28201084, 364.21742182],
                                                  [0., 0., 1.]])

                    dist_coeffs = np.array([[0.17532407, -0.49929528, 0.00333925, -0.00159383, 0.37150572]])
                    """
                    
                    camera_matrix = np.array([[889.20439927, 0., 645.90808215],
                                              [0.,  891.14540673, 363.20254286],
                                              [0., 0., 1.]])
                    
                    dist_coeffs = np.array([[0.19324328, -0.6149969,   0.00296531, -0.00147052,  0.58687461]])
                    
                    # Optimal new camera matrix
                    new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))

                    # Undistort pixel coordinates
                    undistorted_pixel = cv2.undistortPoints(
                        np.array([[pixel_x, pixel_y]], dtype=np.float32),
                        camera_matrix,
                        dist_coeffs,
                        None,
                        new_camera_matrix
                    ).reshape(-1)

                    # Updated camera parameters after distortion correction
                    fx, fy = new_camera_matrix[0, 0], new_camera_matrix[1, 1]
                    cx, cy = new_camera_matrix[0, 2], new_camera_matrix[1, 2]

                    X = (undistorted_pixel[0] - cx) * depth / fx
                    Y = (undistorted_pixel[1] - cy) * depth / fy
                    Z = depth

                    return (X * self.depth_scale, Y * self.depth_scale, Z * self.depth_scale)

            return None

        except Exception as e:
            self.get_logger().warn(f"Error calculating 3D point: {str(e)}")
            return None

    

    def process_frame(self):
        """Process frame, detect objects, and estimate 3D coordinates."""
        try:
            if self.color_image is None or self.depth_image is None:
                return

            color_image = self.color_image.copy()
            results = self.model.predict(color_image, conf=self.CONFIDENCE_THRESHOLD)

            for result in results:
                if not hasattr(result, "obb") or result.obb is None:
                    continue

                for xywhr, conf, cls, points in zip(
                    result.obb.xywhr.cpu().numpy(),
                    result.obb.conf.cpu().numpy(),
                    result.obb.cls.cpu().numpy(),
                    result.obb.xyxyxyxy.cpu().numpy()):

                    #center_x, center_y = int(xywhr[0]), int(xywhr[1])
                    
                    points_reshaped = points.reshape(-1, 2)  # (8, 2)
                    center_x, center_y = np.mean(points_reshaped[:, 0]), np.mean(points_reshaped[:, 1])
                    point_3d = self.get_3d_point(center_x, center_y, self.depth_image)

                    if point_3d:
                        self.publish_data(self.model.names[int(cls)], point_3d)
                        self.visualize_detection(color_image, self.model.names[int(cls)], point_3d, points)

            cv2.imshow("YOLOv8 OBB with RealSense", color_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error processing frame: {str(e)}")

    def publish_data(self, name, point_3d):
        """Publish detected object data."""
        if self.mani_busy:
           return
        else: 
            data_msg = ObjectData()
            #data_msg.id = obj_id
            data_msg.name = name
            data_msg.x = point_3d[0]
            data_msg.y = point_3d[1]
            data_msg.z = point_3d[2]

            self.data_pub.publish(data_msg)

    def visualize_detection(self, color_image, name, point_3d, box_points):
        """Draw detection on image."""
        try:
            box = box_points.reshape(-1, 2).astype(int)
            cv2.polylines(color_image, [box], isClosed=True, color=(0, 255, 0), thickness=2)
            
            label_text = f"{name} ({point_3d[0]:.3f}, {point_3d[1]:.3f}, {point_3d[2]:.3f})m"
            cv2.putText(color_image, label_text, (box[0][0], box[0][1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        except Exception as e:
            self.get_logger().warn(f"Error visualizing detection: {str(e)}")

    def run(self):
        while rclpy.ok():
            time.sleep(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseYOLOv8Processor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":  
    main()
