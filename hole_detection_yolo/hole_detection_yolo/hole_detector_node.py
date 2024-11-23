import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
import message_filters
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException
import os

class HoleDetector(Node):
    def __init__(self):
        super().__init__('hole_detector')
        self.bridge = CvBridge()
        
        # Load model
        pkg_dir = get_package_share_directory('hole_detection_yolo')
        model_path = os.path.join(pkg_dir, 'models/best.pt')
        self.get_logger().info(f'Loading model from: {model_path}')
        self.model = YOLO(model_path)

        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.rgb_sub = message_filters.Subscriber(
            self, Image, '/wrist_rgbd_depth_sensor/image_raw')
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/wrist_rgbd_depth_sensor/depth/image_raw')
        
        # Synchronize RGB and depth images
        ts = message_filters.TimeSynchronizer([self.rgb_sub, self.depth_sub], 10)
        ts.registerCallback(self.image_callback)
        
        # Publisher for hole positions
        self.holes_pub = self.create_publisher(PoseArray, '/detected_holes', 10)
        
    def transform_pose(self, pose_stamped, from_frame, to_frame, timestamp):
        try:
            transform = self.tf_buffer.lookup_transform(
                to_frame,
                from_frame,
                timestamp,
                rclpy.duration.Duration(seconds=1.0))
        
            # Transform the pose
            transformed_pose = tf2_geometry_msgs.do_transform_pose_stamped(pose_stamped, transform)  
            return transformed_pose
    
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform pose: {ex}')
            return None  

    def image_callback(self, rgb_msg, depth_msg):
        cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
        results = self.model(cv_image)
        
        # Create PoseArray message
        pose_array = PoseArray()
        pose_array.header = rgb_msg.header
        
       # Draw detections and get 3D positions
        holes = []
        for result in results:
            boxes = result.boxes
            for box in boxes:
                # Get coordinates and confidence
                x1, y1, x2, y2 = box.xyxy[0]
                conf = float(box.conf[0])
            
                # Calculate center point
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
            
                # Get depth value and ensure it's float
                depth = float(depth_image[center_y, center_x])
                if np.isnan(depth):
                    depth = 0.0
            
                # Store hole info
                holes.append({
                    'center': (center_x, center_y),
                    'depth': depth,
                    'conf': conf
                })
            
                # Draw circle and center point
                radius = int(((x2 - x1) + (y2 - y1)) / 4)
                cv2.circle(cv_image, (center_x, center_y), radius, (0, 255, 0), 2)
                cv2.circle(cv_image, (center_x, center_y), 2, (0, 0, 255), -1)
                
        # Order holes
        if len(holes) == 4:
            sorted_holes = sorted(holes, key=lambda h: h['center'][1])
            top_holes = sorted(sorted_holes[:2], key=lambda h: h['center'][0])
            bottom_holes = sorted(sorted_holes[2:], key=lambda h: h['center'][0])
            ordered_holes = top_holes + bottom_holes
        
        for i, hole in enumerate(ordered_holes, 1):
            center_x, center_y = hole['center']
            cv2.putText(cv_image, str(i), (center_x-10, center_y-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            
            # Create PoseStamped in camera frame
            camera_pose = tf2_geometry_msgs.PoseStamped()
            camera_pose.header.frame_id = "wrist_rgbd_camera_depth_optical_frame"
            camera_pose.header.stamp = self.get_clock().now().to_msg()
            camera_pose.pose.position.x = float((center_x - cv_image.shape[1]/2) * 0.001)
            camera_pose.pose.position.y = float((center_y - cv_image.shape[0]/2) * 0.001)
            camera_pose.pose.position.z = float(hole['depth'] * 0.001)
            camera_pose.pose.orientation.w = 1.0
            
            transformed_pose = self.transform_pose(
                camera_pose,
                "wrist_rgbd_camera_depth_optical_frame",
                "base_link",
                self.get_clock().now().to_msg()
            )
            
            if transformed_pose:
                pose_array.poses.append(transformed_pose.pose)
                # Add position logging
                self.get_logger().info(
                    f"Hole {i}: "
                    f"x: {transformed_pose.pose.position.x:.4f}, "
                    f"y: {transformed_pose.pose.position.y:.4f}, "
                    f"z: {transformed_pose.pose.position.z:.4f}"
                )
        
        # Publish transformed poses
        self.holes_pub.publish(pose_array)
        
        # Display
        cv2.imshow("Hole Detection", cv_image)
        cv2.waitKey(1)

def main():
    rclpy.init()
    detector = HoleDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()