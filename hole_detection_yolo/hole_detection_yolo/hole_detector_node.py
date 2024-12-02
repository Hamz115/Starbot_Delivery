import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, Point, TransformStamped
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
import message_filters
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException, TransformBroadcaster, StaticTransformBroadcaster
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

        # Camera parameters
        self.camera_params = {
            'fx': 520.7813804684724,
            'fy': 520.7813804684724,
            'cx': 320.5,
            'cy': 240.5,
            'hole_depth': 0.040  # meters
        }

        # Create camera matrix for reprojection
        self.camera_matrix = np.array([
            [self.camera_params['fx'], 0, self.camera_params['cx']],
            [0, self.camera_params['fy'], self.camera_params['cy']],
            [0, 0, 1]
        ])
        self.dist_coeffs = np.zeros((4,1))

        # System state
        self.detection_data = {
            'found': False,
            'positions': [],
            'last_depth': None,
            'world_coords': {'x': 0.0, 'y': 0.0, 'z': 0.0}
        }

        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.broadcaster = StaticTransformBroadcaster(self)
        
        # Publishers
        self.image_pub = self.create_publisher(Image, '/hole_detection/image', 10)
        self.holes_pub = self.create_publisher(PoseArray, '/detected_holes', 10)
        self.rel_pos_pub = self.create_publisher(Point, '/detect_holes', 10)
        self.abs_pos_pub = self.create_publisher(Point, '/holes_detected', 10)
        
        # Subscribers with synchronization
        self.rgb_sub = message_filters.Subscriber(
            self, Image, '/wrist_rgbd_depth_sensor/image_raw')
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/wrist_rgbd_depth_sensor/depth/image_raw')
        ts = message_filters.TimeSynchronizer([self.rgb_sub, self.depth_sub], 10)
        ts.registerCallback(self.image_callback)

        # Processing timer
        self.create_timer(0.5, self._processing_callback)

    def _adjust_for_hole_depth(self, x, y, z):
        """Adjust coordinates for hole depth"""
        current_mag = np.sqrt(x*x + y*y + z*z)
        desired_z = z - self.camera_params['hole_depth']
        new_mag = current_mag * (desired_z / z)
        norm_vector = np.array([x, y, z]) / current_mag
        adjusted_position = norm_vector * new_mag
        return tuple(adjusted_position)

    def _update_transform(self, x, y, z, hole_number):
        """Update transform for hole position"""
        try:
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'wrist_rgbd_camera_depth_frame'
            transform.child_frame_id = f'HOLE_{hole_number}'
            
            # Set translation
            transform.transform.translation.x = z
            transform.transform.translation.y = -x
            transform.transform.translation.z = -y
            
            # Set rotation 
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = -0.5
            transform.transform.rotation.z = 0.0
            transform.transform.rotation.w = 0.8660254037844387
            
            self.broadcaster.sendTransform([transform])
            
        except Exception as e:
            self.get_logger().error(f'Error broadcasting transform: {e}')

    def _processing_callback(self):
        """Regular processing callback for transform updates"""
        if not self.detection_data['found']:
            return

        try:
            for i in range(1, 5):  # For each hole
                # Check transform availability
                if not self.tf_buffer.can_transform('world', f'HOLE_{i}', rclpy.time.Time(), 
                                                timeout=rclpy.duration.Duration(seconds=0.1)):
                    continue

                # Look up the transform
                transform = self.tf_buffer.lookup_transform(
                    'world',
                    f'HOLE_{i}',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )

                # Publish absolute position
                msg = Point(
                    x=float(transform.transform.translation.x),
                    y=float(transform.transform.translation.y),
                    z=float(transform.transform.translation.z)
                )
                self.abs_pos_pub.publish(msg)
                
                self.get_logger().info(
                    f'Absolute position HOLE_{i}: ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f})'
                )
                
        except Exception as e:
            self.get_logger().debug(f'Transform processing: {str(e)}')

    def image_callback(self, rgb_msg, depth_msg):
        """Process incoming RGB and depth images"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
            display_image = cv_image.copy()
            
            results = self.model(cv_image, verbose=False)
            
            pose_array = PoseArray()
            pose_array.header = rgb_msg.header
            pose_array.header.frame_id = 'wrist_rgbd_camera_depth_frame'
            
            holes = []
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0]
                    conf = float(box.conf[0])
                    
                    if conf < 0.25:  # Skip low confidence detections
                        continue
                        
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)
                    
                    depth = float(depth_image[center_y, center_x])
                    if np.isnan(depth):
                        continue
                    
                    holes.append({
                        'center': (center_x, center_y),
                        'depth': depth,
                        'bbox': (int(x1), int(y1), int(x2), int(y2))
                    })
            
            if len(holes) == 4:
                # Sort holes
                sorted_holes = sorted(holes, key=lambda h: h['center'][1])
                top_holes = sorted(sorted_holes[:2], key=lambda h: h['center'][0])
                bottom_holes = sorted(sorted_holes[2:], key=lambda h: h['center'][0])
                ordered_holes = top_holes + bottom_holes
                
                self.detection_data['found'] = True
                self.detection_data['positions'] = []
                
                for i, hole in enumerate(ordered_holes, 1):
                    center_x, center_y = hole['center']
                    x1, y1, x2, y2 = hole['bbox']
                    depth = hole['depth']
                    
                    # Store detection
                    self.detection_data['positions'].append((center_x, center_y))
                    
                    # Calculate 3D position using camera matrix
                    image_point = np.array([[center_x, center_y]], dtype=np.float32)

                    undistorted_point = cv2.undistortPoints(image_point, self.camera_matrix, self.dist_coeffs)

                    homogeneous_point = cv2.convertPointsToHomogeneous(undistorted_point)

                    point_3d = homogeneous_point[0][0] * depth
                    
                    # Adjust for hole depth
                    adj_x, adj_y, adj_z = self._adjust_for_hole_depth(
                        float(point_3d[0]), float(point_3d[1]), float(point_3d[2])
                    )
                    
                    # Update transform for this hole
                    self._update_transform(adj_x, adj_y, adj_z, i)
                    
                    # Publish relative position
                    rel_point = Point(x=float(adj_x), y=float(adj_y), z=float(adj_z))
                    self.rel_pos_pub.publish(rel_point)
                    
                    # Draw visualization
                    radius = int(((x2 - x1) + (y2 - y1)) / 4)
                    cv2.circle(display_image, (center_x, center_y), radius, (0, 0, 255), 2)
                    cv2.circle(display_image, (center_x, center_y), 3, (255, 0, 0), -1)
                    
                    # Add numbered labels
                    text = str(i)
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    scale = 1.0
                    thickness = 2
                    cv2.putText(display_image, text, 
                              (center_x-10, center_y-10),
                              font, scale, (255, 255, 255), thickness+1)
                    cv2.putText(display_image, text,
                              (center_x-10, center_y-10),
                              font, scale, (255, 0, 0), thickness)
            else:
                self.detection_data['found'] = False
                self.detection_data['positions'] = []
            
            # Publish visualization
            viz_msg = self.bridge.cv2_to_imgmsg(display_image, "bgr8")
            viz_msg.header = rgb_msg.header
            self.image_pub.publish(viz_msg)
            
            # Display
            cv2.imshow("Hole Detection", display_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error in image callback: {str(e)}')
            self.detection_data['found'] = False
            self.detection_data['positions'] = []

def main():
    rclpy.init()
    try:
        detector = HoleDetector()
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

    