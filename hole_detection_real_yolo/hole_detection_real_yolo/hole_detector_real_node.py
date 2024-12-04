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

class HoleDetectorReal(Node):
    def __init__(self):
        super().__init__('hole_detector_real')
        self.bridge = CvBridge()
        
        # Load model
        pkg_dir = get_package_share_directory('hole_detection_real_yolo')
        model_path = os.path.join(pkg_dir, 'models/best.pt')
        self.get_logger().info(f'Loading model from: {model_path}')
        self.model = YOLO(model_path)

        # Camera intrinsics for D415
        self.camera_params = {
            'fx': 306.80584716796875,  # focal length in x
            'fy': 306.6424560546875,   # focal length in y
            'cx': 214.4418487548828,   # optical center x
            'cy': 124.9103012084961,   # optical center y
            'hole_depth': 0.04,        # meters
            'safety_offset': 0.0       # Additional safety offset in meters
        }

        # Parameters for depth processing
        self.depth_params = {
            'min_depth': 0.1,    # meters
            'max_depth': 2.0,    # meters
            'window_size': 5,    # pixels
            'min_valid_pixels': 10  # minimum number of valid depth pixels
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
        self.depth_debug_pub = self.create_publisher(Image, '/hole_detection/depth_debug', 10)
        self.holes_pub = self.create_publisher(PoseArray, '/detected_holes', 10)
        self.rel_pos_pub = self.create_publisher(Point, '/detect_holes', 10)
        self.abs_pos_pub = self.create_publisher(Point, '/holes_detected', 10)
        
        # Subscribers with synchronization
        self.rgb_sub = message_filters.Subscriber(
            self, Image, '/D415/color/image_raw')
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/D415/aligned_depth_to_color/image_raw')
        ts = message_filters.TimeSynchronizer([self.rgb_sub, self.depth_sub], 10)
        ts.registerCallback(self.image_callback)

        # Camera info subscriber
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/D415/aligned_depth_to_color/camera_info',
            self.camera_info_callback,
            10
        )

        # Processing timer
        self.create_timer(0.5, self._processing_callback)

    def camera_info_callback(self, msg):
        """Update camera parameters from camera info message"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        self.camera_params['fx'] = self.camera_matrix[0, 0]
        self.camera_params['fy'] = self.camera_matrix[1, 1]
        self.camera_params['cx'] = self.camera_matrix[0, 2]
        self.camera_params['cy'] = self.camera_matrix[1, 2]

    def get_filtered_depth(self, depth_image, center_x, center_y):
        """Get filtered depth value from a window around the center point"""
        window_size = self.depth_params['window_size']
        y_start = max(0, center_y - window_size)
        y_end = min(depth_image.shape[0], center_y + window_size + 1)
        x_start = max(0, center_x - window_size)
        x_end = min(depth_image.shape[1], center_x + window_size + 1)
        
        # Extract depth window
        depth_window = depth_image[y_start:y_end, x_start:x_end]
        
        # Convert to meters and filter invalid values
        depth_window = depth_window.astype(np.float32) / 1000.0  # 16UC1 is in mm
        valid_depths = depth_window[(depth_window > self.depth_params['min_depth']) & 
                                  (depth_window < self.depth_params['max_depth'])]
        
        if len(valid_depths) < self.depth_params['min_valid_pixels']:
            return None
            
        # Use median for robustness
        return np.median(valid_depths)

    def calculate_3d_point(self, image_point, depth):
        """Calculate 3D point from image coordinates and depth using OpenCV"""
        # Convert to format expected by OpenCV
        image_point = np.array([[image_point[0], image_point[1]]], dtype=np.float32)
        
        # Undistort the point
        undistorted_point = cv2.undistortPoints(image_point, self.camera_matrix, self.dist_coeffs)
        
        # Convert to homogeneous coordinates
        homogeneous_point = cv2.convertPointsToHomogeneous(undistorted_point)
        
        # Scale by depth to get 3D point
        point_3d = homogeneous_point[0][0] * depth
        
        # Validate the calculated point
        if not self.validate_3d_point(point_3d, depth):
            return None
            
        return point_3d

    def validate_3d_point(self, point_3d, depth):
        """Validate 3D point calculation"""
        # Calculate expected magnitude
        magnitude = np.sqrt(np.sum(point_3d**2))
        
        # Check if magnitude is close to measured depth
        depth_error = abs(magnitude - depth)
        if depth_error > 0.1:  # 10cm threshold
            self.get_logger().warn(f'Depth calculation mismatch - measured: {depth:.3f}m, calculated: {magnitude:.3f}m')
            return False
            
        return True

    def _adjust_for_hole_depth(self, x, y, z):
        """Adjust coordinates for hole depth with safety offset"""
        current_mag = np.sqrt(x*x + y*y + z*z)
        # Add safety offset to hole depth
        total_offset = self.camera_params['hole_depth'] + self.camera_params['safety_offset']
        desired_z = z - total_offset
        new_mag = current_mag * (desired_z / z)
        norm_vector = np.array([x, y, z]) / current_mag
        adjusted_position = norm_vector * new_mag
        return tuple(adjusted_position)

    def _update_transform(self, x, y, z, hole_number):
        """Update transform for hole position"""
        try:
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'D415_color_frame'
            transform.child_frame_id = f'HOLE_{hole_number}'
            
            # Set translation (applying coordinate frame conversion)
            transform.transform.translation.x = z  # Forward
            transform.transform.translation.y = -x # Left
            transform.transform.translation.z = -y # Up
            
            # Set rotation (standard orientation)
            transform.transform.rotation.x = 0.0018529052660899727
            transform.transform.rotation.y = -0.5634106983633584
            transform.transform.rotation.z = 0.0012635872076802214
            transform.transform.rotation.w = 0.8261769695227001
            
            self.broadcaster.sendTransform([transform])
            
        except Exception as e:
            self.get_logger().error(f'Error broadcasting transform: {e}')

    def _processing_callback(self):
        """Regular processing callback for transform updates"""
        if not self.detection_data['found']:
            return

        try:
            for i in range(1, 5):  # For each hole
                if not self.tf_buffer.can_transform('world', f'HOLE_{i}', rclpy.time.Time(), 
                                                timeout=rclpy.duration.Duration(seconds=0.1)):
                    continue

                transform = self.tf_buffer.lookup_transform(
                    'world',
                    f'HOLE_{i}',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )

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
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="16UC1")
            display_image = cv_image.copy()
            
            # Create depth visualization
            depth_debug = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            depth_debug = cv2.applyColorMap(depth_debug, cv2.COLORMAP_JET)
            
            results = self.model(cv_image, verbose=False)
            
            pose_array = PoseArray()
            pose_array.header = rgb_msg.header
            pose_array.header.frame_id = 'D415_color_frame'
            
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
                    
                    # Get filtered depth
                    depth = self.get_filtered_depth(depth_image, center_x, center_y)
                    
                    if depth is None:
                        self.get_logger().warn(f'Invalid depth for hole at ({center_x}, {center_y})')
                        # Draw red circle for invalid depth
                        cv2.circle(depth_debug, (center_x, center_y), 5, (0, 0, 255), -1)
                        continue
                    
                    # Calculate 3D point
                    point_3d = self.calculate_3d_point((center_x, center_y), depth)
                    if point_3d is None:
                        continue
                        
                    self.get_logger().info(f'Hole depth: {depth:.3f}m at ({center_x}, {center_y})')
                    holes.append({
                        'center': (center_x, center_y),
                        'depth': depth,
                        'bbox': (int(x1), int(y1), int(x2), int(y2)),
                        'point_3d': point_3d
                    })
                    
                    # Draw green circle for valid depth
                    cv2.circle(depth_debug, (center_x, center_y), 5, (0, 255, 0), -1)
            
            # Publish depth debug visualization
            depth_debug_msg = self.bridge.cv2_to_imgmsg(depth_debug, "bgr8")
            depth_debug_msg.header = depth_msg.header
            self.depth_debug_pub.publish(depth_debug_msg)
            
            if len(holes) == 4:
                # Improved hole sorting based on both x and y coordinates
                # First sort by y to separate top and bottom pairs
                sorted_holes = sorted(holes, key=lambda h: h['center'][1])
                
                # Get top and bottom pairs
                top_pair = sorted_holes[:2]
                bottom_pair = sorted_holes[2:]
                
                # Sort each pair by x coordinate
                top_holes = sorted(top_pair, key=lambda h: h['center'][0])
                bottom_holes = sorted(bottom_pair, key=lambda h: h['center'][0])
                
                # Combine into final ordered list
                ordered_holes = top_holes + bottom_holes
                
                self.detection_data['found'] = True
                self.detection_data['positions'] = []
                
                pose_array = PoseArray()
                pose_array.header.stamp = self.get_clock().now().to_msg()
                pose_array.header.frame_id = 'D415_color_frame'
                
                for i, hole in enumerate(ordered_holes, 1):
                    center_x, center_y = hole['center']
                    x1, y1, x2, y2 = hole['bbox']
                    point_3d = hole['point_3d']
                    depth = hole['depth']
                    
                    # Store detection
                    self.detection_data['positions'].append((center_x, center_y))
                    
                    # Add debug text
                    debug_text = f"#{i} D:{depth:.3f}m"
                    
                    # Adjust for hole depth
                    adj_x, adj_y, adj_z = self._adjust_for_hole_depth(
                        float(point_3d[0]), float(point_3d[1]), float(point_3d[2])
                    )
                    
                    # Create and append pose
                    pose = Pose()
                    pose.position.x = adj_x
                    pose.position.y = adj_y
                    pose.position.z = adj_z
                    pose_array.poses.append(pose)
                    
                    # Update transform
                    self._update_transform(adj_x, adj_y, adj_z, i)
                    
                    # Publish relative position
                    rel_point = Point(x=float(adj_x), y=float(adj_y), z=float(adj_z))
                    self.rel_pos_pub.publish(rel_point)
                    
                    # Visualization
                    radius = int(((x2 - x1) + (y2 - y1)) / 4)
                    # Draw hole circle
                    cv2.circle(display_image, (center_x, center_y), radius, (0, 0, 255), 2)
                    # Draw center point
                    cv2.circle(display_image, (center_x, center_y), 3, (255, 0, 0), -1)
                    
                    # Draw text with background for better visibility
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    scale = 0.7
                    thickness = 2
                    debug_text = f"{i}"  # Simplified text to just show hole number
                    text_size = cv2.getTextSize(debug_text, font, scale, thickness)[0]
                    
                    # Calculate position above the hole
                    text_x = center_x - text_size[0]//2
                    text_y = center_y - radius - 10  # Position above the hole
                    
                    # Text background
                    cv2.rectangle(display_image,
                                (text_x, text_y - text_size[1] - 5),
                                (text_x + text_size[0], text_y + 5),
                                (255, 255, 255),
                                -1)
                    
                    # Draw text
                    cv2.putText(display_image, debug_text,
                              (text_x, text_y),
                              font, scale, (0, 0, 255), thickness)
                
                # Publish the pose array
                self.holes_pub.publish(pose_array)

            else:
                self.detection_data['found'] = False
                self.detection_data['positions'] = []
            
            # Publish visualization
            viz_msg = self.bridge.cv2_to_imgmsg(display_image, "bgr8")
            viz_msg.header = rgb_msg.header
            self.image_pub.publish(viz_msg)
            
            # Display
            cv2.imshow("Hole Detection", display_image)
            cv2.imshow("Depth View", depth_debug)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error in image callback: {str(e)}')
            self.detection_data['found'] = False
            self.detection_data['positions'] = []

def main():
    rclpy.init()
    try:
        detector = HoleDetectorReal()
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()