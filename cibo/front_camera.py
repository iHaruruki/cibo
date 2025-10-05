#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np

class FrontCameraNode(Node):
    def __init__(self):
        super().__init__('front_camera')
        
        # Parameters
        self.declare_parameter('roi_x', 0)
        self.declare_parameter('roi_y', 0)
        self.declare_parameter('roi_width', 640)
        self.declare_parameter('roi_height', 480)
        self.declare_parameter('enable_roi', True)
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Initialize MediaPipe
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_holistic = mp.solutions.holistic
        self.mp_face_mesh = mp.solutions.face_mesh
        self.mp_drawing_styles = mp.solutions.drawing_styles
        
        # Initialize models with refined landmarks for detailed face mesh
        self.holistic = self.mp_holistic.Holistic(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        
        # Face mesh with refined landmarks (468 points)
        self.face_mesh = self.mp_face_mesh.FaceMesh(
            max_num_faces=1,
            refine_landmarks=True,  # This enables 468 detailed landmarks
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        
        # ROI selection state
        self.roi_selecting = False
        self.roi_start_point = None
        self.roi_end_point = None
        self.roi_params = {
            'x': self.get_parameter('roi_x').value,
            'y': self.get_parameter('roi_y').value,
            'width': self.get_parameter('roi_width').value,
            'height': self.get_parameter('roi_height').value,
            'enabled': self.get_parameter('enable_roi').value
        }
        
        # OpenCV window setup
        self.setup_opencv_window()
        
        # Subscriber
        self.image_sub = self.create_subscription(
            Image, '/camera_01/color/image_raw', self.image_callback, 10)
        
        # Publishers
        self.annotated_pub = self.create_publisher(
            Image, '/front_camera/annotated_image', 10)
        self.pose_landmarks_pub = self.create_publisher(
            Float32MultiArray, '/front_camera/pose_landmarks', 10)
        self.face_landmarks_pub = self.create_publisher(
            Float32MultiArray, '/front_camera/face_landmarks', 10)
        self.left_hand_landmarks_pub = self.create_publisher(
            Float32MultiArray, '/front_camera/left_hand_landmarks', 10)
        self.right_hand_landmarks_pub = self.create_publisher(
            Float32MultiArray, '/front_camera/right_hand_landmarks', 10)
        
        self.get_logger().info('Front Camera Node initialized')

    def setup_opencv_window(self):
        """OpenCV window setup for ROI selection"""
        try:
            cv2.namedWindow('Front Camera - ROI Selection', cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback('Front Camera - ROI Selection', self.mouse_callback)
            self.get_logger().info('OpenCV window setup for ROI selection')
        except Exception as e:
            self.get_logger().error(f'Failed to setup OpenCV window: {str(e)}')

    def mouse_callback(self, event, x, y, flags, param):
        """Mouse callback for ROI selection"""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.roi_selecting = True
            self.roi_start_point = (x, y)
            self.get_logger().info(f'ROI start point: ({x}, {y})')
            
        elif event == cv2.EVENT_LBUTTONUP and self.roi_selecting:
            self.roi_selecting = False
            self.roi_end_point = (x, y)
            
            # Calculate ROI parameters
            x1, y1 = self.roi_start_point
            x2, y2 = self.roi_end_point
            
            roi_x = min(x1, x2)
            roi_y = min(y1, y2)
            roi_width = abs(x2 - x1)
            roi_height = abs(y2 - y1)
            
            self.roi_params = {
                'x': roi_x,
                'y': roi_y,
                'width': roi_width,
                'height': roi_height,
                'enabled': True
            }
            
            # Update ROS parameters
            self.set_parameters([
                rclpy.parameter.Parameter('roi_x', rclpy.Parameter.Type.INTEGER, roi_x),
                rclpy.parameter.Parameter('roi_y', rclpy.Parameter.Type.INTEGER, roi_y),
                rclpy.parameter.Parameter('roi_width', rclpy.Parameter.Type.INTEGER, roi_width),
                rclpy.parameter.Parameter('roi_height', rclpy.Parameter.Type.INTEGER, roi_height),
                rclpy.parameter.Parameter('enable_roi', rclpy.Parameter.Type.BOOL, True)
            ])
            
            self.get_logger().info(f'ROI set: x={roi_x}, y={roi_y}, width={roi_width}, height={roi_height}')

    def image_callback(self, msg):
        """Image callback for processing"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            height, width = cv_image.shape[:2]
            
            # Apply ROI if enabled
            if self.roi_params['enabled']:
                x = max(0, self.roi_params['x'])
                y = max(0, self.roi_params['y'])
                w = min(self.roi_params['width'], width - x)
                h = min(self.roi_params['height'], height - y)
                
                if w > 0 and h > 0:
                    roi_image = cv_image[y:y+h, x:x+w]
                    processing_image = roi_image
                    offset_x, offset_y = x, y
                else:
                    processing_image = cv_image
                    offset_x, offset_y = 0, 0
            else:
                processing_image = cv_image
                offset_x, offset_y = 0, 0
            
            # MediaPipe processing
            image_rgb = cv2.cvtColor(processing_image, cv2.COLOR_BGR2RGB)
            image_rgb.flags.writeable = False
            
            # Process with both Holistic and Face Mesh
            holistic_results = self.holistic.process(image_rgb)
            face_results = self.face_mesh.process(image_rgb)
            
            image_rgb.flags.writeable = True
            annotated_image = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
            
            # Draw holistic landmarks
            if holistic_results.pose_landmarks:
                self.mp_drawing.draw_landmarks(
                    annotated_image, holistic_results.pose_landmarks, self.mp_holistic.POSE_CONNECTIONS,
                    landmark_drawing_spec=self.mp_drawing_styles.get_default_pose_landmarks_style())
            
            if holistic_results.left_hand_landmarks:
                self.mp_drawing.draw_landmarks(
                    annotated_image, holistic_results.left_hand_landmarks, self.mp_holistic.HAND_CONNECTIONS)
            
            if holistic_results.right_hand_landmarks:
                self.mp_drawing.draw_landmarks(
                    annotated_image, holistic_results.right_hand_landmarks, self.mp_holistic.HAND_CONNECTIONS)
            
            # Draw detailed face mesh (468 points)
            if face_results.multi_face_landmarks:
                for face_landmarks in face_results.multi_face_landmarks:
                    # Draw face mesh connections
                    self.mp_drawing.draw_landmarks(
                        annotated_image, face_landmarks, self.mp_face_mesh.FACEMESH_CONTOURS,
                        landmark_drawing_spec=None,
                        connection_drawing_spec=self.mp_drawing_styles.get_default_face_mesh_contours_style())
                    
                    # Draw tesselation for more detailed view
                    self.mp_drawing.draw_landmarks(
                        annotated_image, face_landmarks, self.mp_face_mesh.FACEMESH_TESSELATION,
                        landmark_drawing_spec=None,
                        connection_drawing_spec=mp.solutions.drawing_styles.DrawingSpec(
                            color=(80, 110, 10), thickness=1, circle_radius=1))
            
            # Create full-size annotated image
            full_annotated = cv_image.copy()
            if self.roi_params['enabled'] and w > 0 and h > 0:
                full_annotated[y:y+h, x:x+w] = annotated_image
            else:
                full_annotated = annotated_image
            
            # Display image with ROI rectangle
            display_image = full_annotated.copy()
            if self.roi_params['enabled']:
                cv2.rectangle(display_image, 
                            (self.roi_params['x'], self.roi_params['y']),
                            (self.roi_params['x'] + self.roi_params['width'],
                             self.roi_params['y'] + self.roi_params['height']),
                            (0, 255, 0), 2)
                cv2.putText(display_image, 'ROI', 
                           (self.roi_params['x'], self.roi_params['y'] - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            cv2.putText(display_image, 'Drag to select ROI, Press Q to quit, R to reset ROI',
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Display landmark count info
            face_count = 0
            if face_results.multi_face_landmarks:
                for face_landmarks in face_results.multi_face_landmarks:
                    face_count = len(face_landmarks.landmark)
            
            cv2.putText(display_image, f'Face landmarks: {face_count}', 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            cv2.imshow('Front Camera - ROI Selection', display_image)
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                cv2.destroyAllWindows()
            elif key == ord('r'):
                self.roi_params['enabled'] = False
                self.set_parameters([
                    rclpy.parameter.Parameter('enable_roi', rclpy.Parameter.Type.BOOL, False)
                ])
                self.get_logger().info('ROI reset')
            
            # Publish annotated image
            annotated_msg = self.bridge.cv2_to_imgmsg(full_annotated, "bgr8")
            annotated_msg.header = msg.header
            self.annotated_pub.publish(annotated_msg)
            
            # Extract and publish landmarks
            proc_height, proc_width = processing_image.shape[:2]
            
            # Pose landmarks
            pose_landmarks = []
            if holistic_results.pose_landmarks:
                for landmark in holistic_results.pose_landmarks.landmark:
                    x_coord = (landmark.x * proc_width) + offset_x
                    y_coord = (landmark.y * proc_height) + offset_y
                    pose_landmarks.extend([x_coord, y_coord, landmark.z])
            
            pose_msg = Float32MultiArray()
            pose_msg.data = pose_landmarks
            self.pose_landmarks_pub.publish(pose_msg)
            
            # Face landmarks (468 detailed points from Face Mesh)
            face_landmarks = []
            if face_results.multi_face_landmarks:
                for face_landmarks_data in face_results.multi_face_landmarks:
                    for landmark in face_landmarks_data.landmark:
                        x_coord = (landmark.x * proc_width) + offset_x
                        y_coord = (landmark.y * proc_height) + offset_y
                        face_landmarks.extend([x_coord, y_coord, landmark.z])
            
            face_msg = Float32MultiArray()
            face_msg.data = face_landmarks
            self.face_landmarks_pub.publish(face_msg)
            
            # Left hand landmarks
            left_hand_landmarks = []
            if holistic_results.left_hand_landmarks:
                for landmark in holistic_results.left_hand_landmarks.landmark:
                    x_coord = (landmark.x * proc_width) + offset_x
                    y_coord = (landmark.y * proc_height) + offset_y
                    left_hand_landmarks.extend([x_coord, y_coord, landmark.z])
            
            left_hand_msg = Float32MultiArray()
            left_hand_msg.data = left_hand_landmarks
            self.left_hand_landmarks_pub.publish(left_hand_msg)
            
            # Right hand landmarks
            right_hand_landmarks = []
            if holistic_results.right_hand_landmarks:
                for landmark in holistic_results.right_hand_landmarks.landmark:
                    x_coord = (landmark.x * proc_width) + offset_x
                    y_coord = (landmark.y * proc_height) + offset_y
                    right_hand_landmarks.extend([x_coord, y_coord, landmark.z])
            
            right_hand_msg = Float32MultiArray()
            right_hand_msg.data = right_hand_landmarks
            self.right_hand_landmarks_pub.publish(right_hand_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    node = FrontCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()