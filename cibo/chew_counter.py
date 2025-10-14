#!/usr/bin/env python3
import time
import math
from collections import deque
from enum import Enum
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import Int32, Float32, Float32MultiArray, String

class EatingState(Enum):
    IDLE = 0
    FEEDING = 1
    CHEWING = 2
    SPEAKING = 3

# MediaPipe landmark indices
class LandmarkIndices:
    # Face landmarks (468 points)
    NOSE_TIP = 1
    UPPER_LIP = 13
    LOWER_LIP = 14
    LEFT_MOUTH_CORNER = 61
    RIGHT_MOUTH_CORNER = 291
    CHIN = 175
    
    # Hand landmarks (21 points each)
    WRIST = 0
    THUMB_TIP = 4
    INDEX_TIP = 8
    MIDDLE_TIP = 12
    RING_TIP = 16
    PINKY_TIP = 20

NUM_FACE_LANDMARKS = 468
NUM_HAND_LANDMARKS = 21
STRIDE = 3  # x, y, z coordinates

def get_xyz(flat_array, idx, stride=STRIDE):
    """Extract x, y, z coordinates from flattened landmark array"""
    base = idx * stride
    if base + 2 >= len(flat_array):
        return None
    return np.array([flat_array[base], flat_array[base + 1], flat_array[base + 2]])

def safe_distance(point1, point2):
    """Calculate Euclidean distance between two 3D points"""
    if point1 is None or point2 is None:
        return float('inf')
    return float(np.linalg.norm(point1 - point2))

class EatingStateDetector(Node):
    def __init__(self):
        super().__init__('eating_state_detector')
        
        # Parameters
        self.declare_parameter('ema_alpha', 0.1)
        self.declare_parameter('feeding_distance_threshold', 0.15)  # Distance threshold for feeding detection
        self.declare_parameter('speaking_mar_threshold', 0.02)     # MAR threshold for speaking
        self.declare_parameter('chewing_mar_high', 0.008)
        self.declare_parameter('chewing_mar_low', 0.005)
        self.declare_parameter('min_chewing_interval', 0.3)
        self.declare_parameter('state_stability_frames', 3)
        
        # Get parameters
        self.ema_alpha = float(self.get_parameter('ema_alpha').value)
        self.feeding_threshold = float(self.get_parameter('feeding_distance_threshold').value)
        self.speaking_threshold = float(self.get_parameter('speaking_mar_threshold').value)
        self.chewing_high = float(self.get_parameter('chewing_mar_high').value)
        self.chewing_low = float(self.get_parameter('chewing_mar_low').value)
        self.min_chewing_interval = float(self.get_parameter('min_chewing_interval').value)
        self.stability_frames = int(self.get_parameter('state_stability_frames').value)
        
        # QoS profile
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            Float32MultiArray, '/front_camera/pose_landmarks', 
            self.pose_callback, qos)
        self.face_sub = self.create_subscription(
            Float32MultiArray, '/front_camera/face_landmarks', 
            self.face_callback, qos)
        self.left_hand_sub = self.create_subscription(
            Float32MultiArray, '/front_camera/left_hand_landmarks', 
            self.left_hand_callback, qos)
        self.right_hand_sub = self.create_subscription(
            Float32MultiArray, '/front_camera/right_hand_landmarks', 
            self.right_hand_callback, qos)
        
        # Publishers
        self.state_pub = self.create_publisher(String, '/eating_state/current_state', 10)
        self.chewing_count_pub = self.create_publisher(Int32, '/eating_state/chewing_count', 10)
        
        # Individual distance metric publishers
        self.dh_pub = self.create_publisher(Float32, '/eating_state/dh', 10)
        self.dj_pub = self.create_publisher(Float32, '/eating_state/dj', 10)
        self.dm_pub = self.create_publisher(Float32, '/eating_state/dm', 10)
        
        # Combined metrics publisher (optional)
        self.metrics_pub = self.create_publisher(Float32MultiArray, '/eating_state/metrics', 10)
        
        # State variables
        self.current_landmarks = {
            'pose': None,
            'face': None,
            'left_hand': None,
            'right_hand': None
        }
        
        # Metrics
        self.dh_r = float('inf')  # Distance: nose tip to right palm
        self.dh_l = float('inf')  # Distance: nose tip to left palm
        self.dh = float('inf')    # Min of dhR and dhL
        self.dj = float('inf')    # Distance: nose tip to chin
        self.dm = 0.0             # Distance: upper lip to lower lip (MAR related)
        self.mar_ema = None       # Exponential moving average of MAR
        
        # State tracking
        self.current_state = EatingState.IDLE
        self.state_history = deque(maxlen=self.stability_frames)
        self.chewing_count = 0
        self.last_chewing_time = 0.0
        self.chewing_cycle_open = False
        
        self.get_logger().info('Eating State Detector initialized')
        self.get_logger().info('Publishing individual distance metrics:')
        self.get_logger().info('  - /eating_state/dh')
        self.get_logger().info('  - /eating_state/dj')  
        self.get_logger().info('  - /eating_state/dm')
    
    def pose_callback(self, msg: Float32MultiArray):
        """Handle pose landmarks"""
        self.current_landmarks['pose'] = msg.data
        self.update_state()
    
    def face_callback(self, msg: Float32MultiArray):
        """Handle face landmarks"""
        self.current_landmarks['face'] = msg.data
        self.update_state()
    
    def left_hand_callback(self, msg: Float32MultiArray):
        """Handle left hand landmarks"""
        self.current_landmarks['left_hand'] = msg.data
        self.update_state()
    
    def right_hand_callback(self, msg: Float32MultiArray):
        """Handle right hand landmarks"""
        self.current_landmarks['right_hand'] = msg.data
        self.update_state()
    
    def calculate_hand_palm_center(self, hand_landmarks):
        """Calculate palm center from hand landmarks"""
        if not hand_landmarks or len(hand_landmarks) < NUM_HAND_LANDMARKS * STRIDE:
            return None
        
        # Use wrist and base of fingers to estimate palm center
        wrist = get_xyz(hand_landmarks, LandmarkIndices.WRIST)
        index_base = get_xyz(hand_landmarks, 5)  # Index finger base
        middle_base = get_xyz(hand_landmarks, 9)  # Middle finger base
        ring_base = get_xyz(hand_landmarks, 13)   # Ring finger base
        pinky_base = get_xyz(hand_landmarks, 17)  # Pinky base
        
        if any(p is None for p in [wrist, index_base, middle_base, ring_base, pinky_base]):
            return None
        
        # Average of key palm points
        palm_center = (wrist + index_base + middle_base + ring_base + pinky_base) / 5.0
        return palm_center
    
    def calculate_metrics(self):
        """Calculate all required distance metrics"""
        face_landmarks = self.current_landmarks['face']
        left_hand = self.current_landmarks['left_hand']
        right_hand = self.current_landmarks['right_hand']
        
        if not face_landmarks or len(face_landmarks) < NUM_FACE_LANDMARKS * STRIDE:
            return
        
        # Get nose tip
        nose_tip = get_xyz(face_landmarks, LandmarkIndices.NOSE_TIP)
        if nose_tip is None:
            return
        
        # Calculate dhR (nose tip to right palm)
        if right_hand and len(right_hand) >= NUM_HAND_LANDMARKS * STRIDE:
            right_palm = self.calculate_hand_palm_center(right_hand)
            if right_palm is not None:
                self.dh_r = safe_distance(nose_tip, right_palm)
        
        # Calculate dhL (nose tip to left palm)
        if left_hand and len(left_hand) >= NUM_HAND_LANDMARKS * STRIDE:
            left_palm = self.calculate_hand_palm_center(left_hand)
            if left_palm is not None:
                self.dh_l = safe_distance(nose_tip, left_palm)
        
        # Calculate dh (minimum of dhR and dhL)
        valid_distances = [d for d in [self.dh_r, self.dh_l] if d != float('inf')]
        if valid_distances:
            self.dh = min(valid_distances)
        else:
            self.dh = float('inf')
        
        # Calculate dj (nose tip to chin)
        chin = get_xyz(face_landmarks, LandmarkIndices.CHIN)
        if chin is not None:
            self.dj = safe_distance(nose_tip, chin)
        
        # Calculate dm and MAR (mouth aspect ratio)
        upper_lip = get_xyz(face_landmarks, LandmarkIndices.UPPER_LIP)
        lower_lip = get_xyz(face_landmarks, LandmarkIndices.LOWER_LIP)
        left_corner = get_xyz(face_landmarks, LandmarkIndices.LEFT_MOUTH_CORNER)
        right_corner = get_xyz(face_landmarks, LandmarkIndices.RIGHT_MOUTH_CORNER)
        
        if all(p is not None for p in [upper_lip, lower_lip, left_corner, right_corner]):
            vertical_dist = safe_distance(upper_lip, lower_lip)
            horizontal_dist = safe_distance(left_corner, right_corner)
            
            self.dm = vertical_dist
            
            # Calculate MAR (Mouth Aspect Ratio)
            if horizontal_dist > 0:
                mar = vertical_dist / horizontal_dist
                
                # Apply exponential moving average
                if self.mar_ema is None:
                    self.mar_ema = mar
                else:
                    self.mar_ema = self.ema_alpha * mar + (1.0 - self.ema_alpha) * self.mar_ema
    
    def publish_distance_metrics(self):
        """Publish individual distance metrics"""
        # Publish dh (minimum hand-to-nose distance)
        dh_msg = Float32()
        dh_msg.data = float(self.dh) if self.dh != float('inf') else -1.0
        self.dh_pub.publish(dh_msg)
        
        # Publish dj (nose-to-chin distance)
        dj_msg = Float32()
        dj_msg.data = float(self.dj) if self.dj != float('inf') else -1.0
        self.dj_pub.publish(dj_msg)
        
        # Publish dm (mouth opening distance)
        dm_msg = Float32()
        dm_msg.data = float(self.dm)
        self.dm_pub.publish(dm_msg)
    
    def detect_feeding(self):
        """Detect feeding state based on hand-to-mouth distance"""
        return self.dh != float('inf') and self.dh < self.feeding_threshold
    
    def detect_speaking(self):
        """Detect speaking state based on mouth opening patterns"""
        if self.mar_ema is None:
            return False
        
        # Speaking typically involves moderate mouth opening with variation
        return self.mar_ema > self.speaking_threshold and not self.detect_chewing()
    
    def detect_chewing(self):
        """Detect chewing state and count chewing cycles"""
        if self.mar_ema is None:
            return False
        
        current_time = time.time()
        
        # Chewing cycle detection with hysteresis
        if not self.chewing_cycle_open and self.mar_ema > self.chewing_high:
            # Start of chewing cycle (mouth opening)
            self.chewing_cycle_open = True
        elif self.chewing_cycle_open and self.mar_ema < self.chewing_low:
            # End of chewing cycle (mouth closing)
            self.chewing_cycle_open = False
            
            # Count chewing if enough time has passed since last chew
            if current_time - self.last_chewing_time > self.min_chewing_interval:
                self.chewing_count += 1
                self.last_chewing_time = current_time
                
                # Publish chewing count
                count_msg = Int32()
                count_msg.data = self.chewing_count
                self.chewing_count_pub.publish(count_msg)
                
                self.get_logger().info(f'Chewing count: {self.chewing_count}')
        
        # Return True if we're in a chewing pattern (moderate mouth movement)
        return (self.chewing_low < self.mar_ema < self.chewing_high * 2.0 and 
                not self.detect_feeding())
    
    def determine_eating_state(self):
        """Determine the current eating state based on all metrics"""
        # Priority order: Feeding > Speaking > Chewing > Idle
        
        if self.detect_feeding():
            return EatingState.FEEDING
        elif self.detect_speaking():
            return EatingState.SPEAKING
        elif self.detect_chewing():
            return EatingState.CHEWING
        else:
            return EatingState.IDLE
    
    def update_state(self):
        """Update eating state based on current landmarks"""
        # Calculate all metrics
        self.calculate_metrics()
        
        # Publish individual distance metrics
        self.publish_distance_metrics()
        
        # Determine new state
        new_state = self.determine_eating_state()
        
        # Add to history for stability
        self.state_history.append(new_state)
        
        # Only change state if it's been stable for required frames
        if len(self.state_history) >= self.stability_frames:
            if all(state == new_state for state in self.state_history):
                if self.current_state != new_state:
                    self.current_state = new_state
                    self.get_logger().info(f'State changed to: {self.current_state.name}')
        
        # Publish current state
        state_msg = String()
        state_msg.data = self.current_state.name
        self.state_pub.publish(state_msg)
        
        # Publish combined metrics (optional)
        metrics_msg = Float32MultiArray()
        metrics_msg.data = [
            float(self.dh_r) if self.dh_r != float('inf') else -1.0,
            float(self.dh_l) if self.dh_l != float('inf') else -1.0,
            float(self.dh) if self.dh != float('inf') else -1.0,
            float(self.dj) if self.dj != float('inf') else -1.0,
            float(self.dm),
            float(self.mar_ema if self.mar_ema is not None else 0.0)
        ]
        self.metrics_pub.publish(metrics_msg)

def main(args=None):
    rclpy.init(args=args)
    node = EatingStateDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()