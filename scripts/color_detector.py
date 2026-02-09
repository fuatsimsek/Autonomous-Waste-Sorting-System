#!/usr/bin/env python3
"""
Professional Color Detector for 6-Category Sorting
Detects: Metal, Plastic, Glass, Paper, Battery, Organic
"""

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
import json


class ColorDetector:
    def __init__(self):
        rospy.init_node('color_detector', anonymous=True)
        
        self.bridge = CvBridge()
        
        # Parameters
        self.image_topic = rospy.get_param('~image_topic', '/camera/image_raw')
        self.min_area = rospy.get_param('~min_area', 150)
        self.show_debug = rospy.get_param('~show_debug', True)
        
        # HSV Color Ranges (6 categories)
        self.color_ranges = {
            'metal': {  # White (metal)
                'lower': np.array([0, 0, 180]),
                'upper': np.array([180, 80, 255]),
                'category': 'metal'
            },
            'plastic': {  # Blue
                'lower': np.array([100, 120, 70]),
                'upper': np.array([130, 255, 255]),
                'category': 'plastic'
            },
            'glass': {  # Green
                'lower': np.array([45, 100, 70]),
                'upper': np.array([75, 255, 255]),
                'category': 'glass'
            },
            'paper': {  # Yellow
                'lower': np.array([20, 100, 100]),
                'upper': np.array([30, 255, 255]),
                'category': 'paper'
            },
            'battery': {  # Purple
                'lower': np.array([130, 100, 70]),
                'upper': np.array([160, 255, 255]),
                'category': 'battery'
            },
            'organic': {  # Red (needs two ranges)
                'lower1': np.array([0, 120, 70]),
                'upper1': np.array([10, 255, 255]),
                'lower2': np.array([170, 120, 70]),
                'upper2': np.array([180, 255, 255]),
                'category': 'organic'
            }
        }
        
        # Publishers
        self.detection_pub = rospy.Publisher('/object_detection', String, queue_size=10)
        
        if self.show_debug:
            self.debug_pub = rospy.Publisher('/detection_debug', Image, queue_size=10)
        
        # Subscriber
        self.image_sub = rospy.Subscriber(self.image_topic, Image, 
                                         self.image_callback, queue_size=1)
        
        # Statistics
        self.frame_count = 0
        self.detection_count = 0
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("COLOR DETECTOR INITIALIZED")
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"Image topic: {self.image_topic}")
        rospy.loginfo(f"Min area: {self.min_area}")
        rospy.loginfo(f"Debug mode: {self.show_debug}")
        rospy.loginfo("Categories: Metal, Plastic, Glass, Paper, Battery, Organic")
        rospy.loginfo("=" * 60)

    def detect_color(self, hsv_image):
        """Detect objects by color and return list of detections"""
        detections = []
        
        for color_name, ranges in self.color_ranges.items():
            # Create mask
            if 'lower1' in ranges:  # Red has two ranges
                mask1 = cv2.inRange(hsv_image, ranges['lower1'], ranges['upper1'])
                mask2 = cv2.inRange(hsv_image, ranges['lower2'], ranges['upper2'])
                mask = cv2.bitwise_or(mask1, mask2)
            else:
                mask = cv2.inRange(hsv_image, ranges['lower'], ranges['upper'])
            
            # Morphological operations to reduce noise
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, 
                                          cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < self.min_area:
                    continue
                
                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)
                
                # Calculate center
                cx = x + w // 2
                cy = y + h // 2
                
                # Normalized coordinates (0-1)
                h_img, w_img = hsv_image.shape[:2]
                norm_x = cx / w_img
                norm_y = cy / h_img
                
                detection = {
                    'color': color_name,
                    'category': ranges['category'],
                    'x': int(cx),
                    'y': int(cy),
                    'w': int(w),
                    'h': int(h),
                    'area': int(area),
                    'norm_x': float(norm_x),
                    'norm_y': float(norm_y)
                }
                
                detections.append(detection)
        
        return detections

    def draw_detections(self, image, detections):
        """Draw detection boxes on image"""
        # Color map for visualization (metal özel: kalın siyah çerçeve)
        color_map = {
            'metal': (0, 0, 0),          # Siyah (metali netleştirmek için)
            'plastic': (255, 100, 0),    # Blue
            'glass': (0, 255, 0),        # Green
            'paper': (0, 255, 255),      # Yellow
            'battery': (255, 0, 255),    # Purple
            'organic': (0, 0, 255)       # Red
        }
        
        for det in detections:
            color = color_map.get(det['color'], (255, 255, 255))
            
            # Draw rectangle
            thickness = 2
            font_scale = 0.5
            if det.get('category') == 'metal':
                thickness = 4   # metal için daha kalın siyah çerçeve
                font_scale = 0.7

            cv2.rectangle(image, 
                         (det['x'] - det['w']//2, det['y'] - det['h']//2),
                         (det['x'] + det['w']//2, det['y'] + det['h']//2),
                         color, thickness)
            
            # Draw center point
            cv2.circle(image, (det['x'], det['y']), 5, color, -1)
            
            # Draw label
            label = f"{det['category'].upper()}"
            cv2.putText(image, label, 
                       (det['x'] - det['w']//2, det['y'] - det['h']//2 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, thickness)
        
        # Draw info
        info_text = f"Frame: {self.frame_count} | Detected: {len(detections)}"
        cv2.putText(image, info_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return image

    def image_callback(self, msg):
        """Process incoming image"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.frame_count += 1
            
            # Convert to HSV
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Detect objects
            detections = self.detect_color(hsv_image)
            
            # Publish detections
            if detections:
                detection_msg = String()
                detection_msg.data = json.dumps(detections)
                self.detection_pub.publish(detection_msg)
                
                self.detection_count += len(detections)
                
                # Log detections
                for det in detections:
                    rospy.loginfo_throttle(
                        1.0,
                        f"🎯 Detected {det['category'].upper()} "
                        f"at ({det['x']}, {det['y']}) area={det['area']}"
                    )
            
            # Publish debug image
            if self.show_debug:
                debug_image = self.draw_detections(cv_image.copy(), detections)
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
                self.debug_pub.publish(debug_msg)
                
        except Exception as e:
            rospy.logerr(f"Image callback error: {e}")

    def run(self):
        """Main loop"""
        rate = rospy.Rate(10)  # 10 Hz status update
        
        while not rospy.is_shutdown():
            # Periodic status
            if self.frame_count % 100 == 0 and self.frame_count > 0:
                rospy.loginfo(
                    f"📊 Status: Frames={self.frame_count}, "
                    f"Detections={self.detection_count}"
                )
            
            rate.sleep()


if __name__ == '__main__':
    try:
        detector = ColorDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
