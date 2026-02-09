#!/usr/bin/env python3
"""
Professional System Dashboard v2.0
Enhanced real-time monitoring with better visuals
"""

import rospy
from std_msgs.msg import String, Float32, Bool
from control_msgs.msg import JointControllerState
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import json
import sys
from collections import deque
from datetime import datetime

try:
    from PyQt5.QtWidgets import *
    from PyQt5.QtCore import *
    from PyQt5.QtGui import *
except ImportError:
    print("ERROR: PyQt5 not installed!")
    print("Install: sudo apt install python3-pyqt5")
    sys.exit(1)

import numpy as np

try:
    import cv2
except ImportError:
    print("ERROR: OpenCV (cv2) not installed.")
    print("Install: sudo apt install python3-opencv")
    sys.exit(1)


class SystemDashboard(QMainWindow):
    # Signals for thread-safe updates
    update_image_signal = pyqtSignal(np.ndarray)
    update_detection_signal = pyqtSignal(str)
    update_stats_signal = pyqtSignal(dict)
    update_gate_signal = pyqtSignal(int, float)
    update_conveyor_signal = pyqtSignal(float, bool)
    
    def __init__(self):
        super().__init__()
        
        rospy.init_node('system_dashboard', anonymous=True)
        
        # Data
        self.bridge = CvBridge()
        self.current_image = None
        self.detection_history = deque(maxlen=20)
        self.gate_states = {i: 0.0 for i in range(1, 7)}
        self.conveyor_speed = 0.0
        self.conveyor_enabled = False
        
        # Statistics
        self.stats = {
            'total_detected': 0,
            'metal': 0,
            'plastic': 0,
            'glass': 0,
            'paper': 0,
            'battery': 0,
            'organic': 0,
            'start_time': datetime.now()
        }
        
        # Gate info (matches sorter_controller.py)
        self.gate_info = {
            1: {'side': 'L', 'color': (128, 128, 128), 'name': 'Metal'},
            2: {'side': 'R', 'color': (255, 100, 50), 'name': 'Plastic'},
            3: {'side': 'L', 'color': (50, 255, 100), 'name': 'Glass'},
            4: {'side': 'R', 'color': (255, 230, 50), 'name': 'Paper'},
            5: {'side': 'L', 'color': (180, 80, 230), 'name': 'Battery'},
            6: {'side': 'R', 'color': (255, 100, 80), 'name': 'Organic'}
        }
        
        # Setup GUI
        self.init_ui()
        
        # Connect signals
        self.update_image_signal.connect(self.update_camera_display)
        self.update_detection_signal.connect(self.update_detection_display)
        self.update_stats_signal.connect(self.update_statistics_display)
        self.update_gate_signal.connect(self.update_gate_display)
        self.update_conveyor_signal.connect(self.update_conveyor_display)
        
        # ROS Subscribers
        self.setup_ros_subscribers()
        
        # Update timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.periodic_update)
        self.timer.start(100)  # 10 Hz
        
        rospy.loginfo("=" * 70)
        rospy.loginfo("📊 PROFESSIONAL DASHBOARD v2.0 INITIALIZED")
        rospy.loginfo("=" * 70)

    def init_ui(self):
        """Initialize UI"""
        self.setWindowTitle('Professional Sorting System Dashboard v2.0')
        self.setGeometry(100, 100, 1600, 900)
        
        # Main widget
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        
        # Left panel - Camera
        left_panel = self.create_left_panel()
        main_layout.addWidget(left_panel, 60)
        
        # Right panel - Status
        right_panel = self.create_right_panel()
        main_layout.addWidget(right_panel, 40)
        
        # Status bar
        self.statusBar().showMessage('🟢 System Ready')
        
        # Apply theme
        self.apply_stylesheet()

    def create_left_panel(self):
        """Left panel with camera"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # Title
        title = QLabel('📹 LIVE CAMERA FEED')
        title.setStyleSheet('font-size: 20px; font-weight: bold; padding: 10px; color: #00ff88;')
        layout.addWidget(title)
        
        # Camera display
        self.camera_label = QLabel()
        self.camera_label.setMinimumSize(800, 600)
        self.camera_label.setStyleSheet('border: 3px solid #00ff88; background: black;')
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera_label.setText('⏳ Waiting for camera...')
        layout.addWidget(self.camera_label)
        
        # Detection log
        detection_group = QGroupBox('📊 DETECTION LOG')
        detection_layout = QVBoxLayout()
        
        self.detection_text = QTextEdit()
        self.detection_text.setReadOnly(True)
        self.detection_text.setMaximumHeight(150)
        self.detection_text.setStyleSheet(
            'font-family: "Courier New"; '
            'background: #1a1a1a; '
            'color: #00ff88; '
            'border: 2px solid #00ff88;'
        )
        detection_layout.addWidget(self.detection_text)
        
        detection_group.setLayout(detection_layout)
        layout.addWidget(detection_group)
        
        return panel

    def create_right_panel(self):
        """Right panel with status"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # System Status
        status_group = self.create_status_group()
        layout.addWidget(status_group)
        
        # Gate Status
        gates_group = self.create_gates_group()
        layout.addWidget(gates_group)
        
        # Statistics
        stats_group = self.create_statistics_group()
        layout.addWidget(stats_group)
        
        # Conveyor Control
        conveyor_group = self.create_conveyor_group()
        layout.addWidget(conveyor_group)
        
        layout.addStretch()
        
        return panel

    def create_status_group(self):
        """System status indicators"""
        group = QGroupBox('⚡ SYSTEM STATUS')
        layout = QGridLayout()
        
        self.status_indicators = {}
        components = [
            ('Camera', 0, 0),
            ('Detector', 0, 1),
            ('Controller', 1, 0),
            ('Belt', 1, 1)
        ]
        
        for name, row, col in components:
            label = QLabel(name)
            label.setStyleSheet('font-size: 12px; font-weight: bold;')
            indicator = QLabel('●')
            indicator.setStyleSheet('color: gray; font-size: 28px;')
            self.status_indicators[name] = indicator
            
            layout.addWidget(label, row*2, col)
            layout.addWidget(indicator, row*2+1, col, Qt.AlignCenter)
        
        group.setLayout(layout)
        return group

    def create_gates_group(self):
        """Gate status displays"""
        group = QGroupBox('🚪 SMART GATES')
        layout = QVBoxLayout()
        
        self.gate_widgets = {}
        
        for gate_id in range(1, 7):
            info = self.gate_info[gate_id]
            
            # Gate container
            gate_widget = QWidget()
            gate_layout = QHBoxLayout(gate_widget)
            gate_layout.setContentsMargins(5, 2, 5, 2)
            
            # Color indicator
            color_label = QLabel('●')
            r, g, b = info['color']
            color_label.setStyleSheet(f'color: rgb({r},{g},{b}); font-size: 20px;')
            gate_layout.addWidget(color_label)
            
            # Info
            info_label = QLabel(f"Gate {gate_id} ({info['side']}) - {info['name']}")
            info_label.setStyleSheet('font-size: 11px; font-weight: bold;')
            gate_layout.addWidget(info_label)
            
            # Progress bar
            progress = QProgressBar()
            progress.setMaximum(100)
            progress.setValue(0)
            progress.setTextVisible(False)
            progress.setMaximumHeight(12)
            progress.setMaximumWidth(120)
            self.gate_widgets[gate_id] = progress
            gate_layout.addWidget(progress)
            
            gate_layout.addStretch()
            
            layout.addWidget(gate_widget)
        
        group.setLayout(layout)
        return group

    def create_statistics_group(self):
        """Statistics display"""
        group = QGroupBox('📈 STATISTICS')
        layout = QVBoxLayout()
        
        self.stats_text = QTextEdit()
        self.stats_text.setReadOnly(True)
        self.stats_text.setMaximumHeight(200)
        self.stats_text.setStyleSheet(
            'font-family: "Courier New"; '
            'background: #1a1a1a; '
            'color: #00d4ff; '
            'border: 2px solid #00d4ff;'
        )
        layout.addWidget(self.stats_text)
        
        # Reset button
        reset_btn = QPushButton('🔄 Reset Statistics')
        reset_btn.setStyleSheet(
            'background: #ff4444; '
            'color: white; '
            'font-weight: bold; '
            'padding: 8px; '
            'border-radius: 5px;'
        )
        reset_btn.clicked.connect(self.reset_statistics)
        layout.addWidget(reset_btn)
        
        group.setLayout(layout)
        return group

    def create_conveyor_group(self):
        """Conveyor control"""
        group = QGroupBox('🭐 BELT CONTROL')
        layout = QVBoxLayout()
        
        # Speed display
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(QLabel('Speed:'))
        self.speed_label = QLabel('0.0 m/s')
        self.speed_label.setStyleSheet(
            'font-weight: bold; '
            'font-size: 16px; '
            'color: #00ff88;'
        )
        speed_layout.addWidget(self.speed_label)
        speed_layout.addStretch()
        layout.addLayout(speed_layout)
        
        # Buttons
        button_layout = QHBoxLayout()
        
        self.enable_btn = QPushButton('▶️ Start')
        self.enable_btn.clicked.connect(lambda: self.publish_conveyor_enable(True))
        self.enable_btn.setStyleSheet(
            'background: #44ff44; '
            'color: black; '
            'font-weight: bold; '
            'padding: 10px;'
        )
        button_layout.addWidget(self.enable_btn)
        
        self.disable_btn = QPushButton('⏸️ Stop')
        self.disable_btn.clicked.connect(lambda: self.publish_conveyor_enable(False))
        self.disable_btn.setStyleSheet(
            'background: #ff4444; '
            'color: white; '
            'font-weight: bold; '
            'padding: 10px;'
        )
        button_layout.addWidget(self.disable_btn)
        
        layout.addLayout(button_layout)
        
        # Speed slider
        speed_control_layout = QHBoxLayout()
        speed_control_layout.addWidget(QLabel('Set Speed:'))
        
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setMinimum(0)
        self.speed_slider.setMaximum(20)
        self.speed_slider.setValue(6)
        self.speed_slider.valueChanged.connect(self.on_speed_changed)
        speed_control_layout.addWidget(self.speed_slider)
        
        self.speed_value_label = QLabel('0.6 m/s')
        speed_control_layout.addWidget(self.speed_value_label)
        
        layout.addLayout(speed_control_layout)
        
        group.setLayout(layout)
        return group

    def apply_stylesheet(self):
        """Dark theme"""
        self.setStyleSheet("""
            QMainWindow {
                background-color: #1a1a1a;
            }
            QWidget {
                background-color: #1a1a1a;
                color: #ffffff;
            }
            QGroupBox {
                border: 2px solid #00ff88;
                border-radius: 8px;
                margin-top: 12px;
                font-weight: bold;
                font-size: 13px;
                padding: 10px;
                color: #00ff88;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 15px;
                padding: 0 8px;
            }
            QPushButton {
                border-radius: 6px;
                font-size: 12px;
            }
            QPushButton:hover {
                opacity: 0.8;
            }
            QProgressBar {
                border: 1px solid #00ff88;
                border-radius: 4px;
                background-color: #0a0a0a;
            }
            QProgressBar::chunk {
                background-color: #00ff88;
            }
        """)

    def setup_ros_subscribers(self):
        """Setup ROS"""
        rospy.Subscriber('/detection_debug', Image, self.camera_callback)
        rospy.Subscriber('/object_detection', String, self.detection_callback)
        rospy.Subscriber('/conveyor/speed', Float32, self.conveyor_speed_callback)
        rospy.Subscriber('/conveyor/status', Bool, self.conveyor_status_callback)
        
        # Gate controllers - doğru mesaj tipi
        for i in range(1, 7):
            rospy.Subscriber(
                f'/gate{i}_controller/state',
                JointControllerState,
                lambda msg, gate_id=i: self.gate_callback(gate_id, msg)
            )
        
        # Publishers
        self.conveyor_enable_pub = rospy.Publisher('/conveyor/enable', Bool, queue_size=1)
        self.conveyor_speed_pub = rospy.Publisher('/conveyor/set_speed', Float32, queue_size=1)

    def camera_callback(self, msg):
        """Handle camera"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.update_image_signal.emit(cv_image)
            self.update_status_indicator('Camera', True)
        except Exception as e:
            rospy.logerr(f"Camera error: {e}")

    def detection_callback(self, msg):
        """Handle detection"""
        try:
            detections = json.loads(msg.data)
            self.update_detection_signal.emit(msg.data)
            
            # Update stats
            for det in detections:
                category = det.get('category', 'unknown')
                if category in self.stats:
                    self.stats[category] += 1
                    self.stats['total_detected'] += 1
            
            self.update_stats_signal.emit(self.stats.copy())
            self.update_status_indicator('Detector', True)
            
        except Exception as e:
            rospy.logerr(f"Detection error: {e}")

    def conveyor_speed_callback(self, msg):
        """Handle speed"""
        self.update_conveyor_signal.emit(msg.data, self.conveyor_enabled)

    def conveyor_status_callback(self, msg):
        """Handle status"""
        self.conveyor_enabled = msg.data
        self.update_conveyor_signal.emit(self.conveyor_speed, msg.data)
        self.update_status_indicator('Belt', msg.data)

    def gate_callback(self, gate_id, msg):
        """Handle gate - JointControllerState mesajından pozisyon al"""
        # msg.process_value kapının mevcut açısını içeriyor
        self.update_gate_signal.emit(gate_id, msg.process_value)

    @pyqtSlot(np.ndarray)
    def update_camera_display(self, image):
        """Update camera"""
        display_image = cv2.resize(image, (800, 600))
        height, width, channel = display_image.shape
        bytes_per_line = 3 * width
        q_image = QImage(display_image.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
        pixmap = QPixmap.fromImage(q_image)
        self.camera_label.setPixmap(pixmap)

    @pyqtSlot(str)
    def update_detection_display(self, data):
        """Update detection log"""
        try:
            detections = json.loads(data)
            timestamp = datetime.now().strftime('%H:%M:%S')
            
            text = f"[{timestamp}] 🎯 {len(detections)} object(s):\n"
            for det in detections:
                color = det.get('color', 'unknown')
                category = det.get('category', 'unknown')
                gate = self.get_gate_for_category(category)
                text += f"  • {color.upper()} → {category.upper()} → Gate {gate}\n"
            
            self.detection_text.append(text)
            
            # Keep last 20
            cursor = self.detection_text.textCursor()
            cursor.movePosition(QTextCursor.Start)
            while self.detection_text.document().blockCount() > 20:
                cursor.select(QTextCursor.BlockUnderCursor)
                cursor.removeSelectedText()
                cursor.deleteChar()
                
        except Exception as e:
            rospy.logerr(f"Update detection error: {e}")

    @pyqtSlot(dict)
    def update_statistics_display(self, stats):
        """Update stats"""
        elapsed = (datetime.now() - stats['start_time']).total_seconds()
        hours = int(elapsed // 3600)
        minutes = int((elapsed % 3600) // 60)
        seconds = int(elapsed % 60)
        
        rate = stats['total_detected'] / elapsed if elapsed > 0 else 0
        
        text = f"""
╔═══════════════════════════════════════╗
║     SORTING STATISTICS v2.0           ║
╠═══════════════════════════════════════╣
║ Runtime: {hours:02d}:{minutes:02d}:{seconds:02d}                      ║
║ Total Sorted: {stats['total_detected']:5d}                 ║
║ Rate: {rate:5.2f} obj/sec                ║
╠═══════════════════════════════════════╣
║ 🔩 Metal:    {stats['metal']:5d}                    ║
║ ♻️  Plastic:  {stats['plastic']:5d}                    ║
║ 🾾 Glass:    {stats['glass']:5d}                    ║
║ 📄 Paper:    {stats['paper']:5d}                    ║
║ 🔋 Battery:  {stats['battery']:5d}                    ║
║ 🗑️  Organic:  {stats['organic']:5d}                    ║
╚═══════════════════════════════════════╝
        """
        
        self.stats_text.setText(text)

    @pyqtSlot(int, float)
    def update_gate_display(self, gate_id, position):
        """Update gate"""
        if gate_id in self.gate_widgets:
            # position -1.15 ile +1.15 arasında, 0-100'e normalize et
            percentage = int(abs(position) / 1.2 * 100)
            self.gate_widgets[gate_id].setValue(min(percentage, 100))
            
            if percentage > 10:
                r, g, b = self.gate_info[gate_id]['color']
                self.gate_widgets[gate_id].setStyleSheet(f"""
                    QProgressBar::chunk {{ background-color: rgb({r},{g},{b}); }}
                """)
            else:
                self.gate_widgets[gate_id].setStyleSheet("""
                    QProgressBar::chunk { background-color: #00ff88; }
                """)

    @pyqtSlot(float, bool)
    def update_conveyor_display(self, speed, enabled):
        """Update conveyor"""
        self.speed_label.setText(f'{speed:.2f} m/s')
        
        if enabled:
            self.speed_label.setStyleSheet('font-weight: bold; font-size: 16px; color: #00ff88;')
        else:
            self.speed_label.setStyleSheet('font-weight: bold; font-size: 16px; color: #ff4444;')

    def update_status_indicator(self, component, active):
        """Update indicator"""
        if component in self.status_indicators:
            if active:
                self.status_indicators[component].setStyleSheet('color: #00ff88; font-size: 28px;')
            else:
                self.status_indicators[component].setStyleSheet('color: #ff4444; font-size: 28px;')

    def periodic_update(self):
        """Periodic"""
        try:
            controller_ok = not rospy.is_shutdown()
            self.update_status_indicator('Controller', controller_ok)
        except:
            self.update_status_indicator('Controller', False)
        
        self.statusBar().showMessage(
            f'🟢 System Running | Total: {self.stats["total_detected"]} objects sorted'
        )

    def reset_statistics(self):
        """Reset stats"""
        for key in self.stats:
            if key != 'start_time':
                self.stats[key] = 0
        self.stats['start_time'] = datetime.now()
        self.update_stats_signal.emit(self.stats.copy())
        rospy.loginfo("Statistics reset")

    def on_speed_changed(self, value):
        """Speed slider"""
        speed = value / 10.0
        self.speed_value_label.setText(f'{speed:.1f} m/s')

    def publish_conveyor_enable(self, enable):
        """Publish enable"""
        msg = Bool()
        msg.data = enable
        self.conveyor_enable_pub.publish(msg)
        rospy.loginfo(f"Belt {'enabled' if enable else 'disabled'}")

    def get_gate_for_category(self, category):
        """Get gate ID"""
        mapping = {
            'metal': 1, 'plastic': 2, 'glass': 3,
            'paper': 4, 'battery': 5, 'organic': 6
        }
        return mapping.get(category, '?')

    def closeEvent(self, event):
        """Handle close"""
        rospy.signal_shutdown("Dashboard closed")
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    
    dashboard = SystemDashboard()
    dashboard.show()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass