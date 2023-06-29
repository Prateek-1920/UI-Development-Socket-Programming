import rospy
import pandas as pd
import sys
import cv2
import math
import tf.transformations as tf
import datetime
import pandas as pd

from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QLabel, QPushButton, QMessageBox, QHBoxLayout
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import QTimer, Qt, QObject , pyqtSignal
from PyQt5.QtGui import QColor
from sensor_msgs.msg import Imu, NavSatFix, LaserScan
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from tf.transformations import quaternion_from_euler

class ObstacleDetector(QObject):
    obstacle_detected = pyqtSignal(bool)

    def detect_obstacle(self, obstacle_range, threshold):
        obstacle = min(obstacle_range) < threshold
        self.obstacle_detected.emit(obstacle)

class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.setWindowTitle('GUI')
        self.setGeometry(100, 100, 800, 600)

        self.imu_data = None
        self.gps_data = None
        # self.obstacle_detected = False
        self.show_quaternions = True
        self.video_writer = None
        self.recording = False
        self.recording_data = False
        self.data_stored = []

        self.obstacle_detector = ObstacleDetector()
        self.obstacle_detector.obstacle_detected.connect(self.handle_obstacle_detected)


        self.init_ui()
        self.init_ros()

    def init_ui(self):
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)

        self.layout = QVBoxLayout(self.central_widget)

        self.imu_label = QLabel('IMU Data:')
        self.layout.addWidget(self.imu_label)

        self.test_label = QLabel('Test Feed:')
        self.layout.addWidget(self.test_label)

        self.video_label = QLabel('Video Feed:')
        self.video_label.setFixedSize(400, 300)
        self.layout.addWidget(self.video_label)

        self.gps_label = QLabel('GPS Data:')
        self.layout.addWidget(self.gps_label)

        self.clear_box = QLabel('ALL CLEAR')
        self.clear_box.setStyleSheet("background-color: green")
        self.layout.addWidget(self.clear_box)


        # Add the QHBoxLayout to the main layout

        self.imu_button = QPushButton('Toggle IMU Data Representation')
        self.imu_button.clicked.connect(self.toggle_imu_representation)
        self.layout.addWidget(self.imu_button)

        self.screenshot_button = QPushButton('Screenshot')
        self.screenshot_button.clicked.connect(self.capture_screenshot)
        self.layout.addWidget(self.screenshot_button)

        self.record_button = QPushButton('Record')
        self.record_button.clicked.connect(self.toggle_record)
        self.layout.addWidget(self.record_button)

        self.data_button = QPushButton('Record Sensor Data')
        self.data_button.clicked.connect(self.toggle_data_record)
        self.layout.addWidget(self.data_button)

    def init_ros(self):
        rospy.init_node('ros_data_visualizer')

        # Subscribe to IMU topic
        rospy.Subscriber('/phone_imu', Imu, self.imu_callback)

        # Subscribe to video topic
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.video_callback)

        # Subscribe to GPS topic
        rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)

        # Subscribe to obstacle detection topic
        rospy.Subscriber('/laserscan', LaserScan, self.obstacle_callback)

    def video_callback(self, msg):

        self.test_label.setText('Video Feed:')
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        height, width, channel = rgb_image.shape
        bytes_per_line = 3 * width
        q_image = QImage(rgb_image.data, width, height,
                         bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(q_image)
        self.video_label.setPixmap(pixmap.scaledToWidth(400))
        if self.recording:
            if self.video_writer is None:
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                self.video_writer = cv2.VideoWriter(
                    'recorded_video.avi', fourcc, 25.0, (width, height))

            self.video_writer.write(cv_image)

    def toggle_record(self):
        if self.recording:
            self.stop_record()
        else:
            self.start_record()

    def start_record(self):
        self.recording = True
        self.record_button.setText('Stop')

    def stop_record(self):
        self.recording = False
        self.record_button.setText('Record')

        if self.video_writer is not None:

            self.video_writer.release()
            self.video_writer = None
            current_time = datetime.datetime.now().strftime("%Y%m%d%H%M%S")

            file_name = f"video_{current_time}.avi"
            QMessageBox.information(self, 'Video Recording Stopped',
                                    f"Video recording stopped. File saved as '{file_name}'.")

    def capture_screenshot(self):

        if self.video_label.pixmap():

            screenshot = self.video_label.pixmap().toImage()
            current_time = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
            file_name = f"screenshot_{current_time}.png"
            screenshot.save(file_name)
            # print(f"Screenshot captured and saved as '{file_name}'")
            QMessageBox.information(
                self, 'Screenshot Taken', f"Screenshot has been captured and saved as '{file_name}'")

    def toggle_data_record(self):
        if self.recording_data:
            self.stop_data_record()
        else:
            self.start_data_record()

    def start_data_record(self):
        self.recording_data = True
        self.data_button.setText('Stop Recording Sensor Data')

        # Create a pandas DataFrame to store the recorded data
        # self.data_stored = pd.DataFrame(columns=['Timestamp', 'Latitude', 'Longitude', 'Obstacle'
        #                                         'Linear Acceleration X', 'Linear Acceleration Y', 'Linear Acceleration Z',
        #                                         'Angular Velocity X', 'Angular Velocity Y', 'Angular Velocity Z',
        #                                         'Orientation X', 'Orientation Y', 'Orientation Z', 'Orientation W'])

    def stop_data_record(self):
        self.recording_data = False
        self.data_button.setText('Record Sensor Data')

        # if not self.data_stored.empty:
        if len(self.data_stored) > 0:
            # Save the recorded data to a CSV file
            current_time = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
            file_name = f"data_{current_time}.csv"
            df = pd.DataFrame(self.data_stored)
            df.to_csv(f'{file_name}', index=False)
            # self.data_stored.to_csv(f'{file_name}', index=False)

            QMessageBox.information(
                self, 'Data Saved', f'Data has been saved in "{file_name}".')

    def gps_callback(self, msg):
        # gps_data = msg.data.split(",")
        latitude = msg.latitude
        longitude = msg.longitude
        self.gps_label.setText(
            f'GPS Data:\nLatitude: {latitude}\nLongitude: {longitude}')
        if self.recording_data:

            self.data_stored.append({'GPS': [latitude, longitude]})

    def handle_obstacle_detected(self, obstacle):
        if obstacle:
            self.clear_box.setText('OBSTACLE DETECTED')
            self.clear_box.setStyleSheet('background-color: red;')
        else:
            self.clear_box.setText('ALL CLEAR')
            self.clear_box.setStyleSheet('background-color: green;')


    def obstacle_callback(self, msg):
        lidar_range = msg.ranges
        obstacle_range = lidar_range[0:270]
        threshold = 2.0

        self.obstacle_detector.detect_obstacle(obstacle_range, threshold)

        if self.recording_data:
            obstacle = min(obstacle_range) < threshold
            self.data_stored.append({'Obstacle': [obstacle]})

        if self.recording_data:

            self.data_stored.append({'Obstacle': [obstacle]})

    def quaternion_to_euler(self, x, y, z, w):
        quaternion = (x, y, z, w)
        euler = tf.euler_from_quaternion(quaternion)
        return euler

    # def quaternions_to_euler(self, x, y, z, w):

    #     # Conversion from quaternions to euler angles
    #     t0 = +2.0 * (w * x + y * z)
    #     t1 = +1.0 - 2.0 * (x * x + y * y)
    #     roll = math.atan2(t0, t1)

    #     t2 = +2.0 * (w * y - z * x)
    #     t2 = +1.0 if t2 > +1.0 else t2
    #     t2 = -1.0 if t2 < -1.0 else t2
    #     pitch = math.asin(t2)

    #     t3 = +2.0 * (w * z + x * y)
    #     t4 = +1.0 - 2.0 * (y * y + z * z)
    #     yaw = math.atan2(t3, t4)

    #     return roll, pitch, yaw

    def imu_callback(self, msg):
        linear_acceleration = msg.linear_acceleration
        angular_velocity = msg.angular_velocity
        orientation = msg.orientation

        if self.show_quaternions:
            imu_data = f"Linear Acceleration:\nX: {linear_acceleration.x}\nY: {linear_acceleration.y}\nZ: {linear_acceleration.z}\n" \
                       f"Angular Velocity:\nX: {angular_velocity.x}\nY: {angular_velocity.y}\nZ: {angular_velocity.z}\n" \
                       f"Orientation (Quaternions):\nX: {orientation.x}\nY: {orientation.y}\nZ: {orientation.z}\nW: {orientation.w}"
            self.imu_label.setText(f"IMU DATA: Quaternions\n{imu_data}")
        else:
            roll, pitch, yaw = self.quaternion_to_euler(
                orientation.x, orientation.y, orientation.z, orientation.w)
            imu_data = f"Linear Acceleration:\nX: {linear_acceleration.x}\nY: {linear_acceleration.y}\nZ: {linear_acceleration.z}\n" \
                f"Angular Velocity:\nX: {angular_velocity.x}\nY: {angular_velocity.y}\nZ: {angular_velocity.z}\n" \
                f"Orientation (Euler Angles in degrees):\nRoll: {math.degrees(roll)}\nPitch: {math.degrees(pitch)}\nYaw: {math.degrees(yaw)}"
            self.imu_label.setText(f"IMU DATA: Euler Angles\n{imu_data}")

        if self.recording_data:

            self.data_stored.append({'Acceleration': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
                                     'Velocity': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
                                     'Orientation': [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]})

    def toggle_imu_representation(self):
        self.show_quaternions = not self.show_quaternions
        if self.show_quaternions:
            self.imu_label.setText("IMU DATA: Quaternions")
        else:
            self.imu_label.setText("IMU DATA: Euler Angles")


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
