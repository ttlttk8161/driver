import queue
import threading
import time
from data_structures import SensorData
import rospy # ROS 사용
from sensor_msgs.msg import Image as RosImage # ROS Image 메시지
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge # CvBridge 사용 (선언만 하고 실제 인스턴스는 외부에서 주입받도록 수정)

class SensorInputManager:
    def __init__(self, config: dict, output_queue: queue.Queue, ros_bridge: CvBridge = None):
        """
        Initializes the sensor input manager.
        config: Dictionary containing sensor configurations.
        output_queue: Queue to send SensorData to.
        ros_bridge: CvBridge instance for converting ROS images.
        """
        self.config = config
        self.output_queue = output_queue
        self._running = False
        self._thread = None
        self.bridge = ros_bridge
        if self.bridge is None:
            print("SensorInputManager: Warning - CvBridge not provided. ROS Image conversion will fail.")
        
        self.latest_image_data = None
        self.latest_lidar_data = None
        self.image_timestamp = None
        self.lidar_timestamp = None
        self.lock = threading.Lock() # For thread-safe access to latest_image/lidar_data
        print("SensorInputManager: Initialized.")

    def _ros_image_callback(self, data: RosImage):
        if not self.bridge:
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            with self.lock:
                self.latest_image_data = cv_image
                self.image_timestamp = data.header.stamp.to_sec()
        except Exception as e:
            print(f"SensorInputManager: Error converting ROS Image: {e}")

    def _ros_lidar_callback(self, data: LaserScan):
        with self.lock:
            # Process or store raw ranges as needed by PerceptionModule
            self.latest_lidar_data = list(data.ranges[0:360]) # 예시: 0-359도 범위 사용
            self.lidar_timestamp = data.header.stamp.to_sec()

    def _publish_sensor_data_loop(self):
        """ Periodically checks for new data and puts it on the queue. """
        publish_rate = self.config.get("publish_rate_hz", 10) # Hz
        sleep_duration = 1.0 / publish_rate

        while self._running:
            current_image = None
            current_lidar = None
            img_ts = None
            lid_ts = None
            
            with self.lock:
                if self.latest_image_data is not None:
                    current_image = self.latest_image_data.copy()
                    img_ts = self.image_timestamp
                    # self.latest_image_data = None # Consume on use or allow overwrite
                if self.latest_lidar_data is not None:
                    current_lidar = list(self.latest_lidar_data)
                    lid_ts = self.lidar_timestamp
                    # self.latest_lidar_data = None # Consume on use or allow overwrite

            # For simplicity, use the latest available data.
            # More sophisticated synchronization might be needed in a real system.
            # We'll use the image timestamp as the primary timestamp for the SensorData bundle.
            if current_image is not None and current_lidar is not None and img_ts is not None:
                sensor_bundle = SensorData(
                    timestamp=img_ts, # Use image timestamp
                    lidar_data=current_lidar,
                    vision_data=current_image,
                    gnss_data=None, # 실제 GNSS 데이터가 있다면 추가
                    imu_data=None   # 실제 IMU 데이터가 있다면 추가
                )
                try:
                    self.output_queue.put(sensor_bundle, timeout=0.5)
                except queue.Full:
                    print("SensorInputManager: Output queue is full. Discarding data.")
            time.sleep(sleep_duration) 
        print("SensorInputManager: Data publishing loop stopped.")

    def start_sensors(self):
        """Starts the sensor data acquisition thread."""
        if not self._running:
            self._running = True
            # ROS Subscribers
            if self.bridge: # Only subscribe if bridge is available
                rospy.Subscriber("/usb_cam/image_raw/", RosImage, self._ros_image_callback, queue_size=1)
                rospy.Subscriber("/scan", LaserScan, self._ros_lidar_callback, queue_size=1)
                print("SensorInputManager: ROS subscribers for Camera and LiDAR started.")
            else:
                print("SensorInputManager: CvBridge not available, ROS subscribers not started. Will use dummy data if _read_and_publish_data is used.")
            
            # Start the thread that packages and queues data
            self._thread = threading.Thread(target=self._publish_sensor_data_loop, name="SensorDataPublisherThread")
            self._thread.start()
            print("SensorInputManager: Sensors started.")
        else:
            print("SensorInputManager: Sensors already running.")

    def stop_sensors(self):
        """Stops the sensor data acquisition thread."""
        if self._running:
            self._running = False
            if self._thread:
                self._thread.join(timeout=2.0) # Add timeout
            # ROS 구독 해제는 보통 노드 종료 시 자동으로 처리되지만, 명시적으로 할 수도 있음
            # (여기서는 별도 해제 로직 불필요)
            print("SensorInputManager: Sensors stopped.")