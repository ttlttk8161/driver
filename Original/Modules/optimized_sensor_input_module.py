"""
최적화된 센서 입력 모듈
- 메모리 풀 사용으로 메모리 할당/해제 오버헤드 감소
- 링 버퍼 큐를 통한 고속 데이터 처리
- 타임스탬프 동기화 강화
- 성능 모니터링 내장
"""

import threading
import time
import logging
import numpy as np
from typing import Optional, Dict, Any
import rospy
from sensor_msgs.msg import Image as RosImage, LaserScan
from cv_bridge import CvBridge

from .optimized_data_structures import (
    OptimizedSensorInput, PerformanceMetrics, DataPriority,
    RingBufferQueue, TimestampSynchronizer, MemoryPool, 
    monitor_performance
)

logger = logging.getLogger(__name__)

class OptimizedSensorInputManager:
    """최적화된 센서 입력 관리자"""
    
    def __init__(self, config: Dict[str, Any], output_queue, ros_bridge: CvBridge = None):
        self.config = config
        self.output_queue = output_queue
        self.bridge = ros_bridge
        self._running = False
        self._thread = None
        
        # 성능 설정
        self.publish_rate_hz = config.get("publish_rate_hz", 20)
        self.enable_performance_monitoring = config.get("enable_performance_monitoring", True)
        self.memory_pool_size = config.get("memory_pool_size", 10)
        self.sync_tolerance_ms = config.get("sync_tolerance_ms", 50.0)
        
        # 최적화된 데이터 구조 초기화
        self.memory_pool = MemoryPool(
            pool_size=self.memory_pool_size,
            image_shape=(480, 640, 3)  # 기본 이미지 크기
        )
        
        self.timestamp_sync = TimestampSynchronizer(tolerance_ms=self.sync_tolerance_ms)
        
        # 고속 링 버퍼 (최신 데이터 위주)
        self.image_buffer = RingBufferQueue(maxsize=5)
        self.lidar_buffer = RingBufferQueue(maxsize=5)
        
        # 센서 데이터 저장
        self.latest_sensor_data = {}
        self.data_lock = threading.RLock()
        
        # 성능 모니터링
        self.performance_metrics = {
            'frames_processed': 0,
            'frames_dropped': 0,
            'avg_processing_time_ms': 0.0,
            'last_fps': 0.0,
            'memory_usage_mb': 0.0
        }
        self.last_fps_calculation = time.time()
        self.fps_frame_count = 0
        
        # 시퀀스 ID 관리
        self.sequence_counter = 0
        self.sequence_lock = threading.Lock()
        
        logger.info("OptimizedSensorInputManager: Initialized with enhanced performance features")
    
    def _get_next_sequence_id(self) -> int:
        """스레드 안전한 시퀀스 ID 생성"""
        with self.sequence_lock:
            self.sequence_counter += 1
            return self.sequence_counter
    
    @monitor_performance
    def _ros_image_callback(self, data: RosImage):
        print("=== _ros_image_callback 진입 ===", flush=True)
        print("self._running:", self._running, "self.bridge:", self.bridge, flush=True)
        logger.info("OptimizedSensorInputManager: _ros_image_callback called")
        """최적화된 이미지 콜백"""
        if not self.bridge or not self._running:
            return
        
        start_time = time.time()
        
        try:
            # 메모리 풀에서 버퍼 획득
            image_buffer = self.memory_pool.get_buffer()
            
            # ROS 이미지를 버퍼에 직접 변환
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            # 버퍼 크기가 맞지 않으면 리사이즈
            if cv_image.shape != image_buffer.shape:
                image_buffer = cv_image.copy()
                logger.warning(f"Image shape mismatch: expected {image_buffer.shape}, got {cv_image.shape}")
            else:
                np.copyto(image_buffer, cv_image)
            
            # 타임스탬프 업데이트
            timestamp = data.header.stamp.to_sec()
            self.timestamp_sync.update_timestamp('camera', timestamp)
            
            # 이미지 데이터를 링 버퍼에 저장
            image_data = {
                'data': image_buffer,
                'timestamp': timestamp,
                'metadata': {
                    'width': data.width,
                    'height': data.height,
                    'encoding': data.encoding,
                    'frame_id': data.header.frame_id
                }
            }
            
            self.image_buffer.put(image_data, overwrite_old=True)
            
            # 성능 메트릭 업데이트
            processing_time = (time.time() - start_time) * 1000
            self._update_performance_metrics(processing_time, 'image')
            
        except Exception as e:
            logger.error(f"OptimizedSensorInputManager: Image callback error: {e}")
            self.performance_metrics['frames_dropped'] += 1
    
    @monitor_performance
    def _ros_lidar_callback(self, data: LaserScan):
        print("=== _ros_lidar_callback 진입 ===", flush=True)
        print("self._running:", self._running, flush=True)
        logger.info("OptimizedSensorInputManager: _ros_lidar_callback called")
        """최적화된 라이다 콜백"""
        if not self._running:
            return
        
        start_time = time.time()
        
        try:
            # 라이다 데이터를 NumPy 배열로 변환 (메모리 효율성)
            ranges = np.array(data.ranges, dtype=np.float32)
            intensities = np.array(data.intensities, dtype=np.float32) if data.intensities else None
            
            # 무한값과 NaN 처리
            ranges = np.where(np.isinf(ranges), data.range_max, ranges)
            ranges = np.where(np.isnan(ranges), 0.0, ranges)
            
            timestamp = data.header.stamp.to_sec()
            self.timestamp_sync.update_timestamp('lidar', timestamp)
            
            # 라이다 데이터를 링 버퍼에 저장
            lidar_data = {
                'ranges': ranges,
                'intensities': intensities,
                'timestamp': timestamp,
                'metadata': {
                    'angle_min': data.angle_min,
                    'angle_max': data.angle_max,
                    'angle_increment': data.angle_increment,
                    'range_min': data.range_min,
                    'range_max': data.range_max,
                    'frame_id': data.header.frame_id
                }
            }
            
            self.lidar_buffer.put(lidar_data, overwrite_old=True)
            
            # 성능 메트릭 업데이트
            processing_time = (time.time() - start_time) * 1000
            self._update_performance_metrics(processing_time, 'lidar')
            
        except Exception as e:
            logger.error(f"OptimizedSensorInputManager: LiDAR callback error: {e}")
    
    def _update_performance_metrics(self, processing_time_ms: float, sensor_type: str):
        """성능 메트릭 업데이트"""
        if not self.enable_performance_monitoring:
            return
        
        with self.data_lock:
            self.performance_metrics['frames_processed'] += 1
            
            # 지수 이동 평균으로 처리 시간 계산
            alpha = 0.1
            current_avg = self.performance_metrics['avg_processing_time_ms']
            self.performance_metrics['avg_processing_time_ms'] = (
                alpha * processing_time_ms + (1 - alpha) * current_avg
            )
            
            # FPS 계산 (1초마다)
            self.fps_frame_count += 1
            current_time = time.time()
            if current_time - self.last_fps_calculation >= 1.0:
                self.performance_metrics['last_fps'] = self.fps_frame_count / (
                    current_time - self.last_fps_calculation
                )
                self.fps_frame_count = 0
                self.last_fps_calculation = current_time
                
                # 메모리 사용량 업데이트
                pool_stats = self.memory_pool.get_stats()
                estimated_memory = (
                    pool_stats['used_buffers'] * 480 * 640 * 3 / (1024 * 1024)
                )
                self.performance_metrics['memory_usage_mb'] = estimated_memory
    
    def _create_optimized_sensor_input(self) -> Optional[OptimizedSensorInput]:
        """최적화된 센서 입력 데이터 생성"""
        try:
            # 최신 이미지 데이터 가져오기
            image_data = None
            lidar_data = None
            try:
                image_data = self.image_buffer.get_latest()
                logger.debug(f"OptimizedSensorInputManager: image_buffer.get_latest() returned: {type(image_data)}")
            except Exception as e:
                logger.warning(f"OptimizedSensorInputManager: image_buffer.get_latest() exception: {e}")
            try:
                lidar_data = self.lidar_buffer.get_latest()
                logger.debug(f"OptimizedSensorInputManager: lidar_buffer.get_latest() returned: {type(lidar_data)}")
            except Exception as e:
                logger.warning(f"OptimizedSensorInputManager: lidar_buffer.get_latest() exception: {e}")
            # 최소한 하나의 센서 데이터는 있어야 함
            if image_data is None and lidar_data is None:
                logger.debug("OptimizedSensorInputManager: Both image_data and lidar_data are None in _create_optimized_sensor_input")
                return None
            # 타임스탬프 동기화 확인
            required_sensors = []
            if image_data:
                required_sensors.append('camera')
            if lidar_data:
                required_sensors.append('lidar')
            is_synchronized, max_offset_ms = self.timestamp_sync.check_synchronization(required_sensors)
            logger.debug(f"OptimizedSensorInputManager: is_synchronized={is_synchronized}, max_offset_ms={max_offset_ms}")
            print(f"[DEBUG] is_synchronized={is_synchronized}, max_offset_ms={max_offset_ms}", flush=True)
            # 동기화되지 않은 경우 경고
            if not is_synchronized:
                logger.warning(f"Sensor synchronization failed: offset {max_offset_ms:.1f}ms")
                print(f"[DEBUG] Sensor synchronization failed: offset {max_offset_ms:.1f}ms", flush=True)
            # 기준 타임스탬프 결정
            base_timestamp = time.time()
            if image_data:
                base_timestamp = image_data['timestamp']
            elif lidar_data:
                base_timestamp = lidar_data['timestamp']
            # 우선순위 결정 (동기화 상태에 따라)
            priority = DataPriority.NORMAL
            if is_synchronized and max_offset_ms < 20.0:
                priority = DataPriority.HIGH
            elif max_offset_ms > 100.0:
                priority = DataPriority.LOW
            # 성능 메트릭 생성
            performance_metrics = PerformanceMetrics(
                timestamp=base_timestamp,
                processing_time_ms=self.performance_metrics['avg_processing_time_ms'],
                queue_size=self.output_queue.qsize() if hasattr(self.output_queue, 'qsize') else 0,
                memory_usage_mb=self.performance_metrics['memory_usage_mb'],
                fps=self.performance_metrics['last_fps'],
                dropped_frames=self.performance_metrics['frames_dropped']
            )
            # OptimizedSensorInput 생성
            sensor_input = OptimizedSensorInput(
                timestamp=base_timestamp,
                sequence_id=self._get_next_sequence_id(),
                priority=priority,
                camera_data=image_data['data'] if image_data else None,
                camera_metadata=image_data['metadata'] if image_data else {},
                lidar_ranges=lidar_data['ranges'] if lidar_data else None,
                lidar_intensities=lidar_data['intensities'] if lidar_data else None,
                lidar_metadata=lidar_data['metadata'] if lidar_data else {},
                performance_metrics=performance_metrics
            )
            print(f"[DEBUG] Created OptimizedSensorInput: ts={sensor_input.timestamp}, seq={sensor_input.sequence_id}, priority={sensor_input.priority}", flush=True)
            return sensor_input
        except Exception as e:
            logger.error(f"OptimizedSensorInputManager: Error creating sensor input: {e}")
            print(f"[DEBUG] Error creating sensor input: {e}", flush=True)
            return None
    
    def _publish_sensor_data_loop(self):
        """최적화된 센서 데이터 발행 루프"""
        sleep_duration = 1.0 / self.publish_rate_hz
        last_publish_time = time.time()
        print("[PRINT] _publish_sensor_data_loop started", flush=True)
        logger.info("OptimizedSensorInputManager: _publish_sensor_data_loop started")
        while self._running:
            try:
                current_time = time.time()
                if current_time - last_publish_time < sleep_duration:
                    time.sleep(0.001)
                    continue
                print("[PRINT] Loop tick - attempting to create sensor input", flush=True)
                logger.debug("OptimizedSensorInputManager: Loop tick - attempting to create sensor input")
                sensor_input = self._create_optimized_sensor_input()
                if sensor_input:
                    print(f"[PRINT] Putting sensor data to queue (timestamp={sensor_input.timestamp}, seq={sensor_input.sequence_id})", flush=True)
                    logger.info(f"OptimizedSensorInputManager: Putting sensor data to queue (timestamp={sensor_input.timestamp}, seq={sensor_input.sequence_id})")
                    try:
                        self.output_queue.put(sensor_input)
                        print("[PRINT] Successfully put sensor_input to output_queue", flush=True)
                        last_publish_time = current_time
                    except Exception as e:
                        print(f"[PRINT] Output queue full or error: {e}", flush=True)
                        logger.warning(f"OptimizedSensorInputManager: Output queue full or error: {e}")
                        self.performance_metrics['frames_dropped'] += 1
                        if sensor_input.camera_data is not None:
                            self.memory_pool.return_buffer(sensor_input.camera_data)
                else:
                    print("[PRINT] No sensor_input created (image_data or lidar_data may be missing)", flush=True)
                    logger.warning("OptimizedSensorInputManager: No sensor_input created (image_data or lidar_data may be missing)")
                if self.performance_metrics['last_fps'] > self.publish_rate_hz * 1.2:
                    time.sleep(sleep_duration * 0.5)
                else:
                    time.sleep(sleep_duration * 0.1)
            except Exception as e:
                print(f"[PRINT] Publish loop error: {e}", flush=True)
                logger.error(f"OptimizedSensorInputManager: Publish loop error: {e}")
                time.sleep(0.01)
    
    def start_sensors(self):
        """센서 시작"""
        if self._running:
            logger.warning("OptimizedSensorInputManager: Already running")
            return
        
        self._running = True
        
        # ROS 구독자 설정
        try:
            self.image_subscriber = rospy.Subscriber(
                "/usb_cam/image_raw", RosImage, self._ros_image_callback, queue_size=1
            )
            self.lidar_subscriber = rospy.Subscriber(
                "/scan", LaserScan, self._ros_lidar_callback, queue_size=1
            )
            logger.info("OptimizedSensorInputManager: ROS subscribers created")
        except Exception as e:
            logger.error(f"OptimizedSensorInputManager: Failed to create ROS subscribers: {e}")
            self._running = False
            return
        
        # 발행 스레드 시작
        self._thread = threading.Thread(target=self._publish_sensor_data_loop, name="OptimizedSensorPublisher")
        self._thread.daemon = True
        self._thread.start()
        
        logger.info("OptimizedSensorInputManager: Started successfully")
    
    def stop(self):
        """센서 중지"""
        if not self._running:
            return
        
        logger.info("OptimizedSensorInputManager: Stopping...")
        self._running = False
        
        # ROS 구독자 해제
        try:
            if hasattr(self, 'image_subscriber'):
                self.image_subscriber.unregister()
            if hasattr(self, 'lidar_subscriber'):
                self.lidar_subscriber.unregister()
        except Exception as e:
            logger.error(f"OptimizedSensorInputManager: Error unregistering subscribers: {e}")
        
        # 스레드 종료 대기
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=5.0)
            if self._thread.is_alive():
                logger.warning("OptimizedSensorInputManager: Thread did not terminate gracefully")
        
        # 성능 통계 출력
        self._log_final_statistics()
        
        logger.info("OptimizedSensorInputManager: Stopped")
    
    def _log_final_statistics(self):
        """최종 성능 통계 로깅"""
        try:
            sync_stats = self.timestamp_sync.get_sync_statistics()
            pool_stats = self.memory_pool.get_stats()
            
            logger.info("=== OptimizedSensorInputManager Final Statistics ===")
            logger.info(f"Frames processed: {self.performance_metrics['frames_processed']}")
            logger.info(f"Frames dropped: {self.performance_metrics['frames_dropped']}")
            logger.info(f"Average processing time: {self.performance_metrics['avg_processing_time_ms']:.2f}ms")
            logger.info(f"Final FPS: {self.performance_metrics['last_fps']:.1f}")
            logger.info(f"Sync rate: {sync_stats.get('sync_rate', 0.0) * 100:.1f}%")
            logger.info(f"Max sync offset: {sync_stats.get('max_offset_ms', 0.0):.1f}ms")
            logger.info(f"Memory pool usage: {pool_stats}")
            logger.info(f"Image buffer overwrites: {self.image_buffer.get_overwrite_count()}")
            logger.info(f"LiDAR buffer overwrites: {self.lidar_buffer.get_overwrite_count()}")
            
        except Exception as e:
            logger.error(f"OptimizedSensorInputManager: Error logging statistics: {e}")
    
    def get_performance_metrics(self) -> Dict[str, Any]:
        """현재 성능 메트릭 반환"""
        with self.data_lock:
            metrics = self.performance_metrics.copy()
            metrics.update({
                'sync_stats': self.timestamp_sync.get_sync_statistics(),
                'memory_pool_stats': self.memory_pool.get_stats(),
                'image_buffer_size': self.image_buffer.qsize(),
                'lidar_buffer_size': self.lidar_buffer.qsize(),
                'output_queue_size': self.output_queue.qsize() if hasattr(self.output_queue, 'qsize') else 0
            })
            return metrics
