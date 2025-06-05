# 최적화된 데이터 구조 모듈

import time
import threading
import queue
import _queue
import collections
import weakref
from typing import NamedTuple, List, Any, Tuple, Optional, Dict, Union
from dataclasses import dataclass, field
from enum import Enum
import numpy as np
import logging

@dataclass
class PerformanceMetrics:
    """성능 메트릭"""
    timestamp: float = field(default_factory=time.time)
    processing_time_ms: float = 0.0
    queue_size: int = 0
    memory_usage_mb: float = 0.0
    fps: float = 0.0
    dropped_frames: int = 0

class DataPriority(Enum):
    """데이터 우선순위"""
    LOW = 1
    NORMAL = 2
    HIGH = 3
    CRITICAL = 4

# 우선순위 큐 아이템
@dataclass
class PriorityQueueItem:
    """우선순위 큐 아이템"""
    priority: DataPriority
    timestamp: float
    data: Any
    
    def __lt__(self, other):
        # 우선순위가 높을수록 먼저 처리
        if self.priority.value != other.priority.value:
            return self.priority.value > other.priority.value
        # 우선순위가 같으면 타임스탬프 순
        return self.timestamp < other.timestamp

# 최적화된 센서 데이터 구조
@dataclass
class OptimizedSensorInput:
    """메모리 효율적인 센서 입력"""
    timestamp: float
    sequence_id: int = 0
    priority: DataPriority = DataPriority.NORMAL
    
    # 이미지 데이터 (참조로 관리하여 복사 최소화)
    camera_data: Optional[np.ndarray] = None
    camera_metadata: Dict[str, Any] = field(default_factory=dict)
    
    # 라이다 데이터 (압축된 형태로 저장)
    lidar_ranges: Optional[np.ndarray] = None
    lidar_intensities: Optional[np.ndarray] = None
    lidar_metadata: Dict[str, Any] = field(default_factory=dict)
    
    # GNSS/IMU 데이터
    gnss_data: Optional[Dict[str, float]] = None
    imu_data: Optional[Dict[str, float]] = None
    
    # 성능 메트릭
    performance_metrics: Optional[PerformanceMetrics] = None
    
    def get_memory_footprint(self) -> float:
        """데이터의 메모리 사용량 계산 (MB)"""
        size_bytes = 0
        if self.camera_data is not None:
            size_bytes += self.camera_data.nbytes
        if self.lidar_ranges is not None:
            size_bytes += self.lidar_ranges.nbytes
        if self.lidar_intensities is not None:
            size_bytes += self.lidar_intensities.nbytes
        return size_bytes / (1024 * 1024)
    
    def is_synchronized(self, tolerance_ms: float = 50.0) -> bool:
        """센서 데이터 간 동기화 상태 확인"""
        timestamps = []
        if self.camera_data is not None and 'timestamp' in self.camera_metadata:
            timestamps.append(self.camera_metadata['timestamp'])
        if self.lidar_ranges is not None and 'timestamp' in self.lidar_metadata:
            timestamps.append(self.lidar_metadata['timestamp'])
        
        if len(timestamps) < 2:
            return True
        
        max_diff = max(timestamps) - min(timestamps)
        return max_diff <= (tolerance_ms / 1000.0)

# 통합된 인지 출력 구조
@dataclass
class OptimizedPerceptionOutput:
    """최적화된 인지 모듈 출력"""
    timestamp: float
    sequence_id: int
    processing_time_ms: float = 0.0
    
    # 차선 감지 결과 (HSV 기반)
    white_line_metrics: Optional['WhiteLineHsvMetrics'] = None
    yellow_line_metrics: Optional['YellowLineHsvMetrics'] = None
    
    # 객체 감지 결과
    detected_objects: List['DetectedObject'] = field(default_factory=list)
    
    # 세그멘테이션 결과 (필요시에만 저장)
    lane_mask: Optional[np.ndarray] = None
    drivable_area_mask: Optional[np.ndarray] = None
    
    # 신뢰도 및 품질 지표
    detection_confidence: float = 0.0
    quality_score: float = 0.0
    
    def compress_masks(self):
        """마스크 데이터 압축 (메모리 절약)"""
        if self.lane_mask is not None:
            # 이진 마스크를 압축된 형태로 변환
            self.lane_mask = np.packbits(self.lane_mask.astype(np.uint8))
        if self.drivable_area_mask is not None:
            self.drivable_area_mask = np.packbits(self.drivable_area_mask.astype(np.uint8))

# 최적화된 제어 명령 구조
@dataclass
class OptimizedActionCommand:
    """최적화된 행동 명령"""
    timestamp: float
    sequence_id: int
    
    # 제어 명령
    steering_angle_rad: float
    target_speed_mps: float
    
    # 추가 제어 파라미터
    acceleration_mps2: float = 0.0
    brake_force: float = 0.0
    
    # 명령 메타데이터
    urgency_level: DataPriority = DataPriority.NORMAL
    execution_deadline_ms: float = 100.0  # 실행 마감시간
    
    # 안전성 검증
    safety_validated: bool = False
    validation_timestamp: float = 0.0

# 우선순위 기반 큐 시스템
class PriorityQueue:
    """우선순위와 타임스탬프를 고려한 큐"""
    
    def __init__(self, maxsize: int = 0, enable_metrics: bool = True):
        self.maxsize = maxsize
        self.queue = queue.PriorityQueue(maxsize=maxsize)
        self.enable_metrics = enable_metrics
        self.metrics = {
            'total_items': 0,
            'dropped_items': 0,
            'avg_wait_time': 0.0,
            'last_access_time': time.time()
        }
        self._lock = threading.RLock()
    
    def put(self, item, priority: DataPriority = DataPriority.NORMAL, timeout: float = None):
        """우선순위 기반 아이템 삽입"""
        timestamp = time.time()
        priority_value = -priority.value  # 높은 우선순위가 먼저 처리되도록
        
        try:
            # (우선순위, 타임스탬프, 아이템) 튜플로 저장
            queue_item = (priority_value, timestamp, item)
            self.queue.put(queue_item, timeout=timeout)
            
            if self.enable_metrics:
                with self._lock:
                    self.metrics['total_items'] += 1
                    self.metrics['last_access_time'] = timestamp
                    
        except queue.Full:
            if self.enable_metrics:
                with self._lock:
                    self.metrics['dropped_items'] += 1
            raise
    
    def get(self, timeout: float = None):
        """우선순위 기반 아이템 추출"""
        try:
            priority_value, put_timestamp, item = self.queue.get(timeout=timeout)
            
            if self.enable_metrics:
                current_time = time.time()
                wait_time = current_time - put_timestamp
                with self._lock:
                    # 지수 이동 평균으로 평균 대기시간 계산
                    alpha = 0.1
                    self.metrics['avg_wait_time'] = (
                        alpha * wait_time + (1 - alpha) * self.metrics['avg_wait_time']
                    )
                    self.metrics['last_access_time'] = current_time
            
            return item
            
        except (queue.Empty, _queue.Empty):
            raise
    
    def get_nowait(self):
        """비블로킹 방식으로 우선순위가 가장 높은 아이템 추출"""
        try:
            return self.queue.get_nowait()
        except (queue.Empty, _queue.Empty):
            raise queue.Empty("PriorityQueue is empty")
    
    def put_with_priority(self, item: 'PriorityQueueItem', timeout: Optional[float] = None):
        """우선순위 아이템 삽입"""
        try:
            if timeout is None:
                self.queue.put(item)
            else:
                self.queue.put(item, timeout=timeout)
                
            if self.enable_metrics:
                with self._lock:
                    self.metrics['total_items'] += 1
        except queue.Full:
            if self.enable_metrics:
                with self._lock:
                    self.metrics['dropped_items'] += 1
            raise
    
    def resize(self, new_maxsize: int):
        """큐 크기 조정 (적응적 최적화용)"""
        # PriorityQueue는 동적 크기 조정이 제한적이므로 로그만 남김
        logger.info(f"PriorityQueue resize requested to {new_maxsize} (current implementation limitations)")
        
    def task_done(self):
        """작업 완료 표시"""
        self.queue.task_done()
        
    @property
    def qsize(self):
        """현재 큐 크기"""
        return self.queue.qsize()
    
    @property 
    def utilization(self):
        """큐 사용률 (0.0 ~ 1.0)"""
        if self.maxsize <= 0:
            return 0.0
        return self.qsize / self.maxsize

# 링 버퍼 기반 고속 큐
class RingBufferQueue:
    """링 버퍼 기반 고속 데이터 큐 (센서 데이터용)"""
    
    def __init__(self, maxsize: int = 10):
        self.maxsize = maxsize
        self.buffer = [None] * maxsize
        self.head = 0
        self.tail = 0
        self.size = 0
        self._lock = threading.RLock()
        self.overwrite_count = 0
    
    def put(self, item, overwrite_old: bool = True):
        """아이템 삽입 (오래된 데이터 덮어쓰기 가능)"""
        with self._lock:
            if self.size == self.maxsize and overwrite_old:
                # 오래된 데이터 덮어쓰기
                self.head = (self.head + 1) % self.maxsize
                self.overwrite_count += 1
            elif self.size == self.maxsize:
                # 큐가 가득 찬 경우 예외 발생
                raise queue.Full("RingBufferQueue is full")
            else:
                self.size += 1
            
            self.buffer[self.tail] = item
            self.tail = (self.tail + 1) % self.maxsize
    
    def get(self):
        """가장 오래된 아이템 추출"""
        with self._lock:
            if self.size == 0:
                raise queue.Empty("RingBufferQueue is empty")
            
            item = self.buffer[self.head]
            self.buffer[self.head] = None  # 메모리 정리
            self.head = (self.head + 1) % self.maxsize
            self.size -= 1
            return item
    
    def get_latest(self):
        """가장 최신 아이템 추출"""
        with self._lock:
            if self.size == 0:
                raise queue.Empty("RingBufferQueue is empty")
            
            # 가장 최신 아이템 인덱스
            latest_idx = (self.tail - 1) % self.maxsize
            item = self.buffer[latest_idx]
            
            # 큐를 비우고 최신 아이템만 남김
            self.buffer = [None] * self.maxsize
            self.buffer[0] = item
            self.head = 0
            self.tail = 1
            self.size = 1
            
            return item
    
    def peek_latest(self):
        """최신 아이템 확인 (제거하지 않음)"""
        with self._lock:
            if self.size == 0:
                return None
            latest_idx = (self.tail - 1) % self.maxsize
            return self.buffer[latest_idx]
    
    def qsize(self) -> int:
        """현재 큐 크기"""
        with self._lock:
            return self.size
    
    def get_overwrite_count(self) -> int:
        """덮어쓰기 발생 횟수"""
        with self._lock:
            return self.overwrite_count

# 타임스탬프 동기화 매니저
class TimestampSynchronizer:
    """센서 데이터 타임스탬프 동기화 관리"""
    
    def __init__(self, tolerance_ms: float = 50.0):
        self.tolerance_ms = tolerance_ms
        self.sensor_timestamps = {}
        self._lock = threading.RLock()
        self.sync_stats = {
            'total_checks': 0,
            'sync_success': 0,
            'sync_failures': 0,
            'max_offset_ms': 0.0
        }
    
    def update_timestamp(self, sensor_name: str, timestamp: float):
        """센서 타임스탬프 업데이트"""
        with self._lock:
            self.sensor_timestamps[sensor_name] = timestamp
    
    def check_synchronization(self, required_sensors: List[str]) -> Tuple[bool, float]:
        """필요한 센서들의 동기화 상태 확인"""
        with self._lock:
            self.sync_stats['total_checks'] += 1
            
            timestamps = []
            for sensor in required_sensors:
                if sensor in self.sensor_timestamps:
                    timestamps.append(self.sensor_timestamps[sensor])
                else:
                    self.sync_stats['sync_failures'] += 1
                    return False, float('inf')
            
            if len(timestamps) < 2:
                self.sync_stats['sync_success'] += 1
                return True, 0.0
            
            max_diff_ms = (max(timestamps) - min(timestamps)) * 1000.0
            self.sync_stats['max_offset_ms'] = max(self.sync_stats['max_offset_ms'], max_diff_ms)
            
            is_synchronized = max_diff_ms <= self.tolerance_ms
            if is_synchronized:
                self.sync_stats['sync_success'] += 1
            else:
                self.sync_stats['sync_failures'] += 1
            
            return is_synchronized, max_diff_ms
    
    def get_sync_statistics(self) -> Dict[str, Any]:
        """동기화 통계 반환"""
        with self._lock:
            stats = self.sync_stats.copy()
            if stats['total_checks'] > 0:
                stats['sync_rate'] = stats['sync_success'] / stats['total_checks']
            else:
                stats['sync_rate'] = 0.0
            return stats

# 메모리 풀 관리자
class MemoryPool:
    """이미지 데이터용 메모리 풀"""
    
    def __init__(self, pool_size: int = 10, image_shape: Tuple[int, int, int] = (480, 640, 3)):
        self.pool_size = pool_size
        self.image_shape = image_shape
        self.available_buffers = queue.Queue(maxsize=pool_size)
        self.used_buffers = weakref.WeakSet()
        self._lock = threading.RLock()
        
        # 미리 버퍼 할당
        for _ in range(pool_size):
            buffer = np.zeros(image_shape, dtype=np.uint8)
            self.available_buffers.put(buffer)
    
    def get_buffer(self) -> np.ndarray:
        """사용 가능한 버퍼 획득"""
        try:
            buffer = self.available_buffers.get_nowait()
            self.used_buffers.add(buffer)
            return buffer
        except (queue.Empty, _queue.Empty):
            # 풀이 비어있으면 새 버퍼 생성
            logging.warning("MemoryPool: Creating new buffer (pool exhausted)")
            buffer = np.zeros(self.image_shape, dtype=np.uint8)
            self.used_buffers.add(buffer)
            return buffer
    
    def return_buffer(self, buffer: np.ndarray):
        """버퍼를 풀로 반환"""
        if buffer in self.used_buffers:
            try:
                self.available_buffers.put_nowait(buffer)
                self.used_buffers.discard(buffer)
            except queue.Full:
                # 풀이 가득 찬 경우 버퍼 버림
                self.used_buffers.discard(buffer)
    
    def get_stats(self) -> Dict[str, int]:
        """메모리 풀 통계"""
        return {
            'available_buffers': self.available_buffers.qsize(),
            'used_buffers': len(self.used_buffers),
            'total_capacity': self.pool_size
        }

# 기존 데이터 구조들 (호환성을 위해 유지)
class WhiteLineHsvMetrics(NamedTuple):
    timestamp: float
    total_white_pixels: int
    left_ratio: float
    mid_ratio: float
    right_ratio: float
    is_detected: bool

class YellowLineHsvMetrics(NamedTuple):
    timestamp: float
    area: float
    center_x: Optional[int]
    is_detected: bool

class DetectedObject(NamedTuple):
    id: int
    type: str
    position_3d: Tuple[float, float, float]
    bounding_box_2d: Optional[Tuple[int, int, int, int]]
    velocity: Optional[Tuple[float, float, float]]
    confidence: float
    tracked_history: Optional[List[Tuple[float, float, float]]] = None
    predicted_trajectory_short_term: Optional[List[Tuple[float, float, float]]] = None

# Essential data structures from original version
class SensorData(NamedTuple):
    timestamp: float
    lidar_data: Optional[Any]  # Raw or pre-processed LiDAR point cloud
    vision_data: Optional[Any] # Raw or pre-processed image/video frames
    gnss_data: Optional[Any]   # Raw GNSS readings
    imu_data: Optional[Any]    # Raw IMU readings

class LocalizationInfo(NamedTuple):
    timestamp: float
    position: Tuple[float, float, float]  # x, y, z in global frame
    orientation_quaternion: Tuple[float, float, float, float]  # w, x, y, z
    velocity_vector: Tuple[float, float, float] # vx, vy, vz in global frame
    covariance_matrix: Optional[Any] # Uncertainty

class PredictedTrajectory(NamedTuple):
    object_id: int
    probability: float
    path_points: List[Tuple[float, float, float]]  # Sequence of (x, y, time_offset)

class BehavioralPredictionOutput(NamedTuple):
    timestamp: float
    predicted_trajectories: List[PredictedTrajectory] # For various objects in the scene

class PlannedPath(NamedTuple):
    timestamp: float
    waypoints: List[Tuple[float, float]] # Sequence of (x,y) waypoints

class ManeuverDecision(NamedTuple):
    timestamp: float
    chosen_maneuver: str  # e.g., "LANE_KEEP", "LANE_CHANGE_LEFT", "OVERTAKE"
    target_speed_kph: float
    lead_vehicle_id: Optional[int]

class ActionCommand(NamedTuple):
    timestamp: float
    target_velocity_mps: float
    target_steering_angle_rad: float # Or curvature
    # Could also include acceleration/braking commands

class ControlActuatorCommands(NamedTuple):
    timestamp: float
    steering_command_rad: float # Target steering angle in radians
    target_velocity_mps: float  # Target velocity in meters per second

# Additional structures for compatibility
class LaneMarking(NamedTuple):
    points: List[Tuple[float, float]] # 2D points defining the lane
    type: str # e.g., solid, dashed, center
    confidence: float

class TrafficSignInfo(NamedTuple):
    type: str # e.g., stop_sign, speed_limit_60
    position_3d: Tuple[float, float, float]
    confidence: float

# 성능 모니터링을 위한 데코레이터
def monitor_performance(func):
    """함수 실행 시간을 모니터링하는 데코레이터"""
    def wrapper(*args, **kwargs):
        start_time = time.time()
        try:
            result = func(*args, **kwargs)
            execution_time = (time.time() - start_time) * 1000  # ms
            logging.debug(f"{func.__name__}: Execution time: {execution_time:.2f}ms")
            return result
        except Exception as e:
            execution_time = (time.time() - start_time) * 1000  # ms
            logging.error(f"{func.__name__}: Failed after {execution_time:.2f}ms - {e}")
            raise
    return wrapper
