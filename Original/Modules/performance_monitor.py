# 성능 모니터링 시스템
import time
import threading
import logging
import psutil
import queue
from typing import Dict, Any, List, Optional
from dataclasses import dataclass, field
from collections import defaultdict, deque
import json
import os

logger = logging.getLogger(__name__)

def performance_timing(func):
    """성능 측정 데코레이터"""
    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        execution_time = (end_time - start_time) * 1000  # ms
        logger.debug(f"{func.__name__} executed in {execution_time:.2f}ms")
        return result
    return wrapper

class PerformanceTracker:
    """성능 추적 클래스"""
    def __init__(self, name: str):
        self.name = name
        self.metrics = {}
        self.start_time = None
    
    def start_timing(self):
        """성능 측정 시작"""
        self.start_time = time.time()
    
    def end_timing(self, operation: str):
        """성능 측정 종료"""
        if self.start_time:
            duration = (time.time() - self.start_time) * 1000
            self.metrics[operation] = duration
            self.start_time = None
            return duration
        return 0
    
    def record_metric(self, metric_name: str, value: float):
        """메트릭 기록"""
        self.metrics[metric_name] = value

@dataclass
class SystemMetrics:
    """시스템 성능 메트릭"""
    timestamp: float = field(default_factory=time.time)
    
    # CPU 및 메모리 사용률
    cpu_percent: float = 0.0
    memory_percent: float = 0.0
    memory_available_mb: float = 0.0
    
    # 스레드 정보
    active_threads: int = 0
    thread_details: Dict[str, Dict] = field(default_factory=dict)
    
    # 큐 상태
    queue_metrics: Dict[str, Dict] = field(default_factory=dict)
    
    # 처리 성능
    fps_metrics: Dict[str, float] = field(default_factory=dict)
    latency_metrics: Dict[str, float] = field(default_factory=dict)
    
    # 에러 및 경고
    error_count: int = 0
    warning_count: int = 0
    dropped_frames: int = 0

@dataclass
class PerformanceAlert:
    """성능 경고"""
    timestamp: float
    severity: str  # "INFO", "WARNING", "ERROR", "CRITICAL"
    component: str  # 문제가 발생한 컴포넌트
    message: str
    metric_value: float
    threshold: float
    recommendation: str

class PerformanceMonitor:
    """실시간 성능 모니터링 시스템"""
    
    def __init__(self, config: Dict[str, Any] = None):
        self.config = config or {}
        self.monitoring_enabled = self.config.get("enable_monitoring", True)
        self.monitoring_interval = self.config.get("monitoring_interval_sec", 1.0)
        self.alert_thresholds = self.config.get("alert_thresholds", {
            "cpu_percent_high": 80.0,
            "memory_percent_high": 85.0,
            "queue_size_high": 10,
            "fps_low": 10.0,
            "latency_high_ms": 100.0
        })
        
        # 모니터링 상태
        self._running = False
        self._monitor_thread = None
        
        # 메트릭 저장
        self.metrics_history = deque(maxlen=300)  # 5분간 데이터 (1초 간격)
        self.alerts_history = deque(maxlen=100)
        
        # 등록된 모듈들
        self.registered_modules = {}
        self.registered_queues = {}
        
        # 통계
        self.performance_stats = {
            "total_alerts": 0,
            "critical_alerts": 0,
            "avg_cpu_usage": 0.0,
            "avg_memory_usage": 0.0,
            "max_latency_ms": 0.0,
            "min_fps": float('inf')
        }
        
        self._lock = threading.RLock()
        self.metrics_lock = threading.RLock()
        self.current_metrics = {}
        
        logger.info("PerformanceMonitor: Initialized")
    
    def register_module(self, module_name: str, module_instance):
        """모듈 등록 (성능 메트릭 수집용)"""
        with self._lock:
            self.registered_modules[module_name] = module_instance
            logger.debug(f"PerformanceMonitor: Registered module '{module_name}'")
    
    def register_queue(self, queue_name: str, queue_instance):
        """큐 등록 (큐 상태 모니터링용)"""
        with self._lock:
            self.registered_queues[queue_name] = queue_instance
            logger.debug(f"PerformanceMonitor: Registered queue '{queue_name}'")
    
    def _collect_system_metrics(self) -> SystemMetrics:
        """시스템 메트릭 수집"""
        try:
            # 기본 시스템 정보
            cpu_percent = psutil.cpu_percent(interval=0.1)
            memory_info = psutil.virtual_memory()
            
            # 스레드 정보
            current_process = psutil.Process()
            active_threads = current_process.num_threads()
            
            thread_details = {}
            for thread in threading.enumerate():
                thread_details[thread.name] = {
                    "alive": thread.is_alive(),
                    "daemon": thread.daemon,
                    "ident": thread.ident
                }
            
            # 큐 상태 수집
            queue_metrics = {}
            with self._lock:
                for queue_name, queue_obj in self.registered_queues.items():
                    try:
                        if hasattr(queue_obj, 'qsize'):
                            queue_size = queue_obj.qsize()
                        elif hasattr(queue_obj, 'size'):
                            queue_size = queue_obj.size  # RingBufferQueue
                        else:
                            queue_size = 0
                        
                        queue_metrics[queue_name] = {
                            "size": queue_size,
                            "type": type(queue_obj).__name__
                        }
                        
                        # 우선순위 큐 메트릭
                        if hasattr(queue_obj, 'get_metrics'):
                            queue_metrics[queue_name].update(queue_obj.get_metrics())
                        
                        # 링 버퍼 큐 메트릭
                        if hasattr(queue_obj, 'get_overwrite_count'):
                            queue_metrics[queue_name]['overwrite_count'] = queue_obj.get_overwrite_count()
                            
                    except Exception as e:
                        logger.debug(f"Error collecting metrics for queue '{queue_name}': {e}")
                        queue_metrics[queue_name] = {"error": str(e)}
            
            # 모듈별 성능 메트릭 수집
            fps_metrics = {}
            latency_metrics = {}
            error_count = 0
            warning_count = 0
            dropped_frames = 0
            
            with self._lock:
                for module_name, module_obj in self.registered_modules.items():
                    try:
                        if hasattr(module_obj, 'get_performance_metrics'):
                            metrics = module_obj.get_performance_metrics()
                            if 'last_fps' in metrics:
                                fps_metrics[module_name] = metrics['last_fps']
                            if 'avg_processing_time_ms' in metrics:
                                latency_metrics[module_name] = metrics['avg_processing_time_ms']
                            if 'frames_dropped' in metrics:
                                dropped_frames += metrics['frames_dropped']
                    except Exception as e:
                        logger.debug(f"Error collecting metrics for module '{module_name}': {e}")
            
            return SystemMetrics(
                cpu_percent=cpu_percent,
                memory_percent=memory_info.percent,
                memory_available_mb=memory_info.available / (1024 * 1024),
                active_threads=active_threads,
                thread_details=thread_details,
                queue_metrics=queue_metrics,
                fps_metrics=fps_metrics,
                latency_metrics=latency_metrics,
                error_count=error_count,
                warning_count=warning_count,
                dropped_frames=dropped_frames
            )
            
        except Exception as e:
            logger.error(f"PerformanceMonitor: Error collecting system metrics: {e}")
            return SystemMetrics()
    
    def _analyze_metrics(self, metrics: SystemMetrics) -> List[PerformanceAlert]:
        """메트릭 분석 및 경고 생성"""
        alerts = []
        
        # CPU 사용률 검사
        if metrics.cpu_percent > self.alert_thresholds["cpu_percent_high"]:
            alerts.append(PerformanceAlert(
                timestamp=metrics.timestamp,
                severity="WARNING" if metrics.cpu_percent < 90 else "ERROR",
                component="System",
                message=f"High CPU usage: {metrics.cpu_percent:.1f}%",
                metric_value=metrics.cpu_percent,
                threshold=self.alert_thresholds["cpu_percent_high"],
                recommendation="Consider reducing processing load or optimizing algorithms"
            ))
        
        # 메모리 사용률 검사
        if metrics.memory_percent > self.alert_thresholds["memory_percent_high"]:
            alerts.append(PerformanceAlert(
                timestamp=metrics.timestamp,
                severity="WARNING" if metrics.memory_percent < 95 else "CRITICAL",
                component="System",
                message=f"High memory usage: {metrics.memory_percent:.1f}%",
                metric_value=metrics.memory_percent,
                threshold=self.alert_thresholds["memory_percent_high"],
                recommendation="Check for memory leaks or reduce memory pool sizes"
            ))
        
        # 큐 크기 검사
        for queue_name, queue_info in metrics.queue_metrics.items():
            queue_size = queue_info.get("size", 0)
            if queue_size > self.alert_thresholds["queue_size_high"]:
                alerts.append(PerformanceAlert(
                    timestamp=metrics.timestamp,
                    severity="WARNING",
                    component=f"Queue_{queue_name}",
                    message=f"Queue '{queue_name}' is nearly full: {queue_size} items",
                    metric_value=queue_size,
                    threshold=self.alert_thresholds["queue_size_high"],
                    recommendation="Increase processing speed or queue size"
                ))
        
        # FPS 성능 검사
        for module_name, fps in metrics.fps_metrics.items():
            if fps < self.alert_thresholds["fps_low"]:
                alerts.append(PerformanceAlert(
                    timestamp=metrics.timestamp,
                    severity="WARNING",
                    component=f"Module_{module_name}",
                    message=f"Low FPS in '{module_name}': {fps:.1f}",
                    metric_value=fps,
                    threshold=self.alert_thresholds["fps_low"],
                    recommendation="Optimize processing algorithms or reduce input rate"
                ))
        
        # 지연시간 검사
        for module_name, latency in metrics.latency_metrics.items():
            if latency > self.alert_thresholds["latency_high_ms"]:
                alerts.append(PerformanceAlert(
                    timestamp=metrics.timestamp,
                    severity="WARNING",
                    component=f"Module_{module_name}",
                    message=f"High latency in '{module_name}': {latency:.1f}ms",
                    metric_value=latency,
                    threshold=self.alert_thresholds["latency_high_ms"],
                    recommendation="Profile and optimize bottleneck operations"
                ))
        
        return alerts
    
    def _update_statistics(self, metrics: SystemMetrics, alerts: List[PerformanceAlert]):
        """통계 업데이트"""
        with self._lock:
            self.performance_stats["total_alerts"] += len(alerts)
            self.performance_stats["critical_alerts"] += sum(1 for a in alerts if a.severity == "CRITICAL")
            
            # 지수 이동 평균으로 평균값 업데이트
            alpha = 0.1
            self.performance_stats["avg_cpu_usage"] = (
                alpha * metrics.cpu_percent + (1 - alpha) * self.performance_stats["avg_cpu_usage"]
            )
            self.performance_stats["avg_memory_usage"] = (
                alpha * metrics.memory_percent + (1 - alpha) * self.performance_stats["avg_memory_usage"]
            )
            
            # 최대/최소값 업데이트
            max_latency = max(metrics.latency_metrics.values()) if metrics.latency_metrics else 0
            self.performance_stats["max_latency_ms"] = max(self.performance_stats["max_latency_ms"], max_latency)
            
            min_fps = min(metrics.fps_metrics.values()) if metrics.fps_metrics else float('inf')
            if min_fps != float('inf'):
                self.performance_stats["min_fps"] = min(self.performance_stats["min_fps"], min_fps)
    
    def _monitoring_loop(self):
        """모니터링 메인 루프"""
        logger.info("PerformanceMonitor: Monitoring loop started")
        
        while self._running:
            try:
                start_time = time.time()
                
                # 메트릭 수집
                metrics = self._collect_system_metrics()
                
                # 메트릭 분석
                alerts = self._analyze_metrics(metrics)
                
                # 결과 저장
                with self._lock:
                    self.metrics_history.append(metrics)
                    self.alerts_history.extend(alerts)
                
                # 통계 업데이트
                self._update_statistics(metrics, alerts)
                
                # 경고 로깅
                for alert in alerts:
                    if alert.severity == "CRITICAL":
                        logger.critical(f"PERFORMANCE CRITICAL: {alert.message}")
                    elif alert.severity == "ERROR":
                        logger.error(f"PERFORMANCE ERROR: {alert.message}")
                    elif alert.severity == "WARNING":
                        logger.warning(f"PERFORMANCE WARNING: {alert.message}")
                
                # 모니터링 간격 조절
                elapsed = time.time() - start_time
                sleep_time = max(0, self.monitoring_interval - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    logger.warning(f"PerformanceMonitor: Monitoring loop is running slow (took {elapsed:.2f}s)")
                
            except Exception as e:
                logger.error(f"PerformanceMonitor: Error in monitoring loop: {e}")
                time.sleep(1.0)  # 에러 시 짧은 대기
        
        logger.info("PerformanceMonitor: Monitoring loop stopped")
    
    def start(self):
        """성능 모니터링 시작"""
        if not self.monitoring_enabled:
            logger.info("PerformanceMonitor: Monitoring disabled by configuration")
            return
        
        if self._running:
            logger.warning("PerformanceMonitor: Already running")
            return
        
        self._running = True
        self._monitor_thread = threading.Thread(target=self._monitoring_loop, name="PerformanceMonitor")
        self._monitor_thread.daemon = True
        self._monitor_thread.start()
        
        logger.info("PerformanceMonitor: Started")
    
    def start_monitoring(self):
        """성능 모니터링 시작 (별칭)"""
        return self.start()
    
    def stop(self):
        """성능 모니터링 중지"""
        if not self._running:
            return
        
        logger.info("PerformanceMonitor: Stopping...")
        self._running = False
        
        if self._monitor_thread and self._monitor_thread.is_alive():
            self._monitor_thread.join(timeout=5.0)
            if self._monitor_thread.is_alive():
                logger.warning("PerformanceMonitor: Thread did not terminate gracefully")
        
        # 최종 보고서 생성
        self._generate_final_report()
        
        logger.info("PerformanceMonitor: Stopped")
    
    def stop_monitoring(self):
        """성능 모니터링 중지 (별칭)"""
        return self.stop()
    
    def get_current_metrics(self) -> Optional[SystemMetrics]:
        """현재 메트릭 반환"""
        with self._lock:
            return self.metrics_history[-1] if self.metrics_history else None
    
    def get_recent_alerts(self, count: int = 10) -> List[PerformanceAlert]:
        """최근 경고 반환"""
        with self._lock:
            return list(self.alerts_history)[-count:]
    
    def get_performance_statistics(self) -> Dict[str, Any]:
        """성능 통계 반환"""
        with self._lock:
            stats = self.performance_stats.copy()
            stats.update({
                "monitoring_duration_sec": len(self.metrics_history) * self.monitoring_interval,
                "total_metrics_collected": len(self.metrics_history),
                "active_queues": len(self.registered_queues),
                "active_modules": len(self.registered_modules)
            })
            return stats
    
    def _generate_final_report(self):
        """최종 성능 보고서 생성"""
        try:
            report = {
                "timestamp": time.time(),
                "statistics": self.get_performance_statistics(),
                "recent_alerts": [
                    {
                        "timestamp": alert.timestamp,
                        "severity": alert.severity,
                        "component": alert.component,
                        "message": alert.message,
                        "metric_value": alert.metric_value,
                        "threshold": alert.threshold
                    }
                    for alert in self.get_recent_alerts(50)
                ],
                "recommendations": self._generate_recommendations()
            }
            
            # 보고서 파일 저장
            report_path = "/tmp/performance_report.json"
            with open(report_path, 'w') as f:
                json.dump(report, f, indent=2)
            
            logger.info(f"PerformanceMonitor: Final report saved to {report_path}")
            
        except Exception as e:
            logger.error(f"PerformanceMonitor: Error generating final report: {e}")
    
    def _generate_recommendations(self) -> List[str]:
        """성능 개선 권장사항 생성"""
        recommendations = []
        stats = self.performance_stats
        
        if stats["avg_cpu_usage"] > 70:
            recommendations.append("Consider optimizing CPU-intensive algorithms or distributing load")
        
        if stats["avg_memory_usage"] > 80:
            recommendations.append("Review memory usage patterns and implement memory pool optimization")
        
        if stats["max_latency_ms"] > 50:
            recommendations.append("Profile and optimize high-latency operations")
        
        if stats["min_fps"] < 15:
            recommendations.append("Increase processing efficiency or reduce input data rate")
        
        if stats["critical_alerts"] > 0:
            recommendations.append("Address critical performance issues immediately")
        
        return recommendations
    
    def get_current_metrics(self) -> Dict[str, Any]:
        """현재 시스템 메트릭을 반환"""
        with self.metrics_lock:
            return {
                'cpu_percent': self.current_metrics.get('cpu_percent', 0.0),
                'memory_percent': self.current_metrics.get('memory_percent', 0.0),
                'queue_metrics': self.current_metrics.get('queue_metrics', {}),
                'processing_times': self.current_metrics.get('processing_times', {}),
                'timestamp': time.time()
            }

    def register_queue(self, name: str, queue_obj):
        """큐를 등록하여 모니터링"""
        with self.metrics_lock:
            self.registered_queues[name] = queue_obj
            logger.debug(f"PerformanceMonitor: Registered queue '{name}'")

    def register_module(self, name: str, module_obj):
        """모듈을 등록하여 모니터링"""
        with self.metrics_lock:
            self.registered_modules[name] = module_obj
            logger.debug(f"PerformanceMonitor: Registered module '{name}'")
