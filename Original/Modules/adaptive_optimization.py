"""
적응적 시스템 최적화 모듈
실시간 성능 메트릭을 기반으로 시스템 파라미터를 자동 조정
"""

import threading
import time
import logging
from typing import Dict, Any, Optional
from .performance_monitor import PerformanceMonitor
from .optimized_data_structures import PriorityQueue, RingBufferQueue

logger = logging.getLogger(__name__)

class AdaptiveOptimizer:
    """
    실시간 성능 모니터링을 바탕으로 시스템 파라미터를 자동 조정하는 클래스
    """
    
    def __init__(self, performance_monitor: PerformanceMonitor, config: Dict[str, Any]):
        self.performance_monitor = performance_monitor
        self.config = config
        self.optimization_interval = config.get("optimization_interval_sec", 5.0)
        self.running = False
        self.thread = None
        
        # 최적화 임계값들
        self.cpu_high_threshold = config.get("cpu_high_threshold", 85.0)
        self.cpu_low_threshold = config.get("cpu_low_threshold", 60.0)
        self.memory_high_threshold = config.get("memory_high_threshold", 80.0)
        self.queue_full_threshold = config.get("queue_full_threshold", 0.8)
        self.latency_high_threshold = config.get("latency_high_threshold_ms", 50.0)
        
        # 최적화 상태 추적
        self.optimization_history = []
        self.last_optimization_time = 0
        
        # 관리할 큐와 모듈들
        self.managed_queues = {}
        self.managed_modules = {}
        
        logger.info(f"AdaptiveOptimizer: Initialized with optimization interval {self.optimization_interval}s")
    
    def register_queue(self, name: str, queue_obj):
        """최적화 관리할 큐를 등록"""
        self.managed_queues[name] = queue_obj
        logger.debug(f"AdaptiveOptimizer: Registered queue '{name}'")
    
    def register_module(self, name: str, module_obj):
        """최적화 관리할 모듈을 등록"""
        self.managed_modules[name] = module_obj
        logger.debug(f"AdaptiveOptimizer: Registered module '{name}'")
    
    def start(self):
        """적응적 최적화 시작"""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._optimization_loop, name="AdaptiveOptimizer")
            self.thread.daemon = True
            self.thread.start()
            logger.info("AdaptiveOptimizer: Started optimization loop")
    
    def stop(self):
        """적응적 최적화 정지"""
        if self.running:
            self.running = False
            if self.thread:
                self.thread.join(timeout=2.0)
            logger.info("AdaptiveOptimizer: Stopped optimization loop")
    
    def _optimization_loop(self):
        """메인 최적화 루프"""
        while self.running:
            try:
                current_time = time.time()
                
                # 성능 메트릭 수집
                metrics = self.performance_monitor.get_current_metrics()
                
                # 최적화 결정 및 실행
                optimizations = self._analyze_and_optimize(metrics)
                
                if optimizations:
                    self._apply_optimizations(optimizations)
                    self.optimization_history.append({
                        'timestamp': current_time,
                        'metrics': metrics,
                        'optimizations': optimizations
                    })
                    self.last_optimization_time = current_time
                
                # 다음 최적화까지 대기
                time.sleep(self.optimization_interval)
                
            except Exception as e:
                logger.error(f"AdaptiveOptimizer: Error in optimization loop: {e}", exc_info=True)
                time.sleep(1.0)
    
    def _analyze_and_optimize(self, metrics: Dict[str, Any]) -> Dict[str, Any]:
        """성능 메트릭을 분석하고 최적화 방안을 결정"""
        optimizations = {}
        
        # CPU 사용률 기반 최적화
        cpu_usage = metrics.get('cpu_percent', 0.0)
        if cpu_usage > self.cpu_high_threshold:
            optimizations['cpu_optimization'] = self._get_cpu_optimization(cpu_usage)
        
        # 메모리 사용률 기반 최적화
        memory_usage = metrics.get('memory_percent', 0.0)
        if memory_usage > self.memory_high_threshold:
            optimizations['memory_optimization'] = self._get_memory_optimization(memory_usage)
        
        # 큐 상태 기반 최적화
        queue_optimizations = self._get_queue_optimizations(metrics.get('queue_metrics', {}))
        if queue_optimizations:
            optimizations['queue_optimization'] = queue_optimizations
        
        # 레이턴시 기반 최적화
        latency_optimizations = self._get_latency_optimizations(metrics)
        if latency_optimizations:
            optimizations['latency_optimization'] = latency_optimizations
        
        return optimizations
    
    def _get_cpu_optimization(self, cpu_usage: float) -> Dict[str, Any]:
        """CPU 사용률이 높을 때의 최적화 방안"""
        optimizations = {}
        
        if cpu_usage > 90.0:
            # 매우 높은 CPU 사용률 - 적극적 최적화
            optimizations['reduce_processing_frequency'] = True
            optimizations['enable_frame_skip'] = True
            optimizations['reduce_queue_sizes'] = True
        elif cpu_usage > self.cpu_high_threshold:
            # 높은 CPU 사용률 - 보수적 최적화
            optimizations['reduce_processing_frequency'] = True
            optimizations['optimize_algorithms'] = True
        
        return optimizations
    
    def _get_memory_optimization(self, memory_usage: float) -> Dict[str, Any]:
        """메모리 사용률이 높을 때의 최적화 방안"""
        optimizations = {}
        
        if memory_usage > 90.0:
            # 매우 높은 메모리 사용률
            optimizations['force_garbage_collection'] = True
            optimizations['reduce_buffer_sizes'] = True
            optimizations['clear_old_data'] = True
        elif memory_usage > self.memory_high_threshold:
            # 높은 메모리 사용률
            optimizations['optimize_memory_pools'] = True
            optimizations['reduce_cache_sizes'] = True
        
        return optimizations
    
    def _get_queue_optimizations(self, queue_metrics: Dict[str, Any]) -> Dict[str, Any]:
        """큐 상태 기반 최적화 방안"""
        optimizations = {}
        
        for queue_name, metrics in queue_metrics.items():
            utilization = metrics.get('utilization', 0.0)
            avg_wait_time = metrics.get('avg_wait_time_ms', 0.0)
            
            if utilization > self.queue_full_threshold:
                # 큐가 거의 가득 참
                if queue_name not in optimizations:
                    optimizations[queue_name] = {}
                optimizations[queue_name]['increase_capacity'] = True
                optimizations[queue_name]['increase_processing_speed'] = True
            
            if avg_wait_time > self.latency_high_threshold:
                # 큐 대기 시간이 너무 김
                if queue_name not in optimizations:
                    optimizations[queue_name] = {}
                optimizations[queue_name]['prioritize_processing'] = True
                optimizations[queue_name]['reduce_input_rate'] = True
        
        return optimizations
    
    def _get_latency_optimizations(self, metrics: Dict[str, Any]) -> Dict[str, Any]:
        """레이턴시 기반 최적화 방안"""
        optimizations = {}
        
        processing_times = metrics.get('processing_times', {})
        for module_name, times in processing_times.items():
            avg_time = times.get('avg_ms', 0.0)
            max_time = times.get('max_ms', 0.0)
            
            if avg_time > self.latency_high_threshold:
                if module_name not in optimizations:
                    optimizations[module_name] = {}
                optimizations[module_name]['optimize_algorithm'] = True
                
            if max_time > self.latency_high_threshold * 2:
                if module_name not in optimizations:
                    optimizations[module_name] = {}
                optimizations[module_name]['add_timeout_limits'] = True
        
        return optimizations
    
    def _apply_optimizations(self, optimizations: Dict[str, Any]):
        """최적화 방안을 실제로 적용"""
        logger.info(f"AdaptiveOptimizer: Applying optimizations: {list(optimizations.keys())}")
        
        # CPU 최적화 적용
        if 'cpu_optimization' in optimizations:
            self._apply_cpu_optimizations(optimizations['cpu_optimization'])
        
        # 메모리 최적화 적용
        if 'memory_optimization' in optimizations:
            self._apply_memory_optimizations(optimizations['memory_optimization'])
        
        # 큐 최적화 적용
        if 'queue_optimization' in optimizations:
            self._apply_queue_optimizations(optimizations['queue_optimization'])
        
        # 레이턴시 최적화 적용
        if 'latency_optimization' in optimizations:
            self._apply_latency_optimizations(optimizations['latency_optimization'])
    
    def _apply_cpu_optimizations(self, optimizations: Dict[str, Any]):
        """CPU 최적화 적용"""
        if optimizations.get('reduce_processing_frequency'):
            logger.info("AdaptiveOptimizer: Reducing processing frequency to lower CPU usage")
            # 센서 입력 주기를 늘림
            for module_name, module in self.managed_modules.items():
                if hasattr(module, 'reduce_processing_frequency'):
                    module.reduce_processing_frequency()
        
        if optimizations.get('enable_frame_skip'):
            logger.info("AdaptiveOptimizer: Enabling frame skipping for perception module")
            perception_module = self.managed_modules.get('perception_module')
            if perception_module and hasattr(perception_module, 'enable_frame_skip'):
                perception_module.enable_frame_skip(True)
        
        if optimizations.get('reduce_queue_sizes'):
            logger.info("AdaptiveOptimizer: Reducing queue sizes to decrease memory pressure")
            for queue_name, queue in self.managed_queues.items():
                if hasattr(queue, 'resize') and hasattr(queue, 'maxsize'):
                    new_size = max(2, int(queue.maxsize * 0.8))
                    queue.resize(new_size)
    
    def _apply_memory_optimizations(self, optimizations: Dict[str, Any]):
        """메모리 최적화 적용"""
        if optimizations.get('force_garbage_collection'):
            import gc
            logger.info("AdaptiveOptimizer: Forcing garbage collection")
            gc.collect()
        
        if optimizations.get('optimize_memory_pools'):
            logger.info("AdaptiveOptimizer: Optimizing memory pools")
            sensor_manager = self.managed_modules.get('sensor_manager')
            if sensor_manager and hasattr(sensor_manager, 'optimize_memory_pools'):
                sensor_manager.optimize_memory_pools()
    
    def _apply_queue_optimizations(self, optimizations: Dict[str, Any]):
        """큐 최적화 적용"""
        for queue_name, queue_opts in optimizations.items():
            queue = self.managed_queues.get(queue_name)
            if not queue:
                continue
            
            if queue_opts.get('increase_capacity'):
                if hasattr(queue, 'resize') and hasattr(queue, 'maxsize'):
                    new_size = min(queue.maxsize * 2, 20)  # 최대 20까지
                    queue.resize(new_size)
                    logger.info(f"AdaptiveOptimizer: Increased capacity of {queue_name} to {new_size}")
            
            if queue_opts.get('prioritize_processing'):
                # 해당 큐를 처리하는 모듈의 처리 우선순위를 높임
                logger.info(f"AdaptiveOptimizer: Prioritizing processing for {queue_name}")
    
    def _apply_latency_optimizations(self, optimizations: Dict[str, Any]):
        """레이턴시 최적화 적용"""
        for module_name, latency_opts in optimizations.items():
            module = self.managed_modules.get(module_name)
            if not module:
                continue
            
            if latency_opts.get('optimize_algorithm'):
                if hasattr(module, 'enable_fast_mode'):
                    module.enable_fast_mode(True)
                    logger.info(f"AdaptiveOptimizer: Enabled fast mode for {module_name}")
            
            if latency_opts.get('add_timeout_limits'):
                if hasattr(module, 'set_timeout_limit'):
                    module.set_timeout_limit(self.latency_high_threshold / 1000.0)
                    logger.info(f"AdaptiveOptimizer: Added timeout limit for {module_name}")
    
    def get_optimization_report(self) -> str:
        """최적화 보고서 생성"""
        if not self.optimization_history:
            return "No optimizations performed yet."
        
        recent_optimizations = self.optimization_history[-5:]  # 최근 5개
        
        report = "=== Adaptive Optimization Report ===\n"
        report += f"Total optimizations performed: {len(self.optimization_history)}\n"
        report += f"Last optimization: {time.ctime(self.last_optimization_time)}\n\n"
        
        report += "Recent optimizations:\n"
        for i, opt in enumerate(recent_optimizations, 1):
            report += f"{i}. {time.ctime(opt['timestamp'])}\n"
            report += f"   CPU: {opt['metrics'].get('cpu_percent', 0):.1f}%, "
            report += f"Memory: {opt['metrics'].get('memory_percent', 0):.1f}%\n"
            report += f"   Applied: {list(opt['optimizations'].keys())}\n\n"
        
        return report
