#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
최적화된 자율주행 시스템 성능 테스트 스크립트
"""

import sys
import os
import time
import logging
import threading
from datetime import datetime

# 경로 설정
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from Modules.main_system import MainSystem, load_dummy_config
from Modules.performance_monitor import PerformanceMonitor
from Modules.adaptive_optimization import AdaptiveOptimizer

def setup_logging():
    """로깅 설정"""
    log_file = f"/home/xytron/xycar_ws/src/kookmin/driver/Original/performance_test_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
    
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler(log_file, encoding='utf-8'),
            logging.StreamHandler()
        ]
    )
    
    return logging.getLogger(__name__)

def run_performance_test(duration_seconds=60):
    """성능 테스트 실행"""
    logger = setup_logging()
    logger.info("=== 최적화된 자율주행 시스템 성능 테스트 시작 ===")
    
    try:
        # 설정 로드 및 최적화 파라미터 추가
        config = load_dummy_config()
        
        # 성능 테스트를 위한 최적화된 설정
        config.update({
            "sensor_input_config": {
                "publish_rate_hz": 30,  # 높은 센서 데이터 주기
                "enable_performance_monitoring": True,
                "memory_pool_size": 20,
                "sync_tolerance_ms": 20.0
            },
            "performance_monitor_config": {
                "monitor_interval_sec": 1.0,  # 더 빈번한 모니터링
                "cpu_high_threshold": 80.0,
                "memory_high_threshold": 75.0,
                "latency_high_ms": 40.0
            },
            "adaptive_optimizer_config": {
                "optimization_interval_sec": 3.0,  # 더 빈번한 최적화
                "cpu_high_threshold": 80.0,
                "memory_high_threshold": 75.0,
                "queue_full_threshold": 0.7,
                "latency_high_threshold_ms": 40.0
            }
        })
        
        # MainSystem 초기화 (ROS 없이 테스트 모드)
        config["test_mode"] = True  # ROS 의존성 제거
        config["ros_bridge"] = None
        config["ros_motor_publisher"] = None
        config["ros_motor_msg_template"] = None
        
        logger.info("MainSystem 초기화 중...")
        system = MainSystem(config=config)
        
        # 시스템 시작
        logger.info("시스템 시작...")
        system.start()
        
        # 성능 테스트 실행
        logger.info(f"{duration_seconds}초 동안 성능 테스트 실행...")
        test_start_time = time.time()
        
        # 중간 보고서 생성을 위한 스레드
        def generate_interim_reports():
            while time.time() - test_start_time < duration_seconds:
                time.sleep(10)  # 10초마다 보고서 생성
                elapsed = time.time() - test_start_time
                logger.info(f"=== {elapsed:.1f}초 경과 - 중간 성능 보고서 ===")
                
                try:
                    report = system.get_performance_report()
                    logger.info(f"성능 보고서:\n{report}")
                except Exception as e:
                    logger.error(f"중간 보고서 생성 실패: {e}")
        
        # 보고서 스레드 시작
        report_thread = threading.Thread(target=generate_interim_reports)
        report_thread.daemon = True
        report_thread.start()
        
        # 테스트 실행
        while time.time() - test_start_time < duration_seconds:
            time.sleep(1.0)
            
            # 시스템 상태 체크
            if not hasattr(system, 'performance_monitor') or not system.performance_monitor:
                logger.warning("성능 모니터가 비활성 상태입니다.")
                break
        
        # 최종 보고서 생성
        logger.info("=== 최종 성능 보고서 생성 ===")
        try:
            final_report = system.get_performance_report()
            logger.info(f"최종 성능 보고서:\n{final_report}")
            
            if hasattr(system, 'adaptive_optimizer'):
                optimization_report = system.adaptive_optimizer.get_optimization_report()
                logger.info(f"적응적 최적화 보고서:\n{optimization_report}")
        except Exception as e:
            logger.error(f"최종 보고서 생성 실패: {e}")
        
        # 시스템 정지
        logger.info("시스템 정지 중...")
        system.stop()
        
        logger.info("=== 성능 테스트 완료 ===")
        
    except Exception as e:
        logger.error(f"성능 테스트 중 오류 발생: {e}", exc_info=True)
    
    finally:
        logger.info("성능 테스트 종료")

def run_stress_test():
    """스트레스 테스트 - 높은 부하 상황 시뮬레이션"""
    logger = setup_logging()
    logger.info("=== 스트레스 테스트 시작 ===")
    
    try:
        config = load_dummy_config()
        
        # 스트레스 테스트를 위한 고부하 설정
        config.update({
            "sensor_input_config": {
                "publish_rate_hz": 50,  # 매우 높은 센서 데이터 주기
                "memory_pool_size": 10,  # 작은 메모리 풀로 압박 상황 시뮬레이션
                "sync_tolerance_ms": 10.0  # 엄격한 동기화
            },
            "adaptive_optimizer_config": {
                "optimization_interval_sec": 1.0,  # 매우 빈번한 최적화
                "cpu_high_threshold": 70.0,  # 낮은 임계값으로 더 빈번한 최적화 유발
                "memory_high_threshold": 60.0,
                "queue_full_threshold": 0.5,
                "latency_high_threshold_ms": 20.0
            }
        })
        
        config["test_mode"] = True
        config["ros_bridge"] = None
        config["ros_motor_publisher"] = None
        config["ros_motor_msg_template"] = None
        
        system = MainSystem(config=config)
        system.start()
        
        # 30초 스트레스 테스트
        stress_duration = 30
        logger.info(f"{stress_duration}초 동안 스트레스 테스트 실행...")
        
        start_time = time.time()
        while time.time() - start_time < stress_duration:
            time.sleep(2.0)
            
            # 현재 성능 메트릭 출력
            try:
                if hasattr(system, 'performance_monitor'):
                    metrics = system.performance_monitor.get_current_metrics()
                    cpu_usage = metrics.get('cpu_percent', 0)
                    memory_usage = metrics.get('memory_percent', 0)
                    logger.info(f"현재 상태 - CPU: {cpu_usage:.1f}%, Memory: {memory_usage:.1f}%")
            except Exception as e:
                logger.error(f"메트릭 수집 실패: {e}")
        
        # 최종 보고서
        final_report = system.get_performance_report()
        optimization_report = system.adaptive_optimizer.get_optimization_report()
        
        logger.info(f"스트레스 테스트 최종 보고서:\n{final_report}")
        logger.info(f"적응적 최적화 보고서:\n{optimization_report}")
        
        system.stop()
        logger.info("=== 스트레스 테스트 완료 ===")
        
    except Exception as e:
        logger.error(f"스트레스 테스트 중 오류: {e}", exc_info=True)

if __name__ == "__main__":
    print("최적화된 자율주행 시스템 성능 테스트")
    print("1. 일반 성능 테스트 (60초)")
    print("2. 스트레스 테스트 (30초)")
    print("3. 둘 다 실행")
    
    choice = input("선택하세요 (1/2/3): ").strip()
    
    if choice == "1":
        run_performance_test(60)
    elif choice == "2":
        run_stress_test()
    elif choice == "3":
        run_performance_test(60)
        time.sleep(5)  # 잠시 대기
        run_stress_test()
    else:
        print("잘못된 선택입니다.")
