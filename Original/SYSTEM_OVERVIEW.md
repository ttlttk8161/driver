# 자율주행 시스템 전체 프로세스 분석

## 시스템 아키텍처

### 1. 메인 실행 흐름
```
track_drive.py → MainSystem → 모듈 초기화 및 시작 → 실시간 처리 루프
```

### 2. 모듈 구성 및 데이터 흐름
```
센서 입력 → 인지 → 위치인식/예측 → 계획 → 제어 → 차량 출력
     ↓        ↓         ↓           ↓      ↓
[SensorInput] → [Perception] → [Localization] → [Planning] → [Control]
                      ↓              ↓            ↓
                 [Prediction] ────────┘            ↓
                                               [Vehicle Interface]
```

## 주요 모듈별 역할

### 1. SensorInputManager (OptimizedSensorInputManager)
- **역할**: ROS 토픽에서 카메라/라이다 데이터 수신
- **출력**: sensor_to_perception_queue (RingBufferQueue)
- **특징**: 메모리 풀과 타임스탬프 동기화

### 2. PerceptionModule  
- **입력**: sensor_to_perception_queue
- **역할**: HSV 차선 검출, 객체 인식
- **출력**: 
  - perception_to_localization_queue (PriorityQueue)
  - perception_to_prediction_queue (PriorityQueue)  
  - perception_to_planning_queue (PriorityQueue)

### 3. LocalizationModule
- **입력**: perception_to_localization_queue
- **역할**: 차량 위치 추정 (플레이스홀더/GPS-IMU 융합)
- **출력**:
  - localization_to_prediction_queue (PriorityQueue)
  - localization_to_planning_queue (PriorityQueue)

### 4. PredictionModule
- **입력**: 
  - perception_to_prediction_queue
  - localization_to_prediction_queue
- **역할**: 칼만 필터 기반 객체 궤적 예측
- **출력**: prediction_to_planning_queue (PriorityQueue)

### 5. PlanningModule
- **입력**:
  - perception_to_planning_queue
  - localization_to_planning_queue  
  - prediction_to_planning_queue
- **역할**: 경로 계획 및 행동 결정
- **구성요소**:
  - PathPlannerComponent: 경로 생성
  - DecisionMakerComponent: 의사결정
  - ActionPlannerComponent: HSV 차선 추종
- **출력**: planning_to_control_queue (PriorityQueue)

### 6. ControlModule
- **입력**: planning_to_control_queue
- **역할**: PID 제어를 통한 조향/속도 제어
- **출력**: VehicleInterface → ROS 모터 토픽

## 큐 기반 통신 구조

### 큐 타입별 특성
- **RingBufferQueue**: 센서 데이터용 (고속, 메모리 효율)
- **PriorityQueue**: 처리 결과용 (우선순위, 메트릭 지원)

### 큐 연결 맵
```
sensor_to_perception_queue: SensorInput → Perception
perception_to_localization_queue: Perception → Localization  
perception_to_prediction_queue: Perception → Prediction
perception_to_planning_queue: Perception → Planning
localization_to_prediction_queue: Localization → Prediction
localization_to_planning_queue: Localization → Planning
prediction_to_planning_queue: Prediction → Planning
planning_to_control_queue: Planning → Control
```

## 성능 모니터링 시스템

### PerformanceMonitor
- CPU/메모리 사용률 추적
- 큐 상태 모니터링  
- FPS 및 지연시간 측정
- 실시간 알림 및 보고서 생성

### AdaptiveOptimizer
- 성능 병목 지점 자동 감지
- 동적 큐 크기 조정
- 처리 주기 최적화
- 자동 성능 튜닝

## 설정 구조

### 주요 설정 값
- **액션 플래너**: "hsv_lane_following"
- **제어 법칙**: "basic_pid"  
- **속도 변환**: Planning(0.028) ↔ Control(35.71)
- **모니터링 주기**: 1.0초
- **최적화 주기**: 5.0초

## 시스템 시작/종료 시퀀스

### 시작 순서
1. 성능 모니터 초기화
2. 큐 시스템 초기화
3. 모듈 인스턴스 생성
4. 성능 모니터에 등록
5. 센서 관리자 시작
6. 나머지 모듈 스레드 시작

### 종료 순서  
1. 적응적 최적화 정지
2. 성능 모니터 정지
3. 모듈들 역순으로 정지
4. 스레드 조인 및 정리

## 타임스탬프 동기화

### TimestampSynchronizer
- 모듈 간 데이터 동기화
- 지연시간 보상
- 50ms 허용 오차

## 프로그램 실행 명령
```bash
cd /home/xytron/xycar_ws
source devel/setup.bash  
python3 src/kookmin/driver/Original/track_drive.py
```

## 주요 개선사항
1. **오류 수정 완료**: 
   - PerformanceMonitor에 metrics_lock 및 누락된 메서드 추가
   - time 모듈 import 누락 문제 해결
   - 큐 예외 처리 통합 (queue.Empty, _queue.Empty)
2. **주석 간소화 완료**: 모든 모듈 파일 주석을 핵심만 유지하도록 정리
3. **시스템 안정성 확보**: import 및 큐 처리 문제 완전 해결
4. **성능 최적화**: 큐 기반 비동기 처리 및 우선순위 시스템
5. **모니터링 시스템**: 실시간 성능 추적 및 적응적 최적화

## 최종 검증 완료
- `python3 track_drive.py` 명령어로 정상 실행 확인
- 모든 모듈 스레드 정상 시작
- 큐 기반 통신 안정적 동작
- 성능 모니터링 시스템 정상 작동
