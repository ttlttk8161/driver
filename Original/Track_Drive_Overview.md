track_drive.py 실행 후 프로세스를 분석해보겠습니다. 이제 track_drive.py 실행 프로세스를 요약 설명하겠습니다:

## track_drive.py 실행 프로세스 요약

### 1. **초기화 단계 (Initialization Phase)**
```
ROS 노드 초기화 → 로깅 설정 → 센서 토픽 대기 → MainSystem 생성
```

**상세 프로세스:**
- **ROS 노드 등록**: `Track_Driver` 노드로 ROS 네트워크에 등록
- **로깅 시스템 구축**: 콘솔 + 파일 로그 동시 출력 설정
- **모터 퍼블리셔 생성**: `/xycar_motor` 토픽에 제어 명령 발행 준비
- **센서 토픽 확인**: 카메라(`/usb_cam/image_raw`) 및 라이다(`/scan`) 토픽 활성 상태 검증
- **stdout 리디렉션**: 모든 print 출력을 로그 파일과 콘솔에 동시 기록

### 2. **시스템 설정 단계 (Configuration Phase)**
```
Config 로딩 → 알고리즘 설정 → ROS 객체 주입 → MainSystem 초기화
```

**상세 프로세스:**
- **설정 파일 로딩**: `load_dummy_config()`로 기본 설정값 적용
- **인식 알고리즘 설정**: HSV 차선 감지 등 perception 알고리즘 구성
- **ROS 객체 주입**: CvBridge, 모터 퍼블리셔, 메시지 템플릿을 config에 추가
- **MainSystem 생성**: 모든 모듈(센서, 인지, 계획, 제어)을 포함한 통합 시스템 초기화

### 3. **시스템 구동 단계 (System Launch Phase)**
```
MainSystem 시작 → 모든 모듈 스레드 활성화 → 데이터 파이프라인 가동
```

**상세 프로세스:**
- **MainSystem.start() 호출**:
  - SensorInputManager: ROS 토픽 구독 시작
  - PerceptionModule: 센서 데이터 처리 스레드 시작
  - LocalizationModule: 위치 인식 스레드 시작
  - PredictionModule: 행동 예측 스레드 시작
  - PlanningModule: 경로 계획 스레드 시작
  - ControlModule: 차량 제어 스레드 시작
  - PerformanceMonitor: 성능 모니터링 시작

### 4. **실행 루프 단계 (Main Loop Phase)**
```
무한 루프 → ROS 상태 확인 → 0.1초 대기 → 반복
```

**상세 프로세스:**
- **메인 스레드**: ROS 노드 생존 확인 (`rospy.is_shutdown()`)
- **백그라운드 스레드들**: 각 모듈이 독립적으로 데이터 처리
- **데이터 흐름**: 센서 → 인지 → 위치인식/예측 → 계획 → 제어

### 5. **데이터 파이프라인 흐름**
```
센서 입력 → Queue → 인지 처리 → Queue → 계획 → Queue → 제어 → 모터 명령
```

**실제 데이터 흐름:**
1. **SensorInputManager**: `/usb_cam/image_raw`, `/scan` 토픽 구독 → SensorData 생성
2. **PerceptionModule**: HSV 차선 감지 알고리즘 실행 → PerceptionOutput 생성
3. **LocalizationModule**: 위치 정보 처리 → LocalizationInfo 생성
4. **PlanningModule**: 경로 계획 및 행동 결정 → PlannedPath, ManeuverDecision 생성
5. **ControlModule**: PID 제어기로 조향각/속도 계산 → `/xycar_motor` 토픽 발행

### 6. **종료 처리 단계 (Shutdown Phase)**
```
종료 신호 감지 → 모든 모듈 정지 → 리소스 정리 → 로그 마무리
```

**안전한 종료 프로세스:**
- **KeyboardInterrupt 처리**: Ctrl+C 신호 감지
- **MainSystem.stop()**: 모든 모듈 스레드 안전 종료
- **ROS 정리**: 토픽 구독 해제, 퍼블리셔 정리
- **로그 정리**: 파일 핸들러 종료, stdout 복원

### 7. **현재 상태 분석**
- ✅ **정상 동작**: 모든 모듈 정상 시작, 센서 데이터 수신 확인
- ⚠️ **문제점**: `/xycar_motor` 토픽에 실제 명령이 발행되지 않음
- 🔍 **원인 추정**: 데이터 파이프라인 중간 단계에서 처리 지연 또는 로직 오류

이 시스템은 **모듈화된 자율주행 아키텍처**로, 각 모듈이 독립적인 스레드에서 실행되며 Queue를 통해 통신하는 구조입니다.