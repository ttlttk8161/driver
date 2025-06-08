readme.txt
세 모듈(lane_detector.py, path_planner.py, obstacle_detector.py)은 상호 의존성이 거의 없고, 각각 독립적으로 동작하도록 설계되어 있어 다른 프로젝트나 환경에 이식이 비교적 쉽습니다.
아래는 각 모듈의 이식성에 대한 설명입니다.

1. lane_detector.py
입력: OpenCV 이미지(np.ndarray)
출력: 차선 포인트, 시각화 이미지 등
외부 의존성: OpenCV(cv2), numpy
특징: ROS 등 특정 프레임워크에 종속적이지 않고, 이미지 처리만 담당
이식성: 매우 높음 (OpenCV와 numpy만 있으면 사용 가능)
2. path_planner.py
입력: 차선 웨이포인트(np.ndarray), 차량 상태(dict), 장애물 정보 등
출력: 경로 후보, 최적 경로 등
외부 의존성: numpy, scipy, math
특징: 경로 생성 및 비용 평가만 담당, 센서/이미지/ROS 메시지와 직접 연결 없음
이식성: 매우 높음 (입출력 데이터 포맷만 맞추면 사용 가능)
3. obstacle_detector.py
입력: 라이다 스캔 데이터(포인트 클라우드), numpy 배열
출력: 장애물 리스트, 충돌 위험 등
외부 의존성: numpy, sklearn(DBSCAN), dataclasses
특징: 라이다 데이터만 처리, ROS 메시지 타입만 맞추면 독립적 사용 가능
이식성: 높음 (입력 데이터 구조만 맞추면 사용 가능)
결론
세 모듈 모두 특정 프레임워크(ROS 등)에 강하게 종속되어 있지 않으며, 핵심 로직은 순수 Python/numpy/scipy/OpenCV 등 범용 라이브러리만 사용합니다.
입출력 데이터 포맷만 맞추면, 다른 프로젝트나 환경(예: ROS가 아닌 일반 Python 프로젝트, 시뮬레이터 등)에서도 쉽게 이식하여 사용할 수 있습니다.
단,

obstacle_detector.py의 경우 라이다 데이터 포맷(sensor_msgs/LaserScan 등)만 맞춰주면 되고,
path_planner.py는 차선 웨이포인트와 차량 상태를 넘겨주는 방식만 맞추면 됩니다.
lane_detector.py는 OpenCV 이미지만 넘기면 바로 사용 가능합니다.
즉, 세 모듈 모두 높은 이식성을 가집니다.