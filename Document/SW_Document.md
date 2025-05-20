## `/home/xytron/xycar_ws/src/kookmin/driver/track_drive.py` 코드 분석

이 파이썬 스크립트는 ROS(Robot Operating System) 환경에서 작동하는 자율주행 차량(Xycar로 추정)용 노드입니다. 주로 카메라 및 라이다 센서 데이터를 수신하여 시각화하고, 기본적인 모터 제어 명령을 보내는 기능을 수행합니다.

### 소프트웨어 아키텍처

이 스크립트는 일반적인 ROS 노드의 구조를 따릅니다.

1.  **노드 초기화**:
    *   `rospy.init_node('Track_Driver')`를 사용하여 `Track_Driver`라는 이름의 ROS 노드를 초기화합니다.

2.  **Subscribers (입력부)**:
    *   **카메라 데이터**: `/usb_cam/image_raw` 토픽을 구독합니다. 이 토픽은 `sensor_msgs.msg.Image` 타입의 메시지를 발행할 것으로 예상되며, 새로운 이미지 데이터가 수신될 때마다 `usbcam_callback` 함수가 호출됩니다.
    *   **라이다 데이터**: `/scan` 토픽을 구독합니다. 이 토픽은 `sensor_msgs.msg.LaserScan` 타입의 메시지를 발행할 것으로 예상되며, `lidar_callback` 함수가 라이다 데이터를 처리합니다.

3.  **Publisher (출력부)**:
    *   **모터 제어**: `xycar_motor`라는 이름의 Publisher를 생성하여 `xycar_msgs.msg.XycarMotor` 타입의 메시지를 `xycar_motor` 토픽으로 발행합니다. 이를 통해 차량의 조향각과 속도를 제어합니다.

4.  **데이터 처리 및 저장**:
    *   `image`: 수신된 최신 카메라 이미지를 저장하는 전역 NumPy 배열입니다. `CvBridge`를 통해 OpenCV 형식으로 변환됩니다.
    *   `ranges`: 최신 라이다 스캔 데이터(처음 360개의 거리 값)를 저장하는 전역 리스트입니다.
    *   `motor_msg`: `XycarMotor` 메시지 타입의 인스턴스로, 모터 제어 메시지를 발행하기 전에 데이터를 담는 데 사용됩니다.
    *   `Fix_Speed`: 차량의 고정 속도 값(10)을 저장하는 상수입니다.
    *   `bridge`: ROS 이미지 메시지와 OpenCV 이미지 간의 변환을 위한 `CvBridge` 객체입니다.

5.  **핵심 로직 구성 요소**:
    *   **콜백 함수 (`usbcam_callback`, `lidar_callback`)**: 이벤트 기반으로 동작합니다. 센서 데이터가 도착할 때마다 호출되어 전역 변수(`image`, `ranges`)를 최신 데이터로 업데이트합니다.
    *   **`drive(angle, speed)` 함수**: 조향각과 속도 값을 받아 모터 제어 메시지를 발행하는 유틸리티 함수입니다.
    *   **`start()` 함수**: 노드의 설정 및 메인 처리 루프를 포함하는 주 실행 함수입니다.
    *   **시각화**:
        *   OpenCV (`cv2`): 카메라 영상을 원본(컬러) 및 그레이스케일로 화면에 표시합니다.
        *   `matplotlib.pyplot`: 라이다 스캔 데이터를 실시간으로 플롯팅합니다.

6.  **의존성**:
    *   `rospy`: ROS 통신을 위한 라이브러리.
    *   `cv2` (OpenCV): 이미지 표시를 위한 라이브러리.
    *   `cv_bridge`: ROS 이미지 메시지를 OpenCV 이미지로 변환하기 위한 브릿지.
    *   `numpy`: 이미지 데이터 및 라이다 좌표 계산 등 수치 연산을 위한 라이브러리.
    *   `matplotlib`: 라이다 데이터 플로팅을 위한 라이브러리.
    *   `sensor_msgs.msg.Image`, `sensor_msgs.msg.LaserScan`, `xycar_msgs.msg.XycarMotor`: 사용되는 ROS 메시지 타입.

### 프로그램 플로우 (실행 흐름)

프로그램은 다음과 같은 순서로 실행됩니다.

1.  **초기화 단계 (`start()` 함수 내부)**:
    a.  스크립트가 시작되고 전역 변수들이 정의됩니다.
    b.  `start()` 함수가 호출됩니다 (`if __name__ == '__main__':` 조건에 의해).
    c.  "Start program --------------" 메시지가 콘솔에 출력됩니다.
    d.  `Track_Driver` ROS 노드가 초기화됩니다.
    e.  `/usb_cam/image_raw` 토픽 (콜백: `usbcam_callback`)과 `/scan` 토픽 (콜백: `lidar_callback`)에 대한 Subscriber가 설정됩니다.
    f.  `xycar_motor` 토픽으로 메시지를 발행할 `motor` Publisher가 생성됩니다.
    g.  `rospy.wait_for_message()`: 카메라와 라이다 토픽으로부터 첫 번째 메시지가 수신될 때까지 대기합니다. 이는 센서가 활성화되어 데이터를 발행하기 시작했음을 보장합니다. 성공 시 "Camera Ready", "Lidar Ready" 메시지가 출력됩니다.
    h.  `matplotlib`이 실시간 플로팅을 위해 설정되고 (`plt.ion()`, `plt.show()`), "Lidar Visualizer Ready" 메시지가 출력됩니다.
    i.  "S T A R T D R I V I N G ..." 메시지가 출력됩니다.

2.  **메인 루프 (`start()` 함수 내부, `while not rospy.is_shutdown():`)**:
    이 루프는 ROS 마스터가 실행 중이고 노드가 종료되지 않는 한 (예: Ctrl+C 입력) 계속 실행됩니다.
    a.  **이미지 표시**:
        i.  현재 전역 변수 `image` (비동기적으로 `usbcam_callback`에 의해 업데이트됨)를 그레이스케일로 변환합니다.
        ii. 원본 컬러 이미지 (`cv2.imshow("original", image)`)와 그레이스케일 이미지 (`cv2.imshow("gray", gray)`)를 OpenCV 창에 표시합니다.
    b.  **라이다 시각화**:
        i.  `ranges` (라이다 데이터, `lidar_callback`에 의해 업데이트됨)가 `None`이 아닌지 확인합니다.
        ii. 데이터가 존재하면, 극좌표계인 `ranges` 데이터를 직교좌표계(X, Y)로 변환합니다.
        iii. `matplotlib` 플롯 (`lidar_points`)을 이 새로운 좌표로 업데이트합니다.
        iv. 플롯을 다시 그립니다 (`fig.canvas.draw_idle()`, `plt.pause(0.01)`).
    c.  **모터 제어**:
        i.  `drive(angle=0.0, speed=10.0)` 함수가 호출됩니다. 이는 `xycar_motor` 토픽으로 조향각을 `0.0`(직진), 속도를 `10.0`으로 설정하는 명령을 보냅니다. **중요: 이 스크립트 버전에서는 센서 입력과 관계없이 항상 고정된 값으로 직진 주행합니다.**
    d.  **루프 지연**:
        i.  `time.sleep(0.1)`: 루프가 0.1초 동안 일시 중지됩니다. 이는 메인 루프의 처리 속도를 약 10Hz로 설정합니다.
    e.  **OpenCV 창 처리**:
        i.  `cv2.waitKey(1)`: OpenCV의 `imshow`가 올바르게 작동하기 위해 필요합니다. GUI 이벤트를 처리하고 1ms 동안 키 입력을 대기합니다.

3.  **종료**:
    *   ROS가 종료되면 (예: 사용자가 터미널에서 Ctrl+C를 누르면) `rospy.is_shutdown()` 조건이 참이 되어 `while` 루프가 종료됩니다.
    *   스크립트가 종료됩니다. (참고: 이 버전의 코드에서는 루프 종료 후 `cv2.destroyAllWindows()`나 `plt.close()`를 명시적으로 호출하지 않지만, 일반적으로 리소스 정리를 위해 추가하는 것이 좋습니다.)

**요약하자면,**

`/home/xytron/xycar_ws/src/kookmin/driver/track_drive.py` 스크립트는 다음을 수행하는 ROS 노드를 설정합니다:
*   카메라 및 라이다 데이터를 수신합니다.
*   카메라 영상을 컬러 및 그레이스케일로 표시합니다.
*   라이다 스캔 데이터를 2D 플롯으로 시각화합니다.
*   지속적으로 Xycar 차량에게 직진(`angle=0.0`)하고 고정된 속도(`speed=10.0`)로 주행하도록 명령합니다.

이 스크립트는 센서 데이터 시각화 및 기본적인 모터 제어를 위한 기본 템플릿 역할을 하지만, 이전에 제공해주셨던 참조 파일에 있던 차선 감지나 장애물 회피와 같은 고급 주행 로직은 포함하고 있지 않습니다.

### `def usbcam_callback(data)` 함수 상세 설명

`usbcam_callback` 함수는 ROS 시스템에서 카메라 이미지 데이터가 수신될 때마다 자동으로 호출되는 콜백(callback) 함수입니다.

*   **함수 정의**: `def usbcam_callback(data):`
*   **목적**: `/usb_cam/image_raw` 토픽을 통해 전달되는 ROS의 표준 이미지 메시지(`sensor_msgs.msg.Image`)를 수신하여, 프로그램의 다른 부분(특히 OpenCV를 사용하는 부분)에서 쉽게 사용할 수 있는 OpenCV 이미지 형식으로 변환하고, 이를 전역 변수 `image`에 저장하는 것입니다.
*   **파라미터 `data`**:
    *   이 파라미터는 `/usb_cam/image_raw` 토픽으로부터 수신된 `sensor_msgs.msg.Image` 타입의 메시지 객체입니다.
    *   이 메시지 객체 안에는 이미지의 높이(height), 너비(width), 픽셀 데이터의 인코딩 방식(encoding), 실제 이미지 데이터(data 필드) 등의 정보가 포함되어 있습니다.
*   **함수 내부 동작**:
    1.  `global image`: 이 줄은 함수 내에서 전역 범위에 선언된 `image` 변수의 값을 변경할 것임을 파이썬 인터프리터에게 알립니다. 이렇게 선언하지 않으면, 함수 내에서 `image`에 값을 할당할 때 새로운 지역 변수 `image`가 생성됩니다.
    2.  `image = bridge.imgmsg_to_cv2(data, "bgr8")`:
        *   `bridge`: 이것은 `CvBridge` 클래스의 인스턴스입니다. `CvBridge`는 ROS의 `sensor_msgs.msg.Image` 메시지와 OpenCV에서 사용하는 이미지 형식(주로 NumPy 배열) 간의 변환을 편리하게 수행할 수 있도록 도와주는 유틸리티입니다.
        *   `imgmsg_to_cv2()`: `CvBridge` 객체의 메소드로, 첫 번째 인자로 받은 ROS 이미지 메시지(`data`)를 OpenCV 이미지 형식으로 변환합니다.
        *   `"bgr8"`: 두 번째 인자로, 변환될 OpenCV 이미지의 원하는 인코딩 형식을 지정합니다.
            *   `bgr`: OpenCV에서 기본적으로 사용하는 색상 채널 순서인 파란색(Blue), 녹색(Green), 빨간색(Red)을 의미합니다.
            *   `8`: 각 색상 채널당 8비트(0~255 범위의 값)를 사용함을 의미합니다. 즉, 일반적인 24비트 컬러 이미지를 나타냅니다.
        *   이 변환 과정을 거친 결과 (OpenCV 이미지, NumPy 배열 형태)가 전역 변수 `image`에 할당됩니다. 이렇게 저장된 `image` 변수는 메인 루프 등에서 `cv2.imshow()`를 통해 화면에 표시되거나 다른 이미지 처리 작업에 사용될 수 있습니다.

결론적으로, `usbcam_callback` 함수는 ROS 네트워크를 통해 들어오는 카메라 이미지 스트림을 실시간으로 받아 OpenCV가 이해할 수 있는 형태로 변환하여 프로그램의 나머지 부분이 활용할 수 있도록 하는 중요한 역할을 담당합니다. 이 함수는 비동기적으로, 즉 새로운 이미지 프레임이 도착할 때마다 독립적으로 실행됩니다.