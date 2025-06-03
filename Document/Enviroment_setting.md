# `track_drive.py` 및 `Modules` 실행 환경 설정 안내

이 문서는 `track_drive.py` 프로그램과 `Modules` 폴더 내의 파이썬 모듈들이 올바르게 동작하기 위해 필요한 패키지 설치 및 환경 변수 설정 방법을 안내합니다. 작업물을 다른 사람에게 전달할 때 이 문서를 함께 제공하여 원활한 환경 구성을 도울 수 있습니다.

## 1. 필수 Python 패키지 설치

다음 Python 패키지들이 설치되어 있어야 합니다. `pip`을 사용하여 설치할 수 있습니다.

*   **NumPy**: 수치 연산을 위한 필수 라이브러리입니다.
    ```bash
    pip install numpy
    ```
*   **OpenCV (cv2)**: 이미지 처리 및 컴퓨터 비전 기능을 위해 필요합니다.
    ```bash
    pip install opencv-python
    ```
*   **가상환경 사용2**: 첨부된 cwnu2 가상환경을 사용하셔야합니다. 절차는 다음과 같습니다.
    ```
    1. 가상환경을 활성화합니다.
        ```bash
        source /path/to/cwnu2/bin/activate
        ```
    2. (cwnu2) xytron@DongHyun:~/cwnu2$ cd YOLOv8-multi-task/
        (cwnu2) xytron@DongHyun:~/cwnu2/YOLOv8-multi-task$ pip install -r requirements.txt
    3. (cwnu2) xytron@DongHyun:~/cwnu2/YOLOv8-multi-task$ pip install -e .

    이 문서는 `track_drive.py` 프로그램과 `Modules` 폴더 내의 파이썬 모듈들이 올바르게 동작하기 위해 필요한 패키지 설치 및 환경 변수 설정 방법을 안내합니다. 작업물을 다른 사람에게 전달할 때 이 문서를 함께 제공하여 원활한 환경 구성을 도울 수 있습니다.
    
    ## 0. (선택 사항) 메모리 부족 시 스왑(Swap) 공간 확장 안내
    
    `requirements.txt`에 포함된 일부 패키지(예: `torch`, `tensorflow` 등)는 설치 과정에서 많은 메모리를 사용할 수 있습니다. 시스템의 RAM이 부족한 경우, 설치가 실패하거나 시스템이 매우 느려질 수 있습니다. 이 경우 스왑 공간을 확장하여 문제를 해결할 수 있습니다.
    
    **주의**: 아래 명령어들은 시스템 설정을 변경하며, `sudo` 권한이 필요합니다. 진행하기 전에 현재 시스템 상태를 이해하고 신중하게 실행하십시오.
    
    **1. 현재 스왑 공간 확인**
    
    터미널에서 다음 명령어를 실행하여 현재 스왑 공간을 확인합니다.
    ```bash
    free -h
    swapon --show
    ```
    `Swap` 항목의 `total` 및 `used` 값을 확인합니다. `used`가 `total`에 근접하거나 `total`이 매우 작다면(예: 1GB 이하), 스왑 공간 확장을 고려하십시오.
    
    **2. 스왑 파일 생성 및 활성화 (예: 4GB 추가)**
    
    다음은 `/swapfile_extra`라는 이름으로 4GB 크기의 스왑 파일을 생성하고 활성화하는 예시입니다. 필요에 따라 용량(`4G`)이나 파일명은 변경할 수 있습니다.
    
    ```bash
    # 스왑 파일 생성 (4GB)
    sudo fallocate -l 4G /swapfile_extra
    # 만약 fallocate 명령어 사용이 불가능하다면 dd 명령어를 사용합니다:
    # sudo dd if=/dev/zero of=/swapfile_extra bs=1M count=4096
    
    # 스왑 파일 권한 설정
    sudo chmod 600 /swapfile_extra
    
    # 스왑 파일로 포맷
    sudo mkswap /swapfile_extra
    
    # 생성한 스왑 파일 활성화
    sudo swapon /swapfile_extra
    ```
    
    **3. 스왑 공간 재확인**
    
    다시 다음 명령어로 스왑 공간이 정상적으로 확장되었는지 확인합니다.
    ```bash
    free -h
    swapon --show
    ```
    새로 추가한 `/swapfile_extra`가 목록에 보이고, 전체 스왑 크기가 증가했는지 확인합니다.
    
    **4. (선택 사항) 재부팅 후에도 스왑 파일 유지**
    
    위 설정은 현재 세션에만 유효합니다. 재부팅 후에도 이 스왑 파일을 계속 사용하려면 `/etc/fstab` 파일에 등록해야 합니다.
    
    먼저, 만약을 위해 `/etc/fstab` 파일을 백업합니다.
    ```bash
    sudo cp /etc/fstab /etc/fstab.bak
    ```
    그 다음, `/etc/fstab` 파일에 다음 내용을 추가합니다.
    ```bash
    echo '/swapfile_extra none swap sw 0 0' | sudo tee -a /etc/fstab
    ```
    이제 시스템을 재부팅해도 `/swapfile_extra` 스왑 파일이 자동으로 활성화됩니다.
    
    이 과정을 통해 메모리 부족으로 인한 패키지 설치 문제를 예방할 수 있습니다.
    

## 2. ROS (Robot Operating System) 환경 설정

이 프로젝트는 ROS를 기반으로 동작하므로, ROS 환경이 올바르게 설치 및 구성되어 있어야 합니다.

### 2.1. ROS 설치

사용 중인 ROS 버전(예: Melodic, Noetic 등)이 시스템에 설치되어 있어야 합니다. ROS 설치는 공식 ROS 위키 문서를 참고하십시오.

### 2.2. 필수 ROS 패키지

다음 ROS 패키지들이 필요합니다. 대부분 ROS 설치 시 함께 설치되거나, `apt-get`을 통해 쉽게 설치할 수 있습니다.

*   `rospy`: ROS의 Python 클라이언트 라이브러리입니다. (ROS 설치 시 기본 포함)
*   `sensor_msgs`: 표준 ROS 메시지 타입(Image, LaserScan 등)을 포함합니다. (ROS 설치 시 기본 포함)
*   `cv_bridge`: ROS 이미지 메시지와 OpenCV 이미지 간의 변환을 담당합니다. ROS의 `vision_opencv` 스택에 포함되어 있으며, 다음과 같이 설치할 수 있습니다 (ROS 버전에 따라 `<your_ros_distro>` 부분을 수정하세요. 예: `noetic`).
    ```bash
    sudo apt-get install ros-<your_ros_distro>-cv-bridge
    ```
    예시 (ROS Noetic):
    ```bash
    sudo apt-get install ros-noetic-cv-bridge
    ```

### 2.3. 커스텀 ROS 패키지 (`xycar_msgs`)

*   `xycar_msgs`: Xycar 차량 제어를 위한 커스텀 ROS 메시지(`XycarMotor`)가 포함된 패키지입니다.
    *   이 패키지는 사용자의 ROS 작업 공간(예: `/home/xytron/xycar_ws/src`) 내에 위치해야 합니다.
    *   패키지를 작업 공간에 복사한 후, `catkin_make` (또는 ROS2의 경우 `colcon build` 등)를 사용하여 빌드해야 합니다.
    *   작업물을 전달받는 사람도 이 `xycar_msgs` 패키지를 자신의 ROS 작업 공간에 포함시키고 빌드하는 과정이 필요합니다.

## 3. ROS 환경 변수 설정

ROS 프로그램을 실행하기 전에 터미널에서 ROS 환경을 활성화(source)해야 합니다.

1.  **기본 ROS 환경 설정**:
    터미널을 열 때마다 또는 `.bashrc` (또는 `.zshrc` 등 사용하는 쉘 설정 파일)에 추가하여 자동으로 실행되도록 합니다.
    ```bash
    source /opt/ros/<your_ros_distro>/setup.bash
    ```
    예시 (ROS Noetic):
    ```bash
    source /opt/ros/noetic/setup.bash
    ```

2.  **현재 작업 공간(workspace) 환경 설정**:
    `track_drive.py` 파일 및 `xycar_msgs` 패키지가 포함된 ROS 작업 공간(예: `xycar_ws`)의 `devel/setup.bash` (또는 `install/setup.bash`) 파일을 source해야 합니다. 이는 Python 인터프리터가 로컬 패키지를 찾을 수 있도록 합니다.
    ```bash
    source /path/to/your/xycar_ws/devel/setup.bash
    ```
    예시 (`/home/xytron/xycar_ws` 경로 사용 시):
    ```bash
    source /home/xytron/xycar_ws/devel/setup.bash
    ```
    이 명령어도 터미널 실행 시 또는 쉘 설정 파일에 추가하는 것이 편리합니다.

## 4. PYTHONPATH 환경 변수 (일반적으로 불필요)

`track_drive.py` 파일 내에 다음 코드가 포함되어 있어,
```python
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
```
`Modules` 패키지를 Python 경로에 동적으로 추가합니다. 따라서 `track_drive.py`를 직접 실행하는 경우에는 별도의 `PYTHONPATH` 환경 변수 설정이 필요 없을 가능성이 높습니다.

만약 모듈을 찾지 못하는 문제가 발생한다면, `PYTHONPATH`에 `Modules` 폴더의 상위 디렉토리 (즉, `track_drive.py`가 있는 디렉토리)를 추가하는 것을 고려해볼 수 있습니다.
예:
```bash
export PYTHONPATH="/home/xytron/xycar_ws/src/kookmin/driver/Original:$PYTHONPATH"
```
하지만 이는 위 `sys.path.append` 로직으로 인해 대부분 필요하지 않습니다.

## 5. 파일 경로 및 권한

*   **로그 파일**: `track_drive.py`에서 로그 파일 경로가 `/home/xytron/xycar_ws/src/kookmin/driver/Original/track_drive.log`로 지정되어 있습니다. 이 경로에 로그 파일을 생성하고 쓸 수 있는 권한이 필요합니다.
*   **HD Map**: `Modules/main_system.py`의 `load_dummy_config()` 함수 내 `hd_map_path`가 `path/to/dummy_map.osm` (또는 `.hd`)로 설정되어 있습니다. 실제 맵을 사용한다면 해당 경로가 올바른지, 그리고 파일에 접근 권한이 있는지 확인해야 합니다. `main_system.py`의 `if __name__ == "__main__":` 블록에서는 이 더미 경로에 파일을 생성하려고 시도합니다.

## 6. 전달 시 요약 안내 사항

다른 사용자에게 작업물을 전달할 때, 다음 사항들을 안내해주십시오:

1.  **Python 패키지 설치**: `numpy`, `opencv-python` 설치.
2.  **ROS 환경 확인**: 사용 중인 ROS 버전 설치 확인.
3.  **필수 ROS 패키지 설치**: `ros-<distro>-cv-bridge` 설치.
4.  **커스텀 ROS 패키지 설정**: `xycar_msgs` 패키지를 ROS 작업 공간(`src` 폴더 내)에 복사하고 `catkin_make` (또는 해당 ROS 버전 빌드 명령어) 실행.
5.  **ROS 환경 활성화**: 터미널 실행 시 또는 쉘 설정 파일(`.bashrc` 등)에 ROS 기본 `setup.bash`와 작업 공간의 `devel/setup.bash` (또는 `install/setup.bash`)를 source하는 명령어 추가.
6.  **실행 권한 및 경로 확인**: 로그 파일 저장 경로에 쓰기 권한 확인 및 HD Map 경로 확인.

이 정보들이 작업물을 성공적으로 전달하고 실행하는 데 도움이 되기를 바랍니다.