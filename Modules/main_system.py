import threading
import queue
import time
import os # For creating dummy config if needed

# Import module classes
from data_structures import SensorData # And others if directly used here
from sensor_input_module import SensorInputManager
from perception_module import PerceptionModule
from localization_module import LocalizationModule
from prediction_module import PredictionModule
from planning_module import PlanningModule
from control_module import ControlModule

# Dummy config function
def load_dummy_config() -> dict:
    return {
        "sensor_input_config": {},
        "perception_config": {
            "detection": {}, "scene_understanding": {}, "tracking": {}, "perception_prediction": {}
        },
        "hd_map_path": "path/to/dummy_map.osm", # Example path
        "localization_config": {},
        "prediction_config": {},
        "planning_config": {
            "path_planner": {}, "decision_maker": {}, "action_planner": {}
        },
        "control_config": {},
        "vehicle_interface_config": { # This will be populated by track_drive.py
            # "ros_motor_publisher": None,
            # "ros_motor_msg_template": None
        } 
    }

class MainSystem:
    def __init__(self, config: dict):
        self.config = config
        self._initialize_queues()
        self._initialize_modules()
        self._threads = []
        print("MainSystem: Initialized.")

    def _initialize_queues(self):
        print("MainSystem: Initializing queues...")
        self.sensor_to_perception_queue = queue.Queue(maxsize=10)

        # Perception outputs to multiple modules
        self.perception_to_localization_queue = queue.Queue(maxsize=5)
        self.perception_to_prediction_queue = queue.Queue(maxsize=5)
        self.perception_to_planning_queue = queue.Queue(maxsize=5) # For direct scene info to planning

        # Localization outputs to multiple modules
        self.localization_to_prediction_queue = queue.Queue(maxsize=5)
        self.localization_to_planning_queue = queue.Queue(maxsize=5)

        self.prediction_to_planning_queue = queue.Queue(maxsize=5)
        self.planning_to_control_queue = queue.Queue(maxsize=5)

        # Optional direct sensor input to localization (e.g., GNSS/IMU if not through perception)
        self.direct_sensor_to_localization_queue = queue.Queue(maxsize=10) # Example

    def _initialize_modules(self):
        print("MainSystem: Initializing modules...")
        # 1. Sensor Input
        # track_drive.py에서 CvBridge 객체를 config 통해 전달받는다고 가정
        ros_bridge_instance = self.config.get("ros_bridge")
        self.sensor_manager = SensorInputManager(
            self.config.get("sensor_input_config", {}),
            self.sensor_to_perception_queue, # Sensor manager directly outputs to perception
            ros_bridge=ros_bridge_instance
            # If direct GNSS/IMU to localization: self.direct_sensor_to_localization_queue (needs sensor manager logic change)
        )

        # 2. Perception Module
        perception_output_queues = {
            "localization": self.perception_to_localization_queue,
            "prediction": self.perception_to_prediction_queue,
            "planning": self.perception_to_planning_queue
        }
        self.perception_module = PerceptionModule(
            self.config.get("perception_config", {}),
            self.sensor_to_perception_queue,
            perception_output_queues
        )

        # 3. Localization Module
        localization_output_queues = {
            "prediction": self.localization_to_prediction_queue,
            "planning": self.localization_to_planning_queue
        }
        self.localization_module = LocalizationModule(
            self.config.get("localization_config", {}),
            self.config.get("hd_map_path", "dummy_map.hd"),
            input_queue_perception=self.perception_to_localization_queue,
            input_queue_sensor=None, # Assuming GNSS/IMU goes through perception or is handled internally by perception for features
            output_queues=localization_output_queues
        )

        # 4. Prediction Module (Behavioral)
        self.prediction_module = PredictionModule(
            self.config.get("prediction_config", {}),
            input_queue_perception=self.perception_to_prediction_queue,
            input_queue_localization=self.localization_to_prediction_queue,
            output_queue=self.prediction_to_planning_queue
        )

        # 5. Planning Module
        planning_input_queues = {
            "localization": self.localization_to_planning_queue,
            "prediction": self.prediction_to_planning_queue,
            "perception": self.perception_to_planning_queue # For static scene context
        }
        self.planning_module = PlanningModule(
            self.config.get("planning_config", {}),
            self.config.get("hd_map_path", "dummy_map.hd"),
            input_queues=planning_input_queues,
            output_queue_control=self.planning_to_control_queue
        )

        # 6. Control Module
        # track_drive.py에서 motor 퍼블리셔와 메시지 템플릿을 config 통해 전달받는다고 가정
        vehicle_if_config = self.config.get("vehicle_interface_config", {}).copy() # 복사해서 사용
        vehicle_if_config["ros_motor_publisher"] = self.config.get("ros_motor_publisher")
        vehicle_if_config["ros_motor_msg_template"] = self.config.get("ros_motor_msg_template")
        self.control_module = ControlModule(
            self.config.get("control_config", {}),
            input_queue_planning=self.planning_to_control_queue,
            vehicle_interface_config=vehicle_if_config
        )

        self.modules = [
            self.sensor_manager, # Sensor manager has start/stop methods, not run in a list of threads here
            self.perception_module,
            self.localization_module,
            self.prediction_module,
            self.planning_module,
            self.control_module
        ]


    def start(self):
        print("MainSystem: Starting all modules...")
        # Start sensor manager separately as it might manage its own thread(s) differently
        self.sensor_manager.start_sensors()
        time.sleep(0.5) # Give sensors a moment

        # Start other modules
        for module in self.modules:
            if hasattr(module, 'start') and module != self.sensor_manager : # sensor_manager already started
                module.start()
                self._threads.append(module._thread) # Assuming module._thread is the worker thread
        print("MainSystem: All modules started.")


    def stop(self):
        print("MainSystem: Stopping all modules...")

        # Stop modules in reverse order of data flow or based on dependencies
        # Control first, then planning etc.
        # Or, signal all to stop and then join
        for module in reversed(self.modules): # sensor_manager will be last
            if hasattr(module, 'stop'):
                print(f"MainSystem: Stopping {module.__class__.__name__}...")
                module.stop()

        # Join threads (if module.stop() doesn't join already)
        # This might be redundant if module.stop() already calls join.
        # For robustness, ensure threads are joined.
        for thread in self._threads:
            if thread and thread.is_alive():
                print(f"MainSystem: Joining thread {thread.name}...")
                thread.join(timeout=5.0) # Add timeout to join
                if thread.is_alive():
                    print(f"MainSystem: WARNING - Thread {thread.name} did not terminate.")

        print("MainSystem: All modules stopped.")

if __name__ == "__main__":
    print("=============== Autonomous Driving System Simulation ===============")
    # Create a dummy HD map file if it doesn't exist for HDMapInterface to load
    dummy_map_path = "path/to/dummy_map.hd"
    os.makedirs(os.path.dirname(dummy_map_path), exist_ok=True)
    if not os.path.exists(dummy_map_path):
        with open(dummy_map_path, 'w') as f:
            f.write("This is a dummy HD map file.\n")
        print(f"Created dummy HD map file: {dummy_map_path}")


    config = load_dummy_config()
    config["hd_map_path"] = dummy_map_path # Ensure config uses the created path

    system = MainSystem(config=config)

    try:
        system.start()
        # Let the system run for a short duration for demonstration
        print("\nMainSystem: Running for 10 seconds...\n")
        time.sleep(10)

    except KeyboardInterrupt:
        print("\nMainSystem: KeyboardInterrupt received. Shutting down...")
    finally:
        print("\nMainSystem: Initiating shutdown sequence...")
        system.stop()
        print("=============== System Simulation Ended ===============")