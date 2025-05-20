# Autonomous Driving System Design ([Doc.md](http://doc.md/))

This document outlines the modular design for an autonomous driving system based on the provided diagrams (Fig. 2 and Fig. 3). The system is designed with threading for concurrent operation and modularity for clarity and maintainability.

## Table of Contents

1. [Core Modules Overview](https://www.notion.so/1f4d05ac5cba8051b977feba0300ebfc?pvs=21)
2. [Data Structures (`data_structures.py`)](https://www.notion.so/1f4d05ac5cba8051b977feba0300ebfc?pvs=21)
3. [Sensor Input Module (`sensor_input_module.py`)](https://www.notion.so/1f4d05ac5cba8051b977feba0300ebfc?pvs=21)
4. [Perception Module (`perception_module.py`)](https://www.notion.so/1f4d05ac5cba8051b977feba0300ebfc?pvs=21)
    - [Detection Component](https://www.notion.so/1f4d05ac5cba8051b977feba0300ebfc?pvs=21)
    - [Scene Understanding Component](https://www.notion.so/1f4d05ac5cba8051b977feba0300ebfc?pvs=21)
    - [Tracking Component](https://www.notion.so/1f4d05ac5cba8051b977feba0300ebfc?pvs=21)
    - [Perception Prediction Component](https://www.notion.so/1f4d05ac5cba8051b977feba0300ebfc?pvs=21)
5. [Localization Module (`localization_module.py`)](https://www.notion.so/1f4d05ac5cba8051b977feba0300ebfc?pvs=21)
6. [Prediction Module (Behavioral) (`prediction_module.py`)](https://www.notion.so/1f4d05ac5cba8051b977feba0300ebfc?pvs=21)
7. [Planning Module (`planning_module.py`)](https://www.notion.so/1f4d05ac5cba8051b977feba0300ebfc?pvs=21)
    - [Path Planner Component](https://www.notion.so/1f4d05ac5cba8051b977feba0300ebfc?pvs=21)
    - [Decision Maker Component](https://www.notion.so/1f4d05ac5cba8051b977feba0300ebfc?pvs=21)
    - [Action Planner Component](https://www.notion.so/1f4d05ac5cba8051b977feba0300ebfc?pvs=21)
8. [Control Module (`control_module.py`)](https://www.notion.so/1f4d05ac5cba8051b977feba0300ebfc?pvs=21)
9. [Main System Orchestration (`main_system.py`)](https://www.notion.so/1f4d05ac5cba8051b977feba0300ebfc?pvs=21)
10. [Inter-Module Communication](https://www.notion.so/1f4d05ac5cba8051b977feba0300ebfc?pvs=21)
11. [Threading Model](https://www.notion.so/1f4d05ac5cba8051b977feba0300ebfc?pvs=21)

---

## 1. Core Modules Overview

The system follows the high-level architecture depicted in Fig. 2, with the Perception module's internal structure detailed by Fig. 3.

- **Sensor Input**: Acquires data from various sensors (LiDAR, Vision, GNSS, IMU).
- **Perception**: Processes sensor data to understand the environment. This includes detection, segmentation, tracking, and short-term prediction of object states.
- **Localization**: Estimates the vehicle's precise position and orientation in the world, often using an HD Map and sensor data.
- **Prediction (Behavioral)**: Forecasts the future behavior and trajectories of other dynamic agents in the scene.
- **Planning**: Determines the vehicle's future course of action, encompassing path planning, maneuver decision-making, and motion/action planning.
- **Control**: Translates the planned actions into commands for the vehicle's actuators.

---

## 2. Data Structures (`data_structures.py`)

This file defines common `NamedTuple` classes for data exchange between modules, ensuring clear and consistent interfaces.

- `SensorData`: Holds raw/pre-processed data from all sensors for a given timestamp.
- `DetectedObject`: Information about a detected object (ID, type, position, velocity, etc.).
- `LaneMarking`: Describes detected lane lines.
- `TrafficSignInfo`: Information about detected traffic signs.
- `PerceptionOutput`: Aggregated output from the Perception module.
- `LocalizationInfo`: Vehicle's estimated pose (position, orientation) and velocity.
- `PredictedTrajectory`: A potential future path for a dynamic agent.
- `BehavioralPredictionOutput`: Collection of predicted trajectories for agents in the scene.
- `PlannedPath`: Sequence of waypoints defining the vehicle's intended geometric path.
- `ManeuverDecision`: High-level driving maneuver selected by the planner.
- `ActionCommand`: Low-level motion commands (target velocity, steering angle) for the Control module.
- `ControlActuatorCommands`: Specific commands for vehicle actuators (steering, throttle, brake).

---

## 3. Sensor Input Module (`sensor_input_module.py`)

Handles data acquisition from sensors like LiDAR, cameras, GNSS, and IMU.

### Class: `SensorInputManager`

- **Purpose**: Manages initialization of sensors and continuous data fetching.
- **Interface**:
    - `__init__(self, config: dict, output_queue: queue.Queue)`
    - `start_sensors(self)`: Starts data acquisition in a separate thread.
    - `stop_sensors(self)`: Stops data acquisition.
- **Output**: Pushes `SensorData` objects to its `output_queue`.

---

## 4. Perception Module (`perception_module.py`)

Processes `SensorData` to build an understanding of the vehicle's surroundings. Its internal structure is guided by Fig. 3.

### Class: `PerceptionModule`

- **Purpose**: Orchestrates various perception sub-tasks.
- **Interface**:
    - `__init__(self, config: dict, input_queue: queue.Queue, output_queues: Dict[str, queue.Queue])`
    - `start(self)`: Starts the perception processing thread.
    - `stop(self)`: Stops the thread.
    - `run(self)`: Main loop consuming `SensorData` and producing `PerceptionOutput`.
- **Input**: `SensorData` from `input_queue`.
- **Output**: `PerceptionOutput` distributed to `output_queues` (for Localization, Prediction, Planning modules).

Internal components managed by `PerceptionModule`:

### Detection Component

- **Class**: `DetectionComponent`
- **Purpose**: Implements tasks from Fig. 3's "Detection" branch (Lane Detection, Drivable Region, Traffic Sign Detection, Visual 3D Detection, LiDAR 3D Detection, Fusion 3D Detection).
- **Interface**: `process(self, sensor_data: SensorData, depth_map: Optional[Any]) -> Tuple[List[DetectedObject], List[LaneMarking], Any, List[TrafficSignInfo]]`

### Scene Understanding Component

- **Class**: `SceneUnderstandingComponent`
- **Purpose**: Implements tasks from Fig. 3's "Scene Understanding" branch (Semantic/Instance/Panoptic Segmentation, Depth/Optical Flow/Scene Flow Estimation).
- **Interface**: `process(self, sensor_data: SensorData, previous_frame_data: Optional[Any] = None) -> Tuple[Optional[Any], ...]`

### Tracking Component

- **Class**: `TrackingComponent`
- **Purpose**: Implements tasks from Fig. 3's "Tracking" branch (Traditional Tracking, Neural Network Tracking). Associates detections over time.
- **Interface**: `process(self, detected_objects: List[DetectedObject], timestamp: float) -> List[DetectedObject]`

### Perception Prediction Component (Short-term)

- **Class**: `PerceptionPredictionComponent`
- **Purpose**: Implements tasks from Fig. 3's "Prediction" branch (Model-based Prediction, Data-driven Prediction). This is for short-term state estimation of tracked objects (e.g., position/velocity in next few frames).
- **Interface**: `process(self, tracked_objects: List[DetectedObject]) -> List[DetectedObject]`

---

## 5. Localization Module (`localization_module.py`)

Estimates the vehicle's global position and orientation. Corresponds to "Localization" in Fig. 2 and uses sub-tasks from Fig. 3's "Localization" branch (GNSS/IMU, Visual SLAM, LiDAR SLAM, Fusion SLAM).

### Class: `HDMapInterface` (Helper)

- **Purpose**: Provides an interface to access High-Definition map data.
- **Interface**: `get_local_map_data(self, position, extent)`

### Class: `LocalizationModule`

- **Purpose**: Fuses sensor data (from Perception module and/or direct sensors) and HD map information to produce an accurate pose.
- **Interface**:
    - `__init__(self, config: dict, hd_map_path: str, input_queue_perception: queue.Queue, input_queue_sensor: Optional[queue.Queue], output_queues: Dict[str, queue.Queue])`
    - `start(self)`: Starts the localization thread.
    - `stop(self)`: Stops the thread.
    - `run(self)`: Main loop consuming `PerceptionOutput` (and optionally direct `SensorData`) and producing `LocalizationInfo`.
- **Input**: `PerceptionOutput` (containing features, object lists, etc.) and/or direct `SensorData` (GNSS/IMU).
- **Output**: `LocalizationInfo` to `output_queues` (for Prediction and Planning modules).

---

## 6. Prediction Module (Behavioral) (`prediction_module.py`)

Forecasts the future behavior and trajectories of other dynamic agents (vehicles, pedestrians) in the scene. Corresponds to the "Prediction" block in Fig. 2 that takes inputs from Perception and Localization.

### Class: `PredictionModule`

- **Purpose**: Predicts long-term intentions and paths of other road users.
- **Interface**:
    - `__init__(self, config: dict, input_queue_perception: queue.Queue, input_queue_localization: queue.Queue, output_queue: queue.Queue)`
    - `start(self)`: Starts the prediction thread.
    - `stop(self)`: Stops the thread.
    - `run(self)`: Main loop consuming `PerceptionOutput` and `LocalizationInfo`, producing `BehavioralPredictionOutput`.
- **Input**: `PerceptionOutput` (tracked objects), `LocalizationInfo` (ego-vehicle state).
- **Output**: `BehavioralPredictionOutput` to its `output_queue` (for Planning module).

---

## 7. Planning Module (`planning_module.py`)

Determines the ego-vehicle's future path and actions based on its current state, environmental understanding, and predictions of other agents. Corresponds to the "Planning" block in Fig. 2.

### Class: `PlanningModule`

- **Purpose**: Orchestrates path planning, decision making, and action planning.
- **Interface**:
    - `__init__(self, config: dict, hd_map_path: str, input_queues: Dict[str, queue.Queue], output_queue_control: queue.Queue)`
    - `start(self)`: Starts the planning thread.
    - `stop(self)`: Stops the thread.
    - `run(self)`: Main loop consuming inputs and producing `ActionCommand`.
- **Input**: `LocalizationInfo`, `BehavioralPredictionOutput`, and `PerceptionOutput` (for static scene context).
- **Output**: `ActionCommand` to `output_queue_control`.

Internal components managed by `PlanningModule`:

### Path Planner Component

- **Class**: `PathPlannerComponent`
- **Purpose**: Generates a collision-free geometric path (route) for the ego-vehicle.
- **Interface**: `plan_path(self, current_pose: LocalizationInfo, scene_info: PerceptionOutput, behavioral_predictions: BehavioralPredictionOutput) -> PlannedPath`

### Decision Maker Component

- **Class**: `DecisionMakerComponent`
- **Purpose**: Selects high-level driving maneuvers (e.g., lane keeping, lane changing, overtaking) based on the planned path, predictions, and rules.
- **Interface**: `make_decision(self, current_pose: LocalizationInfo, planned_path: PlannedPath, behavioral_predictions: BehavioralPredictionOutput, scene_info: PerceptionOutput) -> ManeuverDecision`

### Action Planner Component

- **Class**: `ActionPlannerComponent`
- **Purpose**: Converts the decided maneuver and path into a detailed, drivable trajectory, often resulting in target velocity and steering angle profiles over a short horizon.
- **Interface**: `plan_action(self, current_pose: LocalizationInfo, decision: ManeuverDecision, planned_path: PlannedPath) -> ActionCommand`

---

## 8. Control Module (`control_module.py`)

Executes the planned actions by sending low-level commands to the vehicle's actuators (steering, throttle, brakes).

### Class: `VehicleInterface` (Helper)

- **Purpose**: Abstracts the communication with the vehicle's hardware.
- **Interface**: `send_commands(self, steering: float, throttle: float, brake: float)`

### Class: `ControlModule`

- **Purpose**: Translates `ActionCommand` (e.g., target velocity, steering angle) into `ControlActuatorCommands`.
- **Interface**:
    - `__init__(self, config: dict, input_queue_planning: queue.Queue, vehicle_interface_config: dict)`
    - `start(self)`: Starts the control thread.
    - `stop(self)`: Stops the thread.
    - `run(self)`: Main loop consuming `ActionCommand` and sending commands via `VehicleInterface`.
- **Input**: `ActionCommand` from `input_queue_planning`.
- **Output**: Commands sent to the vehicle via `VehicleInterface`.

---

## 9. Main System Orchestration (`main_system.py`)

Initializes all modules, sets up inter-module communication queues, and manages the lifecycle (start, stop) of their respective threads.

### Class: `MainSystem`

- **Purpose**: The central coordinator of the autonomous driving pipeline.
- **Interface**:
    - `__init__(self, config: dict)`
    - `start(self)`: Initializes and starts all module threads.
    - `stop(self)`: Signals all module threads to terminate gracefully and joins them.
- **Functionality**: Loads configurations, creates communication queues, instantiates all modules, and manages their execution.

---

## 10. Inter-Module Communication

Communication between modules is primarily achieved using thread-safe `queue.Queue` objects from Python's `queue` library. This decouples the modules and allows them to operate asynchronously at their own processing rates.

- `SensorInputManager` -> `PerceptionModule` (via `sensor_to_perception_queue`)
- `PerceptionModule` -> `LocalizationModule` (via `perception_to_localization_queue`)
- `PerceptionModule` -> `PredictionModule` (via `perception_to_prediction_queue`)
- `PerceptionModule` -> `PlanningModule` (via `perception_to_planning_queue` for direct scene context)
- `LocalizationModule` -> `PredictionModule` (via `localization_to_prediction_queue`)
- `LocalizationModule` -> `PlanningModule` (via `localization_to_planning_queue`)
- `PredictionModule` -> `PlanningModule` (via `prediction_to_planning_queue`)
- `PlanningModule` -> `ControlModule` (via `planning_to_control_queue`)

Each queue typically holds objects of the data structures defined in `data_structures.py`.

---

## 11. Threading Model

Each core module (`PerceptionModule`, `LocalizationModule`, `PredictionModule`, `PlanningModule`, `ControlModule`) and the `SensorInputManager` is designed to run in its own dedicated thread. This allows for parallel processing of different stages of the autonomous driving pipeline.

- The `MainSystem` class is responsible for creating and managing these threads.
- `start()` methods in each module typically initialize and start their internal processing thread.
- `stop()` methods signal the thread to terminate and then `join()` the thread to ensure clean shutdown.
- Queues are used with timeouts for `get()` and `put()` operations to prevent indefinite blocking and allow threads to check for termination signals.

This multi-threaded design aims to improve responsiveness and throughput of the system, though careful consideration of synchronization, data consistency, and potential race conditions (largely mitigated by queue usage) is necessary for robust implementation.