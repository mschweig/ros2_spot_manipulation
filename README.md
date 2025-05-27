# Spot Manipulation Interface

## Overview

`spot_manipulation_interface` is a ROS 2 package that enables grasping functionality for Boston Dynamics Spot robots equipped with an arm. The package provides a ROS 2 **action server** that receives bounding box coordinates of detected objects and commands Spot's arm to grasp at the calculated location.

The package supports dynamic camera configurations, processes image-based grasp requests, and integrates directly with Spot's SDK for arm manipulation.

---

## Features

* ROS 2 **Action Server** for object grasping.
* Accepts bounding box input `[xmin, ymin, xmax, ymax]`.
* Parameterized Spot credentials via ROS parameters.
* Dynamic grasp calculation based on bounding box center.
* Integrated with Spot's grasping API.

---

## Requirements

* ROS 2 Humble.
* Boston Dynamics Spot SDK.
* Python 3.8+
* Spot robot with an arm.

---

## Installation

Clone the package into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone <your-repo-url> ros2_spot_manipulation
cd ~/ros2_ws
colcon build --packages-select spot_manipulation_interface
source install/setup.bash
```

---

## Usage

### 1. Launch the Grasp Action Server:

```bash
ros2 launch spot_manipulation_interface grasp_action_server.launch.py param_file:=/path/to/your/robot.yaml
```

The `param_file` should contain Spot's login credentials and connection parameters.

### 2. Send a Grasp Goal:

```bash
ros2 action send_goal /timon/grasp_object spot_manipulation_interface/action/Grasp \
"{x_min: 150, y_min: 580, x_max: 550, y_max: 780, camera_name: 'frontleft'}"
```

---

## Parameters

| Parameter  | Type   | Description                            |
| ---------- | ------ | -------------------------------------- |
| `username` | String | Spot login username                    |
| `password` | String | Spot login password                    |
| `hostname` | String | Spot IP address (e.g., `192.168.50.3`) |

---

## Action Interface

### Grasp.action

```action
# Goal
int32 x_min
int32 y_min
int32 x_max
int32 y_max
string camera_name

# Result
bool success
string message

# Feedback
string current_state
```

---

## License

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at:

[http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0)

---