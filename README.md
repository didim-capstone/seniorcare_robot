# Senior Care Robot

## Overview
This ROS2 package is developed for the Capstone project Senior Care Robot.  
It provides the basic structure for implementing the high-level control logic of a senior care robot.

---

## Operating Environment
- **ROS2 Humble**
- **Ubuntu 22.04**

---

## Requirements

```bash
sudo apt install ros-humble-py-trees
sudo apt install ros-humble-py-trees-ros
```

## LLM
ros2 llama launch ~/colcon_ws/src/llama_ros/llama_bringup/models/StableLM-Zephyr.yaml
ros2 run whisper_trt_node.py base --backend whisper_trt
