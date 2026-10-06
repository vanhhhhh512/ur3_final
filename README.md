# UR3 Final

ROS 2 Humble package for the UR3e Gazebo workcell, camera-based cube sorting, RViz/MoveIt scene, and student-ID assignment task. The ROS package name is `ur3_perception_llm_control`.

## Requirements

Install ROS 2 Humble with the UR simulation, MoveIt, Gazebo, and `ros_gz` packages. From the ROS workspace root, install package dependencies and build:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select ur3_perception_llm_control
source install/setup.bash
```

Place this repository at `src/ur3_final` in the ROS workspace before building. If the workspace also contains another checkout of `ur3_perception_llm_control`, remove that duplicate from the build source path.

## Run

Use two terminals. Source ROS and the workspace overlay in each terminal and set the same environment:

```bash
cd ~/ur3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=70 ROS_LOCALHOST_ONLY=1 IGN_PARTITION=ur3_bai03_five_blocks
```

Terminal 1, start Gazebo, controllers, MoveIt, and RViz without starting an automatic task:

```bash
ros2 launch ur3_perception_llm_control robot_sorting_demo.launch.py \
  execute_demo:=false launch_rviz:=true
```

Terminal 2, start the interactive task prompt:

```bash
export NINEROUTER_BASE_URL=http://127.0.0.1:11434/v1
export NINEROUTER_API_KEY=local-ollama
export NINEROUTER_MODEL=qwen2.5:3b
ros2 run ur3_perception_llm_control assignment3_runtime --execute
```

At `Task (quit to stop):`, enter a supported task, for example:

```text
Arrange all objects according to my student ID.
```

The configured student ID is `23020719`. To use the LLM task planner, start a local OpenAI-compatible Ollama endpoint at the configured URL before running Terminal 2. Start only one workcell launch per `IGN_PARTITION`.
