# UR3 Final

Gói ROS 2 Humble mô phỏng UR3e trên Gazebo, nhận diện và sắp xếp khối theo camera, hiển thị cảnh bằng RViz/MoveIt và hỗ trợ nhiệm vụ theo mã số sinh viên. Tên package ROS là `ur3_perception_llm_control`.

## Yêu cầu

Cài ROS 2 Humble cùng các package mô phỏng UR, MoveIt, Gazebo và `ros_gz`. Đặt repo này tại `src/ur3_final` trong workspace ROS, sau đó chạy từ thư mục gốc workspace:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select ur3_perception_llm_control
source install/setup.bash
```

Nếu workspace có một bản khác của `ur3_perception_llm_control`, hãy bỏ bản trùng khỏi đường dẫn build.

## Chạy mô phỏng và nhập nhiệm vụ

Mở hai terminal. Trong **cả hai terminal**, vào workspace, source ROS và workspace overlay, rồi đặt cùng biến môi trường:

```bash
cd ~/ur3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=70 ROS_LOCALHOST_ONLY=1 IGN_PARTITION=ur3_bai03_five_blocks
```

**Terminal 1:** khởi chạy Gazebo, controller, MoveIt và RViz; chưa tự chạy nhiệm vụ:

```bash
ros2 launch ur3_perception_llm_control sorting_demo.launch.py \
  execute_demo:=false launch_rviz:=true
```

**Terminal 2:** khởi chạy giao diện nhập nhiệm vụ. Cần chạy Ollama với endpoint tương thích OpenAI tại địa chỉ bên dưới:

```bash
export NINEROUTER_BASE_URL=http://127.0.0.1:11434/v1
export NINEROUTER_API_KEY=local-ollama
export NINEROUTER_MODEL=qwen2.5:3b
ros2 run ur3_perception_llm_control assignment3_runtime --execute
```

Tại lời nhắc `Task (quit to stop):`, nhập nhiệm vụ cần chạy, ví dụ:

```text
Arrange all objects according to my student ID.
```

Mã số sinh viên cấu hình sẵn là `23020719`. Chỉ chạy một workcell trên mỗi `IGN_PARTITION`.
