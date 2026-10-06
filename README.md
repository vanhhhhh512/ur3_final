# UR3 Final

Package ROS 2 Humble mô phỏng UR3e trong Gazebo Fortress, nhận diện và sắp xếp khối bằng camera RGB, MoveIt 2 và bộ lập kế hoạch LLM. Tên package ROS là `llm`.

## Yêu cầu

Cần Ubuntu 22.04, ROS 2 Humble, `ur_simulation_gz`, `ur_moveit_config`, MoveIt 2, Gazebo Fortress, `ros_gz`, OpenCV, NumPy và Ollama với model `qwen2.5:3b`.

Đặt repository trong thư mục `src/ur3_final` của workspace ROS. Package nằm ở `src/ur3_final/ros_package`. Từ thư mục gốc workspace, chạy:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select llm
source install/setup.bash
```

Nếu workspace đã có package ROS tên `llm`, chỉ giữ một bản trong đường dẫn build để tránh package trùng.

## Chạy mô phỏng

Mở hai terminal. Chạy các lệnh môi trường sau trong **cả hai terminal**, từ thư mục gốc workspace:

```bash
cd ~/ur3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=70
export ROS_LOCALHOST_ONLY=1
export IGN_PARTITION=ur3_bai03_five_blocks
```

**Terminal 1 — Gazebo, controller, MoveIt và RViz:**

```bash
ros2 launch llm sorting_demo.launch.py \
  execute_demo:=false launch_rviz:=true
```

**Terminal 2 — giao diện nhập task:** bảo đảm Ollama đang chạy và model đã tải, sau đó chạy:

```bash
export NINEROUTER_BASE_URL=http://127.0.0.1:11434/v1
export NINEROUTER_API_KEY=local-ollama
export NINEROUTER_MODEL=qwen2.5:3b
ros2 run llm assignment3_runtime --execute
```

Khi hiện `Task (quit to stop):`, nhập yêu cầu, ví dụ:

```text
Arrange all objects according to my student ID.
```

MSSV mặc định trong cấu hình là `23020719` (P=1: Zone A=red, Zone B=blue, Zone C=yellow). Chỉ khởi chạy một workcell trong mỗi `IGN_PARTITION`. Để dừng giao diện, nhập `quit`; dừng launch ở Terminal 1 bằng `Ctrl+C`.

## Cấu trúc package

- `ros_package/src/`: mã Python của package `llm`.
- `ros_package/assets/`: cấu hình, launch, prompt, RViz, URDF/SRDF, world và marker ament; `setup.py` dựng lại cây ROS chuẩn khi build.
- `ros_package/package.xml`, `setup.py`, `setup.cfg`: metadata, entry point và cài đặt tài nguyên.
