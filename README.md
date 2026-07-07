# vehicle_model
## 1. 车辆运动学模型 (`vehicle_model.py`)
*   **功能定位**：仿真器的“物理引擎”，根据控制指令计算车辆下一时刻的真实位姿（Ground Truth）。
*   **输入**：订阅 `/control/command` (`autoware_msgs/Command`)，获取目标车速 (m/s) 和转角 (degrees)。
*   **输出**：发布 `/sim/ground_truth` (`nav_msgs/Odometry`)，提供车辆绝对位姿和速度。
*   **核心算法**：
    *   采用**自行车运动学模型**（Wheelbase = 1.53m，dt = 0.02s）。
    *   转角限幅：`steer = clamp(command.angle, -25°, 25°)`。
    *   状态更新：
        *   $x_{t+1} = x_t + v \cdot \cos(\theta) \cdot dt$
        *   $y_{t+1} = y_t + v \cdot \sin(\theta) \cdot dt$
        *   $\theta_{t+1} = \theta_t + \frac{v \cdot \tan(steer)}{L} \cdot dt$
*   **关键约束**：`start_pose` 必须严格读取并对齐赛道 YAML 文件中的初始位姿，确保与感知/定位组的坐标系原点完全重合。
