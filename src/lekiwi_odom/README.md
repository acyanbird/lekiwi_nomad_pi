# LeKiwi 全向三轮机器人 Odom 节点

这个包提供了一个专门为全向三轮机器人设计的里程计节点，能够读取三个全向轮的编码器数据并发布里程计信息。

## 功能特性

- 支持全向三轮机器人运动学
- 读取三个轮子的编码器数据
- 计算并发布里程计信息 (`/odom`)
- 发布 TF 变换 (`odom` → `base_link`)
- 发布关节状态 (`/joint_states`)
- 可配置的机器人参数

## 全向三轮机器人配置

全向三轮机器人通常采用等边三角形布局，三个轮子分别位于：
- 轮子1：0° (前方)
- 轮子2：120° (右后方)
- 轮子3：240° (左后方)

## 参数配置

| 参数名 | 默认值 | 描述 |
|--------|--------|------|
| `wheel_radius` | 0.05 | 轮子半径 (米) |
| `robot_radius` | 0.15 | 机器人半径 (轮子到中心的距离，米) |
| `wheel_angle_1` | 0.0 | 第一个轮子的角度 (弧度) |
| `wheel_angle_2` | 2π/3 | 第二个轮子的角度 (弧度) |
| `wheel_angle_3` | 4π/3 | 第三个轮子的角度 (弧度) |
| `ticks_per_revolution` | 360 | 每转编码器脉冲数 |
| `publish_rate` | 50.0 | 发布频率 (Hz) |

## 发布的话题

- `/odom` (nav_msgs/Odometry): 里程计信息
- `/joint_states` (sensor_msgs/JointState): 三个轮子的关节状态
- `/tf` (tf2_msgs/TFMessage): TF 变换

## 使用方法

### 1. 构建包

```bash
cd ~/lekiwi/lekiwi_ws
colcon build --packages-select lekiwi_odom
source install/setup.bash
```

### 2. 运行节点

#### 方法一：直接运行
```bash
ros2 run lekiwi_odom omni_wheel_odom_node
```

#### 方法二：使用启动文件
```bash
ros2 launch lekiwi_odom odom_launch.py
```

#### 方法三：带参数运行
```bash
ros2 launch lekiwi_odom odom_launch.py \
    wheel_radius:=0.04 \
    robot_radius:=0.12 \
    ticks_per_revolution:=720
```

### 3. 查看发布的数据

```bash
# 查看里程计信息
ros2 topic echo /odom

# 查看关节状态
ros2 topic echo /joint_states

# 查看 TF 变换
ros2 run tf2_tools view_frames
```

## 全向轮运动学

节点使用全向轮运动学矩阵来计算机器人速度：

```
[v_x]   [cos(θ1)  sin(θ1)  L] [ω1]
[v_y] = [cos(θ2)  sin(θ2)  L] [ω2]
[ω_z]   [cos(θ3)  sin(θ3)  L] [ω3]
```

其中：
- v_x, v_y: 机器人线速度
- ω_z: 机器人角速度
- θ1, θ2, θ3: 三个轮子的角度
- L: 机器人半径
- ω1, ω2, ω3: 三个轮子的角速度

## 自定义编码器读取

当前节点使用模拟的编码器数据。要使用真实的编码器数据，请修改 `read_wheel_encoders()` 方法：

```python
def read_wheel_encoders(self):
    """
    读取三个轮子的编码器数据
    替换为您的实际编码器读取代码
    """
    # 示例：从 GPIO 或串口读取编码器数据
    # encoder_1 = read_encoder_1_from_hardware()
    # encoder_2 = read_encoder_2_from_hardware()
    # encoder_3 = read_encoder_3_from_hardware()
    
    # 当前使用模拟数据
    import random
    for i in range(3):
        self.encoders[i] += random.randint(-3, 3)
    
    return self.encoders.copy()
```

## 调试

启用调试日志：
```bash
ros2 run lekiwi_odom omni_wheel_odom_node --ros-args --log-level DEBUG
```

## 注意事项

1. 确保轮子角度参数与实际机器人布局匹配
2. 机器人半径参数影响里程计精度
3. 当前使用模拟数据，实际使用时需要替换为真实的编码器读取代码
4. 全向轮运动学计算需要 numpy 库支持

## 依赖项

- rclpy
- geometry_msgs
- nav_msgs
- sensor_msgs
- tf2_ros
- numpy 