import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from autoware_msgs.msg import Command
import math

class VehicleModel(Node):
    def __init__(self):
        super().__init__('vehicle_model')
        
        # 参数
        self.declare_parameter('wheel_base', 1.53)        # m
        self.declare_parameter('max_steer_angle', 25.0)   # 度
        self.declare_parameter('dt', 0.02)                # 50Hz
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)
        
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_steer = math.radians(
            self.get_parameter('max_steer_angle').value)
        self.dt = self.get_parameter('dt').value
        
        # 状态
        self.x = self.get_parameter('start_x').value
        self.y = self.get_parameter('start_y').value
        self.yaw = self.get_parameter('start_yaw').value
        self.speed = 0.0
        self.angle = 0.0
        
        # 订阅控制指令
        self.sub = self.create_subscription(
            Command, '/control/command', self.on_command, 10)
        
        # 发布ground truth pose
        self.pub = self.create_publisher(
            PoseStamped, '/localization/pose', 10)
        
        # 定时更新
        self.timer = self.create_wall_timer(
            self.dt, self.step)
    
    def on_command(self, msg: Command):
        self.speed = msg.speed
        self.angle = msg.angle  # 单位是度
    
    def step(self):
        # 限幅转向角
        steer = max(-self.max_steer, 
                    min(math.radians(self.angle), self.max_steer))
        
        # 自行车运动学更新
        self.x += self.speed * math.cos(self.yaw) * self.dt
        self.y += self.speed * math.sin(self.yaw) * self.dt
        self.yaw += self.speed * math.tan(steer) / self.wheel_base * self.dt
        
        now = self.get_clock().now().to_msg()
        
        # 发布PoseStamped
        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = 'map'
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        # yaw → 四元数
        pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        self.pub.publish(pose)