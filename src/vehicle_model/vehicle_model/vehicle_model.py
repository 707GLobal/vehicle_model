import math
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from autoware_msgs.msg import Command
from nav_msgs.msg import Odometry


class VehicleModel(Node):
    def __init__(self):
        super().__init__('vehicle_model')

        # 几何/仿真参数
        self.declare_parameter('wheel_base', 1.53)       # m
        self.declare_parameter('max_steer_angle', 25.0)  # 度
        self.declare_parameter('dt', 0.02)               # 50Hz
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)

        # 动力学参数（默认值按 WUTA 方程式赛车）
        self.declare_parameter('mass', 250.0)            # kg 整车质量(含车手)
        self.declare_parameter('iz', 300.0)              # kg*m^2 横摆转动惯量
        self.declare_parameter('lf', 0.77)               # m 质心到前轴
        self.declare_parameter('cf', 200000.0)           # N/rad 前轴侧偏刚度
        self.declare_parameter('cr', 200000.0)           # N/rad 后轴侧偏刚度
        self.declare_parameter('friction_coeff', 1.2)    # 轮胎附着系数 μ
        self.declare_parameter('tau_steer', 0.1)         # s 转向一阶时间常数
        self.declare_parameter('speed_gain', 2.0)        # 1/s 速度 P 增益
        self.declare_parameter('max_accel', 3.0)         # m/s^2 最大加速度
        self.declare_parameter('max_decel', 5.0)         # m/s^2 最大减速度
        self.declare_parameter('v_kin_thresh', 1.0)      # m/s 低速回退阈值

        self._load_params()

        # 参数运行时调整即时生效（ros2 param set / GUI）
        self.add_on_set_parameters_callback(self._on_set_params)

        # 状态
        self.x = self.start_x
        self.y = self.start_y
        self.yaw = self.start_yaw
        self.v = 0.0           # 纵向速度 m/s
        self.r = 0.0           # 横摆角速度 rad/s
        self.beta = 0.0        # 质心侧偏角 rad
        self.steer = 0.0       # 实际前轮转角 rad
        self.cmd_speed = 0.0   # 指令速度 m/s
        self.cmd_angle = 0.0   # 指令转角 度
        self._kin_mode = True  # 低速运动学 / 高速动力学（滞回）

        # 订阅控制指令 (#6 /control/command)
        self.sub = self.create_subscription(
            Command, '/control/command', self.on_command, 50)

        # 发布 ground truth (#1 /sim/ground_truth)
        self.pub = self.create_publisher(
            Odometry, '/sim/ground_truth', 50)

        # 定时更新 50Hz
        self.timer = self.create_timer(self.dt, self.step)

    def _load_params(self):
        """读取全部参数到实例变量"""
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_steer = math.radians(
            self.get_parameter('max_steer_angle').value)
        self.dt = self.get_parameter('dt').value
        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.start_yaw = self.get_parameter('start_yaw').value
        self.mass = self.get_parameter('mass').value
        self.iz = self.get_parameter('iz').value
        self.lf = self.get_parameter('lf').value
        self.cf = self.get_parameter('cf').value
        self.cr = self.get_parameter('cr').value
        self.friction_coeff = self.get_parameter('friction_coeff').value
        self.tau_steer = self.get_parameter('tau_steer').value
        self.speed_gain = self.get_parameter('speed_gain').value
        self.max_accel = self.get_parameter('max_accel').value
        self.max_decel = self.get_parameter('max_decel').value
        self.v_kin_thresh = self.get_parameter('v_kin_thresh').value
        self._update_derived()

    def _update_derived(self):
        """根据 lf/wheel_base 更新派生量"""
        self.lr = max(0.05, self.wheel_base - self.lf)

    def _on_set_params(self, params):
        for p in params:
            if p.name == 'wheel_base':
                self.wheel_base = p.value
            elif p.name == 'max_steer_angle':
                self.max_steer = math.radians(p.value)
            elif p.name == 'dt':
                self.dt = p.value
            elif p.name == 'mass':
                self.mass = p.value
            elif p.name == 'iz':
                self.iz = p.value
            elif p.name == 'lf':
                self.lf = p.value
            elif p.name == 'cf':
                self.cf = p.value
            elif p.name == 'cr':
                self.cr = p.value
            elif p.name == 'friction_coeff':
                self.friction_coeff = p.value
            elif p.name == 'tau_steer':
                self.tau_steer = p.value
            elif p.name == 'speed_gain':
                self.speed_gain = p.value
            elif p.name == 'max_accel':
                self.max_accel = p.value
            elif p.name == 'max_decel':
                self.max_decel = p.value
            elif p.name == 'v_kin_thresh':
                self.v_kin_thresh = p.value
        self._update_derived()
        return SetParametersResult(successful=True)

    def on_command(self, msg: Command):
        s = msg.speed          # m/s
        a = msg.angle          # 度
        s_ok = isinstance(s, (int, float)) and math.isfinite(s)
        a_ok = isinstance(a, (int, float)) and math.isfinite(a)
        if not s_ok:
            self.get_logger().warn(
                'Invalid speed=%s (not finite), keeping previous speed=%.3f m/s'
                % (s, self.cmd_speed))
        if not a_ok:
            self.get_logger().warn(
                'Invalid angle=%s (not finite), keeping previous angle=%.1f deg'
                % (a, self.cmd_angle))
        if not s_ok or not a_ok:
            return
        self.cmd_speed = s
        self.cmd_angle = a

    def step(self):
        dt = self.dt

        # 1. 转向执行器一阶惯性
        steer_cmd = math.radians(self.cmd_angle)
        self.steer += (steer_cmd - self.steer) * dt / max(self.tau_steer, 1e-4)
        self.steer = max(-self.max_steer,
                         min(self.steer, self.max_steer))

        # 2. 速度跟踪（P 控制 + 加速度限幅）
        accel = (self.cmd_speed - self.v) * self.speed_gain
        accel = max(-self.max_decel, min(accel, self.max_accel))

        # 3. 运动学/动力学切换（滞回，避免阈值附近抖动）
        if abs(self.v) < self.v_kin_thresh:
            self._kin_mode = True
        elif abs(self.v) > 1.5 * self.v_kin_thresh:
            self._kin_mode = False

        if self._kin_mode:
            # 低速运动学，避免除以 v≈0 发散
            self.x += self.v * math.cos(self.yaw) * dt
            self.y += self.v * math.sin(self.yaw) * dt
            self.yaw += self.v * math.tan(self.steer) / self.wheel_base * dt
            self.r = self.v * math.tan(self.steer) / self.wheel_base
            self.beta = 0.0
            self.v += accel * dt
        else:
            # 4. 轮胎侧偏角（小角度近似）
            v_safe = max(abs(self.v), 0.5)   # 下限保护
            alpha_f = self.steer - self.beta - self.lf * self.r / v_safe
            alpha_r = -self.beta + self.lr * self.r / v_safe
            # 5. 线性轮胎力 + 摩擦圆饱和（Fy ≤ μ·Fz，防止数值发散）
            fz_f = self.mass * 9.81 * self.lr / max(self.wheel_base, 1e-3)
            fz_r = self.mass * 9.81 * self.lf / max(self.wheel_base, 1e-3)
            fyf_max = self.friction_coeff * fz_f
            fyr_max = self.friction_coeff * fz_r
            f_yf = fyf_max * math.tanh(self.cf * alpha_f / max(fyf_max, 1e-3))
            f_yr = fyr_max * math.tanh(self.cr * alpha_r / max(fyr_max, 1e-3))
            # 6. 动力学更新
            self.v += accel * dt
            self.beta += ((f_yf * math.cos(self.steer) + f_yr)
                          / (self.mass * v_safe) - self.r) * dt
            self.r += (f_yf * self.lf * math.cos(self.steer)
                       - f_yr * self.lr) / self.iz * dt
            self.beta = max(-1.0, min(self.beta, 1.0))   # 防发散
            self.r = max(-3.0, min(self.r, 3.0))
            # 7. 位置积分（速度方向 = yaw + beta）
            self.yaw += self.r * dt
            self.x += self.v * math.cos(self.yaw + self.beta) * dt
            self.y += self.v * math.sin(self.yaw + self.beta) * dt

        now = self.get_clock().now().to_msg()

        # 发布 Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.r
        self.pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleModel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down VehicleModel...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
