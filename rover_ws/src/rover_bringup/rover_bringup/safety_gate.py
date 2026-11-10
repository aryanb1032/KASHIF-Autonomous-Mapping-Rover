import math
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from .core import MotionGate


class SafetyGate(Node):
    def __init__(self):
        super().__init__('safety_gate')
        self.declare_parameter('require_base_status', True)
        self.require_base = self.get_parameter('require_base_status').value
        self.gate = MotionGate()
        self.scan_at = self.odom_at = self.wheel_at = self.base_at = -math.inf
        self.fault = True
        self.pub = self.create_publisher(Twist, '/cmd_vel_safe', 10)
        self.state = self.create_publisher(Bool, '/rover/armed', 10)
        self.create_subscription(Twist, '/cmd_vel', self.command, 10)
        self.create_subscription(Bool, '/rover/permit', self.permit, 10)
        self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odometry/filtered', self.odom, 10)
        self.create_subscription(Odometry, '/wheel/odom', self.wheel, 10)
        self.create_subscription(Bool, '/rover/base_fault', self.base, 10)
        self.create_timer(0.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def healthy(self):
        now = time.monotonic()
        return (now - self.scan_at < 0.6 and now - self.odom_at < 0.4 and now - self.wheel_at < 0.4
                and (not self.require_base or (now - self.base_at < 0.3 and not self.fault)))

    def fresh_stamp(self, stamp, age):
        seconds = self.get_clock().now().nanoseconds / 1e9 - (stamp.sec + stamp.nanosec / 1e9)
        return -0.1 <= seconds <= age

    def scan(self, msg):
        # An empty or entirely invalid scan must not permit driving.
        if self.fresh_stamp(msg.header.stamp, 0.6) and any(math.isfinite(r) and msg.range_min <= r <= msg.range_max for r in msg.ranges):
            self.scan_at = time.monotonic()

    def odom(self, msg):
        if self.fresh_stamp(msg.header.stamp, 0.4) and all(math.isfinite(v) for v in (msg.pose.pose.position.x, msg.pose.pose.position.y,
                                         msg.twist.twist.linear.x, msg.twist.twist.angular.z)):
            self.odom_at = time.monotonic()

    def wheel(self, msg):
        if self.fresh_stamp(msg.header.stamp, 0.4): self.wheel_at = time.monotonic()

    def base(self, msg):
        self.fault = msg.data
        self.base_at = time.monotonic()

    def permit(self, msg):
        self.gate.permit(msg.data, time.monotonic(), self.healthy())
        if msg.data and not self.gate.armed:
            self.get_logger().warning('Disarmed: wait for fresh sensors/base status, then press o followed by w.', throttle_duration_sec=3.0)
        if not msg.data: self.tick()

    def command(self, msg):
        self.gate.command(msg.linear.x, msg.angular.z, time.monotonic())

    def tick(self):
        v, w = self.gate.output(time.monotonic(), self.healthy())
        msg = Twist(); msg.linear.x = v; msg.angular.z = w
        self.pub.publish(msg)
        state = Bool(); state.data = self.gate.armed; self.state.publish(state)


def main():
    rclpy.init(); node = SafetyGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok(): node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
