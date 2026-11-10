"""Bounded, nonblocking serial transport. Lost commands cannot be replayed forever."""
import math
import time
import serial
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int64MultiArray
from .core import frame, unframe, counter_delta, integrate


class BaseBridge(Node):
    def __init__(self):
        super().__init__('base_bridge')
        defaults = {'port': '/dev/rover_esp32', 'baud': 115200, 'wheel_radius': 0.05,
                    'track_width': 0.32, 'encoder_ticks_per_wheel_rev': 1680.0,
                    'hardware_calibrated': False, 'imu_gyro_z_bias': 0.0}
        for name, value in defaults.items(): self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value
        self.r, self.b, self.tpr = p('wheel_radius'), p('track_width'), p('encoder_ticks_per_wheel_rev')
        if not all(math.isfinite(v) and v > 0 for v in (self.r, self.b, self.tpr)):
            raise ValueError('Wheel radius, track width, and encoder ticks must be positive')
        self.calibrated, self.bias = p('hardware_calibrated'), p('imu_gyro_z_bias')
        self.port, self.baud = p('port'), p('baud')
        self.ser = None; self.buffer = bytearray(); self.retry_at = 0.0
        self.previous = None; self.telemetry_at = self.command_at = -math.inf
        self.v = self.w = self.x = self.y = self.yaw = 0.0
        self.fault = True
        self.odom = self.create_publisher(Odometry, '/wheel/odom', 20)
        self.imu = self.create_publisher(Imu, '/imu/data_raw', 20)
        self.joints = self.create_publisher(JointState, '/joint_states', 20)
        self.status = self.create_publisher(Bool, '/rover/base_fault', 10)
        self.counts = self.create_publisher(Int64MultiArray, '/rover/encoder_ticks', 10)
        self.create_subscription(Twist, '/cmd_vel_safe', self.command, 10)
        self.create_timer(0.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        if not self.calibrated:
            self.get_logger().warning('Motor output locked: hardware_calibrated is false. Telemetry remains available.')

    def command(self, msg):
        if all(math.isfinite(v) for v in (msg.linear.x, msg.angular.z)):
            self.v, self.w, self.command_at = msg.linear.x, msg.angular.z, time.monotonic()
        else:
            self.v = self.w = 0.0; self.command_at = -math.inf

    def disconnect(self):
        if self.ser:
            self.ser.close()
        self.ser = None; self.previous = None; self.buffer.clear()
        self.telemetry_at = self.command_at = -math.inf; self.fault = True
        self.retry_at = time.monotonic() + 1.0

    def tick(self):
        now = time.monotonic()
        try:
            if self.ser is None and now >= self.retry_at:
                self.ser = serial.Serial(self.port, self.baud, timeout=0, write_timeout=0.05)
                self.ser.reset_input_buffer(); self.command_at = -math.inf
                self.get_logger().info('Connected to ' + self.port)
            if self.ser:
                self.buffer.extend(self.ser.read(min(self.ser.in_waiting, 4096)))
                while b'\n' in self.buffer:
                    raw, _, rest = self.buffer.partition(b'\n'); self.buffer = bytearray(rest)
                    self.telemetry(raw)
                if len(self.buffer) > 512:
                    self.buffer.clear(); self.fault = True
                fresh = now - self.command_at <= 0.25 and now - self.telemetry_at <= 0.25
                v, w = (self.v, self.w) if self.calibrated and fresh and not self.fault else (0.0, 0.0)
                scale = max(1.0, abs(v) / 0.18, abs(w) / 0.6)
                v /= scale; w /= scale
                k = self.tpr / (2 * math.pi * self.r)
                left, right = (v-w*self.b/2)*k, (v+w*self.b/2)*k
                self.ser.write(frame(f'C,{left:.3f},{right:.3f}'))
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=3.0); self.disconnect()
        status = Bool()
        status.data = not self.calibrated or self.fault or now-self.telemetry_at > 0.25
        self.status.publish(status)

    def telemetry(self, raw):
        try:
            fields = unframe(raw)
            if len(fields) != 12 or fields[0] != 'T': return
            ms, left, right, imu_ok, fault = map(int, fields[1:6])
            values = list(map(float, fields[6:]))
            if not all(math.isfinite(v) for v in values): return
            if not 0 <= ms < 2**32 or imu_ok not in (0, 1) or fault not in (0, 1): return
        except (ValueError, UnicodeError): return
        now = time.monotonic()
        self.fault = bool(fault)
        counts = Int64MultiArray(); counts.data = [left, right]; self.counts.publish(counts)
        if not imu_ok: self.get_logger().warning('MPU9250 unavailable; EKF has wheel odometry only.', throttle_duration_sec=3.0)
        stamp = self.get_clock().now().to_msg()
        if imu_ok:
            im = Imu(); im.header.stamp = stamp; im.header.frame_id = 'imu_link'
            im.orientation_covariance[0] = -1.0
            im.linear_acceleration.x, im.linear_acceleration.y, im.linear_acceleration.z = values[:3]
            im.angular_velocity.x, im.angular_velocity.y, im.angular_velocity.z = values[3:]
            im.angular_velocity.z -= self.bias
            im.linear_acceleration_covariance = [0.08,0.,0.,0.,0.08,0.,0.,0.,0.08]
            im.angular_velocity_covariance = [0.02,0.,0.,0.,0.02,0.,0.,0.,0.03]
            self.imu.publish(im)
        previous = self.previous
        self.previous = ms, left, right
        if previous is None or now-self.telemetry_at > 0.3:
            self.telemetry_at = now; self.command_at = -math.inf
            return
        dt = ((ms-previous[0]) % 2**32) / 1000.0
        dl, dr = counter_delta(left, previous[1]), counter_delta(right, previous[2])
        self.telemetry_at = now
        if not 0.005 <= dt <= 0.3 or max(abs(dl),abs(dr)) > 22000*dt:
            self.command_at = -math.inf; self.fault = True; return
        k = 2*math.pi*self.r/self.tpr
        self.x,self.y,self.yaw,ds,da = integrate(self.x,self.y,self.yaw,dl*k,dr*k,self.b)
        od = Odometry(); od.header.stamp = stamp; od.header.frame_id = 'odom'; od.child_frame_id = 'base_link'
        od.pose.pose.position.x = self.x; od.pose.pose.position.y = self.y
        od.pose.pose.orientation.z = math.sin(self.yaw/2); od.pose.pose.orientation.w = math.cos(self.yaw/2)
        od.twist.twist.linear.x = ds/dt; od.twist.twist.angular.z = da/dt
        for i in (0,7,14,21,28,35):
            od.pose.covariance[i] = 0.05 if i in (0,7,35) else 1e6
            od.twist.covariance[i] = 0.05 if i in (0,35) else 1e6
        od.twist.covariance[7] = 0.01  # Nonholonomic lateral velocity constraint.
        self.odom.publish(od)
        js = JointState(); js.header.stamp = stamp; js.name = ['left_wheel_joint','right_wheel_joint']
        js.position = [left*2*math.pi/self.tpr, right*2*math.pi/self.tpr]
        js.velocity = [dl*2*math.pi/self.tpr/dt, dr*2*math.pi/self.tpr/dt]
        self.joints.publish(js)

    def close(self):
        if self.ser:
            try: self.ser.write(frame('C,0,0'))
            except (serial.SerialException, OSError): pass
        self.disconnect()


def main():
    rclpy.init(); node = BaseBridge()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
