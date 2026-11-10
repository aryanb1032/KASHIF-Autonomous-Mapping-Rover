import math
import unittest
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from rover_bringup.safety_gate import SafetyGate

class GateTests(unittest.TestCase):
    def setUp(self):rclpy.init();self.n=SafetyGate()
    def tearDown(self):self.n.destroy_node();rclpy.shutdown()
    def test_requires_all_fresh_sources(self):
        n=self.n;self.assertFalse(n.healthy())
        od=Odometry();od.header.stamp=n.get_clock().now().to_msg();n.odom(od);n.wheel(od)
        status=Bool();status.data=False;n.base(status)
        scan=LaserScan();scan.header.stamp=n.get_clock().now().to_msg();scan.range_min=.1;scan.range_max=8.;scan.ranges=[float('nan')]
        n.scan(scan);self.assertFalse(n.healthy())
        scan.ranges=[1.];n.scan(scan);self.assertTrue(n.healthy())
    def test_replayed_old_scan_cannot_refresh_watchdog(self):
        scan=LaserScan();scan.range_min=.1;scan.range_max=8.;scan.ranges=[1.]
        self.n.scan(scan);self.assertEqual(self.n.scan_at,-math.inf)

if __name__=='__main__':unittest.main()
