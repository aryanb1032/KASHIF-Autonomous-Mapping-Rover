import os
import select
import subprocess
import sys
import termios
import time
import unittest
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class KeyTests(unittest.TestCase):
    def test_w_o_q_over_a_real_terminal(self):
        master,slave=os.openpty();original=termios.tcgetattr(slave)
        rclpy.init();node=Node('keys_test');seen=[]
        node.create_subscription(Bool,'/rover/permit',lambda m:seen.append(m.data),10)
        child=subprocess.Popen([sys.executable,'-m','rover_bringup.explore_keys'],stdin=slave,stdout=slave,stderr=slave)
        def wait_for(value):
            deadline=time.monotonic()+3
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.05)
                if seen and seen[-1]==value:return True
            return False
        try:
            self.assertTrue(wait_for(False))
            seen.clear();os.write(master,b'w');self.assertTrue(wait_for(True))
            seen.clear();os.write(master,b'o');self.assertTrue(wait_for(False))
            os.write(master,b'q');self.assertEqual(child.wait(timeout=3),0)
            self.assertEqual(termios.tcgetattr(slave),original)
        finally:
            if child.poll() is None:child.terminate();child.wait(timeout=3)
            node.destroy_node();rclpy.shutdown();os.close(master);os.close(slave)
