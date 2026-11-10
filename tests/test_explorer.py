import unittest
from concurrent.futures import Future
from types import SimpleNamespace
import rclpy
from std_msgs.msg import Bool
from rover_bringup.frontier_explorer import FrontierExplorer

class FakeGoal:
    accepted=True
    def __init__(self):self.cancels=0;self.result=Future()
    def cancel_goal_async(self):self.cancels+=1;return Future()
    def get_result_async(self):return self.result

class ExplorerTests(unittest.TestCase):
    def setUp(self):rclpy.init();self.node=FrontierExplorer()
    def tearDown(self):self.node.destroy_node();rclpy.shutdown()
    def test_starts_disarmed(self):self.assertFalse(self.node.armed);self.assertFalse(self.node.busy)
    def test_late_goal_acceptance_is_cancelled_after_stop(self):
        n=self.node;n.busy=True;n.token=2;n.armed=False
        h=FakeGoal();f=Future();f.set_result(h);n.accepted(f,1,(1,2))
        self.assertEqual(h.cancels,1);self.assertTrue(n.busy)
        h.result.set_result(SimpleNamespace(status=5));self.assertFalse(n.busy)
    def test_off_on_does_not_overwrite_pending_goal(self):
        n=self.node;n.armed=True;n.busy=True;n.goal_handle=FakeGoal()
        m=Bool();m.data=False;n.on_arm(m);m.data=True;n.on_arm(m)
        self.assertTrue(n.busy);self.assertEqual(n.goal_handle.cancels,1);self.assertEqual(n.token,1)

if __name__=='__main__':unittest.main()
