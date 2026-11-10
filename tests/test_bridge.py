import os
import time
import unittest
import rclpy
from geometry_msgs.msg import Twist
from rover_bringup.base_bridge import BaseBridge
from rover_bringup.core import frame,unframe

class BridgeTests(unittest.TestCase):
    def setUp(self):
        self.master,self.slave=os.openpty();os.set_blocking(self.master,False)
        rclpy.init(args=['--ros-args','-p','port:='+os.ttyname(self.slave),'-p','hardware_calibrated:=true'])
        self.node=BaseBridge();self.ms=100;self.lines=bytearray()
        self.node.tick()
    def tearDown(self):
        self.node.close();self.node.destroy_node();rclpy.shutdown();os.close(self.master);os.close(self.slave)
    def sample(self,ms,left=0,right=0,ok=1):
        return frame(f'T,{ms},{left},{right},{ok},0,0,0,9.80665,0,0,0')
    def pump(self,seconds,command=False):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            self.ms+=20;os.write(self.master,self.sample(self.ms))
            if command:
                msg=Twist();msg.linear.x=.1;self.node.command(msg)
            self.node.tick()
            try:self.lines.extend(os.read(self.master,8192))
            except BlockingIOError:pass
            time.sleep(.02)
    def commands(self):
        result=[]
        for line in self.lines.splitlines():
            try:
                fields=unframe(line)
                if fields[0]=='C': result.append(tuple(map(float,fields[1:])))
            except ValueError:pass
        return result
    def test_pseudoterminal_command_timeout(self):
        self.pump(.12,True);self.assertTrue(any(l>0 and r>0 for l,r in self.commands()))
        self.lines.clear();self.pump(.5,False)
        self.assertEqual(self.commands()[-1],(0.,0.))
    def test_corruption_and_reboot_do_not_jump_odometry(self):
        self.node.telemetry(self.sample(100,1000,1000));self.node.telemetry(self.sample(120,1010,1010))
        x=self.node.x
        self.node.telemetry(b'T,1,0,0,1,0,0,0,0,0,0,0*0000')
        self.assertEqual(self.node.x,x)
        self.node.telemetry(self.sample(5,0,0));self.assertEqual(self.node.x,x);self.assertTrue(self.node.fault)
    def test_telemetry_loss_stops_despite_fresh_commands(self):
        self.pump(.12,True);time.sleep(.28)
        msg=Twist();msg.linear.x=.1;self.node.command(msg);self.node.tick()
        try:self.lines.extend(os.read(self.master,8192))
        except BlockingIOError:pass
        self.assertEqual(self.commands()[-1],(0.,0.))
    def test_hardware_calibration_lock(self):
        self.node.calibrated=False;self.pump(.15,True)
        self.assertTrue(all(l==0 and r==0 for l,r in self.commands()))

if __name__=='__main__':unittest.main()
