import math
import unittest
import numpy as np
from rover_bringup.core import crc16,frame,unframe,counter_delta,integrate,MotionGate
from rover_bringup.frontiers import candidates

class CoreTests(unittest.TestCase):
    def test_crc_known_vector(self): self.assertEqual(crc16(b'123456789'),0x29B1)
    def test_bad_and_truncated_frames(self):
        for value in [b'C,1,2*0000',b'C,1,2',frame('C,1,2')[:-2]]:
            with self.assertRaises(ValueError): unframe(value)
    def test_round_trip(self): self.assertEqual(unframe(frame('C,-123.4,567.8')),['C','-123.4','567.8'])
    def test_encoder_rollover_both_directions(self):
        self.assertEqual(counter_delta(-2147483648,2147483647),1)
        self.assertEqual(counter_delta(2147483647,-2147483648),-1)
    def test_exact_arc(self):
        x,y,yaw,_,_=integrate(0,0,0,0,math.pi,2)
        self.assertAlmostEqual(x,1);self.assertAlmostEqual(y,1);self.assertAlmostEqual(yaw,math.pi/2)
    def armed(self):
        g=MotionGate();g.permit(True,0,True);g.command(.1,.2,.01);return g
    def test_stop_wins_over_later_nav_command(self):
        g=self.armed();g.permit(False,.02,True);g.command(.1,.2,.03)
        self.assertEqual(g.output(.04,True),(0,0))
    def test_lease_loss_latches_until_off_on(self):
        g=self.armed();self.assertEqual(g.output(.5,True),(0,0))
        g.permit(True,.6,True);g.command(.1,.2,.61);self.assertEqual(g.output(.62,True),(0,0))
        g.permit(False,.63,True);g.permit(True,.64,True);g.command(.1,.2,.65)
        self.assertEqual(g.output(.66,True),(.1,.2))
    def test_command_timeout_and_invalid_float(self):
        g=self.armed();g.permit(True,.32,True);self.assertEqual(g.output(.33,True),(0,0))
        g.command(float('nan'),0,.34);self.assertFalse(g.armed)
    def test_sensor_fault_disarms(self):
        g=self.armed();self.assertEqual(g.output(.1,False),(0,0));self.assertFalse(g.armed)
    def test_rearm_does_not_reuse_old_command(self):
        g=self.armed();g.permit(False,.02,True);g.permit(True,.03,True)
        self.assertEqual(g.output(.04,True),(0,0))
    def test_velocity_limits(self):
        g=self.armed();g.command(9,30,.02);v,w=g.output(.03,True)
        self.assertLessEqual(abs(v),.18);self.assertLessEqual(abs(w),.6)
    def test_frontier_is_known_free_reachable_and_clear(self):
        grid=np.full((80,80),-1,dtype=int);grid[10:70,10:70]=0
        grid[10:70,40]=100  # Separates reachable region from far side.
        goals=candidates(grid,.05,(35,25))
        self.assertTrue(goals)
        for _,y,x in goals:
            self.assertEqual(grid[y,x],0);self.assertLess(x,34)
    def test_small_lidar_blind_spot_is_not_an_exploration_target(self):
        grid=np.zeros((80,80),dtype=int);grid[[0,-1],:]=100;grid[:,[0,-1]]=100
        grid[38:42,38:42]=-1
        self.assertEqual(candidates(grid,.05,(40,30)),[])
    def test_no_frontiers_in_completed_enclosed_map(self):
        grid=np.zeros((40,40),dtype=int);grid[[0,-1],:]=100;grid[:,[0,-1]]=100
        self.assertEqual(candidates(grid,.05,(20,20)),[])

if __name__=='__main__':unittest.main()
