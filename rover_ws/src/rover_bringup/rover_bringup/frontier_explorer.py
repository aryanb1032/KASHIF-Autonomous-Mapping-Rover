import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from tf2_ros import Buffer, TransformListener, TransformException
from .frontiers import candidates


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        self.map=None; self.armed=False; self.busy=False; self.goal_handle=None; self.token=0
        self.sent_at=0.0; self.goal_xy=None; self.cooldown=[]; self.empty_count=0; self.cancelled=False
        self.done=self.create_publisher(Bool,'/rover/exploration_done',10)
        self.create_subscription(OccupancyGrid,'/map',self.on_map,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Bool,'/rover/armed',self.on_arm,10)
        self.tf=Buffer(); self.listener=TransformListener(self.tf,self)
        self.action=ActionClient(self,NavigateToPose,'navigate_to_pose')
        self.create_timer(2.0,self.tick)

    def on_map(self,msg): self.map=msg

    def on_arm(self,msg):
        if self.armed and not msg.data:
            self.token+=1
            if self.goal_handle: self.goal_handle.cancel_goal_async()
        if msg.data and not self.armed: self.empty_count=0
        self.armed=msg.data

    def tick(self):
        if self.busy:
            if self.goal_handle and time.monotonic()-self.sent_at>60 and not self.cancelled:
                self.cancelled=True; self.goal_handle.cancel_goal_async()
            return
        if not self.armed or self.map is None or not self.action.server_is_ready(): return
        try: pose=self.tf.lookup_transform('map','base_link',Time())
        except TransformException: return  # Never substitute an invented robot pose.
        if self.get_clock().now().nanoseconds - Time.from_msg(pose.header.stamp).nanoseconds > 1e9: return
        m=self.map; r=m.info.resolution
        if r<=0 or len(m.data)!=m.info.width*m.info.height: return
        q=m.info.origin.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)); c,s=math.cos(yaw),math.sin(yaw)
        dx=pose.transform.translation.x-m.info.origin.position.x
        dy=pose.transform.translation.y-m.info.origin.position.y
        cell=(math.floor((-s*dx+c*dy)/r),math.floor((c*dx+s*dy)/r))
        grid=np.asarray(m.data,dtype=np.int16).reshape(m.info.height,m.info.width)
        self.cooldown=[g for g in self.cooldown if time.monotonic()<g[2]]
        goal=None
        for _,y,x in candidates(grid,r,cell):
            lx,ly=(x+0.5)*r,(y+0.5)*r
            gx=m.info.origin.position.x+c*lx-s*ly; gy=m.info.origin.position.y+s*lx+c*ly
            if all(math.hypot(gx-a,gy-b)>0.6 for a,b,_ in self.cooldown): goal=(gx,gy); break
        if goal is None:
            self.empty_count+=1
            if self.empty_count>=5:
                msg=Bool();msg.data=True;self.done.publish(msg)
                self.get_logger().info('No eligible frontiers for 10 seconds. Stopping; this does not prove all space was mapped.')
            return
        self.empty_count=0;self.busy=True;self.cancelled=False;self.goal_xy=goal;self.sent_at=time.monotonic()
        request=NavigateToPose.Goal();request.pose.header.frame_id='map';request.pose.header.stamp=self.get_clock().now().to_msg()
        request.pose.pose.position.x,request.pose.pose.position.y=goal;request.pose.pose.orientation.w=1.0
        token=self.token
        self.action.send_goal_async(request).add_done_callback(lambda f:self.accepted(f,token,goal))

    def accepted(self,future,token,goal):
        try: handle=future.result()
        except Exception as exc:
            self.get_logger().error(str(exc));self.finish(goal,False);return
        if not handle.accepted: self.finish(goal,False);return
        self.goal_handle=handle
        handle.get_result_async().add_done_callback(lambda f:self.result(f,goal))
        if not self.armed or token!=self.token: handle.cancel_goal_async()

    def result(self,future,goal):
        try: success=future.result().status==GoalStatus.STATUS_SUCCEEDED
        except Exception: success=False
        self.finish(goal,success)

    def finish(self,goal,success):
        self.cooldown.append((*goal,time.monotonic()+(30 if success else 90)))
        self.goal_handle=None;self.busy=False


def main():
    rclpy.init();node=FrontierExplorer()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok() and node.goal_handle: node.goal_handle.cancel_goal_async()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
