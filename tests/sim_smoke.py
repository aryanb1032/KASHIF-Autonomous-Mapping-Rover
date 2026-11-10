"""End-to-end Gazebo/Nav2 smoke test. Starts and stops only its own launch process."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from lifecycle_msgs.srv import GetState


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--log-dir',default='sim-test-results');parser.add_argument('--autonomous',action='store_true');args=parser.parse_args()
    output=Path(args.log_dir);output.mkdir(parents=True,exist_ok=True)
    os.environ['ROS_DOMAIN_ID']=str(100+uuid.uuid4().int%60);os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
    os.environ['GZ_PARTITION']='rover_test_'+uuid.uuid4().hex
    log=(output/'launch.log').open('w')
    proc=subprocess.Popen(['ros2','launch','rover_bringup','sim.launch.py','gui:=false','explore:='+str(args.autonomous).lower()],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=Node('rover_sim_test')
    seen={};counts={}
    def receive(topic,msg):seen[topic]=msg;counts[topic]=counts.get(topic,0)+1
    subscriptions=[]
    for topic,kind,qos in [('/scan',LaserScan,qos_profile_sensor_data),('/map',OccupancyGrid,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)),('/wheel/odom',Odometry,10),('/odometry/filtered',Odometry,10),('/cmd_vel_safe',Twist,10),('/rover/armed',Bool,10)]:
        subscriptions.append(node.create_subscription(kind,topic,lambda m,t=topic:receive(t,m),qos))
    permit=node.create_publisher(Bool,'/rover/permit',10);command=node.create_publisher(Twist,'/cmd_vel',10)
    nav=ActionClient(node,NavigateToPose,'navigate_to_pose')
    def pump(duration,enable=None,v=None):
        deadline=time.monotonic()+duration
        next_pub=0.0
        while time.monotonic()<deadline:
            if proc.poll() is not None:raise RuntimeError('Launch process exited')
            publish=time.monotonic()>=next_pub
            if publish: next_pub=time.monotonic()+0.05
            if publish and enable is not None:
                msg=Bool();msg.data=enable;permit.publish(msg)
            if publish and v is not None:
                msg=Twist();msg.linear.x=v;command.publish(msg)
            rclpy.spin_once(node,timeout_sec=.04)
    result={}
    try:
        deadline=time.monotonic()+60
        servers=['controller_server','planner_server','smoother_server','behavior_server','bt_navigator','velocity_smoother','slam_toolbox']
        clients={name:node.create_client(GetState,'/'+name+'/get_state') for name in servers}
        states={};pending={}
        while time.monotonic()<deadline:
            pump(.2,False)
            for name,client in clients.items():
                if name in pending and pending[name].done():
                    response=pending.pop(name).result()
                    if response: states[name]=response.current_state.id
                if client.service_is_ready() and name not in pending and states.get(name)!=3:
                    pending[name]=client.call_async(GetState.Request())
            if all(states.get(name)==3 for name in servers) and all(t in seen for t in ['/scan','/map','/wheel/odom','/odometry/filtered']):break
        assert all(states.get(name)==3 for name in servers),states
        assert seen['/scan'].header.frame_id=='laser_link'
        assert seen['/map'].info.width>0
        pump(.25,False);pump(.3,True)
        assert seen['/rover/armed'].data,'Gate did not arm'
        pose=seen['/odometry/filtered'].pose.pose.position
        start_x,start_y=pose.x,pose.y
        if args.autonomous:
            initial_known=sum(v>=0 for v in seen['/map'].data)
            deadline=time.monotonic()+30
            distance=0.0; next_debug=0.0
            while time.monotonic()<deadline:
                pump(.15,True)
                pose=seen['/odometry/filtered'].pose.pose.position
                distance=math.hypot(pose.x-start_x,pose.y-start_y)
                if time.monotonic()>next_debug:
                    next_debug=time.monotonic()+2
                    q=seen['/odometry/filtered'].pose.pose.orientation
                    cmd=seen['/cmd_vel_safe']; wheel=seen['/wheel/odom'].twist.twist
                    print('AUTO',distance,'cmd',cmd.linear.x,cmd.angular.z,'wheel',wheel.linear.x,wheel.angular.z,'yaw',2*math.atan2(q.z,q.w),'armed',seen['/rover/armed'].data,flush=True)
                if distance>.4: break
            assert distance>.3,('Exploration did not move',distance)
            pump(2.5,True)
            final_known=sum(v>=0 for v in seen['/map'].data)
            assert final_known>initial_known,(initial_known,final_known)
            pump(.6,False)
            assert seen['/cmd_vel_safe'].linear.x==0 and not seen['/rover/armed'].data
            result={'passed':True,'autonomous_distance_m':distance,'known_cells_before':initial_known,'known_cells_after':final_known,'stop_cancels_exploration':True,'topic_counts':counts}
        else:
            request=NavigateToPose.Goal();request.pose.header.frame_id='map';request.pose.header.stamp=seen['/odometry/filtered'].header.stamp
            request.pose.pose.position.x=.8;request.pose.pose.orientation.w=1.
            future=nav.send_goal_async(request)
            deadline=time.monotonic()+5
            while not future.done() and time.monotonic()<deadline:pump(.1,True)
            handle=future.result();assert handle.accepted,'Navigation goal rejected'
            done=handle.get_result_async();deadline=time.monotonic()+35
            while not done.done() and time.monotonic()<deadline:pump(.1,True)
            assert done.done(),'Navigation timed out'
            assert done.result().status==4,done.result().status
            pose=seen['/odometry/filtered'].pose.pose.position
            distance=math.hypot(pose.x-start_x,pose.y-start_y)
            assert distance>.5,distance
            # A stop request must override a continuing nonzero command source.
            pump(.25,False,.12)
            assert seen['/cmd_vel_safe'].linear.x==0 and not seen['/rover/armed'].data
            pump(.5,True,.12);assert seen['/cmd_vel_safe'].linear.x>0
            # Lose the keyboard lease but keep sending motion commands.
            pump(.65,None,.12)
            assert seen['/cmd_vel_safe'].linear.x==0 and not seen['/rover/armed'].data
            pump(.2,False,0.)
            result={'passed':True,'lifecycle_states':states,'topic_counts':counts,'navigation_distance_m':distance,'stop_overrides_commands':True,'lease_loss_stops':True}
    
        print(json.dumps(result,indent=2))
    finally:
        msg=Bool();msg.data=False;permit.publish(msg)
        node.destroy_node();rclpy.shutdown()
        proc.send_signal(signal.SIGINT)
        try:proc.wait(timeout=12)
        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        log.close();(output/'result.json').write_text(json.dumps(result or {'passed':False},indent=2))

if __name__=='__main__':main()
