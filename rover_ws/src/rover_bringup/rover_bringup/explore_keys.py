"""Operator lease: terminal loss expires the command gate in 400 ms."""
import select
import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


def main():
    if not sys.stdin.isatty(): raise SystemExit('Use an interactive terminal for w/o/q controls.')
    rclpy.init(); node=Node('explore_keys'); enabled=False
    pub=node.create_publisher(Bool,'/rover/permit',10)
    def finished(msg):
        nonlocal enabled
        if msg.data: enabled=False
    node.create_subscription(Bool,'/rover/exploration_done',finished,10)
    old=termios.tcgetattr(sys.stdin.fileno())
    print('w: enable autonomous mapping | o: stop | q: stop and quit',flush=True)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            if select.select([sys.stdin],[],[],0)[0]:
                key=sys.stdin.read(1).lower()
                if not key or key=='q': break
                if key=='w': enabled=True
                elif key=='o': enabled=False
            msg=Bool();msg.data=enabled;pub.publish(msg)
            rclpy.spin_once(node,timeout_sec=0.05)
    except KeyboardInterrupt:pass
    finally:
        try: termios.tcsetattr(sys.stdin.fileno(),termios.TCSADRAIN,old)
        except termios.error: pass
        if rclpy.ok():
            msg=Bool();msg.data=False
            for _ in range(3): pub.publish(msg);rclpy.spin_once(node,timeout_sec=0.02)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
