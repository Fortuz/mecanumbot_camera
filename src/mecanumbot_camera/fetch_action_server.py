import rclpy, time
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from mecanumbot_camera.action import FetchBall
from geometry_msgs.msg import Point

class FetchServer(Node):
    def __init__(self):
        super().__init__('fetch_server')
        self._as = ActionServer(self, FetchBall, 'fetch_ball', self.exec_cb)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cx = None; self.size = 0; self.w = 640  # tune width
        self.sub = self.create_subscription(Point, '/detections/ball', self.ball_cb, 10)

    def ball_cb(self, p): self.cx, self.size = p.x, int(p.z)  # assume x=center, z=size

    async def exec_cb(self, goal):
        last_seen = time.time()
        K_ang, K_lin = 0.005, 0.2
        while rclpy.ok():
            if self.cx is not None: last_seen = time.time()
            else:
                if time.time() - last_seen > 2.0:
                    goal.abort(); res = FetchBall.Result(); res.success=False; res.status='lost'; self.cmd.publish(Twist()); return res
                await rclpy.sleep(0.05); continue
            err = (self.cx - self.w/2.0)
            twist = Twist()
            twist.angular.z = -K_ang * err
            twist.linear.x = 0.0 if self.size >= goal.request.stop_px else K_lin
            self.cmd.publish(twist)
            if self.size >= goal.request.stop_px:
                goal.succeed(); res = FetchBall.Result(); res.success=True; res.status='reached'; self.cmd.publish(Twist()); return res
            await rclpy.sleep(0.05)

def main(): rclpy.init(); n=FetchServer(); rclpy.spin(n); rclpy.shutdown()
