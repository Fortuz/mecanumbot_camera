import rclpy, time
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from mecanumbot_camera.action import SearchBall
from std_msgs.msg import Header

class SearchServer(Node):
    def __init__(self):
        super().__init__('search_server')
        self._as = ActionServer(self, SearchBall, 'search_ball', self.exec_cb)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ball_seen = False
        self.sub = self.create_subscription(Header, '/detections/ball', self.ball_cb, 10)

    def ball_cb(self, _): self.ball_seen = True

    async def exec_cb(self, goal):
        self.get_logger().info('Search started')
        self.ball_seen = False
        t0 = time.time()
        twist = Twist(); twist.angular.z = 0.6
        while rclpy.ok():
            if self.ball_seen:
                goal.succeed()
                res = SearchBall.Result(); res.found = True; res.status = 'found'
                self.cmd.publish(Twist())  # stop
                return res
            if time.time() - t0 > 20.0:
                goal.abort()
                res = SearchBall.Result(); res.found = False; res.status = 'timeout'
                self.cmd.publish(Twist())  # stop
                return res
            self.cmd.publish(twist)
            await rclpy.sleep(0.1)

def main(): rclpy.init(); n = SearchServer(); rclpy.spin(n); rclpy.shutdown()
