import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from mecanumbot_camera.action import SearchBall, FetchBall
from std_msgs.msg import Header

class Supervisor(Node):
    def __init__(self):
        super().__init__('supervisor')
        self.ball_visible = False
        self.sub = self.create_subscription(Header, '/detections/ball', self.seen, 10)
        self.search = ActionClient(self, SearchBall, 'search_ball')
        self.fetch  = ActionClient(self, FetchBall,  'fetch_ball')
        self.timer = self.create_timer(0.5, self.loop)

    def seen(self, _): self.ball_visible = True
    async def loop(self):
        self.timer.cancel()
        while not self.search.wait_for_server(timeout_sec=0.5): pass
        while not self.fetch.wait_for_server(timeout_sec=0.5): pass
        while rclpy.ok():
            if not self.ball_visible:
                res = await self.search.send_goal_async(SearchBall.Goal()); await res.get_result_async()
            else:
                goal = FetchBall.Goal(); goal.stop_px = 60
                res = await self.fetch.send_goal_async(goal); result = await res.get_result_async()
                if result.status != 4:  # SUCCEEDED
                    self.ball_visible = False

def main(): rclpy.init(); n=Supervisor(); rclpy.spin(n); rclpy.shutdown()
