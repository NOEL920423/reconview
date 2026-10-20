from rclpy.node import Node
from rosgraph_msgs.msg import Clock


class ClockPublisher(Node):
    def __init__(self, context):
        super().__init__("clock", context=context)
        self.publisher = self.create_publisher(Clock, "/clock", 10)

    def publish_sim_time(self, time_seconds):
        total_ns = round(time_seconds * 1_000_000_000) #乘上十億，轉成整數奈秒

        msg = Clock()
        msg.clock.sec, msg.clock.nanosec = divmod(
            total_ns, 1_000_000_000
        )

        self.publisher.publish(msg)