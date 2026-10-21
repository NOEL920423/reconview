from rosgraph_msgs.msg import Clock


class ClockPublisher:
    """Publish supplied simulation timestamps on the ROS 2 clock topic."""

    def __init__(self, node):
        self.publisher = node.create_publisher(Clock, "/clock", 10)

    def publish_sim_time(self, time_seconds: float) -> None:
        """Convert simulation seconds to integer seconds and nanoseconds."""
        total_ns = round(time_seconds * 1_000_000_000)

        msg = Clock()
        msg.clock.sec, msg.clock.nanosec = divmod(
            total_ns, 1_000_000_000
        )

        self.publisher.publish(msg)
