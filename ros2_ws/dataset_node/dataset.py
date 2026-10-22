"""Inspect exact-time image groups before adding dataset recording."""

from collections import deque
from functools import partial
import math

import rclpy
from rclpy.clock import Clock as RclpyClock
from rclpy.clock_type import ClockType
from rclpy.event_handler import SubscriptionEventCallbacks
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image


class DatasetNode(Node):
    """Match three Image streams by acquisition stamp within one simulation run."""

    STREAMS = {
        "fpv_rgb": ("/fpv_camera/rgb", "rgb8", "fpv_camera_optical_frame"),
        "fpv_depth": ("/fpv_camera/depth", "32FC1", "fpv_camera_optical_frame"),
        "top_rgb": ("/top_camera/rgb", "rgb8", "top_camera_optical_frame"),
    }

    def __init__(self):
        super().__init__("dataset_node")
        self._max_pending = self.declare_parameter("max_pending_groups", 10).value
        wait_seconds = self.declare_parameter("max_wait_sim_sec", 1.0).value
        if self._max_pending <= 0:
            raise ValueError("max_pending_groups must be positive")
        if not math.isfinite(wait_seconds) or wait_seconds <= 0:
            raise ValueError("max_wait_sim_sec must be finite and positive")
        self._max_wait_ns = round(wait_seconds * 1_000_000_000)
        if self._max_wait_ns < 1:
            raise ValueError("max_wait_sim_sec must be at least one nanosecond")

        self._sim_time_ns = None
        self._clock_reset = False
        self._pending = {}
        # Remember recent outcomes so a late message cannot reopen a closed group.
        self._closed_stamps = deque(maxlen=max(100, self._max_pending * 4))
        self.latest_group = None
        self._received = dict.fromkeys(self.STREAMS, 0)
        self._dds_lost = dict.fromkeys(self.STREAMS, 0)
        self._missing = dict.fromkeys(self.STREAMS, 0)
        self._complete = 0
        self._incomplete = 0
        self._discard_reasons = {"timeout": 0, "capacity": 0}
        self._duplicates = 0
        self._closed_ignored = 0
        self._invalid = 0
        self._blocked_after_reset = 0
        self._last_age_ns = None
        self._max_age_ns = None

        clock_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._clock_subscription = self.create_subscription(
            Clock, "/clock", self._on_clock, clock_qos
        )
        self._image_subscriptions = []
        for stream, (topic, _encoding, _frame_id) in self.STREAMS.items():
            subscription = self.create_subscription(
                Image,
                topic,
                partial(self._on_image, stream),
                qos_profile_sensor_data,
                event_callbacks=SubscriptionEventCallbacks(
                    message_lost=partial(self._on_message_lost, stream)
                ),
            )
            self._image_subscriptions.append(subscription)

        # Keep reporting during Pause; expiration uses simulation time only.
        self._report_clock = RclpyClock(clock_type=ClockType.STEADY_TIME)
        self._stats_timer = self.create_timer(
            1.0, self._report, clock=self._report_clock
        )
        self.get_logger().info(
            "Inspecting fpv_rgb/fpv_depth/top_rgb with exact acquisition stamps; "
            f"max_pending_groups={self._max_pending}, "
            f"max_wait_sim_sec={wait_seconds}; depth units=m; "
            "last_sim_age_ms estimates group completion age from the latest /clock"
        )
        # last_sim_age_ms = 最後收到的 /clock − 該完整組的影像擷取時間，這是模擬時間差的估計，會受不同 topic 送達順序影響；先量測數值，再判斷延遲狀況。

    # 更新模擬時間；若時間倒退，清空暫存並停止配對，要求重啟接收端。
    def _on_clock(self, msg: Clock) -> None:
        """Track simulation time and stop pairing if its timeline moves backwards."""
        now_ns = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        if (
            not self._clock_reset
            and self._sim_time_ns is not None
            and now_ns < self._sim_time_ns
        ):
            discarded = len(self._pending)
            self._pending.clear()
            self._closed_stamps.clear()
            self.latest_group = None
            self._last_age_ns = None
            self._clock_reset = True
            self.get_logger().error(
                "Simulation time moved backwards; pairing stopped and "
                f"{discarded} pending groups cleared. "
                "Restart dataset_node for a new simulation run."
            )
        self._sim_time_ns = now_ns
        for group in self._pending.values():
            if group["first_seen_sim_ns"] is None:
                group["first_seen_sim_ns"] = now_ns

    def _on_message_lost(self, stream, event) -> None:
        """Count middleware loss reports separately from incomplete groups."""
        self._dds_lost[stream] += event.total_count_change

    # 檢查格式，將 stamp 轉成整數 nanosecond，以 _pending[stamp_ns] 分組。三路到齊才計入 complete_groups
    def _on_image(self, stream: str, msg: Image) -> None:
        """Keep owned ROS messages until all three streams share the same stamp."""
        self._received[stream] += 1
        if self._clock_reset:
            self._blocked_after_reset += 1
            return

        _topic, encoding, frame_id = self.STREAMS[stream]
        bytes_per_pixel = 3 if encoding == "rgb8" else 4
        if (
            msg.encoding != encoding
            or msg.header.frame_id != frame_id
            or msg.width == 0
            or msg.height == 0
            or msg.step < msg.width * bytes_per_pixel
            or len(msg.data) != msg.step * msg.height
            or msg.header.stamp.sec < 0
            or msg.header.stamp.nanosec >= 1_000_000_000
        ):
            self._invalid += 1
            self.get_logger().error(
                f"Invalid {stream} Image: encoding={msg.encoding}, "
                f"frame_id={msg.header.frame_id}, size={msg.width}x{msg.height}, "
                f"step={msg.step}, data_bytes={len(msg.data)}"
            )
            return

        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self._expire_pending()
        if stamp_ns in self._closed_stamps:
            self._closed_ignored += 1
            return
        if stamp_ns not in self._pending:
            if len(self._pending) >= self._max_pending:
                oldest_stamp = next(iter(self._pending))
                self._discard_group(oldest_stamp, "capacity")
            self._pending[stamp_ns] = {
                "first_seen_sim_ns": self._sim_time_ns,
                "images": {},
            }

        images = self._pending[stamp_ns]["images"]
        if stream in images:
            self._duplicates += 1
            return
        images[stream] = msg
        if len(images) != len(self.STREAMS):
            return

        self._pending.pop(stamp_ns)
        self._closed_stamps.append(stamp_ns)
        self._complete += 1
        # Keep one complete group for inspection; no image copies or disk writes.
        self.latest_group = {
            "stamp_ns": stamp_ns,
            "images": images,
            "received_sim_ns": self._sim_time_ns,
        }
        self._last_age_ns = (
            None if self._sim_time_ns is None else self._sim_time_ns - stamp_ns
        )
        if self._last_age_ns is not None:
            self._max_age_ns = (
                self._last_age_ns
                if self._max_age_ns is None
                else max(self._max_age_ns, self._last_age_ns)
            )

    def _discard_group(self, stamp_ns: int, reason: str) -> None:
        """Close an incomplete observed group and count its absent streams."""
        group = self._pending.pop(stamp_ns)
        self._incomplete += 1
        self._discard_reasons[reason] += 1
        for stream in self.STREAMS:
            if stream not in group["images"]:
                self._missing[stream] += 1
        self._closed_stamps.append(stamp_ns)

    # 未完成組最多保留 10 組，等待上限為 1 秒模擬時間；逾限後統計缺少哪一路。
    def _expire_pending(self) -> None:
        """Bound waiting in simulation time so Pause does not cause expiration."""
        if self._sim_time_ns is None:
            return
        for stamp_ns, group in list(self._pending.items()):
            first_seen = group["first_seen_sim_ns"]
            if first_seen is not None:
                if self._sim_time_ns - first_seen >= self._max_wait_ns:
                    self._discard_group(stamp_ns, "timeout")

    # 輸出配對、DDS 遺失與時間差統計。Pause 期間仍會印統計，等待期限則隨模擬時間暫停。
    def _report(self) -> None:
        """Print one readable block per report, with one value on each line."""
        self._expire_pending()
        latest_stamp = None if self.latest_group is None else self.latest_group["stamp_ns"]
        sim_time_s = (
            "unknown" if self._sim_time_ns is None else f"{self._sim_time_ns / 1e9:.6f}"
        )
        capture_time_s = (
            "unknown" if latest_stamp is None else f"{latest_stamp / 1e9:.6f}"
        )
        age_ms = "unknown" if self._last_age_ns is None else f"{self._last_age_ns / 1e6:.3f}"
        max_age_ms = "unknown" if self._max_age_ns is None else f"{self._max_age_ns / 1e6:.3f}"
        lines = [
            "=" * 48,
            f"sim_time_s              : {sim_time_s}",
            f"latest_capture_time_s   : {capture_time_s}",
            f"latest_stamp_ns         : {latest_stamp}",
            "",
            f"complete_groups         : {self._complete}",
            f"incomplete_groups       : {self._incomplete}",
            f"pending_groups          : {len(self._pending)}",
            f"last_sim_age_ms         : {age_ms}",
            f"max_sim_age_ms          : {max_age_ms}",
        ]
        for label, counts in (
            ("received", self._received),
            ("missing", self._missing),
            ("dds_lost", self._dds_lost),
        ):
            lines.append("")
            for stream, count in counts.items():
                lines.append(f"{label + '_' + stream:<24}: {count}")
        lines.extend([
            "",
            f"discard_timeout         : {self._discard_reasons['timeout']}",
            f"discard_capacity        : {self._discard_reasons['capacity']}",
            f"duplicates              : {self._duplicates}",
            f"closed_ignored          : {self._closed_ignored}",
            f"invalid                 : {self._invalid}",
            f"clock_reset             : {self._clock_reset}",
            f"blocked_after_reset     : {self._blocked_after_reset}",
            "=" * 48,
        ])
        # One log call keeps the ROS prefix outside the field-by-field block.
        self.get_logger().info("\n" + "\n".join(lines))


def main(args=None) -> None:
    """Run the inspection subscriber using the system ROS Python interpreter."""
    rclpy.init(args=args)
    node = None
    try:
        node = DatasetNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
