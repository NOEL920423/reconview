"""Inspect image delivery and Linux hardware load without recording images.

Run with the system ROS 2 Jazzy Python environment on the Isaac Sim host.
Start before Play, wait for one matched publisher per topic, then measure.
Pause and let counters settle before comparing them with the publisher.

Sequence gaps cover only the interval between received messages from the same
publisher. They cannot locate a loss in the network, DDS history, or executor,
or detect losses before the first or after the last received message.
Jazzy does not expose a per-message publisher GID in Python; graph identity and
matched events guard the required single-publisher measurement interval.
Capture timestamp gaps alone are not evidence of transport loss.
"""

from collections import deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from functools import partial
from pathlib import Path
import os
import socket
import subprocess
import time

import rclpy
from rclpy.clock import Clock as RclpyClock
from rclpy.clock_type import ClockType
from rclpy.event_handler import SubscriptionEventCallbacks, UnsupportedEventTypeError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image


STREAMS = {
    "fpv_rgb": ("/fpv_camera/rgb", "rgb8", "fpv_camera_optical_frame", 3),
    "fpv_depth": ("/fpv_camera/depth", "32FC1", "fpv_camera_optical_frame", 4),
    "top_rgb": ("/top_camera/rgb", "rgb8", "top_camera_optical_frame", 3),
}
UNSUPPORTED_SEQUENCE = (1 << 64) - 1


@dataclass
class StreamStats:
    """Track callbacks and publication sequences without retaining image data."""

    received: int = 0
    invalid: int = 0
    dds_lost: int | None = 0
    last_stamp_ns: int | None = None
    max_capture_gap_ns: int = 0
    stamp_regressions: int = 0
    repeated_stamps: int = 0
    publisher_gid: bytes | None = None
    last_sequence: int | None = None
    sequence_samples: int = 0
    unsupported_samples: int = 0
    sequence_anomalies: int = 0
    publisher_changes: int = 0
    missing_between_received: int = 0
    matched_events_supported: bool = True

    @property
    def sequence_status(self) -> str:
        if not self.matched_events_supported:
            return "publisher_guard_unavailable"
        if self.publisher_changes or self.sequence_anomalies:
            return "invalid_comparison"
        if self.unsupported_samples:
            return "partial_metadata" if self.sequence_samples else "unavailable"
        return "available" if self.sequence_samples >= 2 else "waiting_for_samples"

    def observe_sequence(self, info: dict) -> None:
        """Compare sequences only within one publisher identity.

        The sequence contract and unsupported sentinel are defined in ROS 2 rmw:
        https://github.com/ros2/rmw/blob/jazzy/rmw/include/rmw/types.h
        """
        sequence = info.get("publication_sequence_number")
        gid = bytes(info.get("publisher_gid") or b"")
        if (
            not isinstance(sequence, int)
            or not 0 <= sequence < UNSUPPORTED_SEQUENCE
            or not any(gid)
        ):
            self.unsupported_samples += 1
            # Do not bridge an interval whose metadata was unavailable.
            self.last_sequence = None
            return
        if self.publisher_gid is not None and gid != self.publisher_gid:
            self.publisher_changes += 1
            self.last_sequence = None
        self.publisher_gid = gid
        if self.last_sequence is not None:
            if sequence > self.last_sequence:
                self.missing_between_received += sequence - self.last_sequence - 1
            else:
                self.sequence_anomalies += 1
        self.last_sequence = max(sequence, self.last_sequence) if self.last_sequence is not None else sequence
        self.sequence_samples += 1

    def observe_stamp(self, stamp_ns: int) -> None:
        if self.last_stamp_ns is not None:
            difference = stamp_ns - self.last_stamp_ns
            if difference < 0:
                self.stamp_regressions += 1
            elif difference == 0:
                self.repeated_stamps += 1
            else:
                self.max_capture_gap_ns = max(self.max_capture_gap_ns, difference)
        self.last_stamp_ns = stamp_ns


def read_cpu_ticks() -> dict:
    """Read aggregate and logical CPU ticks, excluding duplicate guest fields."""
    ticks = {}
    for line in Path("/proc/stat").read_text().splitlines():
        fields = line.split()
        if fields and fields[0].startswith("cpu"):
            values = [int(value) for value in fields[1:9]]
            ticks[fields[0]] = (sum(values), values[3] + values[4])
    return ticks


def read_key_values(path: str) -> dict:
    return {
        fields[0].rstrip(":"): int(fields[1])
        for line in Path(path).read_text().splitlines()
        if len(fields := line.split()) >= 2
    }


def sample_hardware(previous) -> tuple:
    """Sample host resources in a worker so nvidia-smi cannot block ROS callbacks."""
    now_ns = time.monotonic_ns()
    cpu = read_cpu_ticks()
    memory = read_key_values("/proc/meminfo")
    vm = read_key_values("/proc/vmstat")
    rows = []
    if previous is not None:
        previous_ns, previous_cpu, previous_vm = previous
        elapsed_s = (now_ns - previous_ns) / 1e9
        busy = {}
        for name, (total, idle) in cpu.items():
            if name in previous_cpu:
                old_total, old_idle = previous_cpu[name]
                delta = total - old_total
                if delta > 0:
                    busy[name] = 100 * (delta - (idle - old_idle)) / delta
        rows.append(("cpu_total_percent", f"{busy.get('cpu', 0.0):.1f}"))
        logical = [value for name, value in busy.items() if name != "cpu"]
        rows.append(("cpu_busiest_logical_percent", f"{max(logical, default=0.0):.1f}"))
        page_mib = os.sysconf("SC_PAGE_SIZE") / (1024 * 1024)
        for name, key in (("swap_in_mib_s", "pswpin"), ("swap_out_mib_s", "pswpout")):
            rate = (vm[key] - previous_vm[key]) * page_mib / elapsed_s
            rows.append((name, f"{rate:.3f}"))
    else:
        rows.extend((name, "warming_up") for name in (
            "cpu_total_percent", "cpu_busiest_logical_percent",
            "swap_in_mib_s", "swap_out_mib_s",
        ))
    rows.extend([
        ("ram_total_mib", f"{memory['MemTotal'] / 1024:.1f}"),
        ("ram_available_mib", f"{memory['MemAvailable'] / 1024:.1f}"),
        ("swap_used_mib", f"{(memory['SwapTotal'] - memory['SwapFree']) / 1024:.1f}"),
    ])
    try:
        result = subprocess.run(
            ["nvidia-smi", "--query-gpu=index,utilization.gpu,memory.used,memory.total",
             "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=2.0, check=False,
        )
    except FileNotFoundError:
        rows.append(("gpu_status", "nvidia_smi_not_found"))
    except subprocess.TimeoutExpired:
        rows.append(("gpu_status", "nvidia_smi_timeout"))
    else:
        rows.append(("gpu_status", "available" if result.returncode == 0 else "query_failed"))
        if result.returncode:
            rows.append(("gpu_query_error", " ".join(result.stderr.split())))
        else:
            for line in result.stdout.splitlines():
                index, utilization, used, total = [field.strip() for field in line.split(",")]
                rows.extend([
                    (f"gpu{index}_util_percent", utilization),
                    (f"gpu{index}_memory_used_mib", used),
                    (f"gpu{index}_memory_total_mib", total),
                ])
    return (now_ns, cpu, vm), rows


class TransportCheck(Node):
    """Observe the existing three topics with the same best-effort queue depth."""

    def __init__(self):
        super().__init__("transport_check")
        depth = self.declare_parameter("queue_depth", 5).value
        if type(depth) is not int or depth <= 0:
            raise ValueError("queue_depth must be a positive integer")
        self.declare_parameter("stats_detail", False)
        monitor_hardware = self.declare_parameter("monitor_hardware", True).value
        self.streams = {name: StreamStats() for name in STREAMS}
        self.clock_ns = None
        self.clock_reset = False
        self.last_clock_advance_ns = None
        self.start_ns = self.previous_report_ns = time.monotonic_ns()
        self.previous_clock_ns = None
        self.previous_received = dict.fromkeys(STREAMS, 0)
        self.pending = {}
        self.closed_stamps = deque(maxlen=256)
        self.complete_groups = 0
        self.evicted_unmatched_groups = 0
        self.blocked_after_reset = 0
        self._hardware_previous = None
        self._hardware_rows = [("hardware_status", "warming_up" if monitor_hardware else "disabled")]
        self._hardware_pool = None
        self._hardware_future = None
        self.subscriptions_by_stream = {}
        qos = QoSProfile(depth=depth, reliability=ReliabilityPolicy.BEST_EFFORT)
        for stream, (topic, _encoding, _frame_id, _bytes) in STREAMS.items():
            try:
                subscription = self.create_subscription(
                    Image, topic, partial(self._on_image, stream), qos,
                    event_callbacks=SubscriptionEventCallbacks(
                        message_lost=partial(self._on_lost, stream),
                        matched=partial(self._on_matched, stream),
                    ),
                )
            except UnsupportedEventTypeError:
                self.streams[stream].dds_lost = None
                self.streams[stream].matched_events_supported = False
                self.get_logger().warning(
                    f"Subscription events unsupported for {topic}; "
                    "DDS loss and guarded sequence comparison are unknown"
                )
                subscription = self.create_subscription(
                    Image, topic, partial(self._on_image, stream), qos
                )
            self.subscriptions_by_stream[stream] = subscription
        self.clock_subscription = self.create_subscription(
            Clock, "/clock", self._on_clock,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self._steady_clock = RclpyClock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(1.0, self.report, clock=self._steady_clock)
        if monitor_hardware:
            self._hardware_pool = ThreadPoolExecutor(max_workers=1)
            self._hardware_future = self._hardware_pool.submit(sample_hardware, None)
        self.get_logger().info(
            f"Host={socket.gethostname()}, PID={os.getpid()}, "
            f"ROS_DOMAIN_ID={self.context.get_domain_id()}, "
            f"RMW={rclpy.get_rmw_implementation_identifier()}, queue_depth={depth}. "
            "Read-only diagnostic; no image copies or disk recording. "
            "Sequence gaps exclude startup/tail loss and do not identify the loss layer. "
            "Python publisher identity is guarded by graph data and matched events. "
            "Hardware load is for this host; run here only if Isaac Sim runs here."
        )

    def _on_lost(self, stream: str, event) -> None:
        self.streams[stream].dds_lost += event.total_count_change

    def _on_matched(self, stream: str, event) -> None:
        # A second match invalidates the entire interval, even after a disconnect.
        stats = self.streams[stream]
        stats.publisher_changes = max(stats.publisher_changes, event.total_count - 1)

    def _on_clock(self, msg: Clock) -> None:
        now_ns = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        if self.clock_ns is not None and now_ns < self.clock_ns and not self.clock_reset:
            self.clock_reset = True
            self.get_logger().error(
                "Simulation clock moved backwards; image measurement stopped. "
                "Restart this checker for the next run."
            )
        if self.clock_ns is None or now_ns > self.clock_ns:
            self.last_clock_advance_ns = time.monotonic_ns()
        self.clock_ns = now_ns

    def _on_image(self, stream: str, msg: Image, info: dict) -> None:
        if self.clock_reset:
            self.blocked_after_reset += 1
            return
        stats = self.streams[stream]
        stats.received += 1
        _topic, encoding, frame_id, bytes_per_pixel = STREAMS[stream]
        publishers = self.get_publishers_info_by_topic(_topic)
        metadata = dict(info)
        if len(publishers) == 1:
            metadata.setdefault("publisher_gid", bytes(publishers[0].endpoint_gid))
        else:
            # Never compare sequences while identity is ambiguous or undiscovered.
            metadata.pop("publisher_gid", None)
            if len(publishers) > 1:
                stats.publisher_changes = max(stats.publisher_changes, 1)
        stats.observe_sequence(metadata)
        if (
            msg.encoding != encoding or msg.header.frame_id != frame_id
            or msg.width == 0 or msg.height == 0
            or msg.step < msg.width * bytes_per_pixel
            or len(msg.data) != msg.height * msg.step
            or msg.header.stamp.sec < 0
            or not 0 <= msg.header.stamp.nanosec < 1_000_000_000
        ):
            stats.invalid += 1
            self.get_logger().error(f"Invalid Image metadata or payload length on {stream}")
            return
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        stats.observe_stamp(stamp_ns)
        # Retain only stamps and stream names. No timeout is inferred during Pause.
        if stamp_ns in self.closed_stamps:
            return
        streams = self.pending.setdefault(stamp_ns, set())
        streams.add(stream)
        if len(streams) == len(STREAMS):
            self.complete_groups += 1
            del self.pending[stamp_ns]
            self.closed_stamps.append(stamp_ns)
        if len(self.pending) > 256:
            del self.pending[next(iter(self.pending))]
            self.evicted_unmatched_groups += 1

    def report(self, *, final: bool = False) -> None:
        """Emit one block; wall rates and observed simulation progress stay separate."""
        show_details = self.get_parameter("stats_detail").value
        now_ns = time.monotonic_ns()
        wall_s = (now_ns - self.previous_report_ns) / 1e9
        ratio = "unknown"
        if self.previous_clock_ns is not None and not self.clock_reset:
            ratio = f"{(self.clock_ns - self.previous_clock_ns) / 1e9 / wall_s:.3f}"
        counts = {
            name: subscription.get_publisher_count()
            for name, subscription in self.subscriptions_by_stream.items()
        }
        clock_publishers = self.clock_subscription.get_publisher_count()
        ready = all(count == 1 for count in counts.values()) and clock_publishers == 1
        if self._hardware_future is not None and self._hardware_future.done():
            # Unexpected sampling errors propagate with their original traceback.
            self._hardware_previous, self._hardware_rows = self._hardware_future.result()
            self._hardware_future = None
        hardware_age = (
            "unknown" if self._hardware_previous is None
            else f"{(now_ns - self._hardware_previous[0]) / 1e9:.3f}"
        )
        rows = [
            ("report_type", "final" if final else "periodic"),
            ("elapsed_wall_s", f"{(now_ns - self.start_ns) / 1e9:.3f}"),
            ("window_wall_s", f"{wall_s:.3f}"),
            ("publishers_ready", ready),
            ("clock_publishers", clock_publishers),
            ("sim_wall_ratio_est", ratio),
            ("clock_idle_wall_s", "unknown" if self.last_clock_advance_ns is None
             else f"{(now_ns - self.last_clock_advance_ns) / 1e9:.3f}"),
            ("clock_reset", self.clock_reset),
            ("blocked_after_reset", self.blocked_after_reset),
            None,
            ("complete_groups_total", self.complete_groups),
            ("unmatched_observed_groups", len(self.pending)),
            ("evicted_unmatched_groups", self.evicted_unmatched_groups),
        ]
        for name, stats in self.streams.items():
            rate = (stats.received - self.previous_received[name]) / wall_s
            missing = stats.missing_between_received if stats.sequence_status == "available" else "unknown"
            rows.extend([
                None,
                (f"{name}_publishers", counts[name]),
                (f"{name}_received_total", stats.received),
                (f"{name}_receive_hz_wall", f"{rate:.3f}"),
                (f"{name}_seq_status", stats.sequence_status),
                (f"{name}_seq_missing_total", missing),
                (f"{name}_dds_lost_total", "unknown" if stats.dds_lost is None else stats.dds_lost),
                (f"{name}_invalid_total", stats.invalid),
            ])
            if show_details:
                rows.extend([
                    (f"{name}_last_stamp_ns", stats.last_stamp_ns),
                    (f"{name}_capture_gap_max_ms", f"{stats.max_capture_gap_ns / 1e6:.3f}"),
                    (f"{name}_stamp_regressions", stats.stamp_regressions),
                    (f"{name}_repeated_stamps", stats.repeated_stamps),
                    (f"{name}_publisher_changes", stats.publisher_changes),
                    (f"{name}_sequence_anomalies", stats.sequence_anomalies),
                ])
            self.previous_received[name] = stats.received
        rows.extend([None, ("hardware_sample_age_s", hardware_age), *self._hardware_rows])
        if not show_details:
            summary_fields = {
                "publishers_ready", "sim_wall_ratio_est", "complete_groups_total",
                "fpv_rgb_seq_missing_total", "fpv_depth_seq_missing_total",
                "top_rgb_seq_missing_total", "cpu_busiest_logical_percent",
                "ram_available_mib", "hardware_status", "gpu_query_error",
            }
            # Keep raw per-stream loss counts; presentation does not change counters.
            rows = [
                row for row in rows
                if row is not None and (
                    row[0] in summary_fields
                    or row[0].startswith("gpu") and row[0].endswith(
                        ("_util_percent", "_memory_used_mib")
                    )
                    or row[0] == "gpu_status" and row[1] != "available"
                    or row[0] == "clock_reset" and row[1]
                )
            ]
        lines = ["=" * 64]
        lines.extend("" if row is None else f"{row[0]:<36} : {row[1]}" for row in rows)
        lines.append("=" * 64)
        self.get_logger().info("\n" + "\n".join(lines))
        self.previous_report_ns = now_ns
        self.previous_clock_ns = self.clock_ns
        if self._hardware_pool is not None and self._hardware_future is None and not final:
            self._hardware_future = self._hardware_pool.submit(sample_hardware, self._hardware_previous)

    def close_hardware(self) -> None:
        if self._hardware_pool is not None:
            self._hardware_pool.shutdown(wait=True, cancel_futures=True)


def main(args=None) -> None:
    # Preserve Python's KeyboardInterrupt so a final report can precede shutdown.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = TransportCheck()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                if rclpy.ok():
                    node.report(final=True)
            finally:
                node.close_hardware()
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
