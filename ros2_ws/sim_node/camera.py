from array import array
import sys

import numpy as np
from PIL import Image as PILImage
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CameraReader: # CameraReader 負責讀取、檢查與辨識新 frame
    """Read new camera snapshots with their source rendering timestamps."""
    
    # 輸入架構參數 CameraReader(sensor, include_depth=True)
    def __init__(self, sensor, *, include_depth: bool = False):
        self.sensor = sensor
        self.include_depth = include_depth
        self.reset()

    def reset(self) -> None:
        """Reset source acquisition timing and discard the previous run's frames."""
        # Pegasus creates this Camera before it exposes it through sensor.state.
        self.sensor._camera.post_reset()
        self.last_frame = None
        self.new_frames = 0
        self.repeated_reads = 0
        self.status = "waiting_for_sensor"
        self.wait_detail = ""

    def read(self):
        """Return a new owned snapshot, or None while waiting or repeating."""
        sensor_state = self.sensor.state
        if not sensor_state or "camera" not in sensor_state:
            self.status = "waiting_for_sensor"
            return None

        camera = sensor_state["camera"]
        current = camera.get_current_frame()
        reference = current.get("rendering_frame")
        if not isinstance(reference, dict):
            self.status = "waiting_for_render"
            self.wait_detail = (
                f"source_ref_type={type(reference).__name__}, "
                f"source_ref={repr(reference)[:80]}, "
                f"source_rendering_time={current.get('rendering_time')!r}"
            )
            return None

        reference_key = (
            int(reference["referenceTimeNumerator"]),
            int(reference["referenceTimeDenominator"]),
        )
        if self.last_frame is not None:
            if reference_key == self.last_frame["render_reference"]:
                self.repeated_reads += 1
                self.status = "repeated_frame"
                return None

        # clone=True 會複製目前 frame，讓我們保留的資料具有獨立內容
        snapshot = camera.get_current_frame(clone=True)
        color = snapshot.get("rgb")
        depth = snapshot.get("distance_to_image_plane")
        if color is None or color.size == 0:
            self.status = "waiting_for_rgb"
            return None
        if self.include_depth and (depth is None or depth.size == 0):
            self.status = "waiting_for_depth"
            return None

        expected_shape = (sensor_state["height"], sensor_state["width"])
        if (
            color.ndim != 3
            or color.shape[:2] != expected_shape
            or color.shape[2] not in (3, 4)
            or color.dtype != np.uint8
        ):
            raise ValueError(
                f"{sensor_state['camera_name']}: unexpected color array "
                f"shape={color.shape}, dtype={color.dtype}"
            )
        rgb = np.ascontiguousarray(color[:, :, :3])

        if self.include_depth:
            if depth.ndim == 3 and depth.shape[2] == 1:
                depth = depth[:, :, 0]
            if depth.shape != expected_shape or depth.dtype != np.float32:
                raise ValueError(
                    f"{sensor_state['camera_name']}: unexpected depth array "
                    f"shape={depth.shape}, dtype={depth.dtype}"
                )

        rendering_time = float(snapshot["rendering_time"])
        if not np.isfinite(rendering_time) or rendering_time < 0:
            raise ValueError(f"Invalid rendering time: {rendering_time}")
        stamp_ns = round(rendering_time * 1_000_000_000)
        previous_stamp = None if self.last_frame is None else self.last_frame["stamp_ns"]
        if previous_stamp is not None and stamp_ns <= previous_stamp:
            raise ValueError(
                f"{sensor_state['camera_name']}: capture time did not advance "
                f"({previous_stamp} -> {stamp_ns}); reset reader state before a new run"
            )

        reference = snapshot["rendering_frame"]
        # read() 的輸出
        self.last_frame = {
            "camera_name": sensor_state["camera_name"],
            "camera_prim_path": sensor_state["stage_prim_path"],
            "rendering_time": rendering_time,
            "stamp_ns": stamp_ns,
            "dt_ns": None if previous_stamp is None else stamp_ns - previous_stamp,
            "render_reference": (
                int(reference["referenceTimeNumerator"]),
                int(reference["referenceTimeDenominator"]),
            ),
            "rgb": rgb,
            "depth": depth if self.include_depth else None,
        }
        self.new_frames += 1
        self.status = "new_frame"
        return self.last_frame

    def describe(self) -> str:
        """Describe the latest snapshot without logging full image arrays."""
        if self.last_frame is None or self.status.startswith("waiting_"):
            details = (
                f"status={self.status}, new_frames={self.new_frames}, "
                f"repeated_reads={self.repeated_reads}"
            )
            if self.status == "waiting_for_render":
                details += f", {self.wait_detail}"
            if self.last_frame is not None:
                details += f", last_valid_stamp_ns={self.last_frame['stamp_ns']}"
            return details

        frame = self.last_frame
        rgb = frame["rgb"]
        details = (
            f"status={self.status}, t={frame['rendering_time']:.6f}s, "
            f"stamp_ns={frame['stamp_ns']}, dt_ns={frame['dt_ns']}, "
            f"ref={frame['render_reference']}, rgb={rgb.shape}/{rgb.dtype}, "
            f"new_frames={self.new_frames}, repeated_reads={self.repeated_reads}"
        )
        if frame["depth"] is not None:
            depth = frame["depth"]
            valid = np.isfinite(depth) & (depth > 0)
            details += (
                f", depth={depth.shape}/{depth.dtype}, "
                f"depth_valid_ratio={np.count_nonzero(valid) / depth.size:.3f}"
            )
        return details


class CameraPublisher:
    """Publish optionally downsampled RGB/depth through an existing ROS node."""

    def __init__(
        self,
        node,
        *,
        rgb_topic: str,
        frame_id: str,
        depth_topic=None,
        output_size: tuple[int, int] | None = None,
    ):
        if output_size is not None:
            if len(output_size) != 2 or any(
                type(size) is not int or size <= 0 for size in output_size
            ):
                raise ValueError(
                    "Output size must contain positive integer width and height"
                )
            output_size = tuple(output_size) # 在打包訊息前縮圖
        self.output_size = output_size
        self.frame_id = frame_id
        self.rgb_publisher = node.create_publisher(
            Image, rgb_topic, qos_profile_sensor_data
        )
        self.depth_publisher = (
            None
            if depth_topic is None
            else node.create_publisher(Image, depth_topic, qos_profile_sensor_data)
        )

    def _resize_pixels(self, pixels, resample: PILImage.Resampling):
        """Downsample without changing aspect ratio or modifying source arrays."""
        height, width = pixels.shape[:2]
        if self.output_size is None or self.output_size == (width, height):
            return pixels

        output_width, output_height = self.output_size
        if (
            output_width > width
            or output_height > height
            or output_width * height != output_height * width
        ):
            raise ValueError(
                f"Output size {self.output_size} must preserve the aspect ratio "
                f"and not exceed the source size {(width, height)}"
            )

        resized = PILImage.fromarray(pixels).resize(
            self.output_size, resample=resample
        )
        return np.asarray(resized)

    # 填入尺寸、encoding、frame_id、像素資料，並用 divmod() 將擷取時間拆成 sec 與 nanosec。step 是每列資料的 byte 數；data 是打包後的像素。
    def _make_image(self, pixels, encoding: str, stamp_ns: int) -> Image:
        """Serialize output pixels with their source acquisition stamp."""
        height, width = pixels.shape[:2]
        channels = pixels.shape[2] if pixels.ndim == 3 else 1
        msg = Image()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(
            stamp_ns, 1_000_000_000
        )
        msg.header.frame_id = self.frame_id
        msg.height = int(height)
        msg.width = int(width)
        msg.encoding = encoding
        msg.is_bigendian = int(sys.byteorder == "big")
        msg.step = int(width * channels * pixels.dtype.itemsize)
        # C order packs each row; array('B') avoids per-byte Python validation.
        msg.data = array("B", pixels.tobytes(order="C"))
        return msg

    # rgb8 是每個色彩通道 8 bits；32FC1 是每個像素一個 32-bit 浮點數。
    def publish(self, frame) -> None:
        """Publish one new snapshot, preserving a shared RGB/depth timestamp."""
        rgb = self._resize_pixels(frame["rgb"], PILImage.Resampling.BOX)
        rgb_msg = self._make_image(rgb, "rgb8", frame["stamp_ns"])
        depth_msg = None
        if self.depth_publisher is not None:
            if frame["depth"] is None:
                raise ValueError("Depth publisher requires a snapshot with depth")
            # Nearest-neighbor sampling keeps source depth values in meters.
            depth = self._resize_pixels(
                frame["depth"], PILImage.Resampling.NEAREST
            )
            depth_msg = self._make_image(depth, "32FC1", frame["stamp_ns"])

        self.rgb_publisher.publish(rgb_msg)
        if depth_msg is not None:
            self.depth_publisher.publish(depth_msg)
