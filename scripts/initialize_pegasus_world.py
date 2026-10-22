import asyncio
import argparse
import sys
import time
from pathlib import Path

import rclpy
import omni.timeline
from rclpy.signals import SignalHandlerOptions
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.backends.px4_mavlink_backend import (
    PX4MavlinkBackend,
    PX4MavlinkBackendConfig,
)
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.utils.stage import open_stage_async
from omni.kit.quicklayout import QuickLayout

project_root = str(Path(__file__).resolve().parents[1])
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from ros2_ws.sim_node.clock import ClockPublisher
from ros2_ws.sim_node.camera import CameraPublisher, CameraReader

# 初始化
async def initialize_pegasus(scene_path):
    pg = PegasusInterface()
    if pg.get_all_vehicles():
        print("Pegasus vehicle already active; initialization skipped")
        return

    print(f"Loading scene: {scene_path}")
    stage_opened, error = await open_stage_async(scene_path)
    if not stage_opened:
        raise RuntimeError(f"Failed to open scene: {error}")
        
    pg.initialize_world()
    await pg.world.initialize_simulation_context_async()
    print("Pegasus World ready")
    stage = pg.world.stage
    if stage.GetPrimAtPath("/World/quadrotor"):
        stage.RemovePrim("/World/quadrotor")

    mavlink_config = PX4MavlinkBackendConfig({
        "vehicle_id": 0,
        "connection_type": "tcpin",
        "connection_ip": "localhost",
        "connection_baseport": 4560,
        "px4_autolaunch": True,
        "px4_dir": pg.px4_path,
        "px4_vehicle_model": pg.px4_default_airframe,
        "enable_lockstep": True,
    })
    backend = PX4MavlinkBackend(mavlink_config)

    vehicle_config = MultirotorConfig()
    vehicle_config.backends = [backend]
    vehicle_config.graphical_sensors = [
        MonocularCamera("fpv_camera"),
        MonocularCamera(
            "top_camera",
            config={
                "position": [0.0, 0.0, -0.03], # 相對於 body 原點的 [x, y, z]，單位為公尺
                "orientation": [0.0, -90.0, 180.0],
                "depth": False,
            },
        ),
    ]

# fpv_camera 預設 /home/noel_614420090/PegasusSimulator/extensions/pegasus.simulator/pegasus/simulator/logic/graphical_sensors/monocular_camera.py
#"position": [0.30, 0.0, 0.0]
# "orientation": [0.0, 0.0, 180.0]
# "depth": True

    Multirotor(
        stage_prefix="/World/quadrotor",
        usd_file=ROBOTS["Iris"],
        vehicle_id=0,
        init_pos=[0.0, 0.0, 0.07],
        init_orientation=[0.0, 0.0, 0.0, 1.0],
        config=vehicle_config,
    )

    """
    Isaac Sim physics step
    → 呼叫 publish_clock()
    → 取得目前模擬時間
    → ClockPublisher 轉換時間格式
    → 發布 /clock
    """
    if not rclpy.ok():
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)

    sim_node = rclpy.create_node("sim_node", use_global_arguments=False)
    clock_publisher = ClockPublisher(sim_node)

    def publish_clock(_dt):
        sim_time = SimulationManager.get_simulation_time()
        clock_publisher.publish_sim_time(sim_time)
        rclpy.spin_once(sim_node, timeout_sec=0.0)

    pg.world.add_physics_callback("publish_clock", publish_clock)
    sim_node.get_logger().info(
        "Publishing Isaac Sim time on /clock at each physics step"
    )
    
    # 前視 Reader 讀取 RGB 與 depth；俯視 Reader 讀取 RGB
    camera_readers = {
        "fpv_camera": CameraReader(
            vehicle_config.graphical_sensors[0], include_depth=True
        ),
        "top_camera": CameraReader(vehicle_config.graphical_sensors[1]),
    }
    camera_output_size = (640, 400)  # 提供給前視與俯視 Publisher
    # 建立前視、俯視兩個 Publisher 輔助物件，共用既有 sim_node
    camera_publishers = {
        "fpv_camera": CameraPublisher(
            sim_node,
            rgb_topic="/fpv_camera/rgb",
            depth_topic="/fpv_camera/depth",
            frame_id="fpv_camera_optical_frame",
            output_size=camera_output_size,
        ),
        "top_camera": CameraPublisher(
            sim_node,
            rgb_topic="/top_camera/rgb",
            frame_id="top_camera_optical_frame",
            output_size=camera_output_size,
        ),
    }

    """
    讀取兩個相機的新 frame
    → 確認兩者 stamp 與 render_reference 相同
    → 距上次發布至少經過 200 ms 模擬時間
    → 發布前視 RGB、前視 depth、俯視 RGB
    """
    camera_publish_period_ns = 200_000_000 # 200 ms 模擬時間
    last_camera_publish_stamp_ns = None # 最近發布那組影像的擷取時間
    camera_published_groups = 0 # 完成三路發布呼叫的組數
    next_camera_log_time = 0.0

    def reset_camera_readers(event):
        nonlocal next_camera_log_time, last_camera_publish_stamp_ns
        nonlocal camera_published_groups
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            for reader in camera_readers.values():
                reader.reset()
            last_camera_publish_stamp_ns = None
            camera_published_groups = 0
            next_camera_log_time = 0.0
            sim_node.get_logger().info(
                "Camera acquisition timing, reader state, and publish gate reset after Stop"
            )

    pg.world.add_timeline_callback("reset_camera_readers", reset_camera_readers)

    def read_cameras(_event):
        nonlocal next_camera_log_time, last_camera_publish_stamp_ns
        nonlocal camera_published_groups
        if not pg.world.is_playing():
            return

        try:
            for reader in camera_readers.values():
                # Reader 等待影像或讀到重複 frame 時，不會重複發布
                reader.read()

            fpv_frame = camera_readers["fpv_camera"].last_frame
            top_frame = camera_readers["top_camera"].last_frame
            readers_ready = all(
                reader.status in ("new_frame", "repeated_frame")
                for reader in camera_readers.values()
            )
            frames_aligned = (
                readers_ready
                and fpv_frame is not None
                and top_frame is not None
                and fpv_frame["stamp_ns"] == top_frame["stamp_ns"]
                and fpv_frame["render_reference"] == top_frame["render_reference"]
            )
            if frames_aligned:
                stamp_ns = fpv_frame["stamp_ns"]
                interval_ready = (
                    last_camera_publish_stamp_ns is None
                    or stamp_ns - last_camera_publish_stamp_ns >= camera_publish_period_ns # 核心判斷
                )
                if interval_ready:
                    camera_publishers["fpv_camera"].publish(fpv_frame)
                    camera_publishers["top_camera"].publish(top_frame)
                    last_camera_publish_stamp_ns = stamp_ns
                    # Count completed publish calls, not subscriber deliveries.
                    camera_published_groups += 1
        except Exception:
            pg.world.remove_render_callback("read_cameras")
            sim_node.get_logger().error(
                "Camera reading and publishing stopped after an error; see traceback"
            )
            raise

        # Wall time only limits console output; capture stamps come from snapshots.
        now = time.monotonic()
        if now < next_camera_log_time:
            return
        next_camera_log_time = now + 1.0
        for name, reader in camera_readers.items():
            sim_node.get_logger().info(f"[camera] {name}: {reader.describe()}")
        sim_node.get_logger().info(
            f"[camera] published_groups={camera_published_groups}, "
            f"latest_published_stamp_ns={last_camera_publish_stamp_ns}, "
            f"min_publish_interval_ns={camera_publish_period_ns}"
        )
        if readers_ready and fpv_frame is not None and top_frame is not None:
            delta_ns = top_frame["stamp_ns"] - fpv_frame["stamp_ns"]
            same_reference = top_frame["render_reference"] == fpv_frame["render_reference"]
            sim_node.get_logger().info(
                f"[camera] latest_stamp_delta_ns(top-fpv)={delta_ns}, "
                f"same_render_reference={same_reference}"
            )

    pg.world.add_render_callback("read_cameras", read_cameras)
    sim_node.get_logger().info(
        "Camera snapshot inspection and Image publishing enabled: "
        "/fpv_camera/rgb, /fpv_camera/depth, /top_camera/rgb "
        f"(output {camera_output_size[0]}x{camera_output_size[1]}, "
        "RGB BOX, depth NEAREST, max 5 Hz in simulation time, "
        "sensor data QoS, source capture timestamps)"
    )

    QuickLayout.quick_load(None, None) # 還原整份已 Quick Save 的視窗配置
    print("Iris and PX4 MAVLink backend created")

parser = argparse.ArgumentParser()
parser.add_argument("--scene", required=True)
args = parser.parse_args()

asyncio.ensure_future(initialize_pegasus(args.scene))
