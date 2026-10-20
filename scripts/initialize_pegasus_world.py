import asyncio
import argparse
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.backends.px4_mavlink_backend import (
    PX4MavlinkBackend,
    PX4MavlinkBackendConfig,
)
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from isaacsim.core.utils.stage import open_stage_async
from omni.kit.quicklayout import QuickLayout

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
    QuickLayout.quick_load(None, None) # 還原整份已 Quick Save 的視窗配置
    print("Iris and PX4 MAVLink backend created")

parser = argparse.ArgumentParser()
parser.add_argument("--scene", required=True)
args = parser.parse_args()

asyncio.ensure_future(initialize_pegasus(args.scene))