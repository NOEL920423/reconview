import asyncio
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from rclpy.parameter import Parameter

# 初始化
async def initialize_pegasus():
    pg = PegasusInterface()
    if pg.get_all_vehicles():
        print("Pegasus vehicle already active; initialization skipped")
        return
    pg.initialize_world()
    await pg.world.initialize_simulation_context_async()
    print("Pegasus World ready")
    stage = pg.world.stage
    if stage.GetPrimAtPath("/World/quadrotor"):
        stage.RemovePrim("/World/quadrotor")

    backend = ROS2Backend(vehicle_id=0, config={"namespace": "drone"})
    backend.node.set_parameters([Parameter("use_sim_time", value=True)])

    vehicle_config = MultirotorConfig()
    vehicle_config.backends = [backend]
    vehicle_config.graphical_sensors = [MonocularCamera("camera")]

    Multirotor(
        stage_prefix="/World/quadrotor",
        usd_file=ROBOTS["Iris"],
        vehicle_id=0,
        init_pos=[0.0, 0.0, 1.0],
        init_orientation=[0.0, 0.0, 0.0, 1.0],
        config=vehicle_config,
    )
    print("Iris and ROS 2 backend created")

asyncio.ensure_future(initialize_pegasus())