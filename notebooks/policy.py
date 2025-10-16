from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": False,
    # "hide_ui": True,
    "width": 1280,
    "height": 960,
    # "window_width": 1280,
    # "window_height": 960,
    # "headless": True,
    # "hide_ui": False,  # Show the GUI
    # "disable_viewport_updates": True,
    "renderer": "RaytracedLighting",
    "display_options": 3286,  # Set display options to show default grid
})

import argparse
import carb
import math
import numpy as np
import omni.appwindow  # Contains handle to keyboard
import omni.kit.app
from isaacsim.core.api import World
from isaacsim.core.utils.prims import define_prim
from isaacsim.core.utils import viewports
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.robot.policy.examples.robots import H1FlatTerrainPolicy, SpotFlatTerrainPolicy, AnymalFlatTerrainPolicy
from isaacsim.storage.native import get_assets_root_path

parser = argparse.ArgumentParser(description="Define the number of robots.")
parser.add_argument("--num-robots", type=int, default=6, help="Number of robots (default: 1)")
parser.add_argument(
    "--env-url",
    # default="/Isaac/Environments/Grid/default_environment.usd",
    default="/Isaac/Environments/Simple_Warehouse/warehouse.usd",
    # default="/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd",
    # default="/Isaac/Environments/Simple_Warehouse/full_warehouse.usd",
    # default="/Isaac/Environments/Simple_Room/simple_room.usd",
    required=False,
    help="Path to the environment url",
)
args = parser.parse_args()
print(f"Number of robots: {args.num_robots}")

# Default Livestream settings
# app.set_setting("/app/window/drawMouse", True)

# Enable extension
# enable_extension("omni.services.livestream.nvcf")
enable_extension("omni.physx.supportui")


first_step = True
reset_needed = False
robots = []

cols = math.ceil(math.sqrt(args.num_robots))
rows = math.ceil(args.num_robots / cols)
init_pos=(-2, 2)
init_space = 2
cam_pos = [5, -5, 3]
cam_direct = [init_pos[0], init_pos[1], 1.5]

base_commands = [np.zeros(3) for _ in range(args.num_robots)]

N = 60  
frame_count = 0

# physics callback
def on_physics_step(step_size) -> None:
    global first_step, reset_needed
    if first_step:
        for robot in robots:
            robot.initialize()
        first_step = False
    elif reset_needed:
        my_world.reset(True)
        reset_needed = False
        first_step = True
    else:
        for idx, robot in enumerate(robots):
            robot.forward(step_size, base_commands[idx])

# spawn world
my_world = World(stage_units_in_meters=1.0, physics_dt=1 / 200, rendering_dt=8 / 200)
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")

viewports.set_camera_view(eye=np.array(cam_pos), target=np.array(cam_direct))

# spawn scene
# prim = define_prim("/World/Ground", "Xform")
prim = define_prim("/World/Warehouse", "Xform")
asset_path = assets_root_path + args.env_url
prim.GetReferences().AddReference(asset_path)


# spawn robots
for i in range(0, args.num_robots):
    robot_type = i if i < 3 else np.random.choice([0, 1, 2])
    row = (i // cols - cols // 2) * init_space + init_pos[0]
    col = (i % cols - rows // 2) * init_space + init_pos[1]
    if robot_type == 0:
        robot = H1FlatTerrainPolicy(
            prim_path="/World/H1_" + str(i),
            name="H1_" + str(i),
            usd_path=assets_root_path + "/Isaac/Robots/Unitree/H1/h1.usd",
            position=np.array([row, col, 1.05]),
        )
    elif robot_type == 1:
        robot = SpotFlatTerrainPolicy(
            prim_path="/World/Spot_" + str(i),
            name="Spot_" + str(i),
            position=np.array([row, col, 0.8]),
        )
    else:
        robot = AnymalFlatTerrainPolicy(
            prim_path="/World/Anymal_" + str(i),
            name="Anymal_" + str(i),
            position=np.array([row, col, 0.7]),
        )
    robots.append(robot)


my_world.reset()
my_world.add_physics_callback("physics_step", callback_fn=on_physics_step)


# main loop
while simulation_app.is_running():
    my_world.step(render=True)

    if my_world.is_stopped():
        reset_needed = True

    if my_world.is_playing():
        frame_count += 1
        if frame_count % N == 0:
            for idx in range(args.num_robots):
                if isinstance(robots[idx], H1FlatTerrainPolicy):
                    vx = np.random.uniform(0, 0.5)
                    vy = 0
                    wz = np.random.uniform(-0.5, 0.5)
                elif isinstance(robots[idx], SpotFlatTerrainPolicy):
                    vx = np.random.uniform(-0.5, 2)
                    vy = np.random.uniform(-0.8, 0.8)
                    wz = np.random.uniform(-0.8, 0.8)
                else:
                    vx = np.random.uniform(-0.5, 1)
                    vy = np.random.uniform(-0.5, 0.5)
                    wz = np.random.uniform(-0.5, 0.5)
                base_commands[idx] = np.array([vx, vy, wz])

simulation_app.close()
