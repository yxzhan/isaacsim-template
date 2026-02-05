# In[ ]:

import os
import shutil
from utils import *
from pathlib import Path

# Curent script directory
BASE_DIR = Path(__file__).resolve().parent
print(BASE_DIR)

# Check if cache exist, if not copy the precompiled cache
target_dir = "/isaac-sim/kit/cache"
source_dir = "/mnt/isaacsim-cache/cache"
dest_dir = "/isaac-sim/kit/cache"

if not os.path.isdir(target_dir):
    shutil.copytree(source_dir, dest_dir)

# ## Start SimulationApp
from isaacsim import SimulationApp
import sys
import builtins

simulation_app = SimulationApp({
    # "headless": False,
    # "hide_ui": True,
    "width": 1280,
    "height": 720,
    "renderer": "RaytracedLighting",
    "display_options": 3286,  # Setsimulation_app.update() display options to show default grid
})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
# enable_extension("omni.services.livestream.nvcf")

# ## Create a simulation environment
# 
# Setting physics to update at 200 Hz, rendering at 25 Hz, and using meters as the unit scale.

# In[ ]:

from isaacsim.core.api import World
from isaacsim.core.utils.prims import define_prim

my_world = World(stage_units_in_meters=1.0,
                 physics_dt=1 / 200,
                 rendering_dt=8 / 200)
my_world.reset()

prim = define_prim("/World/Ground", "Xform")
asset_path =  f"{BASE_DIR}/../usd/Grid/default_environment.usd"
prim.GetReferences().AddReference(asset_path)
simulation_app.update()

# ## Spawn Apartment USD to the world
import os
import numpy as np
from isaacsim.core.utils.prims import create_prim
from isaacsim.core.utils import viewports

create_prim(
    usd_path=f"{BASE_DIR}/../usd/apartment/apartmentICRA.usda",
    prim_path=f"/World/Apartment",
    position=np.array([-6, 5, 0.0701]),
)

viewports.set_camera_view(eye=np.array([-3, -2, 2]), 
                          target=np.array([1, 0, 1.5]))

simulation_app.update()

# ## Add lights

# In[ ]:


from isaacsim.core.utils.prims import create_prim

for i in range(1,4):
    gap = 4
    create_prim(
        prim_path=f"/World/Ground/Light_{i}",
        prim_type="SphereLight",
        attributes={"inputs:intensity": 10000},
        position=(-gap * i, 0, 2)
    )

simulation_app.update()

# ## Spawn Robots

# In[ ]:


import os
import numpy as np
from isaacsim.core.utils.prims import create_prim

# IAI robots
robots = [
    "pr2",
]

for i in range(len(robots)):
    robot = robots[i]
    create_prim(
        usd_path=f"{BASE_DIR}/../usd/{robot}/{robot}.usd",
        prim_path=f"/World/{robot}",
        position=np.array([-i * 2 + 2.3, 0, 0.05]),
        orientation=np.array([0, 0, 0, 1])
    )

simulation_app.update()

from isaacsim.core.prims import Articulation

robot_arts = {}

for robot in robots:
    robot_arts[robot] = Articulation(prim_paths_expr=f"/World/{robot}", name=f"my_{robot}")


# ## Low Level Joint Control

# In[ ]:

def set_joint_pos(
    robot_art,
    joint_pos: dict = None,
    teleport: bool = False,
) -> dict:
    """
    Set target joint positions for a robot Articulation.

    Args:
        robot_art: Isaac Sim ArticulationView or similar class
        joint_pos (dict): {joint_name: target_position}
        teleport (bool): If True, instantly set joint positions
    """
    if joint_pos is None:
        return None

    joint_limits = robot_art.get_dof_limits()[0]
    joint_names = np.array(robot_art.dof_names)
    target_joint_pos = robot_art.get_joint_positions()[0].copy()

    # --- Apply limits and fill in target positions ---
    for name, value in joint_pos.items():
        idx = robot_art.get_dof_index(name)
        lower, upper = joint_limits[idx]
        clamped = np.clip(value, lower, upper)
        target_joint_pos[idx] = clamped

    # --- Instant teleport mode ---
    if teleport:
        robot_art.set_joint_positions([target_joint_pos])

    # --- Motion mode: send targets ---
    robot_art.set_joint_position_targets([target_joint_pos])


# ## Create ROS Node

# In[ ]:


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np

if not rclpy.ok():
    rclpy.init(args=None)

class IsaacJointState(Node):
    def __init__(self, robot_art, timer=0.02, prefix=None):
        if prefix is not None:
            node_name = f"{prefix}_"
            topic_prefix = f"/{prefix}"
        else:
            node_name = ''
            topic_prefix = ''

        super().__init__(f"{prefix}isaac_joint_state")

        self.robot_art = robot_art

        self.publisher = self.create_publisher(
            JointState,
            f"{topic_prefix}/joint_states",
            10
        )
        self.subscription = self.create_subscription(
            JointState,
            f"{topic_prefix}/joint_command",
            self.joint_cmd_callback,
            10
        )

        # 50 Hz
        self.timer = self.create_timer(timer, self.publish_joint_states)

    def publish_joint_states(self):
        
        joint_names = self.robot_art.dof_names
        joint_states = self.robot_art.get_joint_positions()[0]

        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.name = joint_names
        msg.position = joint_states

        msg.velocity = [0.0] * len(msg.name)
        msg.effort = [0.0] * len(msg.name)

        self.publisher.publish(msg)

    def joint_cmd_callback(self, msg: JointState):        
        target_positions = {}
        for name, pos in zip(msg.name, msg.position):
            target_positions[name] = pos

        set_joint_pos(self.robot_art, target_positions)

pr2_controller_node = IsaacJointState(robot_arts["pr2"])


# ### Camera Sensor Publisher

# In[ ]:
import omni
from pxr import Usd, UsdGeom
from isaacsim.sensors.camera import Camera
import isaacsim.core.utils.numpy.rotations as rot_utils

def create_cam(prim_path, focal_length=1.0, orientation=[0, 0, 0]):
    stage = omni.usd.get_context().get_stage()
    UsdGeom.Camera.Define(stage, prim_path)
    camera = Camera(
        prim_path=prim_path,
        frequency=30,
        resolution=(640, 360),
        orientation=rot_utils.euler_angles_to_quats(np.array(orientation), degrees=True),
    )
    camera.initialize()
    camera.set_focal_length(focal_length)
    camera.set_clipping_range(near_distance=0.01, far_distance=20)
    return camera

kinect_rgb_prim = '/World/pr2/head_tilt_link/head_plate_frame/head_mount_kinect2_link/head_mount_kinect2_ir_link/head_mount_kinect2_rgb_link/head_mount_kinect2_rgb_optical_frame'
kinect_rgb_cam = create_cam(kinect_rgb_prim, focal_length=1.5)

simulation_app.update()

from sensor_msgs.msg import Image
from rclpy.node import Node
import numpy as np

class CameraPublisher(Node):
    def __init__(self, node_name='camera_publisher', topic="/camera/image_raw"):
        super().__init__(node_name)
        self.pub = self.create_publisher(Image, topic, 10)

    def publish_image(self, rgb_array):
        msg = Image()
        msg.height = rgb_array.shape[0]
        msg.width = rgb_array.shape[1]
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = rgb_array.shape[1] * 3
        msg.data = rgb_array.flatten().tolist()
        self.pub.publish(msg)

camera_node = CameraPublisher()

# ## Run the simulation loop with ROS node spinning
# 
# Once the simulation loop is running, you can drag the  sliders to control the robot's forward/backward movement and steering.

# In[ ]:

frame = 0
while True:
    render = (frame % 3) == 0
    frame += 1

    my_world.step(render=render)

    rclpy.spin_once(pr2_controller_node, timeout_sec=0.0)

    if render:
        image_raw = kinect_rgb_cam.get_rgba()
        if len(image_raw.shape) == 3:
            camera_node.publish_image(image_raw[:, :, :3])
            rclpy.spin_once(camera_node, timeout_sec=0.0)

# ## Shutdown

# In[ ]:

pr2_controller_node.destroy_node()
simulation_app.close()
