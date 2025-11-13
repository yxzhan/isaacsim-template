#!/usr/bin/env python
# coding: utf-8

# # Isaac Sim Jupyter Notebook Tutorial
# 
# This notebook will demonstrate step by step how to load the IAI apartment and a robot into the simulation environment.
#  
# > Select the jupyter kernel `Isaac Sim Python 3.11` if you are running in VScode.
# >
# > "Control + Enter" to execute the selected code cell. 
# 
# <!-- <button data-commandlinker-command="notebook:restart" class="jupyter-button">Force Stop</button> -->

# ## Start the GPU monitor and virtual desktop
# 
# Make sure the GPU has at least 3000M free memory.
# 
# If the virtual desktop does not pop up, manually click the "Open Desktop in new Tab".

# In[ ]:


from IPython import get_ipython
in_notebook = get_ipython().__class__.__name__ == "ZMQInteractiveShell"

from utils import *
# Extract precompiled cache
# run_script(CACHE_EXTRACT_CMD)

# only runs in Jupyter Notebook
if in_notebook:
    from gpu_monitor import GPUMonitor
    # Monitor GPU usage
    gpu_monitor = GPUMonitor()
    # Open Desktop in sidecar
    display_desktop()


# ## Start SimulationApp
# 
# The application window is frozen and non-interactive, which is normal.
# 
# <div style="color:red">This will take some time, so give it a minute and wait for it to say <b>"SimulationApp Ready!"</b> before you go to next step.</div>

# In[ ]:


from isaacsim import SimulationApp
from IPython.display import clear_output
import sys
import builtins

original_stdout = sys.stdout
original_stderr = sys.stderr

simulation_app = SimulationApp({
    "headless": False,
    "hide_ui": not in_notebook,
    "width": 1280,
    "height": 960,
    "renderer": "RaytracedLighting",
    "display_options": 3286,  # Setsimulation_app.update() display options to show default grid
})

# Fix the issue where notebook output is being hijacked by Isaac Sim.
sys.stdout = original_stdout
sys.stderr = original_stderr
clear_output(wait=True)
print('SimulationApp Ready!')


# ## Create a simulation environment
# 
# Setting physics to update at 200 Hz, rendering at 25 Hz, and using meters as the unit scale.

# In[ ]:


from isaacsim.core.api import World
from isaacsim.core.utils.prims import define_prim
from isaacsim.storage.native import get_assets_root_path

my_world = World(stage_units_in_meters=1.0,
                 physics_dt=1 / 200,
                 rendering_dt=8 / 200)
my_world.reset()

assets_root_path = get_assets_root_path()
prim = define_prim("/World/Ground", "Xform")
asset_path = assets_root_path + "/Isaac/Environments/Grid/default_environment.usd"
prim.GetReferences().AddReference(asset_path)


# ## Refresh View
# 
# Create a helper function to step the simulation world and update view. For the later code cells, wait for the progress bar to complete before proceeding to the next step.

# In[ ]:


from tqdm import tqdm

def refresh_view(steps=10, desc="World Stepping"):
    bar_format = "{l_bar}{bar}| {n_fmt}/{total_fmt} steps] {elapsed_s:.2f}s"
    for i in tqdm(range(steps), desc=desc, ncols=60, bar_format=bar_format, file=sys.stdout):
        # clear_output(wait=True)
        my_world.step(render=True)

refresh_view()


# ## Spawn Apartment USD to the world
# 
# Asset Source:
# https://github.com/Multiverse-Framework/Multiverse-Tutorials/tree/main/resources/apartmentICRA_full/usda

# In[ ]:


import numpy as np
from isaacsim.core.utils.prims import create_prim
from isaacsim.core.utils import viewports

env_usd_path = "/mnt/dev-tools/usd/apartment/apartmentICRA.usda"
create_prim(
    usd_path=env_usd_path,
    prim_path=f"/World/Apartment",
    position=np.array([-6, 5, 0.0701]),
    # orientation=np.array([0, 0, 0, 1])
)

viewports.set_camera_view(eye=np.array([-15, 0, 1.5]), 
                          target=np.array([0, 0, 0.5]))

refresh_view(steps=50, desc="Spawn Apartment")


# ## Add lights

# In[ ]:


from isaacsim.core.utils.prims import create_prim

for i in range(1,4):
    gap = 4
    create_prim(
        prim_path=f"/World/Ground/Light_{i}",
        prim_type="SphereLight",
        attributes={"inputs:intensity": 15000},
        position=(-gap * i, 0, 2)
    )
    refresh_view(steps=10, desc="Add Lights")


# ## Move viewport camera

# In[ ]:


import numpy as np
from isaacsim.core.utils import viewports
from omni.kit.viewport.utility.camera_state import ViewportCameraState

def move_camera(position, target, steps=100, smooth=True):
    end_pos = np.array(position)
    end_target = np.array(target)

    if smooth:
        # get current camera state
        cam_state = ViewportCameraState()
        start_pos = np.asarray(cam_state.position_world, dtype=np.double)
        start_target = np.asarray(cam_state.target_world, dtype=np.double)
        offset_pos = (end_pos - start_pos) / steps
        offset_target = (end_target - start_target) / steps
        for i in tqdm(range(steps), desc="Move Camera", file=sys.stdout):
            viewports.set_camera_view(eye=start_pos + offset_pos * i, 
                                      target=start_target + offset_target * i )
            my_world.step(render=True)
    else:
        viewports.set_camera_view(eye=end_pos, target=end_target)

    refresh_view(steps=10)

move_camera(position=[-5, 0, 1.5], target=[1, 0, 1.5], steps=200)


# ## Spawn Robot Anymal
# 
# The USD file for the Anymal robot is provided by Isaac Sim itself. 
# 
# You will see the robot collapse on the ground because no control commands have been sent to it yet.

# In[ ]:


from isaacsim.robot.policy.examples.robots import AnymalFlatTerrainPolicy

robot = AnymalFlatTerrainPolicy(
    prim_path="/World/Anymal1",
    name="Anymal",
    position=np.array([0, 0, 0.6]),
)
refresh_view(steps=80, desc="Spawn Anymal")


# ## Add physics callback function to control the robot
# 
# The control command is a 3-element array, where the first value represents forward velocity, the second represents lateral (left/right) movement, and the third represents rotation. Value range is from -1 to 1.

# In[ ]:


first_step = True
commands = [0.0, 0.0, 0.0]

def on_physics_step(step_size) -> None:
    global first_step, commands
    if first_step:
        robot.initialize()
        first_step = False
    else:
        robot.forward(step_size, commands)

my_world.add_physics_callback("physics_step", callback_fn=on_physics_step)

refresh_view(steps=80, desc="Physics callback")


# ## Restart Simulation and initialize robot

# In[ ]:


first_step = True
my_world.reset()
commands = [0.0, 0.0, 0.0]
refresh_view(steps=50, desc="Restart Simulation")


# ## Send Control Commands

# In[ ]:


commands = [-0.3, 0.0, 0.0]
refresh_view(steps=100, desc="Move Backward")

commands = [0.0, -0.3, 0.0]
refresh_view(steps=100, desc="Move Right")

commands = [0.0, 0.0, 0.8]
refresh_view(steps=100, desc="Turn Around")

commands = [0.0, 0.0, 0.0]
refresh_view(steps=50, desc="Stop Moving")


# ## Spawn IAI robots
# 
# Todos: Articulate robots

# In[ ]:


import os
import numpy as np
from isaacsim.core.utils.prims import create_prim
from isaacsim.core.utils import viewports

move_camera(position=[-12, -1, 2], target=[0, 0, 0.5], steps=50)

object_list = [
    "pr2",
    "stretch",
    "hsrb",
    "tiago_dual",
]

for i in range(len(object_list)):
    obj = object_list[i]
    create_prim(
        usd_path=f"{os.getcwd()}/../usd/{obj}/{obj}.usd",
        prim_path=f"/World/{obj}",
        position=np.array([- i * 2 - 1, 1, 0.2]),
        orientation=np.array([0, 0, 0, 1])
    )
    refresh_view(steps=20, desc=f"Spawn {obj}")

refresh_view(steps=50)


# ## Delete Objects

# In[ ]:


# from isaacsim.core.utils.prims import delete_prim

# for obj in object_list:
#     delete_prim(f"/World/{obj}")
#     refresh_view(steps=10, desc=f"Delete {obj}")


# ## Enable ROS2 Bridge
# 
# Next, we will control the robot's movement through ROS messages.

# In[ ]:


from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.ros2.bridge")
# enable_extension("omni.physx.supportui")


# ## Create a ROS subscriber
# 
# Listen to messages on topic `cmd_vel`

# In[ ]:


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

if not rclpy.ok():
    rclpy.init(args=None)

# Update robot command
def cmd_vel_callback(msg):
    global commands
    commands = [msg.linear.x, 0.0, msg.angular.z]

node = rclpy.create_node('cmd_vel_listener')
node.create_subscription(Twist, 'cmd_vel', cmd_vel_callback, 10)


# ## Open `rqt_robot_steering`
# 
# This is a GUI tool that broadcasts cmd_vel messages.

# In[ ]:


import subprocess
import os
os.environ.pop("PYTHONPATH", None)
steer_gui = """
source /opt/ros/jazzy/setup.bash
source $DEV_TOOLS_PATH/ros_ws/apartment_ws/install/setup.bash
ros2 run rqt_robot_steering rqt_robot_steering
"""
cmd = ["bash", "-c", steer_gui]
subprocess.Popen(
    cmd,
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True,
    bufsize=1,
)


# ## Run the simulation loop with ROS node spinning
# 
# Once the simulation loop is running, you can drag the  sliders to control the robot's forward/backward movement and steering.

# In[ ]:


steps = 1500
desc = "ROS Spin"
bar_format = "{l_bar}{bar}| {n_fmt}/{total_fmt} steps] {elapsed_s:.2f}s"
for i in tqdm(range(steps), desc=desc, ncols=60, bar_format=bar_format):
    my_world.step(render=True)
    rclpy.spin_once(node, timeout_sec=0.0)


# ## Run the simulation continuously
# 
# Once the following code cell is executed, it will enter an infinite loop. You can only terminate the entire program by restarting the kernel, the "Shutdown" button below is a shortcut, after which you'll need to rerun the previous code.
# 
# <button data-commandlinker-command="notebook:restart-clear-output" class="jupyter-button">Shutdown</button>

# In[ ]:


# # Reset Status
# first_step = True
# commands = [0.0, 0.0, 0.0]
# my_world.reset()

# # Start simulation loop
# while simulation_app.is_running():
#     my_world.step(render=True)
#     rclpy.spin_once(node, timeout_sec=0.0)
#     # reset robot status when stop in UI
#     if my_world.is_stopped():
#         first_step = True
#         commands = [0.0, 0.0, 0.0]
# if node:
#     node.destroy_node()
# rclpy.shutdown()


# ## Shutdown

# In[ ]:


simulation_app.close()


# !jupyter nbconvert --to python apartment.ipynb

# ## Covert notebook to Python script
# 
# ```
# jupyter nbconvert --to python apartment.ipynb
# ```

# In[ ]:




