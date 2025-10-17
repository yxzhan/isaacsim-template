import os
from IPython.display import display, HTML
from sidecar import Sidecar
import threading
import subprocess
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import importlib

# publish one ROS message
def publish_one_ros_message(topic, message, msg_type):
    if not rclpy.ok():
        rclpy.init(args=None)
    node = rclpy.create_node("simple_publisher")
    publisher = node.create_publisher(msg_type, topic, 10)
    publisher.publish(message)
    node.destroy_node()

# Get ROS topic message type
def get_topic_type(topic_name):
    node = rclpy.create_node('type_finder')
    for name, types in node.get_topic_names_and_types():
        if name == topic_name:
            node.destroy_node()
            return types[0]
    node.destroy_node()
    return None

def get_message_class(type_str):
    pkg, _, msg_name = type_str.partition('/msg/')
    return getattr(importlib.import_module(f"{pkg}.msg"), msg_name)

# Fetch one ros message
def get_one_ros_message(topic_name: str, timeout: float = 5.0):
    if not rclpy.ok():
        rclpy.init(args=None)
    node = rclpy.create_node('one_shot_subscriber')

    type_str = get_topic_type(topic_name)
    if not type_str:
        node.get_logger().error(f"Topic {topic_name} not found.")
        node.destroy_node()
        rclpy.shutdown()
        return None

    msg_type = get_message_class(type_str)
    node.get_logger().info(f"Detected message type: {type_str}")

    msg_container = {'msg': None}

    def callback(msg):
        msg_container['msg'] = msg
        rclpy.shutdown()

    node.create_subscription(msg_type, topic_name, callback, 10)

    import time
    start = time.time()
    while rclpy.ok() and msg_container['msg'] is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - start > timeout:
            node.get_logger().warn("Timeout waiting for message.")
            break

    node.destroy_node()
    return msg_container

# Open remote desktop in new tab
def display_desktop(anchor="split-right"):
    """
    Display the remote desktop in a JupyterLab Sidecar tab.
    :param anchor: where the Sidecar tab will be placed.('split-right', 'split-left', 'split-top', 'split-bottom', 'tab-before', 'tab-after)
    """
    try:
        JUPYTERHUB_USER = os.environ['JUPYTERHUB_USER']
    except KeyError:
        JUPYTERHUB_USER = None
    url_prefix = f"/user/{JUPYTERHUB_USER}" if JUPYTERHUB_USER is not None else ''
    remote_desktop_url = f"{url_prefix}/desktop"
    sc = Sidecar(title='Desktop', anchor=anchor)
    with sc:
        # The inserted custom HTML and CSS snippets are to make the tab resizable
        display(HTML(f"""
            <style>
            body.p-mod-override-cursor div.iframe-widget {{
                position: relative;
                pointer-events: none;
            }}

            body.p-mod-override-cursor div.iframe-widget:before {{
                content: '';
                position: absolute;
                top: 0;
                left: 0;
                right: 0;
                bottom: 0;
                background: transparent;
            }}
            </style>
            <div class="iframe-widget" style="width: calc(100% + 10px);height:100%;">
                <iframe src="{remote_desktop_url}" width="100%" height="100%"></iframe>
            </div>
        """))

# Bash script to extract Isaacsim cache
CACHE_EXTRACT_CMD="""
${DEV_TOOLS_PATH}/isaacsim-cache/extract.sh
"""

# Bash script to Archive Isaacsim cache
CACHE_ARCHIVE_CMD="""
${DEV_TOOLS_PATH}/isaacsim-cache/archive.sh
"""

# Bash script to setup isaac-sim python env
ISAACSIM_ENV="""
# unset virtualGL ENV (which breaks ROS2 bridge)
unset LD_PRELOAD
# Clear default ROS ENV
unset PYTHONPATH
export LD_LIBRARY_PATH=$ISAACSIM_PATH/exts/isaacsim.ros2.bridge/$ROS_DISTRO/lib
"""

# Display apartment URDF
APARTMENT_CMD="""
source ${DEV_TOOLS_PATH}/ros2_ws/install/setup.bash
ros2 launch iai_apartment apartment_display.launch.py
"""

# rviz
RVIZ_CMD="""
rviz2
"""

# Blender
BLENDER_CMD="""
${DEV_TOOLS_PATH}/blender-4.5.3-linux-x64/blender
"""

# Unreal built IAI kitchen
UNREAL_DEMO_CMD="""
${DEV_TOOLS_PATH}/DemoProject/DemoProject.sh
"""

# Unreal built IAI kitchen
MULTIVERSE_CMD="""
cd ${DEV_TOOLS_PATH}/Multiverse
pip install -r ./Multiverse-Launch/requirements.txt
pip install -r ./Multiverse-Utilities/requirements.txt
pip install -r ./Multiverse-Launch/src/multiverse_connectors/multiverse_simulators_connector/src/mujoco_connector/requirements.txt
# pip install -r ./Multiverse-Launch/src/multiverse_connectors/multiverse_ros_connector/requirements.txt
./Multiverse-Launch/bin/multiverse_launch
"""
