import os
from sidecar import Sidecar
import signal
import html
import time
import subprocess
import threading
import ipywidgets as widgets
import textwrap
from pathlib import Path
from IPython.display import display, HTML

try:
    import rclpy
except:
    print("rclpy not installed!")
    pass
import importlib


# Define global environment variables
os.environ["ROS_DOMAIN_ID"] = "0"
os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
os.environ["DEV_TOOLS_PATH"] = "/mnt/dev-tools"
os.environ["ISAACSIM_VERSION"] = "5.1"
os.environ["ISAACSIM_PATH"] = f"{os.environ['DEV_TOOLS_PATH']}/isaac-sim-{os.environ['ISAACSIM_VERSION']}"
os.environ["ISAACSIM_PYTHON_EXE"] = f"{os.environ['ISAACSIM_PATH']}/python.sh"

# Todos: Config local asset server
LOCAL_ASSET_PATH = ''
# if os.environ["ISAACSIM_VERSION"] == "5.0":
#     LOCAL_ASSET_PATH = ' --/persistent/isaac/asset_root/default="/mnt/dev-tools/isaacsim_assets/Assets/Isaac/5.0"'

# Bash script to setup isaac-sim python env
ISAACSIM_ENV="""
# unset virtualGL ENV (which breaks ROS2 bridge)
unset LD_PRELOAD
# Clear default ROS ENV
unset PYTHONPATH
export LD_LIBRARY_PATH=$ISAACSIM_PATH/exts/isaacsim.ros2.bridge/$ROS_DISTRO/lib
"""

# Bash script to extract Isaacsim cache
CACHE_EXTRACT_CMD="""
${DEV_TOOLS_PATH}/isaacsim-cache/extract.sh
"""

# Bash script to Archive Isaacsim cache
CACHE_ARCHIVE_CMD="""
${DEV_TOOLS_PATH}/isaacsim-cache/archive.sh
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

# Pycharm
PYCHARM_CMD="""
${DEV_TOOLS_PATH}/pycharm/bin/pycharm
"""

# gazebo
GAZEBO_CMD="""
gz sim
"""

# Unreal Editor
UNREAL_EDITOR_CMD="""
${DEV_TOOLS_PATH}/Linux_Unreal_Engine_5.5.3/Engine/Binaries/Linux/UnrealEditor
"""

# Unreal built IAI kitchen
UNREAL_DEMO_CMD="""
${DEV_TOOLS_PATH}/DemoProject/DemoProject.sh
"""

# Multiverse Testing command
MULTIVERSE_CMD="""
cd ${DEV_TOOLS_PATH}/Multiverse
pip install -r ./Multiverse-Launch/requirements.txt
pip install -r ./Multiverse-Utilities/requirements.txt
pip install -r ./Multiverse-Launch/src/multiverse_connectors/multiverse_simulators_connector/src/mujoco_connector/requirements.txt
# pip install -r ./Multiverse-Launch/src/multiverse_connectors/multiverse_ros_connector/requirements.txt
./Multiverse-Launch/bin/multiverse_launch
"""

# Get the main app list
def get_app_list():
    app_list = [
        {
            "name": "Main App",
            "path": "isaac-sim.sh",
            "command": textwrap.dedent("""
                $ISAACSIM_PATH/isaac-sim.sh
            """)
        },
        # {
        #     "name": "Main App Streaming",
        #     "path": "isaac-sim.streaming.sh",
        #     "command": textwrap.dedent("""
        #         $ISAACSIM_PATH/isaac-sim.streaming.sh
        #     """)
        # },
        {
            "name": "Streaming Client",
            "path": "",
            "command": textwrap.dedent("""
                $DEV_TOOLS_PATH/streaming-client/isaacsim-webrtc-streaming-client --no-sandbox
            """)
        },
        {
            "name": "Apartment USD",
            "path": f"{os.getcwd()}/apartment.py"
        },
        {
            "name": "Nova Carter (ROS2)",
            "path": f"{os.getcwd()}/carter_stereo.py",
            "command": textwrap.dedent(f"""
                $ISAACSIM_PYTHON_EXE {os.getcwd()}/carter_stereo.py &
                source /mnt/dev-tools/ros2_ws/install/setup.bash
                rviz2 -d {os.getcwd()}/carter_stereo.rviz &
                ros2 run rqt_robot_steering rqt_robot_steering
            """)
        },
        {
            "name": "Multiple Robot",
            "path": f"{os.getcwd()}/carter_multiple_robot_navigation.py",
            "command": textwrap.dedent(f"""
                $ISAACSIM_PYTHON_EXE {os.getcwd()}/carter_multiple_robot_navigation.py &
                source $DEV_TOOLS_PATH/ros2_ws/install/setup.bash
                ros2 run rqt_robot_steering rqt_robot_steering
            """)
        },
        {
            "name": "RL Policy",
            "path": f"{os.getcwd()}/policy.py"
        },
        {
            "name": "Franka Panda (ROS2)",
            "path": f"{os.getcwd()}/moveit.py"
        },
        {
            "name": "UR10 conveyor",
            "path": f"{os.getcwd()}/demo_ur10_conveyor_main.py"
        }
    ]

    # Add environment setup
    for i in app_list:
        if 'command' not in i:
            i['command'] = f"$ISAACSIM_PYTHON_EXE {i['path']}"
        i['command'] = ISAACSIM_ENV + i['command'] + LOCAL_ASSET_PATH
    return app_list

# Get all offical standalone_examples
def get_all_examples():
    examples_dir = Path(f"{os.environ['ISAACSIM_PATH']}/standalone_examples")
    return [
        {
            "name": f.stem,
            "path": str(f),
            "command": ISAACSIM_ENV +  f"$ISAACSIM_PYTHON_EXE {LOCAL_ASSET_PATH} {str(f)}"
        }
        
        for f in examples_dir.rglob("*.py")
        if ".ipynb_checkpoints" not in f.parts and f.name != "__init__.py"
    ]

# Global variables to control subprocesses
current_process = None
run_in_bg = widgets.Checkbox(
    value=False,
    indent=False,
    description="Run in seperate Terminals.",
    disabled=False
)
output_area = widgets.Textarea(
    rows=15,
    layout={"width": "95%"}
)
# Run bash script
def run_script(script, label):
    global current_process
    if run_in_bg.value:
        cmd = ["gnome-terminal", "--", "bash", "-c", script]
    else:
        cmd = ["bash", "-c", script]
    current_process = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        # preexec_fn=os.setsid
    )
    
    output_area.value = ""
    for line in current_process.stdout:
        output_area.value  = line + output_area.value

    current_process.wait()

# Generate app buttons
def make_button(data, script_type="bash", button_style="primary", min_width='50px', show_source=False):
    label = data['name']
    script = data['command']
    button = widgets.Button(description=label,
                            button_style=button_style,
                            tooltip=script,
                            layout=widgets.Layout(min_width=min_width))
    def on_click(b):
        match script_type:
            case _:
                thread = threading.Thread(target=run_script, args=(script, label))
                thread.start()

    button.on_click(on_click)

    if show_source:
        file_path = os.path.abspath(os.path.join(os.environ["ISAACSIM_PATH"], data['path']))
        try:
            JUPYTERHUB_USER = os.environ['JUPYTERHUB_USER']
        except KeyError:
            JUPYTERHUB_USER = None
        url_prefix = f"/user/{JUPYTERHUB_USER}" if JUPYTERHUB_USER is not None else ''
        url_param = f'folder={os.path.dirname(file_path)}&payload=[["openFile","vscode-remote://{file_path}"]]'
        url_param = html.escape(url_param)
        src_btn = widgets.HTML(
            value=f'<a href="{url_prefix}/vscode/?{url_param}" title="{file_path}" class="jupyter-button" style="width:100%;padding:0;" target="_blank">Source Code</a>',
        )
        
        return widgets.VBox([button, src_btn])
    else:
        return button

# Display app ui
def display_ui():
    app_list = get_app_list()
    examples_list = get_all_examples()
    # Main examples UI
    app_btns = widgets.VBox()

    # Other tools UI
    other_tools_btns = widgets.VBox([
        widgets.HBox([
            make_button({"command": RVIZ_CMD,  "name": "Rviz"}),
            make_button({"command": MULTIVERSE_CMD,  "name": "MULTIVERSE"}),
            make_button({"command": UNREAL_DEMO_CMD, "name": "Kitchen UE"}),

        ]),
        widgets.HBox([
            make_button({"command": APARTMENT_CMD, "name": "Apartment URDF"}),
            make_button({"command": BLENDER_CMD, "name": "Blender"}),
            make_button({"command": PYCHARM_CMD, "name": "PyCharm"}),
        ]),
        widgets.HBox([
            # make_button({"command": UNREAL_EDITOR_CMD, "name": "Unreal Editor"}),
            make_button({"command": GAZEBO_CMD, "name": "Gazebo"}),
        ]),
    ])

    # Stop button
    control_ui = widgets.HBox([
        widgets.HTML(
            value=f'<button data-commandlinker-command="notebook:interrupt-kernel" style="min-width:120px;" class="jupyter-button mod-danger">Stop</button>'
        ),
        # make_button({"command": "echo 'App Killed!'", "name": "Stop All"}, button_style="danger"),
        make_button({"command": CACHE_EXTRACT_CMD, "name": "Extract Cache"}, button_style="info"),
        make_button({"command": CACHE_ARCHIVE_CMD, "name": "Archive Cache"}, button_style="info")
    ])

    # Process Output area
    output_label = widgets.Label(value="Outputs:")
    ui = widgets.VBox([
        output_label,
        output_area
    ])

    # Show UI
    display(ui)

    # Extract isaacsim cache
    if not os.path.isdir(f"{os.environ['HOME']}/.nv"):
        run_script(CACHE_EXTRACT_CMD, "Extract Cache")

    # Full Isaac Sim example UI
    example_btns = [make_button(i, show_source=True) for i in sorted(examples_list, key=lambda x: x['name'])]
    n_cols = 3
    rows = []
    for i in range(0, len(example_btns), n_cols):
        row = widgets.HBox(example_btns[i:i+n_cols])
        rows.append(row)
    examples_list_ui = widgets.VBox(rows)

    run_script("echo 'Launcher Ready!'", "Nothing")

    # Show example buttons after cache ready
    ui.children = [
        run_in_bg,
        widgets.Label(value=f"Isaac Sim {os.environ['ISAACSIM_VERSION']}:"),
        widgets.HBox([make_button(i) for i in app_list[:2]]),
        widgets.Label(value="Isaac sim Python Examples:"),
        widgets.HBox([make_button(i, show_source=True) for i in app_list[2:5]]),
        widgets.HBox([make_button(i, show_source=True) for i in app_list[5:]]),
        widgets.Label(value="Other Tools:"),
        other_tools_btns,
        output_label,
        control_ui,
        output_area,
        widgets.Label(value="Full Isaac Sim Examples:"),
        examples_list_ui
    ]

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

# Get ROS message python class
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
