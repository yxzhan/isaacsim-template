"""
Utility functions for Isaac Sim application launcher and ROS2 integration.
Provides UI components, process management, and ROS2 utilities.
"""

import os
import html
import threading
import subprocess
import importlib
import textwrap
from pathlib import Path

import ipywidgets as widgets
from IPython.display import display, HTML

try:
    import rclpy
except ImportError:
    print("rclpy not installed! Have you source ROS2 environment?")
    pass

try:
    from sidecar import Sidecar
except ImportError:
    print("Sidecar not available!")


# =============================================================================
# GLOBAL ENVIRONMENT VARIABLES
# =============================================================================

os.environ["ROS_DOMAIN_ID"] = "0"
os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
os.environ["DEV_TOOLS_PATH"] = "/mnt/dev-tools"
os.environ.setdefault("ISAACSIM_VERSION", "5.1")
os.environ["ISAACSIM_PATH"] = f"{os.environ['DEV_TOOLS_PATH']}/isaac-sim-{os.environ['ISAACSIM_VERSION']}"
os.environ["ISAACSIM_PYTHON_EXE"] = f"{os.environ['ISAACSIM_PATH']}/python.sh"

# TODO: Configure local asset server
LOCAL_ASSET_PATH = ''
# if os.environ["ISAACSIM_VERSION"] == "5.0":
#     LOCAL_ASSET_PATH = ' --/persistent/isaac/asset_root/default="/mnt/dev-tools/isaacsim_assets/Assets/Isaac/5.0"'

# Bash script to setup Isaac Sim Python environment
ISAACSIM_ENV = """
# unset virtualGL ENV (which breaks ROS2 bridge)
unset LD_PRELOAD
# Clear default ROS ENV
unset PYTHONPATH
export LD_LIBRARY_PATH=$ISAACSIM_PATH/exts/isaacsim.ros2.bridge/$ROS_DISTRO/lib
"""

# Bash script to extract Isaac Sim cache
CACHE_EXTRACT_CMD = """
${DEV_TOOLS_PATH}/isaacsim-cache/extract.sh
"""

# Bash script to archive Isaac Sim cache
CACHE_ARCHIVE_CMD = """
${DEV_TOOLS_PATH}/isaacsim-cache/archive.sh
"""

# =============================================================================
# APPLICATION MANAGEMENT
# =============================================================================

def get_app_list():
    """
    Get the list of main applications with their commands and paths.
    
    Returns:
        list: List of dictionaries containing app information
    """
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

    # Add environment setup to each app
    for app in app_list:
        if 'command' not in app:
            app['command'] = f"$ISAACSIM_PYTHON_EXE {app['path']}"
        app['command'] = ISAACSIM_ENV + app['command'] + LOCAL_ASSET_PATH
    
    return app_list

def get_tools_list():
    """
    Get the list of tools with their commands and display names.
    
    Returns:
        list: List of dictionaries containing tool information
    """
    tools_list = [
        {
            "name": "Rviz",
            "command": "rviz2",
            "description": "ROS Visualization Tool"
        },
        {
            "name": "MULTIVERSE",
            "command": textwrap.dedent(f"""
                cd $DEV_TOOLS_PATH/Multiverse
                pip install -r ./Multiverse-Launch/requirements.txt
                pip install -r ./Multiverse-Utilities/requirements.txt
                pip install -r ./Multiverse-Launch/src/multiverse_connectors/multiverse_simulators_connector/src/mujoco_connector/requirements.txt
                ./Multiverse-Launch/bin/multiverse_launch
            """),
            "description": "Multiverse testing"
        },
        {
            "name": "Kitchen UE",
            "command": "$DEV_TOOLS_PATH/DemoProject/DemoProject.sh",
            "description": "Unreal Engine Kitchen Demo"
        },
        {
            "name": "Apartment URDF",
            "command": textwrap.dedent(f"""
                source $DEV_TOOLS_PATH/ros2_ws/install/setup.bash
                ros2 launch iai_apartment apartment_display.launch.py
            """),
            "description": "Display Apartment URDF Model"
        },
        {
            "name": "Blender",
            "command": "$DEV_TOOLS_PATH/blender-4.5.3-linux-x64/blender",
            "description": "3D Modeling and Animation"
        },
        {
            "name": "PyCharm",
            "command": "$DEV_TOOLS_PATH/pycharm/bin/pycharm",
            "description": "Python IDE"
        },
        {
            "name": "Gazebo",
            "command": "gz sim",
            "description": "Gazebo Simulation"
        },
        # {
        #     "name": "Unreal Editor",
        #     "command": "$DEV_TOOLS_PATH/Linux_Unreal_Engine_5.5.3/Engine/Binaries/Linux/UnrealEditor",
        #     "description": "Unreal Engine Editor"
        # }
    ]
    
    return tools_list


def get_all_examples_list():
    """
    Get all official standalone examples from Isaac Sim.
    
    Returns:
        list: List of dictionaries containing example information
    """
    examples_dir = Path(f"{os.environ['ISAACSIM_PATH']}/standalone_examples")
    examples = []
    
    for example_file in examples_dir.rglob("*.py"):
        # Skip checkpoint files and __init__.py
        if ".ipynb_checkpoints" not in example_file.parts and example_file.name != "__init__.py":
            examples.append({
                "name": example_file.stem,
                "path": str(example_file),
                "command": ISAACSIM_ENV + f"$ISAACSIM_PYTHON_EXE {LOCAL_ASSET_PATH} {str(example_file)}"
            })
    
    return examples


# =============================================================================
# PROCESS MANAGEMENT
# =============================================================================

# Global variables to control subprocesses
current_process = None

# UI widgets
run_in_bg = widgets.Checkbox(
    value=False,
    indent=False,
    description="Run demos in separate terminals.",
    disabled=False
)

output_area = widgets.Textarea(
    rows=15,
    layout={"width": "95%"}
)


def run_script(script, label):
    """
    Run a bash script in a subprocess.
    
    Args:
        script (str): Bash script to execute
        label (str): Label for the process (for logging)
    """
    global current_process
    
    # Determine command based on run mode
    if run_in_bg.value:
        cmd = ["gnome-terminal", "--", "bash", "-c", script]
    else:
        cmd = ["bash", "-c", script]
    
    # Start the subprocess
    current_process = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    
    # Clear and update output area
    output_area.value = ""
    for line in current_process.stdout:
        output_area.value = line + output_area.value

    current_process.wait()


# =============================================================================
# UI COMPONENTS
# =============================================================================

def make_button(data, script_type="bash", button_style="primary", min_width='50px', show_source=False):
    """
    Create a button widget for launching applications.
    
    Args:
        data (dict): Button data containing name and command
        script_type (str): Type of script to run
        button_style (str): Style of the button
        min_width (str): Minimum width of the button
        show_source (bool): Whether to show source code link
    
    Returns:
        widgets.VBox or widgets.Button: Button widget
    """
    label = data['name']
    script = data['command']
    
    button = widgets.Button(
        description=label,
        button_style=button_style,
        tooltip=script,
        layout=widgets.Layout(min_width=min_width)
    )
    
    def on_click(b):
        """Handle button click event."""
        if script_type == "bash":
            thread = threading.Thread(target=run_script, args=(script, label))
            thread.start()
    
    button.on_click(on_click)
    
    # Add source code link if requested
    if show_source:
        file_path = os.path.abspath(os.path.join(os.environ["ISAACSIM_PATH"], data['path']))
        
        try:
            jupyterhub_user = os.environ['JUPYTERHUB_USER']
        except KeyError:
            jupyterhub_user = None
            
        url_prefix = f"/user/{jupyterhub_user}" if jupyterhub_user is not None else ''
        url_param = f'folder={os.path.dirname(file_path)}&payload=[["openFile","vscode-remote://{file_path}"]]'
        url_param = html.escape(url_param)
        
        src_btn = widgets.HTML(
            value=f'<a href="{url_prefix}/vscode/?{url_param}" title="{file_path}" '
                  f'class="jupyter-button" style="width:100%;padding:0;" target="_blank">Source Code</a>',
        )
        
        return widgets.VBox([button, src_btn])
    else:
        return button
    

def create_button_grid(items, buttons_per_row=3, show_source=False, button_style="primary"):
    """
    Create a grid of buttons from a list of items.
    
    Args:
        items (list): List of dictionaries containing button data
        buttons_per_row (int): Number of buttons per row
        show_source (bool): Whether to show source code links
        button_style (str): Style for the buttons
        
    Returns:
        widgets.VBox: Vertical box containing rows of buttons
    """
    # Create buttons for all items
    buttons = [make_button(item, button_style=button_style, show_source=show_source) for item in items]
    
    # Group buttons into rows
    rows = []
    for i in range(0, len(buttons), buttons_per_row):
        row = widgets.HBox(buttons[i:i+buttons_per_row])
        rows.append(row)
    
    return widgets.VBox(rows)


def display_ui():
    """Display the main application launcher UI."""
    app_list = get_app_list()
    examples_list = get_all_examples_list()
    tools_list = get_tools_list()
    
    # Create button grids
    main_apps_grid = create_button_grid(app_list[:2], buttons_per_row=2)
    python_examples_grid = create_button_grid(app_list[2:], buttons_per_row=3, show_source=True)
    tools_grid = create_button_grid(tools_list, buttons_per_row=3)
    examples_grid = create_button_grid(
        sorted(examples_list, key=lambda x: x['name']), 
        buttons_per_row=3, 
        show_source=True
    )
    # Control UI with stop and cache buttons
    control_ui = widgets.HBox([
        widgets.HTML(
            value='<button data-commandlinker-command="notebook:interrupt-kernel" '
                  'style="min-width:120px;" class="jupyter-button mod-danger">Stop</button>'
        ),
        make_button({"command": CACHE_EXTRACT_CMD, "name": "Extract Cache"}, button_style="info"),
        make_button({"command": CACHE_ARCHIVE_CMD, "name": "Archive Cache"}, button_style="info")
    ])

    # Process output area
    output_label = widgets.Label(value="Outputs:")
    
    # Initial UI with just output area
    ui = widgets.VBox([
        output_label,
        output_area
    ])
    
    display(ui)

    # Extract Isaac Sim cache on startup
    run_script(CACHE_EXTRACT_CMD, "Extract Cache")

    # Create example buttons grid (3 columns)
    example_btns = [make_button(example, show_source=True) for example in sorted(examples_list, key=lambda x: x['name'])]
    
    n_cols = 3
    rows = []
    for i in range(0, len(example_btns), n_cols):
        row = widgets.HBox(example_btns[i:i+n_cols])
        rows.append(row)
    
    examples_list_ui = widgets.VBox(rows)

    # Signal that launcher is ready
    run_script("echo 'Launcher Ready!'", "Nothing")

    # Update UI with all components
    ui.children = [
        run_in_bg,
        widgets.Label(value=f"Isaac Sim {os.environ['ISAACSIM_VERSION']}:"),
        main_apps_grid,
        widgets.Label(value="Isaac Sim Python Examples:"),
        python_examples_grid,
        widgets.Label(value="Other Tools:"),
        tools_grid,
        output_label,
        control_ui,
        output_area,
        widgets.Label(value="Full Isaac Sim Examples:"),
        examples_grid
    ]


# =============================================================================
# DESKTOP DISPLAY
# =============================================================================

def display_desktop(anchor="split-right"):
    """
    Display the remote desktop in a JupyterLab Sidecar tab.
    
    Args:
        anchor (str): Where the Sidecar tab will be placed. Options:
                    'split-right', 'split-left', 'split-top', 'split-bottom',
                    'tab-before', 'tab-after'
    """
    try:
        jupyterhub_user = os.environ["JUPYTERHUB_USER"]
        domain_name = os.environ["BINDER_LAUNCH_HOST"]
        domain_name = domain_name.replace("binder", "jupyter")
    except KeyError:
        jupyterhub_user = ""
        domain_name = ""

    url_prefix = f"{domain_name}/user/{jupyterhub_user}"
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

    display(widgets.HTML(
        value=f'<a href="{remote_desktop_url}"  class="jupyter-button" style="color: #fff;background-color: #1976d2;" target="_blank">Open Desktop in new Tab</a>',
    ))


# =============================================================================
# ROS2 UTILITIES
# =============================================================================

def publish_one_ros_message(topic, message, msg_type):
    """
    Publish a single ROS2 message to a topic.
    
    Args:
        topic (str): Topic name to publish to
        message: ROS2 message object
        msg_type: ROS2 message type
    """
    if not rclpy.ok():
        rclpy.init(args=None)
        
    node = rclpy.create_node("simple_publisher")
    publisher = node.create_publisher(msg_type, topic, 10)
    publisher.publish(message)
    node.destroy_node()


def get_topic_type(topic_name):
    """
    Get the message type of a ROS2 topic.
    
    Args:
        topic_name (str): Name of the topic
        
    Returns:
        str or None: Message type string or None if topic not found
    """
    node = rclpy.create_node('type_finder')
    
    for name, types in node.get_topic_names_and_types():
        if name == topic_name:
            node.destroy_node()
            return types[0]
            
    node.destroy_node()
    return None


def get_message_class(type_str):
    """
    Get the Python class for a ROS2 message type.
    
    Args:
        type_str (str): ROS2 message type string (e.g., 'std_msgs/msg/String')
        
    Returns:
        class: Python class for the message type
    """
    pkg, _, msg_name = type_str.partition('/msg/')
    return getattr(importlib.import_module(f"{pkg}.msg"), msg_name)


def get_one_ros_message(topic_name, timeout=5.0):
    """
    Fetch a single message from a ROS2 topic.
    
    Args:
        topic_name (str): Name of the topic to subscribe to
        timeout (float): Maximum time to wait for a message in seconds
        
    Returns:
        object or None: Received message or None if timeout/no message
    """
    if not rclpy.ok():
        rclpy.init(args=None)
        
    node = rclpy.create_node('one_shot_subscriber')

    # Get topic message type
    type_str = get_topic_type(topic_name)
    if not type_str:
        node.get_logger().error(f"Topic {topic_name} not found.")
        node.destroy_node()
        rclpy.shutdown()
        return None

    msg_type = get_message_class(type_str)
    node.get_logger().info(f"Detected message type: {type_str}")

    # Container to store received message
    msg_container = {'msg': None}

    def callback(msg):
        """Callback function to store received message."""
        msg_container['msg'] = msg
        rclpy.shutdown()

    # Create subscription
    node.create_subscription(msg_type, topic_name, callback, 10)

    # Wait for message with timeout
    import time
    start_time = time.time()
    
    while rclpy.ok() and msg_container['msg'] is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - start_time > timeout:
            node.get_logger().warn("Timeout waiting for message.")
            break

    node.destroy_node()
    return msg_container['msg']