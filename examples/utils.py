"""
Utility functions for Isaac Sim application launcher and ROS2 integration.
Provides UI components, process management, and ROS2 utilities.
"""

import os
import subprocess

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

# Bash script to setup Isaac Sim Python environment
ISAACSIM_ENV = """
# unset virtualGL ENV (which breaks ROS2 bridge)
unset LD_PRELOAD
# Clear default ROS ENV
unset PYTHONPATH
export LD_LIBRARY_PATH=$ISAACSIM_PATH/exts/isaacsim.ros2.bridge/$ROS_DISTRO/lib
"""

# =============================================================================
# PROCESS MANAGEMENT
# =============================================================================

def run_script(script, background=True):
    """
    Run a bash script in a subprocess.
    
    Args:
        script (str): Bash script to execute
        label (str): Label for the process (for logging)
    """

    # Determine command based on run mode
    if background:
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
    
    current_process.wait()
    return current_process

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
        jupyterhub_user = None
        domain_name = "http://localhost:8888"
    url_prefix = f"{domain_name}/user/{jupyterhub_user}" if jupyterhub_user is not None else ''

    remote_desktop_url = f"{url_prefix}/desktop"

    display(widgets.HTML(
        value=f'<a href="{remote_desktop_url}"  class="jupyter-button" style="color: #fff;background-color: #1976d2;" target="_blank">Open Desktop in new Tab</a>',
    ))
    
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

