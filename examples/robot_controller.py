import os
import re
from urdf_parser_py.urdf import URDF
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import ipywidgets as widgets
from IPython.display import display, Markdown, clear_output
import time

import logging

logging.getLogger().setLevel(logging.ERROR)

if not rclpy.ok():
    rclpy.init()

def joint_controller(urdf_path, prefix=""):
    with open(urdf_path, 'r') as f:
        urdf_content = f.read()
    clean_urdf = re.sub(r'<transmission.*?</transmission>', '', urdf_content, flags=re.DOTALL)
    robot = URDF.from_xml_string(clean_urdf)
    
    node = NotebookJointUI(prefix=prefix)
    
    print("Waiting for Robot Joint state...")
    while node.latest_state is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    sliders = {}
    joint_msg = node.latest_state
    
    def make_cb(joint_name):
        def cb(change):
            node.publish_joint(joint_name, change['new'])
        return cb

    for joint in robot.joints:
        if joint.type == 'revolute' or joint.type == 'prismatic':
            name = joint.name
            pos = joint_msg.position[joint_msg.name.index(name)]
            lower = joint.limit.lower
            upper = joint.limit.upper
        
            s = widgets.FloatSlider(
                value=pos,
                min=lower,
                max=upper,
                step=0.01,
                description=name,
                continuous_update=True,
                style=dict(
                    description_width='10rem'
                ),
                layout=widgets.Layout(width='100%')
            )
            s.observe(make_cb(name), names='value')
            sliders[name] = s

    return widgets.VBox([sliders[i] for i in sliders])
            

class NotebookJointUI(Node):
    def __init__(self, prefix=""):
        super().__init__(f'{prefix}_notebook_joint_ui')

        self.latest_state = None

        self.sub = self.create_subscription(
            JointState,
            f'/{prefix}/joint_states',
            self._joint_state_cb,
            10
        )

        self.pub = self.create_publisher(
            JointState,
            f'/{prefix}/joint_position_cmd',
            10
        )

    def _joint_state_cb(self, msg):
        self.latest_state = msg

    def publish_joint(self, name, pos):
        msg = JointState()
        msg.name = [name]
        msg.position = [float(pos)]
        self.pub.publish(msg)
