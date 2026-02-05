import os
import re
from urdf_parser_py.urdf import URDF
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import ipywidgets as widgets
from IPython.display import display, Markdown, clear_output
import time

import logging

logging.getLogger().setLevel(logging.ERROR)

if not rclpy.ok():
    rclpy.init()

def joint_controller(urdf_path, prefix=None):
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
                continuous_update=False,
                style=dict(
                    description_width='10rem'
                ),
                layout=widgets.Layout(width='100%')
            )
            s.observe(make_cb(name), names='value')
            sliders[name] = s

    return widgets.VBox([sliders[i] for i in sliders])

def robot_steering(prefix=""):
    cmd_vel = CmdVelPublisher(prefix=prefix)
    
    # robot_steering, similar to "rqt_robot_steering"
    linear_x = widgets.FloatSlider(
        value=0,
        min=-0.5,
        max=0.5,
        step=0.05,
        description='Moving',
        orientation='vertical',
        continuous_update=True,
        readout=True,
        readout_format='.1f',
    )
    
    def on_linear_x_change(v):
        cmd_vel.msg.linear.x = float(v)
        cmd_vel.publish()
    
    linear_x.observe(lambda v: on_linear_x_change(v['new']), names='value')
    
    # slider for rotation velocity
    angular_z = widgets.FloatSlider(
        value=0,
        min=-3,
        max=3,
        step=0.1,
        description='Rotation',
        continuous_update=True,
        readout=True,
        readout_format='.1f',
    )
    
    def on_angular_z_change(v):
        cmd_vel.msg.angular.z = -float(v)
        cmd_vel.publish()
    
    angular_z.observe(lambda v: on_angular_z_change(v['new']), names='value')


    def on_angular_z_change(v):
        cmd_vel.msg.angular.z = -float(v)
        cmd_vel.publish()

    btn_stop = widgets.Button(
        description='STOP'
    )
    
    def on_stop(_):
        linear_x.value = 0.0
        angular_z.value = 0.0
    
    btn_stop.on_click(on_stop)

    return widgets.HBox([linear_x, angular_z, btn_stop])
    


class CmdVelPublisher(Node):
    def __init__(self, prefix=None):
        if prefix is not None:
            node_name = f"{prefix}_"
            topic_prefix = f"/{prefix}"
        else:
            node_name = ''
            topic_prefix = ''

        super().__init__(f'{prefix}robot_steering')
        self.msg = Twist()
        self.pub = self.create_publisher(Twist, f'{topic_prefix}/cmd_vel', 10)

    def publish(self):
        self.pub.publish(self.msg)

class NotebookJointUI(Node):
    def __init__(self, prefix=None):
        if prefix is not None:
            node_name = f"{prefix}_"
            topic_prefix = f"/{prefix}"
        else:
            node_name = ''
            topic_prefix = ''
        
        super().__init__(f'{prefix}notebook_joint_ui')

        self.latest_state = None
        self.sub = self.create_subscription(
            JointState,
            f'{topic_prefix}/joint_states',
            self._joint_state_cb,
            10
        )

        self.pub = self.create_publisher(
            JointState,
            f'{topic_prefix}/joint_command',
            10
        )

    def _joint_state_cb(self, msg):
        self.latest_state = msg

    def publish_joint(self, name, pos):
        msg = JointState()
        msg.name = [name]
        msg.position = [float(pos)]
        self.pub.publish(msg)
