import os
import re
from urdf_parser_py.urdf import URDF
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float64
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
        min=-2,
        max=2,
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


def ee_controller(prefix="stretch"):
    """Cartesian end-effector control: publishes a base-relative target that the
    simulation solves with IK (calls ``move_ee`` on the sim side)."""
    node = EECommandPublisher(prefix=prefix)

    # The Stretch arm is a planar 2-DOF arm: reach (along base -Y) + height
    # (lift). grasp_center x stays ~ -0.02, so we keep it fixed.
    EE_X = -0.02
    GRIPPER_OPEN, GRIPPER_CLOSE = 0.55, 0.0

    reach = widgets.FloatSlider(
        value=-0.55, min=-0.88, max=-0.45, step=0.01,
        description='Reach (y)', continuous_update=False,
        style=dict(description_width='8rem'),
        layout=widgets.Layout(width='100%'),
    )
    height = widgets.FloatSlider(
        value=0.60, min=0.30, max=1.10, step=0.01,
        description='Height (z)', continuous_update=False,
        style=dict(description_width='8rem'),
        layout=widgets.Layout(width='100%'),
    )

    def send_ee(_=None):
        node.publish_ee(EE_X, reach.value, height.value)

    reach.observe(lambda c: send_ee(), names='value')
    height.observe(lambda c: send_ee(), names='value')

    btn_move = widgets.Button(description='Move EE', button_style='primary')
    btn_move.on_click(send_ee)
    btn_open = widgets.Button(description='Open Gripper')
    btn_open.on_click(lambda _: node.publish_gripper(GRIPPER_OPEN))
    btn_close = widgets.Button(description='Close Gripper')
    btn_close.on_click(lambda _: node.publish_gripper(GRIPPER_CLOSE))

    return widgets.VBox([
        reach,
        height,
        widgets.HBox([btn_move, btn_open, btn_close]),
    ])




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


class EECommandPublisher(Node):
    def __init__(self, prefix=None):
        topic_prefix = f"/{prefix}" if prefix else ""
        super().__init__(f'{prefix}ee_command_ui')

        self.pub_ee = self.create_publisher(Point, f'{topic_prefix}/ee_command', 10)
        self.pub_gripper = self.create_publisher(Float64, f'{topic_prefix}/gripper_command', 10)

    def publish_ee(self, x, y, z):
        msg = Point()
        msg.x, msg.y, msg.z = float(x), float(y), float(z)
        self.pub_ee.publish(msg)

    def publish_gripper(self, value):
        msg = Float64()
        msg.data = float(value)
        self.pub_gripper.publish(msg)
