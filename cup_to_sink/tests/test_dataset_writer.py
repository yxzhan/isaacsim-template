# cup_to_sink/tests/test_dataset_writer.py
import numpy as np, h5py, tempfile, os
from cup_to_sink.dataset_writer import Recorder
def _step(i):
    img={"front":{"rgb":np.zeros((4,4,3),np.uint8),"depth":np.full((4,4),1000,np.uint16)}}
    return dict(joint_pos=np.zeros(7),joint_vel=np.zeros(7),gripper_joint_pos=np.array([0.04,0.04]),
        gripper_width=0.08,gripper_aperture=1.0,ee_pose_7d_world=np.array([0.1,0.2,0.3,1.,0,0,0]),
        ee_pose_6d_world=np.zeros(6),ee_pose_7d_base=np.array([0.,0,0,1,0,0,0]),ee_pose_6d_base=np.zeros(6),
        cup_pose=np.array([1.,0,0,1,0,0,0]),sink_target_pose=np.array([1.,1,0,1,0,0,0]),images=img,
        action=dict(joint_pos_target=np.zeros(7),ee_pose_target_7d=np.array([0.,0,0,1,0,0,0]),
            ee_pose_target_6d=np.zeros(6),delta_ee_pose_target_6d=np.zeros(6),
            gripper_joint_pos_target=np.array([0.04,0.04]),gripper_width_target=0.08,gripper_aperture_target=1.0,
            policy_action_command=np.zeros(8),policy_action_ee_command=np.zeros(7)),
        debug=dict(sparse_ee_target_pose=np.array([0.,0,0,1,0,0,0]),grasp_pose_7d=np.array([0.,0,0,1,0,0,0]),
            pregrasp_pose_7d=np.array([0.,0,0,1,0,0,0])),phase=i)
def test_write_roundtrip():
    r=Recorder()
    for i in range(3): r.append(_step(i))
    p=os.path.join(tempfile.mkdtemp(),"ep.hdf5")
    r.write(p, meta=dict(task_name="cup_to_sink",language_instruction="put the cup into the sink",seed=5,
        success=True,enabled_cameras=["front"],joint_names=["j%d"%k for k in range(7)],
        randomization=dict(seed=5,cup_initial_pose=np.zeros(7),sink_target_pose=np.zeros(7))))
    with h5py.File(p,"r") as f:
        assert f["/observations/joint_pos"].shape==(3,7)
        assert f["/observations/images/front/rgb"].shape==(3,4,4,3)
        assert f["/observations/images/front/depth"].dtype==np.uint16
        assert f["/actions/policy_action_command"].shape==(3,8)
        assert f["/actions/policy_action_next_state"].shape==(3,8)   # derived from next joint+gripper
        assert f["/meta/seed"][()]==5
        assert f["/meta/randomization/seed"][()]==5
        assert f.attrs.get("schema_version") is not None or "/meta/task_name" in f
        np.testing.assert_allclose(f["/observations/ee_pose_7d_world"][0][:3], [0.1,0.2,0.3])
        np.testing.assert_allclose(f["/observations/ee_pose_7d"][0][:3], [0.,0,0])
        assert f["/actions/policy_action_ee_next_state"].shape == (3,7)
