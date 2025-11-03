# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from dataclasses import MISSING

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, DeformableObjectCfg, RigidObjectCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import FrameTransformerCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg, UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab.sensors import TiledCameraCfg,CameraCfg

from . import mdp

##
# Scene definition
##


@configclass
class ObjectTableSceneCfg(InteractiveSceneCfg):
    """Configuration for the lift scene with a robot and a object.
    This is the abstract base implementation, the exact scene is defined in the derived classes
    which need to set the target object, robot and end-effector frames
    """

    # robots: will be populated by agent env cfg
    robot: ArticulationCfg = MISSING
    # end-effector sensor: will be populated by agent env cfg
    ee_frame: FrameTransformerCfg = MISSING
    finger_frame_1: FrameTransformerCfg = MISSING
    finger_frame_2: FrameTransformerCfg = MISSING
    ee_tip_probe_frame:FrameTransformerCfg = MISSING
    # target object: will be populated by agent env cfg
    object: RigidObjectCfg | DeformableObjectCfg = MISSING
    # object_id :int=0

    # # Table
    # table = AssetBaseCfg(
    #     prim_path="{ENV_REGEX_NS}/Table",
    #     init_state=AssetBaseCfg.InitialStateCfg(pos=[0.5, 0, 0], rot=[0.707, 0, 0, 0.707]),
    #     spawn=UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd"),
    # )

    # plane
    plane = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0, 0, -1.05]),
        spawn=GroundPlaneCfg(),
    )

    # FloorWithPanels
    FloorWithPanels = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/FloorwithPanels",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=[0.0, 0.0, 0.0],
            rot=[0, 0, 0, 1],
        ),
        spawn=UsdFileCfg(usd_path="/home/roborock/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation/lift/robot_model/arm_description/urdf/R50/assets/FloorWithPanels.usd"),
    )

    # lights
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=5000.0),
    )
    sphere_light_0 = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/SphereLight_0",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=[2.5, 0.0, 4.5],
            rot=[0, 0, 0, 1],
        ),
        spawn=sim_utils.SphereLightCfg(
            color=(1.0, 1.0, 1.0),
            intensity=30000.0,
            radius=0.4,
            enable_color_temperature=True,
            color_temperature=5000.0
        )
    )

    sphere_light_1 = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/SphereLight_1",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=[5.0, 0.0, 4.5],
            rot=[0, 0, 0, 1],
        ),
        spawn=sim_utils.SphereLightCfg(
            color=(1.0, 1.0, 1.0),
            intensity=30000.0,
            radius=0.4,
            enable_color_temperature=True,
            color_temperature=5000.0
        )
    )
    tiled_camera: TiledCameraCfg = TiledCameraCfg(
        prim_path="{ENV_REGEX_NS}/Camera_1",
        # offset=TiledCameraCfg.OffsetCfg(pos=(0.16729, -0.02805, 0.055), rot=(( 0.49102 ,0.49805, -0.50194,   -0.50194)), convention="opengl"),
        offset=TiledCameraCfg.OffsetCfg(pos=(0.162, -0.02805, 0.0575158), rot=(( -0.48897357,-0.51078846,0.51078846,0.48897357)), convention="opengl"),

        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=17.5, focus_distance=400.0, horizontal_aperture=36,vertical_aperture=25.45
        ),
        # spawn=sim_utils.PinholeCameraCfg(
        #     focal_length=1.8, focus_distance=400.0, horizontal_aperture=20.955, clipping_range=(0.1, 20.0)
        # ),
        width=800,
        height=600,
    )

    # tiled_camera2: TiledCameraCfg = TiledCameraCfg(
    #     prim_path="{ENV_REGEX_NS}/Camera_2",
    #     offset=TiledCameraCfg.OffsetCfg(pos=(1.3, 0.0, 0.9), rot=((0.63281, 0.31551, 0.31551, 0.63281)), convention="opengl"),
    #     data_types=["rgb"],
    #     spawn=sim_utils.PinholeCameraCfg(
    #         focal_length=38.3, focus_distance=400.0, horizontal_aperture=20.955, clipping_range=(0.1, 20.0)
    #     ),
    #     width=1000,
    #     height=800,
    # )



##
# MDP settings
##


@configclass
class CommandsCfg:
    """Command terms for the MDP."""

    object_pose = mdp.UniformPoseCommandCfg(
        asset_name="robot",
        body_name=MISSING,  # will be set by agent env cfg
        resampling_time_range=(5.0, 5.0),
        debug_vis=False,
        ranges=mdp.UniformPoseCommandCfg.Ranges(
            # pos_x=(0.4, 0.6), pos_y=(-0.25, 0.25), pos_z=(0.25, 0.5), roll=(0.0, 0.0), pitch=(0.0, 0.0), yaw=(0.0, 0.0)
            # pos_x=(0.25, 0.35),
            # pos_y=(-0.05, 0.05),
            # pos_z=(0.25, 0.5),
            # roll=(0.0, 0.0),
            # pitch=(0.0, 0.0),
            # yaw=(0.0, 0.0),
            
            pos_x=(0.3, 0.3),
            pos_y=(-0.01, 0.01),
            pos_z=(0.1, 0.3),
            roll=(0.0, 0.0),
            pitch=(0.0, 0.0),
            yaw=(0.0, 0.0),
        ),
    )


@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    # will be set by agent env cfg
    arm_action: mdp.JointPositionActionCfg | mdp.DifferentialInverseKinematicsActionCfg = MISSING
    gripper_action: mdp.BinaryJointPositionActionCfg = MISSING


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        object_position = ObsTerm(func=mdp.object_position_in_robot_root_frame)
        target_object_position = ObsTerm(func=mdp.generated_commands, params={"command_name": "object_pose"})
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True
    '''
    @configclass
    class RGBCameraPolicyCfg(ObsGroup):
        """Observations for policy group with RGB images."""

        table_cam = ObsTerm(
            func=mdp.image, params={"sensor_cfg": SceneEntityCfg("table_cam"), "data_type": "rgb", "normalize": False}
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True
    '''



    # observation groups
    policy: PolicyCfg = PolicyCfg()
    #rgb_camera: RGBCameraPolicyCfg = RGBCameraPolicyCfg()


@configclass
class RGBObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class RGBCameraPolicyCfg(ObsGroup):
        """Observations for policy group with RGB images."""

        image = ObsTerm(func=mdp.image, params={"sensor_cfg": SceneEntityCfg("tiled_camera"), "data_type": "rgb"})

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: ObsGroup = RGBCameraPolicyCfg()


@configclass
class DepthObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class DepthCameraPolicyCfg(ObsGroup):
        """Observations for policy group with depth images."""

        image = ObsTerm(
            func=mdp.image, params={"sensor_cfg": SceneEntityCfg("table_cam"), "data_type": "distance_to_camera"}
        )

    policy: ObsGroup = DepthCameraPolicyCfg()


@configclass
class ResNet18ObservationCfg:
    """Observation specifications for the MDP."""

    @configclass
    class ResNet18FeaturesCameraPolicyCfg(ObsGroup):
        """Observations for policy group with features extracted from RGB images with a frozen ResNet18."""

        # joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        # joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        # object_position = ObsTerm(func=mdp.object_position_in_robot_root_frame)
        # target_object_position = ObsTerm(func=mdp.generated_commands, params={"command_name": "object_pose"})
        # actions = ObsTerm(func=mdp.last_action)
        image = ObsTerm(
            func=mdp.image,
            params={"sensor_cfg": SceneEntityCfg("tiled_camera"), "data_type": "rgb"},
        )

        # image = ObsTerm(
        #     func=mdp.image_features,
        #     params={"sensor_cfg": SceneEntityCfg("tiled_camera"), "data_type": "rgb","model_name": "resnet18"}
        # )

        # joint_pos = ObsTerm(func=mdp.joint_pos_rel)

        # def __post_init__(self):
        #     self.enable_corruption = False
        #     self.concatenate_terms = False

    policy: ObsGroup = ResNet18FeaturesCameraPolicyCfg()


@configclass
class TheiaTinyObservationCfg:
    """Observation specifications for the MDP."""

    @configclass
    class TheiaTinyFeaturesCameraPolicyCfg(ObsGroup):
        """Observations for policy group with features extracted from RGB images with a frozen Theia-Tiny Transformer"""

        image = ObsTerm(
            func=mdp.image_features,
            params={
                "sensor_cfg": SceneEntityCfg("table_cam"),
                "data_type": "rgb",
                "model_name": "theia-tiny-patch16-224-cddsv",
                "model_device": "cuda:0",
            },
        )

    policy: ObsGroup = TheiaTinyFeaturesCameraPolicyCfg()


@configclass
class EventCfg:
    """Configuration for events."""

    reset_all = EventTerm(func=mdp.reset_scene_to_default, mode="reset")

    reset_object_position = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            # "pose_range": {"x": (-0.1, 0.1), "y": (-0.25, 0.25), "z": (0.0, 0.0)},
            # "pose_range": {"x": (-0.05, 0.05), "y": (-0.25, 0.25), "z": (0.0, 0.0)},

            "pose_range": {
                "x": (0.00, 0.07),
                "y": (0.00, 0.01),
                "z": (0.0, 0.0),
                # "yaw": (-0.5, 0.5),
            },

            
            "velocity_range": {},
        },
    )
    randomize_lighting_reset = EventTerm(
        func=mdp.randomize_multiple_sphere_lights,
        mode="reset",
        params={"num_lights": 2},
    )
    
    randomize_floor = EventTerm(
        func=mdp.randomize_floor_texture,
        mode="reset",
        params={
            "texture_txt_path" :"/home/roborock/桌面/floor.txt"
        },
    )
    randomize_wall = EventTerm(
        func=mdp.randomize_wall_texture,
        mode="reset",
        params={
            "texture_txt_path" :"/home/roborock/桌面/floor.txt"
        },
    )
@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # reaching_object = RewTerm(func=mdp.object_ee_distance, params={"std": 0.1}, weight=1.0)
    reaching_object = RewTerm(
        func=mdp.object_ee_distance,
        params={"std": 0.1},
        weight=40,  # 2.0
        # weight=20.0,
    )
    lifting_object_linear = RewTerm(
        func=mdp.object_is_lifted_linear,
        params={"minimal_height": 0.025, "max_height": 0.045},
        weight=1000.0,   # 1500  150
    )
    # termination_penalty = RewTerm(func=mdp.is_terminated, weight=-100.0)
    # lifting_object = RewTerm(
    #     func=mdp.object_is_lifted,
    #     params={"minimal_height": 0.02},
    #     weight=50.0,   # 1500  150
    # )

    # lifting_object1 = RewTerm(
    #     func=mdp.object_is_lifted,
    #     params={"minimal_height": 0.03},
    #     weight=100.0,   # 1500  150
    # )

    # lifting_object2 = RewTerm(
    #     func=mdp.object_is_lifted,
    #     params={"minimal_height": 0.04},
    #     weight=200.0,   # 1500  150
    # )

    # lifting_object3 = RewTerm(
    #     func=mdp.object_is_lifted,
    #     params={"minimal_height": 0.05},
    #     weight=500.0,   # 1500  150
    # )


    object_goal_tracking = RewTerm(
        func=mdp.object_goal_distance,
        params={"std": 0.3, "minimal_height": 0.028, "command_name": "object_pose"},
        weight=1000, # 16.0
    )

    object_goal_tracking_fine_grained = RewTerm(
        func=mdp.object_goal_distance,
        #params={"std": 0.05, "minimal_height": 0.04, "command_name": "object_pose"},
        params={"std": 0.05, "minimal_height": 0.028, "command_name": "object_pose"},
        weight=0.5,  # 5.0
    )

    # action penalty
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-1e-4)

    # joint_vel = RewTerm(
    #     func=mdp.joint_vel_l2,
    #     weight=-1e-4,
    #     params={"asset_cfg": SceneEntityCfg("robot")},
    # )

    # grip_object = RewTerm(
    #     func=mdp.grip_object,
    #     weight=10,  # 10.0
    # )
    contain_object = RewTerm(
        func=mdp.contain_object,
        params={"std": 1},
        weight=10,  # 2.0
    )

    clamp_object = RewTerm(
        func=mdp.clamp_object,
        params={"std": 1},
        weight=30,  # 2.0
    )
    # angle_before_grip = RewTerm(
    #     func=mdp.angle_before_grip,
    #     params={"std": 1},
    #     weight=10,  # 2.0
    # )
    gripper_tip_below_ground = RewTerm(
        func=mdp.gripper_tip_below_ground,
        params={"threshold": 0.0006, "ee_probe_cfg": SceneEntityCfg("ee_tip_probe_frame")},
        weight=-200
    )


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)

    # object_dropping = DoneTerm(
    #     func=mdp.object_dropped_after_lifted,
        
    # )
    # gripper_tip_below_ground = DoneTerm(
    #     func=mdp.gripper_tip_below_ground,
    #     params={"threshold": 0.008, "ee_probe_cfg": SceneEntityCfg("ee_tip_probe_frame")},
    # )


@configclass
class CurriculumCfg:
    """Curriculum terms for the MDP."""

    #action_rate = CurrTerm(
    #    func=mdp.modify_reward_weight, params={"term_name": "action_rate", "weight": -1e-1, "num_steps": 10000}
    #)

    #joint_vel = CurrTerm(
    #    func=mdp.modify_reward_weight, params={"term_name": "joint_vel", "weight": -1e-1, "num_steps": 10000}
    #)


##
# Environment configuration
##


@configclass
class LiftEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the lifting environment."""

    # Scene settings
    scene: ObjectTableSceneCfg = ObjectTableSceneCfg(num_envs=32, env_spacing=2.5)
    # Basic settings
    # observations: ObservationsCfg = ObservationsCfg()
    #observations: TheiaTinyObservationCfg = TheiaTinyObservationCfg()
    observations: ResNet18ObservationCfg = ResNet18ObservationCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 10 # 2 20 48
        self.episode_length_s = 0.6
        # simulation settings
        self.sim.dt = 0.01 # 100Hz
        self.sim.render_interval =10

        self.sim.physx.bounce_threshold_velocity = 0.2
        self.sim.physx.bounce_threshold_velocity = 0.01
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024 * 4
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024
        self.sim.physx.friction_correlation_distance = 0.00625
