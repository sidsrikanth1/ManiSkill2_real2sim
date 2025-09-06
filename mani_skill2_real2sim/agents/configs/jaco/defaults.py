from copy import deepcopy

import numpy as np

from mani_skill2_real2sim.agents.controllers import *
from mani_skill2_real2sim.sensors.camera import CameraConfig
from mani_skill2_real2sim.utils.sapien_utils import look_at


class JacoDefaultConfig:
    def __init__(self) -> None:
        self.urdf_path = "{PACKAGE_ASSET_DIR}/descriptions/ada_description/robots_urdf/ada.urdf"

        # finger_min_patch_radius = 0.01  # used to calculate torsional friction
        # self.urdf_config = dict(
        #     _materials=dict(
        #         gripper=dict(static_friction=2.0, dynamic_friction=2.0, restitution=0.0)
        #     ),
        #     link=dict(
        #         left_finger_link=dict(
        #             material="gripper",
        #             patch_radius=finger_min_patch_radius,
        #             min_patch_radius=finger_min_patch_radius,
        #         ),
        #         right_finger_link=dict(
        #             material="gripper",
        #             patch_radius=finger_min_patch_radius,
        #             min_patch_radius=finger_min_patch_radius,
        #         ),
        #     ),
        # )
        self.urdf_config = dict()

        self.arm_joint_names = [
            "j2n6s200_joint_1",
            "j2n6s200_joint_2",
            "j2n6s200_joint_3",
            "j2n6s200_joint_4",
            "j2n6s200_joint_5",
            "j2n6s200_joint_6",
        ]
        self.gripper_joint_names = [
            "j2n6s200_joint_finger_1",
            "j2n6s200_joint_finger_2",
        ]

        # self.gripper_force_limit = [20] * len(self.gripper_joint_names)
        # self.gripper_vel_limit = [1.0] * len(self.gripper_joint_names)

        # # Force control
        # self.arm_stiffness = [1e9] * len(self.arm_joint_names)
        # self.arm_damping = [1e3] * len(self.arm_joint_names)

        # self.gripper_stiffness = [1e9] * len(self.gripper_joint_names)
        # self.gripper_damping = [1e3] * len(self.gripper_joint_names)

        # self.arm_stiffness = [
        #     1169.7891719504198,
        #     730.0,
        #     808.4601346394447,
        #     1229.1299089624076,
        #     1272.2760456418862,
        #     1056.3326605132252,
        # ]
        # self.arm_damping = [
        #     330.0,
        #     180.0,
        #     152.12036565582588,
        #     309.6215302722146,
        #     201.04998711007383,
        #     269.51458932695414,
        # ]

        self.arm_stiffness = 1000
        self.arm_damping = 100

        self.arm_force_limit = [2400, 4800, 2400, 1200, 1200, 1200]
        self.arm_friction = 0.0
        self.arm_vel_limit = 1.0

        self.gripper_stiffness = 1000
        self.gripper_damping = 100
        # self.gripper_stiffness = 1000
        # self.gripper_damping = 200
        # self.gripper_pid_stiffness = 1000
        # self.gripper_pid_damping = 200
        # self.gripper_pid_integral = 300
        self.gripper_force_limit = 120
        self.gripper_vel_limit = 1.0

        self.ee_link_name = "j2n6s200_end_effector"

    @property
    def controllers(self):
        _C = {}

        # -------------------------------------------------------------------------- #
        # Arm
        # -------------------------------------------------------------------------- #
        arm_common_args = [
            self.arm_joint_names,
            -1.0,  # dummy limit, which is unused since normalize_action=False
            1.0,
            np.pi / 2,
            self.arm_stiffness,
            self.arm_damping,
            # self.arm_force_limit,
        ]
        arm_common_kwargs = dict(
            friction=self.arm_friction,
            ee_link=self.ee_link_name,
            normalize_action=False,
        )
        arm_pd_ee_delta_pose = PDEEPoseControllerConfig(
            *arm_common_args, frame="ee", **arm_common_kwargs
        )
        arm_pd_ee_target_delta_pose = PDEEPoseControllerConfig(
            *arm_common_args, frame="ee", use_target=True, **arm_common_kwargs
        )
        _C["arm"] = dict(
            arm_pd_ee_delta_pose=arm_pd_ee_delta_pose,
            arm_pd_ee_target_delta_pose=arm_pd_ee_target_delta_pose,
        )

        # -------------------------------------------------------------------------- #
        # Gripper
        # -------------------------------------------------------------------------- #
        gripper_common_args = [
            self.gripper_joint_names,
            0.1,
            1.6,
            self.gripper_stiffness,
            self.gripper_damping,
            # self.gripper_force_limit,
        ]
        gripper_common_kwargs = dict(
            normalize_action=True,
            drive_mode="force",
        )

        gripper_pd_joint_pos = PDJointPosMimicControllerConfig(
            *gripper_common_args, **gripper_common_kwargs
        )
        gripper_pd_joint_target_pos = PDJointPosMimicControllerConfig(
            *gripper_common_args,
            use_target=True,
            clip_target=True,
            **gripper_common_kwargs,
        )
        gripper_pd_joint_delta_pos = PDJointPosMimicControllerConfig(
            *gripper_common_args, use_delta=True, **gripper_common_kwargs
        )
        gripper_pd_joint_target_delta_pos = PDJointPosMimicControllerConfig(
            *gripper_common_args,
            use_delta=True,
            use_target=True,
            clip_target=True,
            **gripper_common_kwargs,
        )
        _C["gripper"] = dict(
            gripper_pd_joint_pos=gripper_pd_joint_pos,
            gripper_pd_joint_target_pos=gripper_pd_joint_target_pos,
            gripper_pd_joint_delta_pos=gripper_pd_joint_delta_pos,
            gripper_pd_joint_target_delta_pos=gripper_pd_joint_target_delta_pos,
        )

        controller_configs = {}
        for arm_controller_name in _C["arm"]:
            for gripper_controller_name in _C["gripper"]:
                c = {}
                c["arm"] = _C["arm"][arm_controller_name]
                c["gripper"] = _C["gripper"][gripper_controller_name]
                combined_name = (
                    arm_controller_name + "_" + gripper_controller_name
                )
                controller_configs[combined_name] = c

        # Make a deepcopy in case users modify any config
        return deepcopy_dict(controller_configs)

    @property
    def cameras(self):
        # Table width: about 36cm

        # return [
        #     CameraConfig(
        #         uid="3rd_view_camera",  # the camera used for real evaluation
        #         p=[0.0, -0.16, 0.36],
        #         # this rotation allows simulation proxy table to align almost perfectly with real table for bridge_real_eval_1.png
        #         # when calling env.reset(options={'robot_init_options': {'init_xy': [0.147, 0.028], 'init_rot_quat': [0, 0, 0, 1]}})
        #         q=look_at([0, 0, 0], [1, 0.553, -1.085]).q,
        #         width=640,
        #         height=480,
        #         actor_uid="base_link",
        #         intrinsic=np.array(
        #             [[623.588, 0, 319.501], [0, 623.588, 239.545], [0, 0, 1]]
        #         ),  # logitech C920
        #     ),
        # ]

        return CameraConfig(
            uid="overhead_camera",
            p=[-0.50461, -0.413019, 1.23155],
            q=[
                0.934841,
                -0.0167267,
                0.351877,
                0.0444384,
            ],  # SAPIEN uses ros camera convention; the rotation matrix of link_camera's pose is in opencv convention, so we need to transform it to ros convention
            width=640,
            height=480,
            actor_uid="j2n6s200_link_base",
            intrinsic=np.array(
                [[425.0, 0, 320.0], [0, 425.0, 256.0], [0, 0, 1]]
            ),
        )
