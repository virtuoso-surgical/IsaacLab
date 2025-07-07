# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.controllers.differential_ik_cfg import DifferentialIKControllerCfg
from isaaclab.envs.mdp.actions.actions_cfg import DifferentialInverseKinematicsActionCfg
from isaaclab.utils import configclass

from . import joint_pos_env_cfg

##
# Pre-defined configs
##
from isaaclab_assets.robots.virtuoso import VIRTUOSO_CFG  # isort: skip


@configclass
class VirtuosoReachEnvCfg(joint_pos_env_cfg.VirtuosoReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Set Virtuoso as robot
        # We switch here to a stiffer PD controller for IK tracking to be better.
        self.scene.robot = VIRTUOSO_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # Set actions for the specific robot type (Virtuoso)
        self.actions.arm_action = DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=[
                "left_clearance_angle_rotation_joint",
                "left_clearance_angle_translation_joint", 
                "left_outer_tube_translation_joint",
                "left_outer_tube_rotation_joint",
                "left_inner_tube_translation_joint",
                "left_inner_tube_rotation_joint"
            ],
            body_name="left_inner_tube_pose",
            controller=DifferentialIKControllerCfg(command_type="pose", use_relative_mode=False, ik_method="dls"),
            # body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(pos=[0.0, 0.0, 0.107]),
        )
