# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.utils import configclass

import isaaclab_tasks.manager_based.virtuoso.reach.mdp as mdp
from isaaclab_tasks.manager_based.virtuoso.reach.reach_env_cfg import ReachEnvCfg

##
# Pre-defined configs
##
from isaaclab_assets.robots.virtuoso import VIRTUOSO_CFG  # isort: skip


##
# Environment configuration
##


@configclass
class VirtuosoReachEnvCfg(ReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # switch robot to Virtuoso
        self.scene.robot = VIRTUOSO_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # override actions
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=[
                "left_clearance_angle_rotation_joint",
                "left_clearance_angle_translation_joint", 
                "left_outer_tube_translation_joint",
                "left_outer_tube_rotation_joint",
                "left_inner_tube_translation_joint",
                "left_inner_tube_rotation_joint"
            ],
            scale=0.5,
            use_default_offset=True
        )
