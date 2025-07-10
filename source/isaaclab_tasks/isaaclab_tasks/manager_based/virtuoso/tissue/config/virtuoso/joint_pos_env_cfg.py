# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.assets import AssetBaseCfg, DeformableObjectCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.utils import configclass

import isaaclab_tasks.manager_based.virtuoso.tissue.mdp as mdp
from isaaclab_tasks.manager_based.virtuoso.tissue.tissue_env_cfg import TissueEnvCfg
from isaaclab_assets import ISAACLAB_ASSETS_DATA_DIR

##
# Pre-defined configs
##
from isaaclab_assets.robots.virtuoso import VIRTUOSO_CFG  # isort: skip


##
# Environment configuration
##


@configclass
class VirtuosoTissueEnvCfg(TissueEnvCfg):
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
                "left_inner_tube_rotation_joint",
                "right_clearance_angle_rotation_joint",
                "right_clearance_angle_translation_joint",
                "right_outer_tube_translation_joint",
                "right_outer_tube_rotation_joint",
                "right_inner_tube_translation_joint",
                "right_inner_tube_rotation_joint",
            ],
            scale=1.0,
            use_default_offset=False,
            preserve_order=True,
        )

        # Set deformable skin tissue as object
        # self.scene.object = DeformableObjectCfg(
        #     prim_path="{ENV_REGEX_NS}/Object",
        #     init_state=DeformableObjectCfg.InitialStateCfg(pos=[0.094, 0.0, 0.0130], rot=[0, 0, 0, 1]),
        #     spawn=UsdFileCfg(
        #         usd_path=f"{ISAACLAB_ASSETS_DATA_DIR}/Props/Skin/skin.usd",
        #         scale=(0.08, 0.05, 0.0004),
        #     ),
        #     debug_vis=True,
        # )
        # self.scene.object.visualizer_cfg.markers["target"].radius = 0.002

        self.scene.object = AssetBaseCfg(
            prim_path="{ENV_REGEX_NS}/Object",
            init_state=AssetBaseCfg.InitialStateCfg(pos=[0.094, 0.0, 0.0060], rot=[0, 0, 0, 1]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAACLAB_ASSETS_DATA_DIR}/Props/Skin/skin-fixed.usd",
                scale=(0.08, 0.05, 0.0004),
                ),
        )
