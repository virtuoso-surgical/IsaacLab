# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the Virtuoso Endoscopic robot."""

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab_assets import ISAACLAB_ASSETS_DATA_DIR

##
# Configuration
##

VIRTUOSO_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAACLAB_ASSETS_DATA_DIR}/Robots/virtuoso/tube_assembly_with_clearance_angle_v0.2.2.usd",
        # usd_path=f"{ISAACLAB_ASSETS_DATA_DIR}/Robots/virtuoso/robot_left_side.usd",
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=True,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=8, solver_velocity_iteration_count=0
        ),
        # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "left_clearance_angle_rotation_joint": 0.028,
            "left_clearance_angle_translation_joint": 0.0,
            "left_outer_tube_translation_joint": 0.0,
            "left_outer_tube_rotation_joint": 0.0,
            "left_inner_tube_translation_joint": 0.0,
            "left_inner_tube_rotation_joint": 0.0,
            "right_clearance_angle_rotation_joint": 0.028,
            "right_clearance_angle_translation_joint": 0.0,
            "right_outer_tube_translation_joint": 0.0,
            "right_outer_tube_rotation_joint": 0.0,
            "right_inner_tube_translation_joint": 0.0,
            "right_inner_tube_rotation_joint": 0.0,
        },
    ),
    actuators={
        "tube_extension": ImplicitActuatorCfg(
            joint_names_expr=[".*_translation_joint"],
            effort_limit_sim=100000.0,
            velocity_limit_sim=10.0,
            stiffness=100000.0,
            damping=2.0,
        ),
        "tube_rotation": ImplicitActuatorCfg(
            joint_names_expr=[".*_rotation_joint"],
            effort_limit_sim=10000.0,
            velocity_limit_sim=10.0,
            stiffness=100000.0,
            damping=2.0,
        ),
    },
    soft_joint_pos_limit_factor=0.000001,
)

# FRANKA_PANDA_HIGH_PD_CFG = FRANKA_PANDA_CFG.copy()
# FRANKA_PANDA_HIGH_PD_CFG.spawn.rigid_props.disable_gravity = True
# FRANKA_PANDA_HIGH_PD_CFG.actuators["panda_shoulder"].stiffness = 400.0
# FRANKA_PANDA_HIGH_PD_CFG.actuators["panda_shoulder"].damping = 80.0
# FRANKA_PANDA_HIGH_PD_CFG.actuators["panda_forearm"].stiffness = 400.0
# FRANKA_PANDA_HIGH_PD_CFG.actuators["panda_forearm"].damping = 80.0
