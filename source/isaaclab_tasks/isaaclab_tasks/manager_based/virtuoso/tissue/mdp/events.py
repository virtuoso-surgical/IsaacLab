# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import DeformableObject
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv


def reset_nodal_kinematic_target(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
):
    asset: DeformableObject = env.scene[asset_cfg.name]
    # get default root state
    default_nodal_state = asset.data.nodal_pos_w[env_ids].clone()
    nodal_kinematic_target = asset.data.nodal_kinematic_target[env_ids].clone()

    indices = [25, 27] + list(range(528, 530))
    nodal_kinematic_target[..., :3] = default_nodal_state[..., :3]
    nodal_kinematic_target[..., indices, 3] = 0.0

    asset.write_nodal_kinematic_target_to_sim(nodal_kinematic_target, env_ids=env_ids)
