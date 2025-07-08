###############################################################################
# Copyright (c) 2025, Virtuoso Surgical. All rights reserved.
###############################################################################

"""Launches an IsaacSim scene with a Virtuoso robot that can be controlled via ROS."""

from isaaclab.app import AppLauncher

# launch omniverse app
app_launcher = AppLauncher(num_envs=1, sensitivity=1)
simulation_app = app_launcher.app

import gymnasium as gym
import numpy as np
import torch

import omni.log

from isaaclab_tasks.utils import parse_env_cfg

import math

# ROS
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Define a ROS node
class JointStateListener(Node):
    def __init__(self):
        super().__init__('isaaclab_node_thing')
        self.subscription = self.create_subscription(
            JointState,
            '/ves/left/joint/setpoint_jp',
            self.left_joint_state_callback,
            10,
        )
        self.subscription = self.create_subscription(
            JointState,
            '/ves/right/joint/setpoint_jp',
            self.right_joint_state_callback,
            10,
        )

    def left_joint_state_callback(self, msg: JointState):
        self.get_logger().info('Received JointState for LEFT side:')
        self.get_logger().info(f'  Names: {msg.name}')
        self.get_logger().info(f'  Positions: {msg.position}')
        self.get_logger().info(f'  Velocities: {msg.velocity}')
        self.get_logger().info(f'  Efforts: {msg.effort}')
    
    def right_joint_state_callback(self, msg: JointState):
        self.get_logger().info('Received JointState for RIGHT side:')
        self.get_logger().info(f'  Names: {msg.name}')
        self.get_logger().info(f'  Positions: {msg.position}')
        self.get_logger().info(f'  Velocities: {msg.velocity}')
        self.get_logger().info(f'  Efforts: {msg.effort}')

# Define the Virtuoso joint state data structure
# TODO: Do the Virtuoso-to-IsaacLab calculations in this class
class VirtuosoJointState:
    def __init__(self, left_values = (0.0,)*5, right_values = (0.0,)*5):
        self.left_clearance_angle_rotation_joint = left_values[0]
        self.left_clearance_angle_translation_joint = left_values[1]
        self.left_outer_tube_translation_joint = left_values[2]
        self.left_outer_tube_rotation_joint = left_values[3]
        self.left_inner_tube_translation_joint = left_values[4]
        self.left_inner_tube_rotation_joint = left_values[5]
        self.right_clearance_angle_rotation_joint = right_values[0]
        self.right_clearance_angle_translation_joint = right_values[1]
        self.right_outer_tube_translation_joint = right_values[2]
        self.right_outer_tube_rotation_joint = right_values[3]
        self.right_inner_tube_translation_joint = right_values[4]
        self.right_inner_tube_rotation_joint = right_values[5]
    
    def set_left_joint_values(self, inner_rotation, outer_rotation, inner_translation, outer_translation):
        self._set_joint_values(True, inner_rotation, outer_rotation, inner_translation, outer_translation)

    def set_right_joint_values(self, inner_rotation, outer_rotation, inner_translation, outer_translation):
        self._set_joint_values(False, inner_rotation, outer_rotation, inner_translation, outer_translation)
    
    def _set_joint_values(self, is_left, inner_rotation, outer_rotation, inner_translation, outer_translation):
        # do clearance angle first
        clearance_angle_rotation_joint = 0.1900
        clearance_angle_translation_joint = 0.005
        if outer_translation < 0.005:
            clearance_angle_translation_joint = max(min(clearance_angle_translation_joint, outer_translation), 0.0)
            if outer_translation < 1e-6:
                clearance_angle_rotation_joint = 0.05599
            else:
                tubed_length = max(0.005 - outer_translation, 0.0)
                clearance_angle_rotation_joint = 2*math.atan(
                    (math.sqrt(tubed_length*tubed_length + 0.00000991) - tubed_length) / 0.033033333)
        
        # next do the outer and inner tube translations
        ot_translation = max(outer_translation - clearance_angle_translation_joint, 0.0)
        outer_tube_translation_joint = ot_translation * 60.0
        inner_tube_translation_joint = max(inner_translation - outer_tube_translation_joint, 0.0)

        # finally, do outer and inner rotations
        outer_tube_rotation_joint = outer_rotation
        inner_tube_rotation_joint = inner_rotation - outer_rotation

        # there is a better way to do this, but whatever
        if is_left:
            self.left_clearance_angle_rotation_joint = clearance_angle_rotation_joint
            self.left_clearance_angle_translation_joint = clearance_angle_translation_joint
            self.left_outer_tube_translation_joint = outer_tube_translation_joint
            self.left_outer_tube_rotation_joint = outer_tube_rotation_joint
            self.left_inner_tube_translation_joint = inner_tube_translation_joint
            self.left_inner_tube_rotation_joint = inner_tube_rotation_joint
        else:
            self.right_clearance_angle_rotation_joint = clearance_angle_rotation_joint
            self.right_clearance_angle_translation_joint = clearance_angle_translation_joint
            self.right_outer_tube_translation_joint = outer_tube_translation_joint
            self.right_outer_tube_rotation_joint = outer_tube_rotation_joint
            self.right_inner_tube_translation_joint = inner_tube_translation_joint
            self.right_inner_tube_rotation_joint = inner_tube_rotation_joint

    def to_tensor(self):
        return torch.tensor(np.array([
            self.left_clearance_angle_rotation_joint, self.left_clearance_angle_translation_joint,
            self.left_outer_tube_translation_joint, self.left_outer_tube_rotation_joint,
            self.left_inner_tube_rotation_joint, self.left_inner_tube_translation_joint]),
            dtype=torch.float).repeat(1, 1)


def main():
    """Running keyboard teleoperation with Isaac Lab manipulation environment."""
    # create environment
    task = "Isaac-Reach-Tissue-Virtuoso-v0"
    env_cfg = parse_env_cfg(task, num_envs=1)
    env = gym.make(task, cfg=env_cfg).unwrapped

    # make ROS node
    ros_node = JointStateListener()

    # reset environment
    env.reset()

    # simulate environment
    while simulation_app.is_running():
        # run everything in inference mode
        with torch.inference_mode():
            # get joint command
            # TODO: Get ROS input here!!
            joint_command = torch.tensor(np.array([0.002, 0.1, 0.5, 0.5, 0.001, 0.01]), dtype=torch.float).repeat(1, 1)

            # Only apply teleop commands when active
            # if teleoperation_active:
            # compute actions based on environment
            # actions = pre_process_actions(teleop_data, env.num_envs, env.device)
            # apply actions
            env.step(joint_command)
            # else:
            # env.sim.render()

            # spin ROS
            rclpy.spin_once(ros_node)

    # close the simulator
    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()


