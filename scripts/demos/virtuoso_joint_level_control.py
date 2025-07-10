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
from isaaclab.managers import TerminationTermCfg as DoneTerm
import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.manager_based.manipulation.lift import mdp

import math
import threading

# ROS
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
# Node = None
# JointState = None

# Define a ROS node
class JointStateListener(Node):
    def __init__(self):
        super().__init__('isaaclab_node_thing')
        self.subscription = self.create_subscription(
            JointState,
            '/ves/left/joint/measured_jp',
            self.left_joint_state_callback,
            3,
        )
        self.subscription = self.create_subscription(
            JointState,
            '/ves/right/joint/measured_jp',
            self.right_joint_state_callback,
            3,
        )

        self.joint_states = VirtuosoJointState()
    
    def set_joint_state(self, names, positions, left_side):
        try:
            values = (
                positions[names.index('inner_rotation')],
                positions[names.index('outer_rotation')],
                positions[names.index('inner_translation')],
                positions[names.index('outer_translation')],
            )
            if left_side:
                self.joint_states.set_left_joint_values(*values)
            else:
                self.joint_states.set_right_joint_values(*values)

        except Exception as e:
            print(f'Setting joint values failed: {e}')

    def left_joint_state_callback(self, msg: JointState):
        # self.get_logger().info('Received JointState for LEFT side:')
        # self.get_logger().info(f'  Names: {msg.name}')
        # self.get_logger().info(f'  Positions: {msg.position}')

        self.set_joint_state(msg.name, msg.position, True)
    
    def right_joint_state_callback(self, msg: JointState):
        # self.get_logger().info('Received JointState for RIGHT side:')
        # self.get_logger().info(f'  Names: {msg.name}')
        # self.get_logger().info(f'  Positions: {msg.position}')

        self.set_joint_state(msg.name, msg.position, False)


# Define the Virtuoso joint state data structure
class VirtuosoJointState:
    def __init__(self, left_values = (0.0,)*6, right_values = (0.0,)*6):
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

        self.left_outer_rotation_wrapping = 0
        self.left_inner_rotation_wrapping = 0
        self.right_outer_rotation_wrapping = 0
        self.right_inner_rotation_wrapping = 0

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
        inner_tube_translation_joint = max(inner_translation - outer_translation, 0.0)

        # finally, do outer and inner rotations
        outer_tube_rotation_joint = outer_rotation
        inner_tube_rotation_joint = inner_rotation - outer_rotation

        # there is a better way to do this, but whatever
        if is_left:
            self.left_clearance_angle_rotation_joint = clearance_angle_rotation_joint
            self.left_clearance_angle_translation_joint = clearance_angle_translation_joint
            self.left_outer_tube_translation_joint = outer_tube_translation_joint
            self.left_inner_tube_translation_joint = inner_tube_translation_joint

            # outer rotation joints have to be wrapped (inner rotation joints are always borked lol)
            if (self.left_outer_tube_rotation_joint % (2*math.pi)) < math.pi/2 and outer_tube_rotation_joint > 3*math.pi/2:
                self.left_outer_rotation_wrapping -= 1 if abs(self.left_outer_rotation_wrapping) < 2 else 0
            elif (self.left_outer_tube_rotation_joint % (2*math.pi)) > 3*math.pi/2 and outer_tube_rotation_joint < math.pi/2:
                self.left_outer_rotation_wrapping += 1 if abs(self.left_outer_rotation_wrapping) < 2 else 0

            self.left_outer_tube_rotation_joint = outer_tube_rotation_joint + 2*math.pi*self.left_outer_rotation_wrapping

            # if (self.left_inner_tube_rotation_joint % (2*math.pi)) < math.pi/2 and inner_tube_rotation_joint > 3*math.pi/2:
            #     self.left_inner_rotation_wrapping -= 1
            # elif (self.left_inner_tube_rotation_joint % (2*math.pi)) > 3*math.pi/2 and inner_tube_rotation_joint < math.pi/2:
            #     self.left_inner_rotation_wrapping += 1

            self.left_inner_tube_rotation_joint = inner_tube_rotation_joint #+ 2*math.pi*(self.left_inner_rotation_wrapping + 1)

        else:
            self.right_clearance_angle_rotation_joint = clearance_angle_rotation_joint
            self.right_clearance_angle_translation_joint = clearance_angle_translation_joint
            self.right_outer_tube_translation_joint = outer_tube_translation_joint
            self.right_inner_tube_translation_joint = inner_tube_translation_joint

            # outer rotation joints have to be wrapped (inner rotation joints are always borked lol)
            if (self.right_outer_tube_rotation_joint % (2*math.pi)) < math.pi/2 and outer_tube_rotation_joint > 3*math.pi/2:
                self.right_outer_rotation_wrapping -= 1 if abs(self.right_outer_rotation_wrapping) < 2 else 0
            elif (self.right_outer_tube_rotation_joint % (2*math.pi)) > 3*math.pi/2 and outer_tube_rotation_joint < math.pi/2:
                self.right_outer_rotation_wrapping += 1 if abs(self.right_outer_rotation_wrapping) < 2 else 0

            self.right_outer_tube_rotation_joint = outer_tube_rotation_joint + 2*math.pi*self.right_outer_rotation_wrapping
            self.right_inner_tube_rotation_joint = inner_tube_rotation_joint

            # if (self.right_inner_tube_rotation_joint % (2*math.pi)) < math.pi/2 and inner_tube_rotation_joint > 3*math.pi/2:
            #     self.right_inner_rotation_wrapping -= 1
            # elif (self.right_inner_tube_rotation_joint % (2*math.pi)) > 3*math.pi/2 and inner_tube_rotation_joint < math.pi/2:
            #     self.right_inner_rotation_wrapping += 1

        # print(f'left inner rotation: {self.left_inner_tube_rotation_joint}')

    def to_tensor(self, device='cuda:0'):
        return torch.tensor(np.array([
            self.left_clearance_angle_rotation_joint, self.left_clearance_angle_translation_joint,
            # self.left_outer_tube_rotation_joint, self.left_outer_tube_translation_joint,
            self.left_outer_tube_translation_joint, self.left_outer_tube_rotation_joint,
            self.left_inner_tube_translation_joint, self.left_inner_tube_rotation_joint,
            self.right_clearance_angle_rotation_joint, self.right_clearance_angle_translation_joint,
            # self.right_outer_tube_rotation_joint, self.right_outer_tube_translation_joint,
            self.right_outer_tube_translation_joint, self.right_outer_tube_rotation_joint,
            self.right_inner_tube_translation_joint, self.right_inner_tube_rotation_joint]),
            dtype=torch.float, device=device).repeat(1, 1)


def main(args=None):
    """Running keyboard teleoperation with Isaac Lab manipulation environment."""
    # create environment
    task = "Isaac-Reach-Tissue-Virtuoso-v0"
    env_cfg = parse_env_cfg(task, num_envs=1)
    env_cfg.terminations.time_out = None
    env = gym.make(task, cfg=env_cfg).unwrapped

    # make ROS node
    rclpy.init(args=args)
    ros_node = JointStateListener()
    spinner_thread = threading.Thread(target=rclpy.spin, args=(ros_node,))
    spinner_thread.start()

    # reset environment
    env.reset()

    # simulate environment
    while simulation_app.is_running():
        # run everything in inference mode
        with torch.inference_mode():
            # get joint command
            joint_command = ros_node.joint_states.to_tensor()
            # print(f'Joint command: {joint_command}')

            # apply actions
            env.step(joint_command)

    # close the simulator
    env.close()

    # shutdown ROS
    rclpy.shutdown()
    spinner_thread.join()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()


