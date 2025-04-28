# Copyright (c) 2023 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

import time
import bosdyn.client
import bosdyn.client.estop
import bosdyn.client.lease
import bosdyn.client.util
from bosdyn.api import geometry_pb2, manipulation_api_pb2
from bosdyn.client import frame_helpers
from bosdyn.client.frame_helpers import VISION_FRAME_NAME, get_vision_tform_body, math_helpers
from bosdyn.client.image import ImageClient
from bosdyn.client.manipulation_api_client import ManipulationApiClient
from bosdyn.client.robot_command import RobotCommandClient, blocking_stand, RobotCommandBuilder, block_until_arm_arrives
from bosdyn.client.robot_state import RobotStateClient


def arm_object_place(username, password, hostname, center_x, center_y, camera_name):
    """A simple example of using the Boston Dynamics API to command Spot's arm."""

    sdk = bosdyn.client.create_standard_sdk('ROS2 Place Action')
    robot = sdk.create_robot(hostname)
    robot.authenticate(username, password)
    robot.time_sync.wait_for_sync()

    lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
    image_client = robot.ensure_client(ImageClient.default_service_name)
    manipulation_api_client = robot.ensure_client(ManipulationApiClient.default_service_name)

    lease_client.take()

    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=False):

        command_client = robot.ensure_client(RobotCommandClient.default_service_name)

        image_responses = image_client.get_image_from_sources([camera_name+'_fisheye_image'])

        if len(image_responses) != 1:
            print(f'Got invalid number of images: {len(image_responses)}')
            print(image_responses)
            assert False

        walk_vec = geometry_pb2.Vec2(x=center_x, y=center_y)

        # Build the proto
        image = image_responses[0]

        # Walk to Goal
             # Build the proto
        walk_to = manipulation_api_pb2.WalkToObjectInImage(
            pixel_xy=walk_vec, transforms_snapshot_for_camera=image.shot.transforms_snapshot,
            frame_name_image_sensor=image.shot.frame_name_image_sensor,
            camera_model=image.source.pinhole, offset_distance=0.8)

        # Ask the robot to pick up the object
        walk_to_request = manipulation_api_pb2.ManipulationApiRequest(
            walk_to_object_in_image=walk_to)

        # Send the request
        cmd_response = manipulation_api_client.manipulation_api_command(
            manipulation_api_request=walk_to_request)

        # Get feedback from the robot
        while True:
            time.sleep(0.25)
            feedback_request = manipulation_api_pb2.ManipulationApiFeedbackRequest(
                manipulation_cmd_id=cmd_response.manipulation_cmd_id)

            # Send the request
            response = manipulation_api_client.manipulation_api_feedback_command(
                manipulation_api_feedback_request=feedback_request)

            print('Current state: ',
                  manipulation_api_pb2.ManipulationFeedbackState.Name(response.current_state))

            if response.current_state == manipulation_api_pb2.MANIP_STATE_DONE:
                break

        robot.logger.info('At Goal Location... Getting Ready for Manipulation')
        
        # Build to Arm Goal
        vision_tform_obj = frame_helpers.get_a_tform_b(image.shot.transforms_snapshot, frame_helpers.VISION_FRAME_NAME,
                                                       image.shot.image_properties.frame_name_image_coordinates)

        
        hand_ewrt_flat_body = geometry_pb2.Vec3(x=(vision_tform_obj.position.x-0.08),
                                                y=vision_tform_obj.position.y,
                                                z=(vision_tform_obj.position.z))

        # Point the hand straight down with a quaternion.
        flat_body_Q_hand = geometry_pb2.Quaternion(w=1, x=0, y=0, z=0)

        flat_body_tform_hand = geometry_pb2.SE3Pose(position=hand_ewrt_flat_body,
                                                    rotation=flat_body_Q_hand)

        arm_command = RobotCommandBuilder.arm_pose_command_from_pose(
            flat_body_tform_hand,
            frame_helpers.VISION_FRAME_NAME,
            5)

        # Keep the gripper closed.
        gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(0.0)

        # Combine the arm and gripper commands into one RobotCommand
        command = RobotCommandBuilder.build_synchro_command(gripper_command, arm_command)

        # Send the request
        cmd_id = command_client.robot_command(command)

        # Wait until the arm arrives at the goal.
        block_until_arm_arrives(command_client, cmd_id)

        # Open the gripper
        gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(1.0)
        command = RobotCommandBuilder.build_synchro_command(gripper_command)
        cmd_id = command_client.robot_command(command)

        # Wait for the object to fall out
        time.sleep(1.5)
        





