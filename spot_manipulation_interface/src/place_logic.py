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
import numpy as np
from google.protobuf import wrappers_pb2
from bosdyn.api import geometry_pb2, manipulation_api_pb2
from bosdyn.client import frame_helpers
from bosdyn.client.frame_helpers import VISION_FRAME_NAME, get_vision_tform_body, math_helpers
from bosdyn.client.image import ImageClient, pixel_to_camera_space
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
    lease_client.take()

    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=False):

        command_client = robot.ensure_client(RobotCommandClient.default_service_name)

        image_responses = image_client.get_image_from_sources([camera_name+'_fisheye_image', camera_name+'_depth_in_visual_frame'])

        if len(image_responses) != 1:
            print(f'Got invalid number of images: {len(image_responses)}')
            print(image_responses)
            assert False

                # Unpack images
        image_color = image_responses[0]
        image_depth = image_responses[1]

        # Load depth image
        depth_image = np.frombuffer(image_depth.shot.image.data, dtype=np.uint16)
        depth_image = depth_image.reshape(
            image_depth.shot.image.rows,
            image_depth.shot.image.cols
        )
        
        # Read depth at the requested center pixel
        raw_depth = depth_image[int(center_y), int(center_x)]
        if raw_depth == 0:
            raise ValueError("Invalid depth value (zero) at given pixel.")

        # Scale depth properly (depth_scale is usually 1000 for mm->m conversion)
        depth_scale = image_depth.source.depth_scale if image_depth.source.depth_scale else 1000.0
        depth_m = raw_depth / depth_scale

        # Compute 3D point in camera frame
        x_cam, y_cam, z_cam = pixel_to_camera_space(
            image_depth.source, int(center_x), int(center_y), depth=depth_m
        )

        # Build a 3D point in the camera frame
        point_in_camera = geometry_pb2.Vec3(x=x_cam, y=y_cam, z=z_cam)

        # Now transform point from camera frame to vision frame
        vision_T_camera = frame_helpers.get_a_tform_b(
            image_color.shot.transforms_snapshot,  # Use snapshot from color image
            VISION_FRAME_NAME,
            image_color.shot.frame_name_image_sensor
        )

        # Transform the point
        point_in_vision = math_helpers.transform_point(vision_T_camera, point_in_camera)

        
        # Define the desired hand position
        hand_ewrt_flat_body = geometry_pb2.Vec3(
            x=point_in_vision.x,
            y=point_in_vision.y,
            z=point_in_vision.z
        )

        # Use simple orientation: gripper pointing down
        flat_body_Q_hand = geometry_pb2.Quaternion(w=1, x=0, y=0, z=0)

        flat_body_tform_hand = geometry_pb2.SE3Pose(
            position=hand_ewrt_flat_body,
            rotation=flat_body_Q_hand
        )

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

        return True, "Object sucessfully placed"
        





