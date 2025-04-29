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
from google.protobuf import wrappers_pb2
from bosdyn.api import geometry_pb2, manipulation_api_pb2
from bosdyn.client import frame_helpers
from bosdyn.client.frame_helpers import VISION_FRAME_NAME, get_vision_tform_body, math_helpers
from bosdyn.client.image import ImageClient
from bosdyn.client.manipulation_api_client import ManipulationApiClient
from bosdyn.client.robot_command import RobotCommandClient, blocking_stand, RobotCommandBuilder, block_until_arm_arrives
from bosdyn.client.robot_state import RobotStateClient


def arm_object_walk(username, password, hostname, center_x, center_y, camera_name):
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

        image_responses = image_client.get_image_from_sources([camera_name+'_fisheye_image'])

        if len(image_responses) != 1:
            print(f'Got invalid number of images: {len(image_responses)}')
            print(image_responses)
            assert False

        walk_vec = geometry_pb2.Vec2(x=center_x, y=center_y)

        # Build the proto
        image = image_responses[0]

        ### Walk to Goal ###
        # Build the proto
        offset_distance = wrappers_pb2.FloatValue(value=0.8)
        walk_to = manipulation_api_pb2.WalkToObjectInImage(
            pixel_xy=walk_vec, transforms_snapshot_for_camera=image.shot.transforms_snapshot,
            frame_name_image_sensor=image.shot.frame_name_image_sensor,
            camera_model=image.source.pinhole, offset_distance=offset_distance)

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

        return True, "Walk Goal Reached"
        





