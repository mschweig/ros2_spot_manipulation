def arm_object_grasp(config, pixel_x, pixel_y):
    import bosdyn.client.util
    from bosdyn.api import geometry_pb2, manipulation_api_pb2, image_pb2
    from bosdyn.client.frame_helpers import VISION_FRAME_NAME
    from bosdyn.client.image import ImageClient
    from bosdyn.client.manipulation_api_client import ManipulationApiClient
    from bosdyn.client.robot_command import RobotCommandClient, blocking_stand
    from bosdyn.client.robot_state import RobotStateClient
    from bosdyn.client.lease import LeaseKeepAlive
    from bosdyn.client.estop import EstopClient
    from bosdyn.client.frame_helpers import get_vision_tform_body, math_helpers
    import numpy as np
    import cv2
    import time

    sdk = bosdyn.client.create_standard_sdk('ArmObjectGraspClient')
    robot = sdk.create_robot(config.hostname)
    bosdyn.client.util.authenticate(robot)
    robot.time_sync.wait_for_sync()

    assert robot.has_arm(), 'Robot requires an arm to run this example.'
    client = robot.ensure_client(EstopClient.default_service_name)
    if client.get_status().stop_level != 0:
        raise Exception('Robot is estopped')

    lease_client = robot.ensure_client('lease')
    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
    image_client = robot.ensure_client(ImageClient.default_service_name)
    manipulation_api_client = robot.ensure_client(ManipulationApiClient.default_service_name)

    with LeaseKeepAlive(lease_client):
        robot.power_on(timeout_sec=20)
        blocking_stand(robot.ensure_client(RobotCommandClient.default_service_name))

        image = image_client.get_image_from_sources([config.image_source])[0]
        pick_vec = geometry_pb2.Vec2(x=pixel_x, y=pixel_y)

        grasp = manipulation_api_pb2.PickObjectInImage(
            pixel_xy=pick_vec,
            transforms_snapshot_for_camera=image.shot.transforms_snapshot,
            frame_name_image_sensor=image.shot.frame_name_image_sensor,
            camera_model=image.source.pinhole)

        # Grasp constraints
        if config.force_top_down_grasp:
            axis_on_gripper = geometry_pb2.Vec3(x=1, y=0, z=0)
            axis_align_with = geometry_pb2.Vec3(x=0, y=0, z=-1)
        elif config.force_horizontal_grasp:
            axis_on_gripper = geometry_pb2.Vec3(x=0, y=1, z=0)
            axis_align_with = geometry_pb2.Vec3(x=0, y=0, z=1)
        else:
            axis_on_gripper = axis_align_with = None

        if axis_on_gripper:
            grasp.grasp_params.grasp_params_frame_name = VISION_FRAME_NAME
            constraint = grasp.grasp_params.allowable_orientation.add()
            constraint.vector_alignment_with_tolerance.axis_on_gripper_ewrt_gripper.CopyFrom(axis_on_gripper)
            constraint.vector_alignment_with_tolerance.axis_to_align_with_ewrt_frame.CopyFrom(axis_align_with)
            constraint.vector_alignment_with_tolerance.threshold_radians = 0.17

        grasp_request = manipulation_api_pb2.ManipulationApiRequest(pick_object_in_image=grasp)
        cmd_response = manipulation_api_client.manipulation_api_command(grasp_request)

        while True:
            feedback = manipulation_api_client.manipulation_api_feedback_command(
                manipulation_api_pb2.ManipulationApiFeedbackRequest(
                    manipulation_cmd_id=cmd_response.manipulation_cmd_id))
            state = feedback.current_state
            if state in [3, 4]:  # succeeded or failed
                break
            time.sleep(0.25)

        robot.power_off(cut_immediately=False)
        return state == 3