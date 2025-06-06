"""Task implementations for Spot manipulation operations."""

import abc
import time
from typing import Tuple

import numpy as np
from google.protobuf import wrappers_pb2
from bosdyn.api import geometry_pb2, manipulation_api_pb2
from bosdyn.client import frame_helpers
from bosdyn.client.frame_helpers import VISION_FRAME_NAME
from bosdyn.client.image import pixel_to_camera_space
from bosdyn.client.robot_command import RobotCommandBuilder, block_until_arm_arrives

from .spot_client import SpotClient


class ManipulationTask(abc.ABC):
    """Abstract base class for manipulation tasks."""
    
    def __init__(self, spot_client: SpotClient):
        """Initialize task with Spot client.
        
        Args:
            spot_client: Initialized SpotClient instance
        """
        self._spot_client = spot_client
    
    @abc.abstractmethod
    def execute(self, center_x: float, center_y: float, camera_name: str) -> Tuple[bool, str]:
        """Execute the manipulation task.
        
        Args:
            center_x: X coordinate of target center
            center_y: Y coordinate of target center
            camera_name: Name of the camera
            
        Returns:
            Tuple of (success: bool, message: str)
        """
        pass


class PickTask(ManipulationTask):
    """Pick object task implementation."""
    
    def execute(self, center_x: float, center_y: float, camera_name: str) -> Tuple[bool, str]:
        """Execute pick operation."""
        try:
            # Get image for grasp calculation
            image_responses = self._spot_client.image_client.get_image_from_sources(
                [f'{camera_name}_fisheye_image']
            )
            
            if len(image_responses) != 1:
                return False, f"Expected 1 image, got {len(image_responses)}"
            
            image = image_responses[0]
            pick_vec = geometry_pb2.Vec2(x=center_x, y=center_y)
            
            # Build grasp request
            grasp = manipulation_api_pb2.PickObjectInImage(
                pixel_xy=pick_vec,
                transforms_snapshot_for_camera=image.shot.transforms_snapshot,
                frame_name_image_sensor=image.shot.frame_name_image_sensor,
                camera_model=image.source.pinhole
            )
            
            grasp_request = manipulation_api_pb2.ManipulationApiRequest(
                pick_object_in_image=grasp
            )
            
            # Send command and wait for completion
            cmd_response = self._spot_client.manipulation_client.manipulation_api_command(
                manipulation_api_request=grasp_request
            )
            
            # Monitor progress
            while True:
                feedback_request = manipulation_api_pb2.ManipulationApiFeedbackRequest(
                    manipulation_cmd_id=cmd_response.manipulation_cmd_id
                )
                
                response = self._spot_client.manipulation_client.manipulation_api_feedback_command(
                    manipulation_api_feedback_request=feedback_request
                )
                
                state_name = manipulation_api_pb2.ManipulationFeedbackState.Name(response.current_state)
                print(f'Current state: {state_name}')
                
                if response.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_SUCCEEDED:
                    break
                elif response.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_FAILED:
                    return False, "Grasp operation failed"
                
                time.sleep(0.25)
            
            # Transition to carry mode
            time.sleep(2.0)
            carry_cmd = RobotCommandBuilder.arm_carry_command()
            self._spot_client.command_client.robot_command(carry_cmd)
            time.sleep(2.0)
            
            return True, "Object successfully grasped. Continue with 'walk' task to approach the goal location"
            
        except Exception as e:
            return False, f"Pick operation failed: {e}"


class PlaceTask(ManipulationTask):
    """Place object task implementation."""
    
    def execute(self, center_x: float, center_y: float, camera_name: str) -> Tuple[bool, str]:
        """Execute place operation."""
        try:
            # Get depth image for 3D positioning
            image_responses = self._spot_client.image_client.get_image_from_sources(
                [f'{camera_name}_depth_in_visual_frame']
            )
            
            if len(image_responses) != 1:
                return False, f"Expected 1 depth image, got {len(image_responses)}"
            
            image_depth = image_responses[0]
            
            # Process depth image
            depth_image = np.frombuffer(image_depth.shot.image.data, dtype=np.uint16)
            depth_image = depth_image.reshape(
                image_depth.shot.image.rows,
                image_depth.shot.image.cols
            )
            
            # Get valid depth at target location
            raw_depth = self._get_valid_depth(depth_image, int(center_x), int(center_y))
            if raw_depth is None:
                return False, "No valid depth found at target location"
            
            # Calculate 3D position
            depth_scale = image_depth.source.depth_scale or 1000.0
            depth_m = raw_depth / depth_scale
            
            x_cam, y_cam, z_cam = pixel_to_camera_space(
                image_depth.source, int(center_x), int(center_y), depth=depth_m
            )
            
            point_in_camera = geometry_pb2.Vec3(x=x_cam, y=y_cam, z=z_cam)
            
            # Transform to vision frame
            vision_T_camera = frame_helpers.get_a_tform_b(
                image_depth.shot.transforms_snapshot,
                VISION_FRAME_NAME,
                image_depth.shot.frame_name_image_sensor
            )
            
            point_in_vision_np = vision_T_camera.transform_point(
                point_in_camera.x, point_in_camera.y, point_in_camera.z
            )
            
            point_in_vision = geometry_pb2.Vec3(
                x=point_in_vision_np[0],
                y=point_in_vision_np[1],
                z=point_in_vision_np[2]
            )
            
            # Execute place operation
            return self._execute_place_motion(point_in_vision)
            
        except Exception as e:
            return False, f"Place operation failed: {e}"
    
    def _get_valid_depth(self, depth_image: np.ndarray, center_x: int, center_y: int) -> int:
        """Get valid depth value, searching nearby if center is invalid."""
        raw_depth = depth_image[center_y, center_x]
        
        if raw_depth > 0:
            return raw_depth
        
        # Search surrounding area
        search_window = 25
        height, width = depth_image.shape
        best_depth = None
        
        for dy in range(-search_window, search_window + 1):
            for dx in range(-search_window, search_window + 1):
                ny, nx = center_y + dy, center_x + dx
                
                if 0 <= ny < height and 0 <= nx < width:
                    candidate_depth = depth_image[ny, nx]
                    if candidate_depth > 0:
                        if best_depth is None or candidate_depth < best_depth:
                            best_depth = candidate_depth
        
        return best_depth
    
    def _execute_place_motion(self, target_position: geometry_pb2.Vec3) -> Tuple[bool, str]:
        """Execute the physical place motion."""
        # Build desired hand pose
        flat_body_Q_hand = geometry_pb2.Quaternion(w=0.707, x=0, y=0.707, z=0)
        flat_body_tform_hand = geometry_pb2.SE3Pose(
            position=target_position,
            rotation=flat_body_Q_hand
        )
        
        # Move arm to position
        arm_command = RobotCommandBuilder.arm_pose_command_from_pose(
            flat_body_tform_hand,
            frame_helpers.VISION_FRAME_NAME,
            5
        )
        
        gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(0.0)
        command = RobotCommandBuilder.build_synchro_command(gripper_command, arm_command)
        
        cmd_id = self._spot_client.command_client.robot_command(command)
        block_until_arm_arrives(self._spot_client.command_client, cmd_id)
        
        # Open gripper to release object
        gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(1.0)
        command = RobotCommandBuilder.build_synchro_command(gripper_command)
        self._spot_client.command_client.robot_command(command)
        
        time.sleep(3)  # Wait for object to fall out
        
        # Stow arm
        stow_cmd = RobotCommandBuilder.arm_stow_command()
        stow_command_id = self._spot_client.command_client.robot_command(stow_cmd)
        block_until_arm_arrives(self._spot_client.command_client, stow_command_id, 3.0)
        time.sleep(1)

        #close gripper
        robot_cmd = RobotCommandBuilder.claw_gripper_close_command()
        cmd_id = self._spot_client.command_client.robot_command(robot_cmd)
        
        return True, "Place task successfully executed, Arm is now stowed."


class WalkTask(ManipulationTask):
    """Walk to object task implementation."""
    
    def execute(self, center_x: float, center_y: float, camera_name: str) -> Tuple[bool, str]:
        """Execute walk operation."""
        try:
            # Get image for walk calculation
            image_responses = self._spot_client.image_client.get_image_from_sources(
                [f'{camera_name}_fisheye_image']
            )
            
            if len(image_responses) != 1:
                return False, f"Expected 1 image, got {len(image_responses)}"
            
            image = image_responses[0]
            walk_vec = geometry_pb2.Vec2(x=center_x, y=center_y)
            
            # Build walk request
            offset_distance = wrappers_pb2.FloatValue(value=0.5)
            walk_to = manipulation_api_pb2.WalkToObjectInImage(
                pixel_xy=walk_vec,
                transforms_snapshot_for_camera=image.shot.transforms_snapshot,
                frame_name_image_sensor=image.shot.frame_name_image_sensor,
                camera_model=image.source.pinhole,
                offset_distance=offset_distance
            )
            
            walk_request = manipulation_api_pb2.ManipulationApiRequest(
                walk_to_object_in_image=walk_to
            )
            
            # Send command and monitor progress
            cmd_response = self._spot_client.manipulation_client.manipulation_api_command(
                manipulation_api_request=walk_request
            )
            
            while True:
                time.sleep(0.25)
                feedback_request = manipulation_api_pb2.ManipulationApiFeedbackRequest(
                    manipulation_cmd_id=cmd_response.manipulation_cmd_id
                )
                
                response = self._spot_client.manipulation_client.manipulation_api_feedback_command(
                    manipulation_api_feedback_request=feedback_request
                )
                
                state_name = manipulation_api_pb2.ManipulationFeedbackState.Name(response.current_state)
                print(f'Current state: {state_name}')
                
                if response.current_state == manipulation_api_pb2.MANIP_STATE_DONE:
                    break
            
            return True, "Walk goal successfully reached, check again the bounding box of the goal location to continue with 'place' task"
            
        except Exception as e:
            return False, f"Walk operation failed: {e}"

