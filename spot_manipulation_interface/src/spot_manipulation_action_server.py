#!/usr/bin/env python3
"""ROS2 action server for Spot manipulation operations."""

import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from spot_manipulation_interface.action import Manipulate
from src.spot_client import SpotClient, SpotClientError
from src.camera_utils import CameraCoordinateTransformer
from src.task_factory import TaskFactory


class SpotManipulationActionServer(Node):
    """ROS2 action server for Spot pick and place operations."""
    
    def __init__(self):
        """Initialize the action server."""
        super().__init__('spot_manipulation_action_server')
        
        # Get parameters
        self._get_parameters()
        
        self._action_server = ActionServer(
            self,
            Manipulate,
            self._robot_name+'/spot_manipulation_action_server',
            self.execute_callback
        )
                
        self.get_logger().info("Spot Manipulation Action Server initialized")
    
    def _get_parameters(self) -> None:
        """Get parameter values."""
        self._username = os.environ.get('BOSDYN_CLIENT_USERNAME')
        self._password = os.environ.get('BOSDYN_CLIENT_PASSWORD')
        self._hostname = os.environ.get('SPOT_IP')
        self._robot_name = os.environ.get('ROBOT_NAME')
        
        # Validate required parameters
        if not all([self._username, self._password, self._hostname, self._robot_name]):
            raise ValueError("Missing required parameters: username, password, or hostname")
    
    def execute_callback(self, goal_handle):
        """Execute the pick and place action."""
        self.get_logger().info(f"Received {goal_handle.request.task} request")
        
        result = Manipulate.Result()
        
        try:
            # Validate request
            validation_result = self._validate_request(goal_handle.request)
            if not validation_result[0]:
                result.success = False
                result.message = validation_result[1]
                goal_handle.succeed()
                return result
            
            # Calculate target coordinates
            center_x, center_y = self._calculate_target_center(goal_handle.request)
            
            # Transform coordinates based on camera orientation
            orig_center_x, orig_center_y = CameraCoordinateTransformer.transform_coordinates(
                center_x, center_y, goal_handle.request.camera_name
            )
            
            self.get_logger().info(
                f"Target center: ({orig_center_x:.1f}, {orig_center_y:.1f}) "
                f"for camera: {goal_handle.request.camera_name}"
            )
            
            # Execute task
            success, message = self._execute_task(
                goal_handle.request.task,
                orig_center_x,
                orig_center_y,
                goal_handle.request.camera_name
            )
            
            result.success = success
            result.message = message
            
        except Exception as e:
            self.get_logger().error(f"Action execution failed: {e}")
            result.success = False
            result.message = f"Execution failed: {e}"
        
        goal_handle.succeed()
        return result
    
    def _validate_request(self, request) -> tuple:
        """Validate the action request.
        
        Args:
            request: The action request
            
        Returns:
            Tuple of (is_valid: bool, error_message: str)
        """
        # Validate camera name
        if not CameraCoordinateTransformer.validate_camera_name(request.camera_name):
            return False, f"Invalid camera name: {request.camera_name}"
        
        # Validate task type
        if request.task not in TaskFactory.get_supported_tasks():
            available_tasks = TaskFactory.get_supported_tasks()
            return False, f"Unsupported task: {request.task}. Available: {available_tasks}"
        
        # Validate coordinates
        coords = [request.x_min, request.x_max, request.y_min, request.y_max]
        if any(coord is None for coord in coords):
            return False, "Missing bounding box coordinates"
        
        if request.x_min >= request.x_max or request.y_min >= request.y_max:
            return False, "Invalid bounding box: min values must be less than max values"
        
        return True, ""
    
    def _calculate_target_center(self, request) -> tuple:
        """Calculate the center point of the bounding box.
        
        Args:
            request: The action request
            
        Returns:
            Tuple of (center_x, center_y)
        """
        center_x = (request.x_max + request.x_min) / 2.0
        center_y = (request.y_max + request.y_min) / 2.0
        return center_x, center_y
    
    def _execute_task(self, task_type: str, center_x: float, center_y: float, 
                     camera_name: str) -> tuple:
        """Execute the manipulation task.
        
        Args:
            task_type: Type of task to execute
            center_x: X coordinate of target center
            center_y: Y coordinate of target center
            camera_name: Name of the camera
            
        Returns:
            Tuple of (success: bool, message: str)
        """
        spot_client = SpotClient(
            username=self._username,
            password=self._password,
            hostname=self._hostname,
            logger=self.get_logger()
        )
        
        try:
            with spot_client.get_robot_context():
                task = TaskFactory.create_task(task_type, spot_client)
                return task.execute(center_x, center_y, camera_name)
                
        except SpotClientError as e:
            self.get_logger().error(f"Spot client error: {e}")
            return False, f"Robot communication error: {e}"
        except Exception as e:
            self.get_logger().error(f"Task execution error: {e}")
            return False, f"Task execution failed: {e}"


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        action_server = SpotManipulationActionServer()
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Failed to start action server: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()