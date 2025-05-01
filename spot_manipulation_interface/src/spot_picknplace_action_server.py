#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from spot_manipulation_interface.action import PickAndPlace
from src.pick_logic import arm_object_pick
from src.place_logic import arm_object_place
from src.walk_logic import arm_object_walk

class SpotGraspActionServer(Node):
    def __init__(self):
        super().__init__('pick_and_place_object')
        self._action_server = ActionServer(self, PickAndPlace, 'timon/pick_and_place_object', self.execute_callback)
        # Declare parameters for credentials and behavior
        self.declare_parameters(
            namespace='',
            parameters=[
                ('username', ''),
                ('password', ''),
                ('hostname', ''),
            ]
        )
        self.username = self.get_parameter('username').get_parameter_value().string_value
        self.password = self.get_parameter('password').get_parameter_value().string_value
        self.hostname = self.get_parameter('hostname').get_parameter_value().string_value            

    def rotate_pixel_back(self, x, y, camera_name):
        if camera_name in ['frontleft', 'frontright']:
            width, height = 480, 640
            # 90 degree counter-clockwise rotation
            x_orig = y
            y_orig = width - x
        else:
            width, height = 640, 480
            if camera_name == 'right':
                # 180 degree rotation
                x_orig = width - x
                y_orig = height - y
            else:
                # No rotation for other cameras
                x_orig = x
                y_orig = y
        return x_orig, y_orig

    def execute_callback(self, goal_handle):
        self.get_logger().info(f"PicknPlace action received: {goal_handle.request.task}")
        result = PickAndPlace.Result()
        camera_name = goal_handle.request.camera_name
        if camera_name not in ('frontleft', 'frontright', 'left', 'right', 'back'):
            self.get_logger().error("Do not use Hand Camera.")
            result.success, result.message = False, "Use a valid camera"
            return result

        bbox_xmin = goal_handle.request.x_min
        bbox_xmax = goal_handle.request.x_max
        bbox_ymin = goal_handle.request.y_min
        bbox_ymax = goal_handle.request.y_max

        if None in (bbox_xmin, bbox_xmax, bbox_ymin, bbox_ymax):
            # Handle the error or return failure
            self.get_logger().error("One or more bounding box coordinates are not set.")
            result.success, result.message = False, "Missing coordinates, properly set them in the request"
            return result 
        
        center_x = (bbox_xmax + bbox_xmin) / 2.0
        center_y = (bbox_ymax + bbox_ymin) / 2.0

        orig_center_x, orig_center_y = self.rotate_pixel_back(center_x, center_y, camera_name)

        self.get_logger().info(f"Original center after rotation correction: ({orig_center_x}, {orig_center_y}) based on camera: {camera_name}")

        if goal_handle.request.task == "pick":
            result.success, result.message = arm_object_pick(self.username, self.password, self.hostname, orig_center_x, orig_center_y, camera_name)
            goal_handle.succeed()  
            return result
        elif goal_handle.request.task == "walk":
            result.success, result.message = arm_object_walk(self.username, self.password, self.hostname, orig_center_x, orig_center_y, camera_name)
            goal_handle.succeed() 
            return result
        elif goal_handle.request.task == "place":
            result.success, result.message = arm_object_place(self.username, self.password, self.hostname, orig_center_x, orig_center_y, camera_name)
            goal_handle.succeed() 
            return result
        else: 
            self.get_logger().error("Received wrong action type")
            result.success, result.message = False, "Received wrong task type"
            return result


def main(args=None):
    rclpy.init(args=args)
    spot_grasp_action_server = SpotGraspActionServer()
    rclpy.spin(spot_grasp_action_server)

if __name__ == '__main__':
    main()
