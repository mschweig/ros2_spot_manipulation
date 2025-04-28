#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from spot_manipulation_interface.action import PickAndPlace
from src.grasp_logic import arm_object_pick
from src.place_logic import arm_object_place

class SpotGraspActionServer(Node):
    def __init__(self):
        super().__init__('spot_picknplace_action_server')
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
        action_type = "Pick" if PickAndPlace.pick else "Place" if PickAndPlace.place else None
        self.get_logger().info(f"PicknPlace action received: {action_type}")
        feedback_msg = PickAndPlace.Feedback()
        result = PickAndPlace.Result()
        camera_name = goal_handle.request.camera_name
        if camera_name == 'frontleft' or camera_name == 'frontright':
            width, height = 480, 640
        else:
            width, height = 640, 480

        bbox_xmin = goal_handle.request.x_min * width / 1000
        bbox_xmax = goal_handle.request.x_max * width / 1000
        bbox_ymin = goal_handle.request.y_min * height / 1000
        bbox_ymax = goal_handle.request.y_max * height / 1000

        center_x = (bbox_xmax + bbox_xmin) / 2.0
        center_y = (bbox_ymax + bbox_ymin) / 2.0

        orig_center_x, orig_center_y = self.rotate_pixel_back(center_x, center_y, camera_name)

        self.get_logger().info(f"Original center after rotation correction: ({orig_center_x}, {orig_center_y}) based on camera: {camera_name}")

        if PickAndPlace.pick:
            result.success = arm_object_pick(self.username, self.password, self.hostname, orig_center_x, orig_center_y, camera_name)
            result.message = "Grasp suceeded"
            goal_handle.succeed()  
            return result
        elif PickAndPlace.place:
            result.success = arm_object_place(self.username, self.password, self.hostname, orig_center_x, orig_center_y, camera_name)
            result.message = "Grasp suceeded"
            goal_handle.succeed() 
            return result
        else: 
            self.get_logger().error("Received wrong action type")



def main(args=None):
    rclpy.init(args=args)
    spot_grasp_action_server = SpotGraspActionServer()
    rclpy.spin(spot_grasp_action_server)

if __name__ == '__main__':
    main()
