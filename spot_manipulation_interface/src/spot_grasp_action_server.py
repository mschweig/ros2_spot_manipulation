#!/usr/bin/env python3
import rclpy
import math
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionServer
from spot_manipulation_interface.action import Grasp
from src.grasp_logic import arm_object_grasp

class SpotGraspActionServer(Node):
    def __init__(self):
        super().__init__('spot_grasp_action_server')
        self._action_server = ActionServer(self, Grasp, 'timon/grasp_object', self.execute_callback)
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

    def rotate_point(self, x, y, cx, cy, angle_rad):
        cos_theta = math.cos(angle_rad)
        sin_theta = math.sin(angle_rad)
        x_shifted = x - cx
        y_shifted = y - cy
        x_rot = cos_theta * x_shifted - sin_theta * y_shifted + cx
        y_rot = sin_theta * x_shifted + cos_theta * y_shifted + cy
        return [x_rot, y_rot]

    def execute_callback(self, goal_handle):
        self.get_logger().info('Grasp action received')
        feedback_msg = Grasp.Feedback()
        feedback_msg.current_state = 'Starting Grasp'
        result = Grasp.Result()

        bbox_xmin = goal_handle.request.x_min * 640 / 1000
        bbox_xmax = goal_handle.request.x_max * 640 / 1000
        bbox_ymin = goal_handle.request.y_min * 480 / 1000
        bbox_ymax = goal_handle.request.y_max * 480 / 1000

        center_x = (bbox_xmax + bbox_xmin) / 2.0
        center_y = (bbox_ymax + bbox_ymin) / 2.0

        # rotate the bounding box around its center based on camera frame
        image_source = goal_handle.request.camera_name
        rotation_angle_deg = {
            'back': 0,
            'frontleft': -78,
            'frontright': -102,
            'left': 0,
            'right': 180
        }.get(image_source, 0)  # default to 0 if not found

        rotation_angle_rad = math.radians(rotation_angle_deg)

        # rotate each corner of the bounding box
        corners = np.array([
            [bbox_xmin, bbox_ymin],
            [bbox_xmax, bbox_ymin],
            [bbox_xmax, bbox_ymax],
            [bbox_xmin, bbox_ymax]
        ])

        rotated_corners = [self.rotate_point(x, y, center_x, center_y, rotation_angle_rad) for x, y in corners]

        # compute new center after rotation (optional: could still use original center)
        rotated_center_x = sum(pt[0] for pt in rotated_corners) / 4.0
        rotated_center_y = sum(pt[1] for pt in rotated_corners) / 4.0

        self.get_logger().info(f"Rotated center: ({rotated_center_x:.2f}, {rotated_center_y:.2f}) from original center: ({center_x:.2f}, {center_y:.2f})")

        result.message = arm_object_grasp(self.username, self.password, self.hostname, rotated_center_x, rotated_center_y)
        result.success = True
        goal_handle.succeed()  
        return result


def main(args=None):
    rclpy.init(args=args)
    spot_grasp_action_server = SpotGraspActionServer()
    rclpy.spin(spot_grasp_action_server)

if __name__ == '__main__':
    main()
