#!/usr/bin/env python3
import rclpy
import math
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

        result.message = arm_object_grasp(self.username, self.password, self.hostname, center_x, center_y)
        result.success = True
        goal_handle.succeed()  
        return result

def main(args=None):
    rclpy.init(args=args)

    spot_grasp_action_server = SpotGraspActionServer()

    rclpy.spin(spot_grasp_action_server)


if __name__ == '__main__':
    main()

       