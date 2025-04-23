#!/usr/bin/env python3
import rclpy
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
    
        bbox_xmin = goal_handle.request.x_min
        bbox_xmax = goal_handle.request.x_max
        bbox_ymin = goal_handle.request.y_min
        bbox_ymax = goal_handle.request.y_max
        
        center_x = (bbox_xmin + bbox_xmax) // 2
        center_y = (bbox_ymin + bbox_ymax) // 2

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

       