import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from spot_manipulation_interface.action import Grasp
from grasp_logic import arm_object_grasp


class SpotGraspActionServer(Node):
    def __init__(self):
        super().__init__('spot_grasp_action_server')
        self._action_server = ActionServer(self, Grasp, 'grasp', self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Grasp action received')
        feedback_msg = Grasp.Feedback()
        feedback_msg.current_state = 'Starting Grasp'
        result = Grasp.Result()
        result.message = arm_object_grasp()
        goal_handle.succeed()  
        return result

def main(args=None):
    rclpy.init(args=args)

    spot_grasp_action_server = SpotGraspActionServer()

    rclpy.spin(spot_grasp_action_server)


if __name__ == '__main__':
    main()

       