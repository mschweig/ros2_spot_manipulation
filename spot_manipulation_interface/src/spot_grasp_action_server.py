import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from your_package.action import GraspObject
from your_package.grasp_logic import arm_object_grasp
from threading import Thread

class SpotGraspActionServer(Node):
    def __init__(self):
        super().__init__('spot_grasp_action_server')
        self._action_server = ActionServer(self, GraspObject, 'grasp_object', self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Grasp action received')

        class Config:
            hostname = goal_handle.request.hostname
            image_source = goal_handle.request.image_source
            force_top_down_grasp = goal_handle.request.force_top_down_grasp
            force_horizontal_grasp = goal_handle.request.force_horizontal_grasp
            force_45_angle_grasp = goal_handle.request.force_45_angle_grasp
            force_squeeze_grasp = goal_handle.request.force_squeeze_grasp
            verbose = True

        def run():
            try:
                success = arm_object_grasp(Config(), goal_handle.request.pixel_x, goal_handle.request.pixel_y)
                goal_handle.succeed()
                result = GraspObject.Result()
                result.success = success
                result.message = 'Grasp success' if success else 'Grasp failed'
                return result
            except Exception as e:
                goal_handle.abort()
                result = GraspObject.Result()
                result.success = False
                result.message = str(e)
                return result

        thread = Thread(target=run)
        thread.start()
        return rclpy.task.Future()