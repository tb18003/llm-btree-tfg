from rclpy.node import Node
from py_trees.common import Status
from py_trees.composites import Sequence
from std_msgs.msg import String
from geometry_msgs.msg import Point32
from task_manager.task_model import Task # type: ignore
from robot_sim_interfaces.srv import TTSService # type: ignore
from nav2_msgs.action import NavigateToPose
#from task_manager.task_executor_node import TaskExecutorNode

from action_msgs.msg import GoalStatus

import json
import rclpy

class MoveTask(Task):
    """
    A Task that sends a navigation goal to the robot using the nav2 stack.

    Args:
        args (dict): Dictionary with keys 'x', 'y', and 'theta' specifying the target position and orientation.

    Behavior:
        - On setup, checks for required arguments and node.
        - On update, sends a navigation goal and monitors its status.
        - Sets status to SUCCESS if the robot reaches the goal, FAILURE otherwise.
        - If the task cannot be executed, returns a TTSTask apologizing as opposite behavior.
    """

    def __init__(self, args):
        super().__init__(args, 'move')
        self.node = None

        self.status = Status.RUNNING
        self.opposite_task = TTSTask({"speech": "Lo siento, no he podido llegar a la localización"})

    def setup(self, **kwargs):
        if kwargs['node'] is None:
            raise Exception("(MoveTask::setup) Cannot find 'node' argument")
        
        if self.args is None:
            raise Exception("(MoveTask::setup) Cannot find 'args' argument")
        
        if 'x' not in self.args.keys() or \
           'y' not in self.args.keys() or \
           'theta' not in self.args.keys():
            raise Exception("(MoveTask::setup) Malformed arguments, check that 'x', 'y' and 'theta' variables are set correctly" \
            f" Arguments type: {type(self.args)} JSON argument: {json.JSONEncoder().encode(self.args)}")
            
        self.node : Node = kwargs['node']

    def _send_goal_nav2(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = float(self.args['x'])
        goal_msg.pose.pose.position.y = float(self.args['y'])
        goal_msg.pose.pose.orientation.z = 1.0
        goal_msg.pose.pose.orientation.w = float(self.args['theta'])

        self.node.nav_client.wait_for_server(timeout_sec=3.0)
        f = self.node.nav_client.send_goal_async(goal_msg)
        f.add_done_callback(self._goal_callback)
        self.sent = True

    def _goal_callback(self, future):
        res = future.result()

        if not res.accepted:
            self.node.get_logger().error("NAV2 rejected btw")
            self.status = Status.FAILURE
        else:
            self.node.get_logger().info("Goal accepted!")
            f = res.get_result_async()
            f.add_done_callback(self._result_callback)
            self._r_f = f
    
    def _result_callback(self, future):
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info("Navegación completada con éxito")
            self.status = Status.SUCCESS
        elif status == GoalStatus.STATUS_ABORTED:
            self.node.get_logger().error("Navigation error: Can't reach the position")
            self.status = Status.FAILURE
        else:
            self.node.get_logger().error("Navigation error: Unknown error")
            self.status = Status.FAILURE

    def update(self):
        if not hasattr(self, '_future'):
            self._future = self._send_goal_nav2()

        rclpy.spin_once(self.node, timeout_sec=0.1)

        if 'id' in self.args.keys():
            self.node.log_tasks(self.args['id'], self.status)
        return self.status
    
    def opposite_behavior(self):
        # if can't go to goal, it goes back to home
        if self.args['x'] != -1 and self.args['y'] != 0:
            return MoveTask({'x': -1, 'y': 0, 'theta': 1})
        
        return self.opposite_task
    
class MoveSanchoTask(MoveTask):
    """
    Same as MoveTask, but changing the 'home' coordinates for the Sancho robot context.
    """
    
    def __init__(self, args):
        super().__init__(args)


    def opposite_behavior(self):
        if self.args['x'] != 0 and self.args['y'] != 0:
            return MoveTask({'x': 0, 'y': 0, 'theta': 1})
        
        return self.opposite_task

class TTSTask(Task):
    """
    A Task that sends a text-to-speech (TTS) request using a ROS2 service.

    Args:
        args (dict): Dictionary with at least the key 'speech' containing the text to be spoken.

    Behavior:
        - On setup, checks for required arguments and node.
        - On update, sends a TTS request to the service and waits for completion.
        - Sets status to SUCCESS if the TTS service call succeeds, FAILURE otherwise.
        - If the task cannot be executed, returns a LogTask with an error message as opposite behavior.
    """

    def __init__(self, args):
        super().__init__(args, 'tts')
        self.node = None

    def setup(self, **kwargs):
        if kwargs['node'] is None:
            raise Exception("(TTSTask::setup) Cannot find 'node' argument")
        
        if self.args is None:
            raise Exception("(TTSTask::setup) Cannot find 'args' argument")
        
        if 'speech' not in self.args.keys():
            raise Exception("(TTSTask::setup) Malformed arguments, check that 'speech' variable is set correctly" \
            f" Arguments type: {type(self.args)} JSON argument: {json.JSONEncoder().encode(self.args)}")
        self.node : Node = kwargs['node']
        self.status = Status.RUNNING

    def _finished_service_callback(self, future):
        res = future.result()
        if res.error == True:
            self.node.get_logger().error(f"(TTSTask::update) Error: {res.error_message}")
            self.status = Status.FAILURE
        else:
            self.node.get_logger().debug("(TTSTask::update) TTS service finished successfully")
            self.status = Status.SUCCESS

    def _send_tts_request(self):
        if not self.node.tts_client.wait_for_service(timeout_sec=3.0):
            self.node.get_logger().error("(TTSTask::send_tts_request) TTS service not available")
            self.status = Status.FAILURE
        
        self.node.get_logger().info("(TTSTask::send_tts_request) Sending TTS request...")
        f = self.node.tts_client.call_async(TTSService.Request(text=self.args['speech']))
        f.add_done_callback(self._finished_service_callback)
        return f

    def update(self):
        if not hasattr(self, '_future'):
            self._future = self._send_tts_request()

        rclpy.spin_once(self.node, timeout_sec=0)

        if 'id' in self.args.keys():
            self.node.log_tasks(self.args['id'], self.status)

        return self.status
    
    def opposite_behavior(self):
        return LogTask({"tag": "error", "msg": "The TTS task cannot be executed."})

class LogTask(Task):

    def __init__(self, args):
        super().__init__(args, 'log')

    def setup(self, **kwargs):
        if kwargs['node'] is None:
            raise Exception("(LogTask::setup) Cannot find 'node' argument")
        
        if 'msg' in self.args.keys() and self.args['msg'] is None:
            raise Exception("(LogTask::setup) Cannot find 'msg' argument")
        
        if 'tag' in self.args.keys() and self.args['tag'] is not None:
            self.tag = self.args['tag']
        else:
            self.tag = "info"

        self.logger = kwargs['node'].get_logger()
        self.msg = self.args['msg']
        
    def update(self):
        msg = f'(LogTask::update) {self.msg}'
        if self.tag == "error":
            self.logger.error(msg)
        elif self.tag == "warn":
            self.logger.warn(msg)
        elif self.tag == "debug":
            self.logger.debug(msg)
        else:
            self.logger.info(msg)
        return Status.SUCCESS          
        
