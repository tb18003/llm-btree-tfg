from rclpy.node import Node
from py_trees.common import Status
from std_msgs.msg import String
from geometry_msgs.msg import Point32
from task_manager.task_model import Task # type: ignore
from nav2_msgs.action import NavigateToPose
#from task_manager.task_executor_node import TaskExecutorNode
import json
import rclpy

class MoveTask(Task):

    def __init__(self, args):
        super().__init__(args, 'move')
        self.node = None

        self.status = Status.RUNNING
        self.opposite_task = TTSTask({"speech": "Sorry, I cannot move to that position."})

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

    def send_goal_nav2(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = float(self.args['x'])
        goal_msg.pose.pose.position.y = float(self.args['y'])
        goal_msg.pose.pose.orientation.z = 1.0
        goal_msg.pose.pose.orientation.w = float(self.args['theta'])

        self.node.nav_client.wait_for_server(timeout_sec=3.0)
        f = self.node.nav_client.send_goal_async(goal_msg)
        f.add_done_callback(self.goal_callback)
        self.sent = True

    def goal_callback(self, future):
        res = future.result()

        if not res.accepted:
            self.node.get_logger().error("NAV2 rejected btw")
        else:
            self.node.get_logger().info("Goal accepted!")
            f = res.get_result_async()
            f.add_done_callback(self.result_callback)
            self._r_f = f
    
    def result_callback(self, future):
        res = future.result().result
        self.node.get_logger().info("Goal reached!")

        if not hasattr(res, 'error_code'):
            self.node.get_logger().info("Navegación completada con éxito")
            self.status = Status.SUCCESS
        else:
            error_messages = {
                1: "El robot no puede alcanzar el objetivo",
                2: "El robot está atascado",
                3: "El objetivo está fuera de los límites",
                4: "Tiempo excedido",
                # ... otros códigos de error
            }
            error_msg = error_messages.get(res.error_code, "Error desconocido")
            if self.args['x'] == -1 and self.args['y'] == 0:
                self.opposite_task.args = {"speech": error_msg}
            self.node.get_logger().error(f"Error en navegación: {error_msg} (Código: {res.error_code})")
            self.status = Status.FAILURE

    def update(self):
        if not hasattr(self, '_future'):
            self._future = self.send_goal_nav2()

        rclpy.spin_once(self.node, timeout_sec=0)

        return self.status
    
    def opposite_behavior(self):
        # if can't go to goal, it goes back to home
        if self.args['x'] != -1 and self.args['y'] != 0:
            return MoveTask({'x': -1, 'y': 0, 'theta': 1})
        
        return self.opposite_task

class TTSTask(Task):

    def __init__(self, args):
        super().__init__(args, 'tts')
        self.node = None
        self.sent = False

    def setup(self, **kwargs):
        if kwargs['node'] is None:
            raise Exception("(TTSTask::setup) Cannot find 'node' argument")
        
        if self.args is None:
            raise Exception("(TTSTask::setup) Cannot find 'args' argument")
        
        if 'speech' not in self.args.keys():
            raise Exception("(TTSTask::setup) Malformed arguments, check that 'speech' variable is set correctly" \
            f" Arguments type: {type(self.args)} JSON argument: {json.JSONEncoder().encode(self.args)}")
            
        self.node : Node = kwargs['node']
        self.pub = self.node.create_publisher(
            String,
            '/robot/tts',
            10
        )
    
    def update(self):
        if self.sent == False:
            try:
                self.pub.publish(String(data=self.args['speech']))
                self.node.get_logger().info("(TTSTask::update) Sending data...")
                self.sent = True
                return Status.SUCCESS
            except Exception:
                return Status.FAILURE
    
    def opposite_behavior(self):
        return LogTask({"tag": "error", "msg": "The TTS task cannot be executed."})

class SuccessTask(Task):

    def __init__(self, args):
        super().__init__(args, 'success')

    def setup(self, **kwargs):
        if kwargs['node'] is None:
            raise Exception("(SuccessTask::setup) Cannot find 'node' argument")

        self.logger = kwargs['node'].get_logger()
        
    def update(self):
        self.logger.info("(SuccessTask::update) Task done succesfully")
        return Status.SUCCESS

class FailureTask(Task):

    def __init__(self, args):
        super().__init__(args, 'failure')

    def setup(self, **kwargs):
        if kwargs['node'] is None:
            raise Exception("(FailureTask::setup) Cannot find 'node' argument")

        self.logger = kwargs['node'].get_logger()
        
    def update(self):
        self.logger.info("(FailureTask::update) Task failured succesfully")
        return Status.FAILURE

    def opposite_behavior(self):
        return LogTask({"msg": "Task failed successfully :)"})
        
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
        
