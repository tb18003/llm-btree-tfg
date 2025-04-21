from rclpy.node import Node
from py_trees.common import Status
from std_msgs.msg import String
from geometry_msgs.msg import Point32
from task_manager.task_model import Task
from py_trees.composites import Sequence
import json

class MoveTask(Task):

    def __init__(self, args):
        super().__init__(args, 'move')
        self.node = None
        self.sent = False

    def setup(self, **kwargs):
        if kwargs['node'] is None:
            raise Exception("(MoveTask::setup) Cannot find 'node' argument")
        
        if self.args is None:
            raise Exception("(MoveTask::setup) Cannot find 'args' argument")
        
        if 'x' not in self.args.keys() or \
           'y' not in self.args.keys():
            raise Exception("(MoveTask::setup) Malformed arguments, check that 'x', 'y' and 'z' variables are set correctly" \
            f" Arguments type: {type(self.args)} JSON argument: {json.JSONEncoder().encode(self.args)}")
        
        if 'z' not in self.args.keys():
            self.args['z'] = 0.0
            
        self.node : Node = kwargs['node']
        self.pub = self.node.create_publisher(
            Point32,
            '/robot/move',
            10
        )
    
    def update(self):
        if self.sent == False:
            try:
                if self.args['z'] != 0:
                    return Status.FAILURE
                self.pub.publish(Point32(x=self.args['x'],y=self.args['y'],z=self.args['z']))
                self.node.get_logger().info("(MoveTask::update) Sending data...")
                self.sent = True
                return Status.SUCCESS
            except Exception:
                return Status.FAILURE
    
    def opposite_behavior(self):
        return TTSTask({"speech": "Sorry, I cannot move to that position."})

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
        
