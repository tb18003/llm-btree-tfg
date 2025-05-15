import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
import json

from PyQt5.QtWidgets import QApplication

from py_trees.trees import BehaviourTree
from py_trees.composites import Sequence, Selector
from py_trees.common import Status

# Tree rendering
from py_trees.display import ascii_tree

from .task_model import TaskFactory, Task
from .gui.task_executor_gui import TaskExecutorGUI 
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from nav2_msgs.action import NavigateToPose

# Tasks
from task_sim.tasks_implementation import MoveTask, SuccessTask, FailureTask, TTSTask, LogTask # type: ignore
import random

from threading import Thread
from time import sleep

class TaskExecutorNode(Node):

    _executingTree : None | BehaviourTree = None

    def __init__(self, actions: dict[str, Task]):
        super().__init__('task_executor_node')

        task_topic_param = self.declare_parameter("TASKS_TOPIC","/task/input")
        task_info_param = self.declare_parameter("TASKS_INFO","/task/info")
        self.gui_sender_param = self.declare_parameter("GUI_SENDER", True)

        self.task_topic = self.create_subscription(String, task_topic_param.value, self.receive_tasks, 10)
        self.task_logger = self.create_publisher(String, task_info_param.value, 10)
        
        # GUI Publisher
        if self.gui_sender_param.value:
            self.task_sender = self.create_publisher(String, task_topic_param.value, 10)

        self.cb_group = ReentrantCallbackGroup()

        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose', callback_group=self.cb_group)

        self.actions = actions
        self.get_logger().info("Task Executor node started!")

    def send_goal_nav2(self, msg, callback):
        self.nav_client.wait_for_server(timeout_sec=3.0)
        self.get_logger().info("Sending goal...")
        future = self.nav_client.send_goal_async(msg)
        future.add_done_callback(callback)
        return future

    def goal_callback_example(self, future):
        res: ClientGoalHandle = future.result()
        if not res.accepted:
            self.get_logger().error("NAV2 rejected btw")
        else:
            self.get_logger().info("Goal accepted!")
            res.get_result_async().add_done_callback(self.result_callback_example)

    def result_callback_example(self, future):
        res: NavigateToPose.Result = future.result()
        self.get_logger().info("Journey completed! [%s]" % res.result)


    def task_sender_func(self, message):
        self.task_sender.publish(String(data=message))
    
    def receive_tasks(self, msg: String):
        if self._executingTree is not None:
            return

        try:
            tasks = TaskFactory.parse_json_tasks(self.actions, msg.data)
            self.tasks: list[dict] = []
            
            self.get_logger().debug("-- Showing received tasks")
            root = Sequence('root-sequence', False)

            self.id = 0

            for task in tasks:
                self.get_logger().debug(f"\tTask: {task.name} & Arguments: {json.JSONEncoder().encode(task.args)}")

                name = task.name

                if type(task) == TTSTask:
                    name = 'talk'

                self.tasks.append({
                    "task": name,
                    "args": task.args,
                    "status": "pending"
                })

                task.args['id'] = len(self.tasks) - 1

                root.add_child(self.opposite_task_rec(task))

            self._executingTree = BehaviourTree(root=root)
            self._executingTree.setup(node=self)

            self.get_logger().debug("-- Starting execution...")
            self.log_tasks(0,-1)
            self._executingTree.tick_tock(
                period_ms=500,
                post_tick_handler=self.finished_tree_tick,
                stop_on_terminal_state=True,
            )

            self._executingTree = None

        except json.JSONDecodeError:
            self.get_logger().error("Malformed JSON, cannot execute task list. Received message:\n %s" % msg.data)
        except Exception as e:
            self.get_logger().error(e)

    def log_tasks(self, id, status):
        task = self.tasks[id]

        last_status = task['status']

        if status == Status.RUNNING:
            task['status'] = 'running'
        elif status == Status.FAILURE:
            task['status'] = 'failure'
        elif status == Status.SUCCESS:
            task['status'] = 'done'
        else:
            task['status'] = 'pending'

        if last_status != task['status']:
            self.task_logger.publish(String(data=json.JSONEncoder().encode(self.tasks)))

    def opposite_task_rec(self, task):
        opposite = task.opposite_behavior()
        if opposite is None:
            return task
        
        sel = Selector(f"task_{self.id}", False)
        self.id += 1

        sel.add_child(task)
        sel.add_child(self.opposite_task_rec(opposite))

        return sel
    
    def finished_tree_tick(self, tree):
        if tree.root.status == Status.SUCCESS:
            self.get_logger().info(f"Complete behavior tree:\n{ascii_tree(self._executingTree.root, show_status=True)}")


        elif tree.root.status == Status.FAILURE:
            self.get_logger().error("Couldn't execute the behavior tree, a task failed")
            self.get_logger().info(f"Complete behavior tree:\n{ascii_tree(self._executingTree.root, show_status=True)}")

        

def rclpy_callback_spin(node):

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)


def main(args=[]):
    rclpy.init(args=args)

    node = TaskExecutorNode({
        'move': MoveTask,
        'true': SuccessTask,
        'false': FailureTask,
        'talk': TTSTask,
        'log': LogTask
    })

    
    if node.gui_sender_param.value:
        app = QApplication(args)
        gui = TaskExecutorGUI(node.task_sender_func)
        thread = Thread(target=rclpy_callback_spin, args=(node,))
        thread.start()
        gui.show()
        app.exec_()
    else:
        rclpy.spin(node=node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()