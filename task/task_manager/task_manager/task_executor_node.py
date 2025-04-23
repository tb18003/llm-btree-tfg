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

# Tasks
from task_sim.tasks_implementation import MoveTask, SuccessTask, FailureTask, TTSTask, LogTask
import random

from threading import Thread

class TaskExecutorNode(Node):

    _executingTree : None | BehaviourTree = None

    def __init__(self, actions: dict[str, Task]):
        super().__init__('task_executor_node')

        task_topic_param = self.declare_parameter("TASKS_TOPIC","/task/input")
        self.gui_sender_param = self.declare_parameter("GUI_SENDER", False)

        self.task_topic = self.create_subscription(String, task_topic_param.value, self.receive_tasks, 10)
        # GUI Publisher
        if self.gui_sender_param.value:
            self.task_sender = self.create_publisher(String, task_topic_param.value, 10)

        self.actions = actions
        self.get_logger().info("Task Executor node started!")

    def task_sender_func(self, message):
        self.task_sender.publish(String(data=message))
    
    def receive_tasks(self, msg: String):
        if self._executingTree is not None:
            return

        try:
            tasks = TaskFactory.parse_json_tasks(self.actions, msg.data)
            
            self.get_logger().debug("-- Showing received tasks")
            root = Sequence('root-sequence', False)

            for task in tasks:
                self.get_logger().debug(f"\tTask: {task.name} & Arguments: {json.JSONEncoder().encode(task.args)}")

                root.add_child(self.opposite_task_rec(task))

            self._executingTree = BehaviourTree(root=root)
            self._executingTree.setup(node=self)

            self.get_logger().debug("-- Starting execution...")
            self._executingTree.tick(
                post_tick_handler=self.finished_tree_state
            )
        except json.JSONDecodeError:
            self.get_logger().error("Malformed JSON, cannot execute task list.")
        except Exception as e:
            self.get_logger().error(e)

    def opposite_task_rec(self, task):
        opposite = task.opposite_behavior()
        if opposite is None:
            return task
        
        sel = Selector(f"subtask{int(random.random()*1000)}", False)
        sel.add_child(task)
        sel.add_child(self.opposite_task_rec(opposite))
        return sel
    
    def finished_tree_state(self, tree):
        if tree.root.status == Status.SUCCESS:
            self.get_logger().info(f"Complete behavior tree:\n{ascii_tree(self._executingTree.root, show_status=True)}")
            self._executingTree = None

        elif tree.root.status == Status.FAILURE:
            self.get_logger().error("Couldn't execute the behavior tree, a task failed")
            self.get_logger().info(f"Complete behavior tree:\n{ascii_tree(self._executingTree.root, show_status=True)}")

            self._executingTree = None

        else:
            self._executingTree.tick(post_tick_handler=self.finished_tree_state)

def main(args=[]):
    rclpy.init(args=args)

    node = TaskExecutorNode({
        'move': MoveTask,
        'true': SuccessTask,
        'false': FailureTask,
        'talk': TTSTask,
        'log': LogTask
    })

    executor = MultiThreadedExecutor()

    executor.add_node(node)

    thread = Thread(target=executor.spin)
    thread.start()

    if node.gui_sender_param.value:
        app = QApplication(args)
        gui = TaskExecutorGUI(node.task_sender_func)
        gui.show()
        app.exec_()
    else:
        thread.join()

    node.destroy_node()

    executor.shutdown()

if __name__ == "__main__":
    main()