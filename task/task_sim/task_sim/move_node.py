import rclpy
from rclpy.node import Node
from py_trees.behaviour import Behaviour
from geometry_msgs.msg import Point32

class MoveNode(Node):

    def __init__(self):
        super().__init__('move_node_sim')
        self.topic_sub = self.create_subscription(
            Point32,
            '/robot/move',
            self.move_callback,
            10
        )

        self.get_logger().info("Move Simulator node started!")

        self.location = Point32()
    
    def move_callback(self, msg: Point32):
        self.location = msg
        self.get_logger().info(f'New position: ({msg.x}, {msg.y}, {msg.z})')


def main(args=[]):
    rclpy.init()

    node = MoveNode()

    rclpy.spin(node=node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()