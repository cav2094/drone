#!/usr/bin/env python3

# --- IMPORTS ---
# Same as sensor.py — you need rclpy, Node, and Float32.
# No need for `random` here though — this node only RECEIVES data.
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32



# --- CLASS ---
# Create a class called ControllerNode that inherits from Node.
class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.subscription = self.create_subscription(Float32, '/obstacle_distance', self.distance_callback, 10)


    def distance_callback(self, msg):
        distance = msg.data
        if distance < 0.5:
            self.get_logger().info('Emergency stop!')
        elif distance < 1.5:
            self.get_logger().info('Slow down!')
        elif distance < 3.0:
            self.get_logger().info('Caution!')
        else:
            self.get_logger().info('Full speed ahead!')


    # --- __init__ ---
    # Set up the node. You need to do 2 things here:
    #
    #   1. Call the parent constructor with a node name string.
    #
    #   2. Create a subscriber using self.create_subscription()
    #      It needs: (message type, topic name, callback method, queue size)
    #      Topic name must EXACTLY match sensor.py's topic: '/obstacle_distance'
    #      The callback is a method you'll write below — pass it without calling it
    #      (i.e., self.my_method not self.my_method())
    #
    # Notice: NO timer here. That's the key difference.
    # This node is REACTIVE — it only does something when data arrives.


    # --- distance_callback ---
    # ROS 2 will call this automatically whenever a new message arrives.
    # It receives `msg` as a parameter — a Float32 object.
    # Access the actual number with msg.data
    #
    # You need to:
    #   1. Extract the distance from msg.data
    #
    #   2. Write an if/elif/else block that decides what the robot should do:
    #      - Less than 0.5m  → emergency stop
    #      - Less than 1.5m  → slow down
    #      - Less than 3.0m  → caution
    #      - Otherwise       → full speed ahead
    #
    #   3. Log the distance and the decision with self.get_logger().info()


def main(args=None):
    rclpy.init(args=args)
    noder = ControllerNode()
    rclpy.spin(noder)
    noder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
