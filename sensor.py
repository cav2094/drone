#!/usr/bin/env python3

# --- IMPORTS ---
# You need 3 things:
#   1. The main ROS 2 Python library (hint: it's called rclpy)
#   2. The Node base class from rclpy.node
#   3. A message type for a single float number from std_msgs.msg
#   Bonus: import random — you'll use it to fake sensor readings
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random


# --- CLASS ---
# Create a class called SensorNode that inherits from Node
class SensorNode(Node):
    def __init__(self):
        #using parent class constrcuto, string = name
        super().__init__('sensor_node')

        #creating a publisher, message type, topic name, queue size
        self.publisher_ = self.create_publisher(Float32, '/obstacle_distance', 10)
        self.create_timer(1.0, self.publish_reading)


    def publish_reading(self):
       #create an empty message object
        msg = Float32()
        #set .data on it to a random float between 0.1 and 5.0
        msg.data = round(random.uniform(0.1, 5.0), 2)
        #publish it using self.publisher_.publish()
        self.publisher_.publish(msg)
        #log it with self.get_logger().info()
        self.get_logger().info(f'Broadcasting: {msg.data}')

        

def main(args=None):
    #start the ROS 2 communications layer
    rclpy.init(args=args)
    #create your node
    mynode = SensorNode()
    #hand control to ROS 2, let it run callbacks forever
    rclpy.spin(mynode)
    #cleanup
    mynode.destroy_node()
    rclpy.shutdown()    
    
    


# WHY inherit from Node?
# The Node class gives you all the ROS 2 tools (publishers, timers, loggers).
# Without it, your class is just a plain Python class with no ROS 2 powers.


    # --- __init__ ---
    # This runs once when the node starts. Use it to SET UP, not to DO work.
    # You need to do 3 things here:
    #
    #   1. Call the parent class constructor with super().__init__()
    #      Pass it a string — this is your node's name in the ROS 2 network.
    #
    #   2. Create a publisher using self.create_publisher()
    #      It needs: (message type, topic name as a string, queue size)
    #      Save it to self.publisher_
    #      Topic name: '/obstacle_distance'
    #      Queue size: 10 (just means "buffer up to 10 messages")
    #
    #   3. Create a timer using self.create_timer()
    #      It needs: (interval in seconds, the method to call each interval)
    #      Fire every 1.0 seconds, calling a method you'll write below.


    # --- publish_reading ---
    # This is the method the timer will call every second.
    # You need to:
    #
    #   1. Create an empty message object (hint: it's the type you imported)
    #
    #   2. Set .data on it to a random float between 0.1 and 5.0
    #      Use random.uniform() and round it to 2 decimal places
    #
    #   3. Publish it using self.publisher_.publish()
    #
    #   4. Log it with self.get_logger().info()
    #      Print something like: "Broadcasting distance: X.XX meters"


# --- main ---
# Every ROS 2 Python script has this same 4-step boilerplate:
#
#   1. rclpy.init()          — start the ROS 2 communications layer
#   2. Create your node      — instantiate your class
#   3. rclpy.spin(node)      — hand control to ROS 2, let it run callbacks forever
#   4. Cleanup               — destroy_node() then rclpy.shutdown()


# Standard Python entry point guard
if __name__ == '__main__':
    main()
