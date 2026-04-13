import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
import socket
import json

from enum import Enum                                                         
                                                                                
class State(Enum):
    STANDBY = 0
    SCAN = 1
    TRACK = 2
    ENGAGE = 3

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        # 1. Listen for the face position
        self.subscription = self.create_subscription(Point, 'face_position', self.intruder_callback, 10)

        # 2. Add the publisher to command the drone to move
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # 3. Setup the heartbeat
        self.timer = self.create_timer(0.1, self.timer_callback)

        # 4. Setup Python UDP Socket (The "Escape Hatch" to the Mac Host)
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # We tell the container to send these to the Mac host on port 9000
        self.host_address = ('host.docker.internal', 9000)

        # Initial Tracking Variables
        self.state = State.STANDBY
        self.latest_target = None
        self.last_target_time = self.get_clock().now()

        # Goal variables (Center of a 324x244 camera feed)
        self.center_x = 162.0
        self.center_y = 122.0
        self.target_area = 15000.0  # The arbitrary distance we want to maintain

    def intruder_callback(self, msg):
        self.latest_target = msg
        self.last_target_time = self.get_clock().now() # Record the exact time we saw it
        
    def timer_callback(self):
        # Create an empty movement command (all zeros = hover in place)
        cmd_msg = Twist()
        
        # 1. TIMEOUT/SAFETY WATCHDOG
        time_since_target = (self.get_clock().now() - self.last_target_time).nanoseconds / 1e9
        if time_since_target > 1.5:
            # If we haven't seen a face for 1.5 seconds, stop tracking so we don't crash
            self.state = State.SCAN
            self.latest_target = None

        # 2. BEHAVIOR BASED ON CURRENT STATE
        if self.state == State.STANDBY:
            # Do nothing, hover. Wait for target.
            if self.latest_target is not None:
                self.state = State.TRACK
                self.get_logger().info("Intruder detected! Switching to TRACK.")

        elif self.state == State.SCAN:
            # Slowly spin around to look for a face
            cmd_msg.angular.z = 0.5  # Positive yaw spins slowly to the left
            if self.latest_target is not None:
                self.state = State.TRACK
                self.get_logger().info("Found a face! Switching to TRACK.")

        elif self.state == State.TRACK:
            if self.latest_target is not None:
                # CALCULATE ERRORS (How far are we from perfect?)
                error_x = self.center_x - self.latest_target.x
                error_y = self.center_y - self.latest_target.y
                error_area = self.target_area - self.latest_target.z
                
                # TURN ERRORS INTO MOVEMENT 
                # (We multiply by very small numbers to keep movements gentle)
                
                # Yaw left/right to center face horizonally
                cmd_msg.angular.z = error_x * 0.005
                
                # Fly up/down to center face vertically
                cmd_msg.linear.z = error_y * 0.005
                
                # Fly forward/backward to maintain distance (area)
                cmd_msg.linear.x = error_area * 0.00005

        # --- NEW CODE: Broadcast to Mac Host ---
        try:
            telemetry = {
                "vx": float(cmd_msg.linear.x),
                "vy": float(cmd_msg.linear.y),
                "vz": float(cmd_msg.linear.z),
                "yaw": float(cmd_msg.angular.z)
            }
            json_data = json.dumps(telemetry).encode('utf-8')
            self.udp_sock.sendto(json_data, self.host_address)
        except Exception as e:
            self.get_logger().error(f"UDP failed: {e}")

        # 3. PUBLISH THE MOVEMENT COMMAND
        self.cmd_publisher.publish(cmd_msg)
        
        # PROOF OF CONCEPT LOGGING: 
        # Print the virtual stick movements to the terminal so we can see it's working!
        if self.state == State.TRACK:
            self.get_logger().info(f"TRACKING -> Yaw: {cmd_msg.angular.z:.3f} | Up/Down: {cmd_msg.linear.z:.3f} | Forward: {cmd_msg.linear.x:.3f}")
        elif self.state == State.SCAN:
            self.get_logger().info(f"SCANNING -> Yaw: {cmd_msg.angular.z:.3f}")
            
def main(args=None):
    # This boilerplate actually starts the ROS 2 node
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
