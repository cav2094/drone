import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
import socket
import json

from enum import Enum


class PID:
    """
    A simple PID controller for one axis.

    Kp — how hard to react to current error
    Ki — how hard to react to accumulated error over time (fixes drift)
    Kd — how hard to brake when closing in fast (prevents overshoot)
    """

    def __init__(self, Kp, Ki, Kd, max_output=1.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.max_output = max_output

        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        # Accumulate error over time (I term)
        self.integral += error * dt

        # Rate of change of error (D term)
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error

        # PID formula
        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

        # Clamp output so the drone doesn't get sent crazy commands
        return max(-self.max_output, min(self.max_output, output))

    def reset(self):
        """ Call this when switching states so stale integral doesn't cause a jerk """
        self.integral = 0.0
        self.prev_error = 0.0


class State(Enum):
    STANDBY = 0
    SCAN    = 1
    TRACK   = 2
    ENGAGE  = 3


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # 1. Listen for the target position from vision node
        self.subscription = self.create_subscription(Point, 'target_position', self.intruder_callback, 10)

        # 2. Publisher to command the drone to move
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # 3. Heartbeat timer (runs at 10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # 4. UDP socket to broadcast telemetry to the Mac host
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.host_address = ('host.docker.internal', 9000)

        # Goal variables (center of a 324x244 camera feed)
        self.center_x   = 162.0
        self.center_y   = 122.0
        self.target_area = 15000.0  # area (px²) we want to maintain — controls distance

        # PID controllers — one per axis
        # Tune Kp, Ki, Kd here after flight testing
        #                     Kp      Ki      Kd      max_output
        self.pid_yaw     = PID(0.005,  0.0001, 0.002,  0.5)   # left/right
        self.pid_z       = PID(0.005,  0.0001, 0.002,  0.5)   # up/down
        self.pid_forward = PID(0.00005, 0.000001, 0.00002, 0.3)  # forward/back

        # Tracking state
        self.state           = State.STANDBY
        self.latest_target   = None
        self.last_target_time = self.get_clock().now()
        self.last_time        = self.get_clock().now()

    def intruder_callback(self, msg):
        self.latest_target    = msg
        self.last_target_time = self.get_clock().now()

    def timer_callback(self):
        cmd_msg = Twist()

        # Calculate dt (time since last tick) for the PID derivative/integral terms
        now = self.get_clock().now()
        dt  = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # 1. SAFETY WATCHDOG — if target lost for 1.5s, go back to scanning
        time_since_target = (now - self.last_target_time).nanoseconds / 1e9
        if time_since_target > 1.5:
            if self.state == State.TRACK:
                self.get_logger().info("Target lost. Switching to SCAN.")
                self.pid_yaw.reset()
                self.pid_z.reset()
                self.pid_forward.reset()
            self.state         = State.SCAN
            self.latest_target = None

        # 2. STATE MACHINE
        if self.state == State.STANDBY:
            # Hover and wait for first detection
            if self.latest_target is not None:
                self.state = State.TRACK
                self.get_logger().info("Target acquired! Switching to TRACK.")

        elif self.state == State.SCAN:
            # Slowly spin to search for a target
            cmd_msg.angular.z = 0.5
            if self.latest_target is not None:
                self.state = State.TRACK
                self.get_logger().info("Target acquired! Switching to TRACK.")

        elif self.state == State.TRACK:
            if self.latest_target is not None:
                # Calculate how far off we are on each axis
                error_x    = self.center_x    - self.latest_target.x
                error_y    = self.center_y    - self.latest_target.y
                error_area = self.target_area - self.latest_target.z

                # Run each error through its PID controller
                cmd_msg.angular.z = self.pid_yaw.compute(error_x, dt)      # yaw left/right
                cmd_msg.linear.z  = self.pid_z.compute(error_y, dt)        # fly up/down
                cmd_msg.linear.x  = self.pid_forward.compute(error_area, dt)  # fly forward/back

        # 3. BROADCAST TELEMETRY TO MAC HOST
        try:
            telemetry = {
                "state": self.state.name,
                "vx":    float(cmd_msg.linear.x),
                "vy":    float(cmd_msg.linear.y),
                "vz":    float(cmd_msg.linear.z),
                "yaw":   float(cmd_msg.angular.z),
            }
            self.udp_sock.sendto(json.dumps(telemetry).encode('utf-8'), self.host_address)
        except Exception as e:
            self.get_logger().error(f"UDP failed: {e}")

        # 4. PUBLISH MOVEMENT COMMAND TO DRONE
        self.cmd_publisher.publish(cmd_msg)

        # 5. LOGGING
        if self.state == State.TRACK:
            self.get_logger().info(
                f"TRACKING -> Yaw: {cmd_msg.angular.z:.3f} | "
                f"Up/Down: {cmd_msg.linear.z:.3f} | "
                f"Forward: {cmd_msg.linear.x:.3f}"
            )
        elif self.state == State.SCAN:
            self.get_logger().info(f"SCANNING -> Yaw: {cmd_msg.angular.z:.3f}")


def main(args=None):
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
