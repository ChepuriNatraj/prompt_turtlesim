import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import re
import math
import time


class PromptTurtle(Node):
    def __init__(self):
        super().__init__('prompt_turtle')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info(
            "PromptTurtle node started.\n"
            "Examples: 'forward 2', 'backward 1.5', 'left 90', 'right 45', "
            "'circle', 'stop', 'quit'"
        )

    def send_twist(self, linear: float = 0.0, angular: float = 0.0, duration: float = 0.0):
        """
        Publish a constant Twist for `duration` seconds.
        After that, automatically send a stop command.
        """
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)

        start = time.time()

        # If duration <= 0, publish once (useful for stop).
        if duration <= 0.0:
            self.pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.0)
        else:
            while rclpy.ok() and (time.time() - start < duration):
                self.pub.publish(twist)
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(0.03)  # ~33 Hz publish rate

        # Stop motion after duration
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub.publish(twist)

    def execute_command(self, cmd: str):
        """
        Parse a text command and execute motion.
        """
        cmd = cmd.strip().lower()
        if not cmd:
            return

        # Exit
        if cmd in ('quit', 'exit'):
            raise KeyboardInterrupt()

        # Hard stop
        if cmd == 'stop':
            self.get_logger().info("Stopping.")
            self.send_twist(0.0, 0.0, 0.0)
            return

        # Circle
        if cmd.startswith('circle'):
            self.get_logger().info("Drawing a circle.")
            # Tune duration to get a full circle in turtlesim
            self.send_twist(linear=1.0, angular=1.0, duration=6.0)
            return

        # Move forward/backward: "forward 2", "backward 1.5"
        m = re.match(r'(forward|backward)\s+([\d\.]+)', cmd)
        if m:
            direction, dist_str = m.groups()
            dist = float(dist_str)
            speed = 1.0  # units per second
            duration = dist / speed
            linear = speed if direction == 'forward' else -speed
            self.get_logger().info(f"Moving {direction} {dist} units.")
            self.send_twist(linear=linear, angular=0.0, duration=duration)
            return

        # Rotate left/right: "left 90", "right 45"
        m = re.match(r'(left|right)\s+([\d\.]+)', cmd)
        if m:
            direction, angle_str = m.groups()
            angle_deg = float(angle_str)
            angle_rad = math.radians(angle_deg)
            ang_speed = 1.0  # rad/s
            duration = angle_rad / ang_speed
            angular = ang_speed if direction == 'left' else -ang_speed
            self.get_logger().info(f"Turning {direction} {angle_deg} degrees.")
            self.send_twist(linear=0.0, angular=angular, duration=duration)
            return

        # If nothing matches
        self.get_logger().info(
            "Could not understand command.\n"
            "Examples: 'forward 2', 'backward 1.5', 'left 90', 'right 45', 'circle', 'stop'."
        )


def main():
    rclpy.init()
    node = PromptTurtle()
    try:
        while rclpy.ok():
            cmd = input("Enter command: ")
            node.execute_command(cmd)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
