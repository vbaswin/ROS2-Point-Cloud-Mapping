#!/usr/bin/env python3
"""
Vim-style teleop for differential drive robots.

Keybindings (vim-motion inspired):
    k = forward      j = backward
    h = turn left     l = turn right
    q = quit

Concept: Raw Terminal Input (No GUI Required)
    We use Python's `termios` module to put the terminal in "raw" mode,
    where each keypress is delivered immediately (no Enter needed).
    This is how all terminal-based teleop nodes work in ROS2.

    Normal terminal mode:  User types "hello" → presses Enter → program gets "hello"
    Raw terminal mode:     User presses 'h' → program gets 'h' immediately

Concept: Publishing Twist Messages
    This node publishes geometry_msgs/Twist at a fixed rate.
    The diff_drive plugin converts Twist → wheel velocities:
        left_wheel  = linear.x - angular.z × (wheel_sep / 2)
        right_wheel = linear.x + angular.z × (wheel_sep / 2)
"""

import sys
import termios
import tty
from typing import Final

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# Movement configuration — const by default (Strictness Mandate)
LINEAR_SPEED: Final[float] = 0.5  # m/s forward/backward
ANGULAR_SPEED: Final[float] = 2.0  # rad/s turning

# Keybindings — vim-style hjkl navigation
KEYMAP: Final[dict[str, tuple[float, float]]] = {
    #  key: (linear.x, angular.z)
    "k": (LINEAR_SPEED, 0.0),  # k = forward  (vim: up)
    "j": (-LINEAR_SPEED, 0.0),  # j = backward (vim: down)
    "h": (0.0, ANGULAR_SPEED),  # h = turn left  (vim: left)
    "l": (0.0, -ANGULAR_SPEED),  # l = turn right (vim: right)
}

HELP_TEXT: str = """
╔═══════════════════════════════════════╗
║     Vim-Style Teleop Controller       ║
╠═══════════════════════════════════════╣
║                                       ║
║            k (forward)                ║
║               ↑                       ║
║    h (left) ←   → l (right)          ║
║               ↓                       ║
║            j (backward)               ║
║                                       ║
║    Speed: {linear:.1f} m/s  |  Turn: {angular:.1f} rad/s   ║
║    Press 'q' to quit                  ║
╚═══════════════════════════════════════╝
""".format(linear=LINEAR_SPEED, angular=ANGULAR_SPEED)


def get_key() -> str:
    """Read a single keypress from terminal in raw mode.

    Concept: termios / tty
        Unix terminals buffer input by line (canonical mode).
        tty.setraw() switches to raw mode where each byte is
        delivered immediately. We restore the original settings
        after reading to prevent terminal corruption.
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        # ALWAYS restore — even on exception. This is RAII-like
        # cleanup using try/finally (Python's equivalent of C++ destructors).
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


class VimTeleop(Node):
    """ROS2 node that reads vim-style keys and publishes Twist.

    Concept: Separation of Concerns
        This node ONLY translates keypresses → Twist messages.
        It knows nothing about wheels, robots, or physics.
        The diff_drive plugin handles the rest.
        This is Dependency Inversion — we depend on the Twist
        abstraction, not the concrete robot hardware.
    """

    def __init__(self) -> None:
        super().__init__("vim_teleop")

        # Publish to /camera_robot/cmd_vel (matching the diff_drive namespace in SDF)
        self.publisher = self.create_publisher(Twist, "/camera_robot/cmd_vel", 10)
        self.get_logger().info(
            "Vim teleop started. Publishing to /camera_robot/cmd_vel"
        )

    def publish_twist(self, linear_x: float, angular_z: float) -> None:
        """Construct and publish a Twist message."""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher.publish(msg)

    def stop(self) -> None:
        """Publish zero velocity (emergency stop)."""
        self.publish_twist(0.0, 0.0)


def main() -> None:
    rclpy.init()
    node = VimTeleop()

    print(HELP_TEXT)

    try:
        while True:
            key = get_key()

            if key == "q" or key == "\x03":  # 'q' or Ctrl+C
                print("\nStopping robot...")
                node.stop()
                break

            if key in KEYMAP:
                linear_x, angular_z = KEYMAP[key]
                node.publish_twist(linear_x, angular_z)
                # Visual feedback: show which direction
                direction = {
                    "k": "↑ Forward",
                    "j": "↓ Backward",
                    "h": "← Turn Left",
                    "l": "→ Turn Right",
                }
                print(f"\r  {direction[key]}      ", end="", flush=True)
            else:
                # Any other key = stop (safety: release key = stop moving)
                node.stop()
                print(f"\r  ■ Stop            ", end="", flush=True)

    except KeyboardInterrupt:
        pass
    finally:
        # Always stop the robot on exit — safety first
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        print("\nTeleop shutdown complete.")


if __name__ == "__main__":
    main()
