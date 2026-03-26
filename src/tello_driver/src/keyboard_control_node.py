#!/usr/bin/env python3
"""Keyboard control node for Tello drone.

This node provides incremental velocity control for the Tello drone using a
keyboard. Each key press adjusts a velocity component, while automatic decay
slows the drone when no keys are pressed. The node publishes to /tello/cmd_vel
and can trigger takeoff/land commands.

The design uses a class-based ROS node with a separate keyboard listener thread
and a publish loop running at 10 Hz.
"""

import threading
import time
import sys

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

try:
    from pynput import keyboard
except ImportError:
    raise ImportError(
        "pynput is required for keyboard_control_node.py. "
        "Install with `pip install pynput` or add it to your environment."
    )


class KeyboardControlNode:
    """ROS node to control Tello using keyboard input."""

    def __init__(self):
        rospy.init_node('keyboard_control_node', anonymous=False)

        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=10)
        self.takeoff_pub = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('/tello/land', Empty, queue_size=1)

        # Velocity state (m/s and rad/s)
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw_rate = 0.0

        # Mode scale
        self.scale = 1.0
        self.scale_name = 'normal'

        # Control tuning
        self.increment = 0.1
        self.linear_limit = 0.3
        self.angular_limit = 1.0
        self.decay_factor = 0.95
        self.deadzone = 0.02

        # Watchdog
        self.last_input_time = time.time()
        self.landed_due_to_timeout = False

        # Thread safety
        self.lock = threading.Lock()

        # Keyboard listener
        self.listener = keyboard.Listener(on_press=self._on_key_press)
        self.listener.daemon = True
        self.listener.start()

        rospy.loginfo('KeyboardControlNode initialized (press T/L for takeoff/land)')
        self._print_help()

    def _clamp(self, value, min_val, max_val):
        return max(min_val, min(max_val, value))

    def _apply_deadzone(self, value):
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def _set_scale(self, scale, name):
        with self.lock:
            self.scale = scale
            self.scale_name = name

    def _update_velocity(self, dvx=0.0, dvy=0.0, dvz=0.0, dyaw=0.0):
        with self.lock:
            self.vx += dvx * self.scale
            self.vy += dvy * self.scale
            self.vz += dvz * self.scale
            self.yaw_rate += dyaw * self.scale

            # Clamp velocities
            self.vx = self._clamp(self.vx, -self.linear_limit, self.linear_limit)
            self.vy = self._clamp(self.vy, -self.linear_limit, self.linear_limit)
            self.vz = self._clamp(self.vz, -self.linear_limit, self.linear_limit)
            self.yaw_rate = self._clamp(self.yaw_rate, -self.angular_limit, self.angular_limit)

            # Deadzone apply
            self.vx = self._apply_deadzone(self.vx)
            self.vy = self._apply_deadzone(self.vy)
            self.vz = self._apply_deadzone(self.vz)
            self.yaw_rate = self._apply_deadzone(self.yaw_rate)

    def _decay_velocities(self):
        with self.lock:
            self.vx *= self.decay_factor
            self.vy *= self.decay_factor
            self.vz *= self.decay_factor
            self.yaw_rate *= self.decay_factor

            self.vx = self._apply_deadzone(self.vx)
            self.vy = self._apply_deadzone(self.vy)
            self.vz = self._apply_deadzone(self.vz)
            self.yaw_rate = self._apply_deadzone(self.yaw_rate)

    def _send_zero_twist(self, times=1):
        zero = Twist()
        for _ in range(times):
            self.cmd_vel_pub.publish(zero)
            rospy.sleep(0.01)

    def _on_key_press(self, key):
        """Handle keyboard key press events."""
        try:
            c = key.char.lower()
        except AttributeError:
            c = None

        # Update last input time
        self.last_input_time = time.time()
        self.landed_due_to_timeout = False

        if c == 'w':
            self._update_velocity(dvx=+self.increment)
        elif c == 's':
            self._update_velocity(dvx=-self.increment)
        elif c == 'a':
            self._update_velocity(dvy=+self.increment)
        elif c == 'd':
            self._update_velocity(dvy=-self.increment)
        elif c == 'r':
            self._update_velocity(dvz=+self.increment)
        elif c == 'f':
            self._update_velocity(dvz=-self.increment)
        elif c == 'q':
            self._update_velocity(dyaw=+self.increment)
        elif c == 'e':
            self._update_velocity(dyaw=-self.increment)
        elif c == 't':
            self.takeoff_pub.publish(Empty())
            rospy.loginfo('Takeoff command sent')
        elif c == 'l':
            self.land_pub.publish(Empty())
            rospy.loginfo('Land command sent')
        elif c == 'h':
            with self.lock:
                self.vx = self.vy = self.vz = self.yaw_rate = 0.0
            rospy.loginfo('Hover command: velocities zeroed')
        elif c == 'z':
            self._set_scale(0.5, 'slow')
            rospy.loginfo('Switched to SLOW mode (0.5x)')
        elif c == 'c':
            self._set_scale(1.5, 'fast')
            rospy.loginfo('Switched to FAST mode (1.5x)')
        elif c == ' ':  # space for emergency stop
            with self.lock:
                self.vx = self.vy = self.vz = self.yaw_rate = 0.0
            self._send_zero_twist(times=5)
            rospy.logwarn('Emergency STOP executed (SPACE pressed)')

    def _maybe_land_due_to_timeout(self, now):
        """Land automatically if no input for too long."""
        elapsed = now - self.last_input_time
        if elapsed > 5.0 and not self.landed_due_to_timeout:
            self.land_pub.publish(Empty())
            rospy.logwarn('No input for >3s, sending automatic land')
            self.landed_due_to_timeout = True

    def _print_help(self):
        """Print controls guide for the user."""
        help_lines = [
            "======================================",
            "Tello Keyboard Control (incremental)",
            "======================================",
            "Movement (press once to increment):",
            "  W/S : forward/back (vx)",
            "  A/D : left/right (vy)",
            "  R/F : up/down (vz)",
            "  Q/E : yaw left/right",
            "", 
            "Flight:",
            "  T : takeoff",
            "  L : land",
            "  H : hover (zero velocities)",
            "  SPACE : emergency stop (zero + send multiple zero commands)",
            "", 
            "Speed modes:",
            "  Z : slow (0.5x)",
            "  C : fast (1.5x)",
            "", 
            "Press Ctrl+C to quit.",
            "======================================",
        ]
        print("\n".join(help_lines))

    def _print_dashboard(self):
        """Print a live dashboard with current velocity and mode."""
        with self.lock:
            vx = self.vx
            vy = self.vy
            vz = self.vz
            yaw = self.yaw_rate
            mode = self.scale_name.upper()

        sys.stdout.write("\x1b[2J\x1b[H")  # clear screen + move cursor home
        sys.stdout.write("Tello Keyboard Control Dashboard\n")
        sys.stdout.write("======================================\n")
        sys.stdout.write("Controls: W/S (fwd/back), A/D (left/right), R/F (up/down), Q/E (yaw)\n")
        sys.stdout.write("Flight: T (takeoff), L (land), H (hover), SPACE (EMERGENCY STOP)\n")
        sys.stdout.write("Modes: Z (slow), C (fast)\n")
        sys.stdout.write("--------------------------------------\n")
        sys.stdout.write(f"  vx   : {vx: .2f} m/s\n")
        sys.stdout.write(f"  vy   : {vy: .2f} m/s\n")
        sys.stdout.write(f"  vz   : {vz: .2f} m/s\n")
        sys.stdout.write(f"  yaw  : {yaw: .2f} rad/s\n")
        sys.stdout.write(f"  mode : {mode}\n")
        sys.stdout.write("======================================\n")
        sys.stdout.flush()

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz publish
        rospy.loginfo('KeyboardControlNode running (press T/L for takeoff/land)')

        while not rospy.is_shutdown():
            now = time.time()

            # Decay velocities when no key pressed (watchdog timeout)
            if now - self.last_input_time > 1.0:
                self._decay_velocities()

            # Watchdog land (long timeout)
            self._maybe_land_due_to_timeout(now)

            # Publish current velocities
            twist = Twist()
            with self.lock:
                twist.linear.x = self.vx
                twist.linear.y = self.vy
                twist.linear.z = self.vz
                twist.angular.z = self.yaw_rate

            self.cmd_vel_pub.publish(twist)
            self._print_dashboard()

            rate.sleep()

        rospy.loginfo('KeyboardControlNode shutting down')


def main():
    try:
        node = KeyboardControlNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo('Keyboard control node terminated')


if __name__ == '__main__':
    main()
