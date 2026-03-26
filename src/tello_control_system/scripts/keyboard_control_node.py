#!/usr/bin/env python3
"""Keyboard teleoperation node for Tello (RMTT).

This node provides incremental velocity control via keyboard and publishes
velocity commands to /tello/cmd_vel. It also provides takeoff/land triggers.

The control scheme is incremental: each key press changes velocity by a fixed
step. When no keys are pressed, velocities decay gradually.
"""

import threading
import sys
import time

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
    """Keyboard teleoperation node for Tello."""

    def __init__(self):
        rospy.init_node('keyboard_control_node', anonymous=False)

        # Load parameters
        self.max_linear = rospy.get_param('~max_linear_speed', rospy.get_param('/max_linear_speed', 0.3))
        self.max_angular = rospy.get_param('~max_angular_speed', rospy.get_param('/max_angular_speed', 1.0))
        self.decay_rate = rospy.get_param('~decay_rate', rospy.get_param('/decay_rate', 0.95))
        self.step_size = rospy.get_param('~step_size', rospy.get_param('/step_size', 0.1))
        self.watchdog_timeout = rospy.get_param('~watchdog_timeout', rospy.get_param('/watchdog_timeout', 3.0))
        
        # Acceleration limits (new safety constraints)
        self.max_linear_accel = rospy.get_param('~max_linear_accel', 0.5)    # m/s^2
        self.max_angular_accel = rospy.get_param('~max_angular_accel', 2.0)  # rad/s^2
        self.control_rate_hz = 20  # control publishing rate

        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=10)
        self.takeoff_pub = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('/tello/land', Empty, queue_size=1)

        # State
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw_rate = 0.0
        self.mode = 'NORMAL'
        self.scale = 1.0
        
        # Previous velocities for acceleration limiting
        self.prev_vx = 0.0
        self.prev_vy = 0.0
        self.prev_vz = 0.0
        self.prev_yaw = 0.0

        # Watchdog
        self.last_input_time = time.time()
        self.landed_due_to_timeout = False

        # Thread lock
        self.lock = threading.Lock()

        # Keyboard listener
        self.listener = keyboard.Listener(on_press=self._on_key_press)
        self.listener.daemon = True
        self.listener.start()

        self._print_help()
        rospy.loginfo('KeyboardControlNode initialized (max_linear_accel=%.2f m/s^2, max_angular_accel=%.2f rad/s^2)' 
                      % (self.max_linear_accel, self.max_angular_accel))

    def _clamp(self, value, min_v, max_v):
        return max(min_v, min(max_v, value))

    def _deadzone(self, v):
        return 0.0 if abs(v) < 0.02 else v

    def _apply_accel_limit(self, desired_v, prev_v, max_accel, dt):
        """Limit velocity change to respect acceleration constraint."""
        max_dv = max_accel * dt
        dv = desired_v - prev_v
        if dv > max_dv:
            return prev_v + max_dv
        elif dv < -max_dv:
            return prev_v - max_dv
        return desired_v

    def _update_velocity(self, dvx=0.0, dvy=0.0, dvz=0.0, dyaw=0.0):
        with self.lock:
            self.vx += dvx * self.scale
            self.vy += dvy * self.scale
            self.vz += dvz * self.scale
            self.yaw_rate += dyaw * self.scale

            self.vx = self._deadzone(self._clamp(self.vx, -self.max_linear, self.max_linear))
            self.vy = self._deadzone(self._clamp(self.vy, -self.max_linear, self.max_linear))
            self.vz = self._deadzone(self._clamp(self.vz, -self.max_linear, self.max_linear))
            self.yaw_rate = self._deadzone(self._clamp(self.yaw_rate, -self.max_angular, self.max_angular))

    def _decay(self):
        with self.lock:
            self.vx *= self.decay_rate
            self.vy *= self.decay_rate
            self.vz *= self.decay_rate
            self.yaw_rate *= self.decay_rate

            self.vx = self._deadzone(self.vx)
            self.vy = self._deadzone(self.vy)
            self.vz = self._deadzone(self.vz)
            self.yaw_rate = self._deadzone(self.yaw_rate)

    def _send_zero_twist(self, times=5):
        zero = Twist()
        for _ in range(times):
            self.cmd_vel_pub.publish(zero)
            rospy.sleep(0.01)

    def _set_mode(self, scale, name):
        with self.lock:
            self.scale = scale
            self.mode = name

    def _print_help(self):
        help_lines = [
            "======================================",
            "Tello Keyboard Control (incremental)",
            "======================================",
            "Controls:",
            "  W/S : forward/back",
            "  A/D : left/right",
            "  R/F : up/down",
            "  Q/E : yaw left/right",
            "", 
            "Flight:",
            "  T : takeoff",
            "  L : land",
            "", 
            "Control:",
            "  H : hover (zero velocity)",
            "  SPACE : emergency stop",
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
        with self.lock:
            vx = self.vx
            vy = self.vy
            vz = self.vz
            yaw = self.yaw_rate
            mode = self.mode

        sys.stdout.write("\x1b[2J\x1b[H")
        sys.stdout.write("Tello Keyboard Control Dashboard\n")
        sys.stdout.write("======================================\n")
        sys.stdout.write("Controls: W/S A/D R/F Q/E  T/L  H  SPACE\n")
        sys.stdout.write("Modes: Z (slow)  C (fast)\n")
        sys.stdout.write("--------------------------------------\n")
        sys.stdout.write(f"  vx   : {vx: .2f} m/s\n")
        sys.stdout.write(f"  vy   : {vy: .2f} m/s\n")
        sys.stdout.write(f"  vz   : {vz: .2f} m/s\n")
        sys.stdout.write(f"  yaw  : {yaw: .2f} rad/s\n")
        sys.stdout.write(f"  mode : {mode}\n")
        sys.stdout.write("======================================\n")
        sys.stdout.flush()

    def _on_key_press(self, key):
        try:
            c = key.char.lower()
        except AttributeError:
            c = None

        self.last_input_time = time.time()
        self.landed_due_to_timeout = False

        if c == 'w':
            self._update_velocity(dvx=self.step_size)
        elif c == 's':
            self._update_velocity(dvx=-self.step_size)
        elif c == 'a':
            self._update_velocity(dvy=self.step_size)
        elif c == 'd':
            self._update_velocity(dvy=-self.step_size)
        elif c == 'r':
            self._update_velocity(dvz=self.step_size)
        elif c == 'f':
            self._update_velocity(dvz=-self.step_size)
        elif c == 'q':
            self._update_velocity(dyaw=self.step_size)
        elif c == 'e':
            self._update_velocity(dyaw=-self.step_size)
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
            self._set_mode(0.5, 'SLOW')
            rospy.loginfo('Switched to SLOW mode (0.5x)')
        elif c == 'c':
            self._set_mode(1.5, 'FAST')
            rospy.loginfo('Switched to FAST mode (1.5x)')
        elif c == ' ':  # emergency stop
            with self.lock:
                self.vx = self.vy = self.vz = self.yaw_rate = 0.0
            self._send_zero_twist(times=5)
            rospy.logwarn('Emergency STOP executed (SPACE pressed)')

    def _maybe_land_due_to_timeout(self, now):
        elapsed = now - self.last_input_time
        if elapsed > self.watchdog_timeout and not self.landed_due_to_timeout:
            self.land_pub.publish(Empty())
            rospy.logwarn('No input for > %.1fs, sending automatic land' % self.watchdog_timeout)
            self.landed_due_to_timeout = True

    def run(self):
        dt = 1.0 / self.control_rate_hz
        rate = rospy.Rate(self.control_rate_hz)  # Updated to configurable rate
        rospy.loginfo('KeyboardControlNode running at %d Hz', self.control_rate_hz)

        while not rospy.is_shutdown():
            now = time.time()
            if now - self.last_input_time > 0.5:
                self._decay()

            self._maybe_land_due_to_timeout(now)

            twist = Twist()
            with self.lock:
                # Apply acceleration limiting before publishing
                limited_vx = self._apply_accel_limit(self.vx, self.prev_vx, self.max_linear_accel, dt)
                limited_vy = self._apply_accel_limit(self.vy, self.prev_vy, self.max_linear_accel, dt)
                limited_vz = self._apply_accel_limit(self.vz, self.prev_vz, self.max_linear_accel, dt)
                limited_yaw = self._apply_accel_limit(self.yaw_rate, self.prev_yaw, self.max_angular_accel, dt)
                
                # Store for next iteration
                self.prev_vx = limited_vx
                self.prev_vy = limited_vy
                self.prev_vz = limited_vz
                self.prev_yaw = limited_yaw
                
                twist.linear.x = limited_vx
                twist.linear.y = limited_vy
                twist.linear.z = limited_vz
                twist.angular.z = limited_yaw

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
