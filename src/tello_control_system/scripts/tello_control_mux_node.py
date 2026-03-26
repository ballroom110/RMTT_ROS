#!/usr/bin/env python3
"""Tello control multiplexer node.

Arbitrates multiple velocity command inputs and publishes a single output:
  - Inputs:
      /tello/cmd_vel_keyboard
      /tello/cmd_vel_joy
      /tello/cmd_vel_agent
  - Output:
      /tello/cmd_vel

Selection rule:
  1) Discard sources whose latest message is older than its timeout.
  2) Among valid sources, choose highest priority.
  3) If none valid, publish zero Twist.
"""

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class TelloControlMuxNode:
    def __init__(self):
        rospy.init_node("tello_control_mux_node", anonymous=False)

        # Topics (configurable, with required defaults)
        self.topic_keyboard = rospy.get_param("~topic_keyboard", "/tello/cmd_vel_keyboard")
        self.topic_joy = rospy.get_param("~topic_joy", "/tello/cmd_vel_joy")
        self.topic_agent = rospy.get_param("~topic_agent", "/tello/cmd_vel_agent")
        self.topic_output = rospy.get_param("~topic_output", "/tello/cmd_vel")
        self.topic_active_source = rospy.get_param("~topic_active_source", "/tello/mux/active_source")

        # Priority: higher wins (default keyboard > joy > agent)
        self.priorities = {
            "keyboard": int(rospy.get_param("~priority_keyboard", 3)),
            "joy": int(rospy.get_param("~priority_joy", 2)),
            "agent": int(rospy.get_param("~priority_agent", 1)),
        }

        # Timeout freshness in seconds
        self.timeouts = {
            "keyboard": float(rospy.get_param("~timeout_keyboard", 0.3)),
            "joy": float(rospy.get_param("~timeout_joy", 0.5)),
            "agent": float(rospy.get_param("~timeout_agent", 0.5)),
        }

        self.rate_hz = float(rospy.get_param("~rate_hz", 30.0))

        # Storage: last message + timestamp per source
        self.last_msg = {
            "keyboard": Twist(),
            "joy": Twist(),
            "agent": Twist(),
        }
        self.last_time = {
            "keyboard": None,
            "joy": None,
            "agent": None,
        }

        # Timeout state cache to avoid log spam
        self.is_valid_cache = {
            "keyboard": False,
            "joy": False,
            "agent": False,
        }

        self.active_source = "none"

        # Publishers
        self.pub_cmd_vel = rospy.Publisher(self.topic_output, Twist, queue_size=20)
        self.pub_active_source = rospy.Publisher(self.topic_active_source, String, queue_size=10, latch=True)

        # Subscribers (separate callbacks per source)
        self.sub_keyboard = rospy.Subscriber(self.topic_keyboard, Twist, self.cb_keyboard, queue_size=20)
        self.sub_joy = rospy.Subscriber(self.topic_joy, Twist, self.cb_joy, queue_size=20)
        self.sub_agent = rospy.Subscriber(self.topic_agent, Twist, self.cb_agent, queue_size=20)

        rospy.loginfo(
            "[mux] started: output=%s, priorities(k/j/a)=%d/%d/%d, timeouts(k/j/a)=%.2f/%.2f/%.2f, rate=%.1fHz",
            self.topic_output,
            self.priorities["keyboard"],
            self.priorities["joy"],
            self.priorities["agent"],
            self.timeouts["keyboard"],
            self.timeouts["joy"],
            self.timeouts["agent"],
            self.rate_hz,
        )

    def cb_keyboard(self, msg: Twist):
        self.last_msg["keyboard"] = msg
        self.last_time["keyboard"] = rospy.Time.now()

    def cb_joy(self, msg: Twist):
        self.last_msg["joy"] = msg
        self.last_time["joy"] = rospy.Time.now()

    def cb_agent(self, msg: Twist):
        self.last_msg["agent"] = msg
        self.last_time["agent"] = rospy.Time.now()

    @staticmethod
    def zero_twist() -> Twist:
        return Twist()

    def source_is_fresh(self, source: str, now: rospy.Time) -> bool:
        t_last = self.last_time[source]
        if t_last is None:
            return False
        age = (now - t_last).to_sec()
        return age <= self.timeouts[source]

    def get_valid_sources(self, now: rospy.Time):
        valid = []
        for source in ("keyboard", "joy", "agent"):
            is_valid = self.source_is_fresh(source, now)

            # Timeout transition logging
            if self.is_valid_cache[source] and not is_valid:
                age = (now - self.last_time[source]).to_sec() if self.last_time[source] is not None else -1.0
                rospy.logwarn("[mux] source timeout: %s (age=%.3fs, limit=%.3fs)", source, age, self.timeouts[source])

            self.is_valid_cache[source] = is_valid
            if is_valid:
                valid.append(source)
        return valid

    def choose_source(self, valid_sources, now: rospy.Time) -> str:
        if not valid_sources:
            return "none"

        # Highest priority wins; if tie, newer timestamp wins
        def key_fn(src):
            t_last = self.last_time[src]
            stamp = t_last.to_sec() if t_last is not None else 0.0
            return (self.priorities[src], stamp)

        return max(valid_sources, key=key_fn)

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        self.pub_active_source.publish(String(data=self.active_source))

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            valid_sources = self.get_valid_sources(now)
            selected = self.choose_source(valid_sources, now)

            if selected != self.active_source:
                rospy.loginfo("[mux] active source: %s -> %s", self.active_source, selected)
                self.active_source = selected
                self.pub_active_source.publish(String(data=self.active_source))

            if selected == "none":
                self.pub_cmd_vel.publish(self.zero_twist())
            else:
                self.pub_cmd_vel.publish(self.last_msg[selected])

            rate.sleep()


def main():
    try:
        node = TelloControlMuxNode()
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
