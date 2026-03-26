#!/usr/bin/env python3
import rospy
from std_msgs.msg import Empty, UInt8, Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
#from dynamic_reconfigure.server import Server
from h264_image_transport.msg import H264Packet
from tello_driver.msg import TelloStatus
from cv_bridge import CvBridge, CvBridgeError

import av
import math
import numpy as np
import threading
import time
import tellopy
from tellopy._internal import error
from tellopy._internal import logger


class RospyLogger(logger.Logger):
    def __init__(self, header=''):
        super(RospyLogger, self).__init__(header)

    def error(self, s):
        if self.log_level < logger.LOG_ERROR:
            return
        rospy.logerr(s)

    def warn(self, s):
        if self.log_level < logger.LOG_WARN:
            return
        rospy.logwarn(s)

    def info(self, s):
        if self.log_level < logger.LOG_INFO:
            return
        rospy.loginfo(s)

    def debug(self, s):
        if self.log_level < logger.LOG_DEBUG:
            return
        rospy.logdebug(s)


def notify_cmd_success(cmd, success):
    if success:
        rospy.loginfo('%s command executed' % cmd)
    else:
        rospy.logwarn('%s command failed' % cmd)


class TelloDriverNode(tellopy.Tello):
    def __init__(self):
        self.connect_timeout_sec = 60.0
        self.stream_h264_video = bool(
            rospy.get_param('~stream_h264_video', False))
        self.bridge = CvBridge()
        self.frame_thread = None

        # Watchdog for cmd_vel (safety feature)
        self.last_cmd_vel_time = time.time()
        self.cmd_vel_timeout_sec = 0.5
        self.cmd_vel_watchdog_thread = None

        # Connect to drone
        log = RospyLogger('Tello')
        log.set_level(self.LOG_WARN)
        super(TelloDriverNode, self).__init__()
        rospy.loginfo('Connecting to drone')
        self.connect()
        try:
            self.wait_for_connection(timeout=self.connect_timeout_sec)
        except error.TelloError as err:
            rospy.logerr(str(err))
            rospy.signal_shutdown(str(err))
            self.quit()
            return
        rospy.loginfo('Connected to drone')
        rospy.on_shutdown(self.cb_shutdown)
        # background thread will try reconnect on disconnection
        self.reconnect_thread = threading.Thread(target=self._reconnect_loop)
        self.reconnect_thread.daemon = True
        self.reconnect_thread.start()


        # Setup topics and services
        # NOTE: ROS interface deliberately made to resemble bebop_autonomy
        # Using private namespace (~) to avoid prefix duplication
        self.pub_status = rospy.Publisher(
            '~status', TelloStatus, queue_size=1, latch=True)
        if self.stream_h264_video:
            self.pub_image_h264 = rospy.Publisher(
                '~image_raw/h264', H264Packet, queue_size=10)
        else:
            self.pub_image_raw = rospy.Publisher(
                '~image_raw', Image, queue_size=10)

        self.sub_takeoff = rospy.Subscriber('~takeoff', Empty, self.cb_takeoff)
        self.sub_land = rospy.Subscriber('~land', Empty, self.cb_land)
        self.sub_cmd_vel = rospy.Subscriber('~cmd_vel', Twist, self.cb_cmd_vel)

        self.subscribe(self.EVENT_FLIGHT_DATA, self.cb_status_telem)
        if self.stream_h264_video:
            self.start_video()
            self.subscribe(self.EVENT_VIDEO_FRAME, self.cb_h264_frame)
        else:
            self.start_video()
            self.frame_thread = threading.Thread(target=self.framegrabber_loop)
            self.frame_thread.daemon = True
            self.frame_thread.start()

            

        # NOTE: odometry from parsing logs might be possible eventually,
        #       but it is unclear from tests what's being sent by Tello
        # (odom publishing disabled)

        # Start cmd_vel watchdog thread
        self.cmd_vel_watchdog_thread = threading.Thread(target=self._cmd_vel_watchdog_loop)
        self.cmd_vel_watchdog_thread.daemon = True
        self.cmd_vel_watchdog_thread.start()

        rospy.loginfo('Tello driver node ready (cmd_vel watchdog: %.1f s)' % self.cmd_vel_timeout_sec)

    def _reconnect_loop(self):
        # periodically check connection state and attempt reconnection
        while not rospy.is_shutdown():
            if self.state == self.STATE_DISCONNECTED:
                rospy.logwarn('Drone disconnected, attempting reconnect...')
                try:
                    self.connect()
                    self.wait_for_connection(timeout=self.connect_timeout_sec)
                    rospy.loginfo('Reconnected to drone')
                except Exception as e:
                    rospy.logwarn('Reconnect failed: %s' % str(e))
                time.sleep(5.0)
            else:
                time.sleep(1.0)

    def _cmd_vel_watchdog_loop(self):
        """Watchdog loop to enforce cmd_vel timeout: zero velocity if no command received."""
        while not rospy.is_shutdown():
            elapsed = time.time() - self.last_cmd_vel_time
            if elapsed > self.cmd_vel_timeout_sec:
                # Timeout: zero all velocity commands
                try:
                    self.set_pitch(0.0)
                    self.set_roll(0.0)
                    self.set_yaw(0.0)
                    self.set_throttle(0.0)
                except Exception:
                    pass
                # Reset timer to avoid spamming
                self.last_cmd_vel_time = time.time()
            time.sleep(0.1)

    def cb_shutdown(self):
        # attempt to land safely then quit
        try:
            self.land()
        except Exception:
            pass
        self.quit()
        if self.frame_thread is not None:
            self.frame_thread.join()
        if hasattr(self, 'reconnect_thread'):
            self.reconnect_thread.join(timeout=1.0)
        if hasattr(self, 'cmd_vel_watchdog_thread'):
            self.cmd_vel_watchdog_thread.join(timeout=1.0)

    def cb_status_telem(self, event, sender, data, **args):
        vx = getattr(data, "vgx", 0.0)
        vy = getattr(data, "vgy", 0.0)
        vz = getattr(data, "vgz", 0.0)
        speed_horizontal_mps = math.sqrt(vx*vx + vy*vy) / 100.0
        speed_northing_mps = -vy / 100.0
        speed_easting_mps = vx / 100.0
        speed_vertical_mps = -vz / 100.0
        # TODO: verify outdoors: anecdotally, observed that:
        # data.east_speed points to South
        # data.north_speed points to East
        msg = TelloStatus(
            height_m=data.height/10.,
            speed_northing_mps=speed_northing_mps,
            speed_easting_mps=speed_easting_mps,
            speed_horizontal_mps=speed_horizontal_mps,
            speed_vertical_mps=speed_vertical_mps,
            flight_time_sec=data.fly_time/10.,
            imu_state=data.imu_state,
            pressure_state=data.pressure_state,
            down_visual_state=data.down_visual_state,
            power_state=data.power_state,
            battery_state=data.battery_state,
            gravity_state=data.gravity_state,
            wind_state=data.wind_state,
            imu_calibration_state=data.imu_calibration_state,
            battery_percentage=data.battery_percentage,
            drone_fly_time_left_sec=data.drone_fly_time_left/10.,
            drone_battery_left_sec=data.drone_battery_left/10.,
            is_flying=data.em_sky,
            is_on_ground=data.em_ground,
            is_em_open=data.em_open,
            is_drone_hover=data.drone_hover,
            is_outage_recording=data.outage_recording,
            is_battery_low=data.battery_low,
            is_battery_lower=data.battery_lower,
            is_factory_mode=data.factory_mode,
            fly_mode=data.fly_mode,
            throw_takeoff_timer_sec=data.throw_fly_timer/10.,
            camera_state=data.camera_state,
            electrical_machinery_state=data.electrical_machinery_state,
            front_in=data.front_in,
            front_out=data.front_out,
            front_lsc=data.front_lsc,
            temperature_height_m=data.temperature_height/10.,
            cmd_roll_ratio=self.right_x,
            cmd_pitch_ratio=self.right_y,
            cmd_yaw_ratio=self.left_x,
            cmd_vspeed_ratio=self.left_y,
        )
        self.pub_status.publish(msg)


    def cb_h264_frame(self, event, sender, data, **args):
        frame, seq_id, frame_secs = data
        pkt_msg = H264Packet()
        pkt_msg.header.seq = seq_id
        pkt_msg.header.frame_id = rospy.get_namespace()
        pkt_msg.header.stamp = rospy.Time.from_sec(frame_secs)
        pkt_msg.data = frame
        self.pub_image_h264.publish(pkt_msg)

    def framegrabber_loop(self):
        # Repeatedly try to connect
        vs = self.get_video_stream()
        while self.state != self.STATE_QUIT:
            try:
                container = av.open(vs)
                break
            except BaseException as err:
                rospy.logerr('fgrab: pyav stream failed - %s' % str(err))
                time.sleep(1.0)

        # Once connected, process frames till drone/stream closes
        while self.state != self.STATE_QUIT:
            try:
                # vs blocks, dies on self.stop
                for frame in container.decode(video=0):
                    img = np.array(frame.to_image())
                    try:
                        img_msg = self.bridge.cv2_to_imgmsg(img, 'rgb8')
                        img_msg.header.frame_id = rospy.get_namespace()
                    except CvBridgeError as err:
                        rospy.logerr('fgrab: cv bridge failed - %s' % str(err))
                        continue
                    self.pub_image_raw.publish(img_msg)
                break
            except BaseException as err:
                rospy.logerr('fgrab: pyav decoder failed - %s' % str(err))


    def cb_takeoff(self, msg):
        success = self.takeoff()
        notify_cmd_success('Takeoff', success)

    def cb_land(self, msg):
        success = self.land()
        notify_cmd_success('Land', success)

    def cb_cmd_vel(self, msg):
        """Callback for velocity commands. Updates watchdog timer and forwards to drone."""
        self.last_cmd_vel_time = time.time()
        self.set_pitch(msg.linear.x)
        self.set_roll(-msg.linear.y)
        self.set_yaw(-msg.angular.z)
        self.set_throttle(msg.linear.z)


def main():
    rospy.init_node('tello_driver_node')
    robot = TelloDriverNode()
    if robot.state != robot.STATE_QUIT:
        rospy.spin()


if __name__ == '__main__':
    main()
