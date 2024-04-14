#!/usr/bin/env python3

"""

This python file runs a ROS-node of name drone_control which holds the position of e-Drone on the given dummy.
This node publishes and subscribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll



Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU TO GAIN MORE MARKS WHILE CODE EVALUATION.
"""

# Importing the required libraries

from edrone_client.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time
import numpy as np
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2
from osgeo import gdal
from sentinel_drone.msg import Geolocation
import csv
import subprocess

my_team_flag = 0
a_cameraframe, b_cameraframe = 0, 0
i, j = 0, 0


class Edrone:
    """docstring for Edrone"""

    def __init__(self):

        self.string = "obj"
        self.message = Geolocation()
        rospy.init_node('drone_control', anonymous=True)

        # initializing ros node with name drone_control

        # This corresponds to your current position of drone. This value must be updated each time in your whycon callback
        # [x,y,z]
        self.drone_position = [0.0, 0.0, 0.0]
        # [x_setpoint, y_setpoint, z_setpoint]
        self.setpoint = [i, j,
                         20]  # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly
        self.hoverpoint = [0, 0, 20]

        # Declaring a cmd of message type edrone_msgs and initializing values
        self.cmd = edrone_msgs()
        self.cmd.rcRoll = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcThrottle = 1500
        self.cmd.rcAUX1 = 1500
        self.cmd.rcAUX2 = 1500
        self.cmd.rcAUX3 = 1500
        self.cmd.rcAUX4 = 1500

        # initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
        # after tuning and computing corresponding PID parameters, change the parameters
        self.Kp = [22, 22, 46.25]
        self.Ki = [0, 0, 0]
        self.Kd = [675, 650, 521]

        # -----------------------Add other required variables for pid here ----------------------------------------------
        self.error = [0, 0, 0]
        self.sum_error = [0, 0, 0]
        self.prev_error = [0, 0, 0]
        self.max_values = 2000
        self.min_values = 1000
        self.image = 0
        self.lon = 0
        self.lat = 0
        self.block_no = 0
        self.objid = ""

        # Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0] where corresponds to [pitch, roll, throttle]		#		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
        #													self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
        #																	You can change the upper limit and lower limit accordingly.
        # ----------------------------------------------------------------------------------------------------------

        # # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
        self.sample_time = 0.060  # in seconds

        # Publishing /drone_command, /alt_error, /pitch_error, /roll_error
        self.command_pub = rospy.Publisher('/drone_command', edrone_msgs, queue_size=1)
        # ------------------------Add other ROS Publishers here-----------------------------------------------------
        self.roll_error_pub = rospy.Publisher('/roll_error', Float64, queue_size=1)
        self.pitch_error_pub = rospy.Publisher('/pitch_error', Float64, queue_size=1)
        self.throttle_error_pub = rospy.Publisher('/throttle_error', Float64, queue_size=1)
        self.location_pub = rospy.Publisher('/geolocation', Geolocation, queue_size=1)
        # self.yaw_error_pub = rospy.Publisher('/yaw_error', Float64, queue_size=1)
        # self.image_pub = rospy.Publisher('/edrone/camera_rgb/image_raw', Image, queue_size=10)

        # -----------------------------------------------------------------------------------------------------------

        # Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
        rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
        rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid)
        # -------------------------Add other ROS Subscribers here----------------------------------------------------
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
        # rospy.Subscriber('/pid_tuning_yaw', PidTune, self.yaw_set_pid)
        rospy.Subscriber('/edrone/camera_rgb/image_raw', Image, self.image_callback)

        # ------------------------------------------------------------------------------------------------------------

        self.arm()  # ARMING THE DRONE
        csvfile = open("./lat_long.csv", 'w')
        self.csvwriter = csv.writer(csvfile)
        # self.csvwriter.writerow(['hi'])

    # Disarming condition of the drone
    def disarm(self):
        self.cmd.rcAUX4 = 1100
        self.command_pub.publish(self.cmd)
        rospy.sleep(1)

    # Arming condition of the drone : Best practise is to disarm and then arm the drone.
    def arm(self):

        self.disarm()

        self.cmd.rcRoll = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcThrottle = 1000
        self.cmd.rcAUX4 = 1500
        self.command_pub.publish(self.cmd)  # Publishing /drone_command
        rospy.sleep(1)

    # Whycon callback function
    # The function gets executed each time when /whycon node publishes /whycon/poses
    def whycon_callback(self, msg):
        self.drone_position[0] = msg.poses[0].position.x

        # --------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z
        # self.drone_position[3] = msg.poses[0].position.

    # ---------------------------------------------------------------------------------------------------------------

    # Callback function for /pid_tuning_altitude
    # This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
    def altitude_set_pid(self, alt):
        self.Kp[2] = alt.Kp * 0.06  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[2] = alt.Ki * 0.008
        self.Kd[2] = alt.Kd * 0.3

    # ----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------
    def roll_set_pid(self, roll):
        self.Kp[0] = roll.Kp * 0.06  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[0] = roll.Ki * 0.008
        self.Kd[0] = roll.Kd * 0.3

    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.Kp * 0.06  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[1] = pitch.Ki * 0.008
        self.Kd[1] = pitch.Kd * 0.3

    def image_callback(self, data):
        br = CvBridge()  # this is for converting image from drone to cv image
        current_frame = br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        self.image = current_frame

    def detection(self):
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        lower = np.array([20, 100, 100])  # 20 100 100
        upper = np.array([30, 255, 255])  # 30 255 255
        mask = cv2.inRange(hsv, lower, upper)
        kernel = np.ones((5, 5), np.uint8)
        erode = cv2.erode(mask, kernel, iterations=1)
        dilate = cv2.dilate(erode, kernel, iterations=1)
        contours, hierarchy = cv2.findContours(dilate, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        global a_cameraframe
        global b_cameraframe

        if len(contours) != 0:
            if cv2.contourArea(contours[0]) > 2550:
                M = cv2.moments(contours[0])
                if M['m00'] != 0.0:
                    a_cameraframe = int(M['m10'] / M['m00'])
                    b_cameraframe = int(M['m01'] / M['m00'])
                    self.hoverpoint[0] = self.drone_position[0] + (a_cameraframe - 320) * 0.0096  # 0.0096
                    self.hoverpoint[1] = self.drone_position[1] + (b_cameraframe - 240) * 0.0096  # 0.0096
                    # print(self.hoverpoint)
                    pixel_x = 2000 + self.hoverpoint[0] * 148
                    pixel_y = 2000 + self.hoverpoint[1] * 140
                    ds = gdal.Open(r'/home/sourav/catkin_ws/src/sentinel_drone/sentinel_drone/scripts/task2d.tif')
                    xoff, a, b, yoff, d, e = ds.GetGeoTransform()

                    self.objid = self.string + str(self.block_no)
                    self.block_no += 1
                    self.lon = a * pixel_x + b * pixel_y + xoff
                    self.lat = d * pixel_x + e * pixel_y + yoff
                    # print(self.objid, self.lon, self.lat)
                    cmd = [self.objid, self.lon, self.lat]
                    self.csvwriter.writerow(cmd)
                    self.message.objectid = self.objid
                    self.message.lat = self.lat
                    self.message.long = self.lon
                    self.location_pub.publish(self.message)

    # ----------------------------------------------------------------------------------------------------------------------

    def pid(self):
        # -----------------------------Write the PID algorithm here--------------------------------------------------------------

        # Steps:
        # 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
        #	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
        #	3. Calculate the pid output required for each axis. For eg: calculate self.out_roll, self.out_pitch, etc.
        #	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
        #	5. Don't run the pid continuously. Run the pid only at a sample time. self.sample time defined above is for this purpose. THIS IS VERY IMPORTANT.
        #	6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
        #																														self.cmd.rcPitch = self.max_values[1]
        #	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
        #	8. Add error_sum

        self.error = [element1 - element2 for (element1, element2) in zip(self.drone_position, self.setpoint)]

        self.cmd.rcRoll = int(1500 - (
                self.error[0] * self.Kp[0] + self.sum_error[0] * self.Ki[0] + (self.error[0] - self.prev_error[0]) *
                self.Kd[0]))
        self.cmd.rcPitch = int(1500 + (
                self.error[1] * self.Kp[1] + self.sum_error[1] * self.Ki[1] + (self.error[1] - self.prev_error[1]) *
                self.Kd[1]))
        self.cmd.rcThrottle = int(1500 + (
                self.error[2] * self.Kp[2] + self.sum_error[2] * self.Ki[2] + (self.error[2] - self.prev_error[2]) *
                self.Kd[2]))
        # self.detection()
        if self.cmd.rcRoll > self.max_values:
            self.cmd.rcRoll = self.max_values
        if self.cmd.rcRoll < self.min_values:
            self.cmd.rcRoll = self.min_values
        if self.cmd.rcPitch > self.max_values:
            self.cmd.rcPitch = self.max_values
        if self.cmd.rcPitch < self.min_values:
            self.cmd.rcPitch = self.min_values
        if self.cmd.rcPitch > self.max_values:
            self.cmd.rcPitch = self.max_values
        if self.cmd.rcPitch < self.min_values:
            self.cmd.rcPitch = self.min_values

        self.prev_error = self.error
        self.sum_error = [element1 + element2 for (element1, element2) in zip(self.sum_error, self.error)]

        global my_team_flag
        global i
        global j

        if (i - 0.1 < self.drone_position[0] < i + 0.1 and
                j - 0.1 < self.drone_position[1] < j + 0.1 and my_team_flag == 0):
            i += 6
            self.detection()

            self.setpoint = [i, j, 20]
            my_team_flag += 1
        elif (i - 0.1 < self.drone_position[0] < i + 0.1 and
              j - 0.1 < self.drone_position[1] < j + 0.1 and my_team_flag == 1):
            j = j * -1
            j += 6
            self.detection()
            self.setpoint = [i, j, 20]
            my_team_flag += 1
        elif (i - 0.1 < self.drone_position[0] < i + 0.1 and
              j - 0.1 < self.drone_position[1] < j + 0.1 and my_team_flag == 2):
            self.detection()
            i = i * -1
            self.setpoint = [i, j, 20]
            my_team_flag += 1
        elif (i - 0.1 < self.drone_position[0] < i + 0.1 and
              j - 0.1 < self.drone_position[1] < j + 0.1 and my_team_flag == 3):
            self.detection()
            j = j * -1
            self.setpoint = [i, j, 20]
            my_team_flag += 1
        elif (i - 0.1 < self.drone_position[0] < i + 0.1 and
              j - 0.1 < self.drone_position[1] < j + 0.1 and my_team_flag == 4):
            i = i * -1
            self.detection()
            self.setpoint = [i, j, 20]
            my_team_flag = 0
        if i < -8:
            self.setpoint[0] = -8
        if i > 8:
            self.setpoint[0] = 8
        if j < -8:
            self.setpoint[1] = -8
        if j > 8:
            self.setpoint[1] = 8

        self.command_pub.publish(self.cmd)
        self.roll_error_pub.publish(self.error[0])
        self.pitch_error_pub.publish(self.error[1])
        self.throttle_error_pub.publish(self.error[2])


if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(
        16)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
        e_drone.pid()
        # our own functions can go here like e_drone.myfunc !!!!!!!!!!!!!!!!
        r.sleep()
