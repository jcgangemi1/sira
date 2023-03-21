#!/usr/bin/env python

'''
Combination of modified versions of stretch_tutorials/voice_teleoperation.py and stretch_tutorials/avoider.py  
'''

import hello_helpers.hello_misc as hm

import math
import rospy
import sys
from numpy import linspace, inf, tanh
from math import sin 
import time
import json 

from std_srvs.srv import Trigger
from stretch_web_interface.srv import recreate_pose

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Int32
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

from speech_recognition_msgs.msg import SpeechRecognitionCandidates

class GetVoiceCommands:
    """
    A class that subscribes to the speech to text recognition messages, prints
    a voice command menu, and defines step size for translational and rotational
    mobile base motion.
    """
    def __init__(self):
        """
        A function that initializes subscribers and defines the three different
        step sizes.
        :param self: The self reference.
        """
        self.rate = rospy.Rate(10) 

        # Set step size as medium by default
        self.step_size = 'medium'
        self.rad_per_deg = math.pi/180.0

        # Small step size parameters
        self.small_deg = 5.0
        self.small_rad = self.rad_per_deg * self.small_deg
        self.small_translate = 0.025

        # Medium step size parameters
        self.medium_deg = 10.0
        self.medium_rad = self.rad_per_deg * self.medium_deg
        self.medium_translate = 0.05

        # Big step size parameters
        self.big_deg = 20.0
        self.big_rad = self.rad_per_deg * self.big_deg
        self.big_translate = 0.1

        # Initialize the voice command
        self.voice_command = None

        # Initialize the sound direction
        self.sound_direction = 0

        # Initialize subscribers
        self.speech_to_text_sub  = rospy.Subscriber("/speech_to_text",  SpeechRecognitionCandidates, self.callback_speech)
        self.sound_direction_sub = rospy.Subscriber("/sound_direction", Int32,                       self.callback_direction)
        self.LIDAR_sub = rospy.Subscriber('/scan', LaserScan, self.callback_LIDAR)

        # Width of the robot to define the y axis bounds of the base
        self.width = 1
        self.extent = self.width / 2.0

        # We want the robot to drive foward or backwards until it is 0.5 m from
        # the closest obstacle measured in front of it
        self.distance = 0.5

        self.file_path = rospy.get_param("/file_path")

        self.recreate_pose = rospy.ServiceProxy('/recreate_pose', recreate_pose)

    def callback_direction(self, msg):
        """
        A callback function that converts the sound direction from degrees to radians.
        :param self: The self reference.
        :param msg: The Int32 message type.
        """
        self.sound_direction = msg.data * -self.rad_per_deg

    def callback_speech(self,msg):
        """
        A callback function that takes all items in the iterable list and join
        them into a single string.
        :param self: The self reference.
        :param msg: The SpeechRecognitionCandidates message type.
        """
        self.voice_command = ' '.join(map(str,msg.transcript))

    def callback_LIDAR(self, msg):
        # Figure out the angles of the scan.  We're going to do this each time, in case we're subscribing to more than one
        # laser, with different numbers of beams
        angles = linspace(msg.angle_min, msg.angle_max, len(msg.ranges))

        # Work out the y coordinates of the ranges
        points = [r * sin(theta) if (theta < -2.5 or theta > 2.5) else inf for r,theta in zip(msg.ranges, angles)]

        # If we're close to the x axis, keep the range, otherwise use inf, which means "no return"
        new_ranges = [r if abs(y) < self.extent else inf for r,y in zip(msg.ranges, points)]

        # Calculate the difference of the closest measured scan and where we want the robot to stop
        error = min(new_ranges) - self.distance
        self.x_distance = tanh(error) if (error > 0.05 or error < -0.05) else 0

    def get_inc(self):
        """
        A function that sets the increment size for translational and rotational
        base motion.
        :param self:The self reference.
        :returns inc: A dictionary type.
        """
        if self.step_size == 'small':
            inc = {'rad': self.small_rad, 'translate': self.small_translate}
        if self.step_size == 'medium':
            inc = {'rad': self.medium_rad, 'translate': self.medium_translate}
        if self.step_size == 'big':
            inc = {'rad': self.big_rad, 'translate': self.big_translate}
        return inc

    def print_commands(self):
        """
        A function that prints the voice teleoperation menu.
        :param self: The self reference.
        """
        print('                                           ')
        print('------------ VOICE TELEOP MENU ------------')
        print('                                           ')
        print('               VOICE COMMANDS              ')
        print(' "forward": BASE FORWARD                   ')
        print(' "back"   : BASE BACK                      ')
        print(' "left"   : BASE ROTATE LEFT               ')
        print(' "right"  : BASE ROTATE RIGHT              ')
        print(' "stretch": BASE ROTATES TOWARDS SOUND     ')
        print('                                           ')
        print('                 STEP SIZE                 ')
        print(' "big"    : BIG                            ')
        print(' "medium" : MEDIUM                         ')
        print(' "small"  : SMALL                          ')
        print('                                           ')
        print('                                           ')
        print(' "quit"   : QUIT AND CLOSE NODE            ')
        print('                                           ')
        print('-------------------------------------------')

    def get_command(self):
        """
        A function that defines the teleoperation command based on the voice command.
        :param self: The self reference.
        :returns rz, dx: A float type.
        """

        rz = 0
        dx = 0
        command = None
        pose = None

        # Update pose dictionary
        saved_file = open(self.file_path + "/saved_poses.json")
        try:
            self.pose_dict = json.load(saved_file)
        except:
            self.pose_dict = {}
        saved_file.close()

        # Move base forward command
        if self.voice_command == 'forward':

            rospy.sleep(rospy.Duration(1))

            rz = 0
            try:
                dx = int(self.voice_command) / 3.28
            except:
                dx = self.get_inc()['translate']
            command = None
            pose = None

        # Move base back command
        if self.voice_command == 'back':
    
            rospy.sleep(rospy.Duration(1))

            rz = 0 
            try:
                dx = -1*int(self.voice_command) / 3.28
            except:
                dx = -self.get_inc()['translate']
            command = None
            pose = None

        # Move base left command
        if self.voice_command == 'left':
            rz = self.get_inc()['rad']
            dx = 0
            command = None
            pose = None

        # Move base right command
        if self.voice_command == 'right':
            rz = -self.get_inc()['rad']
            dx = 0
            command = None
            pose = None

        # Move base to sound direction command
        if self.voice_command == 'stretch':
            rz = self.sound_direction
            dx = 0
            command = None
            pose = None

        if self.voice_command == 'come':
            rz = 0
            dx = self.x_distance
            command = None
            pose = None

        if self.voice_command == 'close':
            rz = 0
            dx = 0
            command = None
            pose = {'joint_gripper_finger_left': -0.30}

        if self.voice_command == 'open':
            rz = 0
            dx = 0
            command = None
            pose = {'joint_gripper_finger_left': 0.15}

        if self.voice_command == '0' or self.voice_command == 'bottom':
            pose = {'joint_lift': 0.15}
        if self.voice_command == '1':
            pose = {'joint_lift': 0.15}
        if self.voice_command == '2':
            pose = {'joint_lift': 0.25}
        if self.voice_command == '3':
            pose = {'joint_lift': 0.35}
        if self.voice_command == '4':
            pose = {'joint_lift': 0.45}
        if self.voice_command == '5' or self.voice_command == 'half':
            pose = {'joint_lift': 0.55}
        if self.voice_command == '6':
            pose = {'joint_lift': 0.65}
        if self.voice_command == '7':
            pose = {'joint_lift': 0.75}
        if self.voice_command == '8':
            pose = {'joint_lift': 0.85}
        if self.voice_command == '9':
            pose = {'joint_lift': 0.95}
        if self.voice_command == '10' or self.voice_command == 'top':
            pose = {'joint_lift': 1.05}
        if self.voice_command == 'up':
            rospy.sleep(rospy.Duration(1))
            try:
                d = int(self.voice_command) * 0.10
            except: 
                d = 0.10
            command = {'joint': ['joint_lift'], 'delta': [d]}
        if self.voice_command == 'down':
            rospy.sleep(rospy.Duration(1))
            try:
                d = int(self.voice_command) * -0.10
            except: 
                d = -0.10
            command = {'joint': ['joint_lift'], 'delta': [-0.10]}

        if self.voice_command == 'arm' or self.voice_command == 'Lyft' or self.voice_command == 'Farm':

            rz = 0
            dx = 0
            command = None
            pose = None

            while self.voice_command == 'arm':
                continue

            if self.voice_command == 'out':
                command = {'joint': ['wrist_extension'], 'delta': [0.10]}
            elif self.voice_command == 'in':
                command = {'joint': ['wrist_extension'], 'delta': [-0.10]}
            elif self.voice_command == 'Max' or self.voice_command == 'Maps':
                pose = {'wrist_extension': 0.5}
            elif self.voice_command == 'min' or self.voice_command == 'men':
                pose = {'wrist_extension': 0.0}
            elif self.voice_command == 'half':
                pose = {'wrist_extension': 0.25}

        if self.voice_command == 'wrist' or self.voice_command == 'gripper':

            rz = 0
            dx = 0
            command = None
            pose = None

            while self.voice_command == 'wrist' or self.voice_command == 'gripper':
                continue

            if self.voice_command == 'out':
                pose = {'joint_wrist_yaw': 0}
            elif self.voice_command == 'in':
                pose = {'joint_wrist_yaw': 3.1415}
            elif self.voice_command == 'right':
                pose = {'joint_wrist_yaw': -1.35}
            elif self.voice_command == 'left':
                pose = {'joint_wrist_yaw': 3.1415/2.0}
            elif self.voice_command == 'turn in':
                command = {'joint': ['joint_wrist_yaw'], 'delta': [0.30]}
            elif self.voice_command == 'turn out':
                command = {'joint': ['joint_wrist_yaw'], 'delta': [-0.30]}

        if self.voice_command == 'stop':
            rz = 0
            dx = 0
            command = {'joint': ['joint_lift','wrist_extension','joint_wrist_yaw', 'joint_gripper_finger_left','joint_head_tilt','joint_head_pan',], 'delta': [0, 0, 0, 0, 0, 0]}
            pose = None

        if self.voice_command == 'Stow':
            rz = 0
            dx = 0
            command = None
            pose = {'joint_lift': 0.15, 'wrist_extension': 0.0, 'joint_wrist_yaw': 3.1415}

        if self.voice_command == 'home':
            rz = 0
            dx = 0
            command = None
            pose = {'joint_lift': 0.70, 'wrist_extension': 0.05, 'joint_wrist_yaw': 0.0}

        if self.voice_command in self.pose_dict.keys():
            pose = None
            rz = 0
            dx = 0
            command = None
            self.recreate_pose(self.voice_command)

        # Set the step size of translational and rotational base motions
        if (self.voice_command == "small") or (self.voice_command == "medium") or (self.voice_command == "big"):
            self.step_size = self.voice_command
            rospy.loginfo('Step size = {0}'.format(self.step_size))

        if self.voice_command == 'quit':
            # Sends a signal to rospy to shutdown the ROS interfaces
            rospy.signal_shutdown("done")

            # Exit the Python interpreter
            sys.exit(0)

        # Reset voice command to None
        self.voice_command = None

        # return the updated command
        return rz, dx, command, pose

class VoiceTeleopNode(hm.HelloNode):
    """
    A class that inherits the HelloNode class from hm and sends joint trajectory
    commands.
    """
    def __init__(self):
        """
        A function that declares object from the GetVoiceCommands class, instantiates
        the HelloNode class, and set the publishing rate.
        """
        hm.HelloNode.__init__(self)
        self.rate = 10.0
        self.joint_state = None


    def joint_states_callback(self, msg):
        """
        A callback function that stores Stretch's joint states.
        :param self: The self reference.
        :param msg: The JointState message type.
        """
        self.joint_state = msg

    def send_command(self, command):
        '''
        Handles single joint control commands by constructing a FollowJointTrajectoryGoal message and sending it to the trajectory_client 
        created in hello_misc.
        '''        
        self.switch_to_position_mode()

        joint_state = self.joint_state
        if (joint_state is not None) and (command is not None):
            point = JointTrajectoryPoint()
            point.time_from_start = rospy.Duration(0.0)
            trajectory_goal = FollowJointTrajectoryGoal()
            trajectory_goal.goal_time_tolerance = rospy.Time(1.0)
            
            j = 0
            for joint_name in command['joint']:
                #joint_name = command['joint']
                trajectory_goal.trajectory.joint_names = [joint_name]
                if 'inc' in command:
                    inc = command['inc'][j]
                    new_value = inc
                elif 'delta' in command:
                    joint_index = joint_state.name.index(joint_name)
                    joint_value = joint_state.position[joint_index]
                    delta = command['delta'][j]
                    new_value = joint_value + delta
                point.positions = [new_value]
                trajectory_goal.trajectory.points = [point]
                trajectory_goal.trajectory.header.stamp = rospy.Time.now()
                self.trajectory_client.send_goal(trajectory_goal)
                self.trajectory_client.wait_for_result()
                j += 1
        
        self.switch_to_navigation_mode()

    def send_twist(self, rz, dx):
        '''
        Calculates and publishes a twist message on the stretch/cmd_vel topic from arguments specifying a rotation about z or a change in x 
        position. Given a fixed angular and linear speed, the length of time to publish the cmd_vel is calculated as the desired change in 
        position divided by the fixed speed.
        '''
        angular_vel = .4 #rad/sec
        linear_vel = .07 #m/sec

        twist = Twist()

        if dx != 0:
            twist.linear.x = linear_vel
            if dx < 0:
                twist.linear.x *= -1
            t = abs(dx) / linear_vel
        else:
            twist.linear.x = 0

        if rz != 0:
            twist.angular.z = angular_vel
            if rz < 0:
                twist.angular.z *= -1
            t = abs(rz) / angular_vel
        else:
            twist.angular.z = 0
        
        if rz !=0 or dx !=0:
            start_time = rospy.Time.now()

            rospy.loginfo(twist)

            while rospy.Time.now() < start_time + rospy.Duration.from_sec(t):
                self.cmd_pub.publish(twist)
                time.sleep(.25)

    def main(self):
        """
        The main function that instantiates the HelloNode class, initializes the subscriber,
        and call other methods in both the VoiceTeleopNode and GetVoiceCommands classes.
        :param self: The self reference.
        """
        hm.HelloNode.main(self, 'voice_teleop', 'voice_teleop', wait_for_first_pointcloud=False)
        self.speech = GetVoiceCommands()
        rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)
        self.switch_to_position_mode = rospy.ServiceProxy('/switch_to_position_mode', Trigger)
        self.switch_to_navigation_mode = rospy.ServiceProxy('/switch_to_navigation_mode', Trigger)
        self.cmd_pub = rospy.Publisher('/stretch/cmd_vel', Twist, queue_size=1)
        rate = rospy.Rate(self.rate)
        self.speech.print_commands()

        while not rospy.is_shutdown():

            # Get voice command
            rz, dx, command, pose = self.speech.get_command()

            # Send twist to follow voice command
            self.send_twist(rz, dx)
            if command is not None:
                rospy.loginfo(command)
                self.send_command(command)
            if pose is not None:
                rospy.loginfo(pose)
                self.move_to_pose(pose)
            rate.sleep()


if __name__ == '__main__':
    try:
        # Declare object from the VoiceTeleopNode class. Then execute the
        # main() method/function
        node = VoiceTeleopNode()
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')