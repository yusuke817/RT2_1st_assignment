#! /usr/bin/env python

## @package final_assignment
# \file mode.py
# \brief python script about UI for drivng simulation
# \author Yusuke Kido
# \version 0.1
# \date 28/06/2022

## @detail
# \Subscribes to: <BR> /input_cmd_vel /move_base/goal /scan

# \Publishes to: <BR>
# /cmd_vel
# /move_base/goal

# \Services : <BR>
# /gazebo/reset_world

# \Description :
# The code enables the robot (car) to take user input and collect car's position and give the velocity and the information on the goal.

import rospy
import time
import os
import actionlib
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from move_base_msgs.msg import MoveBaseActionFeedback
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rt2_first_assignment.srv import Cmd, CmdResponse
from rt2_first_assignment.srv import Dist, DistResponse
from rt2_first_assignment.msg import reach

# range as arriving at the goal  
dis_th = 0.5

# threshold for collision avoidance 
obj_th = 1.0

# 1 minute is the maximum time for the car to reach the goal  
time_th = rospy.Duration(60)

# defining vel_msg 
vel_msg = Twist()  

# There is no goal set at the beginning
yesno_goal = False

# This variable is for data analysis. There is no goal reached at the beginning
reached = 0

# This variable is for data analysis. There is no goal notreach at the beginning
unreached = 0

# It will be true while the robot is aiming at the goal 
goal_aim = False

goal_msg = MoveBaseGoal()

mode = 7

user_input = 8

# to check whether mode is changed or not
mode_change = False

def fix_goal_dist(req):
    '''!In Auto_drive mode, fixing the goal coordinate by receiving the dist service'''     
    global yesno_goal, start_time, input_x, input_y, auto_mode, action_client
    
    # Receiving the destination from jupyter with service in auto driving mode
    input_x = req.pos_x
    input_y = req.pos_y
    print('Goal is set and its coordinates are: (', input_x, ' , ', input_y, ')\n')
    # Check if the autonomus driving mode is selected
    if auto_mode:
    # send a goal to the robot to move to the destination
    	goal_msg.target_pose.header.frame_id = "map"
    	goal_msg.target_pose.pose.orientation.w = 1

    	goal_msg.target_pose.pose.position.x = input_x
    	goal_msg.target_pose.pose.position.y = input_y

    	# publishing message and geting the time of reaching
    	start_time = rospy.Time.now()
    	# goal is set
    	yesno_goal = True
    	# Send the goal msg to the action server
    	action_client.send_goal(goal_msg)
    	return DistResponse(1)
    	
    # Function will return 0 not in auto driving mode
    else:
    	return DistResponse(0)


def receive_goal(msg):
    '''!these two values are stored and subscribed by move_base topic'''
    global decided_x, decided_y
    decided_x = msg.goal.target_pose.pose.position.x
    decided_y = msg.goal.target_pose.pose.position.y
    
   
def change_mode_cmd(req):
    '''!choosing the driving mode by receiving the cmd service''' 
    global mode, user_input, mode_change
    mode = req.cmd
    if mode == 0 or mode == 1 or mode == 2 or mode == 3 or mode == 4 or mode == 5:
    	if mode != user_input:
    		user_input = mode
    		mode_change = True
    		return CmdResponse(1) # success in changing mode
    	else:
    		return CmdResponse(2) # already changed 
    		mode_change = False
    else:
    	return CmdResponse(0) # improper input
    	mode_change = False


def check_goal(msg):
    '''!checking whether the goal is reachable or not and notifying arrival'''
    global yesno_goal, reached
    if yesno_goal:
        # time calculation
        end_time = rospy.Time.now()
        reaching_time = end_time - start_time #line 87
        # If it takes much time, destination will be recognized as unreachable and finish (cancel) the process
        if reaching_time > time_th:
            cancel_goal()
            print('target is far from here: it takes too much time to reach\n\n')
            rospy.sleep(1)
            
        # getting current car position
        current_x = msg.feedback.base_position.pose.position.x
        current_y = msg.feedback.base_position.pose.position.y

        # measuring the distance between here and the destination
        dist_x = current_x - decided_x
        dist_y = current_y - decided_y

        # checking if it's close enough to be considered goal reached and finish the process
        if abs(dist_x) < dis_th and abs(dist_y) < dis_th:
            # Add the number of reaching
            reached = reached + 1
            goal_reached = True     
            finish_process()
            print('Reached the goal\n\n')               
            rospy.sleep(1)
          
def cancel_goal():
    '''!cancelling the process before reaching the goal'''
    global yesno_goal, reached, unreached, pub_status, action_client
    # No goal has been set
    if not yesno_goal:
        return
     # If there is a goal, cancel it
    else:
        action_client.cancel_goal()
        yesno_goal = False
        # Increase the notreach target count
        unreached = unreached + 1
        # The goal is canceled
        pub_status.publish(reached, unreached)
	    
def finish_process():
    '''!finishing the process before reaching the goal'''
    global yesno_goal, reached, unreached, pub_status, action_client
    # No goal has been set
    if not yesno_goal:
        return
     # If there is a goal, cancel it
    else:
        action_client.cancel_goal()
        yesno_goal = False
        # The goal is canceled 
        pub_status.publish(reached, unreached) 


def assisted_driving(msg):
    '''!in each step, car collects the information on the distance between a car and an object with laserscan topic and decide the speed in assisted mode'''

    global parts, vel_msg
    # in manual mode, we don't use 
    if not drive_assist:
        return

    # in assisted mode mode, car uses modified speed depending on the situation related to objects surrounding it
    # how to modify the speed is shown below.
    else:
    
    # 720/5=144. the sensor's coverage is divided into five parts:right, fright, front, fleft and left.
        parts = {
            'right':  min(min(msg.ranges[0:143]), 10),
            'fright': min(min(msg.ranges[144:287]), 10),
            'front':  min(min(msg.ranges[288:431]), 10),
            'fleft':  min(min(msg.ranges[432:575]), 10),
            'left':   min(min(msg.ranges[576:719]), 10),
        }

    ### The speed modification is different from the relationships between car and obstacle ###
        
        # Obstacle located in front of the car
        if parts['front'] < obj_th:
            # cannot move forward
            if vel_msg.linear.x > 0 and vel_msg.angular.z == 0:#i
                # Stop car's linear velocity #k
                vel_msg.linear.x = 0

        # Obstacle located in the front-right of the car    
        elif parts['fright'] < obj_th:
            # cannot move forward and can only rotate in counterclockwise direction
            if vel_msg.linear.x > 0 and vel_msg.angular.z < 0:#o
                # Stop car's linear velocity  #k
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0

        # Obstacle located in the front-left of the car
        elif parts['fleft'] < obj_th: 
            # cannot move forward and can only rotate in clockwise direction
            if vel_msg.linear.x > 0 and vel_msg.angular.z > 0:#u
                # Stop car's linear velocity #k
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0

        # Obstacle located in the right of the car
        elif parts['right'] < obj_th:
            # cannot move forward and can only rotate in counterclockwise direction
            if vel_msg.linear.x == 0 and vel_msg.angular.z < 0:#l
                # Stop car's angular velocity #k
                vel_msg.angular.z = 0

        # Obstacle located in the left of the car
        elif parts['left'] < obj_th: 
            # cannot move forward and can only rotate in clockwise direction
            if vel_msg.linear.x == 0 and vel_msg.angular.z > 0:#j
                # Stop car's angular velocity #k
                vel_msg.angular.z = 0

        # publish modified (assisted) velocity
        pub_vel.publish(vel_msg)


def actual_vel(msg):
    '''!with teleop_twist_keyboard, user can change the speed of a car'''
    global vel_msg, pub_vel
    # in auto mode, this function is not effective
    if not manual_mode and not drive_assist:
    	return
    else:
    	# in manual mode, the velocity is directly published without modification
    	if not drive_assist:
    		pub_vel.publish(msg)
    		return
    	# in assist mode, the velocity is stored
    	else:
    		vel_msg.linear.x = msg.linear.x
    		vel_msg.angular.z = msg.angular.z    		
	
def choose_driving_mode():
    global manual_mode, drive_assist, user_input, auto_mode

    # Auto_drive
    if user_input == 1:
    	print('Auto drive')
    	auto_mode = True
    	manual_mode = False
    	drive_assist = False

    # Manual driving mode
    elif user_input == 2:
    	print('Manual drive\n')
    	auto_mode = False
    	manual_mode = True
    	drive_assist = False

    # Assisted driving mode
    elif user_input == 3:
    	print('Assisted drive\n')
    	auto_mode = False
    	manual_mode = False
    	drive_assist = True

    # to cancel car position 
    elif user_input == 4:
    	auto_mode = False
    	manual_mode = False
    	drive_assist = False       
    	cancel_goal()
    	print('user cancelled the goal\n')
    
    # to reset car position
    elif user_input == 5:
    	auto_mode = False
    	manual_mode = False
    	drive_assist = False
    	reset_world()
    	print('going back to the initial position\n')

    # Exit the loop
    elif user_input == 0:
        # resetting the terminal
        os.system('clear')
        print('finishing the program\n')
        # shutting down nodes
        rospy.on_shutdown()

    # Not one of the possible options  
    else:
        # resetting the terminal
        os.system('clear')
        print('INVALID command.\n')
        

def main():
    # global variables
    global auto_mode, reset_world, get_mode, get_target
    global yesno_goal, drive_assist, manual_mode, user_input, goal_to_cancel, mode_change
    global pub_vel, pub_status, sub_laser, sub_goal, sub_user_vel, sub_robot_pos, action_client

    #initializing the mode: no goal, no assist, no manual(equivalent to automatic) are the original state

    # initialize the goal which means that a car does not have a goal
    yesno_goal = False
    # initialize drive_assist which means that a car does not have a driving assistence
    drive_assist = False
    # initialize the mode which means that a car is not in manual mode
    manual_mode = False
    # initialize the mode which means that a car is not in auto mode
    auto_mode = False  

    # Initialize the node, setup the NodeHandle for handling the communication with the ROS system  
    rospy.init_node('mode')

    # Create a client to reset the simulation environment
    rospy.wait_for_service('/gazebo/reset_world')
    reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    # Create the server to answer the request from the command service
    get_modality = rospy.Service('/cmd', Cmd, change_mode_cmd)
    # Create the server to answer the request from the target service
    get_target = rospy.Service('/dist', Dist, fix_goal_dist)
	
    # Initialize the action client
    action_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

    # Initialize publishers
    pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
    pub_status = rospy.Publisher('/status_goal', reach, queue_size = 100)

    # Initialize subscribers
    sub_laser = rospy.Subscriber('/scan', LaserScan, assisted_driving)
    sub_goal = rospy.Subscriber('move_base/goal', MoveBaseActionGoal, receive_goal)
    sub_user_vel = rospy.Subscriber('/input_cmd_vel', Twist, actual_vel)
    sub_robot_pos = rospy.Subscriber('move_base/feedback', MoveBaseActionFeedback, check_goal)

    # the loop keeps going on until user pushes "5" which is the reset button
    while not rospy.is_shutdown():
        # Check if the modality was changed
        if mode_change:	
        	# Decide what to do based on user decision
        	choose_driving_mode()
        	mode_change = False


if __name__ == '__main__':
    main()
