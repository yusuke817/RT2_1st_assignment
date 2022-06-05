#! /usr/bin/env python
"""
.. module:: set_mode
    :platform: Unix
    :synopsis: Python module for the user Interface
   
.. moduleauthor:: Francesco Ferrazzi <s5262829@studenti.unige.it>
ROS node for the the first assignment of the Research Track 2 course of the Robotics 
Engineering Master program. The software architecture allow to control a mobile robot 
in such a way that:
1) Autonomusly reach a (x,y) coordinate in a 2D space inserted by the user.
2) Let the user drive the robot with the keyboard.
3) Let the user drive the robot assisting them to avoid collisions.
4) Cancel the goal up to user's desire.
5) Reset robot position.
This works by interfacing with the jupyter notebook placed in the "notebook" folder.
Subscribes to:
    /scan topic which contains 720 values that are distances taken by the laser scan
    /move_base/goal to get the goal position 
    /us_cmd_vel remap the desired velocity given by user
    /move_base/feedback where the simulatior publishes the robot position
  
Publishes to:
    /cmd_vel to define the wanted robot velocity
    /status_goal to publish the final status of the goal
  
Service:
    /gazebo/reset_world to reset the robot position
    /get_modality to change the robot's behavior according to the user's choice
    /get_target to set the target that the robot can reach autonomusly
    
Action Client:
    /move_base to set and cancel the goal position
    
"""

# Import libraries
import rospy
import time
import os
import actionlib

# Messages
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from move_base_msgs.msg import MoveBaseActionFeedback
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rt2_first_assignment.srv import Command, CommandResponse
from rt2_first_assignment.srv import Target, TargetResponse
from rt2_first_assignment.msg import GoalStatus


# Global Variables

th_reach = 0.3
# Range threshold distance used to know if the robot reached the goal 

th_collision = 0.7
# Threshold for avoiding collision

max_time = rospy.Duration(45)
# Max time needed by the robot to reach the goal position is 45 seconds

goal_set = False
# No goal is set when we first launch the program 

reached = 0
# No goal has been reached at the beginning of the program 

unreached = 0
# No gaol has been unreached at the beginning of the program 

goal_active = False
# True while the robot tries to reache the goal 

vel_msg = Twist()
# Global velocity message 

goal_msg = MoveBaseGoal()
# Global goal message 

modality = 5
# Set the modality to a default value 

input_u = 6
# Set the user input to some random value 

mod_change = False
# Boolean variable to see if a modality is changed by the user 


def srv_callback_target(req):
    """
    Function that is called each time a request from the "/target" service is received. 
    The aim is to set the target, which is a point inside the two dimensional simulation
    environment. The robot needs to reach autonomusly the given point.
    The function publishes the goal only if the first modality is active.
    
    Args:
    	req (Int): pos_x and pos_y that specify the target position
    	
    Return:
    	res (Int): goal_feedback that specify if a position is reached or not
    
    """
    global goal_set, start_time, in_x, in_y, autonomus_mode, action_client
    # Get the target's coordinates from the user interface in jupyter using the service if the autonomus drive mode is selected
    in_x = req.pos_x
    in_y = req.pos_y
    print('Goal is set and its coordinates are: (', in_x, ' , ', in_y, ')\n')
    # Check if the autonomus driving mode is selected
    if autonomus_mode:
    	goal_msg.target_pose.header.frame_id = "map"
    	goal_msg.target_pose.pose.orientation.w = 1

    	goal_msg.target_pose.pose.position.x = in_x
    	goal_msg.target_pose.pose.position.y = in_y

    	# Get the time as soon as the message is sent
    	start_time = rospy.Time.now()
    	# goal has been set
    	goal_set = True
    	# Send the goal msg to the action server
    	action_client.send_goal(goal_msg)
    	return TargetResponse(1)
    	
    # If the autonomus driving mode is not selected the function returns 0
    else:
    	return TargetResponse(0)


def get_goal(msg):
    """
    Function used to store the goal once it is published.
    Args:
    	msg (/move_base/goal): goal subscribed by move_base topic.
    	
    """
    global goal_x, goal_y
    goal_x = msg.goal.target_pose.pose.position.x
    goal_y = msg.goal.target_pose.pose.position.y
    
   
def srv_callback_command(req):
    '''
    Function called when a request is sent to the service "/command".
    The aim is to select the correct driving modality according to the user decision.
    If the user decision respects one of the possible modalities and the chosen modality
    is different from the previous one, then change it and send a feedback. If not, the
    modality doesn't change and a feedback is sent.
    
    Args:
    	req (Int): command that specify the driving modality
    	
    Return:
    	res (Int): feedback that specify if the driving mode was changed or not
    
    '''
    global modality, input_u, mod_change
    modality = req.command
    if modality == 0 or modality == 1 or modality == 2 or modality == 3 or modality == 4 or modality == 9:
    	if modality != input_u:
    		input_u = modality
    		mod_change = True
    		return CommandResponse(1) # Change of modality was succesful
    	else:
    		return CommandResponse(2) # User is already in this modality
    		mod_change = False
    else:
    	return CommandResponse(0) # The chosen modality does not exist
    	mod_change = False


def goal_reached(msg):
    """
    Function that tells if the goal has been reached or not. If the target is reached, a 
    message is printed.
    A goal is considered to be unreachable if, after five minutes, the robot is not able 
    to arrive at the desired position. If the goal is considered unreachable, a message is
    printed.
    Args:
    	msg (/move_base/feedback): robot position according to the simulation environment.
    	
    """
    global goal_set, reached, unreached, goal_x, goal_y
    if goal_set:
        # Get time
        end_time = rospy.Time.now()
        goal_time = end_time - start_time
        # If time expired, target can not be reached
        if goal_time > max_time:
            goal_unreached = True
            print('Goal was NOT Reached!!\n')
            cancel_goal()
            
        # Get robot position in a certain instant
        rob_x = msg.feedback.base_position.pose.position.x
        rob_y = msg.feedback.base_position.pose.position.y

        # See how far the robot is from the goal
        x_dist = rob_x - goal_x
        y_dist = rob_y - goal_y

        # See if it's close enough to be considered goal reached
        if abs(x_dist) < th_reach and abs(y_dist) < th_reach:
            # Decrease one to the unreached target count since I increase the unreached count every time the 'cancel_goal()' function is called
            unreached = unreached - 1
            # Add one to the reached targets count
            reached = reached + 1
            goal_reached = True
            print('Goal was Reached!!\n')
            cancel_goal()
          

def cancel_goal():
    """
    Function used to cancel the goal once is set by the user.
    If the goal has not been set, the function does nothing.
    A goal canceled counts as unreached. When a goal is canceled, the
    status of the goal is published.
    
    """
    global goal_set, reached, unreached, pub_status, action_client
    # No goal has been set
    if not goal_set:
        return
     # If there is a goal, cancel it
    else:
        action_client.cancel_goal()
        goal_set = False
        # Increase the unreached target count
        unreached = unreached + 1
        # Publish the status of the goal
        # The goal is canceled even if is reached or unreached 
        pub_status.publish(reached, unreached)


def assisted_driving(msg):
    """
    Function called each time arrives a message from the /scan topic.
    If the user asks for driving assistance while it's in manual mode, the function gets  
    the minimum value among a region of the laser scan. Does this for each defined region
    and checks if there is an obstacle which is too close to the robot in that reagion. 
    If this condition is verified, it does not allow the user to go towards the obstacle 
    but only to avoid it. At the end, the correct velocity is published.
    If user doesn't ask for assistance, the function does nothing.
    Args:
    	msg (/scan): array of 720 values defining the distances from the sensor to the
    		objects in the environment  
    	
    """
    global regions, vel_msg
    # If no assistance is required, exit the function
    if not drive_assistance:
        return
    # If assistance is enabled, help the user driving around
    else:
    
        regions = {
            'right':  min(min(msg.ranges[0:143]), 10),
            'fright': min(min(msg.ranges[144:287]), 10),
            'front':  min(min(msg.ranges[288:431]), 10),
            'fleft':  min(min(msg.ranges[432:575]), 10),
            'left':   min(min(msg.ranges[576:719]), 10),
        }
        # Avoid risky situations when the robot is going to collide into walls
        # Ostacle positioned in front of the robot
        if regions['front'] < th_collision:
            # Allow only rotation
            if vel_msg.linear.x > 0 and vel_msg.angular.z == 0:
                # Stop robot's linear velocity
                vel_msg.linear.x = 0

        # Ostacle positioned on the front-right of the robot    
        elif regions['fright'] < th_collision:
            # Allow only rotation on the left
            if vel_msg.linear.x > 0 and vel_msg.angular.z < 0:
                # Stop robot's linear velocity
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0

        # Ostacle positioned on the front-left of the robot
        elif regions['fleft'] < th_collision: 
            # Allow only rotation on the right
            if vel_msg.linear.x > 0 and vel_msg.angular.z > 0:
                # Stop robot's linear velocity
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0

        # Ostacle positioned on the right of the robot
        elif regions['right'] < th_collision:
            # Allow only rotation on the left
            if vel_msg.linear.x == 0 and vel_msg.angular.z < 0:
                # Stop robot's linear velocity
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0

        # Ostacle positioned on the left of the robot
        elif regions['left'] < th_collision: 
            # Allow only rotation on the right
            if vel_msg.linear.x == 0 and vel_msg.angular.z > 0:
                # Stop robot's linear velocity
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
        
        # Set new velocity and publish it
        pub_vel.publish(vel_msg)


def set_user_vel(msg):
    """
    Function called each time the user uses the teleop keyboard to set the robot's velocity.
    If the driving mode is not in second or third modality, the function does nothing.
    If the driving assistance is not active, the velocity decided by the user is published.
    If is active, the value of the velocity is set and checked by the assisted_driving
    function before it gets published.
    Args:
    	msg (/us_cmd_vel): the desired robot velocity.
    	
    """
    global vel_msg, pub_vel
    # If the robot is not in second or third modality, the command given from user is ignored
    if not man_mode and not drive_assistance:
    	return
    else:
    	# If the robot is not in drive assistance mode, the velocity is published
    	if not drive_assistance:
    		pub_vel.publish(msg)
    		return
    	# If the robot is in manual mode and the drive assistance is active the message 
    	# given by user is saved and checked from the assisted driving function
    	else:
    		vel_msg.linear.x = msg.linear.x
    		vel_msg.angular.z = msg.angular.z    		
     
	
def driving_decision():
    """ 
    Function used to decide the behavior of the robot according to the user input.
    This function decides the driving modality and the actions that the robot does according
    to the input given from the keyboard by the user.
    	
    """
    global man_mode, drive_assistance, input_u, autonomus_mode
    # Select what to do
    # Autonomus driving with goal set from user
    if input_u == 1:
    	print('Autonomus drive active')
    	autonomus_mode = True
    	man_mode = False
    	drive_assistance = False

    # Manual driving mode
    elif input_u == 2:
    	print('Manual mode without assistance active\n')
    	# Deactivate the autonomus driving mode
    	autonomus_mode = False
    	# Man mode activated
    	man_mode = True
    	# Set drive assistance to false
    	drive_assistance = False

    # Assisted manual driving mode
    elif input_u == 3:
    	print('Manual mode with assistance active\n')
    	# Deactivate the autonomus driving mode
    	autonomus_mode = False
    	# Deactivate the manual mode
    	man_mode = False
    	# Assistance needed
    	drive_assistance = True

    # Cancel robot's target position 
    elif input_u == 4:
    	print('Goal is canceled\n')
    	# Deactivate the autonomus driving mode
    	autonomus_mode = False
    	# Deactivate the manual mode
    	man_mode = False
    	# Set drive assistance to false
    	drive_assistance = False
    	# Cancel the goal
    	cancel_goal()
    
    # Reset robot position
    elif input_u == 9:
    	print('World is resetted\n')
    	# Deactivate the autonomus driving mode
    	autonomus_mode = False
    	# Deactivate the manual mode
    	man_mode = False
    	# Set drive assistance to false
    	drive_assistance = False
    	# Reset robot's position
    	reset_world()

    # Kill the program
    elif input_u == 0:
    	print('Bye!\n')
    	rospy.on_shutdown()

    # Not one of the possible options  
    else:
        # Clear terminal
        os.system('clear')
        print('Invlid input.\nPlease, insert a new command\n\n')
        

def main():
    """
  
    This is the *main function* which inizializes the ROS node and defines the needed
    service, publishers and subscribers.
    After that it loops until the ROS node is *not* shutdown. 
    While the node is looping, it calls the functions used to:
    - print the user interface, 
    - get and check the input given from the keyboard by the user
    - decide the behavior of the robot according to the user's decision
  
    The node relys on the 'rospy <http://wiki.ros.org/rospy/>'_ module  
    """
    # initialize global variables
    global autonomus_mode, reset_world, get_modality, get_target
    global goal_set, drive_assistance, man_mode, input_u, goal_to_cancel, mod_change
    global pub_vel, pub_status, sub_laser, sub_goal, sub_user_vel, sub_robot_pos, action_client

    # Initialize that the goal has not been set yet
    goal_set = False
    # Initialize that there is no need of driving assistence
    drive_assistance = False
    # Initialize that the robot is not in manual mode
    man_mode = False
    # Initialize that the robot is not in manual mode
    autonomus_mode = False

    # Initialize the node, setup the NodeHandle for handling the communication with the ROS system  
    rospy.init_node('set_mode')

    # Create a client to reset the simulation environment
    rospy.wait_for_service('/gazebo/reset_world')
    reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    # Create the server to answer the request from the command service
    get_modality = rospy.Service('/command', Command, srv_callback_command)
    # Create the server to answer the request from the target service
    get_target = rospy.Service('/target', Target, srv_callback_target)
	
    # Initialize the action client
    action_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

    # Initialize publishers
    pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
    pub_status = rospy.Publisher('/status_goal', GoalStatus, queue_size = 100)

    # Initialize subscribers
    sub_laser = rospy.Subscriber('/scan', LaserScan, assisted_driving)
    sub_goal = rospy.Subscriber('move_base/goal', MoveBaseActionGoal, get_goal)
    sub_user_vel = rospy.Subscriber('/us_cmd_vel', Twist, set_user_vel)
    sub_robot_pos = rospy.Subscriber('move_base/feedback', MoveBaseActionFeedback, goal_reached)

    # Infinite loop until user doesn't press 9 and ros::ok() returns true
    while not rospy.is_shutdown():
        # Check if the modality was changed
        if mod_change:	
        	# Decide what to do based on user decision
        	driving_decision()
        	mod_change = False


if __name__ == '__main__':
    main()
