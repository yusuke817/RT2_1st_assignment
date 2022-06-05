#! /usr/bin/env python

## @package final_assignment
# \file menu.py
# \brief python script about UI for drivng simulation(file:///root/ros_ws/src/final_assignment/docs/html/menu_8py.html)
# \author Yusuke Kido
# \version 0.1
# \date  20/04/2022

## @detail
# \Subscribes to: <BR> /input_cmd_vel /move_base/goal /scan

# \Publishes to: <BR>
# /cmd_vel
# /move_base/goal

# \Services : <BR>
# /gazebo/reset_world

# \Description :
# The code enables the robot (car) to take user input and collect car's position and give the velocity and the information on the goal.

# import ros stuff
import rospy
import time
import os
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseActionFeedback
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalID

# global variable
## In the manual mode, user can get out of the loop when he pushes "p." Here, we set the value as "a" in initial state 
initial_cmd = 'a'
## range as arriving at the goal  
dis_th = 0.5
## threshold for collision avoidance 
obj_th = 1.0
## 1 minute is the maximum time for the car to reach the goal  
time_th = rospy.Duration(60)
## defining vel_msg 
vel_msg = Twist()  

def menu_ui():
    '''!menu'''
    print('Could you select the driving mode?:\n\n')
    print('1: Auto_drive mode\n')
    print('2: Manual_drive mode\n')
    print('3: Assisted_drive mode\n')
    print('4: Cancel the goal\n')
    print('5: Reset car position\n')    
    print('0: Exit\n')

def get_user_input():
    '''!brief getting and checking user input and asking for another user input if it is ineffective'''
    while True:
        try:
            # when input is an integer, this funcion will return user selected mode
            user_select = int(input('user selected: '))
            break
        except:
            print('Could you type an integer number?')
    
    return user_select

def display_goal():
    '''!displaying the goal coordinate'''    
    if yesno_goal:
        print('The user decided the goal coordinate\nCar is reaching the goal automatically\n')
        print('Goal:  x = %.1f   y = %.1f\n\n' % (decided_x, decided_y))
    else:
        print('Goal is not decided yet\n\n')

def fix_goal():
    '''!In Auto_drive mode, fixing the goal coordinate'''     
    global yesno_goal, start_time
    # resetting the terminal
    os.system('clear')
    
    print('Auto_drive\n')    
    
    # The input about the goal's x coordinate is float
    while True:
        try:
            # when input is a float, this funcion will return the goal's x coordinate
            input_x = float(input('\nCould you type the x-coordinate for the goal?: '))
            break
        except:
            print('Could you type a number?')

    # The input about the goal's y coordinate is float
    while True:
        try:
            # when input is a float, this funcion will return the goal's y coordinate
            input_y = float(input('\nCould you type the y-coordinate for the goal?: '))
            break
        except:
            print('Could you type a number?')

    # initializing goal coordinate given by user
    init_goal_msg = MoveBaseActionGoal()
	
    # send a goal to the robot to move to the destination
    init_goal_msg.goal.target_pose.header.frame_id = "map"
    init_goal_msg.goal.target_pose.pose.position.x = input_x
    init_goal_msg.goal.target_pose.pose.position.y = input_y
    init_goal_msg.goal.target_pose.pose.orientation.w = 1.0    

    # publishing message and geting the time of reaching
    pub_goal.publish(init_goal_msg)
    start_time = rospy.Time.now() #line 104

    # goal stays decided
    yesno_goal = True

def receive_goal(msg):
    '''!these two values are stored and subscribed by move_base topic'''
    global decided_x, decided_y
    decided_x = msg.goal.target_pose.pose.position.x
    decided_y = msg.goal.target_pose.pose.position.y

def check_goal(msg):
    '''!checking whether the goal is reachable or not and notifying arrival'''
    global yesno_goal
    if yesno_goal:
        # time calculation
        end_time = rospy.Time.now()
        reaching_time = end_time - start_time #line 87
        # If it takes much time, destination will be recognized as unreachable and finish (cancel) the process
        if reaching_time > time_th:
            cancel_process()
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
            finish_process() 
            print('Reached the goal\n\n')               
            rospy.sleep(1)
            
def cancel_process():
    '''!cancelling the process before reaching the goal'''
    global yesno_goal
    # No goal to be cancelled
    if not yesno_goal:
        print('\nno goal to be cancelled\n\n')
        rospy.sleep(2)
     # cancel the goal
    else:
        cancel_msg = GoalID()
        pub_cancel.publish(cancel_msg)
        yesno_goal = False
        print('\nuser cancelled the goal\n\n') 
        rospy.sleep(2)     
	    
def finish_process():
    '''!finishing the process before reaching the goal'''
    global yesno_goal
    # No goal to be cancelled
    if not yesno_goal:
        print('\nno goal to be cancelled\n\n')
        rospy.sleep(2)
     # cancel the goal
    else:
        cancel_msg = GoalID()
        pub_cancel.publish(cancel_msg)
        yesno_goal = False       

def manual_driving():
    '''!with teleop_twist_keyboard, user can maneuver a car'''
    global initial_cmd
    os.system('clear')
    while initial_cmd != 'p':
        # printing the selected manual mode
        if drive_assist:
            print('\nAssisted_drive mode\nYou should go to "teleop_twist_keyboard.py" terminal to maneuver a car\n\n')
        else:
            print('\nManual_drive mode\nYou should go to "teleop_twist_keyboard.py" terminal to maneuver a car\n\n')

        # checking and getting user input for maneuvering a car. initial_cmd should be a char
        while True:
            try:
                initial_cmd = input('push p if you want to go back to main menu\n')
                break
            except:
                print('Could you choose a character?\n')

        # stop driving, before exiting the manual mode
        if initial_cmd == 'p':
            print('\nfinishing the manual mode\n')
            # initialize the velocity
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            # publishing the velocity
            pub_vel.publish(vel_msg)
        else:
            print('push p if you want to go back to main menu\n')

    # making initial_cmd the initial setting again which is "a"
    initial_cmd = 'a' 

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
    ### in teleop_twist_keyboard, the commands are shown below. 
    ### I thought about the cases when user pushes j,u,i,o,l and change these commands into k(staying at the same position)
    
    # moving around (commands)
    # u i o
    # j k l
    # m , .
        
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
    global vel_msg
    # in auto mode, this function is not effective
    if not manual_mode:
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
     
def choose_driving_mode(user_input):
    '''!choose_driving_mode'''
    global manual_mode, drive_assist

    # Auto_drive
    if user_input == 1:
        # fixing the goal based on user input
        fix_goal()

        if yesno_goal:
            os.system('clear')
            # displaying the goal
            display_goal()

    # Manual driving mode
    elif user_input == 2:
        # turn on manual mode
        manual_mode = True
        # NOT turn on assist mode
        drive_assist = False
        # run manual driving mode
        manual_driving()
        # turn off manual mode
        manual_mode = False

    # Assisted driving mode
    elif user_input == 3:
        # turn on manual mode
        manual_mode = True
        # turn on assist mode
        drive_assist = True
        # run assisted driving mode
        manual_driving()
        # turn off assist mode
        drive_assist = False
        # turn off manual mode        
        manual_mode = False

    # to cancel car position 
    elif user_input == 4:
    	cancel_process()
    	print('user cancelled the goal')
    
    # to reset car position
    elif user_input == 5:
        reset_world()
        print('going back to the initial position\n')

    # Exit the loop
    elif user_input == 0:
        # resetting the terminal
        os.system('clear')
        print('finishing the program')
        # shutting down nodes
        rospy.on_shutdown()

    # Not one of the possible options  
    else:
        # resetting the terminal
        os.system('clear')
        print('INVALID command.\n Could you push 0-5 numbers?:\n\n')
        menu_ui()
        
def main():
    '''!main'''
    global yesno_goal, drive_assist, manual_mode, user_input, goal_to_cancel
    global pub_goal, pub_vel, pub_cancel, sub_laser, sub_goal, sub_vel, sub_car_pos, reset_world

    #initializing the mode: no goal, no assist, no manual(equivalent to automatic) are the original state

    # initialize the goal which means that a car does not have a goal
    yesno_goal = False
    # initialize drive_assist which means that a car does not have a driving assistence
    drive_assist = False
    # initialize the mode which means that a car is not in manual mode
    manual_mode = False

    # initializing the node
    rospy.init_node('manu_ui')

    # resetting gazebo
    rospy.wait_for_service('/gazebo/reset_world')
    reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)

    # setting publishers
    pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=200)
    pub_goal = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=200)
    pub_cancel = rospy.Publisher('move_base/cancel', GoalID, queue_size=200)

    # setting subscribers
    sub_goal = rospy.Subscriber('move_base/goal', MoveBaseActionGoal, receive_goal) #getting feedback on goal
    sub_car_pos = rospy.Subscriber('move_base/feedback', MoveBaseActionFeedback, check_goal) # current position
    sub_laser = rospy.Subscriber('/scan', LaserScan, assisted_driving) # information on laser scanner
    sub_vel = rospy.Subscriber('/input_cmd_vel', Twist, actual_vel) # getting velocity


    # the loop keeps going on until user pushes "5" which is the reset button
    while not rospy.is_shutdown():
        # menu is called
        menu_ui()
        # get user input
        user_input = get_user_input()
        # Decide what to do based on user decision
        choose_driving_mode(user_input)


if __name__ == '__main__':
    main()
