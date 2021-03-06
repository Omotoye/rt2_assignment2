#! /usr/bin/env python

"""
.. module:: go_to_point
    :platform: Unix
    :synopsis: Python module for control of a mobile robot to navigate to a target point
.. moduleauthor:: Omotoye Adekoya adekoyaomotoye@gmail.com 
This node controls a mobile robot to move from it position to some target position
Subscribes to:
    /odom topic where the simulator publishes the robot position
Publishes to: 
    /cmd_vel velocity to move to the desired robot positions
    
Service:
    /go_to_point_switch accepts a request to go to a target position 
    
"""

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
import actionlib
import rt2_assignment2.msg
import math
import time 

# robot state variables
position_ = Point()
yaw_ = 0
position_ = 0
state_ = 0
pub_ = None
_as = None
pose = None

# Initializing the variables for holding the data that'll be published in the 
# parameter server for the analysis plot
canceled_target = 0
reached_target = 0
goal_time_list = []
target_point = []

# parameters for control
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = -3.0
kp_d = 0.2

def ub_a():
    """This is a function for increasing or decreasing the 
    value of the angular speed the robot would move at based
    on the value gotten from the slider in the Notebook UI

    Returns:
        [float]: value of the default speed muliplied by the 
        multiplier set from the Notebook UI
    """
    if (rospy.has_param('/angular_velocity')):
        av_multiplier = rospy.get_param('/angular_velocity')
    else:
        av_multiplier = 1
    return (av_multiplier*0.6)
    
        
def lb_a():
    """This is a function for increasing or decreasing the 
    value of the linear speed the robot would move at based
    on the value gotten from the slider in the Notebook UI

    Returns:
        [float]: value of the default speed muliplied by the 
        multiplier set from the Notebook UI
    """
    if (rospy.has_param('/angular_velocity')):
        av_multiplier = rospy.get_param('/angular_velocity')
    else:
        av_multiplier = 1
    return (av_multiplier*(-0.5))

def ub_d():
    """This is a function for increasing or decreasing the 
    value of the angular speed the robot would move at based
    on the value gotten from the slider in the Notebook UI

    Returns:
        [float]: value of the default speed muliplied by the 
        multiplier set from the Notebook UI
    """
    if (rospy.has_param('/linear_velocity')):
        lv_multiplier = rospy.get_param('/linear_velocity')
    else:
        lv_multiplier = 1
    return (lv_multiplier*0.6)

# create messages that are used to publish feedback/result
_feedback = rt2_assignment2.msg.PositionFeedback()
_result = rt2_assignment2.msg.PositionResult()


def ui_param_data(value, state):
    """This function is used for processing the value of the data
    sent to the Notebook UI based on the state argument provided to it

    Args:
        value ([int]): This is the value to be sent to the Notebook Ui,
        based on the state provided. 
        state ([int]): this determines the type of data that's in the 
        value argument. 
    """
    global canceled_target, reached_target, goal_time_list, target_point
    if (state == 0):
        canceled_target += 1
        rospy.set_param('/canceled_target', canceled_target)
    if (state == 1):
        reached_target += 1
        rospy.set_param('/reached_target', reached_target)
    if (state == 2):
        goal_time_list.append(value) 
        rospy.set_param('/target_time', goal_time_list)
    if (state == 3):
        target_point.append(value)
        if (len(target_point) == 3):
            rospy.set_param('/target_point', target_point)
            target_point = []


def check_preempt():
    """This is function is used for checking if a preemption has be
    requested from the UI node. 

    Returns:
        [bool]: True if preempt is request and False otherwise. 
    """
    # check that preempt has not been requested by the client
    if _as.is_preempt_requested():
        print('The Goal has been Preempted')
        _as.set_preempted()
        ui_param_data(0, 0)
        done()
        return True
    return False


def clbk_odom(msg):
    """Callback function for handling Odometry message comming from the
    odom topic 

    Args:
        msg ([Odometry]): The Odometry message coming from the odom topic. 
    """
    global position_
    global yaw_
    global pose

    # position for feedback
    pose = msg
    _feedback.pose = pose

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def change_state(state):
    global state_
    state_ = state
    print('State changed to [%s]' % state_)


def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


def fix_yaw(des_pos):
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a():
            twist_msg.angular.z = ub_a()
        elif twist_msg.angular.z < lb_a():
            twist_msg.angular.z = lb_a()
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        # print ('Yaw error: [%s]' % err_yaw)
        change_state(1)


def go_straight_ahead(des_pos):
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.3
        if twist_msg.linear.x < ub_d():
            twist_msg.linear.x = ub_d()

        twist_msg.angular.z = kp_a*err_yaw
        pub_.publish(twist_msg)
    else:  # state change conditions
        # print ('Position error: [%s]' % err_pos)
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        # print ('Yaw error: [%s]' % err_yaw)
        change_state(0)


def fix_final_yaw(des_yaw):
    err_yaw = normalize_angle(des_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a():
            twist_msg.angular.z = ub_a()
        elif twist_msg.angular.z < lb_a():
            twist_msg.angular.z = lb_a()
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        # print ('Yaw error: [%s]' % err_yaw)
        change_state(3)


def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub_.publish(twist_msg)


def go_to_point(goal):
    """This is a callback function that handles the go to point action, 
    it calls all other functions that helps it achieve the goal of getting 
    to the goal pose. 

    Args:
        goal : This is an object that contains the target pose that the robot is
        required to reach. 
    """
    desired_position = Point()
    desired_position.x = goal.x
    ui_param_data(goal.x, 3)
    desired_position.y = goal.y
    ui_param_data(goal.y, 3)
    des_yaw = goal.theta
    ui_param_data(goal.theta, 3)

    change_state(0)
    start_time = time.time()
    while True:
        _as.publish_feedback(_feedback)

        if state_ == 0:
            fix_yaw(desired_position)
            if (check_preempt()):
                break
        elif state_ == 1:
            go_straight_ahead(desired_position)
            if (check_preempt()):
                break
        elif state_ == 2:
            fix_final_yaw(des_yaw)
            if (check_preempt()):
                break
        elif state_ == 3:
            done()
            end_time = time.time()
            time_elapsed = end_time - start_time
            _result.ok = True
            _as.set_succeeded(_result)
            ui_param_data(time_elapsed, 2)
            ui_param_data(1,1)
            break


def main():
    global pub_
    global _as
    rospy.init_node('go_to_point')
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    _as = actionlib.SimpleActionServer(
        'go_to_point', rt2_assignment2.msg.PositionAction, execute_cb=go_to_point, auto_start=False)
    _as.start()
    rospy.spin()


if __name__ == '__main__':
    main()
