#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from trajectory_msgs.msg import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry
import dynamic_reconfigure.client

import math

class GoToPose():
    def __init__(self):

        self.goal_sent = False

	rospy.on_shutdown(self.shutdown)
	
	# Tell the action client that we want to spin a thread by default
	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	rospy.loginfo("Starting the AGV1 Navigation...")

	# Allow up to 5 seconds for the action server to come up
	self.move_base.wait_for_server(rospy.Duration(5))

    def goto(self, pos, quat):

        # Send a goal
        self.goal_sent = True
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

	# Start moving
        self.move_base.send_goal(goal)

	# Allow AGV1 up to 60 seconds to complete task
	success = self.move_base.wait_for_result(rospy.Duration(60)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def nav_goto(self, pos, quat):

        # Send a goal
        self.goal_sent = True
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

	# Start moving
        self.move_base.send_goal(goal)

    def odom_callback(self, msg_odom):
        self.x = msg_odom.pose.pose.position.x
        self.y = msg_odom.pose.pose.position.y 
        self.odom_control = True

    def odom_compare(self, position):
        self.odom_control = False
        self.subscriber = rospy.Subscriber('/agv1/odom', Odometry, self.odom_callback)
        self.rate = rospy.Rate(2)

        while not rospy.is_shutdown():
            if self.odom_control & (position['x']!=4.26):
                distance = math.sqrt( math.pow((float(position['x']) -self.x),2) + (math.pow((float(position['y']) - self.y), 2)))
                if abs(distance) < 0.6:    
                    return True
            else:
                client = dynamic_reconfigure.client.Client("agv1_move_base/DWAPlannerROS")
                client.update_configuration({'xy_goal_tolerance': 0.1, 'yaw_goal_tolerance': 0.1})                
            """
            else:
                return False
            """
            self.rate.sleep()


    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stopping the AGV1")
        rospy.sleep(1)



def dynamic_waypoint(navigator, waypoints, coordinat):
    for i in range(len(waypoints)):
        current_waypoint = coordinat[waypoints[i]]
        position = current_waypoint["position"]
        quaternion = current_waypoint["quaternion"]
        tolerance = 0.05

        rospy.loginfo("Go to desired pose: (%s, %s)", position['x'], position['y'])
        success = navigator.goto(position, quaternion)

    return success, position


def dynamic_target_position(position, quaternion):
    navigator = GoToPose()

    rospy.loginfo("Go to desired pose: (%s, %s)", position['x'], position['y'])

    success = navigator.goto(position, quaternion)

    if success:
        rospy.loginfo("AGV1 Reached The Desired Pose: (%s, %s)", position['x'], position['y'])
    else:
        rospy.loginfo("AGV1 failed to reach the desired pose!")

    return success


def dynamic_target_position_and_yaw(position, yaw):
    navigator = GoToPose()

    quaternion = ToQuaternion(yaw, 0.0, 0.0)
    rospy.loginfo("Go to desired pose: (%s, %s)", position['x'], position['y'])

    success = navigator.goto(position, quaternion)

    if success:
        rospy.loginfo("AGV1 Reached The Desired Pose: (%s, %s)", position['x'], position['y'])
    else:
        rospy.loginfo("AGV1 failed to reach the desired pose!")

    return success


def nav_dynamic_target_position_and_yaw(position, yaw):
    navigator = GoToPose()

    quaternion = ToQuaternion(yaw, 0.0, 0.0)
    
    navigator.nav_goto(position, quaternion)

def nav_dynamic_waypoint(navigator, waypoints, coordinat):
    for i in range(len(waypoints)):
        current_waypoint = coordinat[waypoints[i]]
        position = current_waypoint["position"]
        quaternion = current_waypoint["quaternion"]
        tolerance = 0.05

        rospy.loginfo("Go to desired pose: (%s, %s)", position['x'], position['y'])
        navigator.nav_goto(position, quaternion)
        success = navigator.odom_compare(position)
    return success,position


def ToQuaternion(yaw, pitch, roll):
    cy = float(math.cos(float(yaw) * 0.5))
    sy = float(math.sin(float(yaw) * 0.5))
    cp = float(math.cos(float(pitch) * 0.5))
    sp = float(math.sin(float(pitch) * 0.5))
    cr = float(math.cos(float(roll) * 0.5))
    sr = float(math.sin(float(roll) * 0.5))


    quaternion = {
        "r1": (cy * cp * sr - sy * sp * cr),
        "r2": (sy * cp * sr + cy * sp * cr),
        "r3": (sy * cp * cr - cy * sp * sr),
        "r4": (cy * cp * cr + sy * sp * sr)
    }

    return quaternion




if __name__ == '__main__':
    try:
        rospy.init_node('agv1_nav', anonymous=False)
        navigator = GoToPose()

        #tolerance = 0.05
        coordinat = dict(rospy.get_param("~Waypoints"))

        
        waypoints_test_3 = ["P1", "P2", "P3", "P7", "P8","P9","P10","P11","P12","P13","P14","P17","P18","P19"]

        waypoints_test_4 = ["P1", "P2", "P3", "P4","P5","P6","P15","P16","P17","P18","P19"]

        longshort = ["P2", "P3", "P4","P7", "P8","P9","P10","P11","P12","P13","P14","P17","P18","P19","P20","P1","P2","P3","P4","P5","P6","P15","P16","P17","P18","PD1"]
        longlong = ["P1", "P2", "P3", "P4","P7", "P8","P9","P10","P11","P12","P13","P14","P17","P18","P19","P20","P1", "P2", "P3", "P4","P7", "P8","P9","P10","P11","P12","P13","P14","P17","P18","P19","P20"]
        longshortdoc = ["RKD1","RKD2","P2", "P3", "t1d1","P7", "P8","P9","P10","P11","P12","P13","P14","P17","P18","P19","P20","P1","P2","P3","P4","P5","P6","P15","P16","P17","P18","PD1"]

        waypoints = longshort

        success, position = nav_dynamic_waypoint(navigator, waypoints, coordinat)
        if success:
            rospy.loginfo("AGV1 Reached The Desired Pose: (%s, %s)", position['x'], position['y'])
        else:
            rospy.loginfo("AGV1 failed to reach the desired pose!")

        rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("Quit")

