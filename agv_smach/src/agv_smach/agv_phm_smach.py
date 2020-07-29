#!/usr/bin/env python 

import roslib
import rospy
import smach
import smach_ros
import math
import random

from smach import State
from smach import StateMachine

import dynamic_reconfigure.client
import agv_waypoint_navigation as agv_nav
from std_msgs.msg import String
from nav_msgs.msg import Odometry


from actionlib import *
from actionlib_msgs.msg import *


class general_selection_state(smach.State):
    
    def __init__(self):
        smach.State.__init__(self,  outcomes=['Navigation', 'Docking', 'Parking', 'Idle'],
                                    input_keys=['task_input'],
                                    output_keys=['task_output', 'current_task_output'])
        
        


    def execute(self, userdata):
        print("\n\n")
        print("General Select State")
        print("\n\n")

        temp_task_list = userdata.task_input

        if len(temp_task_list) == 0:
            self.odom_func()
            idle_standby_state.position_list.append([self.x, self.y])
            temp_time = idle_standby_state.time_variable
            temp_time_list = idle_standby_state.time_list
            temp_position = idle_standby_state.position_list

            self.publish_func(temp_time_list, temp_position)

            idle_standby_state.time_variable = 0
            idle_standby_state.position_list = list()
            idle_standby_state.time_list = list()
            return 'Idle'

        else:
            self.odom_func()
            idle_standby_state.position_list.append([self.x, self.y])

            docking_count = int(docking_states.docking_count)
            if docking_count > 0:
                for i in range(docking_count):
                    idle_standby_state.position_list.append([self.x, self.y])

                docking_states.docking_count = 0

            temp_task = temp_task_list[0]
            userdata.task_output = temp_task_list
            userdata.current_task_output = idle_standby_state.position_list

            return temp_task['State']


    def publish_func(self, temp_time, temp_position):
        self.pub_time = rospy.Publisher('task_time', String, queue_size=10)
        self.pub_position = rospy.Publisher('task_position', String, queue_size=10)

        msg_time = String()
        msg_position = String()

        msg_time.data = str(temp_time)
        msg_position.data = str(temp_position)

        self.pub_time.publish(msg_time)
        self.pub_position.publish(msg_position)


    def odom_callback(self, msg_odom):
        self.x = msg_odom.pose.pose.position.x
        self.y = msg_odom.pose.pose.position.y
        self.odom_control = True

    def odom_func(self):
        self.odom_control = False
        self.subscriber = rospy.Subscriber('/agv1/odom', Odometry, self.odom_callback)
        self.rate = rospy.Rate(2)

        while not self.odom_control:
            self.rate.sleep()


class idle_standby_state(smach.State):
    position_list = list()
    time_variable = 0
    time_list = list()
    def __init__(self):
        smach.State.__init__(self,  outcomes=['succeeded', 'aborted'],
                                    input_keys=['task_input'],
                                    output_keys=['task_output', 'current_task_output'])
        
        self.control = False

    def robot_task_func(self, data):
        if data.data != "":
            self.task_dict = list(eval(data.data))
            self.control = True

    def execute(self, userdata):
        rospy.loginfo("Idle Standby")
        self.control = False    # Buraya Eklendi

        if len(list(userdata.task_input)) == 0:
            self.pub_task = rospy.Publisher('robot_task_list', String, queue_size=10)
            self.subscriber = rospy.Subscriber('robot_task', String, self.robot_task_func)


            self.rate = rospy.Rate(2)

            counter = 0
            print("\n\n")
            rospy.loginfo("Wait Task")
            print("\n\n")
            userdata.current_task_output = "Wait Task"

            while not rospy.is_shutdown():
                counter += 1

                if self.control:
                    self.publish_robot_task(self.task_dict)
                    userdata.task_output = self.task_dict
                    rospy.loginfo("\n\n@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n\n-> Task Accepted\n\n@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n\n")
                    rospy.loginfo("Wait 3 sn")
                    time.sleep(3)
                    
                    return 'succeeded'

                elif self.control == False and counter == 50:
                    return 'aborted'


    def publish_robot_task(self, task_list):
        published_task_list = list()
        for item in task_list:
            temp_task = dict(item["Waypoint"]["position"])
            move_speed = float(random.uniform(0.25, 0.35))
            temp_array = [temp_task["x"], temp_task["y"], move_speed]
            published_task_list.append(temp_array)
        #pub_task = rospy.Publisher('robot_task_list', String, queue_size=10)
        #rospy.loginfo("\n\n@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n\n->         Robot Task \n\n@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n\n")
        #rospy.loginfo(str(published_task_list))
        msg_task = String()
        msg_task.data = str(published_task_list)       # task_list

        self.pub_task.publish(msg_task)


class wy_states(smach.State):
    def __init__ (self):
        smach.State.__init__(self,  outcomes=['succeeded', 'aborted'],
                                    input_keys=['task_input'],
                                    output_keys=['task_output', 'current_task_output'])

    def execute(self, userdata):
        start = time.time()
        print(userdata.task_input)

        temp_task_list = list(userdata.task_input)
        temp_task = temp_task_list.pop(0)
        userdata.task_output = temp_task_list

        userdata.current_task_output = temp_task

        client = dynamic_reconfigure.client.Client("agv1_move_base/DWAPlannerROS")
        client.update_configuration({'xy_goal_tolerance': 0.1, 'yaw_goal_tolerance': 0.5})


        print("Parameters Updated")
        
        position = temp_task["Waypoint"]["position"]
        yaw = temp_task["Waypoint"]["yaw"]
        if docking_states.docking_val:
            docking_states.docking_val = False
        else:
            client_2 = dynamic_reconfigure.client.Client("agv1_move_base/local_costmap/inflation_layer")
            client_2.update_configuration({'inflation_radius': 0.2})


        agv_nav.nav_dynamic_target_position_and_yaw(position, yaw)

        success = self.odom_compare(position)

        done = time.time()
        time_diff = done - start
        idle_standby_state.time_variable += time_diff
        idle_standby_state.time_list.append(time_diff)
        
        if success:
            return 'succeeded'

        else:
            return 'aborted'


    def odom_callback(self, msg_odom):
        self.x = msg_odom.pose.pose.position.x
        self.y = msg_odom.pose.pose.position.y 
        self.odom_control = True

    def odom_compare(self, position):
        self.odom_control = False
        self.subscriber = rospy.Subscriber('/agv1/odom', Odometry, self.odom_callback)
        self.rate = rospy.Rate(2)

        while not rospy.is_shutdown():
            if self.odom_control:
                distance = math.sqrt( math.pow((float(position['x']) -self.x),2) + (math.pow((float(position['y']) - self.y), 2)))
                if abs(distance) < 0.7:    
                    return True
            """
            else:
                return False
            """
            self.rate.sleep()




class docking_states(smach.State):
    docking_val = False
    docking_count = 0
    def __init__(self):
        smach.State.__init__(self,  outcomes=['succeeded', 'aborted'],  #'robot_arm', 
                                    input_keys=['task_input'],
                                    output_keys=['task_output', 'current_task_output'])

        self.tolerance = 0.1            #0.05
        
    def execute(self, userdata):
        start = time.time()
        print(userdata.task_input)

        temp_task_list = list(userdata.task_input)
        temp_task = dict(temp_task_list[0])
        userdata.task_output = temp_task_list

        userdata.current_task_output = temp_task

        client = dynamic_reconfigure.client.Client("agv1_move_base/DWAPlannerROS")
        client.update_configuration({'xy_goal_tolerance': self.tolerance, 'yaw_goal_tolerance': self.tolerance})

        client_2 = dynamic_reconfigure.client.Client("agv1_move_base/local_costmap/inflation_layer")
        client_2.update_configuration({'inflation_radius': 0.0})
        
        print("Parameters Updated")
        
        position = temp_task["Waypoint"]["position"]
        yaw = temp_task["Waypoint"]["yaw"]

        docking_val = True
        
        success = agv_nav.dynamic_target_position_and_yaw(position, yaw)


        done = time.time()
        time_diff = done - start
        idle_standby_state.time_variable += time_diff
        idle_standby_state.time_list.append(time_diff)

        if success:
            return 'succeeded'

        else:
            return 'aborted'


class robot_arm_states(smach.State):
    def __init__(self):
        smach.State.__init__(self,  outcomes=['succeeded', 'aborted'],
                                    input_keys=['task_input'],
                                    output_keys=['task_output', 'current_task_output'])

    def robot_arm_information(self, data):
        if data.status == 1:
            self.control = True
        
    def execute(self, userdata):
        start = time.time()
        temp_task_list = list(userdata.task_input)
        temp_task = temp_task_list.pop(0)
        userdata.task_output = temp_task_list
        
        userdata.current_task_output = temp_task

        time.sleep(3)
        

        done = time.time()
        time_diff = done - start
        idle_standby_state.time_variable += time_diff
        idle_standby_state.time_list.append(time_diff)

        # Yapilan docking islemi
        docking_states.docking_count += 1

        return 'succeeded'


class parking_states(smach.State):    
    def __init__ (self):
        smach.State.__init__(self,  outcomes=['succeeded', 'aborted'],
                                    input_keys=['task_input'],
                                    output_keys=['task_output', 'current_task_output'])

    def execute(self, userdata):
        start = time.time()
        print(userdata.task_input)

        temp_task_list = list(userdata.task_input)
        temp_task = temp_task_list.pop(0)
        userdata.task_output = temp_task_list

        userdata.current_task_output = temp_task

        client = dynamic_reconfigure.client.Client("agv1_move_base/DWAPlannerROS")
        client.update_configuration({'xy_goal_tolerance': 0.15, 'yaw_goal_tolerance': 0.05})

        client_2 = dynamic_reconfigure.client.Client("agv1_move_base/local_costmap/inflation_layer")
        client_2.update_configuration({'inflation_radius': 0.05})

        print("Parameters Updated")
        
        position = temp_task["Waypoint"]["position"]
        yaw = temp_task["Waypoint"]["yaw"]
        
        success = agv_nav.dynamic_target_position_and_yaw(position, yaw)
        
        #time.sleep(3)
        done = time.time()
        time_diff = done - start
        idle_standby_state.time_variable += time_diff
        idle_standby_state.time_list.append(time_diff)

        if success:
            return 'succeeded'

        else:
            return 'aborted'


class crash_repair_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                                    input_keys=['task_input'],
                                    output_keys=['task_output', 'current_task_output'])

    def execute(self, userdata):
        start = time.time()
        print("\n\n")
        print("Crash Repair")
        print("\n\n")

        time.sleep(3)

        done = time.time()
        time_diff = done - start
        idle_standby_state.time_variable += time_diff
        idle_standby_state.time_list.append(time_diff)

        return 'succeeded'


def main():
    rospy.init_node('agv_phm_smach')


    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['done', 'failed'])

    sm_top.userdata.task = list()

    with sm_top:
        
        smach.StateMachine.add('General_Selection', general_selection_state(),
                                transitions={'Navigation':'Navigation','Docking':'Docking', 'Parking':'Parking', 'Idle':'Idle'},
                                remapping={ 'task_input':'task',
                                            'task_output':'task',
                                            'current_task_output':'current_task'})


        # Create Navigation Container
        sm_navigation = smach.StateMachine(outcomes=['Nav_State_1', 'Nav_State_2'],
                                input_keys=['task'],
                                output_keys=['task', 'current_task'])

        with sm_navigation:

            smach.StateMachine.add('Navigation_Waypoint', wy_states(),
                                transitions={'succeeded':'Nav_State_1','aborted':'Nav_State_2'},
                                remapping={ 'task_input':'task',
                                            'task_output':'task',
                                            'current_task_output':'current_task'})            
                                
        smach.StateMachine.add('Navigation', sm_navigation,
                                transitions={'Nav_State_1':'General_Selection','Nav_State_2':'Crash'},
                                remapping={ 'task_input':'task',
                                            'task_output':'task',
                                            'current_task_output':'current_task'})
        

        # Create Docking Container
        sm_docking = smach.StateMachine(outcomes=['Docking_State_1', 'Docking_State_2'],
                                input_keys=['task'],
                                output_keys=['task', 'current_task'])

        with sm_docking:

            smach.StateMachine.add('Docking_Waypoint', docking_states(),
                                transitions={'succeeded':'Robot_Arm_Communication', 'aborted':'Docking_State_2'},   # transitions={'succeeded':'Docking_State_1', 'robot_arm':'Robot_Arm_Communication', 'aborted':'Docking_State_2'},
                                remapping={ 'task_input':'task',
                                            'task_output':'task',
                                            'current_task_output':'current_task'})

            smach.StateMachine.add('Robot_Arm_Communication', robot_arm_states(),
                                transitions={'succeeded':'Docking_State_1','aborted':'Robot_Arm_Communication'},
                                remapping={ 'task_input':'task',
                                            'task_output':'task',
                                            'current_task_output':'current_task'})

        smach.StateMachine.add('Docking', sm_docking,
                                transitions={'Docking_State_1':'General_Selection','Docking_State_2':'Crash'},
                                remapping={ 'task_input':'task',
                                            'task_output':'task',
                                            'current_task_output':'current_task'})


        # Create Parking Container
        sm_parking = smach.StateMachine(outcomes=['Park_State_1', 'Park_State_2'],
                                input_keys=['task'],
                                output_keys=['task', 'current_task'])

        with sm_parking:

            smach.StateMachine.add('Parking_Waypoint', parking_states(),
                                transitions={'succeeded':'Park_State_1','aborted':'Park_State_2'},
                                remapping={ 'task_input':'task',
                                            'task_output':'task',
                                            'current_task_output':'current_task'})

        smach.StateMachine.add('Parking', sm_parking,
                                transitions={'Park_State_1':'General_Selection','Park_State_2':'Crash'},
                                remapping={ 'task_input':'task',
                                            'task_output':'task',
                                            'current_task_output':'current_task'})


        # Create Idle Container
        sm_idle = smach.StateMachine(outcomes=['Idle_State_1', 'Idle_State_2'],
                                input_keys=['task'],
                                output_keys=['task', 'current_task'])

        with sm_idle:

            smach.StateMachine.add('Idle_Standby', idle_standby_state(),
                                transitions={'succeeded':'Idle_State_1', 'aborted':'Idle_Standby'},
                                remapping={ 'task_input':'task',
                                            'task_output':'task',
                                            'current_task_output':'current_task'})

        smach.StateMachine.add('Idle', sm_idle,
                                transitions={'Idle_State_1':'General_Selection','Idle_State_2':'Crash'},
                                remapping={ 'task_input':'task',
                                            'task_output':'task',
                                            'current_task_output':'current_task'})

        
        # Create Crash Container
        sm_crash = smach.StateMachine(outcomes=['Crash_State_1', 'Crash_State_2'],
                                input_keys=['task'],
                                output_keys=['task', 'current_task'])

        with sm_crash:

            smach.StateMachine.add('Crash_Repair', crash_repair_state(),
                                transitions={'succeeded':'Crash_State_1', 'aborted':'Crash_State_2'},
                                remapping={ 'task_input':'task',
                                            'task_output':'task',
                                            'current_task_output':'current_task'})

        smach.StateMachine.add('Crash', sm_crash,
                                transitions={'Crash_State_1':'General_Selection', 'Crash_State_2':'failed'},
                                remapping={ 'task_input':'task',
                                            'task_output':'task',
                                            'current_task_output':'current_task'})

        
    sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_TASK_SMACH')
    sis.start()
    sm_top.execute()
    rospy.spin()
    sis.stop()
        

if __name__ == '__main__':
    main()
