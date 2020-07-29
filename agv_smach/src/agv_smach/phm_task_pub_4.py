#!/usr/bin/env python
# license removed for brevity
import rospy
import json
from std_msgs.msg import String
import math

gorev_sunucu ={
  "plan": [
    {
      "robotID": 1,
      "robotEquipmentID": 1,
      "taskList": [
        {
          "id": 0,
          "workbenchID": 0,
          "stationID": 0,
          "duty": "null",
          "productName": "null",
          "productCount": 0,
          "route": [
            {
              "tags": {
                "type": "point",
                "type_code": "2",
                "type_definition": "Navigation"
              },
              "x": 4.25,
              "y": 2.18,
              "yaw": "math.pi/2",
              "id": -113499
            },
            {
              "tags": {
                "type": "point",
                "type_code": "3",
                "type_definition": "Navigation"
              },
              "x": 5.55,
              "y": 3.54,
              "yaw": "math.pi/4",
              "id": -107782
            }, 
            {
              "tags": {
                "type": "point",
                "type_code": "5",
                "type_definition": "Navigation"
              },
              "x": 10.34,
              "y": 3.91,
              "yaw": "0.0",
              "id": -113507
            },
            {
              "tags": {
                "type": "point",
                "type_code": "6",
                "type_definition": "Navigation"
              },
              "x": 11.05,
              "y": 3.67,
              "yaw": "-math.pi/4",
              "id": -107834
            },
            {
              "tags": {
                "type": "point",
                "type_code": "6",
                "type_definition": "Navigation"
              },
              "x": 11.42,
              "y": 2.76,
              "yaw": "-math.pi/2",
              "id": -107834
            },
            {
              "tags": {
                "type": "point",
                "type_code": "8",
                "type_definition": "Navigation"
              },
              "x": 11.42,
              "y": 1.93,
              "yaw": "-math.pi/2",
              "id": -113515
            },
            {
              "tags": {
                "type": "point",
                "type_code": "17",
                "type_definition": "Navigation"
              },
              "x": 10.97,
              "y": 1.08,
              "yaw": "-3*math.pi/4",
              "id": -107931
            },
            {
              "tags": {
                "type": "point",
                "type_code": "68",
                "type_definition": "Navigation"
              },
              "x": 8.16,
              "y": 0.82,
              "yaw": "-math.pi",
              "id": -112449
            },
            {
              "tags": {
                "type": "point",
                "type_code": "68",
                "type_definition": "Navigation"
              },
              "x": 6.35,
              "y": 0.83,
              "yaw": "-math.pi",
              "id": -112449
            },
            {
              "tags": {
                "type": "point",
                "type_code": "69",
                "type_definition": "Parking"
              },
              "x": 4.26,
              "y": 0.68,
              "yaw": "math.pi/2",
              "id": -112452
            }
          ]
        }
      ]
    }
  ]
}
gorev_ros = json.dumps(gorev_sunucu)
def talker():
    pub = rospy.Publisher('gorev', String, queue_size=10)
    rospy.init_node('task_pub')
    count = 1
    rate = rospy.Rate(2) # 10hz
    while not rospy.is_shutdown():
      hello_str = gorev_ros
      if count<=30 :
        pub.publish(hello_str)
        rospy.loginfo(hello_str)
        print("************************************************%d"%count)
        count=count+1
      else:
          print("///////////////////////////////////////////////////////%d"%count)
      rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
