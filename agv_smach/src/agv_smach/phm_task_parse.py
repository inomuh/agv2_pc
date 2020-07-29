#!/usr/bin/env python
import rospy
import json
from std_msgs.msg import String
import math

class TaskParsePub:
	def __init__(self):
		self.task = []
		self.control = False
		self.publisher()


	def callback(self,msg):
		gorev_sunucu = msg.data
		gorev_njson = json.loads(gorev_sunucu)
		self.task = []
		for i in range(len(gorev_njson["plan"])):
			robotId = gorev_njson["plan"][i]["robotID"]
			robotEquipmentID = gorev_njson["plan"][i]["robotEquipmentID"]
			robotTaskList = gorev_njson["plan"][i]["taskList"]
			for y in range(len(gorev_njson["plan"][i]["taskList"])):
				#print(y)
				waypoint = []
				signal = []
				robotTaskId = gorev_njson["plan"][i]["taskList"][y]["id"]
				robotTaskWorkBench = gorev_njson["plan"][i]["taskList"][y]["workbenchID"]
				robotTaskStationId = gorev_njson["plan"][i]["taskList"][y]["stationID"]
				robotTaskDuty = str(gorev_njson["plan"][i]["taskList"][y]["duty"])
				robotTaskProductName = str(gorev_njson["plan"][i]["taskList"][y]["productName"])
				robotTaskProductCount = gorev_njson["plan"][i]["taskList"][y]["productCount"]

				for z in range(len(gorev_njson["plan"][i]["taskList"][y]["route"])):

					robotTaskRouteTagTypeDef = str(gorev_njson["plan"][i]["taskList"][y]["route"][z]["tags"]["type_definition"])
					robotTaskRouteX = gorev_njson["plan"][i]["taskList"][y]["route"][z]["x"]
					robotTaskRouteY = gorev_njson["plan"][i]["taskList"][y]["route"][z]["y"]
					robotTaskRouteYaw = float(eval(gorev_njson["plan"][i]["taskList"][y]["route"][z]["yaw"]))
					waypoint.append([robotTaskRouteX,robotTaskRouteY,robotTaskRouteYaw])
					signal.append([robotTaskRouteTagTypeDef])

				taskDraft = [robotTaskId,robotTaskStationId,robotTaskDuty,robotTaskProductName,robotTaskProductCount,waypoint,signal]

				self.task.append(taskDraft)

		self.control = True


	def parser(self, task_list):
		parser_list = list()

		for i in range(len(task_list)):
			temp_docking_id = task_list[i][1]
			temp_duty = task_list[i][2]
			temp_product_name = task_list[i][3]
			temp_product_count = task_list[i][4]
			temp_coordinate_list = task_list[i][5]
			temp_state_list = task_list[i][6]

			for j in range(len(temp_coordinate_list)):
				parser_dict = dict()

				if temp_state_list[j][0] == 'Docking':
					parser_dict = {	"State": temp_state_list[j][0], 
									"Waypoint": 
												{"position": 
															{"x": temp_coordinate_list[j][0], 
															"y": temp_coordinate_list[j][1]
															}, 
												"yaw": temp_coordinate_list[j][2]
												},
									"Product": 
												{"Name": temp_product_name, 
												"Count": temp_product_count, 
												"ID": temp_docking_id, 
												"Duty": temp_duty
												}
									}

				else:
					parser_dict = {	"State": temp_state_list[j][0], 
									"Waypoint": 
												{"position": 
															{"x": temp_coordinate_list[j][0], 
															"y": temp_coordinate_list[j][1]
															}, 
												"yaw": temp_coordinate_list[j][2]
												},
									}

				parser_list.append(parser_dict)
				
		
		return parser_list


	def publisher(self):
		self.subscriber = rospy.Subscriber("gorev", String, self.callback)
		self.pub = rospy.Publisher('robot_task', String, queue_size=10)
		self.rate = rospy.Rate(2)

		msg = String()
		while not rospy.is_shutdown():
			if self.control:
				test = self.parser(self.task)
				print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n\n")
				print(test)
				print("\n\n")
				msg.data = str(test)
				self.pub.publish(msg.data)
				self.control = False

			self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('gorev_parse', anonymous=True)

    TaskParsePub()
