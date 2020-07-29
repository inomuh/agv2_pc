# agv2_pc


sudo apt-get install ros-kinetic-navigation

sudo apt-get install ros-kinetic-gmapping

sudo apt-get install ros-kinetic-timed-roslaunch

sudo apt-get install ros-kinetic-dynamic-reconfigure

sudo apt-get install ros-kinetic-smach-viewer


cd ~/<WORKSPACE_NAME>/src/agv2_pc/agv2_description/meshes/22-10-2019-OTAv07

unzip OTA-v0.7.stl.zip



gedit ~/.bashrc

export GAZEBO_MODEL_PATH=~/<WORKSPACE_NAME>/src/agv2_pc/building_editor_models:~/<WORKSPACE_NAME>/src/agv2_pc/model_editor_models:$GAZEBO_MODEL_PATH



cd <WORKSPACE_NAME>

catkin_make

catkin_make install


Start Simulation

roslaunch agv2_start start_phm.launch


Task Parser

rosrun agv_smach phm_task_parse.py


Task Publishers

rosrun agv_smach phm_task_pub_1.py

rosrun agv_smach phm_task_pub_2.py

rosrun agv_smach phm_task_pub_3.py

rosrun agv_smach phm_task_pub_4.py