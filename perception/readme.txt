To set up:
assume tiago_public_ws has been set up already, with kitchen2.world copied into src/pal_gazebo_worlds/worlds/

copy packages for apriltags and interpreting this info into tiago_public_ws:
	cd <path_to_tiago_public_ws>/src
	git clone https://github.com/AprilRobotics/apriltag_ros.git
	cd <path_to_3yp>/perception
	cp  ./continuous_detection2.launch <path_to_tiago_public_ws>/src/apriltag_ros/apriltag_ros/launch/
	cp -r ./apriltag_to_table_info <path_to_tiago_public_ws>/src/

to launch nodes (may need to run catkin build the first time):
in each terminal:
	cd <path_to_tiago_public_ws>
	source ./devel/setup.bash

in terminal0
	roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true world:=kitchen2

in terminal1
	roslaunch apriltag_ros continuous_detection.launch

this publishes to /tag_detections, and then this information is processed and transformed in a subscriber to this topic
for now, this is just being logged, although this will be changed later

in terminal2
	rosrun apriltag_to_table_info detections_to_table_info.py

once an apriltag has been detected, information about this apriltag should be printed to the screen
