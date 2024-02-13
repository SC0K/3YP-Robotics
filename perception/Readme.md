# Setup Instructions

To set up, follow these steps:

### Initial Assumptions
- Assume `tiago_public_ws` has been set up already, with `kitchen2.world` copied into `src/pal_gazebo_worlds/worlds/`.

### Copying Packages for AprilTags
- Copy packages for apriltags and interpreting this information into `tiago_public_ws`:
  1. Navigate to the `src` directory of `tiago_public_ws`:
     ```bash
     cd <path_to_tiago_public_ws>/src
     ```
  2. Clone the `apriltag_ros` repository:
     ```bash
     git clone https://github.com/AprilRobotics/apriltag_ros.git
     ```
  3. Move to your 3yp project's perception directory:
     ```bash
     cd <path_to_3yp>/perception
     ```
  4. Copy the necessary launch files and packages:
     ```bash
     cp ./continuous_detection2.launch <path_to_tiago_public_ws>/src/apriltag_ros/apriltag_ros/launch/
     cp -r ./apriltag_to_table_info <path_to_tiago_public_ws>/src/
     ```

### Launching Nodes
- To launch nodes (you may need to run `catkin build` the first time):
  1. In each terminal, navigate to `tiago_public_ws` and source the setup file:
     ```bash
     cd <path_to_tiago_public_ws>
     source ./devel/setup.bash
     ```
  2. In Terminal 0, launch TIAGo Gazebo:
     ```bash
     roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true world:=kitchen2
     ```
  3. In Terminal 1, launch AprilTag ROS:
     ```bash
     roslaunch apriltag_ros continuous_detection.launch
     ```
     This publishes to `/tag_detections`, and then this information is processed and transformed in a subscriber to this topic. For now, this is just being logged, although this will be changed later.
  4. In Terminal 2, run the `detections_to_table_info` script:
     ```bash
     rosrun apriltag_to_table_info detections_to_table_info.py
     ```
     Once an apriltag has been detected, information about this apriltag should be printed to the screen.
