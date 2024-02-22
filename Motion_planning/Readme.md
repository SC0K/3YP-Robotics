# Modified Pick and Place Demo with Table Cleaning Task

To run the simulation:
  1. Setup a workspace for Tiago robot by following the steps in [Installation tutorial](https://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/InstallUbuntuAndROS)
  2. Download the package and replace the `tiago_pick_demo` folder in `~/<your-workspace>/src/tiago_tutorials` directory.
  3. The programs are in the `scripts` folder.
  4. `plan_arm_torso.py`, `clean_test.py`, and `clean_cartesian.py` (using cartesian paths) are examples for MoveIt! Python Interface.
  5. Follow the steps in [TIAGo Tutorials - Pick & Place demo](https://wiki.ros.org/Robots/TIAGo/Tutorials/MoveIt/Pick_place) to launch the simulation.


To run the state-machine and path planning simulation (this should integrate with motion planning as well)
1. if not already done, do steps 2 and 3 from above
2. copy the navigation launch file: `cp ./tiago_navigation2.launch ~/<your-workspace>/src/tiago_simulation/tiago_2dnav_gazebo/launch`
3. launch the new navigation file: `roslaunch tiago_2dnav_gazebo tiago_navigation2.launch public_sim:=true`
4. start the state machine: `rosrun tiago_pick_demo tgsm.py`
5. The robot should now move back and forth between the base and one other location

When adding new locations to the map, *I think* the coordinates are relative to the spawn location, but this should be checked.