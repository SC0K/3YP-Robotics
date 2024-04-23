# Modified Pick and Place Demo with Table Cleaning Task

To run the simulation:
  1. Setup a workspace for Tiago robot by following the steps in [Installation tutorial](https://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/InstallUbuntuAndROS)
  2. Download the package and replace the `tiago_pick_demo` folder in `~/<your-workspace>/src/tiago_tutorials` directory.
  3. The programs are in the `scripts` folder.
  4. `plan_arm_torso.py`, `clean_test.py`, and `clean_cartesian.py` (using cartesian paths) are examples for MoveIt! Python Interface.
  5. Follow the steps in [TIAGo Tutorials - Pick & Place demo](https://wiki.ros.org/Robots/TIAGo/Tutorials/MoveIt/Pick_place) to launch the simulation.
  6. Use `roslaunch tiago_2dnav_gazebo tiago_navigation2.launch ` to launch the simulation with path planning instead.
  7. Run `roslaunch tiago_pick_demo tgsm.py` to run the task planning.
  
