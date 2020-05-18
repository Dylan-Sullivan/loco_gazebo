# LoCO Gazebo

This package allows for simulation of LoCO using Gazebo.

## System Setup(s) Verified

- ROS: Melodic
- Gazebo: 9.12.0-1~bionic
- Python: 2.7

## Package Downloads

To install these packages, use the command `sudo apt-get install ros-melodic-insert_package_name`

- teleop-twist-keyboard
- mavros
- liburdfdom-tools
- joint-state-publisher-gui

## Environment Setup

Since development of the simulation resides in various packages and there is not a workspace available for download, a workspace must first be created. Instructions for this can be found at `http://wiki.ros.org/catkin/Tutorials/create_a_workspace`. It is recommended to name the workspace 'loco_ws' rather than 'catkin_ws' for keeping the workspace identifiable.

The four packages required to operate the simulation are listed below. The repositories should be cloned and located under the 'loco_ws/src/' path.

- loco_description: `https://github.umn.edu/loco-auv/loco_description`
- loco_gazebo: `https://github.umn.edu/loco-auv/loco_gazebo` (though you are already here if you are reading this)
- loco_pilot: `https://github.umn.edu/loco-auv/loco_pilot`
- loco_teleop: `https://github.umn.edu/loco-auv/loco_teleop`

Once these packages have been installed, navigate back to the 'loco_ws' folder and enter the command `catkin_make` to finish configuring the workspace.

## Running the Interactive Simulation

To begin running the interactive simulation, first, create a terminal window (or tab) and enter the command `roscore` to initialize ROS. Open a new window and navigate to the 'loco_ws' folder and enter the command `catkin_make` to ensure the workspace is up to date, then `source devel/setup.bash` to source the setup. Any time a new terminal window or tab is created to run something from this workspace, the `source devel/setup.bash` command should be performed first.

### Launch Simulation in Gazebo
In a new tab,
`roslaunch loco_gazebo loco_gazebo.launch`
### Launch Simulation Control Node
In a new tab,
`rosrun loco_gazebo sim_control_node.py`
### Teleoperation
Currently, only keyboard teleoperation of the the simulation has been tested. In a new tab,
`rosrun loco_teleop teleop_keyboard.py`

Teleoperation directions are given in the terminal window and will reappear after enough messages have been output. Control is meant to resemble the left-hand joystick of a controller, where the 's' key is the middle position, and any inputs around that key provide directional commands. For example, 'w' is forward, 'e' provides a gradual turn forward and to the right, and 'd' is to turn directly forward to the right.

Press the play button in the lower left of the simulation window in Gazebo to begin the simulation. Click back onto the teleoperation terminal space and then LoCO can be operated.

## Loading Different Gazebo Worlds
The default world for the LoCO simulation is a pool-like environment. To load a different world, when launching the simulation in gazebo, add the 'world_name' launch file argument to the launch command:
`roslaunch loco_gazebo loco_gazebo.launch world_name:='insert_world_name'`

For example, to launch an empty world with LoCO:
`roslaunch loco_gazebo loco_gazebo.launch world_name:='empty_world.world'`

The world files tested and included with the simulation can be found in the 'loco_gazebo' package under the 'worlds' folder.

## Particle Filter Test Program

There is currently a test implementation of particle filter localization based on bathymetric data. To run this test program, first, create a terminal window (or tab) and navigate to the 'loco_ws' folder and enter the command `catkin_make` to ensure the workspace is up to date, then `source devel/setup.bash` to source the setup. Any time a new terminal window or tab is created to run something from this workspace, the `source devel/setup.bash` command should be performed first.

### Launch Simulation in Gazebo
In a new tab,
`roslaunch loco_gazebo pf_test.launch`
### Launch Simulation Control Node
In a new tab,
`rosrun loco_gazebo sim_control_node.py`
### Launch Test Program
In a new tab,
`rosrun loco_gazebo pf_test.py`

Now, the Gazebo simulation window should show LoCO in front of a staircase, and the particle filter test is ready to run. Press play in the bottom left of the screen. The test does not stop automatically, so once LoCO completes the staircase, the pause button in the bottom left of the screen can be pressed.
