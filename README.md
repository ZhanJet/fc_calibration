# fc_calibration

Force control calibration of sensor bias and load mass properties.

Running for simulation need package of ur5_description, which can be cloned firstly by:
"git clone https://github.com/utecrobotics/ur5"
Then you need to delete the node launch of "joint_state_publisher" in the display.launch in the pkg of ur5_description


## Example usage

cd ~/catkin_ws
catkin build fc_calibration ur5_description
roslaunch fc_calibration fc_calibration.launch

## Running tests/demos

