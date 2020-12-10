# fc_calibration

Calibration of F/T sensor bias and load mass properties for Force control.

The package of ur5_description is needed for simulation, which can be cloned firstly by:
"git clone https://github.com/utecrobotics/ur5" 
 
Then you need to cancel the node launch of "joint_state_publisher" in display.launch loacated in pkg of ur5_description


## Example usage
cd ~/catkin_ws  
catkin build fc_calibration ur5_description   
roslaunch fc_calibration fc_calibration.launch   

## Running tests/demos

