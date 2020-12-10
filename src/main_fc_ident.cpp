/*****************************************************************************
 *    file:	fc_identification_main.hpp
 *	  date:	Feb 16, 2020
*****************************************************************************/

#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <sensor_msgs/JointState.h>
#include <fc_calibration/FtSensorMeasure.h>

#include "fc_sensorloadid_ur.hpp"

//Todo: specify callback function of receiving topic data of ftsensor measure
// fc_calibration::FtSensorMeasure ft_raw;
// void ftSensorCB(const fc_calibration::FtSensorMeasure::ConstPtr& msg) {
// 	//Todo
// }

//Todo: receive joint position of robot
// void jntPosCB()


int main(int argc, char** argv) {
	ros::init(argc, argv, "fc_identification");
	ros::NodeHandle nh("~");
	//publish topic "/joint_states" for simulation
	ros::Publisher jntcmd_pub = nh.advertise<sensor_msgs::JointState>("/joint_states",10);
	// ros::Subscriber ftsensor_sub = nh.subscribe("/ftsensor_measure",1, ftSensorCB);	//need ftsensor package and topic
	// ros::Subscriber init_pos_sub = nh.subscribe("/joint_pos", 1, jntPosCB);	//need robot hardware interface to read and publish joint_pos topic
	sensor_msgs::JointState jntcmd_msg;
	jntcmd_msg.name = {"shoulder_pan_joint","shoulder_lift_joint", "elbow_joint",
						"wrist_1_joint","wrist_2_joint","wrist_3_joint"};
	jntcmd_msg.position.resize(6);

	double rate;
	nh.getParam("rate", rate);
	double dt = 1/rate;
	ros::Rate loop_rate(rate);

	// create object of fc_sensorloadid
	unsigned int jnt_num = 6;
	SensorConfig ftsensor_config;
	Wrench bias_data;
	std::vector<double> param_list;
	nh.getParam("sensor_config/bias/force", param_list);
	for(int i=0; i < 3; i++) {
		ftsensor_config.bias.force[i] = param_list[i];
	}
	nh.getParam("sensor_config/bias/torque", param_list);
	for(int i=0; i < 3; i++) {
		ftsensor_config.bias.torque[i] = param_list[i];
	}
	nh.getParam("sensor_config/pos", param_list);
	for(int i=0; i < 3; i++) {
		ftsensor_config.pos_flan2sensor[i] = param_list[i];
	}
	nh.getParam("sensor_config/rpy", param_list);
	for(int i=0; i < 3; i++) {
		ftsensor_config.rpy_flan2sensor[i] = param_list[i];
	}
	// nh.getParam("/sensor_config/bias/force", ftsensor_config.bias.force);
	// nh.getParam("/sensor_config/bias/touque", ftsensor_config.bias.touque);
	// nh.getParam("/sensor_config/pos", ftsensor_config.pos_flan2sensor);
	// nh.getParam("/sensor_config/rpy", ftsensor_config.rpy_flan2sensor);
	
	Rotation rot_flan2sensor = Rotation::RPY(ftsensor_config.rpy_flan2sensor(0),ftsensor_config.rpy_flan2sensor(1),ftsensor_config.rpy_flan2sensor(2));
	ftsensor_config.flan2sensor = Frame(rot_flan2sensor,ftsensor_config.pos_flan2sensor);
	FTSensor* ft_sensor = new FTSensor(ftsensor_config);

	vector<double> vmax, amax;
	// vector<double> amax(6, PI*5);
	nh.getParam("jntmove_limits/vmax", vmax);
	nh.getParam("jntmove_limits/amax", amax);

	FC_SensorLoadId fc_biasloadid(jnt_num, dt, ft_sensor, vmax, amax);
	
	FCLoadData load_data;
	nh.getParam("load_data/mass", load_data.mass);
	nh.getParam("load_data/cog_inflan", param_list);
	for(int i=0; i<3; i++) {
		load_data.cog_inflan[i] = param_list[i];
	}

	vector<double> init_pos(6, 0.0);
	double speed_overide = 0.1;
	fc_biasloadid.initIdentification(init_pos, speed_overide);

	Wrench ft_raw;
	// populate values for simulation test. For hardware experiment, real data should be passed in through topic.
	// ft_raw.force[0] = 1.0;
	// ft_raw.force[1] = 2.0;
	// ft_raw.force[2] = 3.0;
	// ft_raw.torque[0] = 1.1;
	// ft_raw.torque[1] = 2.2;
	// ft_raw.torque[2] = 3.3;
	vector<double> jnt_pos_cmd(6);
	while(ros::ok() and !fc_biasloadid.is_id_over) {

		fc_biasloadid.calibMove(ft_raw, jnt_pos_cmd);
		for (int i = 0; i < 6; ++i) {
			jntcmd_msg.position[i] = jnt_pos_cmd[i];
		}
		jntcmd_msg.header.stamp = ros::Time::now();
		jntcmd_pub.publish(jntcmd_msg);

		if(fc_biasloadid.is_id_success) {
			// ROS_INFO("Identification succeeded!");
			fc_biasloadid.getLoadData(load_data);
			fc_biasloadid.getSensorBias(bias_data);

			// nh.setParam("/load_data/mass", load_data.mass);
			// nh.setParam("/load_data/cog_inflan", load_data.cog_inflan);
			// nh.setParam("/sensor_config/bias/force", bias_data.force);
			// nh.setParam("/sensor_config/bias/touque", bias_data.torque);
			// ROS_INFO("Please type in following command to save identified result...");
			// ROS_INFO("'rosparam dump config/identified_result.yaml'");
			// std::cout<<"sensor bias is: x- %f, y- %f, z- %f" << bias_data << std::endl;
			// std::cout<<"load mass is: "<< load_data.mass << "kg" << std::endl;
			// std::cout<<"load mass center in flan-frame is: "<< load_data.cog_inflan<< "m" <<std::endl;
			ROS_INFO("\n sensor bias is: \n fx-%f, fy-%f, fz-%f \n tx-%f, ty-%f, tz-%f", 
				bias_data.force[0], bias_data.force[1], bias_data.force[2], bias_data.torque[0], bias_data.torque[1], bias_data.torque[2]);
			ROS_INFO("\n load mass is: %f kg", load_data.mass);
			ROS_INFO("\n load mass center in flan-frame is: \n x- %f, y- %f, z- %f", 
				load_data.cog_inflan[0], load_data.cog_inflan[1], load_data.cog_inflan[2]);
			
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	if(!fc_biasloadid.is_id_success) {
		ROS_INFO("Identification failed!");

	} else {
		string file_path = ros::package::getPath("fc_calibration") + "/config/calib_result.yaml";
		ofstream resfile;
		resfile.open(file_path, ios::trunc);
		ROS_INFO("saving result...");

		resfile << "# Force Control Calibration Results"<<"\n"
				<< "sensor_config:" << "\n"	<<"	bias:\n"
				<<"		force: ["<< bias_data.force[0]<<", "<<bias_data.force[1]<<", "<<bias_data.force[2]<<"]"<<"\n" 
				<<"		torque: ["<< bias_data.torque[0]<<", "<<bias_data.torque[1]<<", "<<bias_data.torque[2]<<"]"<<"\n"<<"\n"
				<<"load_data:"<<"\n"<<"	mass: "<<load_data.mass<<"\n"
				<<"	cog_inflan: ["<<load_data.cog_inflan[0]<<", "<<load_data.cog_inflan[1]<<", "<<load_data.cog_inflan[2]<<"]";
		resfile.close();
	}

	return 0;
}