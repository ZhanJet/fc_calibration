/*****************************************************************************
 *    file:	ftsensor.hpp
 *	  date:	Feb 11, 2020
*****************************************************************************/
#ifndef FTSENSOR_HPP_
#define FTSENSOR_HPP_

#include <vector>
// #include <Eigen/Geometry>
#include <kdl/frames.hpp>

using namespace KDL;

struct SensorConfig {
	Wrench bias;
	Vector pos_flan2sensor;
	Vector rpy_flan2sensor; //orientation of sensor frame refering to flange frame, in RPY angles.
	Frame  flan2sensor;
};

struct FCLoadData {
	double mass;
	Vector cog_inflan;	// center of load represented in flan frame
};

// Receive initial measured data from ftsensor hardware through Pub-Sub interface
// and convert it to real Force/Torque expressed in corresponding Force-Control frame(eg.Flange)
// by substrating sensor bias and transforming from sensor to fc-frame.
class FTSensor
{
private:
	SensorConfig  	_sensor_config;
	FCLoadData 		_load_data;

	bool 	_has_sensorid;	//has bias of ftsensor been identified, false by default

public:
	FTSensor(const SensorConfig& sensor_config);

	~FTSensor(){}
	
	void setFCLoadData(const FCLoadData& fc_load_data) {_load_data = fc_load_data;}

	// used for resetting sensor config after bias-loaddata identification
	void setFTSensorConfig(const SensorConfig& sensor_config) {_sensor_config = sensor_config;}

	void setFTSensorBias(const Wrench& sensor_bias) {_sensor_config.bias = sensor_bias;}

	void setHasSensorID(bool has_id){_has_sensorid = has_id;}

	void getFTInSensor(const Wrench& ft_raw, Wrench& ft_sensor);

	void getFTInFlan(const Wrench& ft_raw, Wrench& ft_flan);

	void getFTForId(const Wrench& ft_raw, Wrench& ft_flan_withbias);

	// void getFT(const Wrench& ft_raw, Wrench& ft_exact);

};

#endif /* FTSENSOR_HPP_ */