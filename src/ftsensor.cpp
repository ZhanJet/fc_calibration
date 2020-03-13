/*****************************************************************************
 *    file:	ftsensor.cpp
 *	author: Tu Zhangjie
 *	  date:	Feb 12, 2020
*****************************************************************************/

#include "ftsensor.hpp"

FTSensor::FTSensor(const SensorConfig& sensor_config):_sensor_config(sensor_config)
{
	_has_sensorid = false;
	_load_data.mass = 0.0;
	_load_data.cog_inflan = Vector::Zero();
}

void FTSensor::getFTInSensor(const Wrench& ft_raw, Wrench& ft_sensor) {
	ft_sensor = ft_raw - _sensor_config.bias;
}

void FTSensor::getFTInFlan(const Wrench& ft_raw, Wrench& ft_flan) {
	Wrench ft_sensor = ft_raw - _sensor_config.bias;
	ft_flan = _sensor_config.flan2sensor * ft_sensor;
}

// void FTSensor::getFT(const Wrench& ft_raw, Wrench& ft_exact) {
// 	if(_has_sensorid)
// 		getFTInFlan(ft_raw, ft_exact);
// 	else
// 		getFTInSensor(ft_raw, ft_exact);
// }

void FTSensor::getFTForId(const Wrench& ft_raw, Wrench& ft_flan_withbias){
	ft_flan_withbias = _sensor_config.flan2sensor * ft_raw;
}