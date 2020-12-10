/*****************************************************************************
 *    file:	fc_sensorloadid_ur.cpp
 *	  date:	Feb 12, 2020
*****************************************************************************/

#include <iostream>
#include "fc_sensorloadid_ur.hpp"

FC_SensorLoadId::FC_SensorLoadId(unsigned int jnt_num, double dt, FTSensor* ft_sensor,
	const vector<double>& vmax, const vector<double>& amax):
	_jnt_num(jnt_num), _dt(dt), _ft_sensor(ft_sensor), _vmax(vmax), _amax(amax)
{
	_bias = Wrench::Zero();
	_load.mass = 0.0;
	_load.cog_inflan = Vector::Zero();

	_overide_factor = 0.01;
	_calib_time = 0;
	_duration1 = 0;
	_duration2 = 0;
	_duration3 = 0;
	_duration_collect = 3.0;
	_phase_index = 0;

	_collect_counts.resize(3);
	_ft_accumulate.resize(3);

	is_id_success = false;
	is_id_over = false;

	for (int i = 0; i < 3; ++i)	{
		_jnt_move[i] = new JointMove(_jnt_num);
	}
}

bool FC_SensorLoadId::initIdentification(const vector<double> pos_init, const double overide_factor){
	
	if(pos_init.size() != _jnt_num) return false;

	// _pos_init = pos_init;
	_overide_factor = overide_factor;

	for(int i=0; i< 3; i++){
		_collect_counts[i] = 0;
		for(int j = 0; j < 3; j++){
			_ft_accumulate[i].force[j] = 0;
			_ft_accumulate[i].torque[j] = 0;
		}
	}
	_calib_time = 0.0;
	_phase_index = 0;
	is_id_success = false;
	is_id_over = false;

	//set identification process
	vector<double> calib_point1{0.0,-PI/2,0.0,0.0,0.0,0.0};
	vector<double> calib_point2{0.0,-PI/2,0.0,-PI/2,0.0,0.0};
	vector<double> calib_point3{0.0,-PI/2,0.0,0.0,-PI/2,0.0};

	for(unsigned int i = 0; i < 3; i++) {
		_jnt_move[i]->setLimits(_vmax, _amax);
	}

	_jnt_move[0]->setStartEnd(pos_init, calib_point1, _overide_factor);
	_jnt_move[1]->setStartEnd(calib_point1, calib_point2, _overide_factor);
	_jnt_move[2]->setStartEnd(calib_point2, calib_point3, _overide_factor);

	_duration1 = _jnt_move[0]->getDuration();
	_duration2 = _jnt_move[1]->getDuration();
	_duration3 = _jnt_move[2]->getDuration();

}

void FC_SensorLoadId::collectData(Wrench& ft_flan_withbias, int calib_point_index){
	_collect_counts[calib_point_index] ++;
	_ft_accumulate[calib_point_index] += ft_flan_withbias;
}

bool FC_SensorLoadId::calibMove(const Wrench& ft_raw, vector<double>& jnt_pos_cmd){
	//Identification Process:
	// initial position->calibration position1->steady data collection
	// ->calibration position2->steady data collection
	// ->calibration position3->steady data collection->calculation
	double phase_time = 0.0;
	_calib_time += _dt;
	if(_calib_time <= _duration1){
		///1.initial position->calibration position1
		_phase_index = 0;
		phase_time = _calib_time;
	} else if(_calib_time <= _duration1+_duration_collect){
		///2.steady data collection
		_phase_index = 1;
	} else if(_calib_time <= _duration1+_duration_collect+_duration2){
		///3.steady data collection->calibration position2
		_phase_index = 2;
		phase_time = _calib_time - (_duration1+_duration_collect);
	} else if(_calib_time <= _duration1+_duration2+_duration_collect*2){
		///4.steady data collection
		_phase_index = 3;
	} else if(_calib_time <= _duration1+_duration2+_duration3+_duration_collect*2){
		///5.steady data collection->calibration position3
		_phase_index = 4;
		phase_time = _calib_time - (_duration1+_duration2+_duration_collect*2);
	} else if(_calib_time <= _duration1+_duration2+_duration3+_duration_collect*3){
		///6.steady data collection
		_phase_index = 5;
	} else {
		///7.calculation
		_phase_index = 6;
	}

	Wrench ft_flan_withbias;
	switch(_phase_index)
	{
		case 0:
			_jnt_move[0]->getPos(phase_time, jnt_pos_cmd);
			return false;
		case 1:
			_ft_sensor->getFTForId(ft_raw, ft_flan_withbias);
			collectData(ft_flan_withbias, 0);
		case 2:
			_jnt_move[1]->getPos(phase_time, jnt_pos_cmd);
			return false;
		case 3:
			_ft_sensor->getFTForId(ft_raw, ft_flan_withbias);
			collectData(ft_flan_withbias, 1);
		case 4:
			_jnt_move[2]->getPos(phase_time, jnt_pos_cmd);
			return false;
		case 5:
			_ft_sensor->getFTForId(ft_raw, ft_flan_withbias);
			collectData(ft_flan_withbias, 2);
			return false;
		case 6:
			std::cout<<"Identification calculating..." << std::endl;
			if(computeParams()){
				is_id_success = true;
				std::cout<<"Identification succeed!" << std::endl;
			} else{
				is_id_success = false;
				std::cout<<"Identification failed!" << std::endl;
			}
			is_id_over = true;
			return is_id_over;
	}
	return true;
}

bool FC_SensorLoadId::computeParams(){
	vector<Wrench> ft_average(3);
	for(int i=0; i<3; i++){
		ft_average[i] = _ft_accumulate[i] / _collect_counts[i];
	}

	_bias.force[0] = (ft_average[1].force[0] + ft_average[2].force[0]) / 2.0;
	_bias.force[1] = (ft_average[0].force[1] + ft_average[2].force[1]) / 2.0;
	_bias.force[2] = (ft_average[0].force[2] + ft_average[1].force[2]) / 2.0;

	_bias.torque[0] = ft_average[0].torque[0];
	_bias.torque[1] = ft_average[1].torque[1];
	_bias.torque[2] = ft_average[2].torque[2];

	_ft_sensor->setFTSensorBias(_bias);
	vector<Wrench> ft_average_load(3);
	for(int i=0; i<3; i++){
		_ft_sensor->getFTInFlan(ft_average[i], ft_average_load[i]);
	}
	
	// _load.mass = (ft_average_load[0].force[0]+ft_average_load[1].force[1]+ft_average_load[2].force[2]) / 3.0 / 9.81;
	// _load.cog_inflan(0) = (ft_average_load[1].torque[2] - ft_average_load[2].torque[1]) / 2.0 / (_load.mass*9.81);
	// _load.cog_inflan(1) = (-ft_average_load[0].torque[2] + ft_average_load[2].torque[0]) / 2.0 / (_load.mass*9.81);
	// _load.cog_inflan(2) = (ft_average_load[0].torque[1] - ft_average_load[1].torque[0]) / 2.0 / (_load.mass*9.81);
	
	//check the identification result
	double mass1 = ft_average_load[0].force[0] / 9.81;
	double mass2 = ft_average_load[1].force[1] / 9.81;
	double mass3 = ft_average_load[2].force[2] / 9.81;
	double mass_error = maxError(mass1, mass2, mass3);
	double cogz_error = fabs(ft_average_load[0].torque[1] + ft_average_load[1].torque[0]) / 2.0 / (_load.mass*9.81);
	if(mass_error >= 0.1 or cogz_error >= 0.02) {
		std::cout<<"Identification Error is too large!!!"<<std::endl;
		return false;
	} else {
		_load.mass = max(0.000001, (mass1 + mass2 + mass3) / 3.0);
		_load.cog_inflan(0) = (ft_average_load[1].torque[2] - ft_average_load[2].torque[1]) / 2.0 / (_load.mass*9.81);
		_load.cog_inflan(1) = (-ft_average_load[0].torque[2] + ft_average_load[2].torque[0]) / 2.0 / (_load.mass*9.81);
		_load.cog_inflan(2) = (ft_average_load[0].torque[1] - ft_average_load[1].torque[0]) / 2.0 / (_load.mass*9.81);
		
		return true;
	}
}

double FC_SensorLoadId::maxError(double a, double b, double c){
	double tmp;
	if(a>b) {tmp=a; a=b; b=tmp;}
	if(b>c) {tmp=b; b=c; c=tmp;}
	return c-a;
}