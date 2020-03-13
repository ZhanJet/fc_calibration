/*****************************************************************************
 *    file:	joint_move.cpp
 *	author: tu zhangjie
 *	  date:	Feb 10, 2020
*****************************************************************************/

// #include <iostream>

#include "joint_move.hpp"

JointMove::JointMove(unsigned int jnt_num):
		_jnt_num(jnt_num), _duration(0),
		_vel_max(jnt_num,0.0),_acc_max(jnt_num,0.0),
		_start(jnt_num,0.0),_end(jnt_num,0.0),
		_scale(jnt_num,0.0)
{ }

int JointMove::setLimits(const std::vector<double>& vmax, const std::vector<double>& amax) {
	if(vmax.size()==_jnt_num and amax.size()==_jnt_num) {
		_vel_max = vmax; _acc_max = amax;
		return 0;
	} else {
		return -1;
	}
}

bool JointMove::setStartEnd(const std::vector<double>& start, const std::vector<double>& end, double overide_factor) {
	// check joint_num
	if(start.size() != _jnt_num or end.size() != _jnt_num) {
		return false;
	}
	//check if start pos equals end pos
	bool equal = true;
	for(unsigned int i=0; i < _jnt_num; i++) {
		if(fabs(end[i]-start[i]) > 1e-6) equal = false;
	}
	if(equal) return false;

	if(overide_factor > 1.0) overide_factor = 1.0;
	if(overide_factor < 1e-6) return false;

	_start = start;
	_end = end;
	//find the joint with the longest duration
	std::vector<double> jnt_duration(_jnt_num);
	unsigned int index;
	double max_duration=0.0;
	for(unsigned int i=0; i < _jnt_num; i++) {
		// double jnt_duration = fabs(end[i]-start[i])/_vel_max[i];
		_vel_profile.setLimits(_vel_max[i], _acc_max[i]);
		_vel_profile.setProfile(0.0, fabs(end[i]-start[i]), overide_factor);
		jnt_duration[i] = _vel_profile.getDuration();
		if(jnt_duration[i] > max_duration) {
			index = i;
			max_duration = jnt_duration[i];
		}
	}
	for(unsigned int i=0; i < _jnt_num; i++) {
		// _scale[i] = (end[i]-start[i])/_vel_max[i] / max_duration;
		_scale[i] = sign(end[i]-start[i]) * jnt_duration[i] / max_duration;
	}

	// initialize motion profile for all joints by the longest one
	_vel_profile.setLimits(_vel_max[index], _acc_max[index]);
	_vel_profile.setProfile(0.0, fabs(end[index]-start[index]), overide_factor);
	_duration = _vel_profile.getDuration();

	return true;
}

int JointMove::getPos(double time, std::vector<double>& pos) {
	if(pos.size() != _jnt_num) {
		return -1;
	}

	if(time < 0.0) time = 0.0;
	if(time > _duration) time = _duration;

	for(unsigned int i=0; i < _jnt_num; i++) {
		pos[i] = _start[i] + _scale[i] * _vel_profile.getPos(time);
	}
	return 0;
}

int JointMove::getVel(double time, std::vector<double>& vel) {
	if(vel.size() != _jnt_num) {
		return -1;
	}

	if(time < 0.0) time = 0.0;
	if(time > _duration) time = _duration;

	for(unsigned int i=0; i < _jnt_num; i++) {
		vel[i] = _scale[i] * _vel_profile.getVel(time);
	}
	return 0;
}

int JointMove::getAcc(double time, std::vector<double>& acc) {
	if(acc.size() != _jnt_num) {
		return -1;
	}

	if(time < 0.0) time = 0.0;
	if(time > _duration) time = _duration;

	for(unsigned int i=0; i < _jnt_num; i++) {
		acc[i] = _scale[i] * _vel_profile.getAcc(time);
	}
	return 0;
}