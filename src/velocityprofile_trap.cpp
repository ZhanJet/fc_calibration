/*****************************************************************************
 *    file:	velocityprofile_trap.cpp
 *	author: tzhj
 *	  date:	Feb 10, 2020
*****************************************************************************/

#include <cmath>
#include "velocityprofile_trap.hpp"

VelocityProfile_Trap::VelocityProfile_Trap(double max_vel, double max_acc):
						_max_vel(max_vel), _max_acc(max_acc),
						_start(0), _end(0),
						_a1(0), _a2(0), _a3(0),
						_b1(0), _b2(0), _b3(0),
						_c1(0), _c2(0), _c3(0),
						_duration(0), _t1(0), _t2(0)

{}


void VelocityProfile_Trap::setProfile(double start, double end, double overide_factor) {
	_start = start;
	_end = end;
	_t1 = _max_vel / _max_acc;

	// compute duration of constant velocity stage
	double direction = sign(_end - _start);
	double x1 = direction*_max_acc*_t1*_t1 / 2.0;
	double t_constvel = (_end - _start - 2.0*x1) / direction / _max_vel;
	if (t_constvel > 0) {
		_t2 = _t1 + t_constvel;
		_duration = _t1 + _t2;
	} else {
		_t1 = sqrt(direction*(_end - _start) / _max_acc);
		_t2 = _t1;
		_duration = _t1 + _t2;
	}

	// check overide_factor
	double overide = (overide_factor > 1.0)? 1.0:(overide_factor > 0.0 ? overide_factor:0.0);
	_a3 = direction * _max_acc / 2.0;
	_a2 = 0.0;
	_a1 = _start;

	_b3 = 0.0;
	_b2 = _a2 + 2.0*_a3*_t1 - 2.0*_b3*_t1;
	_b1 = _a1 + _t1*(_a2 + _a3*_t1) - _t1*(_b2 + _b3*_t1);

	_c3 = -direction * _max_acc / 2.0;
	_c2 = _b2 + 2.0*_b3*_t2 - 2.0*_c3*_t2;
	_c1 = _b1 + _t2*(_b2 + _b3*_t2) - _t2*(_c2 + _c3*_t2);

	_duration /= overide;
	_t1 /= overide;
	_t2 /= overide;

	_a2 *= overide;
	_a3 *= overide*overide;
	_b2 *= overide;
	_b3 *= overide*overide;
	_c2 *= overide;
	_c3 *= overide*overide;
}

void VelocityProfile_Trap::setLimits(double max_vel, double max_acc) {
	_max_vel = max_vel;
	_max_acc = max_acc;
}

double VelocityProfile_Trap::getPos(double time) const {
	if (time < 0.0) {
		return _start;
	} else if (time < _t1) {
		return _a1+time*(_a2+_a3*time);
	} else if (time < _t2) {
		return _b1+time*(_b2+_b3*time);
	} else if (time < _duration) {
		return _c1+time*(_c2+_c3*time);
	} else {
		return _end;
	}
}

double VelocityProfile_Trap::getVel(double time) const {
	if(time < 0.0) {
		return 0.0;
	} else if(time < _t1) {
		return _a2 + 2.0*_a3*time;
	} else if(time < _t2) {
		return _b2 + 2.0*_b3*time;
	} else if(time < _duration) {
		return _c2 + 2.0*_c3*time;
	} else {
		return 0.0;
	}
}

double VelocityProfile_Trap::getAcc(double time) const {
	if(time < 0.0) {
		return 0.0;
	} else if(time < _t1) {
		return 2.0*_a3;
	} else if(time < _t2) {
		return 2.0*_b3;
	} else if(time < _duration) {
		return 2.0*_c3;
	} else {
		return 0.0;
	}
}

double VelocityProfile_Trap::getDuration() const {
	return _duration;
}