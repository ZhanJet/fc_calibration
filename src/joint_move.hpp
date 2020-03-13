/*****************************************************************************
 *    file:	joint_move.hpp
 *	author: tu zhangjie
 *	  date:	Feb 10, 2020
*****************************************************************************/
#ifndef JOINT_MOVE_HPP_
#define JOINT_MOVE_HPP_

#include <vector>
#include <cmath>

// #include <kdl/velocityprofile_trap.hpp>
#include "velocityprofile_trap.hpp"

class JointMove
{
public:
	JointMove(unsigned int jnt_num = 6);
	~JointMove(){}

	int setLimits(const std::vector<double>& vmax, const std::vector<double>& amax);

	/*
	*@brief init function
	*@param overid_fator: 0 < overide_factor <=1.0
	*/
	bool setStartEnd(const std::vector<double>& start, const std::vector<double>& end, double overide_factor=0.1);

	int getPos(double time, std::vector<double>& pos);

	int getVel(double time, std::vector<double>& vel);

	int getAcc(double time, std::vector<double>& acc);

	double getDuration() {return _duration;}

private:
	unsigned int _jnt_num;
	VelocityProfile_Trap _vel_profile;
	std::vector<double> _vel_max;
	std::vector<double> _acc_max;
	std::vector<double> _start;
	std::vector<double> _end;
	std::vector<double> _direction;
	std::vector<double> _scale;
	double _duration;
	
};

#endif /* JOINT_MOVE_HPP_ */