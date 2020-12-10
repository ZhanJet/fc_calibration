/*****************************************************************************
 *    file:	velocityprofile_trap.hpp
 *	  date:	Feb 10, 2020
*****************************************************************************/
#ifndef VELOCITYPROFILE_TRAP_HPP
#define VELOCITYPROFILE_TRAP_HPP

inline double sign(double arg) {
    return (arg<0)?(-1):(1);
}

/**
* Implementation of trapezoidal velocityprofile
* which can be used in generating a smooth trajectory.
* @note: refering to orocos_kdl, just implement for fun
*/
class VelocityProfile_Trap
{
public:
	VelocityProfile_Trap(double max_vel=0, double max_acc=0);
	virtual ~VelocityProfile_Trap(){}

	/*
	*@brief init function
	*@param overid_fator: 0 < overide_factor <=1.0, overiding <max_vel> and <max_acc> for safety reason.
	*/
	void setProfile(double start, double end, double overide_factor=1.0);

	void setLimits(double max_vel_, double max_acc_);

	/*
	* @brief get position at time
	*/
	double getPos(double time) const;

	double getVel(double time) const;

	double getAcc(double time) const;

	/*
	* @brief get duration of profile from start to end
	*/
	double getDuration() const;

private:
	
	double _max_vel;
	double _max_acc;
	double _start;
	double _end;

	// parameters for specificating a trapezoidal velocity profile
	double _a1, _a2, _a3;
	double _b1, _b2, _b3;
	double _c1, _c2, _c3;
	double _duration;
	double _t1, _t2;

};

#endif /* VELOCITYPROFILE_TRAP_HPP */