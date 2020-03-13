/*****************************************************************************
 *    file:	fc_sensorloadid_ur.hpp
 *	author: Tu Zhangjie
 *	  date:	Feb 12, 2020
*****************************************************************************/
#ifndef FC_SENSORLOADID_UR_HPP_
#define FC_SENSORLOADID_UR_HPP_

#include <vector>

#include "joint_move.hpp"
#include "ftsensor.hpp"

using namespace std;
// using KDL::Wrench;
using namespace KDL;

#define PI 3.14159265359

class FC_SensorLoadId
{
public:
	FC_SensorLoadId(unsigned int jnt_num, double dt, FTSensor* ft_sensor,
		const vector<double>& vmax, const vector<double>& amax);
	~FC_SensorLoadId(){}

	bool initIdentification(const vector<double> pos_init, const double overide_factor=0.01);

	//return true when identification is over, return false or else;
	bool calibMove(const Wrench& ft_raw, vector<double>& jnt_pos_cmd);

	void getLoadData(FCLoadData& load_data){load_data = _load;}

	void getSensorBias(Wrench& bias){bias = _bias;}

	bool is_id_success;
	bool is_id_over;
private:
	// compute load data and sensor bias
	bool computeParams();

	void collectData(Wrench& ft_flan, int calib_point_index);

	double maxError(double a, double b, double c);
private:
	double _dt;
	FTSensor* _ft_sensor;
	JointMove* _jnt_move[3];
	unsigned int _jnt_num;
	vector<double> _vmax;
	vector<double> _amax;

	FCLoadData _load;
	Wrench _bias;

	//for calibration process
	double _overide_factor;
	double _calib_time;
	double _duration1;	//pos_init->point_1
	double _duration2;	//point1->point2
	double _duration3;	//point2->point3
	double _duration_collect;
	int _phase_index;

	//for data process
	vector<int> _collect_counts;
	vector<Wrench> _ft_accumulate;

	//for identification calculation
	
};

#endif /* FC_SENSORLOADID_UR_HPP_ */
