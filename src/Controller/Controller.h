/******************************************************************************************************************
* @file       : Controller.h
* @brief      : Lateral controller of the the tractor
* @autor      : Xiaohan
* @emil:      : tangxiaohan@aiforcetech.com
* @version    : 1.0.0.0
* @date       : 2023/12/07
******************************************************************************************************************/

#ifndef LATERALCONTROL_H
#define LATERALCONTROL_H
#include <vector>
#include <eigen3/Eigen/Eigen>
#include "../info_transform_types.h"
#include "../Common.h"
#include "../KinematicModel.h"
#include <unordered_map>
#define PI 3.1415926

using namespace std;
namespace aiforce {
namespace control {


/**
 * @brief 类的简单概述 \n(换行)
 * 类的详细概述
 */
class Controller
{


public:
    Filter avgfilt;
    vector<State> RefPath;
    bool PreTurn_=true; 
    int refpath_limitnum; 
    double max_angle_;
    double min_angle_;
    float gap_=0.2;
    int RND_gear_;
    int Driving_mode_=1;
    std::vector<double> crs_track_err;

public:
    Controller();

    ~Controller() = default;
    virtual double steer(State cur_state,double cur_speed,double wheel_base, vector<State> refer_path)= 0;
    bool UpdatePath(vector<decision::SinglePoint> msg);
    

protected:
    void Controller_preprocessor(vector<decision::SinglePoint> msg,
                         int Pts_Num,
                         double currentspeed,
                         float yaw_rate,
                         float WheelBase,
                         int driving_mode_, //0：手动驾驶， 1:自动驾驶，2:导航模式
                         int rnd_gear_);
    double steer_limit(double steer_angle);
    double calRefPsiRate(std::vector<State>& refer_path, int& target_index, double cur_speed);


};
}
}
#endif // LATERALCONTROL_H
