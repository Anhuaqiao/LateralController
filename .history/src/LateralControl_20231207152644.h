/******************************************************************************************************************
* @file       : LateralControl.h
* @brief      : 农机的横向控制
* @autor      : Liukai
* @emil:      : liukai@aiforcetech.com
* @version    : 1.0.0.0
* @date       : 2023/6/27
******************************************************************************************************************/

#ifndef LATERALCONTROL_H
#define LATERALCONTROL_H
#include <vector>
#include <eigen3/Eigen/Eigen>
#include "info_transform_types.h"
#include "Common.h"
#include "Controller/stanley.h"
#include "Controller/pure_pursuit.h"
#include "Controller/RearWheelFeedback.h"

using namespace std;
namespace aiforce {
namespace control {


/**
 * @brief 类的简单概述 \n(换行)
 * 类的详细概述
 */
class Lateral_Control
{
public:
    Lateral_Control();

    ~Lateral_Control() = default;

    void Initialize();

    float GetTargetSteer(vector<decision::SinglePoint> msg,
                         int Pts_Num,
                         float currentspeed,
                         float yaw_rate,
                         float WheelBase,
                         int driving_mode_, //0：手动驾驶， 1:自动驾驶，2:导航模式
                         int rnd_gear_,
                         int wk_type,
                         int machine_type);

    void CalNearFarPoint(vector<decision::SinglePoint> msg, int point_num,  float current_speed);

public:

    Stanley stanley;
    bool fst_in_autodriving=true;
    Filter avgfilt;

private:
    bool UpdatePath(vector<decision::SinglePoint> msg);
    bool fst_in_lateral_control_=true;
    
private:
    double factor_str_kp,factor_str_ki,factor_str_kd,factor_ridge_kp,factor_ridge_ki,factor_ridge_kd;
    vector<State> RefState_;

    bool PreTurn_=true; //出入垄转弯不停车
    int refpath_limitnum=30; //参考路径的最小点数限制
    float Target_angle_=0;
    float Last_TargetAngle_=0;
    double max_angle_=30;
    double min_angle_=-30;
    double wheel_base_dist_=3; //前后轴距
    float gap_=0.2;
    int RND_gear_;
    int Driving_mode_;


};
}
}
#endif // LATERALCONTROL_H
