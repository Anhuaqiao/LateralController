#ifndef STANLEY_H
#define STANLEY_H
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <vector>
#include<cmath>
#include <algorithm>
#include<eigen3/Eigen/Dense>
#include "../Common.h"

using namespace std;



class Stanley {
private:
    double k_psi=1;  //横向偏差的系数
    double k_lateral=1;  //航向偏差系数
    double k_soft=0.9; // 减少在低速时位置偏差的小幅度改变对车辆的影响
    double k_yaw_rate=0.5; //航向变化阻尼
    double kd=0;

    double last_yaw_=0;
    double last_deta=0;
    double last_angle=0;

public:
    void set_parameter(double k_psi,double k_lateral,double k_soft,double k_yaw_rate,double kd);
public:
    double sum_ey=0;
    double yaw_rate=0;
    double cur_speed=0;
    Stanley();
    ~Stanley() = default;
    int calTargetIndex(State robot_state, vector<State> refer_path);
    double stanleyControl(State cur_state,double cur_speed,double wheel_base, vector<State> refer_path);
    };


#endif // STANLEY_H
