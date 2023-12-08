//
// Created by Xiaohan on 2023/12/08.
//
#include "stanley.h"

Stanley::Stanley():aiforce::control::Controller(){
      this->k_psi = 1.0;
      this->k_lateral = 1.0;
      this->k_soft = 0.9;
      this->k_yaw_rate = 0.5;
      this->kd = 0;
}

void Stanley::set_parameter(double k_psi,double k_lateral,double k_soft,double k_yaw_rate,double kd)
{
    this->k_psi = k_psi;
    this->k_lateral = k_lateral;
    this->k_soft = k_soft;
    this->k_yaw_rate = k_yaw_rate;
    this->kd = kd;
}

/**
 * stanley控制
 * @param robot_state 机器人位姿，包括x,y,yaw,v
 * @param refer_path 参考轨迹的位置和参考轨迹上点的切线方向的角度 x,y,theta
 * @return 控制量+目标点索引
 */
double Stanley::steer(State cur_state,double cur_speed,double wheel_base,vector<State> refer_path) {

    //State cur_front_state(wheel_base,cur_state.y,cur_state.psi); //以后轮中心为车体参考系的中心的情况下，前轮中心的坐标
    //cur_state=cur_front_state;
    double theta_deta_rate;
    double yaw_stability=0;

    if(cur_speed<1.5) //减少在低速时,位置偏差的小幅改变对车辆的影响
        k_soft=0.8*cur_speed;
    else
        k_soft=0;
    if(fabs(cur_state.y)<0.15)
        k_yaw_rate=0;

    int current_target_index = calTargetIndex(cur_state,refer_path);
    State  current_ref_state;  //参考路径上 离车实际位置最近的参考点

    // 当计算出来的目标临近点索引大于等于参考轨迹上的最后一个点索引时
    if(current_target_index>=refer_path.size()){
        current_target_index = refer_path.size()-1;
        current_ref_state = refer_path[refer_path.size()-1];
    }else{
        current_ref_state = refer_path[current_target_index];
    }



    double e_y=0; //横向偏差
    if(current_target_index>1)
    {
        e_y = getDistancePointToLine(refer_path[current_target_index],refer_path[current_target_index-1], cur_state);
    }
    else
    {
        e_y = getDistancePointToLine(refer_path[current_target_index],refer_path[current_target_index+1], cur_state);
    }


    if((cur_state.y-current_ref_state.y)*current_ref_state.psi-(cur_state.x-current_ref_state.x)>0){
    //if((robot_state[1]-current_ref_point[1])*cos(psi_t)-(robot_state[0]-current_ref_point[0])*sin(psi_t)<=0){
        e_y = e_y;
    }else{
        e_y = -e_y;
    }

    sum_ey+=fabs(e_y);
    //# 通过公式(5)计算转角,符号保持一致

    double psi_theta = current_ref_state.psi - cur_state.psi;             //航向偏差
    double dist_theta = atan2(k_lateral*e_y, k_soft+cur_speed);                    //横向偏差
    double delta = psi_theta-k_yaw_rate*yaw_rate+dist_theta;

    /****************** 输出均值滤波 ******************/

//    std::cout<<"delta "<<delta<<std::endl;
    return delta;
}
