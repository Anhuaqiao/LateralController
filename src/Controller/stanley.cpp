//
// Created by Xiaohan on 2023/12/08.
//
#include "stanley.h"

Stanley::Stanley():aiforce::control::Controller(){
      this->k_psi = 1.0;
      this->k_lateral = 0.5;
      this->k_soft = 0.9;
      this->k_yaw_rate = 1;
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

    int current_target_index = calTargetIndex(cur_state,refer_path);
    State  current_ref_state = refer_path[current_target_index];  //参考路径上 离车实际位置最近的参考点
    current_ref_state.x = -current_ref_state.x;
    current_ref_state.y = -current_ref_state.y;
    State negative_y(0,-1,0,0);
    double cross_track_error = dotProduct(current_ref_state,negative_y);

    double ref_psi_rate = calRefPsiRate(refer_path,current_target_index,cur_speed);    

    double psi_theta = normalizeAngle(current_ref_state.psi - cur_state.psi);             //航向偏差
    double dist_theta = atan2(k_lateral*cross_track_error, cur_speed);                    //横向偏差
    double psi_rate_theta = k_yaw_rate*(cur_state.psi_rate-ref_psi_rate); 


    double delta = psi_theta+dist_theta+psi_rate_theta;

    return steer_limit(delta);
}
