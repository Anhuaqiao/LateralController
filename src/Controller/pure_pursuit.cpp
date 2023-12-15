// Created by LiuKai on 2023/06/25.

#include "pure_pursuit.h"
/**
 * 计算邻近路点
 * @param robot_state 当前机器人位置
 * @param refer_path 参考轨迹（数组）
 * @param l_d 前向距离
 * @return
 */
PurePursuit::PurePursuit(){
    this->radius_min = 10;
    this->alpha_max = 5;
    this->beta_max = 0.2;
}

void PurePursuit::calcLookAHeadPoint(State cur_state, vector<State>& refer_path){
    this->i_pw = calTargetIndex(cur_state, refer_path);
    this->i_pd = refer_path.size()-1;
    double shortest_dubins_path_length = 200, current_dubins_length;
    auto start = refer_path.begin() + i_pw; // 第 5 个元素的迭代器
    for (auto refer_path_point = start; refer_path_point != refer_path.end(); ++refer_path_point){
        if(radius_min<Pt_dist(refer_path[i_pw],(*refer_path_point)) && Pt_dist(refer_path[i_pw],(*refer_path_point))<12){
            double q0[3];
            q0[0] = cur_state.x;
            q0[1] = cur_state.y;
            q0[2] = cur_state.psi + steering_angle;
            double q1[3];
            q1[0] = (*refer_path_point).x;
            q1[1] = (*refer_path_point).y;
            q1[2] = (*refer_path_point).psi;
            call_dubins_shortest_path( &path, q0, q1, radius_min);
            current_dubins_length = call_dubins_path_length( &path );
            if(current_dubins_length<shortest_dubins_path_length){
                shortest_dubins_path_length = current_dubins_length;
                i_pd = std::distance(refer_path.begin(), refer_path_point);
            } 
        }
    }
}

void PurePursuit::OutOfPath(State cur_state, vector<State>& refer_path){
    double pd_angle = atan2(refer_path[i_pd].y, refer_path[i_pd].x);
    double pdl_angle = pd_angle - sign(pd_angle)*PI/2;
    double alpha = Pt_dist(cur_state, refer_path[this->i_pw]) < alpha_max? Pt_dist(cur_state, refer_path[this->i_pw])/alpha_max : 1;
    double beta = abs(refer_path[i_pw].K - refer_path[i_pd].K)>beta_max? 1 : (refer_path[i_pd].K - refer_path[i_pw].K)/beta_max;
    double tau = (1 - alpha)*beta;
    double p_wd_dist = Pt_dist(refer_path[i_pw], refer_path[i_pd]);
    double p_dl_dist = p_wd_dist*tan(pd_angle);
    double pl_x = refer_path[i_pd].x + tau*p_dl_dist*cos(pdl_angle);
    double pl_y = refer_path[i_pd].y + tau*p_dl_dist*sin(pdl_angle);
    State pl_tmp(pl_x, pl_y, 0,0);
    pl.emplace_back(pl_tmp);
}

/**
 * purePursuitControl
 * @param robot_state 当前机器人位置
 * @param current_ref_point 参考轨迹点
 * @param l_d 前向距离
 * @param psi 机器人航向角
 * @param L 轴距
 * @return 转角控制量
 */
double PurePursuit::steer(State cur_state,double cur_speed,double wheel_base, vector<State> refer_path)  {
    calcLookAHeadPoint(cur_state, refer_path);
    OutOfPath(cur_state, refer_path);
    double theta_ = atan2(pl.back().y-cur_state.y, pl.back().x-cur_state.x);
    State pl_tmp = pl.back();
    double l_d = Pt_dist(pl_tmp, cur_state);
    double delta = atan2(2*wheel_base*sin(theta_),l_d); //

    double cross_track_error = Pt_dist(refer_path[i_pw],cur_state);
    crs_track_err.emplace_back(cross_track_error);

    return  steer_limit(delta);

}

