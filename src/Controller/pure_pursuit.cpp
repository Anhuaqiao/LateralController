// Created by LiuKai on 2023/06/25.

#include "pure_pursuit.h"
/**
 * 计算邻近路点
 * @param robot_state 当前机器人位置
 * @param refer_path 参考轨迹（数组）
 * @param l_d 前向距离
 * @return
 */
int PurePursuit::calTargetIndex(State cur_state, vector<State> refer_path, double l_d) {
    vector<double>dists;
    for (State xy:refer_path) {
        double dist = sqrt(pow(xy.x-cur_state.x,2)+pow(xy.y-cur_state.y,2));
        dists.push_back(dist);
    }
    int min_ind = min_element(dists.begin(),dists.end())-dists.begin(); //返回vector最小元素的下标

    double delta_l = sqrt(pow(refer_path[min_ind].x-cur_state.x,2)+pow(refer_path[min_ind].y-cur_state.y,2)); //先找到目标路径上距离 当前位置最近的点

    while (l_d>delta_l && min_ind<refer_path.size()-1){ //从最近点向 index增大方向搜索,找到最接近 l_d 的 index
        delta_l = sqrt(pow(refer_path[min_ind+1].x-cur_state.x,2)+pow(refer_path[min_ind+1].y-cur_state.y,2));
        min_ind+=1;
    }


    return min_ind;
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
double PurePursuit::purePursuitControl(State cur_state,double cur_speed, vector<State> refer_path,double L) {

    double l_d=cur_speed*5;  // 参考预瞄距离

    int target_index=calTargetIndex(cur_state,refer_path,l_d);
    double alpha = atan2(refer_path[target_index].x-cur_state.x,refer_path[target_index].y-cur_state.y)-cur_state.psi;
    double delta = atan2(2*L*sin(alpha),l_d); //L 轴距
//    double k=tan(psi);
//    double b=robot_state[1]-k*robot_state[0];
//    double x2=robot_state[0]+1;
//    double y2=x2*k+b;
//    double e_y= getDistancePointToLine(WayPoint(current_ref_point[0],current_ref_point[1]), WayPoint(x2,y2), WayPoint(robot_state[0],robot_state[1]));
//    if((robot_state[0]-current_ref_point[0])*psi-(robot_state[1]-current_ref_point[1])>0){
//    //if((robot_state[1]-current_ref_point[1])*cos(psi_t)-(robot_state[0]-current_ref_point[0])*sin(psi_t)<=0){
//        e_y = e_y;
//    }else{
//        e_y = -e_y;
//    }

//    std::cout<<"psi "<<psi<<std::endl;
//    std::cout<<"e_y "<<e_y<<std::endl;

//    double delta = atan2(2*L*e_y,pow(l_d,2));



    return delta;

}

