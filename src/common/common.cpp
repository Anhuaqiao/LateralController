#include <iostream>
#include <iomanip>
#include <math.h>
#include <limits.h>
#include <ctime>
#include <fstream>
#include<eigen3/Eigen/Dense>
#include "Common.h"

using namespace std;

bool IsEqual(double a,double b)
{
    if(fabs(a-b)<=EP)
        return true;
    else
        return false;
}

double calCurvature(WayPoint pt_prime, WayPoint pt, WayPoint pt_later)
{
    double curvature = 0.0;
    double a = sqrt(pow(pt.x - pt_prime.x, 2) + pow(pt.y - pt_prime.y, 2));
    double b = sqrt(pow(pt_later.x - pt.x, 2) + pow(pt_later.y - pt.y, 2));
    double c = sqrt(pow(pt_later.x - pt_prime.x, 2) + pow(pt_later.y - pt_prime.y, 2));
    double s = (a + b + c) / 2;
    double area = sqrt(s * (s - a) * (s - b) * (s - c));
    double radius = (a * b * c) / (4 * area);
    return 1 / radius;
}

/**
 * 角度归一化到【-PI,PI】
 * @param angle
 * @return
 */

double normalizeAngle(double angle) {
    angle = fmod(angle + PI, 2.0 * PI); // 将角度调整到 [0, 2π] 范围内
    if (angle < 0) {
        angle += 2.0 * PI; // 将负角度转换为正角度
    }
    return angle - PI; // 将角度值调整到 [-π, π] 范围内
}


/**
 * 搜索目标邻近路点
 * @param robot_state 当前机器人位置
 * @param refer_path 参考轨迹（数组）
 * @return
 */
int calTargetIndex(State robot_state, vector<State>& refer_path) {
    vector<double>dists;
    for (State xy:refer_path) {
        double dist = sqrt(pow(xy.x-robot_state.x,2)+pow(xy.y-robot_state.y,2));
        dists.push_back(dist);
    }
    return min_element(dists.begin(),dists.end())-dists.begin(); //返回vector最小元素的下标
}

std::vector<State> SingleP2State(std::vector<aiforce::decision::SinglePoint> & singlep){
    State tmp;
    std::vector<State> stateOut;
    for(const auto & i:singlep){
        tmp.x = i.x;
        tmp.y = i.y;
        stateOut.emplace_back(tmp);
    }
    return stateOut;
}

Filter::Filter(){
}

void Filter::setting_num(double CurrentSpeed)
{   
    if(CurrentSpeed<1)
    {
        ave_num=5;
    }
    else if(CurrentSpeed>=1&&CurrentSpeed<=2)
    {
        ave_num=10;
    }
    else if(CurrentSpeed>=2&&CurrentSpeed<=3)
    {
        ave_num=15;
    }
    else
    {
        ave_num=20;
    }
}
Filter::~Filter()
{
}

double Filter::average_filter(double calculate_angle){
 /**************** 均值滤波 *************/
    float sum_angle=0;
    if (ave_output_list_.size() >= ave_num) {
        while(ave_output_list_.size() >= ave_num)
            ave_output_list_.pop();
        ave_output_list_.push(calculate_angle);
    } else {
        ave_output_list_.push(calculate_angle);
    }
    if(ave_output_list_.size()>0)
    {
        std::queue<float> temp_list(ave_output_list_);
        while (!temp_list.empty()) {
            sum_angle += temp_list.front();
            temp_list.pop();
        }
        calculate_angle=sum_angle/ave_output_list_.size();
    }
    return calculate_angle;
}

std::string doubleToStringWithPrecisionLimit(double value, int precision) {
    std::ostringstream streamObj;
    streamObj << std::fixed << std::setprecision(precision) << value;
    return streamObj.str();
}