#include <iostream>
#include <iomanip>
#include <math.h>
#include <limits.h>
#include <ctime>
#include <fstream>
#include<eigen3/Eigen/Dense>
#include "Common.h"

using namespace std;
double Pt_dist(WayPoint pt1,WayPoint pt2)
{
    return sqrt(pow((pt1.x-pt2.x),2)+pow((pt1.y-pt2.y),2));
}
double Pt_dist(State pt1,State pt2)
{
    return sqrt(pow((pt1.x-pt2.x),2)+pow((pt1.y-pt2.y),2));
}
double Pt_dist(aiforce::decision::SinglePoint pt1,aiforce::decision::SinglePoint pt2)
{
    return sqrt(pow((pt1.x-pt2.x),2)+pow((pt1.y-pt2.y),2));
}
double getDistancePointToLine(WayPoint A, WayPoint B, WayPoint P) {
    double area = abs((B.x - A.x) * (P.y - A.y) - (P.x - A.x) * (B.y - A.y));
    double base = Pt_dist(A, B);
    return area / base;
}

double getDistancePointToLine(State A, State B, State P) {
    double area = abs((B.x - A.x) * (P.y - A.y) - (P.x - A.x) * (B.y - A.y));
    double base = Pt_dist(A, B);
    return area / base;
}

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
    while(angle>PI){
        angle-=2.0*PI;
    }
    while(angle<-PI){
        angle+=2.0*PI;
    }
    return angle;
}


/**
 * 搜索目标邻近路点
 * @param robot_state 当前机器人位置
 * @param refer_path 参考轨迹（数组）
 * @return
 */
int calTargetIndex(State robot_state, vector<State> refer_path) {
    vector<double>dists;
    for (State xy:refer_path) {
        double dist = sqrt(pow(xy.x-robot_state.x,2)+pow(xy.y-robot_state.y,2));
        dists.push_back(dist);
    }
    return min_element(dists.begin(),dists.end())-dists.begin(); //返回vector最小元素的下标
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
