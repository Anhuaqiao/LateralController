#ifndef COMMON_H
#define COMMON_H
#include <vector>
#include <queue>
#include "info_transform_types.h"
#define PI 3.1415926
#define	EP	1E-5

struct WayPoint
{
    //WayPoint();
    //路点编号
    int index;
    //垄编号
    int trackIndex;
    //大地坐标ｘ
    double x;
    //大地坐标ｙ
    double y;
    int mode1;
    int mode2;
    WayPoint(){}
    WayPoint(double x_,double y_){x=x_;y=y_;}
};

struct State
{
    State(double x_,double y_,double psi_){x=x_;y=y_;psi=psi_;} // 横坐标 纵坐标 航向 曲率
    State(){}
    double x,y,psi,K;
};

struct ControlInfo {
    std::string controlMethod;
    int pathLength;
    std::string modelName;
    State init_state;
    int speed;
    double dt;
    std::vector<aiforce::decision::SinglePoint> pathPoints;
};

double Pt_dist(WayPoint pt1,WayPoint pt2);
double Pt_dist(State pt1,State pt2);
double Pt_dist(aiforce::decision::SinglePoint pt1,aiforce::decision::SinglePoint pt2);
double normalizeAngle(double angle);
double calCurvature(WayPoint pt_prime, WayPoint pt, WayPoint pt_later);
bool IsEqual(double a,double b);   //判断两值是否相等
double getDistancePointToLine(WayPoint A, WayPoint B, WayPoint P); //点P到直线AB的距离
double getDistancePointToLine(State A, State B, State P); //点P到直线AB的距离
int calTargetIndex(State robot_state, std::vector<State> refer_path);


class Filter
{
private:
    std::queue<float> ave_output_list_;
    int ave_num; //均值滤波的阈值
public:
    Filter();
    ~Filter();

    void setting_num(double CurrentSpeed);
    double average_filter(double calculate_angle);

};


#endif