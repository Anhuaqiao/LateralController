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
    State(double x_,double y_,double psi_,double psi_rate_){x=x_;y=y_;psi=psi_;psi_rate=psi_rate_;} // 横坐标 纵坐标 航向 曲率
    State(){}
    double x,y,psi,psi_rate,K;
};

struct ControlInfo {
    std::string controlMethod;
    int pathLength;
    std::string modelName;
    State init_state;
    double speed;
    double dt;
    std::vector<aiforce::decision::SinglePoint> pathPoints;
};

double normalizeAngle(double angle);
double calCurvature(WayPoint pt_prime, WayPoint pt, WayPoint pt_later);
bool IsEqual(double a,double b);   //判断两值是否相等
int calTargetIndex(State robot_state, std::vector<State> refer_path);

template<typename T>
double dotProduct(const T& p1, const T& p2) {
    return p1.x * p2.x + p1.y * p2.y;
}

template<typename T>
double Pt_dist(const T& pt1, const T& pt2) {
    return sqrt(pow((pt1.x - pt2.x), 2) + pow((pt1.y - pt2.y), 2));
}

template<typename T>
double getDistancePointToLine(const T& A, const T& B, const T& P) {
    double area = abs((B.x - A.x) * (P.y - A.y) - (P.x - A.x) * (B.y - A.y));
    double base = Pt_dist(A, B);
    return area / base;
}

template <typename T>
std::vector<T> slicing(typename std::vector<T>::const_iterator X,
                  typename std::vector<T>::const_iterator Y)
{
    // Copy the element
    std::vector<T> vector(X, Y);
 
    // Return the results
    return vector;
}
std::vector<State> SingleP2State(std::vector<aiforce::decision::SinglePoint> & singlep);


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