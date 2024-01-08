#ifndef COMMON_H
#define COMMON_H
#include <vector>
#include <queue>
#include "../info_transform_types.h"
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
    bool operator==(const State& other) const {
        return (x == other.x && y == other.y && psi == other.psi && psi_rate == other.psi_rate);
    }
};

template <typename T>
std::vector<T> getSlice(const std::vector<T>& vec, size_t start, size_t end) {
    if (start >= vec.size()) {
        return {}; // 如果起始位置大于向量大小，则返回空向量
    }

    // 修正结束位置，如果超出向量大小则设置为向量的末尾
    end = std::min(end, vec.size());

    // 使用迭代器和标准算法来获取切片
    std::vector<T> result;
    std::copy(vec.begin() + start, vec.begin() + end, std::back_inserter(result));
    return result;
}


template<typename T>
State computeUnitDirection(const T& p1, const T& p2) {
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;

    // Calculate the distance between points
    double distance = std::sqrt(dx * dx + dy * dy);

    // Calculate the unit direction vector components
    double unitX = dx / distance;
    double unitY = dy / distance;
    double psi = atan2(unitY, unitX);

    State output(unitX,unitY,psi);

    // Create and return the unit direction vector
    return output;
}

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
int calTargetIndex(State robot_state, std::vector<State>& refer_path);
std::string doubleToStringWithPrecisionLimit(double value, int precision);

template <typename T>
double calculateMean(const std::vector<T>& vec) {
    double sum = 0.0;

    // 计算总和
    for (double num : vec) {
        sum += num;
    }

    // 计算平均值
    double mean = sum / static_cast<int>(vec.size());
    return mean;
}


template<typename T>
double dotProduct(const T& p1, const T& p2) {
    return p1.x * p2.x + p1.y * p2.y;
}

template<typename T>
inline double Pt_dist(const T& pt1, const T& pt2) {
    return sqrt(pow((pt1.x - pt2.x), 2) + pow((pt1.y - pt2.y), 2));
}

template <typename T>
int sign(T val) {
    return (T(0) < val) - (val < T(0));
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

State SingleP2State(const aiforce::decision::SinglePoint & singlep);
std::vector<State> SingleP2State(std::vector<aiforce::decision::SinglePoint> & singlep);

template <typename T>
T degree2rad(T deg){
    return deg*PI/180;
}

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