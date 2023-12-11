#ifndef AIFORCE_MODULES_DECISION_DECISION_CONTROLLER_INFO_TRANSFORM_TYPES_H_
#define AIFORCE_MODULES_DECISION_DECISION_CONTROLLER_INFO_TRANSFORM_TYPES_H_

namespace aiforce {
namespace decision {

struct SinglePoint {
    SinglePoint(double x_,double y_,double mode1_, double mode2_){x=x_;y=y_;mode1=mode1_;mode2=mode2_;} // 横坐标 纵坐标 航向 曲率
    SinglePoint(){}
    double x;
    double y;
    int mode1;
    int mode2;
};

}
}

#endif //AIFORCE_MODULES_DECISION_DECISION_CONTROLLER_INFO_TRANSFORM_TYPES_H_
