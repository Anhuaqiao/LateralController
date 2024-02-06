#ifndef LQR_H
#define LQR_H
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include "../Common.h"
#include "Controller.h"

using namespace std;
using namespace Eigen;


class LQR :public aiforce::control::Controller{

public:
    MatrixXd Q;
    MatrixXd R;
private:
    KinematicModel model;
    
public:
    LQR(double x, double y, double psi, double v, double L, double dt, MatrixXd Q, MatrixXd R);
    ~LQR() = default;
    double steer(State cur_state,double cur_speed,double wheel_base, vector<State> refer_path) override;
    MatrixXd calRicatti(MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R);
    };


#endif // LQR_H
