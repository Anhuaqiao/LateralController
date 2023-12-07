#ifndef CHHROBOTICS_CPP_KINEMATICMODEL_H
#define CHHROBOTICS_CPP_KINEMATICMODEL_H
#include <iostream>
#include <vector>
#include <cmath>
#include<eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

class KinematicModel {
public:
    double x,y,psi,v,L,dt;
public:
    KinematicModel();
    void Initialization(double x, double y, double psi, double v, double L, double dt);
    vector<double>getState();

    void updateState(double accel, double delta_f);

    vector<MatrixXd> stateSpace(double ref_delta, double ref_yaw);

};


#endif //CHHROBOTICS_CPP_KINEMATICMODEL_H
