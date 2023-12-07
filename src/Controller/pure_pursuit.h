#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H
#include <iostream>
#include <iomanip>
#include <math.h>
#include <limits.h>
#include <ctime>
#include <fstream>
#include<eigen3/Eigen/Dense>
#include "../Common.h"
using namespace std;
class PurePursuit {
public:
    int calTargetIndex(State cur_state, vector<State> refer_path, double l_d);

    double purePursuitControl(State cur_state,double cur_speed, vector<State> refer_path,double L);
};

#endif // PURE_PURSUIT_H
