#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H
#include <iostream>
#include <iomanip>
#include <limits.h>
#include <ctime>
#include <fstream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "../common/Common.h"
#include "Controller.h"
extern "C" {
    #include "../common/dubins.h"
}   


using namespace std;
class PurePursuit: public aiforce::control::Controller {
private:
    void calcLookAHeadPoint(State cur_state, vector<State>& refer_path);
    void OutOfPath(State cur_state, vector<State>& refer_path);
    static void call_dubins_shortest_path(DubinsPath* path, double q0[3], double q1[3], double radius_min){
        dubins_shortest_path( path, q0, q1, radius_min);
    }
    static double call_dubins_path_length(DubinsPath* path){
        double current_dubins_length = dubins_path_length( path );
    }
private:
    DubinsPath path;
public:
    double radius_min, alpha_max, beta_max;
    int i_pw, i_pd;
    vector<State> pl;
    PurePursuit();
    ~PurePursuit(){};
    double steer(State cur_state,double cur_speed,double wheel_base, vector<State> refer_path) override;
};

#endif // PURE_PURSUIT_H
