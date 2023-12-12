#ifndef VEHICLE_SIMU
#define VEHICLE_SIMU
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include "KinematicModel.h"
#include "Common.h"
#include <unordered_map>
#include "Controller/stanley.h"
#define PI 3.1415926

using namespace std;

class Vehicle
{
private:

public:
    double x;           // vehicle x position
    double y;           // vehicle y position
    double psi;         // vehicle heading
    double v;           // longitudal velocity
    double L;           // wheelbase
    double time_length; // simulating time
    double dt;          // sampling time
    State vehicle_state;
    State ego_state;
    vector<double> xout;
    vector<double> yout;
    vector<double> psiout;
    vector<double> steerout;
public:
    Vehicle(double x, double y, double psi, double v, double l, double dt);
    void Update(double acc, double delta);
    vector<aiforce::decision::SinglePoint> getPath(vector<aiforce::decision::SinglePoint>::const_iterator path_begin, vector<aiforce::decision::SinglePoint>::const_iterator path_end);
    void Simulator(double time_length, const ControlInfo & controlInfo);
    ~Vehicle();
};

/**
 * @brief The initialization of the Vehicle setting. 
 * @param model_type    kinematic model or kinetic model
 * @param x             the x-coordinate of the vehicle in the world frame
 * @param y             the x-coordinate of the vehicle in the world frame
 * @param yaw           the yaw angle w.r.t. the x coordinate of the world frame
 * @param v             the longitudal velocity of the vehicle in the world frame
 * @param L             the wheelbase of the vehicle
 * @param dt            the sampling time
 * @return              none
 */
Vehicle::Vehicle(double x, double y, double psi, double v, double L, double dt)
{   
    this->x = x;
    this->y = y;
    this->psi = psi;
    this->v = v;
    this->L = L;
    this->dt = dt;
    this->vehicle_state.x = this->x;
    this->vehicle_state.y = this->y;
    this->vehicle_state.psi = this->psi;
    ego_state.x = L/2;
    ego_state.y = 0;
    ego_state.psi = 0;
}

Vehicle::~Vehicle()
{
}

void Vehicle::Update(double acc, double delta){
    x += v* cos(psi)*dt;
    y += v*sin(psi)*dt;
    psi += v / L * tan(delta)*dt;
    psi = normalizeAngle(psi);
    v += acc*dt;
    this->vehicle_state.x = this->x;
    this->vehicle_state.y = this->y;
    this->vehicle_state.psi = this->psi;
}
/**
 * @brief Transform the path under global frame to vehicle frame .
 * @param path1 A vector of SinglePoint under expressed in the gloabl frame.
 * @return A vector of SinglePoint under expressed in the vehicle frame.
 */

vector<aiforce::decision::SinglePoint> Vehicle::getPath(vector<aiforce::decision::SinglePoint>::const_iterator path_begin, vector<aiforce::decision::SinglePoint>::const_iterator path_end){ //https://eclass.hna.gr/modules/document/file.php/TOM6113/active_and_passive_transformations.pdf
    vector<aiforce::decision::SinglePoint> path2;
    aiforce::decision::SinglePoint point2(0,0,0,0);
    for (auto point1 = path_begin; point1 != path_end; ++point1){ //https://blog.csdn.net/Asimov_Liu/article/details/119931291
        if(((*point1).x==0)&&((*point1).y==0)){
            point2.x = 0;
            point2.y = 0;
            path2.emplace_back(point2);
        }
        else{
            double tmp_x = (*point1).x-this->x;
            double tmp_y = (*point1).y-this->y; 
            point2.x = tmp_x*cos(this->psi) + tmp_y*sin(this->psi);
            point2.y = -tmp_x*sin(this->psi) + tmp_y*cos(this->psi);
            path2.emplace_back(point2);
        }
    }
    return path2;
}



void Vehicle::Simulator(double time_length, const ControlInfo & controlInfo){

    double time_now=0;
    aiforce::control::Controller * controller = new Stanley();

    int cur_speed = controlInfo.speed;

    vector<aiforce::decision::SinglePoint>::const_iterator cur_path_begin=controlInfo.pathPoints.begin(), cur_path_end;
    cur_path_end = std::next(controlInfo.pathPoints.begin(), controlInfo.pathLength);
    vector<aiforce::decision::SinglePoint> current_path_in_vehicle,current_path_in_world;
    vector<State> current_path_in_world_state;
    current_path_in_world = slicing<aiforce::decision::SinglePoint>(cur_path_begin,cur_path_end);
    current_path_in_vehicle = getPath(cur_path_begin,cur_path_end);

    controller->UpdatePath(current_path_in_vehicle);
    vector<State> ego_refer_path = controller->RefState_;

    int path_index_now, path_total_index=controlInfo.pathPoints.size();
    path_index_now = calTargetIndex(ego_state, ego_refer_path);

    double steer_angle;


    while (time_now<=time_length)
    {   
        State ego_cur_state(L/2,0,0);
        steer_angle = controller->steer(ego_cur_state, cur_speed, L, ego_refer_path);
        Update(0, steer_angle);
        xout.emplace_back(this->x);
        yout.emplace_back(this->y);
        psiout.emplace_back(this->psi);
        steerout.emplace_back(steer_angle);

        current_path_in_world_state = SingleP2State(current_path_in_world);
        path_index_now = calTargetIndex(vehicle_state, current_path_in_world_state);

        cur_path_begin = std::next(cur_path_begin, path_index_now);
        cur_path_end = std::next(cur_path_begin, controlInfo.pathLength);

        current_path_in_world = slicing<aiforce::decision::SinglePoint>(cur_path_begin,cur_path_end);
        current_path_in_vehicle = getPath(cur_path_begin, cur_path_end); 
        controller->UpdatePath(current_path_in_vehicle);
        ego_refer_path = controller->RefState_;
        time_now += 1;
    }
    
 }

#endif