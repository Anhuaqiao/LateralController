#ifndef VEHICLE_SIMU
#define VEHICLE_SIMU
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include "KinematicModel.h"
#include "Common.h"
#include <boost/range/combine.hpp>
#include <unordered_map>

using namespace std;

class Vehicle
{
private:

public:
    double x,y,psi,v,L,dt;
public:
    Vehicle(double x, double y, double psi, double v, double l, double dt);
    void Update(int acc, int delta);
    vector<aiforce::decision::SinglePoint> getPath(vector<aiforce::decision::SinglePoint> & path);
    ~Vehicle();
};

/**
 * @brief The initialization of the Vehicle setting. 
 * @param model_type    kinematic model or kinetic model
 * @param x             the x-coordinate of the vehicle in the world frame
 * @param y             the x-coordinate of the vehicle in the world frame
 * @param yaw           the yaw angle w.r.t. the x coordinate of the world frame
 * @param v             the longitudal velocity of the vehicle in the world frame
 * @param l             the wheelbase of the vehicle
 * @param dt            the sampling time
 * @return              none
 */
Vehicle::Vehicle(double x, double y, double psi, double v, double l, double dt)
{   
    this->x = x;
    this->y = y;
    this->psi = psi;
    this->v = v;
    this->L = L;
    this->dt = dt;
}

void Vehicle::Update(int acc, int delta){
    x += v* cos(psi)*dt;
    y += v*sin(psi)*dt;
    psi += v / L * tan(delta)*dt;
    psi = normalizeAngle(psi);
    v += acc*dt;
}
/**
 * @brief Transform the path under global frame to vehicle frame .
 * @param path1 A vector of SinglePoint under expressed in the gloabl frame.
 * @return A vector of SinglePoint under expressed in the vehicle frame.
 */

vector<aiforce::decision::SinglePoint> Vehicle::getPath(vector<aiforce::decision::SinglePoint> & path1){ //https://eclass.hna.gr/modules/document/file.php/TOM6113/active_and_passive_transformations.pdf
    vector<aiforce::decision::SinglePoint> path2;
    aiforce::decision::SinglePoint point2;
    for (const auto & point1 : path1) { //https://blog.csdn.net/Asimov_Liu/article/details/119931291
        int tmp_x = point1.x-this->x;
        int tmp_y = point1.y-this->y; 
        point2.x = tmp_x*cos(this->psi) + tmp_y*sin(this->psi);
        point2.y = -tmp_x*sin(this->psi) + tmp_y*cos(this->psi);
        path2.emplace_back(point2);
    }
    return path2;
}

Vehicle::~Vehicle()
{
}

#endif