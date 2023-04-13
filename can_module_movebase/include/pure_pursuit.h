#ifndef __PURE_PURSUIT_H
#define __PURE_PURSUIT_H



#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include "controlcan.h"
#include <QMutex>
#include <eigen3/Eigen/Dense>
#include <ctime>
#include <cstdlib>
#include "unistd.h"
#include <iostream>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <ros/ros.h>
#include <vector>
#include "canthread1_move.h"
#include <tf/tf.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
using std::vector;

const double lfc = 1.0;
const double l_c = 1.5 ;
#define LFC_K  0.1
#define L0 0.8
#define L_VEHICLE 1.8


struct Car_State{
        double x = 0;          // m
        double y = 0;          // m
        double yaw = 0;        // degree
        double speed = 0;      // m/s
};



// extern State vehicleState;
class pure_pursuit
{
    public:
        pure_pursuit()
        {}

        // ~pure_pursuit();
    public:
        double pure_pursuit_tracker( Car_State &s,  vector<geometry_msgs::Point>
         &path, int &lastIndex);
        
        int get_goal_index( Car_State &s,  vector<geometry_msgs::Point> &path);
        bool send_control_value(double delta, double ind);
        void state_update(Car_State &vehicleState, car_lc_info &car_state);
        double steering_control(double tar_angle, double cur_angle, const int Kp);

    private:
        
};

#endif


