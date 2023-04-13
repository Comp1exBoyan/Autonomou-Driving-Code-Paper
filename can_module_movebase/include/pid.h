#ifndef __PID_H
#define __PID_H



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
#include <pure_pursuit.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

typedef Car_State car_pid_state;
typedef PID car_pid;
class pid_regulator
{

    public:

        car_pid car_pid_v;
        // car_pid car_pid_y;
        car_pid car_pid_yaw;
        // double Kp_x;
        // double Ki_x;
        // double Kd_x;

        // double Kp_y;
        // double Ki_y;
        // double Kd_y;

        // double Kp_yaw;
        // double Ki_yaw;
        // double Kd_yaw;
        pid_regulator();


    public:
        void state_update(car_pid_state &vehicleState,car_lc_info &car_state);
        double pid_regulator_v(car_pid_state &s, const double dvelocity, car_pid &car_pid_v);    
        // double pid_regulator_y(car_pid_state &s, geometry_msgs::Pose &d_pose, car_pid &car_pid_y);    
        double pid_regulator_yaw(car_pid_state &s, const double angle, car_pid &car_pid_yaw);    


};


#endif


