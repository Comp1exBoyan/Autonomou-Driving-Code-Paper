#ifndef __NONLINEAR_CTRL_H_
#define __NONLINEAR_CTRL_H_

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

#include <pure_pursuit.h>
using std::vector;

extern struct Car_state;


const float K1 = -1.0;
const float K2 = -1.6;
const float K3 = -1.0; 

const float lfc = 1.0;  // 车长 

class nonlinear_ctrl
{
    public:
        nonlinear_ctrl();
        {}

    public:
        //非线性控制部分
        double backstepping_tracker(Car_state &s, vector<geometry_msgs::Point> &path);
        //adrc 部分
        double TD();
        double ESO();
        double NLSEF();


}

#endif


