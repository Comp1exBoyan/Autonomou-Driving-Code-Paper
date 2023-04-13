#include "pid.h"


pid_regulator::pid_regulator()
{
    // Kp_x = 1.2;
    // Ki_x = 0.05;
    // Kd_x = 10;

    // Kp_x = 1.2;
    // Ki_x = 0.05;
    // Kd_x = 10;

    // Kp_x = 1.2;
    // Ki_x = 0.05;
    // Kd_x = 10;
    car_pid_v.kp = 1.2;
    car_pid_v.ki = 0.0;
    car_pid_v.kd = 1;

    car_pid_v.error  = 0; 
    car_pid_v.lerror = 0;


    car_pid_yaw.kp = 1.0;
    car_pid_yaw.ki = 0.0;
    car_pid_yaw.kd = 0.8;

    car_pid_yaw.error  = 0; 
    car_pid_yaw.lerror = 0;

    car_pid_v.r_output = 0;
    car_pid_yaw.r_output = 0;

    car_pid_v.r_integer = 0;
    car_pid_yaw.r_integer = 0;

    car_pid_v.r_diff = 0;
    car_pid_yaw.r_diff = 0;


    std::cout << "pid parameters inited" << std::endl;

}



void pid_regulator::state_update(car_pid_state &vehicleState,car_lc_info &car_state)
{
    vehicleState.x = car_state.cur_x;
    vehicleState.y = car_state.cur_y;
    //
    tf::Quaternion quat(car_state.cur_ori_x, car_state.cur_ori_y, car_state.cur_ori_z, car_state.cur_ori_w);
    double roll,pitch,yaw;
    tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
    vehicleState.yaw = yaw;
    vehicleState.speed = WheelCurrentV;
}


double pid_regulator::pid_regulator_v(car_pid_state &s, const double dvelocity, car_pid &car_pid_v)
{
    car_pid_v.error = dvelocity - s.speed;
    car_pid_v.r_integer += car_pid_v.error;
    car_pid_v.r_diff = car_pid_v.error - car_pid_v.lerror;


    car_pid_v.r_output = car_pid_v.kp * car_pid_v.error 
                        + car_pid_v.ki * car_pid_v.r_integer
                        + car_pid_v.kd * car_pid_v.r_diff ; 

    
    if(car_pid_v.r_output > car_pid_v.up_limit) car_pid_v.r_output = car_pid_v.up_limit;

    if(car_pid_v.r_output < car_pid_v.low_limit) car_pid_v.r_output = car_pid_v.low_limit;



    car_pid_v.lerror = car_pid_v.error;


    
    return car_pid_v.r_output ;

}

double pid_regulator::pid_regulator_yaw(car_pid_state &s,const double angle, car_pid &car_pid_yaw)
{
    
    car_pid_yaw.error = angle- s.yaw;
    car_pid_yaw.r_integer += car_pid_yaw.error;
    car_pid_yaw.r_diff = car_pid_yaw.error - car_pid_yaw.lerror;


    car_pid_yaw.r_output = car_pid_yaw.kp * car_pid_yaw.error 
                        + car_pid_yaw.ki * car_pid_yaw.r_integer
                        + car_pid_yaw.kd * car_pid_yaw.r_diff ; 

    if(car_pid_yaw.r_output > car_pid_yaw.up_limit) car_pid_yaw.r_output = car_pid_yaw.up_limit;

    if(car_pid_yaw.r_output < car_pid_yaw.low_limit) car_pid_yaw.r_output = car_pid_yaw.low_limit;

    car_pid_yaw.lerror = car_pid_yaw.error;
    
    
    return car_pid_yaw.r_output;

}