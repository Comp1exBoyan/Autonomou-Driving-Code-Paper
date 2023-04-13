#ifndef CANTHREAD1_MOVE_H
#define CANTHREAD1_MOVE_H

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

#include "yaml-cpp/yaml.h"

using namespace std;
static string config_name = {"/home/znfs/autocar_siat/src/can_module/param/pid.yaml"};
static const YAML::Node& node = YAML::LoadFile(config_name);

extern double WheelCurrentV;
extern int sysdirection;
extern double chelun_angle;
extern double chelun_angle_fangxiangpan;
extern float SysVoltage;

void *receive_func(void* param) ;

 struct PID
    {
      double tar;
      double cur;
      double kp;
      double ki;
      double kd;
      double r_diff;
      double r_output;
      double r_integer;
      double error;
      double lerror;
      int up_limit = 20;
      int low_limit = -20;
    };





class canthread1
{
public:
    canthread1();
    
    VCI_BOARD_INFO pInfo;//用来获取设备信息。
    VCI_BOARD_INFO pInfo1 [50];
    int count;//数据列表中，用来存储列表序号。
    int num;
    
// mpc params  
    int Nx;
    int Ny;
    int HORIZEN;
    int MAX_ITER = 1000;
    double eps = 1e-3;

    double d_pos[3] ; 
    double d_ori[4] ;
    double d_car_v  ;

    double cur_pos[3];
    double cur_ori[4];
    double cur_mov[3];

public:

/*******************************can 通信部分**************************************/
    void sendinstruction(int param0, int param1,int param2,int XCU) ;

    void send2ECU(int ecufunc,int angle, int Direction);

    void send2MCU(int P1,int P2, int P3, int P4, int P5, int P6, float speed);

    void send2MCUtorque(int P1,int P2, int P3, int P4, int P5, int P6, float torque);

    void send2Brake(int direction,int speed,int stopPosition,int mode);

    void sendBrakeInst(int direction,int speed,int stopPosition,int mode);

    void goforward(int st,int speed);

    void goforwardtorque(int torque);

    void goback(int torque);

    void stop();

    void brake();

    void controlfunc(int ecufunc,int mcufunc,float torque,int angle);

    bool initial_openCAN();

    void close_CAN();

    // double PID_VController(double &Feedback_V,double Target_V);

    double get_cur_spd();
    double get_cur_ang();
    double get_cur_ang_fangxiangpan();
    int get_cur_direction();
    float get_voltage();

    double torqueSpeedControl(double targetV,double currentV,double controlRate);

    void send_MCU_PID();

    pthread_t can_rec_th;

    bool isPIDinitial = false;




public:

    double pid_regulter(double &cur, PID *pid_obj);
    PID car_v_pid ; // 车速pid -- 轮速控制已经做好，不用再考虑
    PID steering_pid ;   // 舵角pid
    PID wheel_pid ;  // 轮速pid
    PID pose_pid;  //姿态pid
    
   void fsm_autocar(bool can_state, int control_mode, int car_state, int brake_state, bool joycon_state);
   void fsm_manual_mode(int car_state, int brake_state, bool joycon_state);
   void fsm_auto_mode(int car_state, int brake_state);

};


struct car_lc_info
{

  float cur_x;
  float cur_y;
  float cur_ori_x;
  float cur_ori_y;
  float cur_ori_z;
  float cur_ori_w;
  float cur_vx;
  float cur_vy;
  float cur_omega;

};

struct car_plan_info
{
  float plan_x[100];
  float plan_y[100];
  float plan_ori_x;
  float plan_ori_y;
  float plan_ori_z;
  float plan_ori_w;
  float plan_car_v;

};

#endif // CANTHREAD1_H
