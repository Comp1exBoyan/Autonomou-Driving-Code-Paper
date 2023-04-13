/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QApplication>
//#include "../include/can_module/main_window.hpp"
#include "gl_data.h"
#include "input_subscriber.h"
#include <vector>
using std::vector;
#include "pure_pursuit.h"
#include "pid.h"

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <fstream>

std::ofstream out("/home/znfs/data.txt");

ros::Publisher *p_mycmdvel_pub;

ros::Publisher *p_Control_pub;

ros::Publisher *p_LearningData_pub;
float current_human_cmd_ang;
float current_human_cmd_spd;
float current_carspd;
float current_carangl;
float current_carangl_fangxiangpan;
bool human_enabled;

float bias_fangxiang = 0;

/*****************************************************************************
** Main
*****************************************************************************/
int flag = 1 ;
int timer = 0 ;
const int Kp = 1.0;
extern double chelun_angle;

const int LOOP_RATE = 50;
Car_State vehicleState;
#define _ax_jiasu 5      //1 ~~~~-1  default  1
float speed = 0;
#define _ax_shache 2    //1 ~~~~-1  default  1
float shache;
#define _bt_shousha 5  //0,1 default 0
bool shousha;
#define _ax_angle 6   //-1 --- 1, default 0
#define _ax_backward 7   //-1 --- 1, default 0
float angle;
#define _bt_shineng 1
bool auto_sw;
#define _bt_auto_sw 0

//#define _bt_auto 1


#define bar_y_fb 1
#define bar_x_lr 0

float thr_fb; 
float thr_lr;

// #define _ax_move  7

int direction;
int fb_direction;
int lr_direction;

bool shineng;
bool avoid;

#define MANUAL       1 // 
#define AUTO     2

//InputSubscriber IS;

float ext_curspd;
float ext_curangle;
int ext_currentdir;

bool first_msg1 = true;
bool first_msg2 = true;

float angth = 0;
float angth1 = 0;
float angth_mx = 0;

geometry_msgs::PoseArray tagPoses_world;

car_lc_info car_lc_obj;
double delta;
int enable_trigger[2] = {0,0};

void joyCallBack(const sensor_msgs::Joy::ConstPtr& joymsg)
{

  angle = joymsg->axes[_ax_angle];//-1==1
  speed = 1-joymsg->axes[_ax_jiasu];//0```2
  shache = 1-joymsg->axes[_ax_shache]; //0````2
  shousha = joymsg->buttons[_bt_shousha];
  shineng = joymsg->buttons[_bt_shineng];//use joy control
  auto_sw = joymsg->buttons[_bt_auto_sw];//use auto control
                                         //else :human control
  direction = joymsg->axes[_ax_backward];//0,1 forward; -1 backward
  // direction = joymsg->axes[_ax_move];
  thr_fb = joymsg->axes[bar_y_fb];
  thr_lr = joymsg->axes [bar_x_lr];
  // ROS_INFO("speed %f",speed);
 // if(thr_fb > 0) fb_direction = 1;
 
}

void velCallBack(const geometry_msgs::TwistConstPtr& cmd_msg)
{
  float spd = cmd_msg->linear.x;
  float rot = cmd_msg->angular.z;
/*
  IS.steering_angle = rot*180/3.1415926 * 25;
  if(IS.steering_angle > 0)
      IS.canthread->send2ECU(1, IS.steering_angle, 0);
    else if(IS.steering_angle < 0)
      IS.canthread->send2ECU(1, -IS.steering_angle, 1);
    else
      IS.canthread->send2ECU(1, 50, 0);

  //spd = spd*20;
  if (abs(spd)>1) spd = 1;
  if(spd>0)
  {
    spd = spd*20;
    IS.canthread->send2MCU(1,0,1,1,0,1,spd);
  }
  else if(spd<0)
  {
    spd = spd*5;
    IS.canthread->send2MCU(1,0,2,1,0,1,-spd);
  }
  else{
    IS.canthread->stop();
  }

  std::cout<<"run cmd: spd="<<spd<<"km/h"<<" rot="<<IS.steering_angle/25<<std::endl;
 */
}

float lastcmd_spd;
float lastcmd_ang;
float back_div= 0;
int div_cnt = 0 ;
int start_flag = 0;

void back_control(InputSubscriber &IS, float currentspd, float target_spd)
{
  IS.canthread->send_MCU_PID();
  if(abs(currentspd) >0.5)
  {
     IS.canthread->send2MCU(1,0,2,1,0,0,0);
     //IS.canthread->send2MCU(1,0,2,1,0,0,-1);//
  }
  else
  {
    IS.canthread->send2MCU(1,0,2,1,0,1,-1);
  }

  if(abs(currentspd) >2)
  {
    IS.canthread->send2MCU(1,0,2,1,0,0,0);
  }
}




void manual_ctrl(InputSubscriber &IS)
{
    float car_speed; 
    car_speed = speed * 10; //%20 power max
    float carangle = angle*500;//-300---300

   // IS.steering_angle = angle*300;//-180---180
    //float currentspd=ext_curspd;
      //get real spd and angle
  double real_carang = -1*IS.canthread->get_cur_ang();//in dushu
  float  currentspd = IS.canthread->get_cur_spd(); //m/s



    IS.canthread->send_MCU_PID();
    if(direction != -1)//foward
    {
        IS.canthread->send2MCU(1,0,1,1,0,1,car_speed);
   //     ROS_INFO("forward!!  Car Speed : %f", car_speed);
        lastcmd_spd = car_speed;
       // std::cout<<"current v==============="" ang="<<ext_curangle<<currentspd<<"cmd="<<car_speed<<std::endl;
        std::cout<<"current v==============="<<currentspd<<  " ang="<<ext_curangle<<"cmd= "<<car_speed<<std::endl;
    }
    else if(direction == -1) //back ward
    {    
        std::cout<<"current v==============="<<currentspd<<  " ang="<<ext_curangle<<"cmd= -30"<<std::endl;
        back_control(IS, currentspd, 1);
     //  IS.canthread->send2MCU(1,0,2,0,0,1,10);
       
    
       float backspd = -1* car_speed*0.02;
   //    ROS_INFO("backing!!  Car Speed : %f", backspd);
       lastcmd_spd = backspd;  
    }
   // ROS_INFO("Steering_angle: %f", carangle);
    if(carangle > 0)
      IS.canthread->send2ECU(1, carangle, 0);
    else if(carangle < 0)
      IS.canthread->send2ECU(1, -carangle, 1);
    else
      IS.canthread->send2ECU(1, 0, 0); //zero point of the fangxiangpan

    lastcmd_ang = carangle;


  geometry_msgs::Twist contol_data;

  contol_data.linear.x = car_speed;//x: target speed
  contol_data.linear.y = currentspd;//x: target speed
  contol_data.angular.x = lastcmd_ang;
  contol_data.angular.y = real_carang;
  p_Control_pub->publish(contol_data);
  current_human_cmd_spd = lastcmd_spd/5/3.6;
  current_human_cmd_ang = carangle/IS.steering_ratio_; 

}


//add by jgl 2022-10-09
//the pid contol for the car speed
float value_i = 0;
float spd_last = 0;
float PID_spd(double value_target, double value_now, double value_last)
{
    double kp = 1;
    double ki = 0.05;
    double kd = 0.1;
    float dt = 0.05;

    double value_p = value_target - value_now;
    value_i += (value_target - value_now)*dt;
    double value_d = (value_now-value_last)/dt;

    float control_value = kp*value_p + ki*value_i + kd*value_d;
    return control_value;
}
//add by jgl 2022-10-09
//the pid contol for the car speed

//add by jgl 2022-10-10
//the pid control for the car angle
float value_i_ang = 0;
float angle_last = 0;
float PID_angle(double value_target, double value_now, double value_last)
{
    double kp = 1.6;
    double ki = 0;
    double kd = 0.5;
    float dt = 0.05;

    double value_p = value_target - value_now;
    value_i_ang += (value_target - value_now)*dt;
    double value_d = (value_now-value_last)/dt;

    float control_value = kp*value_p + ki*value_i_ang + kd*value_d;
    return control_value;
}

float bangbang_contol_ang(double value_target, double value_now)
{
  float L_MAX = 30;
  float R_MAX = -30;
  float Zero = 0;

  float err = value_target - value_now;
  if((abs(value_target)<3)||(angth1 <=5 ))
  {
    return Zero;
  }

  if(value_target*value_now >0)
  {
    if((value_target>value_now)&&(value_now>0))
    {
      return L_MAX;
    }
    else if((value_target<value_now)&&(value_now>0))
    {
      return Zero;
    }
     else if((value_target>value_now)&&(value_now<0))
    {
      return Zero;
    }
    else if((value_target<value_now)&&(value_now<0))
    {
      return R_MAX;
    }
    else
    {
      return value_target;
    }
  }
  else if(value_target*value_now <0)
  {
    if(value_target>0)
    {
      return L_MAX;
    }
    else
    {
      return R_MAX;
    }
  }
  else if(value_target*value_now ==0)
  {
    if(value_target == 0)
    {
      return Zero;
    }
    else if(value_target>0)
    {
      return L_MAX;
    }
    else
    {
      return R_MAX;
    } 
  }
  else
  {
    return value_target;
  }
  
}



//add by jgl 2022-10-10
//the pid control for the car angle

int state_from_planner = 0;//0: normal; 1: stop; 2 back;


void auto_ctrl(InputSubscriber &IS, float zero_bias_angle)
{
  if(IS.car_run_status != 0)
  {
    IS.canthread->stop();
    IS.canthread->send2ECU(1, 0, 0);
    std::cout<<" send stop"<<std::endl;
    return;
  }

  if(state_from_planner !=0 )
  {
    IS.canthread->stop();
    IS.canthread->send2ECU(1, 0, 0);
    std::cout<<"planner send stop"<<std::endl;
    return;
  }
  if(abs(zero_bias_angle)>5)
  {
    IS.canthread->stop();
    IS.canthread->send2ECU(1, 0, 0);
    std::cout<<"angle zero bias error>5, need recalibration !!!!!"<<std::endl;   //零漂大于5度则需要重新标定
    return;
  }


  //get plan V and w
  float carangle_send = IS.steering_angle;//40*lunzi jiaodu pianyiliang 250!!!!!!!!!!!!!
  float carspd_send = IS.rear_velocity*3.6*5;//target spd m/s  --->zhuansu baifenbi
 // if(carspd >20) carspd = 20;
  float targetspd = IS.rear_velocity;// m/s
  float targetangle = IS.steering_angle/IS.steering_ratio_; //wheel dushu 

 
  //get real spd and angle
  double real_carang = -1*IS.canthread->get_cur_ang();//in dushu
  float  currentspd = IS.canthread->get_cur_spd(); //m/s

  float spderr = targetspd - currentspd;    //in m/s
  float angerr = targetangle - real_carang;//in dushu

  //pid control of the speed
  float pidspd = PID_spd(targetspd, currentspd, spd_last);
  float carspd_send_PID = 5*3.6*pidspd;
  if(carspd_send_PID>30) 
  {
    carspd_send_PID = 30;
  }
  spd_last = currentspd;
 // std::cout<<"contol speed pid ="<<carspd_send_PID<<" valuei"<<value_i<<std::endl;
  carspd_send = carspd_send_PID; // using pid 
  float speed_lim_down = 0.75;
  if((angth_mx> 8)&&(angth_mx <20))
  {
    std::cout<<"spd low1"<<std::endl;
    carspd_send = carspd_send*speed_lim_down;
  }
  else if((angth_mx >= 20)&&(angth_mx<=40))
  {
    std::cout<<"spd low2"<<std::endl;
    carspd_send = carspd_send*speed_lim_down*speed_lim_down;
  }
  else if(angth_mx >40)
  {
    carspd_send = 0;
  }
  else if(abs(real_carang)>15)
  {
    carspd_send = carspd_send*speed_lim_down; //car angle too large , slow down
  } 

  
  //pid control of the speed

  //pid control of the angle
  float pidang = PID_angle(targetangle, real_carang, angle_last);
  float bangbangang = bangbang_contol_ang(targetangle, real_carang);
 // float carang_send_PID = pidang * IS.steering_ratio_;
  float carang_send_PID = (bangbangang + zero_bias_angle) * IS.steering_ratio_; //mod by jgl 2022 11 02

 // std::cout<<"pid="<<pidang<<"  bangbang"<<bangbangang<<std::endl; 

  angle_last = real_carang;
  carangle_send = carang_send_PID;
  //pid control of the angle

  geometry_msgs::Twist contol_data;

  contol_data.linear.x = targetspd;//x: target speed
  contol_data.linear.y = currentspd;//x: target speed
  contol_data.angular.x = targetangle;
  contol_data.angular.y = real_carang;
  p_Control_pub->publish(contol_data);

  //send the command
  if(carangle_send >= 0 )
        {
          if(carangle_send > 700) 
          {
            carangle_send=700;
          }
          IS.canthread->send2ECU(1, carangle_send, 0);
          lastcmd_ang = carangle_send;
        }
  else if(carangle_send < 0)
        {
          float rcmd = carangle_send;
          if(rcmd <-700) 
          {
            rcmd = -700;
          }
          IS.canthread->send2ECU(1, -rcmd, 1);
          lastcmd_ang = rcmd;
        }     
   

   if( carspd_send >0.1 ) 
    {
      IS.canthread->send2MCU(1,0,1,1,0,1,carspd_send);//niuju kongzhi
    }
    else if ( carspd_send < -0.1)
    {
      //IS.canthread->send2MCU(1,0,2,0,0,1,carspd*0.2);//    
       back_control(IS, currentspd, 1);
    }
    else{
      IS.canthread->stop();
    }
    IS.canthread->send_MCU_PID();

 // lastcmd_ang = carangle;
    lastcmd_spd = carspd_send;
  //  ROS_INFO("angle cmd steering_angle: %f", lastcmd_ang );
 // ROS_INFO("Car Speed : %f", lastcmd_spd);

}

int info_count=0;

void psaCallback(const geometry_msgs::PoseArrayConstPtr pasMsg);

nav_msgs::Odometry cur_lc;
void lcCallBack(const nav_msgs::OdometryConstPtr& lc_msg)
{
   nav_msgs::Odometry temp_lc = *lc_msg;
   car_lc_obj.cur_x = temp_lc.pose.pose.position.x;
   car_lc_obj.cur_y = temp_lc.pose.pose.position.y;
   car_lc_obj.cur_ori_x  = temp_lc.pose.pose.orientation.x;
   car_lc_obj.cur_ori_y  = temp_lc.pose.pose.orientation.y;
   car_lc_obj.cur_ori_z  = temp_lc.pose.pose.orientation.z;
   car_lc_obj.cur_ori_w  = temp_lc.pose.pose.orientation.w;
   car_lc_obj.cur_vx     = temp_lc.twist.twist.linear.x; //speed from location
   car_lc_obj.cur_vy     = temp_lc.twist.twist.linear.y;
   car_lc_obj.cur_omega  = temp_lc.twist.twist.angular.z ;

   cur_lc = temp_lc;
  // printf("-----lc data----- \n");
   //printf("current car position : x: %f y: %f \n",  car_lc_obj.cur_x,  car_lc_obj.cur_y);
  // printf("current car velocity : vx: %f vy: %f \n ", car_lc_obj.cur_vx, car_lc_obj.cur_vy);
}

float autocmd_spd;
float autocmd_ang;

int main(int argc, char **argv) {
    ros::init(argc, argv, "input_subscriber_main");
    InputSubscriber IS;
    ros::Rate r(LOOP_RATE);

    ros::NodeHandle nh_;
    ros::Subscriber joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, joyCallBack);
    ros::Subscriber  state_sub  = nh_.subscribe<nav_msgs::Odometry>("/odom", 10, lcCallBack);
    //ros::Subscriber  traj_sub  = nh_.subscribe<nav_msgs::Path>("/searched_path", 10, pathCallBack);
    ros::Subscriber local_path_sub = nh_.subscribe<geometry_msgs::PoseArray>("localplan_poses",10,psaCallback);
    ros::Publisher refined_odom_pub = nh_.advertise<nav_msgs::Odometry>("/odom_refined", 10);  // 

    p_mycmdvel_pub = new ros::Publisher();
    *p_mycmdvel_pub = nh_.advertise<geometry_msgs::Twist>("/my_contol", 10);

    p_Control_pub = new ros::Publisher();
    *p_Control_pub = nh_.advertise<geometry_msgs::Twist>("/contol_data", 10);

    p_LearningData_pub = new ros::Publisher();
    *p_LearningData_pub = nh_.advertise<geometry_msgs::PoseArray>("/learning_data",10);

    //add by jgl
    //ros::Subscriber  vel_sub = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel_ang_spd", 1, velCallBack);

    int lastindex = 0;

    //vector <geometry_msgs::Point> track_path; 
    while(ros::ok())
    {
      r.sleep();
      ros::spinOnce();
      double real_carspd = IS.canthread->get_cur_spd();
      double real_carang = -1 * IS.canthread->get_cur_ang();
      int direction = IS.canthread->get_cur_direction();
      current_carspd = real_carspd;
      current_carangl = real_carang;
      current_carangl_fangxiangpan = -1 * IS.canthread->get_cur_ang_fangxiangpan();
      bias_fangxiang = current_carangl- current_carangl_fangxiangpan;   // 方向盘零漂
      std::cout<<"bias = "<<bias_fangxiang<<std::endl;
      autocmd_spd = IS.rear_velocity;// m/s
      autocmd_ang = IS.steering_angle/IS.steering_ratio_; //wheel dushu 


      //float vol =  IS.canthread->get_voltage();
      if(direction == 0) 
      {
        real_carspd = real_carspd;
      }
      else if( direction == 1)
      {
        real_carspd = real_carspd;
      }
      else if(direction == 2)
      {
        real_carspd = -real_carspd;

      }
      else{}

      // 车辆状态反馈

      // ext_curspd = real_carspd;
      // ext_curangle = real_carang;
      // ext_currentdir = direction;

      cur_lc.header.stamp = ros::Time::now();
      cur_lc.twist.twist.linear.x = real_carspd;
      cur_lc.twist.twist.linear.y = 0;
      refined_odom_pub.publish(cur_lc);

      if(shineng)//use joy control
      {
         if (shache > 1) IS.canthread->send2Brake(1,100,shache*50,2);
         else
         {
           IS.canthread->send2Brake(1,100,0,2);
         }
          manual_ctrl(IS);
          human_enabled = true;
      }
      /***************************自动控制部分************************************/ 
      else if(auto_sw)
      {//use auto control, from cmd_vel by planner
         auto_ctrl(IS,-bias_fangxiang);
         human_enabled = false;
      }
      else
      {
          human_enabled = false;
      }
      //cout the 
      info_count ++;                                                                                                            
      if(info_count == 10 )
      {
    //    std::cout<<"_________________real spd="<<real_carspd<<" real angle="<<real_carang<<" dir"<<std::endl;
        info_count = 0;
      }

    }//while

    out.close();
    
}


pid_regulator v_pid;
pid_regulator yaw_pid;
car_pid_state car_state;
const double dvelocity = 1.5;

int backcount = 0;
int stopcount = 0;
//int backcount = 0;



void psaCallback(const geometry_msgs::PoseArrayConstPtr pasMsg)     
{
  tagPoses_world = *pasMsg;
  double v_output = 0;
  double angle_output = 0;
  int sz = tagPoses_world.poses.size();

  geometry_msgs::Twist currentorder;
  
 // currentorder.

  // 获取 期望位姿态
  geometry_msgs::Pose tmpps = tagPoses_world.poses[1];
  float tx = tmpps.position.x;
  float ty = tmpps.position.y;
  float t_ori_x = tmpps.orientation.x;
  float t_ori_y = tmpps.orientation.y;
  float t_ori_z = tmpps.orientation.z;
  float t_ori_w = tmpps.orientation.w;


  // 获取期望的 位姿

  tf::Quaternion quat(t_ori_x,t_ori_y,t_ori_z,t_ori_w);
  double roll,pitch,yaw;
  tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
  float t_yaw = yaw; 


  if(tx<0)
  {
   //   std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!there is fall back!!!!!!!!!!!!!"<<"backcount=" 
   //   <<backcount<<" stopcount="<<stopcount
   //   <<"state="<<state_from_planner<<std::endl;
      backcount++;
  }
  else
  {
      backcount = 0;
  }

  if(backcount >= 2)
  {
    state_from_planner = 1;
    //stopcount ++;
 //   std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!fall back confrimed!!!!!!!!!!!!!"<<std::endl;
  }

  if((state_from_planner ==1)&&(stopcount <10))//stop for 20 cycle
  {
    stopcount ++;
    //send stop order
    currentorder.angular.z = 0;
    currentorder.linear.x = 0;
    p_mycmdvel_pub->publish(currentorder);
    std::cout<<"!!!!!!!!!!!!!send stop command!!!!!!!!!"<<stopcount<<std::endl;
    //send stop order
    return;
  }
  else if(stopcount>=10)// after 10 cycle, release and check
  {
    std::cout<<"!!!!!!!!!!!!!stop released, goto replan!!!!!!!!!"<<std::endl;
    state_from_planner = 0;
    backcount = 2;
    stopcount = 0;
    return;
  }


  //backcount = 0;

  if(state_from_planner == 0)
  {
    geometry_msgs::Pose psi = tagPoses_world.poses[6];
    float x = psi.position.x;
    float y = psi.position.y;
    geometry_msgs::Quaternion quant = psi.orientation;
    float angle = tf::getYaw(quant);

    float bangle = abs(atan2f(y,x))*180/3.1415926;
    angth = bangle;// angle > 15 speed down

    geometry_msgs::Pose psi4 = tagPoses_world.poses[4];
    float x4 = psi4.position.x;
    float y4 = psi4.position.y;
    geometry_msgs::Quaternion quant4 = psi4.orientation;
    float angle4 = tf::getYaw(quant4);

    float bangle4 = abs(atan2f(y4,x4))*180/3.1415926;
    angth1 = bangle4;// angle > 15 speed down

    int id;
    float angle_m_abs = 0;
    float angle_m_pn = 0;
    geometry_msgs::Pose psidx;
    float current_ang = current_carangl;// current wheel
    for(int i=4;i<sz;i++)
    {
      float angle_x = tf::getYaw(tagPoses_world.poses[i].orientation); // car pose angle
      float bangle_x = atan2f(tagPoses_world.poses[i].position.y,tagPoses_world.poses[i].position.x)*180/3.1415926; //car position bias
      float abs_bangle_x = abs(bangle_x);

      if((angle_m_abs<abs_bangle_x)&&(tagPoses_world.poses[i].position.x>0.5))
      {
        angle_m_abs = abs_bangle_x;
        angle_m_pn = bangle_x;
        id = i;
        psidx = tagPoses_world.poses[i];
      }
    }
    angth_mx = angle_m_abs; //the max angle bias

    std::cout<<"normal plan, found "<<id<<"th plan point, max="<<angth_mx
    <<" x="<<psidx.position.x<<" y="<<psidx.position.y<<std::endl;
    std::cout<<"autocmd="<<autocmd_spd<<" ang="<<autocmd_ang<<std::endl;

    geometry_msgs::Pose control_data;
    control_data.position.x = current_carspd;//current spd
    control_data.position.y = current_human_cmd_spd;//human cmd spd

    control_data.orientation.x = current_carangl;//current ang
    control_data.orientation.y = current_human_cmd_ang;//human cmd ang

    control_data.orientation.z = autocmd_ang; //auto command angle
    control_data.orientation.w = autocmd_spd;//auto command spd
    //tagPoses_world.poses.push_back(control_data);
    tagPoses_world.poses[0] = control_data;
    if(human_enabled)
    {
      p_LearningData_pub->publish(tagPoses_world);
    }


    //std::cout<<"x="<<x<<" y="<<y<<" a="<<bangle<<std::endl;

    //std::cout<<"plan point="<<"x,y,yaw="<<x<<"  "<<y<<" "<<angle<<std::endl;
    //std::cout<<"carspd="<<car_lc_obj.cur_vx<<" car-w="<<car_lc_obj.cur_omega<<std::endl;
    //float ouspd,ouphi;
   // ou_ctl(ouspd, ouphi, x, y, angle,ext_curspd, ext_curangle);
  //  std::cout<<"contol by ou spd="<<ouspd<<"  ang="<<ouphi*180/3.1415926<<std::endl;
   
   
    car_state.x = car_lc_obj.cur_x;
    car_state.y = car_lc_obj.cur_y;
    tf::Quaternion quat(car_lc_obj.cur_ori_x, car_lc_obj.cur_ori_y, car_lc_obj.cur_ori_z, car_lc_obj.cur_ori_w);
    double roll,pitch,yaw;
    tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
    vehicleState.yaw = yaw;
    vehicleState.speed = WheelCurrentV;
    
    v_output = v_pid.pid_regulator_v(car_state, dvelocity, v_pid.car_pid_v);
    angle_output = yaw_pid.pid_regulator_yaw(car_state, angle, yaw_pid.car_pid_yaw);


    geometry_msgs::Pose tt = tagPoses_world.poses[sz-1];
    float dd = tt.position.x*tt.position.x;
    float dd_m = 5;
    float spdlim = sqrt(dd)/dd_m;

    float theta = atan(abs(ty/tx));
    float spdlim2 = 3.1415926/4/(2*theta+3.1415926/4); //jiaodu yueda ,xiansu

    //std::cout<<"spd lim1  =" <<spdlim <<"  spd lim ang  ="<<spdlim2<<std::endl;

    v_output = spdlim * spdlim2 * v_output /1.5;

    if(sqrt(dd)<0.5) 
    { 
      v_output = 0;
      std::cout<<"goal reached!"<<std::endl;
    }

 //   std::cout<<"_________my contol:"<<"v="<<v_output<<"  ang="<<angle_output*180/3.1415926<<std::endl;

    currentorder.angular.z = angle_output;
    currentorder.linear.x = v_output;
    p_mycmdvel_pub->publish(currentorder);

    return;

  }

  //std::cout<<"new plan get!"<<sz<<std::endl;
}
