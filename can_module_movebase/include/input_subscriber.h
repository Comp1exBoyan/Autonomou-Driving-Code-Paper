#ifndef INPUT_SUBSCRIBER_H
#define INPUT_SUBSCRIBER_H

#include <ros/ros.h>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_msgs/VehicleCmd.h>
#include "canthread1_move.h"
#include "std_msgs/String.h"

#include <sensor_msgs/Joy.h>

#define VEL_MAX 5
#define ANGEL_MAX 720

class InputSubscriber
{
public:
   InputSubscriber();
   ~InputSubscriber();
   bool GetStatus();
   double GetSteeringAngle();
   double GetRearVelocity();
   void SetStatus(bool status);

   canthread1* canthread; //  can thread

   // double temp_x;
   // void Subscriber(std_msgs::String Topic,int length,void *calkbackfunc);
  double steering_ratio_;

  int car_run_status;//0: normal front   1:stop  2:backing   


private:
  ros::NodeHandle nh_;
  ros::Subscriber vehicle_cmd_sub_;
  ros::Subscriber twist_sub_;
  ros::Subscriber keyboard_sub_;

  ros::Subscriber joy_sub_;

  void CmdCallback(const autoware_msgs::VehicleCmd::ConstPtr &input_msg);
  void TwistCallback(const geometry_msgs::Twist::ConstPtr &input_twist_msg);
  void TwistStamped2Car(const geometry_msgs::TwistStamped &input_twist_msg);
  void KeyboardCallBack(const std_msgs::String::ConstPtr& msg);

  //void joyCallBack(const sensor_msgs::Joy &joymsg);

  void CanThreadInitial();
  void msgCallBack(const std_msgs::String::ConstPtr& msg);

  double wheel_base_;   // 前后轮轴距
  double wheel_tread_;  // 前轮轴距
  double wheel_radius_; //


  double steer_rate;

  bool twiststamped_;
  bool ctrl_cmd_;
  bool get_cmd_;
  bool m_canisopen;

  int m_run0;

public:
  double force_control;
  double steering_angle;
  double rear_velocity;
  unsigned int direction;
};


#endif // INPUT_SUBSCRIBER_H
