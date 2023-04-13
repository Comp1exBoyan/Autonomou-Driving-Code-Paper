#include "input_subscriber.h"
#include "std_msgs/String.h"

InputSubscriber::InputSubscriber(): nh_("")
{
  nh_.param("wheel_base", wheel_base_, 1.855);
  nh_.param("wheel_tread", wheel_tread_, 1.31);
  nh_.param("wheel_radius", wheel_radius_, 0.341);
  nh_.param("twiststamped", twiststamped_, true);
  nh_.param("steering_ratio", steering_ratio_, 20.0);

  bool twist_sub = false;

  nh_.param("twist_sub", twist_sub, true);

  canthread = new canthread1;
  CanThreadInitial();
  get_cmd_ = false;
  steering_angle = rear_velocity = 0;
  steer_rate = 20;
  force_control = 0;
  direction = 0 ;

  twist_sub_ = nh_.subscribe("/cmd_vel_ang_spd", 1, &InputSubscriber::TwistCallback, this);
  //my_contol
  //twist_sub_ = nh_.subscribe("/my_contol", 1, &InputSubscriber::TwistCallback, this);
  vehicle_cmd_sub_ = nh_.subscribe("/vehicle_cmd", 1, &InputSubscriber::CmdCallback, this);
  keyboard_sub_ = nh_.subscribe("/keys", 1, &InputSubscriber::KeyboardCallBack, this);
  //joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy_1", 10, &InputSubscriber::joyCallBack, this);
  
  car_run_status = 0;


}

InputSubscriber::~InputSubscriber()
{
//  canthread->brake();
//  usleep(5000000);
  canthread->send2ECU(2,0,0);
  usleep(2000000);
  canthread->close_CAN();
}

//InputSubscriber::joyCallBack(const sensor_msgs::Joy &joymsg)
//{




//}




void InputSubscriber::CanThreadInitial(){
    //open can
    if( m_canisopen == false) {
      ROS_INFO("Start trying to open CAN");
      ROS_INFO("========================");
      if(canthread->initial_openCAN()) {
          int ret;
          usleep(1);
          m_run0 = 1;
          ret=pthread_create(&(canthread->can_rec_th),NULL,receive_func,&m_run0);
          if(ret==0) {
             m_canisopen = true;
          } else {
             m_canisopen = false;
          }
      }
      else{
           m_canisopen = false;
      }
    } else {
      //canthread->close_CAN();
      m_run0=0;//线程关闭指令。
      pthread_join(canthread->can_rec_th,NULL);

      usleep(100000);//延时100ms。
      VCI_ResetCAN(VCI_USBCAN2, 0, 0);//复位CAN1通道。
      usleep(100000);//延时100ms。
      VCI_ResetCAN(VCI_USBCAN2, 0, 1);//复位CAN2通道。
      usleep(100000);//延时100ms。
      VCI_CloseDevice(VCI_USBCAN2,0);//关闭设备。

      m_canisopen = false;
    }

  if(m_canisopen == true)
    ROS_INFO("Can open success");
}

bool InputSubscriber::GetStatus(){
  return get_cmd_;
}

void InputSubscriber::SetStatus(bool status){
  get_cmd_ = status;
}

double InputSubscriber::GetSteeringAngle(){
  return steering_angle;
}

double InputSubscriber::GetRearVelocity(){
  return rear_velocity;
}

void InputSubscriber::CmdCallback(const autoware_msgs::VehicleCmd::ConstPtr &input_msg)
{
  TwistStamped2Car(input_msg->twist_cmd);
}

void InputSubscriber::TwistStamped2Car(const geometry_msgs::TwistStamped &input_twist_msg)
{
  rear_velocity = input_twist_msg.twist.linear.x * 1.5; //max 
 // if(rear_velocity<0) rear_velocity = 0;
  if(rear_velocity > VEL_MAX) rear_velocity = VEL_MAX;

  steering_angle = steering_ratio_ * input_twist_msg.twist.angular.z * 180 / M_PI;
  steering_angle = abs(steering_angle) > ANGEL_MAX? (steering_angle>0? ANGEL_MAX:-ANGEL_MAX): steering_angle;


}


void lregular_angspd(float &spd)
{
  if(abs(spd)<50)
  {
    spd = 0;
    return;
  }
  int flag = spd/abs(spd);
  float val = abs(spd);
  if(val < 300)
  {
     val = val*val/300;
  }
  spd = flag*val;
  return;
}

float kp = 15;
bool prv_is_back = false;
int ii = 0;



int backcount_1 = 0;
int stopcount_1 = 0;
//int backcount = 0;
int state_1 = 0;//0: normal; 1: stop; 2 back;
int stopcycle = 20;

void InputSubscriber::TwistCallback(const geometry_msgs::Twist::ConstPtr &input_twist_msg)
{
  rear_velocity = input_twist_msg->linear.x; // wheel_radius_;
  rear_velocity = rear_velocity > VEL_MAX ? VEL_MAX : rear_velocity;
 // if(rear_velocity < 0) rear_velocity = 0;

  steering_angle = steering_ratio_ * input_twist_msg->angular.z * 180 / M_PI;
  steering_angle = abs(steering_angle) > ANGEL_MAX? (steering_angle>0? ANGEL_MAX:-ANGEL_MAX): steering_angle;

 // std::cout<<"_________________target spd cmd="<< rear_velocity <<"  target ang="<<input_twist_msg->angular.z * 180 / M_PI<<std::endl;
  //std::cout<<"streer = "<<steering_angle<<std::endl;
  float cmdang_cal = input_twist_msg->angular.z * 180 / M_PI;

  //float spd = steering_angle;
  //lregular_angspd(spd);
  //std::cout<<"new ang cmd====="<< spd <<std::endl;
  float realspd =  this->canthread->get_cur_spd();
  float realangle = -1 * this->canthread->get_cur_ang();
  //std::cout<<"real spd===="<< realspd<<"m/s   real ang====="<< realangle <<"du"<<std::endl;
  //feed back control
  float error = cmdang_cal - realangle;

  float ang_control = kp*error;
  float outcontol_ang_bangbang;

 // std::cout<<"current command===="<< outcontol_ang_bangbang <<"du"<<std::endl;
 // steering_angle = outcontol_ang_bangbang;
 if(rear_velocity<0)
  {
  //    std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!there is fall back!!!!!!!!!!!!!"<<std::endl;
      backcount_1++;
  }
  else
  {
      backcount_1 = 0;
  }

  if(backcount_1 >= 2)
  {
    state_1 = 1;
    //stopcount ++;
    car_run_status = 1;
   // std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!fall back confrimed!!!!!!!!!!!!!"<<std::endl;
  }

  if((state_1 ==1)&&(stopcount_1 <stopcycle))//stop for 20 cycle
  {
    stopcount_1 ++;
    //send stop order
   // currentorder.angular.z = 0;
   // currentorder.linear.x = 0;
   // p_mycmdvel_pub->publish(currentorder);
    rear_velocity = 0;
    steering_angle = 0;

  //  std::cout<<"!!!!!!!!!!!!!send stop command!!!!!!!!!"<<std::endl;
    car_run_status = 1;
    //send stop order
    return;
  }
  else if(stopcount_1>=stopcycle)
  {
  //  std::cout<<"!!!!!!!!!!!!!stop released, replan!!!!!!!!!"<<std::endl;
    state_1 = 0;
    car_run_status = 0;
    backcount_1 = 2;
    stopcount_1 = 0;
    return;
  }
 
}

void InputSubscriber::KeyboardCallBack(const std_msgs::String::ConstPtr& msg)
{
    //Speed Control
    if(msg->data == "w") rear_velocity += 0.1;
    if(msg->data == "s" ) rear_velocity -= 0.1;
    if(rear_velocity > 5) rear_velocity = 5 ;
    if(rear_velocity < 0) rear_velocity = 0 ;
    //Angle Control
    if(msg->data == "a") steering_angle += 0.8;
    if(msg->data == "d") steering_angle -= 0.8;
    if(steering_angle > ANGEL_MAX) steering_angle = ANGEL_MAX;
    if(steering_angle < -ANGEL_MAX) steering_angle = -ANGEL_MAX;

    if(msg->data == "q"){
      steering_angle = 0;
    //  rear_velocity = 0;
    }


    ROS_INFO("velocity info: [%f]", rear_velocity);
    ROS_INFO("steering angle: [%f]", steering_angle*20);
}
