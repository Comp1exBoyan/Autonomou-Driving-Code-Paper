#include "pure_pursuit.h"

void pure_pursuit::state_update(Car_State &vehicleState,car_lc_info &car_state)
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

int pure_pursuit::get_goal_index( Car_State &s,  vector <geometry_msgs::Point> &path) {
    vector<double> d;
    for (int i = 0; i < path.size(); i++)
    {
        d.push_back(pow((s.x - path[i].x), 2) + pow((s.y - path[i].y), 2));//距离计算

    }
    int index = 0;
    double d_min = d[0];
    int dVecLen = d.size();

    //找到距离车辆最近的路径点
    for (int i = 0; i < dVecLen; i++) {
        if (d_min > d[i]) {
            d_min = d[i];
            index = i;
        }
    }
    ROS_INFO("lc x : %f, lc y :%f ,plan x: %f y: %f", s.x, s.y, path[index].x , path[index].y);
    double l = 0;
    double lf = lfc * s.speed + L0;
    double dx_, dy_;

    //积分法计算路径长度
    while (l < lf && index < path.size()) {
        dx_ = path[index + 1].x - path[index].x;
        dy_ = path[index + 1].y - path[index].y;
        l += sqrt(dx_ * dx_ + dy_ * dy_);
        index++;
    }

    return index;
}
//根据目标进行转角控制
double pure_pursuit::pure_pursuit_tracker( Car_State &s,  vector<geometry_msgs::Point> &path, int &lastIndex) {
    int index = get_goal_index(s, path); // 搜索目标点，返回目标点的标签

    // 用上一个循环的目标点判断是否是在向前走
    if (index <= lastIndex) {
        index = lastIndex;
    }

    geometry_msgs::Point goal;

    //防止index溢出
    if (index < path.size()) {
        goal = path[index]; 
    } else {
        index = path.size() - 1;
        goal = path[index];
    }

    // 车身坐标系的x轴和目标点与车身坐标系原点连线的夹角
    double alpha = atan2(goal.y - s.y, goal.x - s.x) - s.yaw;

    if (s.speed < 0)
        alpha = M_PI - alpha;

    double lf = LFC_K * s.speed + L0; // 根据车速和道路曲率设置前视距离
    // delta 即为纯跟踪算法的最终输出
    double delta = atan2((2.0 * L_VEHICLE * sin(alpha)) / lf, 1.0);

    lastIndex = index;      // 为下一个循环更新上一个目标点
    return delta;
}

double pure_pursuit::steering_control(double tar_angle, double cur_angle, const int Kp)
{
    double u = Kp * (tar_angle - cur_angle);
    return  u;

}