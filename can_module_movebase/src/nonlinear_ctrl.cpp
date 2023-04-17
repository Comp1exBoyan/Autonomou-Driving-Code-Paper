const float k1 = -2.5;
const float k2 = -3;
const float k3 = -2.5;

double *backstepping_ctrl(car_pid_state &veh_state, car_plan_info &cur_plan, car_plan_info &pre_plan){
    double e[3] = {0,0,0}, item[3] = {0,0,0};

    double u[2] = {0,0};
    // e[0] = veh_state.x - cur_plan.plan_x;
    // e[1] = veh_state.y - cur_plan.plan_y; 
    // e[2] = veh_state.yaw - cur_plan.plan_yaw; 

    e[0] = 0- cur_plan.plan_x;
    e[1] = 0 - cur_plan.plan_y; 
    e[2] = 0- cur_plan.plan_yaw; 
    // std::cout << "veh_state.yaw" << veh_state.yaw << "cur_plan.plan_yaw" << cur_plan.plan_yaw <<std::endl;  
    // std::cout << "veh_state.x  = " << veh_state.x << "plan_x " << cur_plan.plan_x << std::endl; 
    // std::cout << "e0  = " << e[0] << "e1 " << e[1] << "e2 " << e[2] << std::endl; 
    item[0] = k1 * e[0] + (cur_plan.plan_x - pre_plan.plan_x);
    item[1] = k2 * e[1] + (cur_plan.plan_y - pre_plan.plan_y);
    item[2] = k3 * e[2] + (cur_plan.plan_yaw - pre_plan.plan_yaw);


//   std::cout << "cur_plan.plan_x  = " << cur_plan.plan_x  << "pre_plan.plan_x= " << pre_plan.plan_x << std::endl; 
    
    u[0] = sqrt(pow(item[0],2) + pow(item[1], 2)) ;
    u[1] = atan(item[2] * CAR_LEN /  u[0]);

    // std::cout << "v = " << u[0] << " arctan u1  = " << u[1] * 180 / 3.141 << std::endl; 
    if(u[0] > 5) u[0] = 5;
    else if(u[1] < -5) u[0] = -5; 
    if(u[1] > 0.785) u[1] = 0.785;
    else if(u[1] < -0.785) u[1] = -0.785;
    u[1] = u[1] * 180 / 3.141; 
    u[0] /= 5;
    // std::cout << "v = " << u[0] << " delta = " << u[1]  << std::endl; 
    double  *ptr = &u[0]; 
    return ptr ;
 

}


float average_filter(float *data, float angle)
{
   float sum = 0 ;
   for(int i = 0; i < 6 - 1; i++)
   {
     data[i] = data[i+1];
   } 
   data[5] = angle;
   
   for(int i = 0; i < 6; i++)
   {
       sum +=data[i];
   }

   return sum / 6; 

}

