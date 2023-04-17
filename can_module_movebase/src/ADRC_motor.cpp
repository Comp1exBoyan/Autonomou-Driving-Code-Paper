/*---------------------------------自抗扰控制算法C语言包---------------------------
版本号：1.00 2018年7月12日 总决赛版本
作者：唐义丰
功能：底盘电机和摩擦轮电机的控制算法，在抑制超调和抵抗扰动方面性能全面优于pid算法，
      如果时间允许云台可以考虑串级ADRC
包含文件：ADRC_motor.c,ADRC_motor.h
主要调用函数：ADRC_Param 对应下面结构体的命名=NULL;
							ADRC_Init(ADRC_Param *obj,ADRC_Param *abc)，在里面同时对两个结构体初始化，可以改为一个
							ADRC_controller(ADRC_Param *obj, float desired_value, float feedback_value)，对之前初始化好的结构体进行计算，前一个值为目标值，后一个值为当前值
---------------------------------------------------------------------------------*/
#include <stdio.h>
#include "ADRC_motor.h"
const float ADRC_Unit[3][15]=
{
  /*TD跟踪微分器   改进最速TD,h0=N*h      扩张状态观测器ESO           扰动补偿     非线性组合*/
  /*  r     h      N              beta_01   beta_02    beta_03     b0     beta_0  beta_1     beta_2     N1     C    alpha1  alpha2  zeta       b*/
 {300000 ,0.005 , 2,               100,      1000,      2000,     0.001,    0.002,   1.0,      0.0005,    5,    5,    0.8,   1.5,    50},
 {300000 ,0.005 , 2,               100,      1000,      2000,     0.001,    0.002,   1.0,      0.0005,    5,    5,    0.8,   1.5,    50},
 {300000 ,0.005 , 3,               300,      4000,      10000,     100,   0.2,    1.0,      0.0010,       5,      5,    0.8,   1.5,    0.03}
 //{1000000 ,0.005 , 5,               300,      10000,      100000,      0.5,    0.85,   1.5,      0.0003,       2,    5,    0.9,   1.2,    0.03}
};

//ADRC_Param *adrc_param;
/*************************************
 * 
 函数名：Constrain_Float
 参数：限幅函数
 功能描述：限定输出
 * 
*************************************/
float Constrain_Float(float amt, float low, float high){
  return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}
/*************************************
 * 
 函数名：ADRC_Param_Init
 参数：*obj-ADRC结构体中
 功能描述：结构体中的参数部分初始化
 * 
*************************************/
void ADRC_Init(ADRC_Param *obj)//,ADRC_Param *abc)
{
 obj->r=ADRC_Unit[0][0];
 obj->h=ADRC_Unit[0][1];
 obj->N=(uint16_t)(ADRC_Unit[0][2]);
 obj->beta_01=ADRC_Unit[0][3];
 obj->beta_02=ADRC_Unit[0][4];
 obj->beta_03=ADRC_Unit[0][5];
 obj->beta_0=ADRC_Unit[0][7];
 obj->beta_1=ADRC_Unit[0][8];
 obj->beta_2=ADRC_Unit[0][9];

 obj->b0=ADRC_Unit[0][6];
 obj->N1=(uint16_t)(ADRC_Unit[0][10]);

 obj->alpha1=ADRC_Unit[0][12];
 obj->alpha2=ADRC_Unit[0][13];
 obj->lzeta=ADRC_Unit[0][14];
	//   obj->r=ADRC_Unit[2][0];
  // obj->h=ADRC_Unit[2][1];
  // obj->N=(uint16_t)(ADRC_Unit[2][2]);
  // obj->beta_01=ADRC_Unit[2][3];
  // obj->beta_02=ADRC_Unit[2][4];
  // obj->beta_03=ADRC_Unit[2][5];
  // obj->beta_0=ADRC_Unit[2][7];
  // obj->beta_1=ADRC_Unit[2][8];
  // obj->beta_2=ADRC_Unit[2][9];

  // obj->b0=ADRC_Unit[2][6];
  // obj->N1=(uint16_t)(ADRC_Unit[2][10]);
 
  // obj->alpha1=ADRC_Unit[2][12];
  // obj->alpha2=ADRC_Unit[2][13];
  // obj->lzeta=ADRC_Unit[2][14];
	
	//  abc->r=ADRC_Unit[2][0];
  // abc->h=ADRC_Unit[2][1];
  // abc->N=(uint16_t)(ADRC_Unit[2][2]);
  // abc->beta_01=ADRC_Unit[2][3];
  // abc->beta_02=ADRC_Unit[2][4];
  // abc->beta_03=ADRC_Unit[2][5];
  // abc->beta_0=ADRC_Unit[2][7];
  // abc->beta_1=ADRC_Unit[2][8];
  // abc->beta_2=ADRC_Unit[2][9];

  // abc->b0=ADRC_Unit[2][6];
  // abc->N1=(uint16_t)(ADRC_Unit[2][10]);
 
  // abc->alpha1=ADRC_Unit[2][12];
  // abc->alpha2=ADRC_Unit[2][13];
  // abc->lzeta=ADRC_Unit[2][14];
  
}

/*************************************
 * 
 函数名：Sign
 参数：input——输入的数据，output——分段后的参数
 功能描述：数学中的sgn函数
 * 
*************************************/
int16_t Sign(float input)
{
    int16_t output = 0; 
    if(input <= 1e-6) output = -1;
    else if(input > 1e-6) output = 1;
    else output = 0 ;
    return output;

}
/*************************************
 * 
 函数名：Fsg_func
 参数：param——待处理函数，param_d——分界值
 功能描述：作为改进fhan函数的一部分
 * 
**************************************/
int16_t Fsg_func(float param,float param_d)
{
   int16_t output = 0 ; 
   output =(Sign(param + param_d) - 
            Sign(param - param_d))/2 ;
   return output;

}
/*************************************
 * 
 函数名：TD部分->Fhan_Calc函数
 参数：  *obj——ADRC结构体指针，直接做运算
         desired_value——被控对象期望值
 功能描述：作为改进fhan函数的一部分
 * 
**************************************/
void Fhan_Calc(ADRC_Param *obj,float desired_value)
{
    float a0 = 0, y = 0, a1 = 0, a2 = 0, a;    
    float x1_delta=0;//ADRC状态跟踪误差项
    float d_temp = 0; // 定义为 rh^2
    x1_delta = obj->x1 - desired_value;
    obj->h0=obj->N  * obj->h;//用h0替代h，解决最速跟踪微分器速度超调问题
    d_temp = obj->r * obj->h0 * obj->h0; //d = rh^2
    a0 = obj->h0 * obj->x2 ; 
    y = x1_delta + a0;
    a1=sqrt(d_temp*(d_temp+8*ABS(y)));//a1=sqrt(d*(d+8*ABS(y))])
    a2=a0+Sign(y)*(a1-d_temp)/2;//a2=a0+sign(y)*(a1-d)/2;
    a = (a0+y)*Fsg_func(y,d_temp) + a2*(1 - Fsg_func(y,d_temp));
    obj->fhan = -obj->r*(a/d_temp)*Fsg_func(a,d_temp)
                 - obj->r*Sign(a)*(1-Fsg_func(a,d_temp));
    obj->x1+=obj->h*obj->x2;//跟新最速跟踪状态量x1
    obj->x2+=obj->h*obj->fhan;//跟新最速跟踪状态量微分x2
}
/*************************************
 * 
 函数名：ESO部分->Fal函数
 参数：  e——误差, alpha——非线性因子， zeta——线性区段长度
 原点附近有连线性段的连续幂次函数,用在ESO中
 * 
**************************************/


float Fal(float e,float alpha,float zeta)
{
    int16_t s=0;
    float fal_output=0;
    s=(Sign(e+zeta)-Sign(e-zeta))/2;
    fal_output=e*s/(powf(zeta,1-alpha))+powf(ABS(e),alpha)*Sign(e)*(1-s);
    return fal_output;
}
/*************************************
 * 
 函数名：ESO部分->ESO_Calc函数
 参数：  *obj——ADRC结构体指针，直接做运算
 功能描述：扩张扰动观测器，根据系统反馈，观测扰动，传给TD做运算
 * 
**************************************/
void ESO_Calc(ADRC_Param *obj)
{
  obj->e=obj->z1-obj->y;//状态误差
 
  obj->fe=Fal(obj->e,0.5,obj->h);//非线性函数，提取跟踪状态与当前状态误差
  obj->fe1=Fal(obj->e,0.25,obj->h);
 
  /*************扩展状态量更新**********/
  obj->z1+=obj->h*(obj->z2-obj->beta_01*obj->e);
  obj->z2+=obj->h*(obj->z3
                    -obj->beta_02*obj->fe
                    +obj->b0*obj->u);
 //ESO估计状态加速度信号，进行扰动补偿，传统MEMS陀螺仪漂移较大，估计会产生漂移
  obj->z3+=obj->h*(-obj->beta_03*obj->fe1);
}
/*************************************
 * 
 函数名：NLSEF部分-> Nonlinear_fusion
 参数：  *obj——ADRC结构体指针，直接做运算
 功能描述：NLSEF,将所有的信号量做非线性融合，进行控制，可以简单理解为PID中的+
 * 
**************************************/


int mmm;
float data[6],sum;
float smooth_filter(float x)
{
	
	mmm+=1;
	if(mmm>+4)
	{
//		data[5]=data[4];
//		data[4]=data[3];
//		data[3]=data[2];
//		data[2]=data[1];
//		data[1]=data[0];
		data[0]=x;
		sum=(data[0]+data[1]+data[2]+data[3]+data[4]+data[5])/4;
	}
	return sum;	
}



void Nonlinear_fusion(ADRC_Param *obj)
{
  float temp_e2=0;
  temp_e2=Constrain_Float(obj->e2,-3000,3000);
  obj->u0=(obj->beta_1*300*Fal(obj->e1,obj->alpha1,obj->lzeta)
                +200*obj->beta_2*Fal(temp_e2,obj->alpha2,obj->lzeta));
	obj->u0=smooth_filter(obj->u0);
}
/*************************************
 * 
 函数名：Controller部分->ADRC_controller函数
 参数：  *obj——ADRC结构体指针，直接做运算,desired_value期望值，feedback_value传感器反馈值
 功能描述：控制器，接口和pid一样，直接传参
 * 
**************************************/
float ADRC_controller(ADRC_Param *obj, float desired_value, float feedback_value)
{
    float u = 0 ;
    /*
    * adrc控制器第一步：计算TD  
    *
    */
   Fhan_Calc(obj,desired_value);

   /*
    * adrc控制器第二步：计算ESO
    * 刚开始反馈值就是传感器回来的值
    */
   
   obj->y = feedback_value;
   ESO_Calc(obj);
  
   /*
    * adrc控制器第三步：计算控制器输出
    * 把所有误差信号融合起来
    */
   obj->e0+=obj->e1*obj->h;//状态积分项
      obj->e1=obj->x1-obj->z1;//状态偏差项
      obj->e2=obj->x2-obj->z2;//状态微分项，

   //融合所有信号量
   Nonlinear_fusion(obj);
   u = Constrain_Float(obj->u0,-350,350);
   return u;
}

//int main()
//{
//    ADRC_Param *adrc_test_obj = NULL;
//    adrc_test_obj = (ADRC_Param *)malloc(sizeof(ADRC_Param));
//    ADRC_Init(adrc_test_obj);
//    float current = 0;
//    float u =0;
//    while(1)
//    {     
//      u =  ADRC_controller(adrc_test_obj,10000,current);
//      current += u ;
//      printf("current: %f\n",current);
//    }
//    printf("hello world");

//} 
