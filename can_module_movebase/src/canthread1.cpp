#include "canthread1_move.h"

float gl_spd = 0;
float gl_tmp = 0;
float gl_ang = 0;
float gl_bat = 0;

QMutex gl_datalock1;


using namespace std;
double WheelCurrentV = 0;
double chelun_angle = 0;
float SysVoltage = 0;
int sysdirection = 1;
double chelun_angle_fangxiangpan = 0;

canthread1::canthread1()
{
    count = 0;
    num = 0;
    isPIDinitial = true;
}


void canthread1::sendinstruction(int param0, int param1,int param2,int XCU) // Send instruction to ECU if XCU =0 else send instruction to MCU.

{
    int i=0;
    VCI_CAN_OBJ send[1];
    if(XCU==0) send[0].ID=0x18F610D3;
    if(XCU==1) send[0].ID=0x18f600D3;    
    send[0].SendType=0;
    send[0].RemoteFlag=0;
    send[0].ExternFlag=1;
    send[0].DataLen=8;
    send[0].Data[0]=param0;
    send[0].Data[1]=param1;
    send[0].Data[2]=param2;
    send[0].Data[3]=0;
    send[0].Data[4]=0;
    send[0].Data[5]=0;
    send[0].Data[6]=0;
    send[0].Data[7]=0;
        if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)
        {

            // printf("Index:%04d  ",count);count++;
            // printf("CAN1 TX ID:0x%08X",send[0].ID);


            // if(send[0].ExternFlag==0) printf(" Standard ");
            // if(send[0].ExternFlag==1) printf(" Extend   ");
            // if(send[0].RemoteFlag==0) printf(" Data   ");
            // if(send[0].RemoteFlag==1) printf(" Remote ");
            // printf("DLC:0x%02X",send[0].DataLen);
            // printf(" data:0x");

            // for(i=0;i<send[0].DataLen;i++)
            // {
            //     printf(" %02X",send[0].Data[i]);
            // }
            // printf("\n");
            // printf("CAN status OK!");
        }

}

void canthread1::send_MCU_PID()
{
      int i=0;
  VCI_CAN_OBJ send[1];
  send[0].ID=0x18F600D5;
  send[0].SendType=0;
  send[0].RemoteFlag=0;
  send[0].ExternFlag=1;
  send[0].DataLen=8;

  int kp1=5000;
  int ki1=5;
  int kp2=3000;
  int ki2=3;

  send[0].Data[0]=0x88;
  send[0].Data[1]=0x13;

  send[0].Data[2]=0x05;
  send[0].Data[3]=0x00;

  send[0].Data[4]=0xD0;
  send[0].Data[5]=0x07;

  send[0].Data[6]=0x03;
  send[0].Data[7]=0x00;
      if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)
      {
        //   printf("Index:%04d  ",count);count++;
        //   printf("CAN1 TX ID:0x%08X",send[0].ID);

        //----------
        //   if(send[0].ExternFlag==0) printf(" Standard ");
        //   if(send[0].ExternFlag==1) printf(" Extend   ");
        //   if(send[0].RemoteFlag==0) printf(" Data   ");
        //   if(send[0].RemoteFlag==1) printf(" Remote ");
        //   printf("DLC:0x%02X",send[0].DataLen);
        //   printf(" data:0x");

        //   for(i=0;i<send[0].DataLen;i++)
        //   {
        //       printf(" %02X",send[0].Data[i]);
        //   }
        //   printf("\n");
        // printf("CAN status OK!");

      }




}



void canthread1::send2ECU(int ecufunc,int angle, int Direction)
{
    int angledata=0,angleinsl=0,angleinsh=0;
    if (ecufunc==0){
        sendinstruction(0b00001000, 0,0,0);
    }else if (ecufunc==1){
       if (Direction==0) angledata=180-angle/4; //left
       else if (Direction==1) angledata=180+angle/4; //right
       angleinsh=angledata/256;
       angleinsl=angledata%256;
       sendinstruction(0b00010000, angleinsl,angleinsh,0);
    }else if (ecufunc==2){
       sendinstruction(0b00100000, 0,0,0);
    }else
        printf(" ECU function code Error!!! /n");
}

void canthread1::send2Brake(int direction,int speed,int stopPosition,int mode)
{
    if(mode==0)
    {
        sendBrakeInst(0,0,0,mode);
    }else if(mode==1)
    {
        sendBrakeInst(direction,speed,0,mode);
    }
    else if(mode==2)
    {
        sendBrakeInst(0,speed,stopPosition,mode);
    }
}
    

void canthread1::sendBrakeInst(int direction,int speed,int stopPosition,int mode)
{
    int i=0;
    VCI_CAN_OBJ send[1];
    
    send[0].ID=0x18f700D3;    
    send[0].SendType=0;
    send[0].RemoteFlag=0;
    send[0].ExternFlag=1;
    send[0].DataLen=8;
    send[0].Data[0]=direction;
    send[0].Data[1]=speed;
    send[0].Data[2]=stopPosition;
    send[0].Data[3]=mode;
    send[0].Data[4]=0;
    send[0].Data[5]=0;
    send[0].Data[6]=0;
    send[0].Data[7]=0;
    if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)
    {

        // printf("Index:%04d  ",count);count++;
        // printf("CAN1 TX ID:0x%08X",send[0].ID);

        //------------------
        // if(send[0].ExternFlag==0) printf(" Standard ");
        // if(send[0].ExternFlag==1) printf(" Extend   ");
        // if(send[0].RemoteFlag==0) printf(" Data   ");
        // if(send[0].RemoteFlag==1) printf(" Remote ");
        // printf("DLC:0x%02X",send[0].DataLen);
        // printf(" data:0x");

        // for(i=0;i<send[0].DataLen;i++)
        // {
        //     printf(" %02X",send[0].Data[i]);
        // }
        // printf("\n");
// 
        // printf("CAN status OK!");

    }


}

void canthread1::send2MCU(int P1,int P2, int P3, int P4, int P5, int P6, float speed)//0-100
{
    int speedp;
    int sendins=0;
    if (P1==1) sendins=sendins + 0b00000001;
    if (P2==1) sendins=sendins + 0b00000010;
    if (P3==1) sendins=sendins + 0b00000100;
    if (P3==2) sendins=sendins + 0b00001000;
    if (P4==1) sendins=sendins + 0b00010000;
    if (P5==1) sendins=sendins + 0b00100000;
    if (P6==1) sendins=sendins + 0b01000000;
    speedp=speed;
    sendinstruction(sendins, speedp,0,1);
}

void canthread1::send2MCUtorque(int P1,int P2, int P3, int P4, int P5,int P6, float torque)
{
    int sendins=0;
    if (P1==1) sendins=sendins + 0b00000001;
    if (P2==1) sendins=sendins + 0b00000010;
    if (P3==1) sendins=sendins + 0b00000100;
    if (P3==2) sendins=sendins + 0b00001000;
    if (P4==1) sendins=sendins + 0b00010000;
    if (P5==1) sendins=sendins + 0b00100000;
    if (P6==1) sendins=sendins + 0b01000000;
    sendinstruction(sendins, torque,0,1);
}




void canthread1::goforward(int st,int speed)
{
 send2MCU(1,0,1,st,0,1,speed);
}

void canthread1::goforwardtorque(int torque)
{
 send2MCU(1,0,1,0,0,1,torque);
}


void canthread1::goback(int torque)
{
 send2MCU(1,0,2,0,0,1,torque);
}

void canthread1::stop()
{
 send2MCU(1,1,0,0,0,1,0);
}

void canthread1::brake()
{
 send2MCU(1,0,0,0,1,1,0);
}



void canthread1::controlfunc(int ecufunc,int mcufunc,float torque,int angle)
{
  int Direction;
  if (angle <0)
  {
    angle=-angle;
    Direction=0;
  }
  else
  {
    Direction=1;
  }
  send2ECU(ecufunc,angle,Direction);

  if (mcufunc==0)
  {
    send2MCUtorque(1,0,0,0,1,1,0);
  }
  else if (mcufunc==1)
  {
  send2MCUtorque(1,0,1,0,0,1,torque);
  }
  else if (mcufunc==2)
  {
  send2MCUtorque(1,0,2,0,0,1,torque);
  }
  else
  {
//   printf(" MCU function code Error!!! /n");
  send2MCU(1,0,0,0,1,1,0);
  }

  usleep(30000);
}


// double canthread1::PID_VController(double &Feedback_V,double Target_V)
// {
//   Vcontroller.Error = Target_V - Feedback_V;
//   Vcontroller.VCtrlIntergal += Vcontroller.Error;
//   Vcontroller.VCtrlSingal = Vcontroller.KVp * Vcontroller.Error
//                           + Vcontroller.KVi * Vcontroller.VCtrlIntergal
//                           + Vcontroller.KVd * (Vcontroller.Error - Vcontroller.LastError);
//   Vcontroller.LastError = Vcontroller.Error;
//   if(Vcontroller.VCtrlSingal > Vcontroller.UpLimit)
//     Vcontroller.VCtrlSingal = Vcontroller.UpLimit;
//   if(Vcontroller.VCtrlSingal < Vcontroller.LowLimit)
//     Vcontroller.VCtrlSingal = Vcontroller.LowLimit;
//   return Vcontroller.VCtrlSingal;
// }

double canthread1::pid_regulter(double &cur, PID *pid_obj)
{   
    pid_obj->error  =  pid_obj->tar - cur;
    pid_obj->r_integer += pid_obj->error;   
    pid_obj->r_output  = pid_obj->kp * pid_obj->error + pid_obj->ki * pid_obj->r_integer
                        + pid_obj->kd * (pid_obj->error - pid_obj->lerror);
    if(pid_obj->r_output > pid_obj->up_limit)
        pid_obj->r_output = pid_obj->up_limit;
    else if(pid_obj->r_output < pid_obj->low_limit)
        pid_obj->r_output = pid_obj->low_limit;
    return pid_obj->r_output;
}




bool canthread1::initial_openCAN()
{
    printf(">>Hello world\r\n");//??

    num=VCI_FindUsbDevice2(pInfo1);

    printf(">>USBCAN DEVICE NUM:");printf("%d", num);printf(" PCS");printf("\n");

        for(int i=0;i<num;i++)
        {
        printf("Device:");printf("%d", i);printf("\n");
                printf(">>Get VCI_ReadBoardInfo success!\n");

        printf(">>Serial_Num:%c", pInfo1[i].str_Serial_Num[0]);
        printf("%c", pInfo1[i].str_Serial_Num[1]);
        printf("%c", pInfo1[i].str_Serial_Num[2]);
        printf("%c", pInfo1[i].str_Serial_Num[3]);
        printf("%c", pInfo1[i].str_Serial_Num[4]);
        printf("%c", pInfo1[i].str_Serial_Num[5]);
        printf("%c", pInfo1[i].str_Serial_Num[6]);
        printf("%c", pInfo1[i].str_Serial_Num[7]);
        printf("%c", pInfo1[i].str_Serial_Num[8]);
        printf("%c", pInfo1[i].str_Serial_Num[9]);
        printf("%c", pInfo1[i].str_Serial_Num[10]);
        printf("%c", pInfo1[i].str_Serial_Num[11]);
        printf("%c", pInfo1[i].str_Serial_Num[12]);
        printf("%c", pInfo1[i].str_Serial_Num[13]);
        printf("%c", pInfo1[i].str_Serial_Num[14]);
        printf("%c", pInfo1[i].str_Serial_Num[15]);
        printf("%c", pInfo1[i].str_Serial_Num[16]);
        printf("%c", pInfo1[i].str_Serial_Num[17]);
        printf("%c", pInfo1[i].str_Serial_Num[18]);
        printf("%c", pInfo1[i].str_Serial_Num[19]);printf("\n");

        printf(">>hw_Type:%c", pInfo1[i].str_hw_Type[0]);
        printf("%c", pInfo1[i].str_hw_Type[1]);
        printf("%c", pInfo1[i].str_hw_Type[2]);
        printf("%c", pInfo1[i].str_hw_Type[3]);
        printf("%c", pInfo1[i].str_hw_Type[4]);
        printf("%c", pInfo1[i].str_hw_Type[5]);
        printf("%c", pInfo1[i].str_hw_Type[6]);
        printf("%c", pInfo1[i].str_hw_Type[7]);
        printf("%c", pInfo1[i].str_hw_Type[8]);
        printf("%c", pInfo1[i].str_hw_Type[9]);printf("\n");

        printf(">>Firmware Version:V");
        printf("%x", (pInfo1[i].fw_Version&0xF00)>>8);
        printf(".");
        printf("%x", (pInfo1[i].fw_Version&0xF0)>>4);
        printf("%x", pInfo1[i].fw_Version&0xF);
        printf("\n");
    }
    printf(">>\n");
    printf(">>\n");
    printf(">>\n");
    if(VCI_OpenDevice(VCI_USBCAN2,0,0)==1)
    {
        printf(">>open deivce success!\n");
    }else
    {
        printf(">>open deivce error!\n");
        //exit(1);
        return false;
    }
    if(VCI_ReadBoardInfo(VCI_USBCAN2,0,&pInfo)==1)
    {
                printf(">>Get VCI_ReadBoardInfo success!\n");


        printf(">>Serial_Num:%c", pInfo.str_Serial_Num[0]);
        printf("%c", pInfo.str_Serial_Num[1]);
        printf("%c", pInfo.str_Serial_Num[2]);
        printf("%c", pInfo.str_Serial_Num[3]);
        printf("%c", pInfo.str_Serial_Num[4]);
        printf("%c", pInfo.str_Serial_Num[5]);
        printf("%c", pInfo.str_Serial_Num[6]);
        printf("%c", pInfo.str_Serial_Num[7]);
        printf("%c", pInfo.str_Serial_Num[8]);
        printf("%c", pInfo.str_Serial_Num[9]);
        printf("%c", pInfo.str_Serial_Num[10]);
        printf("%c", pInfo.str_Serial_Num[11]);
        printf("%c", pInfo.str_Serial_Num[12]);
        printf("%c", pInfo.str_Serial_Num[13]);
        printf("%c", pInfo.str_Serial_Num[14]);
        printf("%c", pInfo.str_Serial_Num[15]);
        printf("%c", pInfo.str_Serial_Num[16]);
        printf("%c", pInfo.str_Serial_Num[17]);
        printf("%c", pInfo.str_Serial_Num[18]);
        printf("%c", pInfo.str_Serial_Num[19]);printf("\n");

        printf(">>hw_Type:%c", pInfo.str_hw_Type[0]);
        printf("%c", pInfo.str_hw_Type[1]);
        printf("%c", pInfo.str_hw_Type[2]);
        printf("%c", pInfo.str_hw_Type[3]);
        printf("%c", pInfo.str_hw_Type[4]);
        printf("%c", pInfo.str_hw_Type[5]);
        printf("%c", pInfo.str_hw_Type[6]);
        printf("%c", pInfo.str_hw_Type[7]);
        printf("%c", pInfo.str_hw_Type[8]);
        printf("%c", pInfo.str_hw_Type[9]);printf("\n");

        printf(">>Firmware Version:V");
        printf("%x", (pInfo.fw_Version&0xF00)>>8);
        printf(".");
        printf("%x", (pInfo.fw_Version&0xF0)>>4);
        printf("%x", pInfo.fw_Version&0xF);
        printf("\n");
    }else
    {
        printf(">>Get VCI_ReadBoardInfo error!\n");
       // exit(1);
        return false;
    }


    VCI_INIT_CONFIG config;
    config.AccCode=0;
    config.AccMask=0xFFFFFFFF;
    config.Filter=1;//        ?
    config.Timing0=0x01;/*      125 Kbps  0x03  0x1C*/
    config.Timing1=0x1C;
    config.Mode=0;//    ??

    if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1)
    {
        printf(">>Init CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN2,0);
        return false;
    }

    if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
    {
        printf(">>Start CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN2,0);
        return false;
    }

    if(VCI_InitCAN(VCI_USBCAN2,0,1,&config)!=1)
    {
        printf(">>Init can2 error\n");
        VCI_CloseDevice(VCI_USBCAN2,0);
        return false;

    }
    if(VCI_StartCAN(VCI_USBCAN2,0,1)!=1)
    {
        printf(">>Start can2 error\n");
        VCI_CloseDevice(VCI_USBCAN2,0);
        return false;
    }
    return true;
}


void *receive_func(void* param)
{
    int reclen=0;
    VCI_CAN_OBJ rec[3000];
    int i,j;

    int *run=(int*)param;
    int ind=0;
    int count = 0;
    int num =0;

    int sysmode,sysrpml,sysrpmh,sysfault,sysspeedl,sysspeedh;
    int sysvoltagel,sysvoltageh,sysmotortempl,sysmotortemph,syscontroltempl,syscontroltemph;
    int sysp1,sysp2,sysp3,sysp4,sysp5,sysp6;

    int sc1,sc2,sc3,sc4,sc5,sc6,sc7,sc8;

    while(run)
    {
        if((reclen=VCI_Receive(VCI_USBCAN2,0,ind,rec,3000,100))>0)
        {
            for(j=0;j<reclen;j++)
            {
#ifdef DEBUG
                printf("Index:%04d  ",count);count++;//   ?
                printf("CAN%d RX ID:0x%08X", ind+1, rec[j].ID);//ID
                if(rec[j].ExternFlag==0) printf(" Standard ");
                if(rec[j].ExternFlag==1) printf(" Extend   ");
                if(rec[j].RemoteFlag==0) printf(" Data   ");
                if(rec[j].RemoteFlag==1) printf(" Remote ");
                printf("DLC:0x%02X",rec[j].DataLen);//?
                printf(" data:0x");	//
                for(i = 0; i < rec[j].DataLen; i++)
                {
                    printf(" %02X", rec[j].Data[i]);
                }
                printf(" TimeStamp:0x%08X",rec[j].TimeStamp);//?    ?
                printf("\n");
#endif


                if(rec[j].ID==0x10F8109A)
                {
                    sysmode=rec[j].Data[0];
                    sysrpml=rec[j].Data[1];
                    sysrpmh=rec[j].Data[2];
                    sysfault=rec[j].Data[3];
                    sysspeedl=rec[j].Data[4];
                    sysspeedh=rec[j].Data[5];
                }

                if(rec[j].ID==0x10F8108D)
                {
                    sysvoltagel=rec[j].Data[0];
                    sysvoltageh=rec[j].Data[1];
                    sysmotortempl=rec[j].Data[4];
                    sysmotortemph=rec[j].Data[5];
                    syscontroltempl=rec[j].Data[6];
                    syscontroltemph=rec[j].Data[7];
                }

                if(rec[j].ID==0x18F30121)
                {
                    sysp1=rec[j].Data[0];
                    sysp2=rec[j].Data[1];
                    sysp3=rec[j].Data[2];
                    sysp4=rec[j].Data[3];
                    sysp5=rec[j].Data[4];
                    sysp6=rec[j].Data[5];
                }

                if(rec[j].ID==0x18FFFFF)
                {
                    sc1=rec[j].Data[0];
                    sc2=rec[j].Data[1];
                    sc3=rec[j].Data[2];
                    sc4=rec[j].Data[3];
                    sc5=rec[j].Data[4];
                    sc6=rec[j].Data[5];
                    sc7=rec[j].Data[6];
                    sc8=rec[j].Data[7];
                }


/************************************************************************************************************/
                float sysspeed, sysvolatge;

                float weiyi,chelunzhuanjiao_weiyi,chelun_bianmaqi,tuigandianji;
                
                float sysmotortemp, syscontroltemp;
                int sysrpm, sysparameter1, sysparameter2, sysparameter3;
               // int sysp1,sysp2,sysp3,sysp4,sysp5,sysp6;

                sysdirection = sysmode;
               
                sysrpm=sysrpmh*256+sysrpml;
                sysspeed= (sysspeedh*256+sysspeedl)*0.1;
                sysmotortemp=(sysmotortemph*256+sysmotortempl)*0.1;
                sysvolatge=(sysvoltageh*256+sysvoltagel)*0.1;
                syscontroltemp=(syscontroltemph*256+syscontroltempl)*0.1;
                sysparameter1=sysp1+sysp2*256;
                sysparameter2=sysp3+sysp4*256;
                sysparameter3=sysp5+sysp6*256;
                WheelCurrentV = sysspeed; // houlun sudu

                SysVoltage = sysvolatge;

                weiyi = sc1+sc2*256;
                chelunzhuanjiao_weiyi = -40+0.01*(sc3+sc4*256);
                chelun_bianmaqi = -40+0.01*(sc5+sc6*256);
                tuigandianji = sc7+sc8*256;//0-1000

                chelun_angle = chelunzhuanjiao_weiyi; //qianlunzhuanjiao 
                chelun_angle_fangxiangpan = chelun_bianmaqi;
              //  std::cout<<"fangxiangpan bianmaq"<<chelun_bianmaqi<<std::endl;
              // std::cout<<"chelun_angle "<<chelunzhuanjiao_weiyi<<std::endl;
/************************************************************************************************************/
}

        }
    }
    printf("run thread exit\n");
    pthread_exit(0);
}



void canthread1::close_CAN()
{
     VCI_CloseDevice(VCI_USBCAN2,0);
}

double canthread1::get_cur_spd()
{
   return WheelCurrentV*1.0/3.6;  //m/s
}

double canthread1::get_cur_ang()
{
   return chelun_angle; //dushu
}


double canthread1::get_cur_ang_fangxiangpan()
{
   return chelun_angle_fangxiangpan; //dushu
}

int canthread1::get_cur_direction()
{
    return sysdirection;
}

float canthread1::get_voltage()
{
    return SysVoltage;
}