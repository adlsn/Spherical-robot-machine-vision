/**
  ******************************************************************************
  * @file           : main.c
  * @author         : huangwankuan
  * @note
      @1.1--stm32f4xx_hal_conf.h（文件中改成对应MCU的外部晶振实际值）
  **/
#include "main.h"
#include "math.h"
#include "sys.h"
#define Uart_size 18

int16_t WANT_SPD[8];              //六个电机期望速度的保存数组
int16_t SET_Current[8];           //六个电机设置电流
int16_t M_Current[8];              //六个电机期望电流

float MOTOR_Rate=0.6;//整体速度水平    2018/12/9 17:52
float KP=4.0;  //0-2   P越大反应迟钝
float KI=2.0;  //0.1-0.8  I越大反应较大  容易产生震荡
float KD=0;

int16_t Current_MAX=10000;//16384-20A
int16_t SSS;
int16_t xie;
int16_t wants;
float pidss;

u8 SEND_DATA[8]={0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};

extern u8 RX_BUF[Uart_size];
u8 RX_Handle_Buf[Uart_size];
extern u8 UART_STA;

extern u8 CAN_Rmsg[8];
u16 t;
u16 flag1;
u8 flag2;
u16 pos;
u8 CAN_Send_Buff[8]={0};


int16_t RST_Left_X;
int16_t RST_Left_Y;
int16_t RST_Right_X;
int16_t RST_Right_Y;



extern CanRxMsgTypeDef RxMessage;      //接收消息
extern CAN_HandleTypeDef   CAN1_Handler;   //CAN1句柄
extern DJI_REMOTE DJI_RE;
int16_t weizhinow[6];
int16_t weizhilast[6];
int32_t quanshu[6];
int16_t offset[6];

u8 ta[3];
u8 CAN_SEND_DATA[8];
void Romote_To_Currnt_Handle(DJI_REMOTE Remote,int16_t* M_Current,u8 gear,float Factor_Roll);
void Romote_To_Currnt_Handle1(int Remote,int16_t* M_Current,u8 gear,float Factor_Roll);  
void Romote_To_Currnt_Handle2(int Remote,int16_t* M_Current,u8 gear,float Factor_Roll);  
void DBUS_RST(DJI_REMOTE Remote);


void limit_abs(int16_t *a, int16_t ABS_MAX)//限幅
{
    if(*a > ABS_MAX)
        *a = ABS_MAX;
    if(*a < -ABS_MAX)
        *a = -ABS_MAX;
}


int main(void)
{
	  u16 i;
		
    HAL_Init();                     //初始化HAL库
    Stm32_Clock_Init(360,12,2,8); 		//设置时钟,168Mhz  晶振12M
    delay_init(180);                //初始化延时函数

    for(int i=0; i<8; i++)
	{
		PID_Init(&PID_SPD[i],20000,10000,KP,KI,KD	);  //8 motos angular rate closeloop.
	}
    UART_Init();
	   LED_Init();
     KEY_Init();
     ESC_POWER();
     DBUS_Init();
	   TIM1_PWM_Init(143,9999,1000);
	


     CAN1_Mode_Init(CAN_MODE_NORMAL);
     MOTOR_Init();
     delay_ms(1000);
     //DBUS_RST(DJI_RE);
     LED_G=LED_OFF;
     LED_R=LED_OFF;
	   SHACHE_L=SHACHE_OFF;
     SHACHE_R=SHACHE_OFF;
t=0;
flag1=0;
flag2=0;
xie=0;
TIM_SetCompare1(TIM2,0);
TIM_SetCompare2(TIM2,0);
TIM_SetCompare3(TIM2,0);
TIM_SetCompare4(TIM2,0);
M_Current[0]=0;
M_Current[1]=0;
M_Current[2]=0;
M_Current[3]=0;
M_Current[4]=0;
M_Current[5]=0;
M_Current[6]=0;
M_Current[7]=0;

//拧螺栓
//	while(1)
//	{
//		if(DJI_RE.SW1 != 1)
//		{
//			M_Current[0] = 10*DJI_RE.Right_Y;
//			M_Current[1] = -10*DJI_RE.Left_Y;
//			
//			if(DJI_RE.Left_X < -100 && DJI_RE.Right_X > 100)
//			{
//				TIM_SetCompare1(TIM2,9999);
//			  TIM_SetCompare2(TIM2,0);
//				TIM_SetCompare3(TIM2,9999);
//			  TIM_SetCompare4(TIM2,0);
//			}
//			else if(DJI_RE.Left_X > 100 && DJI_RE.Right_X < -100)
//			{
//				TIM_SetCompare1(TIM2,0);
//			  TIM_SetCompare2(TIM2,9999);
//				TIM_SetCompare3(TIM2,0);
//			  TIM_SetCompare4(TIM2,9999);
//			}
//			else
//			{
//				TIM_SetCompare1(TIM2,0);
//			  TIM_SetCompare2(TIM2,0);
//				TIM_SetCompare3(TIM2,0);
//			  TIM_SetCompare4(TIM2,0);
//			}
//		}
//		else
//		{
//			M_Current[0] = 0;
//			M_Current[1] = 0;
//			TIM_SetCompare1(TIM2,0);
//			TIM_SetCompare2(TIM2,0);
//			TIM_SetCompare3(TIM2,0);
//			TIM_SetCompare4(TIM2,0);
//		}

//		for(int i=0; i<2; i++)
//		{
//			pid_calc(&PID_SPD[i],MOTOR_Meas[i].speed_rpm[NOW],M_Current[i]);
////      SET_Current[i]=WANT_SPD[i]*MOTOR_Rate+PID_SPD[i].delta_out;
//			SET_Current[i]=PID_SPD[i].delta_out;
//			limit_abs(&SET_Current[i],Current_MAX);
//		}
//		MOTOR_SET_Current(SET_Current[0],SET_Current[1],SET_Current[2],SET_Current[3],SET_Current[4],SET_Current[5],SET_Current[6],SET_Current[7]);
//		delay_ms(5);
//	}
	while(1)
	{
		if(DJI_RE.SW1 != 1)
		{
			if(DJI_RE.SW1 == 3)  //飞轮低速CL
			{
				M_Current[0] = 1500;
			}
			else if (DJI_RE.SW1 == 2)  //飞轮高速HL
			{
				M_Current[0] = 3000;
			}
			
			
			M_Current[1] = 10*DJI_RE.Left_Y;
		}
		else
		{
			TIM_SetCompare1(TIM2,0);
			TIM_SetCompare2(TIM2,0);
			M_Current[0] = 0;
			M_Current[1] = 0;
		}
		
		
		if(DJI_RE.SW2 == 1)  //TIM2控制刹车
		{
			TIM_SetCompare1(TIM2,9999);
			TIM_SetCompare2(TIM2,0);
		}
		else if(DJI_RE.SW2 == 2)
		{
			TIM_SetCompare1(TIM2,0);
			TIM_SetCompare2(TIM2,9999);
		}
		else
		{
			TIM_SetCompare1(TIM2,0);
			TIM_SetCompare2(TIM2,0);
		}
		
		float a1=1000;
//		float a2=500;
		for(float j=0; j<6.3;j=j+0.01){
		for(int i=0; i<9; i++)
		{
			pid_calc(&PID_SPD[i],MOTOR_Meas[i].speed_rpm[NOW],M_Current[i]);
//      SET_Current[i]=WANT_SPD[i]*MOTOR_Rate+PID_SPD[i].delta_out;
//			SET_Current[i]=PID_SPD[i].delta_out;
			SET_Current[i]=a1*sin(j);
			limit_abs(&SET_Current[i],Current_MAX);
		}
		MOTOR_SET_Current(SET_Current[0],SET_Current[1],SET_Current[2],SET_Current[3],SET_Current[4],SET_Current[5],SET_Current[6],SET_Current[7]);
		delay_ms(5);
	}
//		delay_ms(5000);
		
//	for(float j=0; j<6.3;j=j+0.01){
//		for(int i=0; i<9; i++)
//		{
//			pid_calc(&PID_SPD[i],MOTOR_Meas[i].speed_rpm[NOW],M_Current[i]);
////      SET_Current[i]=WANT_SPD[i]*MOTOR_Rate+PID_SPD[i].delta_out;
////			SET_Current[i]=PID_SPD[i].delta_out;
//			SET_Current[i]=-a2*sin(j);
//			limit_abs(&SET_Current[i],Current_MAX);
//		}
//		MOTOR_SET_Current(SET_Current[0],SET_Current[1],SET_Current[2],SET_Current[3],SET_Current[4],SET_Current[5],SET_Current[6],SET_Current[7]);
//		delay_ms(5);
//	}
//		delay_ms(5000);
	
	
	
	
	}
	
//600mm右侧程序
//	while(1)
//	{
//		if(DJI_RE.SW1 != 1)
//		{
//			if(DJI_RE.SW1 == 3)  //飞轮低速CL
//			{
//				M_Current[0] = -1500;
//			}
//			else if (DJI_RE.SW1 == 2)  //飞轮高速HL
//			{
//				M_Current[0] = -3000;
//			}
//			
//			
//			M_Current[1] = -10*DJI_RE.Right_Y;
//		}
//		else
//		{
//			TIM_SetCompare1(TIM2,0);
//			TIM_SetCompare2(TIM2,0);
//			M_Current[0] = 0;
//			M_Current[1] = 0;
//		}
//		
//		
//		if(DJI_RE.SW2 == 1)  //TIM2控制刹车
//		{
//			TIM_SetCompare1(TIM2,9999);
//			TIM_SetCompare2(TIM2,0);
//		}
//		else if(DJI_RE.SW2 == 2)
//		{
//			TIM_SetCompare1(TIM2,0);
//			TIM_SetCompare2(TIM2,9999);
//		}
//		else
//		{
//			TIM_SetCompare1(TIM2,0);
//			TIM_SetCompare2(TIM2,0);
//		}

//		for(int i=0; i<2; i++)
//		{
//			pid_calc(&PID_SPD[i],MOTOR_Meas[i].speed_rpm[NOW],M_Current[i]);
////      SET_Current[i]=WANT_SPD[i]*MOTOR_Rate+PID_SPD[i].delta_out;
//			SET_Current[i]=PID_SPD[i].delta_out;
//			limit_abs(&SET_Current[i],Current_MAX);
//		}
//		MOTOR_SET_Current(SET_Current[0],SET_Current[1],SET_Current[2],SET_Current[3],SET_Current[4],SET_Current[5],SET_Current[6],SET_Current[7]);
//		delay_ms(5);
//	}
}





void DBUS_RST(DJI_REMOTE Remote)
{
RST_Left_X=Remote.Left_X;
RST_Left_Y=Remote.Left_Y;
RST_Right_X=Remote.Right_X;
RST_Right_Y=Remote.Right_Y;
}


u8 RST_MID=10;

//gear 档位 默认M_G_1
//Factor_Roll 旋转系数  默认为1   该值越大  机器人旋转的角速度越大
void Romote_To_Currnt_Handle(
DJI_REMOTE Remote,int16_t* M_Current,u8 gear,float Factor_Roll
)
{
	 double KP;
	 int Vx=Remote.Left_X-RST_Left_X; //MAX=1684  MIN=354  差值660；
	 int Vy=Remote.Left_Y-RST_Left_Y;
	 int W=Remote.Right_X-RST_Right_X;
   int a=W;

   if((a>=0)&&(a<=RST_MID))W=0;
   if((a<=0)&&(a>=-RST_MID))W=0;

   switch(gear)
   {
   case M_G_1:KP=6;break;
      case M_G_2:KP=8;break;
         case M_G_3:KP=10;break;
            case M_G_4:KP=12;break;
              default:KP=1;break;
   }
		 if(DJI_RE.Flag_S==Connected)
			{
		M_Current[0]=(Vy+Vx+W*Factor_Roll)*KP;
		M_Current[1]=(Vy-Vx-W*Factor_Roll)*KP;
		M_Current[2]=(Vy-Vx+W*Factor_Roll)*KP;
		M_Current[3]=(Vy+Vx-W*Factor_Roll)*KP;

        for(int i=0;i<4;i++)
        {
           if(MOTOR_Meas[i].Dir==M_Back)
           {
            M_Current[i]=-M_Current[i];
           }
        }
			}
			else
			{
		M_Current[0]=0;
		M_Current[1]=0;
		M_Current[2]=0;
		M_Current[3]=0;
			}
}

void Romote_To_Currnt_Handle1(int Remote,int16_t* M_Current,u8 gear,float Factor_Roll)
{
	 double KP;
	 int Vx=350; //MAX=1684  MIN=354  差值660；
	 int Vy=350;
	 int W=10;
   int a=W;

   if((a>=0)&&(a<=9))W=0;
   if((a<=0)&&(a>=-9))W=0;

   switch(gear)
   {
   case M_G_1:KP=-2;break;
      case M_G_2:KP=8;break;
         case M_G_3:KP=10;break;
            case M_G_4:KP=12;break;
              default:KP=1;break;
   }
		 if(Remote==1)
			{
			    M_Current[0]=600;
		      M_Current[1]=600;
		      M_Current[2]=600;
		      M_Current[3]=600;
				  M_Current[4]=600;
		      M_Current[5]=600;
		      M_Current[6]=600;
		      M_Current[7]=600;
				}	
			else
			{
				if(Remote==2)
				{
					M_Current[0]=-600;
		      M_Current[1]=-600;
		      M_Current[2]=-600;
		      M_Current[3]=-600;
					M_Current[4]=-600;
		      M_Current[5]=-600;
		      M_Current[6]=-600;
		      M_Current[7]=-600;
				}
				else
				{
					 M_Current[0]=0;
		       M_Current[1]=0;
		       M_Current[2]=0;
		       M_Current[3]=0;
					 M_Current[4]=0;
		       M_Current[5]=0;
		       M_Current[6]=0;
		       M_Current[7]=0;
				}
			}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin==GPIO_PIN_2)
  {
     LED_R=~LED_R;
  }
}


