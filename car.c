//有效参数第①组： max_speed = 1000, min_speed = 400, average_speed = 700, P = 400, I = 20, D = 50 ,周期10ms
//有效参数第②组： U = 8V, ..., P = 500, I = 10, D = 25，周期10ms
#include <reg52.h>
#define max_speed 1000
#define min_speed 400
#define average_speed 700
sbit left_motorA = P0^2;
sbit left_motorB = P0^3;
sbit right_motorA = P0^5;
sbit right_motorB = P0^4;

sbit LED1 = P2^0;	//小车从右到左的红外标号为1-8
sbit LED2 = P2^1;
sbit LED3 = P2^2;
sbit LED4 = P2^3;
sbit LED5 = P2^4;
sbit LED6 = P2^5;
sbit LED7 = P2^6;
sbit LED8 = P2^7;

int PWM_left = average_speed, PWM_right = average_speed;
int beta_PWM = 0;
int count = 0;
const int PID_P = 400;		// 比例系数P，参数待调
const int PID_I = 10;		// 积分系数I，参数待调
const int PID_D = 25;		// 差分系数D，参数待调
int Error2 = 0;	// n时刻小车的偏移量
int Error1 = 0;	//n-1时刻小车的偏移量
int Error0 = 0;	//n-2时刻小车的偏移量
int Error_1 = 0;
int Error_2 = 0;
bit flag = 0;	//进入8字弯的标志位

void initialize();	//初始化
void Go_ahead(); 	//编写Go_ahead函数，用于前进和前进时的转弯

void main()
{
	initialize();
	while(1)
		Go_ahead();
}

void initialize()
{
	P2 = 0xff;
	right_motorB = 0;  //小车只能前进或刹车，不能后退
	left_motorB = 0;   //同上
	TMOD = 0X11;
	EA = 1;
	ET0 = 1; 	//开启定时器0中断
	ET1 = 1;	//开启定时器1中断

	TH1 = 0XFF;
	TL1 = 0XF7;
	TR1 = 1;	//定时器1开始计时，10us中断一次

	TH0 = 0XD8;
	TL0 = 0XF0;
	TR0 = 1;	//定时器0开始计时，10ms中断一次
}

void Go_ahead()	  			//PWM_left：左轮的占空比
{							//PWM_right: 右轮的占空比	
	if(PWM_left > max_speed)	PWM_left = max_speed;				
	if(PWM_left < min_speed)	PWM_left = min_speed;
	if(PWM_right > max_speed)	PWM_right = max_speed;
	if(PWM_right < min_speed)	PWM_right = min_speed;
	count = 0;
	while(count <= 1000);
}

void time0_up() interrupt 3	  //定时器中断1： 每0.01ms产生一次中断
{	
	if(count == 0)	
	{
		left_motorA = 1;
		right_motorA = 1;
	}
	if(count == PWM_left)	left_motorA = 0;
	if(count == PWM_right)	right_motorA = 0;
	count++;

	TH1 = 0XFF;
	TL1 = 0XF7;
	TR1 = 1;
}
void scan()	  interrupt 1  //中断进行红外扫描检测，每50ms扫描一次
{
	Error_2 = Error_1;
	Error_1 = Error0;
	Error0 = Error1;	//记录n-2时刻的误差Error0	 
	Error1 = Error2;	//记录n-1时刻的误差Error1
	Error2 = -5;
	// 8个if语句将红外的8个布尔值转化成一个int参数Error2，
	//由Error2判断当前小车是否偏离黑线，Error2的符号表示偏离的左右，Error2的大小表示偏离的程度
	if(LED1==0)	Error2 = Error2 + 1;
	if(LED2==0)	Error2 = Error2 + 2;
	if(LED3==0)	Error2 = Error2 + 1;
	if(LED4==0)	Error2 = Error2 + 1;
	if(LED5==0)	Error2 = Error2 + 1;
	if(LED6==0)	Error2 = Error2 + 1;
	if(LED7==0)	Error2 = Error2 + 2;
	if(LED8==0)	Error2 = Error2 + 1;
	
	if(Error2==-5 && ((Error1>=-1 && Error1<=1)||(Error0>=-1 && Error0<=1)||(Error_1>=-1&&Error_1<=1)||(Error_2>=-1&&Error_2<=1)))
		flag = 1;
	if(Error2!=-5)	
		flag = 0;

	if(flag)
	{
		PWM_left = average_speed;
		PWM_right = average_speed;
	}
	else 
	{
		beta_PWM = PID_P*(Error2 - Error1) + PID_I*Error2 + PID_D*(Error2 - 2*Error1 + Error0);  
		//PID算法得到占空比的反馈量beta_PWM
		PWM_left = PWM_left - beta_PWM;
		PWM_right = PWM_right + beta_PWM;	
	}	

	TH0 = 0XD8;
	TL0 = 0XF0;
	TR0 = 1;
}