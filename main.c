#include "reg52.h"
#define uchar unsigned char
#define uint unsigned int
sbit left_motorA = P0^2;
sbit left_motorB = P0^3;
sbit right_motorA = P0^5;
sbit right_motorB = P0^4;
sbit LED1 = P2^0;			//小车从右到左的红外标号为1-8
sbit LED2 = P2^1;
sbit LED3 = P2^2;
sbit LED4 = P2^3;
sbit LED5 = P2^4;
sbit LED6 = P2^5;
sbit LED7 = P2^6;
sbit LED8 = P2^7;

#define FOSC 24000000L
#define BAUD 9600
sfr AUXR = 0X8e;
sfr T2H = 0Xd6;
sfr T2L = 0Xd7;

void Interrupt_Init()
{
	INT0 = 1;
	INT1 = 1;
	IT0 = 1;
	IT1 = 1;
	EX0 = 1;			//打开外部中断0和1，配置为边沿触发
	EX1 = 1;
	
	TMOD = 0X21;	//定时器0：16位定时	定时器1：8位自动重装	
	TH0 = (65536-10000)/256;
	TL0 = (65536-10000)%256;
	ET0 = 1;			//使能定时器0中断，定时5ms
	
	TH1 = 0X9C;	
	TL1 = 0X9C;
	ET1 = 1;			//使能定时器中断1，定时0.05ms
	
	T2L = (65536-(FOSC/4/BAUD));
	T2H = (65536-(FOSC/4/BAUD))>>8;
	AUXR = 0X14;
	AUXR |= 0X01;
	SM0 = 0;
	SM1 = 1;			//定时器2设置波特率115200
	
	EA = 1;
	ES = 1;
	REN = 1;
}


#define max_speed 100
#define min_speed 40
#define average_speed 70
int PWM_left = average_speed;
int PWM_right = average_speed;
int beta_PWM = 0;
const int PID_P = 40;		// 比例系数P
const int PID_I = 1;		// 积分系数I
const int PID_D = 3;		// 差分系数D
int Error[5] = {0,0,0,0,0};	//小车在5个时刻采集到的数据
bit SpecialCircle_Flag = 0;	//进入8字弯的标志
unsigned int count_5ms = 0;
bit TimeStart_Flag = 0;

void Scan_Line() interrupt 1
{
	if(TimeStart_Flag)
		count_5ms++;
	
	Error[4] = Error[3];
	Error[3] = Error[2];
	Error[2] = Error[1];
	Error[1] = Error[0];
	Error[0] = -4;
	
	if(LED1 == 0)	Error[0]++;
	if(LED2 == 0)	Error[0]++;
	if(LED3 == 0)	Error[0]++;
	if(LED4 == 0) Error[0]++;
	if(LED5 == 0) Error[0]++;
	if(LED6 == 0) Error[0]++;
	if(LED7 == 0) Error[0]++;
	if(LED8 == 0)	Error[0]++;
	
	if(Error[0]==-4 && ((Error[1]>=-1 && Error[1]<=1) || (Error[2]>=-1 && Error[2]<=1) || (Error[3]>=-1 && Error[3]<=1) || (Error[4]>=-1 && Error[4]<=1)) )
		SpecialCircle_Flag = 1;
	if(Error[0] != -4)
		SpecialCircle_Flag = 0;
	
	if(SpecialCircle_Flag)
	{
		PWM_left = average_speed;
		PWM_right = average_speed;
	}
	else
	{
		beta_PWM = PID_P*(Error[0]-Error[1]) + PID_I*Error[0] + PID_D*(Error[0] - 2*Error[1] + Error[2]);
		PWM_left -= beta_PWM;
		PWM_right += beta_PWM;
		if(PWM_left > max_speed)	PWM_left = max_speed;				
		if(PWM_left < min_speed)	PWM_left = min_speed;
		if(PWM_right > max_speed)	PWM_right = max_speed;
		if(PWM_right < min_speed)	PWM_right = min_speed;
	}
	TH0 = (65536-10000)/256;
	TL0 = (65536-10000)%256;
	TR0 = 1;
}


int PWM_count = 1;

void Track_Line() interrupt 3
{
	PWM_count++;
	if(PWM_count <= PWM_left)
		left_motorA = 1;
	else
		left_motorA = 0;
	if(PWM_count <= PWM_right)
		right_motorA = 1;
	else 
		right_motorA = 0;
	
	if(PWM_count>=100)
		PWM_count = 1;
}


int bit_count = 0;
unsigned long ID = 0x00000000;
bit ReceivedID = 0;

void get0() interrupt 0
{
	if(1<=bit_count && bit_count<=32)
		ID = ID<<1;
	bit_count++;
	if(bit_count == 34)
	{
		bit_count = 0;
		ReceivedID = 1;
	}
}

void get1() interrupt 2
{
	if(1<=bit_count && bit_count<=32)
		ID = (ID<<1) + 1;
	bit_count++;
	if(bit_count == 34)
	{
		bit_count = 0;
		ReceivedID = 1;
	}
}


char ComputerMessage = 0;

void Uart()	interrupt 4
{	
	if(RI)
	{
		RI = 0;
		ComputerMessage = SBUF;
	}
	else if(TI)
		TI = 0;
}


void Car_Init()		//初始化小车左右轮及红外
{
	P2 = 0XFF;			
	left_motorA = 0;
	left_motorB = 0;
	right_motorA = 0;
	right_motorB = 0;
}


void delay(unsigned int n)
{
	int i;
	while(n--)
		for(i=0;i<228;i++);
}


char SecondStr[7];

void record_time()
{
	int i;
	unsigned int count_10ms = count_5ms/2;
	SecondStr[6] = '\0';
	for(i=5;i>=0;i--)
	{
		if(i == 3)
			SecondStr[i] = '.';
		else
		{	
			SecondStr[i] = count_10ms%10 + 0x30;
			count_10ms = count_10ms/10;
		}
	}
}


void send_byte(char n)
{
	SBUF = n;
	while(!TI);
	TI = 0;
}


void send_str(char *str)
{
	while(*str != '\0')
		send_byte(*str++);
}

unsigned long CheckPoint[3] = {0X43A05195, 0X806CDF4D, 0x80528373};
int round = -1;
void main()
{
	int i = 0;
	Car_Init();
	Interrupt_Init();
	while(ComputerMessage != 'G')
	{
		send_str("Hero is online.\r\n");
		delay(10000);
	}

	ComputerMessage = 0;
	TR0 = 1;
	TR1 = 1;
	
	while(1)
	{
		if(ReceivedID)
		{
			TimeStart_Flag = 1;
			if(ID == CheckPoint[0])
				round++;
			record_time();
			ReceivedID = 0;
			while(1)
			{
				if(round == 0 && ID == CheckPoint[0])
					send_str("Hero is set out.\r\n");
				else if(round > 0 && ID == CheckPoint[0])
				{
					send_str("Hero finished ");
					send_byte(round+0x30);
					send_str(" round in ");
					send_str(SecondStr);
					send_str("s.\r\n");
				}
				else if(ID == CheckPoint[1])
				{	
					send_str("Hero passed point 1 at ");
					send_str(SecondStr);
					send_str("s.\r\n");
				}
				else if(ID == CheckPoint[2])
				{
					send_str("Hero passed point 2 at ");
					send_str(SecondStr);
					send_str("s.\r\n");
				}
					
				delay(5000);
				if(ComputerMessage == 'G')
					break;
			}
		}
		ComputerMessage = 0;
	}
}
