//��Ч�����ڢ��飺 max_speed = 1000, min_speed = 400, average_speed = 700, P = 400, I = 20, D = 50 ,����10ms
//��Ч�����ڢ��飺 U = 8V, ..., P = 500, I = 10, D = 25������10ms
#include <reg52.h>
#define max_speed 1000
#define min_speed 400
#define average_speed 700
sbit left_motorA = P0^2;
sbit left_motorB = P0^3;
sbit right_motorA = P0^5;
sbit right_motorB = P0^4;

sbit LED1 = P2^0;	//С�����ҵ���ĺ�����Ϊ1-8
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
const int PID_P = 400;		// ����ϵ��P����������
const int PID_I = 10;		// ����ϵ��I����������
const int PID_D = 25;		// ���ϵ��D����������
int Error2 = 0;	// nʱ��С����ƫ����
int Error1 = 0;	//n-1ʱ��С����ƫ����
int Error0 = 0;	//n-2ʱ��С����ƫ����
int Error_1 = 0;
int Error_2 = 0;
bit flag = 0;	//����8����ı�־λ

void initialize();	//��ʼ��
void Go_ahead(); 	//��дGo_ahead����������ǰ����ǰ��ʱ��ת��

void main()
{
	initialize();
	while(1)
		Go_ahead();
}

void initialize()
{
	P2 = 0xff;
	right_motorB = 0;  //С��ֻ��ǰ����ɲ�������ܺ���
	left_motorB = 0;   //ͬ��
	TMOD = 0X11;
	EA = 1;
	ET0 = 1; 	//������ʱ��0�ж�
	ET1 = 1;	//������ʱ��1�ж�

	TH1 = 0XFF;
	TL1 = 0XF7;
	TR1 = 1;	//��ʱ��1��ʼ��ʱ��10us�ж�һ��

	TH0 = 0XD8;
	TL0 = 0XF0;
	TR0 = 1;	//��ʱ��0��ʼ��ʱ��10ms�ж�һ��
}

void Go_ahead()	  			//PWM_left�����ֵ�ռ�ձ�
{							//PWM_right: ���ֵ�ռ�ձ�	
	if(PWM_left > max_speed)	PWM_left = max_speed;				
	if(PWM_left < min_speed)	PWM_left = min_speed;
	if(PWM_right > max_speed)	PWM_right = max_speed;
	if(PWM_right < min_speed)	PWM_right = min_speed;
	count = 0;
	while(count <= 1000);
}

void time0_up() interrupt 3	  //��ʱ���ж�1�� ÿ0.01ms����һ���ж�
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
void scan()	  interrupt 1  //�жϽ��к���ɨ���⣬ÿ50msɨ��һ��
{
	Error_2 = Error_1;
	Error_1 = Error0;
	Error0 = Error1;	//��¼n-2ʱ�̵����Error0	 
	Error1 = Error2;	//��¼n-1ʱ�̵����Error1
	Error2 = -5;
	// 8��if��佫�����8������ֵת����һ��int����Error2��
	//��Error2�жϵ�ǰС���Ƿ�ƫ����ߣ�Error2�ķ��ű�ʾƫ������ң�Error2�Ĵ�С��ʾƫ��ĳ̶�
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
		//PID�㷨�õ�ռ�ձȵķ�����beta_PWM
		PWM_left = PWM_left - beta_PWM;
		PWM_right = PWM_right + beta_PWM;	
	}	

	TH0 = 0XD8;
	TL0 = 0XF0;
	TR0 = 1;
}