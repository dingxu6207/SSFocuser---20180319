/**
  ******************************************************************************
  * @file    main.c
  * @author  HeShouSheng & DINGXU
  * @version V1.0
  * @date    2017-11-20
  * @brief   调焦器
  ******************************************************************************
  */ 
	
#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "stm32f10x_flash.h"
#include "bsp_led.h"
#include "bsp_SysTick.h"
#include "bsp_usart.h"
#include "bsp_hc05.h"
#include "bsp_usart_blt.h"
#include <stdio.h>
#include <stdlib.h>
#include "bsp_ds18b20.h"
#include <string.h>
#include "bsp_TiMbase.h"
#include "stdbool.h"
#include "Exti44E.h"
#include "bsp_TimeCover.h"

#define WriteFlashAddress    ((u32)0x0800FC00)//存储到最后一页，地址范围：0x0800 FC00~0x0800 FFFF


typedef union//union只能对第一个变量初始化
{
    struct
    {
			//设备信息
			//char  Device[16];
			//char  Version[16];	
			//蓝牙信息
			char  BTName[16];   
			//电机设置
      u8    IsReverse;
			u8    SubDivision;
			u16   MotorSpeed;
			//温度补偿
			//float CompRatio;
			//int   CompSteps;
			//调焦参数
			//float StepSize;
			//int   MaxStep;
			int   Position;
			u32   DelayCount;
			//加减速曲线数组
			u16   SCurve[256];
			
    } CfgData;
		u32 SaveArray[140];
}UnionData; 

UnionData UD;

char  *pVersion = "SSFocuser SB1.0";    //SE-串口版,BT-蓝牙版,SB-串口蓝牙版,不能在线修改

char  *pBTName = "SSFocuser"; //BT SSID

bool  bIsReverse = false;     //电机是否反向
u8    uSubdivision = 8;       //步进电机细分
u8    uSubdivisionCur = 8;    //步进电机细分
u16   uSpeed  = 1024;		    	//电机转动速度
u16  	uSpeedCur=8;
u16   uSCurve[256]={0};       //加速曲线数据数组
u8    uSCnt=0;                //加速曲线数据个数
u8    uSCntCur=0;
u16   i=0;

//调焦参数
int   iStepCount = 0;         //当前步数
u8    uMoveState = 0;         //运动状态0-Idle,1-Move,2-Slew out&Slew in,3-Align

int   iStepStart = 0;         //命令步数
int   iStepStop  = 0;         //命令步数
int   iStepDelt  = 0;					//命令步数

float fTemperature=-273.15;
u8    uGetTempCycle=10;
u8    uGetTempCount=0;

char  CmdBuff[32] = {0};     //回复字符串
char  ReplyBuff[128] = {0};  //回复字符串
char  TmpBuff[4] = {0};

u32   uDelayCount=0;
u32   uDelayCountMax=0;
u16   uSpeedCover = 5;

//bool  bInitial    =false;      //是否初始化
bool  bIsMoving   =false;	     //运动状态标志true-stop,false-move
bool  bIncreCount =false;      //是否递增计数true-递增计数 false-递减计数
bool  bTempAvail  =false;      //温度是否可用

unsigned char UART_RxPtr_Prv=0;    //上次读取位置
unsigned char BLTUART_RxPtr_Prv=0; //上次读取位置
unsigned char UART_RxCmd=0;        //未读命令个数
unsigned char BLTUART_RxCmd=0;     //未读命令个数
//延迟函数
void Delay(u16 dt)//大于1mS
{
	u32 i=0;
	for(i=0;i<dt*2500;i++)
	;
}
u8 Write_Flash(u32 *buff, u8 len)
{    
	volatile FLASH_Status FLASHStatus;
	u32 Address = WriteFlashAddress;
	FLASHStatus = FLASH_COMPLETE;
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
	FLASHStatus = FLASH_ErasePage(WriteFlashAddress);
	if(FLASHStatus == FLASH_COMPLETE)
	{
		u8 k;
		for(k=0;(k<len) && (FLASHStatus == FLASH_COMPLETE);k++)
		{
				FLASHStatus = FLASH_ProgramWord(Address, buff[k]);
				Address = Address + 4;
		}        
		FLASH_Lock();
	}
	else
	{
		return 0;
	}
	if(FLASHStatus == FLASH_COMPLETE)
	{
		return 1;
	}
	return 0;
}

void Read_Flash(u32 *buff, u8 len)
{
	u32 Address = WriteFlashAddress;
	u8 k;
	for(k=0;k<len;k++)
	{
		buff[k] =  (*(vu32*) Address);
		Address += 4;     
	}
}
//仅读取Flash，不赋值给变量
void ReadCfg0(void)
{
	//读取
	Read_Flash(UD.SaveArray, 140);
}
//读取Flash并赋值给变量
void ReadCfg(void)
{
	//读取
	Read_Flash(UD.SaveArray, 140);
	//赋值
	//pDevice =UD.CfgData.Device;
	//pVersion=UD.CfgData.Version;
	
	pBTName = UD.CfgData.BTName;
	
	bIsReverse = UD.CfgData.IsReverse==0?false:true;
	uSubdivision = UD.CfgData.SubDivision;
	uSpeed  = UD.CfgData.MotorSpeed;
	
	//fCompRatio=UD.CfgData.CompRatio;
	//iCompSteps=UD.CfgData.CompSteps;
		
	//fStepSize=UD.CfgData.StepSize;
	//iMaxStep=UD.CfgData.MaxStep;
	iStepCount = UD.CfgData.Position;
	uDelayCountMax=UD.CfgData.DelayCount;
	memcpy(uSCurve,UD.CfgData.SCurve,512);
}
//将变量写入Flash存储
void WriteCfg(void)
{
	//strcpy(UD.CfgData.Device, pDevice);
	//strcpy(UD.CfgData.Version, pVersion);
	
	strcpy(UD.CfgData.BTName, pBTName);
	
	UD.CfgData.IsReverse=(bIsReverse==true)?1:0;
	UD.CfgData.SubDivision=uSubdivision;
	UD.CfgData.MotorSpeed=uSpeed;
	
	//UD.CfgData.CompRatio=fCompRatio;
	//UD.CfgData.CompSteps=iCompSteps;
	
	//UD.CfgData.StepSize=fStepSize;
	//UD.CfgData.MaxStep=iMaxStep;
	UD.CfgData.Position=iStepCount;
	UD.CfgData.DelayCount=uDelayCountMax;
	//void *memcpy(void *dest, const void *src, size_t n);
	memcpy(UD.CfgData.SCurve,uSCurve,512);
	//写入
	Write_Flash(UD.SaveArray, 140);
}
//设置为默认值
void SetDefault(void)
{
	pBTName = "SSFocuser"; //BT SSID

	bIsReverse = false;     //电机是否反向
	uSubdivision = 8;       //步进电机细分
	uSpeed  = 1024;		    	//电机转动速度
	uDelayCountMax=0;
	//初始化加速曲线
	uSCurve[0]=15;
	uSCurve[1]=uSubdivision;
	uSCurve[2]=uSubdivision/2;
	for(i=3;i<255;)//500毫秒调速一次
	{
		uSCurve[i]=(u16)(uSCurve[i-2]*3/2);
		uSCurve[i+1]=(u16)(uSCurve[i]/2);
		i=i+2;
	}
	WriteCfg();
	Delay(500);
}
float GetTemperature(void)
{
	if(bTempAvail)
	{
		return DS18B20_Get_Temp();//第一次读取返回固定值
	}
	else
		return -273.15;
}
//通过某个串口发送
void SendTo(u8 MyComPort,char MyReplyBuff[])
{
	switch(MyComPort)
	{
		case 0:
			printf("%s", MyReplyBuff);
			break;
		case 1:
			BLTUsart_SendString(USART2,MyReplyBuff);
			break;
	}
	memset(MyReplyBuff,0,128);
}
//设置蓝牙
void SetBluetooth(void)
{
	char  TemBuff[24] = {0};
	sprintf(TemBuff, "AT+NAME=%s", pBTName);
	BLTUsart_SendString(USART2,TemBuff);
	Delay(1000);
	//memset(TemBuff,0,24);
	//sprintf(TemBuff, "AT+RESET");
	//BLTUsart_SendString(USART2,TemBuff);
	//Delay(1000);
	//测试发送到串口
	//printf("%s", TemBuff);
}

//设置电机反向
void SetReverse(void)
{
	bIsReverse = !bIsReverse; 
	UD.CfgData.IsReverse=bIsReverse?1:0;
	WriteCfg();
}
//设置细分
void SetDivision(u8 sMode)
{
	switch(sMode)
	{
		case 1:
		{
			/* M0=0 */
			GPIO_ResetBits(M0_GPIO_PORT, M0_GPIO_PIN);
			/* M1=0 */
			GPIO_ResetBits(M1_GPIO_PORT, M1_GPIO_PIN);
			/* M2=0 */
			GPIO_ResetBits(M2_GPIO_PORT, M2_GPIO_PIN);
			break;
		}
		case 2:
		{
			/* M0=1 */
			GPIO_SetBits(M0_GPIO_PORT, M0_GPIO_PIN);
			/* M1=0 */
			GPIO_ResetBits(M1_GPIO_PORT, M1_GPIO_PIN);
			/* M2=0 */
			GPIO_ResetBits(M2_GPIO_PORT, M2_GPIO_PIN);	
			break;
		}
		case 4:
		{
			/* M0=0 */
			GPIO_ResetBits(M0_GPIO_PORT, M0_GPIO_PIN);
			/* M1=1 */
			GPIO_SetBits(M1_GPIO_PORT, M1_GPIO_PIN);
			/* M2=0 */
			GPIO_ResetBits(M2_GPIO_PORT, M2_GPIO_PIN);	
			break;
		}
		case 8:
		{
			/* M0=1 */
			GPIO_SetBits(M0_GPIO_PORT, M0_GPIO_PIN);	   
			/* M1=1 */
			GPIO_SetBits(M1_GPIO_PORT, M1_GPIO_PIN);
			/* M2=0 */
			GPIO_ResetBits(M2_GPIO_PORT, M2_GPIO_PIN);	
			break;
		}
		case 16:
		{
			/* M0=0 */
			GPIO_ResetBits(M0_GPIO_PORT, M0_GPIO_PIN);	   
			/* M1=0 */
			GPIO_ResetBits(M1_GPIO_PORT, M1_GPIO_PIN);
			/* M2=1 */
			GPIO_SetBits(M2_GPIO_PORT, M2_GPIO_PIN);		
			break;
		}
		case 32:
		{
			/* M0=1 */
			GPIO_SetBits(M0_GPIO_PORT, M0_GPIO_PIN);   
			/* M1=1 */
			GPIO_SetBits(M1_GPIO_PORT, M1_GPIO_PIN);
			/* M2=1 */
			GPIO_SetBits(M2_GPIO_PORT, M2_GPIO_PIN);
			break;
		}
	}
}
//设置速度
void SetSpeed(u16 uSetSpeed)
{
	SetSpeedMoter(10000/uSetSpeed);
}
//初始化电机
void InitMotor(void)
{
	bIsMoving = false;
	bIncreCount = true;
	//TIM_Cmd(BASIC_TIM, ENABLE);//定时器开启
	SetDivision(uSubdivision);
	SetSpeed(uSpeed);
}
//往外转动
void SlewOut(void)//:F+#
{
	bIsMoving = true;//运动状态标志
	bIncreCount = true;//增计数标志
	//起始速度，用于加速
	uSCntCur=0;
	uSpeedCur=uSCurve[uSCntCur*2+1];
	iStepDelt=uSCurve[uSCntCur*2+2];
	SetSpeed(uSpeedCur);
	//PWM输出置低电平
	GPIO_ResetBits(ADVANCE_TIM_CH1_PORT, ADVANCE_TIM_CH1_PIN); 
	//设置电机运动方向
	if(bIsReverse)
		GPIO_ResetBits(LED2_GPIO_PORT, LED2_GPIO_PIN);  //DRV8825dir信号
	else
		GPIO_SetBits(LED2_GPIO_PORT, LED2_GPIO_PIN);    //DRV8825dir信号
	//使能DRV8825
	GPIO_ResetBits(LED5_GPIO_PORT, LED5_GPIO_PIN); //DRV8825使能信号
	Delay(50);
	//使能PWM时钟
	ControlMotor(ENABLE);
}
//往里转动
void SlewIn(void)//:F-#
{
	bIsMoving = true; //运动状态标志
	bIncreCount = false;//减计数标志
	//起始速度，用于加速
	uSCntCur=0;
	uSpeedCur=uSCurve[uSCntCur*2+1];
	iStepDelt=uSCurve[uSCntCur*2+2];
	SetSpeed(uSpeedCur);
	//PWM输出置低电平
	GPIO_ResetBits(ADVANCE_TIM_CH1_PORT, ADVANCE_TIM_CH1_PIN); 
	//设置电机运动方向
	if(bIsReverse)
		GPIO_SetBits(LED2_GPIO_PORT, LED2_GPIO_PIN);    //DRV8825dir信号
	else
		GPIO_ResetBits(LED2_GPIO_PORT, LED2_GPIO_PIN);  //DRV8825dir信号
	//使能DRV8825
	GPIO_ResetBits(LED5_GPIO_PORT, LED5_GPIO_PIN); //DRV8825使能信号
	Delay(50);
	//使能PWM时钟
	ControlMotor(ENABLE);
}
//转动固定步数
void MoveTo()
{ 
	if(iStepStop>iStepStart)
	{
		SlewOut();
	}
	else
	{
		SlewIn();		
	}
}
//停止
void Halt(void)
{
	ControlMotor(DISABLE);
	bIsMoving = false;
	uMoveState = 0;
}
//暂停
void Pause(void)//不改变运动状态
{
	ControlMotor(DISABLE);
	bIsMoving = false;
}
//控制镜盖
void OperCover(u16 uSpeedCover)
{
	Halt();//不用Halt也会停止电机，不知为什么
  ControlCover(ENABLE); 
	SetSpeedCover(uSpeedCover);
	Delay(1000);
	ControlCover(DISABLE);
}

//掉电保存数据
void SaveData(void)
{
	//停止运动及计数
	Halt();
	//保存配置数据
	WriteCfg();
}
//查找下一个
u8 Next(u8 Prv)
{
	if(Prv<255)
		return Prv+1;
	else //255
		return 0;
}
//读取一条命令，去掉头尾:#，指针指向下一条命令
u8 ReadCmd(unsigned char *RxBuffer,unsigned char *Ptr)
{
	u8 Flag=0,i=0;
	memset(CmdBuff,0,32);
	for(;;)
	{
		//下面三条判断的顺序不能调整
		if(RxBuffer[*Ptr]==35)//#结束命令
		{ 
			if(Flag==1)//命令完整
			{
				CmdBuff[i]=NULL;//没有结束符，split函数会出错
				Flag=2;
			}
			else
				Flag=3;
		}
		if(Flag==1)//:后才开始读取命令字符串
		{
			CmdBuff[i]=RxBuffer[*Ptr];
			i++;
		}
		if(RxBuffer[*Ptr]==58)//:
		{
			Flag=1;
			i=0;
		}
		//循环读取串口缓冲区
		*Ptr=Next(*Ptr);
		if(Flag>1)//跳出循环
			break;
	}
	return i;
}
//命令处理
bool CmdProcess(u8 MyComPort,unsigned char *RxBuffer,unsigned char *Ptr)
{
	if(ReadCmd(RxBuffer,Ptr)<1)
		return false;
	memset(ReplyBuff,0,128);
	if(CmdBuff[0]=='F')//调焦器命令以F开头
	{
		switch (CmdBuff[1])
		{
			case '?':  //Focuser version
				{
					sprintf(ReplyBuff, ":?%s#\r\n", pVersion);
					SendTo(MyComPort,ReplyBuff);
					break;
				}
			case '+': 	//Slew Out		 
				{
					sprintf(ReplyBuff, ":+#\r\n");
					SendTo(MyComPort,ReplyBuff);
					Halt();
					uMoveState = 2;
					iStepStart=iStepCount;//起始位置，用于加速
					SlewOut();
					break;
				}
			case '-':   //Slew In
				{
					sprintf(ReplyBuff, ":-#\r\n");
					SendTo(MyComPort,ReplyBuff);
					Halt();
					uMoveState = 2;
					iStepStart=iStepCount;//起始位置，用于加速
					SlewIn();
					break;
				}
			case 'B': // Busy Status  			
				{
					sprintf(ReplyBuff,":B%d#\r\n", bIsMoving);
					SendTo(MyComPort,ReplyBuff);
					break;
				}
			case 'b': //Bluetooth setting  			
				{
					sprintf(ReplyBuff, ":b#\r\n");
					SendTo(MyComPort,ReplyBuff);
					strcpy(pBTName, CmdBuff+2);
					SetBluetooth();
					break;
				}
			case 'C':  //Mirror Cover operation 			
				{
					sprintf(ReplyBuff, ":C#\r\n");
					SendTo(MyComPort,ReplyBuff);
					uSpeedCover = atoi((char const *)CmdBuff+2);
					OperCover(uSpeedCover);
					break;
				}
			case 'D':  //Define current position			
				{
					sprintf(ReplyBuff, ":D#\r\n");
					SendTo(MyComPort,ReplyBuff);
					Halt();
					iStepCount = atoi((char const *)CmdBuff+2)*uSubdivision;
					break;
				}
			case 'M': //Set Motor subdivision,Speed and Reverse
				{
					sprintf(ReplyBuff, ":M#\r\n");
					SendTo(MyComPort,ReplyBuff);
					Halt();
					//设置反向
					if(bIsReverse != (CmdBuff[2] == '1')?true:false)
					{
						bIsReverse = (CmdBuff[2] == '1')?true:false;		
					}
					//设置细分
					memset(TmpBuff,0,8);
					TmpBuff[0]=CmdBuff[3];TmpBuff[1]=CmdBuff[4];TmpBuff[2]=CmdBuff[5];
					uSubdivisionCur = atoi((char const *)TmpBuff+0);
					if(uSubdivisionCur!=uSubdivision)//不同才设置
					{
						switch(uSubdivisionCur)//判断有效性
						{
							case 1:
							case 2:
							case 4:
							case 8:
							case 16:
							case 32:
							{
								//计算当前细分下的计数值
								iStepCount = (iStepCount *uSubdivisionCur)/uSubdivision;
								uSubdivision=uSubdivisionCur;
								SetDivision(uSubdivision);
								break;
							}
						}
					}
					//设置速度
					memset(TmpBuff,0,8);
					TmpBuff[0]=CmdBuff[6];TmpBuff[1]=CmdBuff[7];TmpBuff[2]=CmdBuff[8];
					uSpeedCur = atoi((char const *)TmpBuff+0)*uSubdivision;
					if(uSpeedCur!=uSpeed)
					{
						if((uSpeedCur>=uSubdivision)&&(uSpeedCur<=uSubdivision*500))//保证不小于1Hz,不大于500Hz
						{
							uSpeed=uSpeedCur;
						}
					}
					
					//保存配置
					WriteCfg();
					break;
				}
			case 'm': //Get Subdivision		
				{
					sprintf(ReplyBuff,":m%d~%d~%d#\r\n",bIsReverse,uSubdivision,uSpeed/uSubdivision);
					SendTo(MyComPort,ReplyBuff);
					break;
				}
			case 'P':    //Move command
				{
					sprintf(ReplyBuff, ":P#\r\n");
					SendTo(MyComPort,ReplyBuff);
					Halt();
					iStepStart=iStepCount;//起始位置，用于加速
					iStepStop = atoi((char const *)CmdBuff+2)*uSubdivision;
					if(iStepStart!=iStepStop)
					{
						uMoveState=1;
						MoveTo();
					}
					break;
				}
			case 'p':  //Get Position
				{
					sprintf(ReplyBuff,":p%d#\r\n", iStepCount/uSubdivision);
					SendTo(MyComPort,ReplyBuff);
					break;
				}
			case 'Q': 	//Halt    
				{
					sprintf(ReplyBuff, ":Q#\r\n");
					SendTo(MyComPort,ReplyBuff);
					Halt();
					break;
				}
			case 'R': //Rotator or CAA			
				{
					
					break;
				}
			case 'r': //		
				{
					
					break;
				}
			case 'S':    //加减速曲线设置
				{
					sprintf(ReplyBuff, ":S#\r\n");
					SendTo(MyComPort,ReplyBuff);
					if (CmdBuff[2] == 'E')//结束标志，存储数据
						WriteCfg();
					else
					{
						if (CmdBuff[2] == 'B')//开始标志，第一个为数据个数
						{
							Halt();//不停止会出错
							uSCnt=0;
							memset(uSCurve,0,256);
						}
						else if (CmdBuff[2] == 'D')//数据传输
							uSCnt++;
						uSCurve[uSCnt]=atoi((char const *)CmdBuff+3);
					}
					break;							
				}
			case 's':    //打印加减速曲线
				{
					int ss=0;
					sprintf(ReplyBuff, ":sTotal:%d---#\r\n",uSCurve[0]);
					SendTo(MyComPort,ReplyBuff);
					for(i=0;i<uSCurve[0];)
					{
						ss=ss+uSCurve[i*2+2];
						sprintf(ReplyBuff, ":s%d,%d,%d#\r\n",uSCurve[i*2+1],uSCurve[i*2+2],ss);
						SendTo(MyComPort,ReplyBuff);
						i++;
					}
					break;
				}
			case 't':   //Get temperature
				{
					sprintf(ReplyBuff,":t%.2f#\r\n", GetTemperature());//保留2位小数
					SendTo(MyComPort,ReplyBuff);
					break;
				}
			case 'Z':    //Test delay time
				{
					sprintf(ReplyBuff, ":Z#\r\n");
					SendTo(MyComPort,ReplyBuff);
					if(uDelayCountMax != atoi((char const *)CmdBuff+2)*200)
					{
						uDelayCountMax = atoi((char const *)CmdBuff+2)*200;
						WriteCfg();
					}
					break;
				}
			case 'z': 
				{
					sprintf(ReplyBuff, ":z%d#\r\n",uDelayCountMax/200);
					SendTo(MyComPort,ReplyBuff);
					break;
				}
			default://Unknow Command
				{
					sprintf(ReplyBuff, ":U#\r\n");
					SendTo(MyComPort,ReplyBuff);
					break;
				}
		}
	}
	return true;
}
//主程序
int main()
{	
	LED_GPIO_Config();

	SysTick_Init();
	
	USART_Config();
	
	BASIC_TIM_Init();
	
	Cover_TIM_Init();
	
	EXTI_44E_Config();
	
	BLT_USART_Config();

	//初始化温度传感器
	if( DS18B20_Init())	
		bTempAvail=false;
	else
	{
		DS18B20_Get_Temp();//第一次读取返回固定值
		bTempAvail=true;
	}
	//初始化参数
	ReadCfg0();//读取参数
	if((UD.CfgData.IsReverse>1)||(UD.CfgData.SubDivision<1))//未初始化过就初始化
	{
		SetDefault();
		//SysTick_Delay_Ms(500);//这个函数会引起WiFi设置失效，不能使用
	}
	ReadCfg();
	//初始化电机
	InitMotor();
	
	while(1)
	{	
		//处理串口1数据
		while(UART_RxCmd>0)
		{
			UART_RxCmd--;//命令计数减一
			CmdProcess(0,UART_RxBuffer,&UART_RxPtr_Prv);
		}	
	
	  //处理串口2数据	
	  while(BLTUART_RxCmd>0)
	  { 
			BLTUART_RxCmd--;//命令计数减一
			CmdProcess(1,BLTUART_RxBuffer,&BLTUART_RxPtr_Prv);
	  }
		
		//工作状态
	  switch(uMoveState)
		{
			case 1://Move
			{
				;
				break;
			}	
			case 2://Slew
			{
				;
				break;
			}
			case 3://Alignment
			{
				;
				break;
			}
			default://Idle
			{
				Halt();//uMoveState=0;
				break;
			}
		}
		//#if 0
		
		uDelayCount++;
		if(uDelayCountMax>0)//只有大于零才发送回
		{
			if(uDelayCount>=uDelayCountMax)
			{
				sprintf(ReplyBuff,":a%d~%d~%.2f~%d~%d~%d#\r\n", iStepCount/uSubdivision,bIsMoving,fTemperature,bIsReverse,uSubdivision,uSpeed/uSubdivision);
				printf("%s", ReplyBuff);
				BLTUsart_SendString(USART2,ReplyBuff);
				memset(ReplyBuff,0,128);
				uDelayCount=0;
				uGetTempCount++;
				if(uGetTempCount>=uGetTempCycle)
				{
					fTemperature=GetTemperature();
					uGetTempCount=0;
				}
			}
		}
		//#endif
	}
}

/*********************************************END OF FILE**********************/
