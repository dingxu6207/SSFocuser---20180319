/**
  ******************************************************************************
  * @file    main.c
  * @author  HeShouSheng & DINGXU
  * @version V1.0
  * @date    2017-11-20
  * @brief   ������
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

#define WriteFlashAddress    ((u32)0x0800FC00)//�洢�����һҳ����ַ��Χ��0x0800 FC00~0x0800 FFFF


typedef union//unionֻ�ܶԵ�һ��������ʼ��
{
    struct
    {
			//�豸��Ϣ
			//char  Device[16];
			//char  Version[16];	
			//������Ϣ
			char  BTName[16];   
			//�������
      		u8    IsReverse;
			u8    SubDivision;
			u16   MotorSpeed;
			//�¶Ȳ���
			//float CompRatio;
			//int   CompSteps;
			//��������
			//float StepSize;
			//int   MaxStep;
			int   Position;
			//�Ӽ�����������
			u16   SCurve[256];
			
    } CfgData;
		u32 SaveArray[140];
}UnionData; 

UnionData UD;

char  *pDevice = "SSFocuser";
char  *pVersion = "SB1.0";//SE-���ڰ�,BT-������,SB-����������,���������޸�

char  *pBTName = "SSFocuser"; //BT SSID

bool  bIsReverse = false;     //����Ƿ���
u8    uSubdivision = 8;       //�������ϸ��
u8    uSubdivisionCur = 8;    //�������ϸ��
u16   uSpeed  = 2048;		    	//���ת���ٶ�
u16  	uSpeedCur=8;
u16   uSCurve[256]={0};       //����������������
u8    uSCnt=0;                //�����������ݸ���
u8    uSCntCur=0;
u16   i=0;

//��������
int   iStepCount = 0;         //��ǰ����
u8    uMoveState = 0;         //�˶�״̬0-Idle,1-Move,2-Slew out&Slew in,3-Align

int   iStepStart = 0;         //�����
int   iStepStop  = 0;         //�����
int   iStepDelt  = 0;					//�����

char  CmdBuff[32] = {0};     //�ظ��ַ���
char  ReplyBuff[128] = {0};  //�ظ��ַ���

u32   uDelayCount=0;
u32   uDelayCountMax=200000;
u16   uSpeedCover = 5;

bool  bInitial    =false;      //�Ƿ��ʼ��
bool  bIsMoving   =false;	     //�˶�״̬��־true-stop,false-move
bool  bIncreCount =false;      //�Ƿ��������true-�������� false-�ݼ�����
bool  bTempAvail  =false;      //�¶��Ƿ����

unsigned char UART_RxPtr_Prv=0;    //�ϴζ�ȡλ��
unsigned char BLTUART_RxPtr_Prv=0; //�ϴζ�ȡλ��
unsigned char UART_RxCmd=0;        //δ���������
unsigned char BLTUART_RxCmd=0;     //δ���������

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
//����ȡFlash������ֵ������
void ReadCfg0(void)
{
	//��ȡ
	Read_Flash(UD.SaveArray, 140);
}
//��ȡFlash����ֵ������
void ReadCfg(void)
{
	//��ȡ
	Read_Flash(UD.SaveArray, 140);
	//��ֵ
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
	
	memcpy(uSCurve,UD.CfgData.SCurve,512);
}
//������д��Flash�洢
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
	
	//void *memcpy(void *dest, const void *src, size_t n);
	memcpy(UD.CfgData.SCurve,uSCurve,512);
	//д��
	Write_Flash(UD.SaveArray, 143);
}
//����ΪĬ��ֵ
void SetDefault(void)
{
	pBTName = "SSFocuser"; //BT SSID

	bIsReverse = false;     //����Ƿ���
	uSubdivision = 8;       //�������ϸ��
	uSpeed  = 2048;		    	//���ת���ٶ�
	//��ʼ����������
	uSCurve[0]=15;
	uSCurve[1]=uSubdivision;
	uSCurve[2]=uSubdivision/2;
	for(i=3;i<255;)//500�������һ��
	{
		uSCurve[i]=(u16)(uSCurve[i-2]*3/2);
		uSCurve[i+1]=(u16)(uSCurve[i]/2);
		i=i+2;
	}
	WriteCfg();
}
//ͨ��ĳ�����ڷ���
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
//��������
void SetBluetooth(void)
{
	char  TemBuff[16] = {0};
	sprintf(TemBuff, "AT+NAME=%s", pBTName);
	BLTUsart_SendString(USART2,TemBuff);
	//���Է��͵�����
	//printf("%s", TemBuff);
}
//�򿪾���
void OpenCover(void)
{
	
	SetSpeedCover(35);
	
	//���Է��͵�����
	printf("Opened");
}
//�رվ���
void CloseCover(void)
{
    
	SetSpeedCover(5);

	printf("Closed");
}
//���õ������
void SetReverse(void)
{
	bIsReverse = !bIsReverse; 
	UD.CfgData.IsReverse=bIsReverse?1:0;
	WriteCfg();
}
//����ϸ��
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
//�����ٶ�
void SetSpeed(u16 uSetSpeed)
{
	SetSpeedMoter(10000/uSetSpeed);
}
//�ӳٺ���
void Delay(u16 dt)//����1mS
{
	u32 i=0;
	for(i=0;i<dt*2500;i++)
	;
}
//��ʼ�����
void InitMotor(void)
{
	bIsMoving = false;
	bIncreCount = true;
	TIM_Cmd(BASIC_TIM, ENABLE);//��ʱ������
	SetDivision(uSubdivision);
	SetSpeed(uSpeed);
}
//����ת��
void SlewOut(void)//:F+#
{
	bIsMoving = true;//�˶�״̬��־
	bIncreCount = true;//��������־
	//��ʼ�ٶȣ����ڼ���
	uSCntCur=0;
	uSpeedCur=uSCurve[uSCntCur*2+1];
	iStepDelt=uSCurve[uSCntCur*2+2];
	SetSpeed(uSpeedCur);
	//PWM����õ͵�ƽ
	GPIO_ResetBits(ADVANCE_TIM_CH1_PORT, ADVANCE_TIM_CH1_PIN); 
	//���õ���˶�����
	if(bIsReverse)
		GPIO_ResetBits(LED2_GPIO_PORT, LED2_GPIO_PIN);  //DRV8825dir�ź�
	else
		GPIO_SetBits(LED2_GPIO_PORT, LED2_GPIO_PIN);    //DRV8825dir�ź�
	//ʹ��DRV8825
	GPIO_ResetBits(LED5_GPIO_PORT, LED5_GPIO_PIN); //DRV8825ʹ���ź�
	Delay(50);
	//ʹ��PWMʱ��
	ControlMotor(ENABLE);
}
//����ת��
void SlewIn(void)//:F-#
{
	bIsMoving = true; //�˶�״̬��־
	bIncreCount = false;//��������־
	//��ʼ�ٶȣ����ڼ���
	uSCntCur=0;
	uSpeedCur=uSCurve[uSCntCur*2+1];
	iStepDelt=uSCurve[uSCntCur*2+2];
	SetSpeed(uSpeedCur);
	//PWM����õ͵�ƽ
	GPIO_ResetBits(ADVANCE_TIM_CH1_PORT, ADVANCE_TIM_CH1_PIN); 
	//���õ���˶�����
	if(bIsReverse)
		GPIO_SetBits(LED2_GPIO_PORT, LED2_GPIO_PIN);    //DRV8825dir�ź�
	else
		GPIO_ResetBits(LED2_GPIO_PORT, LED2_GPIO_PIN);  //DRV8825dir�ź�
	//ʹ��DRV8825
	GPIO_ResetBits(LED5_GPIO_PORT, LED5_GPIO_PIN); //DRV8825ʹ���ź�
	Delay(50);
	//ʹ��PWMʱ��
	ControlMotor(ENABLE);
}
//ת���̶�����
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
//ֹͣ
void Halt(void)
{
	ControlMotor(DISABLE);
	bIsMoving = false;
	uMoveState = 0;
}
//��ͣ
void Pause(void)//���ı��˶�״̬
{
	ControlMotor(DISABLE);
	bIsMoving = false;
}
//���籣������
void SaveData(void)
{
	//ֹͣ�˶�������
	Halt();
	//������������
	WriteCfg();
}
//������һ��
u8 Next(u8 Prv)
{
	if(Prv<255)
		return Prv+1;
	else //255
		return 0;
}
//��ȡһ�����ȥ��ͷβ:#��ָ��ָ����һ������
u8 ReadCmd(unsigned char *RxBuffer,unsigned char *Ptr)
{
	u8 Flag=0,i=0;
	memset(CmdBuff,0,32);
	for(;;)
	{
		//���������жϵ�˳���ܵ���
		if(RxBuffer[*Ptr]==35)//#��������
		{ 
			if(Flag==1)//��������
			{
				CmdBuff[i]=NULL;//û�н�������split���������
				Flag=2;
			}
			else
				Flag=3;
		}
		if(Flag==1)//:F��ſ�ʼ��ȡ�����ַ���
		{
			CmdBuff[i]=RxBuffer[*Ptr];
			i++;
		}
		if(RxBuffer[*Ptr]==58)//:
		{
			Flag=1;
			i=0;
		}
		//ѭ����ȡ���ڻ�����
		*Ptr=Next(*Ptr);
		if(Flag>1)//����ѭ��
			break;
	}
	return i;
}
//�����
bool CmdProcess(u8 MyComPort,unsigned char *RxBuffer,unsigned char *Ptr)
{
	if(ReadCmd(RxBuffer,Ptr)<1)
		return false;
	memset(ReplyBuff,0,128);
	if(CmdBuff[0]=='F')//������������F��ͷ
	{
		switch (CmdBuff[1])
		{
			case '?':  //ACK , return version
				{
					sprintf(ReplyBuff, ":?%s~%s#\r\n", pDevice,pVersion);
					SendTo(MyComPort,ReplyBuff);
					break;
				}
			case '+': 	//Slew Out		 
				{
					sprintf(ReplyBuff, ":+#\r\n");
					SendTo(MyComPort,ReplyBuff);
					Halt();
					uMoveState = 2;
					iStepStart=iStepCount;//��ʼλ�ã����ڼ���
					SlewOut();
					break;
				}
			case '-':   //Slew In
				{
					sprintf(ReplyBuff, ":-#\r\n");
					SendTo(MyComPort,ReplyBuff);
					Halt();
					uMoveState = 2;
					iStepStart=iStepCount;//��ʼλ�ã����ڼ���
					SlewIn();
					break;
				}
			case 'B': //Bluetooth setting  			
				{
					sprintf(ReplyBuff, ":B#\r\n");
					SendTo(MyComPort,ReplyBuff);
					strcpy(pBTName, CmdBuff+2);
					SetBluetooth();
					break;
				}
			case 'C':  //Mirror Cover operation 			
				{
					sprintf(ReplyBuff, ":C#\r\n");
					SendTo(MyComPort,ReplyBuff);
					if (CmdBuff[2] == '1')
						OpenCover();
					else if (CmdBuff[2] == '0')
						CloseCover();
					break;
				}
			case 'c':  //Mirror Cover state 			
				{					
					//OpenCover();
					//Halt();
					uSpeedCover = atoi((char const *)CmdBuff+2);
					SetSpeedCover(uSpeedCover);
					break;
				}
			case 'd':  //Mirror Cover state 			
			{
					CloseCover();
					break;
			}
			case 'D':  //Set to Default			
				{
					sprintf(ReplyBuff, ":D#\r\n");
					Halt();
					SetDefault();
					break;
				}
			case 'G': //Save config infomation
				{
					sprintf(ReplyBuff, ":G#\r\n");
					SendTo(MyComPort,ReplyBuff);
					WriteCfg();
					break;
				}
			case 'g': //Get config infomation
				{
					ReadCfg(); 
					sprintf(ReplyBuff, ":g%s~%s~%d~%d~%d#\r\n",
					pDevice,pVersion,UD.CfgData.IsReverse,UD.CfgData.SubDivision,uSpeed/uSubdivision);//,UD.CfgData.Position,UD.CfgData.CommType);
					SendTo(MyComPort,ReplyBuff);
					break;
				}
			case 'M': //Motor Subdivision
				{
					sprintf(ReplyBuff, ":M#\r\n");
					SendTo(MyComPort,ReplyBuff);
					uSubdivisionCur = atoi((char const *)CmdBuff+2);
					if(uSubdivisionCur!=uSubdivision)//��ͬ������
					{
						Halt();
						switch(uSubdivisionCur)//�ж���Ч��
						{
							case 1:
							case 2:
							case 4:
							case 8:
							case 16:
							case 32:
							{
								//���㵱ǰϸ���µļ���ֵ
								iStepCount = (iStepCount *uSubdivisionCur)/uSubdivision;
								uSubdivision=uSubdivisionCur;
								SetDivision(uSubdivision);
								WriteCfg();	
								break;
							}
						}
					}
					break;
				}
			case 'm': //Get Subdivision		
				{
					sprintf(ReplyBuff,":m%d#\r\n",uSubdivision);
					SendTo(MyComPort,ReplyBuff);
					break;
				}
			case 'P':    //Move command//Moveģʽ��ϸ��
				{
					sprintf(ReplyBuff, ":P#\r\n");
					SendTo(MyComPort,ReplyBuff);
					Halt();
					iStepStart=iStepCount;//��ʼλ�ã����ڼ���
					iStepStop = atoi((char const *)CmdBuff+2)*uSubdivision;
					if(iStepStart!=iStepStop)
					{
						uMoveState=1;
						MoveTo();
					}
					break;
				}
			case 'p':  //Get Position and Moving state
				{
					sprintf(ReplyBuff,":p%d~%d~%d#\r\n", iStepCount/uSubdivision,bIsMoving,bInitial);
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
			case 'R': //Motor Reverse			
				{
					sprintf(ReplyBuff, ":R#\r\n");
					SendTo(MyComPort,ReplyBuff);
					Halt();
					if(bIsReverse != (CmdBuff[2] == '1')?true:false)
					{
						bIsReverse = (CmdBuff[2] == '1')?true:false;
						WriteCfg();	
					}
					break;
				}
			case 'r': //Get Reverse	Status		
				{
					sprintf(ReplyBuff,":r%d#\r\n",bIsReverse);
					SendTo(MyComPort,ReplyBuff);
					break;
				}
			case 'S':    //Define Position
				{
					sprintf(ReplyBuff, ":S#\r\n");
					SendTo(MyComPort,ReplyBuff);
					Halt();
					iStepCount = atoi((char const *)CmdBuff+2)*uSubdivision;
					bInitial=true;
					break;							
				}
			case 't':   //Get temperature
				{
					if(bTempAvail)
						sprintf(ReplyBuff,":t%.2f#\r\n", DS18B20_Get_Temp());//����2λС��
					else
						sprintf(ReplyBuff,":t%.2f#\r\n", 999.99);//�¶ȴ����������÷���999.99
					SendTo(MyComPort,ReplyBuff);
					break;
				}
			case 'V': //Motor Velocity
				{
					sprintf(ReplyBuff, ":V#\r\n");
					SendTo(MyComPort,ReplyBuff);
					uSpeedCur = atoi((char const *)CmdBuff+2);
					if(uSpeedCur!=uSpeed)
					{
						if((uSpeedCur>=uSubdivision)&&(uSpeedCur<=uSubdivision*500))//��֤��С��1Hz,������500Hz
						{
							uSpeed=uSpeedCur*uSubdivision;
							SetSpeed(uSpeed);
						}
					}
					break;
				}
			case 'v': //Get Velocity		
				{
					sprintf(ReplyBuff,":v%d#\r\n",uSpeed/uSubdivision);
					SendTo(MyComPort,ReplyBuff);
					break;
				}
			case 'Y':    //Test delay time
				{
					sprintf(ReplyBuff, ":Y#\r\n");
					SendTo(MyComPort,ReplyBuff);
					uDelayCountMax = atoi((char const *)CmdBuff+2);
					break;
				}
			case 'Z':    //S�������ݣ�
				{
					sprintf(ReplyBuff, ":Z#\r\n");
					SendTo(MyComPort,ReplyBuff);
					if (CmdBuff[2] == 'E')//������־���洢����
						WriteCfg();
					else
					{
						if (CmdBuff[2] == 'B')//��ʼ��־����һ��Ϊ���ݸ���
						{
							Halt();//��ֹͣ�����
							uSCnt=0;
							memset(uSCurve,0,256);
						}
						else if (CmdBuff[2] == 'D')//���ݴ���
							uSCnt++;
						uSCurve[uSCnt]=atoi((char const *)CmdBuff+3);
					}
					break;
				}
			case 'z':    //Test delay time
				{
					int ss=0;
					sprintf(ReplyBuff, ":zTotal:%d---#\r\n",uSCurve[0]);
					SendTo(MyComPort,ReplyBuff);
					for(i=0;i<uSCurve[0];)
					{
						ss=ss+uSCurve[i*2+2];
						sprintf(ReplyBuff, ":z%d,%d,%d#\r\n",uSCurve[i*2+1],uSCurve[i*2+2],ss);
						SendTo(MyComPort,ReplyBuff);
						i++;
					}
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
//������
int main()
{	
	LED_GPIO_Config();

	SysTick_Init();
	
	USART_Config();
	
	BASIC_TIM_Init();
	
	EXTI_44E_Config();
	
	BLT_USART_Config();

	//��ʼ���¶ȴ�����
	if( DS18B20_Init())	
		bTempAvail=false;
	else
	{
		DS18B20_Get_Temp();//��һ�ζ�ȡ���ع̶�ֵ
		bTempAvail=true;
	}
	//��ʼ������
	ReadCfg0();
	if((UD.CfgData.IsReverse>1)||(UD.CfgData.SubDivision<1))//δ��ʼ�����ͳ�ʼ��
	{
		SetDefault();
		//SysTick_Delay_Ms(500);//�������������WiFi����ʧЧ������ʹ��
	}
	ReadCfg();
	//��ʼ�����
	InitMotor();

	Cover_TIM_Init();
	
	while(1)
	{	
		//������1����
		while(UART_RxCmd>0)
		{
			UART_RxCmd--;//���������һ
			CmdProcess(0,UART_RxBuffer,&UART_RxPtr_Prv);
		}	
	
	  //������2����	
	  while(BLTUART_RxCmd>0)
	  { 
			BLTUART_RxCmd--;//���������һ
			CmdProcess(1,BLTUART_RxBuffer,&BLTUART_RxPtr_Prv);
	  }
		
		//����״̬
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
		#if 1
		uDelayCount++;
		if(uDelayCount>=uDelayCountMax)
		{
			sprintf(ReplyBuff,":p%d~%d~%d#\r\n", iStepCount/uSubdivision,bIsMoving,bInitial);
			printf("%s", ReplyBuff);
			BLTUsart_SendString(USART2,ReplyBuff);
			memset(ReplyBuff,0,128);
			uDelayCount=0;
		}
		#endif
	}
}

/*********************************************END OF FILE**********************/
