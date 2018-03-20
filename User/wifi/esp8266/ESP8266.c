#include "ESP8266.h"
#include "WifiUsart.h"
#include "bsp_SysTick.h"
#include "bsp_usart.h"
#include "bsp_led.h"


void ESP8266_Set(unsigned char *puf) // 数组指针*puf指向字符串数组  
{
	  while(*puf!='\0')    //遇到空格跳出循环
      {
           WifiUsart_SendByte(USART3, *puf);
					 Usart_SendByte(USART1, *puf);
           Delay_us(5);
           puf++;      
      }
      Delay_us(5);
			
      WifiUsart_SendByte(USART3, '\r');
	    Usart_SendByte(USART1, '\r');
      Delay_us(5);
      
	    WifiUsart_SendByte(USART3, '\n');
      Usart_SendByte(USART1, '\n');
			
      Delay_ms(2000);
}
void ESP8266_Set2(unsigned char *puf) // 数组指针*puf指向字符串数组  
{
	  while(*puf!='\0')    //遇到空格跳出循环
      {
           WifiUsart_SendByte(USART3, *puf);
					 Usart_SendByte(USART1, *puf);
           Delay_us(5);
           puf++;      
      }
      Delay_ms(2000);
}
void CmdString(unsigned char *pCmdtr)
{
	int i;		
	for(i = 0; i < 3 ;i++)
	{
		Wifiuart_FlushRxBuffer();
		ESP8266_Set(pCmdtr);
		if (strstr((char*)WIFIUART_RxBuffer, "OK"))
			break;
	}
	pCmdtr = NULL;
}
void CmdString2(unsigned char *pCmdtr)//不加\r\n
{
	int i;		
	for(i = 0; i < 3 ;i++)
	{
		Wifiuart_FlushRxBuffer();
		ESP8266_Set2(pCmdtr);
		if (strstr((char*)WIFIUART_RxBuffer, "OK"))
			break;
	}
	pCmdtr = NULL;
}

unsigned char ipstr[100] = {0};
unsigned char Cmdstr[100] = "AT+CIPSTART=\"TCP\",\"192.168.43.78\",8080";

void SetIP(unsigned char *pReIP)
{
	//pReIP = pReIP + 3;
	sprintf ( (char*)ipstr, "\"%s\",\"%s\",%s", "TCP", pReIP, "8080" );
	sprintf ( (char*)Cmdstr, "AT+CIPSTART=%s", ipstr );
	printf("ip is ok!\n");
}

unsigned char NameStr[100] = "\"A305\"";
void SetWifiName(unsigned char *pReName)
{
	//pReName = pReName + 3;
	sprintf((char *)NameStr, "\"%s\"", pReName);
	//printf("set wifi name is ok!\n");
}

unsigned char CodeString[100] = "\"wildfired\"";
void SetWifiCode(unsigned char *pReCode)
{
	//pReCode = pReCode + 3;
	sprintf((char *)CodeString, "\"%s\"", pReCode);
	//printf("set code is ok!\n");
}

unsigned char CmdNameCode[100] = "AT+CWJAP=\"A304\",\"wildfire\"";
void SetNameCode(void)
{
	sprintf((char *)CmdNameCode, "AT+CWJAP=%s,%s", NameStr, CodeString);
	//printf("%s\n", CmdNameCode);
}
//+++,结束透传，重新接受AT指令
	//AT+CWJAP-加入AP
	//AT+CWQAP-退出AP
	//AT+CIPSTART 建立连接
	//AT+CIPCLOSE 关闭连接
	//AT+CIPMODE  透传模式
	//AT+CIPSEND  发送数据
void SetWifiConnect(void)
{ 
	ESP8266_Rst();
	//CmdString("AT"); //测试
	//CmdString("AT+RST"); //测试
	CmdString("AT+CWMODE=1");     //设置路由器模式 1 station模式 2 AP	
	CmdString(CmdNameCode);
	CmdString("AT+CIPMUX=0");//开启多连接模式，允许多个各客户端接入
	CmdString(Cmdstr);
	CmdString("AT+CIPMODE=1");//透传模式	
	CmdString("AT+CIPSEND");//检测是否连接成功
	//#endif
	Wifiuart_FlushRxBuffer();
}
void CloseWifiConnect(void)
{
	CmdString2("+++");
	CmdString("AT+SAVETRANSLINK=0");
	//CmdString("AT+CWQAP");
}

void SetRouter(unsigned char *NameStr,unsigned char *CodeString)
{
	sprintf((char *)CmdNameCode, "AT+CWJAP=\"%s\",\"%s\"", NameStr, CodeString);
}
void SetTCPHost(unsigned char *pReIP,unsigned char *pRePort)
{
	//sprintf ( (char*)ipstr, "\"%s\",\"%s\",%s", "TCP", pReIP, pRePort);
	//sprintf ( (char*)Cmdstr, "AT+CIPSTART=%s", ipstr );
	sprintf ( (char*)Cmdstr, "AT+CIPSTART=\"TCP\",\"%s\",%s", pReIP, pRePort );
}
void SetUDPHost(unsigned char *pReIP,unsigned char *pRePort,unsigned char *pLoPort)
{
	//sprintf ( (char*)ipstr, "\"%s\",\"%s\",%s,%s,0", "UDP", pReIP, pRePort,pLoPort);//本地端口8086
	//sprintf ( (char*)Cmdstr, "AT+CIPSTART=%s", ipstr );
	sprintf ( (char*)Cmdstr, "AT+CIPSTART=\"UDP\",\"%s\",%s,%s,0", pReIP, pRePort,pLoPort);
}

void ESP8266IO(void)
{
	 GPIO_InitTypeDef GPIO_InitStructure;
	  
	 //不能重复开启和禁用
	 //RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE); 
   //  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

     /*开启LED相关的GPIO外设时钟*/
	 RCC_APB2PeriphClockCmd( Wirst_GPIO_CLK | Wien_GPIO_CLK, ENABLE);

     /*选择要控制的GPIO引脚*/
	 GPIO_InitStructure.GPIO_Pin = Wirst_GPIO_PIN;	

	/*设置引脚模式为通用推挽输出*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

	/*设置引脚速率为50MHz */   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

	/*调用库函数，初始化GPIO*/
	GPIO_Init(Wirst_GPIO_PORT, &GPIO_InitStructure);	
		
	/*选择要控制的GPIO引脚*/
	GPIO_InitStructure.GPIO_Pin = Wien_GPIO_PIN;

	/*调用库函数，初始化GPIO*/
	GPIO_Init(Wien_GPIO_PORT, &GPIO_InitStructure);	

  GPIO_SetBits(Wirst_GPIO_PORT, Wirst_GPIO_PIN);		
	GPIO_SetBits(Wien_GPIO_PORT, Wien_GPIO_PIN);	
}


void ESP8266_Rst( void )
{
		macESP8266_RST_LOW_LEVEL();
		Delay_ms ( 500 );
		macESP8266_RST_HIGH_LEVEL();
}

