#include "WifiUsart.h"


//���ڽ�������
unsigned char WIFIUART_RxBuffer[WIFIUART_RX_BUFFER_SIZE];
//���ڽ�������ָ��
unsigned char WIFIUART_RxPtr;


 /**
   * @brief  ����Ƕ�������жϿ�����NVIC
   * @param  ��
   * @retval ��
   */
 static void NVIC_Configuration(void)
 {
   NVIC_InitTypeDef NVIC_InitStructure;
   
   /* Ƕ�������жϿ�������ѡ�� */
   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
   
   /* ����USARTΪ�ж�Դ */
   NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART3_IRQ;
   /* �������ȼ�*/
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
   /* �����ȼ� */
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
   /* ʹ���ж� */
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   /* ��ʼ������NVIC */
   NVIC_Init(&NVIC_InitStructure);
 }


 /**
  * @brief  USART GPIO ����,������������
  * @param  ��
  * @retval ��
  */
void WifiUSART_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	// �򿪴���GPIO��ʱ��
	DEBUG_USART3_GPIO_APBxClkCmd(DEBUG_USART3_GPIO_CLK, ENABLE);
	
	// �򿪴��������ʱ��
	DEBUG_USART3_APBxClkCmd(DEBUG_USART3_CLK, ENABLE);

	// ��USART Tx��GPIO����Ϊ���츴��ģʽ
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART3_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DEBUG_USART3_TX_GPIO_PORT, &GPIO_InitStructure);

  // ��USART Rx��GPIO����Ϊ��������ģʽ
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART3_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(DEBUG_USART3_RX_GPIO_PORT, &GPIO_InitStructure);
	
	// ���ô��ڵĹ�������
	// ���ò�����
	USART_InitStructure.USART_BaudRate = DEBUG_USART3_BAUDRATE;
	// ���� �������ֳ�
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// ����ֹͣλ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// ����У��λ
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	// ����Ӳ��������
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	// ���ù���ģʽ���շ�һ��
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// ��ɴ��ڵĳ�ʼ������
	USART_Init(DEBUG_USART3x, &USART_InitStructure);
	
	// �����ж����ȼ�����
	NVIC_Configuration();
	
	// ʹ�ܴ��ڽ����ж�
	USART_ITConfig(DEBUG_USART3x, USART_IT_RXNE, ENABLE);	
	
	// ʹ�ܴ���
	USART_Cmd(DEBUG_USART3x, ENABLE);		

  // ���������ɱ�־
	//USART_ClearFlag(USART1, USART_FLAG_TC);     
}


/*****************  ����һ���ַ� **********************/
void WifiUsart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch)
{
	/* ����һ���ֽ����ݵ�USART */
	USART_SendData(pUSARTx,ch);
		
	/* �ȴ��������ݼĴ���Ϊ�� */
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}

/*****************  �����ַ��� **********************/
void WifiUsart_SendString( USART_TypeDef * pUSARTx, char *str)

{
	unsigned int k=0;
  do 
  {
      WifiUsart_SendByte( pUSARTx, *(str + k) );
      k++;
  } while(*(str + k)!='\0');
  
  /* �ȴ�������� */
  while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET)
  {}
}


/*****************  ����һ��16λ�� **********************/
void WifiUsart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch)
{
	uint8_t temp_h, temp_l;
	
	/* ȡ���߰�λ */
	temp_h = (ch&0XFF00)>>8;
	/* ȡ���Ͱ�λ */
	temp_l = ch&0XFF;
	
	/* ���͸߰�λ */
	USART_SendData(pUSARTx,temp_h);	
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
	
	/* ���͵Ͱ�λ */
	USART_SendData(pUSARTx,temp_l);	
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}


//��շ��ͻ���
void Wifiuart_FlushRxBuffer(void)
{
  WIFIUART_RxPtr = 0;
  WIFIUART_RxBuffer[WIFIUART_RxPtr] = 0;
}
