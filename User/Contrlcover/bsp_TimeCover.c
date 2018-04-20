#include "bsp_TimeCover.h" 
#include "bsp_usart.h"


static void ADVANCE_TIM_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO, ENABLE);
	
	//GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);
	
	// 输出比较通道 GPIO 初始化
	RCC_APB2PeriphClockCmd(COVER_TIM_CH2_GPIO_CLK, ENABLE);
	GPIO_InitStructure.GPIO_Pin = COVER_TIM_CH2_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(COVER_TIM_CH2_PORT, &GPIO_InitStructure);
}

#if 0
// 中断优先级配置
static void COVER_TIM_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 
    // 设置中断组为0
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);		
		// 设置中断来源
    NVIC_InitStructure.NVIC_IRQChannel = COVER_TIM_IRQ ;	
		// 设置主优先级为 0
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;	 
	  // 设置抢占优先级为6
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
#endif

static void COVER_TIM_Mode_Config(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;   //时基结构体
    TIM_OCInitTypeDef  TIM_OCInitStructure;           //输出比较结构体初始化
		
    // 开启定时器时钟,即内部时钟CK_INT=72M
    COVER_TIM_APBxClock_FUN(COVER_TIM_CLK, ENABLE);
	
    // 自动重装载寄存器的值，累计TIM_Period+1个频率后产生一个更新或者中断
    TIM_TimeBaseStructure.TIM_Period = COVER_TIM_Period;	

	// 时钟预分频数为
    TIM_TimeBaseStructure.TIM_Prescaler = COVER_TIM_Prescaler;
	
    // 时钟分频因子 ，基本定时器没有，不用管
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
		
	// 计数器计数模式，基本定时器只能向上计数，没有计数模式的设置
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
			
	// 初始化定时器
    TIM_TimeBaseInit(COVER_TIM, &TIM_TimeBaseStructure);


	/*--------------------输出比较结构体初始化-------------------*/	
	// 配置为PWM模式2
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	// 输出使能
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
	// 设置占空比大小
	TIM_OCInitStructure.TIM_Pulse = COVER_TIM_Period/2;
	// 输出通道电平极性配置
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	// 输出通道空闲电平极性配置
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;

	TIM_OC2Init(TIM2, &TIM_OCInitStructure);

	
	//使能TIM1_CH2预装载寄存器
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	//使能TIM2预装载寄存器
    TIM_ARRPreloadConfig(TIM2, ENABLE); 
	//设置中断源，只有溢出时才中断
    TIM_UpdateRequestConfig(TIM2, TIM_UpdateSource_Regular);
		
	// 清除计数器中断标志位
    TIM_ClearFlag(COVER_TIM, TIM_FLAG_Update);
	  
	// 开启计数器中断
    TIM_ITConfig(COVER_TIM,TIM_IT_Update,ENABLE);
		
	// 使能计数器
    TIM_Cmd(COVER_TIM, ENABLE);	
	  TIM_CtrlPWMOutputs(TIM2, ENABLE);
}


	
void Cover_TIM_Init(void)
{
	ADVANCE_TIM_GPIO_Config();	
	COVER_TIM_Mode_Config();
	//COVER_TIM_NVIC_Config();
	 //Timer2_Configuration();
}



void SetSpeedCover(u16 uSpeedCoverOne)
{
	
    TIM_SetCompare2(COVER_TIM, uSpeedCoverOne);
	
}

#if 0
void Timer2_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    TIM_TimeBaseStructure.TIM_Period = 1000-1;             
    TIM_TimeBaseStructure.TIM_Prescaler = 72-1;            
    TIM_TimeBaseStructure.TIM_ClockDivision = 0x00;    
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    
    TIM_ARRPreloadConfig(TIM2, DISABLE);
    TIM_ClearITPendingBit(TIM2,  TIM_IT_Update); 
    TIM_ITConfig(TIM2,  TIM_IT_Update, ENABLE);
    
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
    TIM_OCInitStructure.TIM_Pulse = 500-1;     
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OC2Init(TIM2, &TIM_OCInitStructure); 
    
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);  
    TIM_CtrlPWMOutputs(TIM2,ENABLE);       
    TIM_Cmd(TIM2, ENABLE);
}
#endif
