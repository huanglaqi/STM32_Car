#include "stm32f10x.h"
#include "motor.h"

void  Sonarspeed_Init(void)
{
	GPIO_InitTypeDef  pinInitStruct;
	TIM_TimeBaseInitTypeDef  timInitStruct;
	TIM_ICInitTypeDef  icInitStruct;
	GPIO_InitTypeDef  GPIOInitStruct;
	SPI_InitTypeDef   SPIInitStruct;
	
	//Initialize PB6(IN_FLOATING), PE0(IN_FLOATING), PE1(OUT_PP)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOE, ENABLE);
	
	//Initialize PB6 -- IN_FLOATING
	pinInitStruct.GPIO_Pin = GPIO_Pin_1;
	pinInitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	pinInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &pinInitStruct);
	
	pinInitStruct.GPIO_Pin = GPIO_Pin_2;
	pinInitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	pinInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &pinInitStruct);

	//Initialize TIM4 -- count-up mode, 1MHz
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	TIM_TimeBaseStructInit(&timInitStruct);			//以默认值初始化timInitStruct结构体变量中所有成员
	//Initialize TIM4 -- count up Mode
	timInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	timInitStruct.TIM_Prescaler = 7199;
	timInitStruct.TIM_Period = 65535;
	TIM_TimeBaseInit(TIM2, &timInitStruct);
	
	//config external trigger -- rising_edge
	//TIM_ETRConfig(TIM2, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);
	
	//select external trigger source -- ETRF
	//TIM_SelectInputTrigger(TIM4, TIM_TS_ETRF);
	
	//select slave mode -- trigger
  //TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Trigger);
	
	
	// initialize Input Capture Mode
	icInitStruct.TIM_Channel = TIM_Channel_2;
	icInitStruct.TIM_ICFilter = 0;
	icInitStruct.TIM_ICPolarity = TIM_ICPolarity_Falling;
	icInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	icInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM2, &icInitStruct);
	icInitStruct.TIM_Channel = TIM_Channel_3;
	icInitStruct.TIM_ICFilter = 0;
	icInitStruct.TIM_ICPolarity = TIM_ICPolarity_Falling;
	icInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	icInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM2, &icInitStruct);
	
	//clear CC1 interrupt flag, stop TIM4
	TIM_ClearFlag(TIM2, TIM_FLAG_CC2|TIM_FLAG_Update);
	TIM_ClearFlag(TIM2, TIM_FLAG_CC3|TIM_FLAG_Update);
	TIM2->CNT =0;
	TIM_Cmd(TIM2, DISABLE);
	//TIM7

	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);//调用rcc的接口函数把tim3开放

	//初始化TIM6 -- 1MHZ时钟，定时1ms
	TIM_TimeBaseStructInit(&timInitStruct);//对结构体变量进行初始化，把&timInitStruct变量的地址传递给函数
	
	timInitStruct.TIM_Prescaler = 71;		//1MHz
	timInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	timInitStruct.TIM_Period = 999;			//1ms
	timInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM7, &timInitStruct);
	
	TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
	
	TIM_Cmd(TIM7, ENABLE);
	//SPI2

	
	//enable GPIOA and SPI1 clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	//initialize PA4 -- 软件控制NSS
	GPIOInitStruct.GPIO_Pin = GPIO_Pin_12;
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIOInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIOInitStruct);
	NSSSet();		
	
	//initialize PA5~7
	GPIOInitStruct.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIOInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIOInitStruct);
	
	//initialize SPI1, 
	SPIInitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPIInitStruct.SPI_CPOL = SPI_CPOL_Low;
	SPIInitStruct.SPI_CPHA = SPI_CPHA_1Edge;
	SPIInitStruct.SPI_DataSize=SPI_DataSize_16b;
	SPIInitStruct.SPI_Direction=SPI_Direction_2Lines_FullDuplex;
	SPIInitStruct.SPI_FirstBit=	SPI_FirstBit_MSB;
	SPIInitStruct.SPI_Mode=SPI_Mode_Master;
	SPIInitStruct.SPI_NSS=SPI_NSS_Soft;
	SPIInitStruct.SPI_CRCPolynomial=7;
	SPI_Init(SPI2, &SPIInitStruct);
	
	//enable SPI1
	SPI_Cmd(SPI2, ENABLE);
	
}


void  Sonar_Init(void)
{
	GPIO_InitTypeDef  pinInitStruct;
	TIM_TimeBaseInitTypeDef  timInitStruct;
	TIM_ICInitTypeDef  icInitStruct;
	
	//Initialize PB6(IN_FLOATING), PE0(IN_FLOATING), PE1(OUT_PP)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOE, ENABLE);
	
	//Initialize PB6 -- IN_FLOATING
	pinInitStruct.GPIO_Pin = GPIO_Pin_6;
	pinInitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	pinInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &pinInitStruct);
	
	//Initialize PE0 -- IN_FLOATING
	pinInitStruct.GPIO_Pin = GPIO_Pin_7;
	pinInitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	pinInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &pinInitStruct);
	
	//Initialize PE1 -- OUT_PP
	pinInitStruct.GPIO_Pin = GPIO_Pin_1;
	pinInitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	pinInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &pinInitStruct);
	
	GPIO_ResetBits(GPIOE, GPIO_Pin_1);
	pinInitStruct.GPIO_Pin = GPIO_Pin_0;
	pinInitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	pinInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &pinInitStruct);
	
	GPIO_ResetBits(GPIOE, GPIO_Pin_0);
	
	//Initialize TIM4 -- count-up mode, 1MHz
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	TIM_TimeBaseStructInit(&timInitStruct);			//以默认值初始化timInitStruct结构体变量中所有成员
	//Initialize TIM4 -- count up Mode
	timInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	timInitStruct.TIM_Prescaler = 71;
	timInitStruct.TIM_Period = 65535;
	TIM_TimeBaseInit(TIM4, &timInitStruct);
	
	//config external trigger -- rising_edge
	TIM_ETRConfig(TIM4, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);
	
	//select external trigger source -- ETRF
	TIM_SelectInputTrigger(TIM4, TIM_TS_ETRF);
	
	//select slave mode -- trigger
	TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Trigger);
	
	
	// initialize Input Capture Mode
	icInitStruct.TIM_Channel = TIM_Channel_1;
	icInitStruct.TIM_ICFilter = 0;
	icInitStruct.TIM_ICPolarity = TIM_ICPolarity_Falling;
	icInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	icInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM4, &icInitStruct);
	
	icInitStruct.TIM_Channel = TIM_Channel_2;
	icInitStruct.TIM_ICFilter = 0;
	icInitStruct.TIM_ICPolarity = TIM_ICPolarity_Falling;
	icInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	icInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM4, &icInitStruct);
	
	//clear CC1 interrupt flag, stop TIM4
	TIM_ClearFlag(TIM4, TIM_FLAG_CC1|TIM_FLAG_Update);
	TIM_ClearFlag(TIM4, TIM_FLAG_CC2|TIM_FLAG_Update);
	TIM4->CNT =0;
	TIM_Cmd(TIM4, DISABLE);
}


void  Sonar_trig(void)
{
	GPIO_SetBits(GPIOE, GPIO_Pin_1);
	delay_us(15);
	GPIO_ResetBits(GPIOE, GPIO_Pin_1);

}
void  Sonar_trig1(void)
{
	GPIO_SetBits(GPIOE, GPIO_Pin_0);
	delay_us(15);
	GPIO_ResetBits(GPIOE, GPIO_Pin_0);

}

void  MOTOR_Init1(void)
{
	GPIO_InitTypeDef  pinInitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);		//使能AFIO

	//GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	pinInitStruct.GPIO_Pin = GPIO_Pin_1;
	pinInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	pinInitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &pinInitStruct);

	pinInitStruct.GPIO_Pin = GPIO_Pin_2;
	pinInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	pinInitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &pinInitStruct);

	pinInitStruct.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	pinInitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	pinInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &pinInitStruct);
	
	//reset all pins
	GPIO_ResetBits(GPIOC, GPIO_Pin_6|GPIO_Pin_7);
	
	//initialize PWM signal and disable it
	//PWM_Init();
	
}

void PWM_Init1(void)
{
	GPIO_InitTypeDef   pinInitStruct;
	TIM_TimeBaseInitTypeDef  timInitStruct;
	TIM_OCInitTypeDef  ocInitStruct;
	
	//enable GPIOB clock, initialize PB6
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
	pinInitStruct.GPIO_Pin = GPIO_Pin_6;
	pinInitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	pinInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &pinInitStruct);
	
	//initialize TIM4 -- PSC=71, count-up mode, period = 999, i.e., 1ms
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	TIM_TimeBaseStructInit(&timInitStruct);		//initialize all members with its default value
	
	timInitStruct.TIM_Prescaler = 71;			//系统提供72MHz时钟，经过预分频后作为定时器时钟。 Prescaler+1 为预分频值, 分频后定时器时钟为1MHz
	timInitStruct.TIM_CounterMode = TIM_CounterMode_Up;		//向上计数模式
	timInitStruct.TIM_Period = 999;			//写入ARR，Period+1 为一个定时周期中包含的时钟周期数, 定时器时间为1ms
	timInitStruct.TIM_ClockDivision= TIM_CKD_DIV1;		// can be ignored.
	TIM_TimeBaseInit(TIM3, &timInitStruct);
	
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	
	//initialize TIM4 channel 1 -- mode is PWM1, pulse=500, i.e., ratio is 0.5
	ocInitStruct.TIM_OCMode = TIM_OCMode_PWM1;		//设定PWM模式
	ocInitStruct.TIM_Pulse = 50;				//写入CCR， 设定占空比为0.5. 由于定时器周期为1000个clk周期, 0.5*1000=500, 
	ocInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;		//设定占空比极性, 
	ocInitStruct.TIM_OutputState = TIM_OutputState_Enable;	//使能输出
	TIM_OC1Init(TIM3, &ocInitStruct);		//初始化OC1, 即通道1

	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);			//使能预装载功能, 
	
	//disable TIM4 interrupts, and start TIM4
	TIM_ITConfig(TIM3, TIM_IT_Update|TIM_IT_CC1, DISABLE);////////////////////
	TIM_Cmd(TIM3, ENABLE);		//使能TIM4
	
	//stop channel 1
	TIM_CCxCmd(TIM3, TIM_Channel_1, TIM_CCx_Disable);		//禁止TIM4的通道1
}


void  MOTOR_Init(void)
{
	GPIO_InitTypeDef  pinInitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);		//使能AFIO

	//GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	pinInitStruct.GPIO_Pin = GPIO_Pin_1;
	pinInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	pinInitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &pinInitStruct);

	pinInitStruct.GPIO_Pin = GPIO_Pin_2;
	pinInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	pinInitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &pinInitStruct);

	pinInitStruct.GPIO_Pin = MOTOR_Pin_IN1|MOTOR_Pin_IN2;
	pinInitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	pinInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MOTOR_Port, &pinInitStruct);
	
	//reset all pins
	GPIO_ResetBits(MOTOR_Port, MOTOR_Pin_IN1|MOTOR_Pin_IN2);
	
	//initialize PWM signal and disable it
	//PWM_Init();
	
}

void PWM_Init(void)
{
	GPIO_InitTypeDef   pinInitStruct;
	TIM_TimeBaseInitTypeDef  timInitStruct;
	TIM_OCInitTypeDef  ocInitStruct;
	
	//enable GPIOB clock, initialize PB6
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
	pinInitStruct.GPIO_Pin = GPIO_Pin_7;
	pinInitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	pinInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &pinInitStruct);
	
	//initialize TIM4 -- PSC=71, count-up mode, period = 999, i.e., 1ms
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	TIM_TimeBaseStructInit(&timInitStruct);		//initialize all members with its default value
	
	timInitStruct.TIM_Prescaler = 71;			//系统提供72MHz时钟，经过预分频后作为定时器时钟。 Prescaler+1 为预分频值, 分频后定时器时钟为1MHz
	timInitStruct.TIM_CounterMode = TIM_CounterMode_Up;		//向上计数模式
	timInitStruct.TIM_Period = 999;			//写入ARR，Period+1 为一个定时周期中包含的时钟周期数, 定时器时间为1ms
	timInitStruct.TIM_ClockDivision= TIM_CKD_DIV1;		// can be ignored.
	TIM_TimeBaseInit(TIM3, &timInitStruct);
	
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	
	//initialize TIM4 channel 1 -- mode is PWM1, pulse=500, i.e., ratio is 0.5
	ocInitStruct.TIM_OCMode = TIM_OCMode_PWM1;		//设定PWM模式
	ocInitStruct.TIM_Pulse = 50;				//写入CCR， 设定占空比为0.5. 由于定时器周期为1000个clk周期, 0.5*1000=500, 
	ocInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;		//设定占空比极性, 
	ocInitStruct.TIM_OutputState = TIM_OutputState_Enable;	//使能输出
	TIM_OC2Init(TIM3, &ocInitStruct);		//初始化OC1, 即通道1

	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);			//使能预装载功能, 
	
	//disable TIM4 interrupts, and start TIM4
	TIM_ITConfig(TIM3, TIM_IT_Update|TIM_IT_CC1, DISABLE);////////////////////
	TIM_Cmd(TIM3, ENABLE);		//使能TIM4
	
	//stop channel 1
	TIM_CCxCmd(TIM3, TIM_Channel_2, TIM_CCx_Disable);		//禁止TIM4的通道1
}




void uart_init(u32 bound){
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
  
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
   
  //USART1_RX	  GPIOA.10初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  

  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
   //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART1, &USART_InitStructure); //初始化串口1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART1, ENABLE);                    //使能串口1 

}

void USART1_IRQHandler(void)                	//串口1中断服务程序
	{
	u8 Res;

	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断
		{
			USART_ClearITPendingBit(USART1,USART_IT_RXNE);
		  Res=USART_ReceiveData(USART1);	//读取接收到的数据
			USART_SendData(USART1,Res);//回传接收到的数据		
			
			GPIO_SetBits(GPIOA, GPIO_Pin_8);		
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送完毕
			delay_ms(100);
			GPIO_SetBits(GPIOA, GPIO_Pin_8);//LED灯闪烁，接收成功发送完成
			
			if(Res=='1')//接收到1，点亮LED
			{
				speed12(speed/3*2); 
				speed21(speed); 
				
			}
			if(Res=='2')//其他情况熄灭LED
			{
				speed12(speed); 
				speed21(speed/3*2);
			}		
			if(Res=='3')//接收到1，点亮LED
			{
				speed12(speed); 
				speed21(speed);
			}
			if(Res=='4')//接收到1，点亮LED
			{
				speed11(speed); 
				speed22(speed);
			}
			if(Res=='5')//接收到1，点亮LED
			{
				bizha=0;
				GPIO_ResetBits(GPIOB, GPIO_Pin_2);
	      GPIO_ResetBits(GPIOB, GPIO_Pin_1); 
				GPIO_ResetBits(GPIOC, GPIO_Pin_6);
	      GPIO_ResetBits(GPIOC, GPIO_Pin_7); 
				
			}
			if(Res=='6')//接收到1，点亮LED
			{
				if(0==bizha)
				{
				bizha = 1;
				USART_SendData(USART1,bizha);
				}
				else
				{
					bizha=0;
					USART_SendData(USART1,bizha);
				}
			}
			//USART_RX_STA=0;
		
    }
} 
	



void speed11(__IO uint16_t ess)
{
	GPIO_SetBits(GPIOB, GPIO_Pin_1);
	GPIO_ResetBits(GPIOB, GPIO_Pin_2);
	TIM3->CCR2=(ess);
	
}
void speed12(__IO uint16_t ess)
{
	GPIO_ResetBits(GPIOB, GPIO_Pin_1);
	GPIO_SetBits(GPIOB, GPIO_Pin_2);
	TIM3->CCR2=(ess);
	
}
void speed21(__IO uint16_t ess)
{
	GPIO_SetBits(GPIOC, GPIO_Pin_6);
	GPIO_ResetBits(GPIOC, GPIO_Pin_7);
	TIM3->CCR1=(ess);
	
}
void speed22(__IO uint16_t ess)
{
	GPIO_ResetBits(GPIOC, GPIO_Pin_6);
	GPIO_SetBits(GPIOC, GPIO_Pin_7);
	TIM3->CCR1=(ess);
	
}

uint16_t measu1(void)
{
	__IO uint16_t  cntVal=0;
	__IO float distance =0;
	TIM_Cmd(TIM4, ENABLE);
		TIM_OC2PolarityConfig(TIM4,TIM_ICPolarity_Rising); 
    Sonar_trig();
		
		while(TIM_GetFlagStatus(TIM4, TIM_FLAG_CC2)==RESET);
		
		cntVal = TIM4->CCR2;
		TIM_OC2PolarityConfig(TIM4,TIM_ICPolarity_Falling);
    while(TIM_GetFlagStatus(TIM4, TIM_FLAG_CC2)==RESET);
		distance = TIM4->CCR2;
		distance = distance-cntVal;
		
		
		TIM_Cmd(TIM4, DISABLE);
		TIM4->CNT = 0;
		USART_SendData(USART1,(distance/58.82));
	return distance;
}
 
uint16_t measu2(void)
{
	__IO uint16_t  cntVal=0;
	__IO float distance =0;
	
	  TIM_Cmd(TIM4, ENABLE);
		TIM_OC1PolarityConfig(TIM4,TIM_ICPolarity_Rising); 
    Sonar_trig1();
		
		while(TIM_GetFlagStatus(TIM4, TIM_FLAG_CC1)==RESET);
		
		cntVal = TIM4->CCR1;
		TIM_OC1PolarityConfig(TIM4,TIM_ICPolarity_Falling);
    while(TIM_GetFlagStatus(TIM4, TIM_FLAG_CC1)==RESET);
		distance = TIM4->CCR1;
		distance = distance-cntVal;
		//distance = distance/58.82;
		
		TIM_Cmd(TIM4, DISABLE);
		TIM4->CNT = 0;
		USART_SendData(USART1,(distance/58.82));
	
	return distance;
}
uint16_t cesu1(void)
{
	
	 __IO uint16_t  cntVal=0;
	 u8 su =0;
		 
	  TIM_Cmd(TIM2, ENABLE);
		TIM_OC2PolarityConfig(TIM2,TIM_ICPolarity_Rising); 
    //Sonar_trig();
		
		while(TIM_GetFlagStatus(TIM2, TIM_FLAG_CC2)==RESET);
		
		cntVal = TIM2->CCR2;
    TIM_OC2PolarityConfig(TIM4,TIM_ICPolarity_Falling);
    while(TIM_GetFlagStatus(TIM2, TIM_FLAG_CC2)==RESET);
		su = TIM2->CCR2;
		su = su-cntVal;
		su = 109900/su;
	  USART_SendData(USART1,su);
		 
		 
		TIM_Cmd(TIM2, DISABLE);
		TIM2->CNT = 0;
		
		return su;
		
}
uint16_t cesu2(void)
{
	 __IO uint16_t  cntVal=0;
	 u8 su =0;
	
		TIM_Cmd(TIM2, ENABLE);
		TIM_OC3PolarityConfig(TIM2,TIM_ICPolarity_Rising); 
    //Sonar_trig();
		
		while(TIM_GetFlagStatus(TIM2, TIM_FLAG_CC3)==RESET);
		
		cntVal = TIM2->CCR3;

    while(TIM_GetFlagStatus(TIM2, TIM_FLAG_CC3)==RESET);
		su = TIM2->CCR3;
		su = su-cntVal;
		su = 10990/su;
	
		TIM_Cmd(TIM2, DISABLE);
		TIM2->CNT = 0;
		USART_SendData(USART1,su);
	
	  return su;
		
}


void biz1(__IO uint16_t ees)
{
	if((ees<2353)&&(ees>2059))
			{
				
			 speed12((400/(2353-2059)*(ees-2059)+400));
			
				
				
			}
			if((ees<=2059)&&(ees>1765))
			{
			  
					speed11((400/(2059-1765)*(2059-ees)+400));
					
				
			}
			if(ees<=1765)
			{
			 speed11(400);
			}
			if(ees>=2353)
			{
			 speed12(speed);
			}
}
 
void biz2(__IO uint16_t ees)
{
	
			if((ees<2353)&&(ees>2059))
			{
				
			    speed21((400/(2353-2059)*(ees-2059)+400));
					
				
			}
			if((ees<=2059)&&(ees>1765))
			{
			 
					speed22((400/(2059-1765)*(2059-ees)+400));
				
				
			}
			if(ees<=1765)
			{
			 speed22(400);
			}
			if(ees>=2353)
			{
			 speed21(speed);
			}
}
//SPI
void  SPI_Led_WrWord(uint16_t  num)
{
	NSSReset();	
	SPI_I2S_SendData(SPI2, num);
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY)==SET);
	NSSSet();

}


void Max7219_Init(void)
{

	SPI_Led_WrWord(Intensity);
	SPI_Led_WrWord(ScanAll);
	SPI_Led_WrWord(NoDecode);
		
	SPI_Led_WrWord(Digit(0,0x0));
	SPI_Led_WrWord(Digit(1,0x0));
	SPI_Led_WrWord(Digit(2,0x0));
	SPI_Led_WrWord(Digit(3,0x0));
	SPI_Led_WrWord(Digit(4,0x0));
	SPI_Led_WrWord(Digit(5,0x0));
	SPI_Led_WrWord(Digit(6,0x0));
	SPI_Led_WrWord(Digit(7,0x0));
	
	SPI_Led_WrWord(TurnOn);

}

void TIM7_IRQHandler(void)//定义成为静态局部变量，从而使得变量不被销毁
{
	static  __IO uint32_t  cnt5s=0;
	static uint8_t cnt=0;  
	
	if(TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);//中断清零
		
		cnt5s++;

		if(cnt5s >= 700)  //5s定时到0.001*5000
		{
			cnt5s = 0;
			cnt++;
			if(cnt==1)
			{
				SPI_Led_WrWord(Digit(0,0x00));
				SPI_Led_WrWord(Digit(1,0x7e));
				SPI_Led_WrWord(Digit(2,0x18));
				SPI_Led_WrWord(Digit(3,0x18));
				SPI_Led_WrWord(Digit(4,0x18));
				SPI_Led_WrWord(Digit(5,0x18));
				SPI_Led_WrWord(Digit(6,0x18));
				SPI_Led_WrWord(Digit(7,0x7e));//i
			}
			else if(cnt==2)
			{
				SPI_Led_WrWord(Digit(0,0x00));
				SPI_Led_WrWord(Digit(1,0x66));
				SPI_Led_WrWord(Digit(2,0xff));
				SPI_Led_WrWord(Digit(3,0xff));
				SPI_Led_WrWord(Digit(4,0xff));
				SPI_Led_WrWord(Digit(5,0x7e));
				SPI_Led_WrWord(Digit(6,0x3c));
				SPI_Led_WrWord(Digit(7,0x18));
			}
			else if(cnt==3)
			{
					SPI_Led_WrWord(Digit(0,0x00));
					SPI_Led_WrWord(Digit(1,0x3c));
					SPI_Led_WrWord(Digit(2,0x66));
					SPI_Led_WrWord(Digit(3,0x60));
					SPI_Led_WrWord(Digit(4,0x60));
					SPI_Led_WrWord(Digit(5,0x60));
					SPI_Led_WrWord(Digit(6,0x66));
					SPI_Led_WrWord(Digit(7,0x3c));
	
			}
			else if(cnt==4)
			{	SPI_Led_WrWord(Digit(0,0x00));
				SPI_Led_WrWord(Digit(1,0x3c));
				SPI_Led_WrWord(Digit(2,0x66));
				SPI_Led_WrWord(Digit(3,0x66));
				SPI_Led_WrWord(Digit(4,0x66));
				SPI_Led_WrWord(Digit(5,0x66));
				SPI_Led_WrWord(Digit(6,0x3f));
				SPI_Led_WrWord(Digit(7,0x03));
			}
			else if(cnt==5)
			{
				SPI_Led_WrWord(Digit(0,0x00));
				SPI_Led_WrWord(Digit(1,0x66));
				SPI_Led_WrWord(Digit(2,0x66));
				SPI_Led_WrWord(Digit(3,0x66));
				SPI_Led_WrWord(Digit(4,0x66));
				SPI_Led_WrWord(Digit(5,0x66));
				SPI_Led_WrWord(Digit(6,0x3c));
				SPI_Led_WrWord(Digit(7,0x18));
			}
			else
			{
				if(cnt>5)
				{
					cnt=0;
				}
			}
		}
	}
}

void  myTIM_init(void)
{
	TIM_TimeBaseInitTypeDef  timInitStruct;//定义了结构体变量
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);//调用rcc的接口函数把tim3开放

	//初始化TIM6 -- 1MHZ时钟，定时1ms
	TIM_TimeBaseStructInit(&timInitStruct);//对结构体变量进行初始化，把&timInitStruct变量的地址传递给函数
	
	timInitStruct.TIM_Prescaler = 71;		//1MHz
	timInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	timInitStruct.TIM_Period = 999;			//1ms
	timInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM7, &timInitStruct);
	
	TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
	
	TIM_Cmd(TIM7, ENABLE);
}



