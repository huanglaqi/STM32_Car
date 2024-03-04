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
	TIM_TimeBaseStructInit(&timInitStruct);			//��Ĭ��ֵ��ʼ��timInitStruct�ṹ����������г�Ա
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

	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);//����rcc�Ľӿں�����tim3����

	//��ʼ��TIM6 -- 1MHZʱ�ӣ���ʱ1ms
	TIM_TimeBaseStructInit(&timInitStruct);//�Խṹ��������г�ʼ������&timInitStruct�����ĵ�ַ���ݸ�����
	
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
	//initialize PA4 -- �������NSS
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
	TIM_TimeBaseStructInit(&timInitStruct);			//��Ĭ��ֵ��ʼ��timInitStruct�ṹ����������г�Ա
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
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);		//ʹ��AFIO

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
	
	timInitStruct.TIM_Prescaler = 71;			//ϵͳ�ṩ72MHzʱ�ӣ�����Ԥ��Ƶ����Ϊ��ʱ��ʱ�ӡ� Prescaler+1 ΪԤ��Ƶֵ, ��Ƶ��ʱ��ʱ��Ϊ1MHz
	timInitStruct.TIM_CounterMode = TIM_CounterMode_Up;		//���ϼ���ģʽ
	timInitStruct.TIM_Period = 999;			//д��ARR��Period+1 Ϊһ����ʱ�����а�����ʱ��������, ��ʱ��ʱ��Ϊ1ms
	timInitStruct.TIM_ClockDivision= TIM_CKD_DIV1;		// can be ignored.
	TIM_TimeBaseInit(TIM3, &timInitStruct);
	
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	
	//initialize TIM4 channel 1 -- mode is PWM1, pulse=500, i.e., ratio is 0.5
	ocInitStruct.TIM_OCMode = TIM_OCMode_PWM1;		//�趨PWMģʽ
	ocInitStruct.TIM_Pulse = 50;				//д��CCR�� �趨ռ�ձ�Ϊ0.5. ���ڶ�ʱ������Ϊ1000��clk����, 0.5*1000=500, 
	ocInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;		//�趨ռ�ձȼ���, 
	ocInitStruct.TIM_OutputState = TIM_OutputState_Enable;	//ʹ�����
	TIM_OC1Init(TIM3, &ocInitStruct);		//��ʼ��OC1, ��ͨ��1

	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);			//ʹ��Ԥװ�ع���, 
	
	//disable TIM4 interrupts, and start TIM4
	TIM_ITConfig(TIM3, TIM_IT_Update|TIM_IT_CC1, DISABLE);////////////////////
	TIM_Cmd(TIM3, ENABLE);		//ʹ��TIM4
	
	//stop channel 1
	TIM_CCxCmd(TIM3, TIM_Channel_1, TIM_CCx_Disable);		//��ֹTIM4��ͨ��1
}


void  MOTOR_Init(void)
{
	GPIO_InitTypeDef  pinInitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);		//ʹ��AFIO

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
	
	timInitStruct.TIM_Prescaler = 71;			//ϵͳ�ṩ72MHzʱ�ӣ�����Ԥ��Ƶ����Ϊ��ʱ��ʱ�ӡ� Prescaler+1 ΪԤ��Ƶֵ, ��Ƶ��ʱ��ʱ��Ϊ1MHz
	timInitStruct.TIM_CounterMode = TIM_CounterMode_Up;		//���ϼ���ģʽ
	timInitStruct.TIM_Period = 999;			//д��ARR��Period+1 Ϊһ����ʱ�����а�����ʱ��������, ��ʱ��ʱ��Ϊ1ms
	timInitStruct.TIM_ClockDivision= TIM_CKD_DIV1;		// can be ignored.
	TIM_TimeBaseInit(TIM3, &timInitStruct);
	
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	
	//initialize TIM4 channel 1 -- mode is PWM1, pulse=500, i.e., ratio is 0.5
	ocInitStruct.TIM_OCMode = TIM_OCMode_PWM1;		//�趨PWMģʽ
	ocInitStruct.TIM_Pulse = 50;				//д��CCR�� �趨ռ�ձ�Ϊ0.5. ���ڶ�ʱ������Ϊ1000��clk����, 0.5*1000=500, 
	ocInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;		//�趨ռ�ձȼ���, 
	ocInitStruct.TIM_OutputState = TIM_OutputState_Enable;	//ʹ�����
	TIM_OC2Init(TIM3, &ocInitStruct);		//��ʼ��OC1, ��ͨ��1

	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);			//ʹ��Ԥװ�ع���, 
	
	//disable TIM4 interrupts, and start TIM4
	TIM_ITConfig(TIM3, TIM_IT_Update|TIM_IT_CC1, DISABLE);////////////////////
	TIM_Cmd(TIM3, ENABLE);		//ʹ��TIM4
	
	//stop channel 1
	TIM_CCxCmd(TIM3, TIM_Channel_2, TIM_CCx_Disable);		//��ֹTIM4��ͨ��1
}




void uart_init(u32 bound){
  //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��
  
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9
   
  //USART1_RX	  GPIOA.10��ʼ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10  

  //Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
   //USART ��ʼ������

	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
  USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ���1 

}

void USART1_IRQHandler(void)                	//����1�жϷ������
	{
	u8 Res;

	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�
		{
			USART_ClearITPendingBit(USART1,USART_IT_RXNE);
		  Res=USART_ReceiveData(USART1);	//��ȡ���յ�������
			USART_SendData(USART1,Res);//�ش����յ�������		
			
			GPIO_SetBits(GPIOA, GPIO_Pin_8);		
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ��������
			delay_ms(100);
			GPIO_SetBits(GPIOA, GPIO_Pin_8);//LED����˸�����ճɹ��������
			
			if(Res=='1')//���յ�1������LED
			{
				speed12(speed/3*2); 
				speed21(speed); 
				
			}
			if(Res=='2')//�������Ϩ��LED
			{
				speed12(speed); 
				speed21(speed/3*2);
			}		
			if(Res=='3')//���յ�1������LED
			{
				speed12(speed); 
				speed21(speed);
			}
			if(Res=='4')//���յ�1������LED
			{
				speed11(speed); 
				speed22(speed);
			}
			if(Res=='5')//���յ�1������LED
			{
				bizha=0;
				GPIO_ResetBits(GPIOB, GPIO_Pin_2);
	      GPIO_ResetBits(GPIOB, GPIO_Pin_1); 
				GPIO_ResetBits(GPIOC, GPIO_Pin_6);
	      GPIO_ResetBits(GPIOC, GPIO_Pin_7); 
				
			}
			if(Res=='6')//���յ�1������LED
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

void TIM7_IRQHandler(void)//�����Ϊ��̬�ֲ��������Ӷ�ʹ�ñ�����������
{
	static  __IO uint32_t  cnt5s=0;
	static uint8_t cnt=0;  
	
	if(TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);//�ж�����
		
		cnt5s++;

		if(cnt5s >= 700)  //5s��ʱ��0.001*5000
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
	TIM_TimeBaseInitTypeDef  timInitStruct;//�����˽ṹ�����
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);//����rcc�Ľӿں�����tim3����

	//��ʼ��TIM6 -- 1MHZʱ�ӣ���ʱ1ms
	TIM_TimeBaseStructInit(&timInitStruct);//�Խṹ��������г�ʼ������&timInitStruct�����ĵ�ַ���ݸ�����
	
	timInitStruct.TIM_Prescaler = 71;		//1MHz
	timInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	timInitStruct.TIM_Period = 999;			//1ms
	timInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM7, &timInitStruct);
	
	TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
	
	TIM_Cmd(TIM7, ENABLE);
}



