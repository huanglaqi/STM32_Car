#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "lcd.h"
#include "usart.h"
#include "motor.h"
 
/************************************************
 ALIENTEK ս��STM32F103������ʵ��13
 TFTLCD��ʾʵ��  
 ����֧�֣�www.openedv.com
 �Ա����̣�http://eboard.taobao.com 
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 ������������ӿƼ����޹�˾  
 ���ߣ�����ԭ�� @ALIENTEK
************************************************/
 	
__IO float  bizha=0;
 __IO uint16_t  speed = 800;

void  myNVIC_Init(void)
{
	NVIC_InitTypeDef NVICinit;
	
	//���ȼ�����ģʽ -- 2λռ��ʽ, 2λ�����ȼ�
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	//Usart1 �ж�, ���ȼ���
  NVICinit.NVIC_IRQChannel = TIM7_IRQn;
	NVICinit.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	NVICinit.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVICinit.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVICinit);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

}


 int main(void)
 {	 
 	u8 x=0;
	u8 lcd_id[12];			//���LCD ID�ַ���
	
  __IO uint16_t  distance1,distance2;
  

  __IO uint16_t  cntVal=0;
	u8 su1 =0;
	u8 su2 =0;
	
	
  Sonarspeed_Init();
	Sonar_Init();
	
	MOTOR_Init();   
  PWM_Init();	
	MOTOR_Init1();   
  PWM_Init1();		
	
	TIM_CCxCmd(TIM3, TIM_Channel_1, TIM_CCx_Enable);
	TIM_CCxCmd(TIM3, TIM_Channel_2, TIM_CCx_Enable);
	GPIO_ResetBits(GPIOB, GPIO_Pin_2);
	GPIO_ResetBits(GPIOB, GPIO_Pin_1);  
	 delay_init();	    	 //��ʱ������ʼ��	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(9600);	 	//���ڳ�ʼ��Ϊ115200
 	//LED_Init();			     //LED�˿ڳ�ʼ��
	LCD_Init();
	POINT_COLOR=RED;
	myTIM_init();
	myNVIC_Init(); 
	Max7219_Init();
	
  	while(1) 
	{		 

				if(bizha==1)
		{
	  
			distance1 = measu1();
		  distance2=measu2();
    //delay_ms(1000);
			if((distance1>1765)&&(distance2>1765))
			{
			  if(distance1>distance2)
			  {
				  if((distance1-distance2)<30)
				  {
					speed11(400);
					speed21(400);
					delay_ms(350);
				  }
			  }
			  else
			  {
				  if((distance2-distance1)<30)
				  {
					speed12(400);
					speed22(400);
					delay_ms(350);
		
				  }
			  }
		  }
			biz1(distance1);
			biz2(distance2);
			delay_ms(10);
		
	   }
		

		
	  su1=cesu1();
		su2=cesu2();
		
		POINT_COLOR=BLACK;	  
		LCD_Clear(WHITE);
    delay_ms(10);			 
		LCD_ShowString(30,40,210,24,24,"WarShip STM32 ^_^"); 
		LCD_ShowString(30,70,200,16,16,"2020/12/21");
		 
		sprintf((char*)lcd_id,"speed:%02u cm/s",su2);//��LCD ID��ӡ��lcd_id���顣			 
		LCD_ShowString(30,90,200,16,16,lcd_id);
		sprintf((char*)lcd_id,"speed:%02u cm/s",su1);//��LCD ID��ӡ��lcd_id���顣			
	  LCD_ShowString(30,130,200,16,16,lcd_id);		//��ʾLCD ID
		
	
	    x++;
	 
		if(x==12)x=0;
		   		 
		delay_ms(10);	

	} 
}
