#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "lcd.h"
#include "usart.h"
#include "motor.h"
 
/************************************************
 ALIENTEK 战舰STM32F103开发板实验13
 TFTLCD显示实验  
 技术支持：www.openedv.com
 淘宝店铺：http://eboard.taobao.com 
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司  
 作者：正点原子 @ALIENTEK
************************************************/
 	
__IO float  bizha=0;
 __IO uint16_t  speed = 800;

void  myNVIC_Init(void)
{
	NVIC_InitTypeDef NVICinit;
	
	//优先级管理模式 -- 2位占先式, 2位子优先级
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	//Usart1 中断, 优先级低
  NVICinit.NVIC_IRQChannel = TIM7_IRQn;
	NVICinit.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVICinit.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVICinit.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVICinit);	//根据指定的参数初始化VIC寄存器

}


 int main(void)
 {	 
 	u8 x=0;
	u8 lcd_id[12];			//存放LCD ID字符串
	
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
	 delay_init();	    	 //延时函数初始化	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(9600);	 	//串口初始化为115200
 	//LED_Init();			     //LED端口初始化
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
		 
		sprintf((char*)lcd_id,"speed:%02u cm/s",su2);//将LCD ID打印到lcd_id数组。			 
		LCD_ShowString(30,90,200,16,16,lcd_id);
		sprintf((char*)lcd_id,"speed:%02u cm/s",su1);//将LCD ID打印到lcd_id数组。			
	  LCD_ShowString(30,130,200,16,16,lcd_id);		//显示LCD ID
		
	
	    x++;
	 
		if(x==12)x=0;
		   		 
		delay_ms(10);	

	} 
}
