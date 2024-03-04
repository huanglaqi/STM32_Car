#include "stm32f10x.h"
#define		Intensity			((uint16_t) 0x0A02)
#define		ScanAll				((uint16_t) 0x0B07)
#define		NoDecode			((uint16_t) 0x0900)
#define		Digit(i,data)		 (((uint16_t)(i+1)<<8)|data)
#define		TurnOn				((uint16_t) 0x0C01)
#define  NSSSet()			GPIO_SetBits(GPIOB, GPIO_Pin_12)
#define  NSSReset()		GPIO_ResetBits(GPIOB, GPIO_Pin_12)



#define		MOTOR_Port				GPIOB
#define		MOTOR_Pin_IN1			GPIO_Pin_1
#define		MOTOR_Pin_IN2			GPIO_Pin_2
extern  __IO float bizha;
extern	 __IO uint16_t  speed;

void  Sonarspeed_Init(void);
void  Sonar_Init(void);
void  Sonar_trig(void);
void  Sonar_trig1(void);

void  MOTOR_Init(void);
void PWM_Init(void);
void  MOTOR_Init1(void);
void PWM_Init1(void);

uint16_t cesu1(void);
uint16_t cesu2(void);
void uart_init(u32 bound);
void USART1_IRQHandler(void);

void speed11(__IO uint16_t ess);
void speed12(__IO uint16_t ess);
void speed21(__IO uint16_t ess);
void speed22(__IO uint16_t ess);
uint16_t measu1(void);
uint16_t measu2(void);
void biz1(__IO uint16_t ees);
void biz2(__IO uint16_t ees);
//SPI
void Max7219_Init(void);
void  mySPI_Init(void);
void  myTIM_init(void);
