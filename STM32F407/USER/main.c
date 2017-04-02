#include "sys.h" 
#include "stm32f4xx_gpio.h"
#include "delay.h" 
#include "GPIO_Init.h"
#include "AT24C02.h"
#include "lcd.h"
#include "DAC.h"
#include "TFT_LCD.h"

volatile u8 ADNum;
volatile u16 Pulse;
volatile u16 Sine_Fre1;
volatile u16 Sine_Amp1;
volatile u16 Sine_Fre2;
volatile u16 Sine_Amp2;
volatile u8 Switch_Flag1;
volatile u8 Switch_Flag2;

volatile u8 ADNumt;
volatile u16 Pulset;
volatile u16 Squa_Fret;
volatile u16 Sine_Fret;
volatile u16 Sine_Ampt;
volatile u8 Switch_Flagt1;
volatile u8 Switch_Flagt2;

volatile u16 tableSize1;
volatile u16 tableSize2;
volatile u16 PulseNum;
volatile u16 VoltageValue;   //0-5000

void StartInit(void);
void AD_Scan(void);
void Pulse_Scan(void);
void CheckFlag(void);
volatile u16 FreqNuma;
int main(void)
{
	u8 i;
	u16 k;
	delay_init(168);
	AT24C02_Init();
	Gpio_Init();
	LCD_Init();
	LCD_Display();
	StartInit();
//	ADNum=100;
//	Pulse=100;
//	Sine_Amp1 =100;
//	Sine_Fre1 =100;
//	Sine_Amp2 =100;
//	Sine_Fre2 =100;
//	GPIO_SetBits(GPIOG,GPIO_Pin_10);
	GPIO_ResetBits(GPIOG,GPIO_Pin_10);
	while(1)
	{
		AD_Scan();
		Pulse_Scan();
		
	}
//	TFT_LCD_Init();
//	TFT_LCD_Clear(WHITE);
	
	
//	while(1);
//	Gpio_Init();
//	GPIO_Configuration(); 
//	AT24C02_Init();
//	LCD_Init();
//	StartInit();
////	ADNum=100;
////	Squa_Fre=100;
////	Sine_Fre=100;
////	Sine_Amp =100;
//	while(1)
//	{
//		//AD转换
//		CheckFlag();
//		if(Switch_Flag1!=Switch_Flagt1)
//		{
//			Switch_Flagt1=Switch_Flag1;
//			if(Switch_Flagt1==1)
//			{
//				Key_Out1=1;
//			}
//			else
//				Key_Out1=0;	
//			LCD_ShowSwitch_Flag1(Switch_Flagt1);
//		}
//		if(Switch_Flag2!=Switch_Flagt2)
//		{
//			Switch_Flagt2=Switch_Flag2;
//			if(Switch_Flagt2==1)
//			{
//				Key_Out21=1;
//				delay_ms(100);
//				Key_Out22=1;
//				delay_ms(100);
//				Key_Out23=1;
//			}
//			else
//			{
//				Key_Out21=0;
//				Key_Out22=0;
//				Key_Out23=0;
//			}
//			LCD_ShowSwitch_Flag2(Switch_Flagt2);
//		}
//		if((ADNumt!=ADNum))//AD数值变化，并且开关闭合，执行相关动作
//		{
//			ADNumt=ADNum;
//			SetAD(ADNumt);
//			delay_ms(10);
//		}
//		
//		//方波频率设置
//		if(Squa_Fret!=Squa_Fre)
//		{
//			Squa_Fret=Squa_Fre;
//			FreqNuma=84000/Squa_Fret-1;
//			TIM4_PWM_Init(FreqNuma,0);
//			delay_ms(10);
//		}
//		//正弦波频率设置
//		if((Sine_Fret!=Sine_Fre))
//		{
//			Sine_Fret=Sine_Fre;
//      sin_Generation(Sine_Fret,Sine_Amp);	
//	    FreqNum=42000000/Sine_Fret/tableSize-1;
//	    TIM6_Configuration(FreqNum);
//	    DAC_DMA_Configuration(); 
//			delay_ms(10);
//		}
//		
//		//正弦波幅值设置
//		if((Sine_Ampt!=Sine_Amp))
//		{
//			Sine_Ampt=Sine_Amp;
//			sin_Generation(Sine_Fret,Sine_Ampt);	
//	    FreqNum=42000000/Sine_Fret/tableSize-1;
//	    TIM6_Configuration(FreqNum);
//	    DAC_DMA_Configuration(); 
//			delay_ms(10);
//		}
//	}
}

void StartInit(void)
{
	LCD_Display();
//	//AD设置部分
	ADNum = AT24C02_ReadOneByte(0x00);
	LCD_ShowADNum(ADNum);
	ADNumt =ADNum ;
	SetAD(ADNum);
	
//	SetAD(ADNum);
//	ADNumt=ADNum;
//	LCD_ShowADNum(ADNum);
//	//开关状态监测
////	if(PDin(0)==0)    //开关闭合
////	{
////		Switch_Flag =1;
//////		AD9=0;
//////		ADNumt=ADNum;
//////		SetAD(ADNumt);
////	}
////	else            //开关打开
////	{
////		Switch_Flag =0;
//////		AD9=1;
//////		ADNumt=ADNum;
//////		SetAD(0);
////	}
////	LCD_ShowSwitch_Flag1(Switch_Flag);
	  //方波频率设置部分
    Pulse=AT24C02_ReadOneByte(0x01);
		Pulse<<=8;
		Pulse+=AT24C02_ReadOneByte(0x02);
		Pulset = Pulse;
		PulseNum=84000/Pulset-1;
		TIM1_PWM_Init(FreqNuma,0);
		LCD_ShowPulse(Pulse);
//	Squa_Fre =AT24C02_ReadOneByte(0x01);
//	Squa_Fre<<=8;
//	Squa_Fre +=AT24C02_ReadOneByte(0x02);
//	LCD_ShowSqua_Fre(Squa_Fre);
//	Squa_Fret=Squa_Fre;
//	FreqNuma=84000/Squa_Fret-1;
//  TIM4_PWM_Init(FreqNuma,0);
    //三角形波1
		Sine_Amp1 =AT24C02_ReadOneByte(0x03);
		Sine_Amp1<<=8;
		Sine_Amp1+=AT24C02_ReadOneByte(0x04);
		Sine_Fre1 =AT24C02_ReadOneByte(0x05);
		Sine_Fre1<<=8;
		Sine_Fre1+=AT24C02_ReadOneByte(0x06);
		LCD_ShowSine1(Sine_Amp1,Sine_Fre1);
		//三角形波1
		Sine_Amp2 =AT24C02_ReadOneByte(0x07);
		Sine_Amp2<<=8;
		Sine_Amp2+=AT24C02_ReadOneByte(0x08);
		Sine_Fre2 =AT24C02_ReadOneByte(0x09);
		Sine_Fre2<<=8;
		Sine_Fre2+=AT24C02_ReadOneByte(0x0A);
		LCD_ShowSine2(Sine_Amp2,Sine_Fre2);
//	//正弦波1频率设置部分
//	Sine_Fre =AT24C02_ReadOneByte(0x03);
//	Sine_Fre<<=8;
//	Sine_Fre +=AT24C02_ReadOneByte(0x04);
//	LCD_ShowSine_Fre(Sine_Fre);
//	Sine_Fret=Sine_Fre;
//	
//	//正弦波幅值设置部分
//	Sine_Amp =AT24C02_ReadOneByte(0x05);
//	Sine_Amp<<=8;
//	Sine_Amp +=AT24C02_ReadOneByte(0x06);
//	LCD_ShowSine_Amp(Sine_Amp);
//	
//  Sine_Ampt=Sine_Amp;
//	sin_Generation(Sine_Fre,Sine_Ampt);	
//	FreqNum=42000000/Sine_Fre/tableSize-1;
//	TIM6_Configuration(FreqNum);
//	DAC_DMA_Configuration();
//	CheckFlag();
//	Switch_Flagt1=Switch_Flag1;
//	if(Switch_Flagt1==1)
//	{

//	}
//	else

//	LCD_ShowSwitch_Flag1(Switch_Flagt1);
//	Switch_Flagt2=Switch_Flag2;
//	if(Switch_Flagt2==1)
//	{
//		Key_Out21=1;
//		delay_ms(10);
//		Key_Out22=1;
//		delay_ms(10);
//		Key_Out23=1;
//	}
//	else
//	{
//		Key_Out21=0;
//		Key_Out22=0;
//		Key_Out23=0;
//	}
//	LCD_ShowSwitch_Flag2(Switch_Flagt2);
//	delay_ms(20);
}
void AD_Scan(void)
{
	if(ADNumt!=ADNum)
	{
		delay_ms(10);
		ADNumt =ADNum ;
		SetAD(ADNum);
	}
}
void Pulse_Scan(void)
{
	if(Pulset!=Pulse)
	{
		delay_ms(10);
		Pulset =Pulse;
		PulseNum=84000/Pulset-1;
		TIM1_PWM_Init(FreqNuma,0);
	}
}
void CheckFlag(void)
{
	if(PDin(0)==0)
		Switch_Flag1=1;
	else
		Switch_Flag1=0;
	if(PDin(1)==0)
		Switch_Flag2=1;
	else
		Switch_Flag2=0;
	delay_ms(10);
}
