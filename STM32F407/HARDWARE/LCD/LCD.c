#include "lcd.h"

#define XLevelL		0x00
#define XLevelH		0x10
#define XLevel		((XLevelH&0x0F)*16+XLevelL)
#define Max_Column	128
#define Max_Row		  64
#define	Brightness	0xCF 

#define X_WIDTH 128
#define Y_WIDTH 64

const u8 F6x8[][6] =
{
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // sp
    { 0x00, 0x00, 0x00, 0x2f, 0x00, 0x00 },   // !
    { 0x00, 0x00, 0x07, 0x00, 0x07, 0x00 },   // "
    { 0x00, 0x14, 0x7f, 0x14, 0x7f, 0x14 },   // #
    { 0x00, 0x24, 0x2a, 0x7f, 0x2a, 0x12 },   // $
    { 0x00, 0x62, 0x64, 0x08, 0x13, 0x23 },   // %
    { 0x00, 0x36, 0x49, 0x55, 0x22, 0x50 },   // &
    { 0x00, 0x00, 0x05, 0x03, 0x00, 0x00 },   // '
    { 0x00, 0x00, 0x1c, 0x22, 0x41, 0x00 },   // (
    { 0x00, 0x00, 0x41, 0x22, 0x1c, 0x00 },   // )
    { 0x00, 0x14, 0x08, 0x3E, 0x08, 0x14 },   // *
    { 0x00, 0x08, 0x08, 0x3E, 0x08, 0x08 },   // +
    { 0x00, 0x00, 0x00, 0xA0, 0x60, 0x00 },   // ,
    { 0x00, 0x08, 0x08, 0x08, 0x08, 0x08 },   // -
    { 0x00, 0x00, 0x60, 0x60, 0x00, 0x00 },   // .
    { 0x00, 0x20, 0x10, 0x08, 0x04, 0x02 },   // /
    { 0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E },   // 0
    { 0x00, 0x00, 0x42, 0x7F, 0x40, 0x00 },   // 1
    { 0x00, 0x42, 0x61, 0x51, 0x49, 0x46 },   // 2
    { 0x00, 0x21, 0x41, 0x45, 0x4B, 0x31 },   // 3
    { 0x00, 0x18, 0x14, 0x12, 0x7F, 0x10 },   // 4
    { 0x00, 0x27, 0x45, 0x45, 0x45, 0x39 },   // 5
    { 0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30 },   // 6
    { 0x00, 0x01, 0x71, 0x09, 0x05, 0x03 },   // 7
    { 0x00, 0x36, 0x49, 0x49, 0x49, 0x36 },   // 8
    { 0x00, 0x06, 0x49, 0x49, 0x29, 0x1E },   // 9
    { 0x00, 0x00, 0x36, 0x36, 0x00, 0x00 },   // :
    { 0x00, 0x00, 0x56, 0x36, 0x00, 0x00 },   // ;
    { 0x00, 0x08, 0x14, 0x22, 0x41, 0x00 },   // <
    { 0x00, 0x14, 0x14, 0x14, 0x14, 0x14 },   // =
    { 0x00, 0x00, 0x41, 0x22, 0x14, 0x08 },   // >
    { 0x00, 0x02, 0x01, 0x51, 0x09, 0x06 },   // ?
    { 0x00, 0x32, 0x49, 0x59, 0x51, 0x3E },   // @
    { 0x00, 0x7C, 0x12, 0x11, 0x12, 0x7C },   // A
    { 0x00, 0x7F, 0x49, 0x49, 0x49, 0x36 },   // B
    { 0x00, 0x3E, 0x41, 0x41, 0x41, 0x22 },   // C
    { 0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C },   // D
    { 0x00, 0x7F, 0x49, 0x49, 0x49, 0x41 },   // E
    { 0x00, 0x7F, 0x09, 0x09, 0x09, 0x01 },   // F
    { 0x00, 0x3E, 0x41, 0x49, 0x49, 0x7A },   // G
    { 0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F },   // H
    { 0x00, 0x00, 0x41, 0x7F, 0x41, 0x00 },   // I
    { 0x00, 0x20, 0x40, 0x41, 0x3F, 0x01 },   // J
    { 0x00, 0x7F, 0x08, 0x14, 0x22, 0x41 },   // K
    { 0x00, 0x7F, 0x40, 0x40, 0x40, 0x40 },   // L
    { 0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F },   // M
    { 0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F },   // N
    { 0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E },   // O
    { 0x00, 0x7F, 0x09, 0x09, 0x09, 0x06 },   // P
    { 0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E },   // Q
    { 0x00, 0x7F, 0x09, 0x19, 0x29, 0x46 },   // R
    { 0x00, 0x46, 0x49, 0x49, 0x49, 0x31 },   // S
    { 0x00, 0x01, 0x01, 0x7F, 0x01, 0x01 },   // T
    { 0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F },   // U
    { 0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F },   // V
    { 0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F },   // W
    { 0x00, 0x63, 0x14, 0x08, 0x14, 0x63 },   // X
    { 0x00, 0x07, 0x08, 0x70, 0x08, 0x07 },   // Y
    { 0x00, 0x61, 0x51, 0x49, 0x45, 0x43 },   // Z
    { 0x00, 0x00, 0x7F, 0x41, 0x41, 0x00 },   // [
    { 0x00, 0x55, 0x2A, 0x55, 0x2A, 0x55 },   // 55
    { 0x00, 0x00, 0x41, 0x41, 0x7F, 0x00 },   // ]
    { 0x00, 0x04, 0x02, 0x01, 0x02, 0x04 },   // ^
    { 0x00, 0x40, 0x40, 0x40, 0x40, 0x40 },   // _
    { 0x00, 0x00, 0x01, 0x02, 0x04, 0x00 },   // '
    { 0x00, 0x20, 0x54, 0x54, 0x54, 0x78 },   // a
    { 0x00, 0x7F, 0x48, 0x44, 0x44, 0x38 },   // b
    { 0x00, 0x38, 0x44, 0x44, 0x44, 0x20 },   // c
    { 0x00, 0x38, 0x44, 0x44, 0x48, 0x7F },   // d
    { 0x00, 0x38, 0x54, 0x54, 0x54, 0x18 },   // e
    { 0x00, 0x08, 0x7E, 0x09, 0x01, 0x02 },   // f
    { 0x00, 0x18, 0xA4, 0xA4, 0xA4, 0x7C },   // g
    { 0x00, 0x7F, 0x08, 0x04, 0x04, 0x78 },   // h
    { 0x00, 0x00, 0x44, 0x7D, 0x40, 0x00 },   // i
    { 0x00, 0x40, 0x80, 0x84, 0x7D, 0x00 },   // j
    { 0x00, 0x7F, 0x10, 0x28, 0x44, 0x00 },   // k
    { 0x00, 0x00, 0x41, 0x7F, 0x40, 0x00 },   // l
    { 0x00, 0x7C, 0x04, 0x18, 0x04, 0x78 },   // m
    { 0x00, 0x7C, 0x08, 0x04, 0x04, 0x78 },   // n
    { 0x00, 0x38, 0x44, 0x44, 0x44, 0x38 },   // o
    { 0x00, 0xFC, 0x24, 0x24, 0x24, 0x18 },   // p
    { 0x00, 0x18, 0x24, 0x24, 0x18, 0xFC },   // q
    { 0x00, 0x7C, 0x08, 0x04, 0x04, 0x08 },   // r
    { 0x00, 0x48, 0x54, 0x54, 0x54, 0x20 },   // s
    { 0x00, 0x04, 0x3F, 0x44, 0x40, 0x20 },   // t
    { 0x00, 0x3C, 0x40, 0x40, 0x20, 0x7C },   // u
    { 0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C },   // v
    { 0x00, 0x3C, 0x40, 0x30, 0x40, 0x3C },   // w
    { 0x00, 0x44, 0x28, 0x10, 0x28, 0x44 },   // x
    { 0x00, 0x1C, 0xA0, 0xA0, 0xA0, 0x7C },   // y
    { 0x00, 0x44, 0x64, 0x54, 0x4C, 0x44 },   // z
    { 0x80, 0x80, 0x80, 0x80, 0x80, 0x80 }    // horiz lines
};

void LCD_WrDat(u8 data)
{
	u8 i=8;
	//LCD_CS=0;;
	SET_LCD_DC;
	delay_us(1);
  CLR_LCD_SCL;
	delay_us(5);   
  while(i--)
  {
    if(data&0x80)
		{
			SET_LCD_SDA;
			delay_us(1);
		}
    else
		{
			CLR_LCD_SDA;
			delay_us(1);
		}
    SET_LCD_SCL; 
    delay_us(3);            
    CLR_LCD_SCL;
		delay_us(1); 
    data<<=1;    
		delay_us(1);
  }
	//LCD_CS=1;
}

void LCD_WrCmd(u8 cmd)
{
	u8 i=8;
	//LCD_CS=0;;
	CLR_LCD_DC;
	delay_us(1);
  CLR_LCD_SCL;
	delay_us(5);
  while(i--)
  {
    if(cmd&0x80)
		{
			SET_LCD_SDA;
		  delay_us(1);
		}
    else
		{
			CLR_LCD_SDA;
			delay_us(1);
		}
    SET_LCD_SCL;
		delay_us(3);          
    CLR_LCD_SCL;
	  delay_us(1); 
    cmd<<=1;
		delay_us(1);
  } 	
	//LCD_CS=1;
}

void LCD_Set_Pos(u8 x, u8 y)
{ 
  LCD_WrCmd(0xb0+y);
  LCD_WrCmd(((x&0xf0)>>4)|0x10);
  LCD_WrCmd((x&0x0f)|0x01); 
} 

void LCD_Fill(u8 bmp_data)
{
	u8 y,x;
	
	for(y=0;y<8;y++)
	{
		LCD_WrCmd(0xb0+y);
		LCD_WrCmd(0x01);
		LCD_WrCmd(0x10);
		for(x=0;x<X_WIDTH;x++)
			LCD_WrDat(bmp_data);
	}
}

void LCD_CLS(void)
{
	u8 y,x;	
	for(y=0;y<8;y++)
	{
		LCD_WrCmd(0xb0+y);
		LCD_WrCmd(0x01);
		LCD_WrCmd(0x10); 
		for(x=0;x<X_WIDTH;x++)
			LCD_WrDat(0);
	}
}

void Set_Display_On_Off(unsigned char d)
{
	LCD_WrCmd(0xAE|d);			// Set Display On/Off						
}

void adjust(u8 a)
{
  LCD_WrCmd(a);	//????0x0000~0x003f  
}

void Set_Display_Clock(unsigned char d)
{
	LCD_WrCmd(0xD5);			// Set Display Clock Divide Ratio / Oscillator Frequency
	LCD_WrCmd(d);			//   Default => 0x80
}

void Set_Multiplex_Ratio(unsigned char d)
{
	LCD_WrCmd(0xA8);			// Set Multiplex Ratio
	LCD_WrCmd(d);			//   Default => 0x3F (1/64 Duty)
}

void Set_Display_Offset(unsigned char d)
{
	LCD_WrCmd(0xD3);			// Set Display Offset
	LCD_WrCmd(d);			//   Default => 0x00
}

void Set_Charge_Pump(unsigned char d)
{
	LCD_WrCmd(0x8D);			// Set Charge Pump
	LCD_WrCmd(0x10|d);			//   Default => 0x10
}

void SetStartLine(unsigned char d)
{
	LCD_WrCmd(0x40|d);			// Set Display Start Line
}

void SetAddressingMode(unsigned char d)
{
	LCD_WrCmd(0x20);			// Set Memory Addressing Mode
	LCD_WrCmd(d);			//   Default => 0x02
}

void Set_Segment_Remap(unsigned char d)
{
	LCD_WrCmd(0xA0|d);			// Set Segment Re-Map
}

void Set_Common_Remap(unsigned char d)
{
	LCD_WrCmd(0xC0|d);			// Set COM Output Scan Direction
}

void Set_Common_Config(unsigned char d)
{
	LCD_WrCmd(0xDA);			// Set COM Pins Hardware Configuration
	LCD_WrCmd(0x02|d);			//   Default => 0x12 (0x10)
}

void SetContrastControl(unsigned char d)
{
	LCD_WrCmd(0x81);			// Set Contrast Control
	LCD_WrCmd(d);			//   Default => 0x7F
}

void Set_Precharge_Period(unsigned char d)
{
	LCD_WrCmd(0xD9);			// Set Pre-Charge Period
	LCD_WrCmd(d);			//   Default => 0x22 (2 Display Clocks [Phase 2] / 2 Display Clocks [Phase 1])
}

void Set_VCOMH(unsigned char d)
{
	LCD_WrCmd(0xDB);			// Set VCOMH Deselect Level
	LCD_WrCmd(d);			//   Default => 0x20 (0.77*VCC)
}

void Set_Entire_Display(unsigned char d)
{
	LCD_WrCmd(0xA4|d);			// Set Entire Display On / Off
}

void Set_Inverse_Display(unsigned char d)
{
	LCD_WrCmd(0xA6|d);			// Set Inverse Display On/Off
}

void LCD_Init(void)        
{
  SET_LCD_SCL;
  CLR_LCD_RST;
  delay_ms(50);
  SET_LCD_RST;

  Set_Display_On_Off(0x00);
	delay_ms(10);	  // Display Off (0x00/0x01)
  Set_Display_Clock(0x80);	
	delay_ms(10);		  // Set Clock as 100 Frames/Sec
  Set_Multiplex_Ratio(0x3F);
	delay_ms(10);		// 1/64 Duty (0x0F~0x3F)
  Set_Display_Offset(0x00);
	delay_ms(10);	  // Shift Mapping RAM Counter (0x00~0x3F)
  SetStartLine(0x00);
	delay_ms(10);	      // Set Mapping RAM Display Start Line (0x00~0x3F)
  Set_Charge_Pump(0x04);
	delay_ms(10);		    // Enable Embedded DC/DC Converter (0x00/0x04)
  SetAddressingMode(0x02);
	delay_ms(10);		  // Set Page Addressing Mode (0x00/0x01/0x02)
  Set_Segment_Remap(0x01);
	delay_ms(10);			  // Set SEG/Column Mapping     0x00???? 0x01??
  Set_Common_Remap(0x08);
	delay_ms(10);				  // Set COM/Row Scan Direction 0x00???? 0x08??
  Set_Common_Config(0x10);
	delay_ms(10);		  // Set Sequential Configuration (0x00/0x10)
  SetContrastControl(Brightness);
	delay_ms(10);	// Set SEG Output Current
  Set_Precharge_Period(0xF1);
	delay_ms(10);	// Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
  Set_VCOMH(0x40);		
	delay_ms(10);	        // Set VCOM Deselect Level
  Set_Entire_Display(0x00);
	delay_ms(10);	  // Disable Entire Display On (0x00/0x01)
  Set_Inverse_Display(0x00);
	delay_ms(10);	// Disable Inverse Display On (0x00/0x01)  
  Set_Display_On_Off(0x01);	
	delay_ms(10);	  // Display On (0x00/0x01)
  LCD_Fill(0x00); 
	delay_ms(10);	 
	LCD_Set_Pos(0,0); 
	delay_ms(10);	
} 

void LCD_P6x8Str(u8 x,u8 y,u8 ch[])
{
  u8 i=0,j=0;      
	LCD_Set_Pos(x,y);
	j=ch[0]-32;
	for(i=0;i<6;i++)     
  	  LCD_WrDat(F6x8[j][i]); 
}
void LCD_Show_M(u8 y,unsigned long int num)
{
	u8 num1, i, j, k, c;
	num1=num/65536;
	k=100;
	for(i=0;i<3;i++)
	{
		LCD_Set_Pos(0+6*i,y);
		c=(num1/k)%10+16;
		for(j=0;j<6;j++)     
  	  LCD_WrDat(F6x8[c][j]); 
		k/=10;	
	}
	num1=(num%65536)/256;
	k=100;
	for(i=0;i<3;i++)
	{
		LCD_Set_Pos(40+6*i,y);
		c=(num1/k)%10+16;
		for(j=0;j<6;j++)     
  	  LCD_WrDat(F6x8[c][j]); 
		k/=10;	
	}
	num1=num%256;
	k=100;
	for(i=0;i<3;i++)
	{
		LCD_Set_Pos(80+6*i,y);
		c=(num1/k)%10+16;
		for(j=0;j<6;j++)     
  	  LCD_WrDat(F6x8[c][j]); 
		k/=10;	
	}	
}
void LCD_ShowNum(u8 x,u8 y, unsigned int num)
{
	u8 i,j,c=0;
	unsigned int t=1,k=1;
	if(num==0)
	{
		LCD_Set_Pos(x,y);
		for(j=0;j<6;j++)     
  	  LCD_WrDat(F6x8[16][j]); 
	}		
	t=num;
	for(i=0;t>0;i++)
	{
		t=t/10;
		k=k*10;	
	}
	if(x+6*i>126)
		x=126-6*i;
	k=k/10;
	while(i>0)
	{
		LCD_Set_Pos(x,y);
		c=(num/k)%10;
		c=c+16;
		for(j=0;j<6;j++)     
  	  LCD_WrDat(F6x8[c][j]); 
		k=k/10;
		x=x+6;
		i--;
	}
}
void LCD_Display(void)
{
	u8 StartAddress1=5;
	u8 StartAddress2=5;
	u8 StartAddress3=5;
	u8 StartAddress4=5;
	u8 StartAddress6=5;
	u8 StartAddress7=5;
	u8 StartAddress8=5;
	u8 StartAddress5=5;
	u8 i,j;
	//        A  D  N  u  m   :
	u8 M1[6]={33,36,46,85,77,26};
	//        Pulse
	u8 M2[6]={48,85,76,83,69,26};
	//        S  i   n  e 1:
	u8 M3[6]={51,73,78,69,17,26};
	//        S  i   n  e 2:
	u8 M4[6]={51,73,78,69,18,26};
	//       Red
	u8 M5[4]={50,69,68,26};
	//      Laser
	u8 M6[6]={44,65,83,69,82,26};
	//       Mode
	u8 M7[5]={45,79,68,69,26};
	//       Warning
	u8 M8[8]={55,65,82,78,73,78,71,26};
  for(i=0;i<6;i++)
	{
		LCD_Set_Pos(StartAddress1+6*i,0);
	  for(j=0;j<6;j++)     
			LCD_WrDat(F6x8[M1[i]][j]); 
	}
	for(i=0;i<6;i++)
	{
		LCD_Set_Pos(StartAddress2+6*i,1);
	  for(j=0;j<6;j++)     
			LCD_WrDat(F6x8[M2[i]][j]); 
	}
	for(i=0;i<6;i++)
	{
		LCD_Set_Pos(StartAddress3+6*i,2);
	  for(j=0;j<6;j++)     
			LCD_WrDat(F6x8[M3[i]][j]); 
	}
	for(i=0;i<6;i++)
	{
		LCD_Set_Pos(StartAddress4+6*i,3);
	  for(j=0;j<6;j++)     
			LCD_WrDat(F6x8[M4[i]][j]); 
	}
	for(i=0;i<4;i++)
	{
		LCD_Set_Pos(StartAddress5+6*i,4);
	  for(j=0;j<6;j++)     
			LCD_WrDat(F6x8[M5[i]][j]); 
	}
	for(i=0;i<6;i++)
	{
		LCD_Set_Pos(StartAddress6+6*i,5);
	  for(j=0;j<6;j++)     
			LCD_WrDat(F6x8[M6[i]][j]); 
	}
	for(i=0;i<5;i++)
	{
		LCD_Set_Pos(StartAddress7+6*i,6);
	  for(j=0;j<6;j++)     
			LCD_WrDat(F6x8[M7[i]][j]); 
	}
	for(i=0;i<8;i++)
	{
		LCD_Set_Pos(StartAddress8+6*i,7);
	  for(j=0;j<6;j++)     
			LCD_WrDat(F6x8[M8[i]][j]); 
	}
  //��ʾA
	LCD_Set_Pos(42,2);
	for(j=0;j<6;j++)     
		LCD_WrDat(F6x8[33][j]); 
	LCD_Set_Pos(42,3);
	for(j=0;j<6;j++)     
		LCD_WrDat(F6x8[33][j]);
	
	LCD_Set_Pos(75,2);
	for(j=0;j<6;j++)     
		LCD_WrDat(F6x8[38][j]); 
	LCD_Set_Pos(75,3);
	for(j=0;j<6;j++)     
		LCD_WrDat(F6x8[38][j]);
}
void LCD_ShowADNum(u8 Value)
{
	u8 i,j;
	u8 StartAddress1=42;
	for(i=7;i>=0;i--)
	{
		
	  LCD_Set_Pos(StartAddress1+42-6*i,0);
		if(Value&1<<i)
		{
			for(j=0;j<6;j++)     
			LCD_WrDat(F6x8[17][j]); 
		}
		else
		{
			for(j=0;j<6;j++)     
			LCD_WrDat(F6x8[16][j]); 
		}	
		if(i==0)
			break;
	}
	LCD_Set_Pos(95,0);
	i=Value/100;
	if(Value>99)
	{
		for(j=0;j<6;j++)     
			LCD_WrDat(F6x8[16+i][j]); 
	}
	else
	{
		for(j=0;j<6;j++)     
			LCD_WrDat(0); 
	}
	LCD_Set_Pos(101,0);
	i=(Value%100)/10;
	if(Value>9)
	{
		for(j=0;j<6;j++)     
			LCD_WrDat(F6x8[16+i][j]); 
	}
	else
	{
		for(j=0;j<6;j++)     
			LCD_WrDat(0); 
	}
	LCD_Set_Pos(107,0);
	i=Value%10;
  for(j=0;j<6;j++)     
		LCD_WrDat(F6x8[16+i][j]); 
}
void LCD_ShowPulse(u16 Value)
{
	u8 i,j;
	u8 StartAddress=50;
	i=Value/1000;
	LCD_Set_Pos(StartAddress,1);
	if(Value>999)
	{
		for(j=0;j<6;j++)     
			LCD_WrDat(F6x8[16+i][j]); 
	}
	else
	{
			for(j=0;j<6;j++)     
				LCD_WrDat(0); 
	}
	i=(Value%1000)/100;
	LCD_Set_Pos(StartAddress+6,1);
	if(Value>99)
	{
		for(j=0;j<6;j++)     
			LCD_WrDat(F6x8[16+i][j]); 
	}
	else
	{
			for(j=0;j<6;j++)     
				LCD_WrDat(0); 
	}
	i=(Value%100)/10;
	LCD_Set_Pos(StartAddress+12,1);
	if(Value>9)
	{
		for(j=0;j<6;j++)     
			LCD_WrDat(F6x8[16+i][j]); 
	}
	else
	{
			for(j=0;j<6;j++)     
				LCD_WrDat(0); 
	}
	i=Value%10;
	LCD_Set_Pos(StartAddress+18,1);
	
		for(j=0;j<6;j++)     
			LCD_WrDat(F6x8[16+i][j]); 
}
void LCD_ShowSine1(u16 Amp,u16 Fre)
{
	u8 i,j;
	u8 StartAddress1=50;
	u8 StartAddress2=85;
	i=Amp/100;
	LCD_Set_Pos(StartAddress1,2);
	if(Amp>99)
	{
		for(j=0;j<6;j++)     
			LCD_WrDat(F6x8[16+i][j]); 
	}
	else
	{
		for(j=0;j<6;j++)     
			LCD_WrDat(0); 
	}
	i=(Amp%100)/10;
	LCD_Set_Pos(StartAddress1+6,2);
	if(Amp>9)
	{
		for(j=0;j<6;j++)     
			LCD_WrDat(F6x8[16+i][j]); 
	}
	else
	{
			for(j=0;j<6;j++)     
				LCD_WrDat(0); 
	}
	i=Amp%10;
	LCD_Set_Pos(StartAddress1+12,2);
	for(j=0;j<6;j++)     
		LCD_WrDat(F6x8[16+i][j]); 
	
	i=Fre/100;
	LCD_Set_Pos(StartAddress2,2);
	if(Fre>99)
	{
		for(j=0;j<6;j++)     
			LCD_WrDat(F6x8[16+i][j]); 
	}
	else
	{
		for(j=0;j<6;j++)     
			LCD_WrDat(0); 
	}
	i=(Fre%100)/10;
	LCD_Set_Pos(StartAddress2+6,2);
	if(Fre>9)
	{
		for(j=0;j<6;j++)     
			LCD_WrDat(F6x8[16+i][j]); 
	}
	else
	{
			for(j=0;j<6;j++)     
				LCD_WrDat(0); 
	}
	i=Fre%10;
	LCD_Set_Pos(StartAddress2+12,2);
	for(j=0;j<6;j++)     
		LCD_WrDat(F6x8[16+i][j]); 

}

void LCD_ShowSine2(u16 Amp,u16 Fre)
{
	u8 i,j;
	u8 StartAddress1=50;
	u8 StartAddress2=85;
	i=Amp/100;
	LCD_Set_Pos(StartAddress1,3);
	if(Amp>99)
	{
		for(j=0;j<6;j++)     
			LCD_WrDat(F6x8[16+i][j]); 
	}
	else
	{
		for(j=0;j<6;j++)     
			LCD_WrDat(0); 
	}
	i=(Amp%100)/10;
	LCD_Set_Pos(StartAddress1+6,3);
	if(Amp>9)
	{
		for(j=0;j<6;j++)     
			LCD_WrDat(F6x8[16+i][j]); 
	}
	else
	{
			for(j=0;j<6;j++)     
				LCD_WrDat(0); 
	}
	i=Amp%10;
	LCD_Set_Pos(StartAddress1+12,3);
	for(j=0;j<6;j++)     
		LCD_WrDat(F6x8[16+i][j]); 
	
	i=Fre/100;
	LCD_Set_Pos(StartAddress2,3);
	if(Fre>99)
	{
		for(j=0;j<6;j++)     
			LCD_WrDat(F6x8[16+i][j]); 
	}
	else
	{
		for(j=0;j<6;j++)     
			LCD_WrDat(0); 
	}
	i=(Fre%100)/10;
	LCD_Set_Pos(StartAddress2+6,3);
	if(Fre>9)
	{
		for(j=0;j<6;j++)     
			LCD_WrDat(F6x8[16+i][j]); 
	}
	else
	{
			for(j=0;j<6;j++)     
				LCD_WrDat(0); 
	}
	i=Fre%10;
	LCD_Set_Pos(StartAddress2+12,3);
	for(j=0;j<6;j++)     
		LCD_WrDat(F6x8[16+i][j]); 

}
void LCD_ShowRed(u8 Value)
{
	u8 StartAddress=50;
	u8 j;
	LCD_Set_Pos(StartAddress,4);
	if(Value==1)
	{
		
		for(j=0;j<6;j++)     
				LCD_WrDat(0); 
		for(j=0;j<6;j++)       
			LCD_WrDat(F6x8[79][j]);
		for(j=0;j<6;j++)       
			LCD_WrDat(F6x8[78][j]); 		
	}
	else
	{
		for(j=0;j<6;j++)       
			LCD_WrDat(F6x8[79][j]);
		for(j=0;j<6;j++)       
			LCD_WrDat(F6x8[70][j]); 	
		for(j=0;j<6;j++)       
			LCD_WrDat(F6x8[70][j]); 
	}
}
void LCD_ShowLaser(u8 Value)
{
	u8 StartAddress=50;
	u8 j;
	LCD_Set_Pos(StartAddress,5);
	if(Value==1)
	{
		
		for(j=0;j<6;j++)     
				LCD_WrDat(0); 
		for(j=0;j<6;j++)       
			LCD_WrDat(F6x8[79][j]);
		for(j=0;j<6;j++)       
			LCD_WrDat(F6x8[78][j]); 		
	}
	else
	{
		for(j=0;j<6;j++)       
			LCD_WrDat(F6x8[79][j]);
		for(j=0;j<6;j++)       
			LCD_WrDat(F6x8[70][j]); 	
		for(j=0;j<6;j++)       
			LCD_WrDat(F6x8[70][j]); 
	}
}
void LCD_ShowMode(u8 Value)
{
	u8 StartAddress=56;
	u8 j;
	LCD_Set_Pos(StartAddress,6);
	if(Value==1)
	{
		for(j=0;j<6;j++)       
			LCD_WrDat(F6x8[18][j]);	
	}
	else
	{	
		for(j=0;j<6;j++)       
			LCD_WrDat(F6x8[17][j]); 
	}
}

void LCD_ShowWarning(u8 Value)
{
	u8 StartAddress=56;
	u8 j;
	LCD_Set_Pos(StartAddress,7);
	if(Value==1)
	{
		for(j=0;j<6;j++)       
			LCD_WrDat(F6x8[17][j]);	
	}
	else
	{	
		for(j=0;j<6;j++)       
			LCD_WrDat(F6x8[16][j]); 
	}
}

