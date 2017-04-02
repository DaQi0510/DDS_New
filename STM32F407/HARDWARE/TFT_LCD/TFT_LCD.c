#include "TFT_LCD.h"
#include "delay.h"
void TFT_LCD_Writ_Bus(char dat)   //��������д��
{	
//	 while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET){}//�ȴ���������  
//	 SPI_I2S_SendData(SPI2, dat); //ͨ������SPIx����һ��byte  ����
	u8 i;			  
	for(i=0;i<8;i++)
	{			  
		TFT_LCD_CLK_Clr;
		if(dat&0x80)
		   TFT_LCD_MOSI_Set;
		else 
		   TFT_LCD_MOSI_Clr;
		TFT_LCD_CLK_Set;
		dat<<=1;   
	}			
}
void TFT_LCD_WR_DATA8(char da) //��������8λ
{	
	TFT_LCD_DC_Set ;
	TFT_LCD_Writ_Bus(da);  
} 
void TFT_LCD_WR_DATA(int da)   //��������16λ
{	
	TFT_LCD_DC_Set ;
	TFT_LCD_Writ_Bus(da>>8);  
  TFT_LCD_Writ_Bus(da);
}	

void TFT_LCD_WR_REG(char da)	//д�Ĵ���
{	
	TFT_LCD_DC_Clr ;
	TFT_LCD_Writ_Bus(da);
}
void LCD_WR_REG_DATA(int reg,int da)  //�ڼĴ�������д����
{
	TFT_LCD_WR_REG(reg);
	TFT_LCD_WR_DATA(da);
}
void Address_set(unsigned int x1,unsigned int y1,unsigned int x2,unsigned int y2)   //���õ�ַ
{ 
	 TFT_LCD_WR_REG(0x2a);
   TFT_LCD_WR_DATA8(x1>>8);
   TFT_LCD_WR_DATA8(x1);
   TFT_LCD_WR_DATA8(x2>>8);
   TFT_LCD_WR_DATA8(x2);
  
   TFT_LCD_WR_REG(0x2b);
   TFT_LCD_WR_DATA8(y1>>8);
   TFT_LCD_WR_DATA8(y1);
   TFT_LCD_WR_DATA8(y2>>8);
   TFT_LCD_WR_DATA8(y2);

   TFT_LCD_WR_REG(0x2C);					 						 
}

void TFT_LCD_Init(void)   //��ʼ��Һ����ʾ
{
	u16 i,j;
	//SPI��ʼ��
	GPIO_InitTypeDef  GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��GPIOAʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOAʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//ʹ��GPIOAʱ��
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);//ʹ��SPI2ʱ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//RST��ӦIO��
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ����ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//����
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIO
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;//PB3~5���ù������	
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
//  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PB3~5���ù������	
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
//  GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��
//	
//	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI2); //PB3����Ϊ SPI2
//	GPIO_PinAFConfig(GPIOC,GPIO_PinSource3,GPIO_AF_SPI2); //PB4����Ϊ SPI2
//	
//	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,ENABLE);//��λSPI1
//	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,DISABLE);//ֹͣ��λSPI1

//	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
//	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//����SPI����ģʽ:����Ϊ��SPI
//	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
//	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
//	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
//	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
//	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
//	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
//	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRCֵ����Ķ���ʽ
//	SPI_Init(SPI2, &SPI_InitStructure);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
//	
//	SPI_Cmd(SPI2, ENABLE); //ʹ��SPI����
	TFT_LCD_CLK_Set ;
	TFT_LCD_MOSI_Set ;
	TFT_LCD_DC_Set ;
  TFT_LCD_RES_Clr;
	for(i=0;i<1000;i++)
		for(j=0;j<100;j++);
	TFT_LCD_RES_Set ;
	for(i=0;i<1000;i++)
		for(j=0;j<100;j++);
		
	//************* Start Initial Sequence **********// 
	TFT_LCD_WR_REG(0xCF);  
	TFT_LCD_WR_DATA8(0x00); 
	TFT_LCD_WR_DATA8(0xD9); 
	TFT_LCD_WR_DATA8(0X30); 
	 
	TFT_LCD_WR_REG(0xED);  
	TFT_LCD_WR_DATA8(0x64); 
	TFT_LCD_WR_DATA8(0x03); 
	TFT_LCD_WR_DATA8(0X12); 
	TFT_LCD_WR_DATA8(0X81); 
	 
	TFT_LCD_WR_REG(0xE8);  
	TFT_LCD_WR_DATA8(0x85); 
	TFT_LCD_WR_DATA8(0x10); 
	TFT_LCD_WR_DATA8(0x78); 
	 
	TFT_LCD_WR_REG(0xCB);  
	TFT_LCD_WR_DATA8(0x39); 
	TFT_LCD_WR_DATA8(0x2C); 
	TFT_LCD_WR_DATA8(0x00); 
	TFT_LCD_WR_DATA8(0x34); 
	TFT_LCD_WR_DATA8(0x02); 
	 
	TFT_LCD_WR_REG(0xF7);  
	TFT_LCD_WR_DATA8(0x20); 
	 
	TFT_LCD_WR_REG(0xEA);  
	TFT_LCD_WR_DATA8(0x00); 
	TFT_LCD_WR_DATA8(0x00); 
	 
	TFT_LCD_WR_REG(0xC0);    //Power control 
	TFT_LCD_WR_DATA8(0x21);   //VRH[5:0] 
	 
	TFT_LCD_WR_REG(0xC1);    //Power control 
	TFT_LCD_WR_DATA8(0x12);   //SAP[2:0];BT[3:0] 
	 
	TFT_LCD_WR_REG(0xC5);    //VCM control 
	TFT_LCD_WR_DATA8(0x32); 
	TFT_LCD_WR_DATA8(0x3C); 
	 
	TFT_LCD_WR_REG(0xC7);    //VCM control2 
	TFT_LCD_WR_DATA8(0XC1); 
	 
	TFT_LCD_WR_REG(0x36);    // Memory Access Control 
	TFT_LCD_WR_DATA8(0x08); 
	 
	TFT_LCD_WR_REG(0x3A);   
	TFT_LCD_WR_DATA8(0x55); 

	TFT_LCD_WR_REG(0xB1);   
	TFT_LCD_WR_DATA8(0x00);   
	TFT_LCD_WR_DATA8(0x18); 
	 
	TFT_LCD_WR_REG(0xB6);    // Display Function Control 
	TFT_LCD_WR_DATA8(0x0A); 
	TFT_LCD_WR_DATA8(0xA2); 

	 
	 
	TFT_LCD_WR_REG(0xF2);    // 3Gamma Function Disable 
	TFT_LCD_WR_DATA8(0x00); 
	 
	TFT_LCD_WR_REG(0x26);    //Gamma curve selected 
	TFT_LCD_WR_DATA8(0x01); 
	 
	TFT_LCD_WR_REG(0xE0);    //Set Gamma 
	TFT_LCD_WR_DATA8(0x0F); 
	TFT_LCD_WR_DATA8(0x20); 
	TFT_LCD_WR_DATA8(0x1E); 
	TFT_LCD_WR_DATA8(0x09); 
	TFT_LCD_WR_DATA8(0x12); 
	TFT_LCD_WR_DATA8(0x0B); 
	TFT_LCD_WR_DATA8(0x50); 
	TFT_LCD_WR_DATA8(0XBA); 
	TFT_LCD_WR_DATA8(0x44); 
	TFT_LCD_WR_DATA8(0x09); 
	TFT_LCD_WR_DATA8(0x14); 
	TFT_LCD_WR_DATA8(0x05); 
	TFT_LCD_WR_DATA8(0x23); 
	TFT_LCD_WR_DATA8(0x21); 
	TFT_LCD_WR_DATA8(0x00); 
	 
	TFT_LCD_WR_REG(0XE1);    //Set Gamma 
	TFT_LCD_WR_DATA8(0x00); 
	TFT_LCD_WR_DATA8(0x19); 
	TFT_LCD_WR_DATA8(0x19); 
	TFT_LCD_WR_DATA8(0x00); 
	TFT_LCD_WR_DATA8(0x12); 
	TFT_LCD_WR_DATA8(0x07); 
	TFT_LCD_WR_DATA8(0x2D); 
	TFT_LCD_WR_DATA8(0x28); 
	TFT_LCD_WR_DATA8(0x3F); 
	TFT_LCD_WR_DATA8(0x02); 
	TFT_LCD_WR_DATA8(0x0A); 
	TFT_LCD_WR_DATA8(0x08); 
	TFT_LCD_WR_DATA8(0x25); 
	TFT_LCD_WR_DATA8(0x2D); 
	TFT_LCD_WR_DATA8(0x0F); 
	 
	TFT_LCD_WR_REG(0x11);    //Exit Sleep  
	for(i=0;i<1000;i++)
		for(j=0;j<100;j++);
	TFT_LCD_WR_REG(0x29);    //Display on 
}
void TFT_LCD_Clear(u16 Color)
{
	u16 i,j;  	
	Address_set(0,0,TFT_LCD_W-1,TFT_LCD_H-1);
  for(i=0;i<TFT_LCD_W;i++)
  {
		for (j=0;j<TFT_LCD_H;j++)
		{
				TFT_LCD_WR_DATA(Color);	 			 
		}
	 }
}
