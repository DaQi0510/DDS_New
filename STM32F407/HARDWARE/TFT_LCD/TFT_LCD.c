#include "TFT_LCD.h"
#include "delay.h"
extern volatile  u16 BACK_COLOR, POINT_COLOR;   //背景色，画笔色
const u8 asc2_1608[3120]={
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0x00,0x98,0x01,0x0C,0x03,
0x0C,0x03,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,
0x06,0x06,0x0C,0x03,0x0C,0x03,0x98,0x01,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"0",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x60,0x00,0x7C,0x00,
0x60,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0x60,0x00,
0x60,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0xFC,0x03,0x00,0x00,0x00,0x00,0x00,0x00,/*"1",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF8,0x00,0x84,0x01,0x02,0x03,
0x06,0x03,0x06,0x03,0x00,0x03,0x80,0x01,0x80,0x01,0xC0,0x00,0x20,0x00,0x10,0x00,
0x08,0x02,0x04,0x02,0x02,0x02,0xFE,0x03,0xFE,0x03,0x00,0x00,0x00,0x00,0x00,0x00,/*"2",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x78,0x00,0xC4,0x00,0x86,0x01,
0x86,0x01,0x86,0x01,0x80,0x01,0xC0,0x00,0x70,0x00,0x80,0x01,0x00,0x01,0x00,0x03,
0x06,0x03,0x06,0x03,0x06,0x03,0x84,0x01,0xF8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"3",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x80,0x01,0xC0,0x01,0xC0,0x01,
0xA0,0x01,0x90,0x01,0x90,0x01,0x88,0x01,0x84,0x01,0x84,0x01,0x82,0x01,0xFE,0x07,
0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0xE0,0x07,0x00,0x00,0x00,0x00,0x00,0x00,/*"4",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFC,0x03,0xFC,0x03,0x04,0x00,
0x04,0x00,0x04,0x00,0x04,0x00,0xF4,0x00,0x8C,0x01,0x04,0x03,0x00,0x03,0x00,0x03,
0x06,0x03,0x06,0x03,0x82,0x01,0x84,0x01,0xF8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"5",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0x01,0x18,0x03,0x0C,0x03,
0x0C,0x00,0x04,0x00,0x06,0x00,0xE6,0x01,0x16,0x03,0x0E,0x06,0x06,0x06,0x06,0x06,
0x06,0x06,0x04,0x06,0x0C,0x02,0x18,0x03,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"6",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF8,0x07,0xFC,0x07,0x0C,0x02,
0x04,0x01,0x04,0x01,0x00,0x01,0x80,0x00,0x80,0x00,0x40,0x00,0x40,0x00,0x40,0x00,
0x60,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"7",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF8,0x01,0x0C,0x03,0x06,0x06,
0x06,0x06,0x06,0x06,0x0E,0x02,0x3C,0x03,0xF0,0x00,0xCC,0x01,0x04,0x03,0x06,0x06,
0x06,0x06,0x06,0x06,0x06,0x06,0x0C,0x03,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x00,/*"8",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0x00,0x0C,0x01,0x0C,0x03,
0x06,0x02,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x07,0x8C,0x06,0x78,0x06,0x00,0x06,
0x00,0x03,0x00,0x03,0x0C,0x01,0x8C,0x01,0x78,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"9",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x60,0x00,0x60,0x00,0x70,0x00,
0xD0,0x00,0xD0,0x00,0xC8,0x00,0x88,0x01,0x88,0x01,0x88,0x01,0xF8,0x01,0x04,0x03,
0x04,0x03,0x04,0x03,0x04,0x06,0x06,0x06,0x0F,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,/*"A",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0x86,0x01,0x06,0x03,
0x06,0x03,0x06,0x03,0x06,0x03,0x86,0x01,0xFE,0x00,0x06,0x03,0x06,0x02,0x06,0x06,
0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x03,0xFF,0x01,0x00,0x00,0x00,0x00,0x00,0x00,/*"B",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0x07,0x18,0x06,0x0C,0x04,
0x0C,0x04,0x04,0x00,0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x00,
0x06,0x04,0x0C,0x04,0x0C,0x02,0x18,0x01,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"C",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7F,0x00,0x86,0x01,0x06,0x03,
0x06,0x03,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,
0x06,0x06,0x06,0x03,0x06,0x03,0xC6,0x01,0x7F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"D",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x03,0x06,0x02,0x06,0x04,
0x06,0x00,0x06,0x00,0x86,0x00,0x86,0x00,0xFE,0x00,0x86,0x00,0x86,0x00,0x06,0x00,
0x06,0x00,0x06,0x04,0x06,0x04,0x06,0x02,0xFF,0x03,0x00,0x00,0x00,0x00,0x00,0x00,/*"E",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x03,0x06,0x03,0x06,0x04,
0x06,0x04,0x06,0x00,0x86,0x00,0x86,0x00,0xFE,0x00,0x86,0x00,0x86,0x00,0x06,0x00,
0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x00,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"F",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0x02,0x18,0x03,0x0C,0x02,
0x0C,0x02,0x04,0x00,0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x00,0xC6,0x0F,0x06,0x03,
0x06,0x03,0x0C,0x03,0x0C,0x03,0x18,0x03,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"G",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0F,0x0F,0x06,0x06,0x06,0x06,
0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0xFE,0x07,0x06,0x06,0x06,0x06,0x06,0x06,
0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x0F,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,/*"H",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFC,0x03,0x60,0x00,0x60,0x00,
0x60,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0x60,0x00,
0x60,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0xFC,0x03,0x00,0x00,0x00,0x00,0x00,0x00,/*"I",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0x0F,0x80,0x01,0x80,0x01,
0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,
0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x86,0x01,0xC6,0x00,0x7C,0x00,/*"J",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xCF,0x07,0x06,0x01,0x86,0x00,
0x46,0x00,0x46,0x00,0x26,0x00,0x36,0x00,0x3E,0x00,0x6E,0x00,0xE6,0x00,0xC6,0x00,
0xC6,0x01,0x86,0x01,0x06,0x03,0x06,0x07,0x8F,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,/*"K",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0F,0x00,0x06,0x00,0x06,0x00,
0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x00,
0x06,0x00,0x06,0x04,0x06,0x04,0x06,0x02,0xFF,0x03,0x00,0x00,0x00,0x00,0x00,0x00,/*"L",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0F,0x0F,0x0E,0x07,0x0E,0x07,
0x0E,0x07,0x9A,0x06,0x9A,0x06,0x9A,0x06,0x9A,0x06,0x5A,0x06,0x72,0x06,0x72,0x06,
0x72,0x06,0x72,0x06,0x22,0x06,0x22,0x06,0x27,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,/*"M",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x0E,0x0E,0x04,0x0E,0x04,
0x1A,0x04,0x1A,0x04,0x32,0x04,0x62,0x04,0x62,0x04,0xC2,0x04,0xC2,0x04,0x82,0x05,
0x02,0x07,0x02,0x07,0x02,0x06,0x02,0x06,0x07,0x04,0x00,0x00,0x00,0x00,0x00,0x00,/*"N",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0x00,0x98,0x01,0x0C,0x03,
0x0C,0x02,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,
0x06,0x06,0x0C,0x02,0x0C,0x03,0x98,0x01,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"O",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x01,0x06,0x03,0x06,0x06,
0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x03,0xFE,0x01,0x06,0x00,0x06,0x00,
0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x00,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"P",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0x00,0x98,0x01,0x0C,0x03,
0x0C,0x02,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,
0x76,0x06,0x4C,0x02,0x8C,0x03,0x88,0x01,0xF0,0x01,0x80,0x07,0x00,0x03,0x00,0x00,/*"Q",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x01,0x06,0x03,0x06,0x06,
0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x03,0xFE,0x00,0x66,0x00,0xC6,0x00,0xC6,0x00,
0x86,0x01,0x86,0x01,0x06,0x03,0x06,0x03,0x0F,0x0E,0x00,0x00,0x00,0x00,0x00,0x00,/*"R",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF8,0x04,0x0C,0x07,0x06,0x04,
0x06,0x04,0x06,0x00,0x0E,0x00,0x3C,0x00,0xF0,0x00,0xC0,0x03,0x00,0x03,0x00,0x06,
0x02,0x06,0x02,0x06,0x06,0x06,0x0E,0x03,0xF2,0x01,0x00,0x00,0x00,0x00,0x00,0x00,/*"S",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0x07,0x62,0x04,0x61,0x08,
0x61,0x08,0x60,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0x60,0x00,
0x60,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"T",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0F,0x0E,0x06,0x04,0x06,0x04,
0x06,0x04,0x06,0x04,0x06,0x04,0x06,0x04,0x06,0x04,0x06,0x04,0x06,0x04,0x06,0x04,
0x06,0x04,0x06,0x04,0x06,0x04,0x0C,0x02,0xF8,0x01,0x00,0x00,0x00,0x00,0x00,0x00,/*"U",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0x0F,0x0E,0x06,0x0C,0x02,
0x0C,0x02,0x0C,0x02,0x0C,0x01,0x18,0x01,0x18,0x01,0x18,0x01,0x18,0x01,0xB0,0x00,
0xB0,0x00,0xB0,0x00,0xF0,0x00,0x60,0x00,0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"V",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xEF,0x0E,0x66,0x04,0x66,0x04,
0x66,0x02,0x66,0x02,0xE6,0x02,0xEC,0x02,0xEC,0x02,0xDC,0x01,0xDC,0x01,0xDC,0x01,
0xDC,0x01,0x9C,0x01,0x88,0x00,0x88,0x00,0x88,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"W",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x9E,0x07,0x0C,0x03,0x18,0x01,
0x18,0x01,0x98,0x00,0xB0,0x00,0x70,0x00,0x60,0x00,0x60,0x00,0xE0,0x00,0xD0,0x00,
0xD0,0x01,0x98,0x01,0x88,0x01,0x0C,0x03,0x9E,0x07,0x00,0x00,0x00,0x00,0x00,0x00,/*"X",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0x0F,0x0E,0x06,0x0C,0x02,
0x0C,0x01,0x18,0x01,0x18,0x01,0xB0,0x00,0xB0,0x00,0x70,0x00,0x60,0x00,0x60,0x00,
0x60,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0xF8,0x01,0x00,0x00,0x00,0x00,0x00,0x00,/*"Y",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFC,0x07,0x04,0x03,0x82,0x03,
0x80,0x01,0xC0,0x01,0xC0,0x00,0xC0,0x00,0x60,0x00,0x60,0x00,0x30,0x00,0x30,0x00,
0x18,0x00,0x18,0x04,0x1C,0x04,0x0C,0x02,0xFE,0x03,0x00,0x00,0x00,0x00,0x00,0x00,/*"Z",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0xF0,0x01,0x0C,0x03,0x0C,0x03,0xE0,0x03,0x38,0x03,0x0C,0x03,
0x06,0x03,0x06,0x03,0x06,0x03,0x8E,0x0B,0x7C,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,/*"a",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x00,0x0E,0x00,0x0C,0x00,0x0C,0x00,
0x0C,0x00,0x0C,0x00,0xCC,0x01,0x3C,0x03,0x1C,0x06,0x0C,0x06,0x0C,0x06,0x0C,0x06,
0x0C,0x06,0x0C,0x06,0x0C,0x02,0x1C,0x03,0xF4,0x01,0x00,0x00,0x00,0x00,0x00,0x00,/*"b",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0xF0,0x00,0x8C,0x01,0x8C,0x01,0x86,0x01,0x06,0x00,0x06,0x00,
0x06,0x00,0x06,0x02,0x0C,0x02,0x0C,0x01,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"c",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x80,0x03,0x00,0x03,0x00,0x03,
0x00,0x03,0x00,0x03,0x78,0x03,0x8C,0x03,0x0C,0x03,0x06,0x03,0x06,0x03,0x06,0x03,
0x06,0x03,0x06,0x03,0x04,0x03,0x8C,0x07,0x78,0x01,0x00,0x00,0x00,0x00,0x00,0x00,/*"d",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0xE0,0x01,0x18,0x03,0x08,0x06,0x0C,0x06,0xFC,0x07,0x0C,0x00,
0x0C,0x00,0x0C,0x00,0x18,0x04,0x38,0x02,0xE0,0x01,0x00,0x00,0x00,0x00,0x00,0x00,/*"e",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xC0,0x03,0x60,0x06,0x30,0x06,
0x30,0x00,0x30,0x00,0xFE,0x03,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,
0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0xFC,0x01,0x00,0x00,0x00,0x00,0x00,0x00,/*"f",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0xF0,0x0E,0x98,0x09,0x0C,0x03,0x0C,0x03,0x0C,0x03,0x98,0x01,
0xF8,0x00,0x0C,0x00,0x7C,0x00,0xF8,0x03,0x06,0x06,0x06,0x06,0x0E,0x07,0xF8,0x01,/*"g",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x00,0x0E,0x00,0x0C,0x00,0x0C,0x00,
0x0C,0x00,0x0C,0x00,0xEC,0x01,0x1C,0x03,0x0C,0x03,0x0C,0x03,0x0C,0x03,0x0C,0x03,
0x0C,0x03,0x0C,0x03,0x0C,0x03,0x0C,0x03,0x9E,0x07,0x00,0x00,0x00,0x00,0x00,0x00,/*"h",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x60,0x00,0x60,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x7C,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0x60,0x00,
0x60,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0xFC,0x03,0x00,0x00,0x00,0x00,0x00,0x00,/*"i",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x01,0x80,0x01,0x00,0x00,
0x00,0x00,0x00,0x00,0xF0,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,
0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0xCC,0x00,0x7C,0x00,/*"j",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x00,0x0E,0x00,0x0C,0x00,0x0C,0x00,
0x0C,0x00,0x0C,0x00,0xCC,0x03,0x8C,0x00,0xCC,0x00,0x4C,0x00,0x6C,0x00,0x7C,0x00,
0xDC,0x00,0xCC,0x00,0x8C,0x01,0x8C,0x01,0x9E,0x07,0x00,0x00,0x00,0x00,0x00,0x00,/*"k",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x7C,0x00,0x60,0x00,0x60,0x00,
0x60,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0x60,0x00,
0x60,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0xFC,0x03,0x00,0x00,0x00,0x00,0x00,0x00,/*"l",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x77,0x07,0xEE,0x06,0x66,0x06,0x66,0x06,0x66,0x06,0x66,0x06,
0x66,0x06,0x66,0x06,0x66,0x06,0x66,0x06,0xEF,0x0E,0x00,0x00,0x00,0x00,0x00,0x00,/*"m",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0xCE,0x01,0x3C,0x03,0x0C,0x03,0x0C,0x03,0x0C,0x03,0x0C,0x03,
0x0C,0x03,0x0C,0x03,0x0C,0x03,0x0C,0x03,0x9E,0x07,0x00,0x00,0x00,0x00,0x00,0x00,/*"n",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0xF0,0x00,0x98,0x01,0x0C,0x03,0x06,0x06,0x06,0x06,0x06,0x06,
0x06,0x06,0x06,0x06,0x0C,0x03,0x0C,0x03,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"o",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0xEE,0x01,0x1C,0x03,0x0C,0x06,0x0C,0x06,0x0C,0x06,0x0C,0x06,
0x0C,0x06,0x0C,0x06,0x0C,0x03,0x1C,0x03,0xEC,0x01,0x0C,0x00,0x0C,0x00,0x3E,0x00,/*"p",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x78,0x02,0x8C,0x03,0x0C,0x03,0x06,0x03,0x06,0x03,0x06,0x03,
0x06,0x03,0x06,0x03,0x04,0x03,0x8C,0x03,0x78,0x03,0x00,0x03,0x00,0x03,0xC0,0x07,/*"q",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x9F,0x07,0x58,0x06,0x38,0x00,0x18,0x00,0x18,0x00,0x18,0x00,
0x18,0x00,0x18,0x00,0x18,0x00,0x18,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"r",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0xF0,0x07,0x18,0x06,0x0C,0x04,0x0C,0x00,0x38,0x00,0xF0,0x01,
0x80,0x03,0x04,0x06,0x04,0x06,0x0C,0x03,0xFC,0x01,0x00,0x00,0x00,0x00,0x00,0x00,/*"s",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x20,0x00,0x20,0x00,
0x30,0x00,0x30,0x00,0xFE,0x01,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,
0x30,0x00,0x30,0x00,0x30,0x02,0x30,0x02,0xE0,0x01,0x00,0x00,0x00,0x00,0x00,0x00,/*"t",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x08,0x02,0x8E,0x03,0x0C,0x03,0x0C,0x03,0x0C,0x03,0x0C,0x03,0x0C,0x03,
0x0C,0x03,0x0C,0x03,0x0C,0x03,0x9C,0x07,0x78,0x01,0x00,0x00,0x00,0x00,0x00,0x00,/*"u",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x3E,0x0F,0x1C,0x06,0x18,0x02,0x18,0x02,0x30,0x01,0x30,0x01,
0x30,0x01,0xE0,0x00,0xE0,0x00,0xE0,0x00,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"v",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0xEF,0x0D,0xC6,0x04,0xC6,0x04,0xE6,0x04,0xEC,0x04,0xAC,0x03,
0x9C,0x03,0x9C,0x03,0x9C,0x03,0x08,0x01,0x08,0x01,0x00,0x00,0x00,0x00,0x00,0x00,/*"w",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0xBE,0x07,0x18,0x01,0x98,0x01,0xB0,0x00,0x70,0x00,0x60,0x00,
0xE0,0x00,0xD0,0x00,0x98,0x01,0x88,0x03,0xDE,0x07,0x00,0x00,0x00,0x00,0x00,0x00,/*"x",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0xBE,0x07,0x1C,0x01,0x18,0x01,0x18,0x01,0xB0,0x00,0xB0,0x00,
0xB0,0x00,0x60,0x00,0x60,0x00,0x40,0x00,0x20,0x00,0x20,0x00,0x14,0x00,0x1C,0x00,/*"y",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0xFC,0x03,0x84,0x01,0xC4,0x01,0xC0,0x00,0xE0,0x00,0x60,0x00,
0x70,0x00,0x30,0x04,0x38,0x04,0x18,0x06,0xFC,0x03,0x00,0x00,0x00,0x00,0x00,0x00,/*"z",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x1C,0x00,0x1C,0x00,0x1C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*".",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0E,0x03,0x0A,0x01,0x11,0x01,
0x91,0x00,0x91,0x00,0xD1,0x00,0x51,0x00,0xFA,0x03,0xAE,0x02,0x60,0x04,0x50,0x04,
0x50,0x04,0x58,0x04,0x48,0x04,0x88,0x02,0x84,0x03,0x00,0x00,0x00,0x00,0x00,0x00,/*"%",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*" ",0*/
}; 
const u8 Char[960]={
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0x27,0x30,0x38,
0x18,0x30,0x0C,0x20,0x0C,0x20,0x0C,0x00,0x0C,0x00,0x18,0x00,0x78,0x00,0xE0,0x03,
0x80,0x0F,0x00,0x1E,0x00,0x38,0x00,0x70,0x00,0x60,0x04,0x60,0x04,0x60,0x08,0x60,
0x18,0x30,0x38,0x18,0xC8,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"S",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7E,0x7C,0x18,0x18,
0x18,0x08,0x18,0x04,0x18,0x06,0x18,0x02,0x18,0x01,0x98,0x01,0x98,0x01,0xD8,0x01,
0xB8,0x03,0x38,0x03,0x18,0x07,0x18,0x06,0x18,0x0E,0x18,0x0C,0x18,0x1C,0x18,0x18,
0x18,0x30,0x18,0x30,0x7E,0xFC,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"K",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7E,0x7C,0x1C,0x10,
0x18,0x10,0x18,0x08,0x30,0x08,0x30,0x0C,0x70,0x04,0x60,0x04,0x60,0x02,0xC0,0x02,
0xC0,0x02,0xC0,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,
0x80,0x01,0x80,0x01,0xE0,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"Y",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7E,0x00,0x18,0x00,
0x18,0x00,0x18,0x00,0x18,0x00,0x18,0x00,0x18,0x00,0x18,0x00,0x18,0x00,0x18,0x00,
0x18,0x00,0x18,0x00,0x18,0x00,0x18,0x00,0x18,0x00,0x18,0x00,0x18,0x40,0x18,0x40,
0x18,0x20,0x18,0x30,0xFE,0x3F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"L",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0xC0,0x01,0xC0,0x01,
0xC0,0x01,0x40,0x01,0x60,0x03,0x20,0x03,0x20,0x03,0x20,0x03,0x30,0x06,0x10,0x06,
0x10,0x06,0x10,0x06,0xF8,0x0F,0x08,0x0C,0x08,0x0C,0x08,0x0C,0x0C,0x0C,0x04,0x18,
0x04,0x18,0x06,0x18,0x1F,0x7C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"A",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0x3F,0x18,0x30,
0x18,0x20,0x18,0x60,0x18,0x40,0x18,0x00,0x18,0x08,0x18,0x08,0x18,0x0C,0xF8,0x0F,
0x18,0x0C,0x18,0x08,0x18,0x08,0x18,0x00,0x18,0x00,0x18,0x00,0x18,0x40,0x18,0x40,
0x18,0x20,0x18,0x30,0xFE,0x3F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"E",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0x07,0x18,0x1C,
0x18,0x38,0x18,0x30,0x18,0x30,0x18,0x30,0x18,0x30,0x18,0x18,0x18,0x0C,0xF8,0x07,
0x98,0x03,0x18,0x03,0x18,0x07,0x18,0x06,0x18,0x06,0x18,0x0E,0x18,0x0C,0x18,0x0C,
0x18,0x1C,0x18,0x18,0x7E,0x78,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"R",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xC0,0x27,0x60,0x38,
0x10,0x30,0x18,0x20,0x0C,0x40,0x0C,0x40,0x04,0x00,0x06,0x00,0x06,0x00,0x06,0x00,
0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x00,0x0C,0x40,0x0C,0x40,0x0C,0x20,
0x18,0x30,0x30,0x18,0xC0,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"C",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0xF8,0x01,0x80,0x01,
0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,
0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,
0x80,0x01,0x80,0x01,0xF8,0x1F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"l",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xC0,0x07,0x30,0x0C,0x18,0x18,
0x08,0x10,0x0C,0x30,0x0C,0x30,0xFC,0x3F,0x0C,0x00,0x0C,0x00,0x0C,0x00,0x18,0x20,
0x18,0x10,0x70,0x18,0xC0,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"e",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0x03,0x18,0x06,0x0C,0x0C,
0x0C,0x0C,0x00,0x0C,0x80,0x0F,0x70,0x0C,0x1C,0x0C,0x0C,0x0C,0x06,0x0C,0x06,0x0C,
0x06,0x4C,0x0C,0x4F,0xF8,0x38,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"a",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x10,0x00,0x1E,0x0F,0xD8,0x18,0x38,0x30,
0x18,0x30,0x18,0x30,0x18,0x30,0x18,0x30,0x18,0x30,0x18,0x30,0x18,0x30,0x18,0x30,
0x18,0x30,0x18,0x30,0x7E,0xFC,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"n",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x03,0x80,0x03,
0x80,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0xF8,0x01,0x80,0x01,0x80,0x01,
0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,
0x80,0x01,0x80,0x01,0xF8,0x1F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"i",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xC0,0x77,0x30,0x6C,0x10,0x18,
0x18,0x18,0x18,0x18,0x18,0x18,0x10,0x18,0x30,0x0C,0xF0,0x07,0x18,0x00,0x18,0x00,
0xF0,0x0F,0xF0,0x3F,0x08,0x70,0x0C,0x60,0x0C,0x60,0x0C,0x60,0x38,0x38,0xE0,0x0F,/*"g",0*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*     */
};


void TFT_LCD_Writ_Bus(char dat)   //串行数据写入
{	
//	 while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET){}//等待发送区空  
//	 SPI_I2S_SendData(SPI2, dat); //通过外设SPIx发送一个byte  数据
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
void TFT_LCD_WR_DATA8(char da) //发送数据8位
{	
	TFT_LCD_DC_Set ;
	TFT_LCD_Writ_Bus(da);  
} 
void TFT_LCD_WR_DATA(int da)   //发送数据16位
{	
	TFT_LCD_DC_Set ;
	TFT_LCD_Writ_Bus(da>>8);  
  TFT_LCD_Writ_Bus(da);
}	

void TFT_LCD_WR_REG(char da)	//写寄存器
{	
	TFT_LCD_DC_Clr ;
	TFT_LCD_Writ_Bus(da);
}
void LCD_WR_REG_DATA(int reg,int da)  //在寄存器里面写数据
{
	TFT_LCD_WR_REG(reg);
	TFT_LCD_WR_DATA(da);
}
void Address_set(unsigned int x1,unsigned int y1,unsigned int x2,unsigned int y2)   //设置地址
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

void TFT_LCD_Init(void)   //初始化液晶显示
{
	u16 i,j;
	//SPI初始化
	GPIO_InitTypeDef  GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOA时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOA时钟
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);//使能SPI2时钟
	
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//RST对应IO口
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉
//	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIO
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;//PB3~5复用功能输出	
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
//  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PB3~5复用功能输出	
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
//  GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化
//	
//	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI2); //PB3复用为 SPI2
//	GPIO_PinAFConfig(GPIOC,GPIO_PinSource3,GPIO_AF_SPI2); //PB4复用为 SPI2
//	
//	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,ENABLE);//复位SPI1
//	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,DISABLE);//停止复位SPI1

//	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
//	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
//	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收8位帧结构
//	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//串行同步时钟的空闲状态为高电平
//	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//串行同步时钟的第二个跳变沿（上升或下降）数据被采样
//	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
//	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;		//定义波特率预分频的值:波特率预分频值为256
//	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
//	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
//	SPI_Init(SPI2, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
//	
//	SPI_Cmd(SPI2, ENABLE); //使能SPI外设
  TFT_LCD_BLK_Set;
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
	for(i=0;i<5000;i++)
		for(j=0;j<200;j++);
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
void TFT_LCD_ShowChar(u16 x,u16 y,u8 Num)
{
	u8 temp;
	u8 pos,t;
	u16 x0;
	u16 colortemp=POINT_COLOR;
  x0=x;
	if(Num>='0'&&Num<='9')
		Num=Num-48;
	else if(Num>='A'&&Num<='Z')
		Num=Num-55;
	else if(Num>='a'&&Num<='z')
		Num=Num-61;
	else if(Num==' ')
		Num=64;
	else if(Num=='.')
		Num=62;
	else
		Num=63;
	Address_set(x-23,y-15,x,y);      //设置光标位置 
	for(t=0;t<8;t++)
	{
		for(pos=0;pos<24;pos++)
		{ 
			temp=asc2_1608[Num*48+46-2*pos];		 //调用1608字体                
			if(temp&(0x01<<t))POINT_COLOR=colortemp;
			else POINT_COLOR=BACK_COLOR;
			TFT_LCD_WR_DATA(POINT_COLOR);	
		}
	}
	for(t=0;t<8;t++)
	{
		for(pos=0;pos<24;pos++)
		{ 
			temp=asc2_1608[Num*48+47-2*pos];		 //调用1608字体                
			if(temp&(0x01<<t))POINT_COLOR=colortemp;
			else POINT_COLOR=BACK_COLOR;
			TFT_LCD_WR_DATA(POINT_COLOR);	
		}
	}	
	POINT_COLOR=colortemp;	 	
}
void TFT_LCD_ShowString(u16 x,u16 y,const u8 *p)
{
	u16 x0,y0;  
	y0=x;
	x0=240-y;
	while(*p!='\0')
	{       
			TFT_LCD_ShowChar(x0,y0,*p);
			y0+=12;
			p++;
	}  
}
void TFT_LCD_Display(void)
{
	u16 Addressy1,Addressy2,Addressy3,Addressy4,Addressy5,Addressy6,Addressy7;
	Addressy1=30;
	Addressy2=62;
	Addressy3=97;
	Addressy4=132;
	Addressy5=167;
	Addressy6=202;
	Addressy7=212;
	TFT_LCD_ShowTop();
//	TFT_LCD_ShowString(48,Addressy1,"SKY LASER Cleaning");
	TFT_LCD_ShowString(60,Addressy2,"Laser Power");
	TFT_LCD_ShowString(60,Addressy3,"Pulse");
	TFT_LCD_ShowString(60,Addressy4,"Red");
	TFT_LCD_ShowString(60,Addressy5,"Laser");
	TFT_LCD_ShowString(60,Addressy6,"Mode");
//	TFT_LCD_ShowString(60,Addressy7,"Warning");
	
}
void TFT_LCD_ShowChars(u16 x,u16 y,u8 Num)
{

	u8 temp;
	u8 pos,t;
	u16 x0;
	u16 colortemp=POINT_COLOR;
  x0=x;
	Address_set(x-31,y-15,x,y);      //设置光标位置 
	for(t=0;t<8;t++)
	{
		for(pos=0;pos<32;pos++)
		{ 
			temp=Char[Num*64+62-2*pos];		 //调用1608字体                
			if(temp&(0x01<<t))POINT_COLOR=colortemp;
			else POINT_COLOR=BACK_COLOR;
			TFT_LCD_WR_DATA(POINT_COLOR);	
		}
	}
	for(t=0;t<8;t++)
	{
		for(pos=0;pos<32;pos++)
		{ 
			temp=Char[Num*64+63-2*pos];		 //调用1608字体                
			if(temp&(0x01<<t))POINT_COLOR=colortemp;
			else POINT_COLOR=BACK_COLOR;
			TFT_LCD_WR_DATA(POINT_COLOR);	
		}
	}	
	
	POINT_COLOR=colortemp;	 
}
void TFT_LCD_ShowTop(void)
{
	u8 i;
	u16 x,y,x0,y0;
	
	//          S K Y    L A S E R   C l e  a n  i  n  g
	u8 Arr[18]={0,1,2,14,3,4,0,5,6,14,7,8,9,10,11,12,11,13};
	x=28,y=15;
	y0=x;
	x0=240-y;
	for(i=0;i<18;i++)
	{
		TFT_LCD_ShowChars(x0,y0,Arr[i]);
		y0=y0+16;
	}
}
void TFT_LCD_ShowADNum(u8 Value)
{
	u16 Addressx1=215;     
	u16 Addressy1=62;
	u16 x,y;
	u16 Percent;
	u8 N;
	Percent =1000*Value/255;
	if(Percent<10)
	{
		y=Addressx1;
		x=240-Addressy1;
		TFT_LCD_ShowChar(x,y,'0');
		y+=12;
		TFT_LCD_ShowChar(x,y,'.');
		y+=12;
		TFT_LCD_ShowChar(x,y,48+Percent);
		y+=12;
		TFT_LCD_ShowChar(x,y,'%');
		y+=12;
		TFT_LCD_ShowChar(x,y,' ');
		y+=12;
		TFT_LCD_ShowChar(x,y,' ');
	}
	if((Percent>=10)&&(Percent<100))
	{
		y=Addressx1;
		x=240-Addressy1;
		N=Percent/10;
		TFT_LCD_ShowChar(x,y,N+48);
		y+=12;
		TFT_LCD_ShowChar(x,y,'.');
		y+=12;
		N=Percent%10;
		TFT_LCD_ShowChar(x,y,48+N);
		y+=12;
		TFT_LCD_ShowChar(x,y,'%');
		y+=12;
		TFT_LCD_ShowChar(x,y,' ');
		TFT_LCD_ShowChar(x,y,' ');
	}
	if((Percent>=100)&&(Percent<1000))
	{
		y=Addressx1;
		x=240-Addressy1;
		N=Percent/100;
		TFT_LCD_ShowChar(x,y,N+48);
		y+=12;
		N=(Percent%100)/10;
		TFT_LCD_ShowChar(x,y,N+48);
		y+=12;
		TFT_LCD_ShowChar(x,y,'.');
		y+=12;
		N=Percent%10;
		TFT_LCD_ShowChar(x,y,48+N);
		y+=12;
		TFT_LCD_ShowChar(x,y,'%');
		y+=12;
		TFT_LCD_ShowChar(x,y,' ');
	}
	if(Percent==1000)
	{
		y=Addressx1;
		x=240-Addressy1;
		TFT_LCD_ShowChar(x,y,1+48);
		y+=12;
		TFT_LCD_ShowChar(x,y,48);
		y+=12;
		TFT_LCD_ShowChar(x,y,48);
		y+=12;
		TFT_LCD_ShowChar(x,y,'.');
		y+=12;
		TFT_LCD_ShowChar(x,y,48);
		y+=12;
		TFT_LCD_ShowChar(x,y,'%');
	}
}
void TFT_LCD_ShowPulse(u16 Value)
{
	u16 Addressx1=215;
	u16 Addressy1=97;
	u16 x,y;
	u8 N;
	y=Addressx1;
	x=240-Addressy1;
	if(Value<100)
	{
		N=Value/10;
		TFT_LCD_ShowChar(x,y,N+48);
		y+=12;
		N=Value%10;
		TFT_LCD_ShowChar(x,y,N+48);
//		y+=12;
//		TFT_LCD_ShowChar(x,y,'k');
//		y+=12;
//		TFT_LCD_ShowChar(x,y,'H');
//		y+=12;
//		TFT_LCD_Sh v vowChar(x,y,'z');
		y+=12;
		TFT_LCD_ShowChar(x,y,' ');
	}
	if(Value>=100)
	{
		N=Value/100;
		TFT_LCD_ShowChar(x,y,N+48);
		y+=12;
		N=(Value%100)/10;
		TFT_LCD_ShowChar(x,y,N+48);
		y+=12;
		N=Value%10;
		TFT_LCD_ShowChar(x,y,N+48);
		y+=12;
//		TFT_LCD_ShowChar(x,y,'k');
//		y+=12;
//		TFT_LCD_ShowChar(x,y,'H');
//		y+=12;
//		TFT_LCD_ShowChar(x,y,'z');
	}
}
void TFT_LCD_ShowRed(u8 Value)
{
	u16 Addressx1=215;
	u16 Addressy1=132;
	u16 x,y;
	y=Addressx1;
	x=240-Addressy1;
	if(Value==1)
	{
		TFT_LCD_ShowChar(x,y,'o');
		y+=12;
		TFT_LCD_ShowChar(x,y,'n');
		y+=12;
		TFT_LCD_ShowChar(x,y,' ');
	}
	else
	{
		TFT_LCD_ShowChar(x,y,'o');
		y+=12;
		TFT_LCD_ShowChar(x,y,'f');
		y+=12;
		TFT_LCD_ShowChar(x,y,'f');
	}
}
void TFT_LCD_ShowLaser(u8 Value)
{
	u16 Addressx1=215;
	u16 Addressy1=167;
	u16 x,y;
	y=Addressx1;
	x=240-Addressy1;
	if(Value==1)
	{
		TFT_LCD_ShowChar(x,y,'o');
		y+=12;
		TFT_LCD_ShowChar(x,y,'n');
		y+=12;
		TFT_LCD_ShowChar(x,y,' ');
	}
	else
	{
		TFT_LCD_ShowChar(x,y,'o');
		y+=12;
		TFT_LCD_ShowChar(x,y,'f');
		y+=12;
		TFT_LCD_ShowChar(x,y,'f');
	}
}
void TFT_LCD_ShowMode(u8 Value)
{
	u16 Addressx1=215;
	u16 Addressy1=202;
	u16 x,y;
	y=Addressx1;
	x=240-Addressy1;
	TFT_LCD_ShowChar(x,y,49+Value);
}
void TFT_LCD_ShowWarning(u8 Value)
{
	u16 Addressx1=215;
	u16 Addressy1=212;
	u16 x,y;
	y=Addressx1;
	x=240-Addressy1;
	if(Value==1)
		TFT_LCD_ShowChar(x,y,48);	
	else
		TFT_LCD_ShowChar(x,y,49);	
}