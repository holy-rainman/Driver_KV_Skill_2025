/*
 * Program to control a 4-digit 7-segment display with multiplexing.
 * The display is used for showing numeric and character values (e.g., "ELEC").
 * The program utilizes active-low logic for reading button inputs and displays
 * numbers or characters using a common anode 7-segment display.
 *
 * Key Variables:
 * - num[]: Array representing numeric values (0-9) for the 7-segment display.
 * - alp[]: Array representing characters (E, L, C) for the 7-segment display.
 * - seg[]: Array representing segment selection for multiplexing.
 * - buff[]: Buffer array holding the 7-segment data to be displayed.
 * - ptr2disp: Pointer to the digit to display.
 *
 * Functions:
 * - getPB(): Reads the states of three push buttons and returns their values.
 * - displaySegment(): Multiplexes the 7-segment display for showing values.
 * - multiplexSegment(): Converts a 4-digit number to be displayed on the 7-segment.
 * - multiplexChar(): Displays a character string (e.g., "ELEC") on the display.
 * - multiplexClear(): Clears the 7-segment display by turning off all segments.
 */

#include <KVSkillsdriver2025.h>

// Array data & variables used for 7-segment Common Anode - numeric value only
//                  INDEX: 0    1    2    3    4    5    6    7    8    9    10
//                    NUM: 0    1    2    3    4    5    6    7    8    9    BLNK
const uint8_t num[] 	= {0xC0,0XF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,0x80,0x90,0xFF};
//                   CHAR: OFF  E    L    C
const uint8_t alp[]		= {0xFF,0x86,0xC7,0xC6};
const uint8_t seg[4]	= {0xF1,0xF2,0xF4,0xF8};
uint16_t ptr2disp;
uint16_t buff[4];
char extern text;

uint8_t getPB(void)
{	uint8_t PBvalue=0;
	if(S1==LOW)	PBvalue |= 1<<0;
	if(S2==LOW)	PBvalue |= 1<<1;
	if(S3==LOW)	PBvalue |= 1<<2;
	return PBvalue;
}
void displaySegment(void)
{	static uint8_t seg=0;

	stCLK(LOW);
	for(int8_t i=15;i>-1;i--)
	{	shCLK(LOW);
		if(buff[seg] & (1<<i))	dataS(HIGH);
		else					dataS(LOW);
		shCLK(HIGH);
	}
	stCLK(HIGH);
	if(++seg>3) seg=0;
}
void multiplexSegment(uint16_t num4digit)
{	for(uint8_t i=0;i<4;i++)
	{	switch(i)
		{	case 0:
				ptr2disp = num4digit / 1000;
				break;
			case 1:
				ptr2disp = (num4digit % 1000) / 100;
				break;
			case 2:
				ptr2disp = (num4digit % 100) / 10;
				break;
			case 3:
				ptr2disp = num4digit % 10;
				break;
		}
		buff[i] = (num[ptr2disp]<<8) | seg[i];
	}
}
void multiplexChar(const char* text)
{	if(strcmp(text,"ELEC")==0)
	{	buff[0]=(alp[1]<<8)|seg[0];
		buff[1]=(alp[2]<<8)|seg[1];
		buff[2]=(alp[1]<<8)|seg[2];
		buff[3]=(alp[3]<<8)|seg[3];
	}
}
void multiplexClear()
{	buff[0]=(alp[0]<<8)|seg[0];
	buff[1]=(alp[0]<<8)|seg[1];
	buff[2]=(alp[0]<<8)|seg[2];
	buff[3]=(alp[0]<<8)|seg[3];
}
