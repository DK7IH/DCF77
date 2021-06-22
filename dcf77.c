/*****************************************************************/
/*                   DCF77 Time signal receiver                  */
/*  ************************************************************ */
/*  MUC:              ATMEL AVR ATmega8, 16 MHz                  */
/*  DISPLAY:          1106 OLED                                  */
/*  Compiler:         GCC (GNU AVR C-Compiler)                   */
/*  Author:           Peter Rachow (DK7IH)                       */
/*  Last change:      JUN 2021                                   */
/*****************************************************************/
 
//PORTS

//OUT
//PD0 Led

//TWI
//PC4=SDA, PC5=SCL: I²C-Bus lines: 

//INPUT
//PB0: DCF77 RX pulse signal

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <util/twi.h>
#include <avr/pgmspace.h>

#define VDD 3.279

#undef F_CPU 
#define F_CPU 16000000

#define OLEDCMD 0x00   //Command follows
#define OLEDDATA 0x40  //Data follows
#define OLEDADDR 0x78  //address for the chip - usually 0x7C or 0x78. 

#define FONTW 6
#define FONTH 8

#define S_SETLOWCOLUMN           0x00
#define S_SETHIGHCOLUMN          0x10
#define S_PAGEADDR               0xB0
#define S_SEGREMAP               0xA0

#define S_LCDWIDTH               128
#define S_LCDHEIGHT              64 
//////////////////////////////////////
//   L   C   D   
//////////////////////////////////////

// Font 6x8 for OLED
const char font[97][6] PROGMEM={
{0x00,0x00,0x00,0x00,0x00,0x00},	// 0x20
{0x00,0x00,0x06,0x5F,0x06,0x00},	// 0x21
{0x00,0x07,0x03,0x00,0x07,0x03},	// 0x22
{0x00,0x24,0x7E,0x24,0x7E,0x24},	// 0x23
{0x00,0x24,0x2B,0x6A,0x12,0x00},	// 0x24
{0x00,0x63,0x13,0x08,0x64,0x63},	// 0x25
{0x00,0x36,0x49,0x56,0x20,0x50},	// 0x26
{0x00,0x00,0x07,0x03,0x00,0x00},	// 0x27
{0x00,0x00,0x3E,0x41,0x00,0x00},	// 0x28
{0x00,0x00,0x41,0x3E,0x00,0x00},	// 0x29
{0x00,0x08,0x3E,0x1C,0x3E,0x08},	// 0x2A
{0x00,0x08,0x08,0x3E,0x08,0x08},	// 0x2B
{0x00,0x00,0xE0,0x60,0x00,0x00},	// 0x2C
{0x00,0x08,0x08,0x08,0x08,0x08},	// 0x2D
{0x00,0x00,0x60,0x60,0x00,0x00},	// 0x2E
{0x00,0x20,0x10,0x08,0x04,0x02},	// 0x2F
{0x00,0x3E,0x51,0x49,0x45,0x3E},	// 0x30
{0x00,0x00,0x42,0x7F,0x40,0x00},	// 0x31
{0x00,0x62,0x51,0x49,0x49,0x46},	// 0x32
{0x00,0x22,0x49,0x49,0x49,0x36},	// 0x33
{0x00,0x18,0x14,0x12,0x7F,0x10},	// 0x34
{0x00,0x2F,0x49,0x49,0x49,0x31},	// 0x35
{0x00,0x3C,0x4A,0x49,0x49,0x30},	// 0x36
{0x00,0x01,0x71,0x09,0x05,0x03},	// 0x37
{0x00,0x36,0x49,0x49,0x49,0x36},	// 0x38
{0x00,0x06,0x49,0x49,0x29,0x1E},	// 0x39
{0x00,0x00,0x6C,0x6C,0x00,0x00},	// 0x3A
{0x00,0x00,0xEC,0x6C,0x00,0x00},	// 0x3B
{0x00,0x08,0x14,0x22,0x41,0x00},	// 0x3C
{0x00,0x24,0x24,0x24,0x24,0x24},	// 0x3D
{0x00,0x00,0x41,0x22,0x14,0x08},	// 0x3E
{0x00,0x02,0x01,0x59,0x09,0x06},	// 0x3F
{0x00,0x3E,0x41,0x5D,0x55,0x1E},	// 0x40
{0x00,0x7E,0x11,0x11,0x11,0x7E},	// 0x41
{0x00,0x7F,0x49,0x49,0x49,0x36},	// 0x42
{0x00,0x3E,0x41,0x41,0x41,0x22},	// 0x43
{0x00,0x7F,0x41,0x41,0x41,0x3E},	// 0x44
{0x00,0x7F,0x49,0x49,0x49,0x41},	// 0x45
{0x00,0x7F,0x09,0x09,0x09,0x01},	// 0x46
{0x00,0x3E,0x41,0x49,0x49,0x7A},	// 0x47
{0x00,0x7F,0x08,0x08,0x08,0x7F},	// 0x48
{0x00,0x00,0x41,0x7F,0x41,0x00},	// 0x49
{0x00,0x30,0x40,0x40,0x40,0x3F},	// 0x4A
{0x00,0x7F,0x08,0x14,0x22,0x41},	// 0x4B
{0x00,0x7F,0x40,0x40,0x40,0x40},	// 0x4C
{0x00,0x7F,0x02,0x04,0x02,0x7F},	// 0x4D
{0x00,0x7F,0x02,0x04,0x08,0x7F},	// 0x4E
{0x00,0x3E,0x41,0x41,0x41,0x3E},	// 0x4F
{0x00,0x7F,0x09,0x09,0x09,0x06},	// 0x50
{0x00,0x3E,0x41,0x51,0x21,0x5E},	// 0x51
{0x00,0x7F,0x09,0x09,0x19,0x66},	// 0x52
{0x00,0x26,0x49,0x49,0x49,0x32},	// 0x53
{0x00,0x01,0x01,0x7F,0x01,0x01},	// 0x54
{0x00,0x3F,0x40,0x40,0x40,0x3F},	// 0x55
{0x00,0x1F,0x20,0x40,0x20,0x1F},	// 0x56
{0x00,0x3F,0x40,0x3C,0x40,0x3F},	// 0x57
{0x00,0x63,0x14,0x08,0x14,0x63},	// 0x58
{0x00,0x07,0x08,0x70,0x08,0x07},	// 0x59
{0x00,0x71,0x49,0x45,0x43,0x00},	// 0x5A
{0x00,0x00,0x7F,0x41,0x41,0x00},	// 0x5B
{0x00,0x02,0x04,0x08,0x10,0x20},	// 0x5C
{0x00,0x00,0x41,0x41,0x7F,0x00},	// 0x5D
{0x00,0x04,0x02,0x01,0x02,0x04},	// 0x5E
{0x80,0x80,0x80,0x80,0x80,0x80},	// 0x5F
{0x00,0x00,0x03,0x07,0x00,0x00},	// 0x60
{0x00,0x20,0x54,0x54,0x54,0x78},	// 0x61
{0x00,0x7F,0x44,0x44,0x44,0x38},	// 0x62
{0x00,0x38,0x44,0x44,0x44,0x28},	// 0x63
{0x00,0x38,0x44,0x44,0x44,0x7F},	// 0x64
{0x00,0x38,0x54,0x54,0x54,0x08},	// 0x65
{0x00,0x08,0x7E,0x09,0x09,0x00},	// 0x66
{0x00,0x18,0xA4,0xA4,0xA4,0x7C},	// 0x67
{0x00,0x7F,0x04,0x04,0x78,0x00},	// 0x68
{0x00,0x00,0x00,0x7D,0x40,0x00},	// 0x69
{0x00,0x40,0x80,0x84,0x7D,0x00},	// 0x6A
{0x00,0x7F,0x10,0x28,0x44,0x00},	// 0x6B
{0x00,0x00,0x00,0x7F,0x40,0x00},	// 0x6C
{0x00,0x7C,0x04,0x18,0x04,0x78},	// 0x6D
{0x00,0x7C,0x04,0x04,0x78,0x00},	// 0x6E
{0x00,0x38,0x44,0x44,0x44,0x38},	// 0x6F
{0x00,0xFC,0x44,0x44,0x44,0x38},	// 0x70
{0x00,0x38,0x44,0x44,0x44,0xFC},	// 0x71
{0x00,0x44,0x78,0x44,0x04,0x08},	// 0x72
{0x00,0x08,0x54,0x54,0x54,0x20},	// 0x73
{0x00,0x04,0x3E,0x44,0x24,0x00},	// 0x74
{0x00,0x3C,0x40,0x20,0x7C,0x00},	// 0x75
{0x00,0x1C,0x20,0x40,0x20,0x1C},	// 0x76
{0x00,0x3C,0x60,0x30,0x60,0x3C},	// 0x77
{0x00,0x6C,0x10,0x10,0x6C,0x00},	// 0x78
{0x00,0x9C,0xA0,0x60,0x3C,0x00},	// 0x79
{0x00,0x64,0x54,0x54,0x4C,0x00},	// 0x7A
{0x00,0x08,0x3E,0x41,0x41,0x00},	// 0x7B
{0x00,0x00,0x00,0x77,0x00,0x00},	// 0x7C
{0x00,0x00,0x41,0x41,0x3E,0x08},	// 0x7D
{0x00,0x02,0x01,0x02,0x01,0x00},	// 0x7E
{0x00,0x3C,0x26,0x23,0x26,0x3C},	// 0x7F
{0x00,0x1E,0xA1,0xE1,0x21,0x12}};	// 0x80
///////////////////////////
//     DECLARATIONS
///////////////////////////
//
//I²C
void TWIInit(void);
void TWIStart(void);
void TWIStop(void);
uint8_t TWIReadACK(void);
uint8_t TWIReadNACK(void);
uint8_t TWIGetStatus(void);

//OLED
void oled_command(int value);
void oled_data(unsigned int*, unsigned int);
void oled_gotoxy(unsigned int, unsigned int);
void oled_cls(int);
void oled_init(void);
void oled_byte(unsigned char);
void oled_putchar1(unsigned int x, unsigned int y, unsigned char ch, int);
void oled_putchar2(unsigned int x, unsigned int y, unsigned char ch, int);
void oled_putnumber(int, int, long, int, int, int);
void oled_putstring(int, int, char*, char, int);
void oled_write_section(int, int, int, int);

//String
int int2asc(long num, int dec, char *buf, int buflen);
int strlen(char *s);

//ADC and voltage measurement
int get_adc(int);
int get_panel_voltage(void);
int get_batt_voltage(void);
int get_charger_current(void);
//MISC
int main(void);

//DCF77
int get_bits(int*, int, int);
int get_parity(int*, int, int);

unsigned long ms = 0;

///////////////////////////
//
//         TWI
//
///////////////////////////

void twi_init(void)
{
    //set SCL to 400kHz
    TWSR = 0x00;
    TWBR = 0x0C;
	
    //enable TWI
    TWCR = (1<<TWEN);
}

//Send start signal
void twi_start(void)
{
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
    while ((TWCR & (1<<TWINT)) == 0);
}

//send stop signal
void twi_stop(void)
{
    TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
}

void twi_write(uint8_t u8data)
{
    TWDR = u8data;
    TWCR = (1<<TWINT)|(1<<TWEN);
    while ((TWCR & (1<<TWINT)) == 0);
}

uint8_t TWIGetStatus(void)
{
    uint8_t status;
    //mask status
    status = TWSR & 0xF8;
    return status;
}

////////////////////////////////
//
// OLED routines
//
///////////////////////////////

//Send comand to OLED
void oled_command(int value)
{
   twi_start();
   twi_write(OLEDADDR); //Device address
   twi_write(OLEDCMD);  //Command follows
   twi_write(value);    //
   twi_stop();
} 

//Send a 'number' bytes of data to display - from RAM
void oled_data(unsigned int *data, unsigned int number)
{
   int t1;
   twi_start();
   twi_write(OLEDADDR); //Device address
   twi_write(OLEDDATA); //Data follows
   
   for(t1 = 0; t1 < number; t1++)
   {
      twi_write(data[t1]); //send the byte(s)
   }   
   twi_stop ();   
}

//Set "cursor" to current position to screen
void oled_gotoxy(unsigned int x, unsigned int y)
{
   int x2 = x + 2;
   twi_start();
   twi_write(OLEDADDR); //Select display  I2C address
   twi_write(OLEDCMD);  //Be ready for command
   twi_write(S_PAGEADDR + y); //Select display row
   twi_write(S_SETLOWCOLUMN + (x2 & 0x0F)); //Col addr lo byte
   twi_write(S_SETHIGHCOLUMN + ((x2 >> 4) & 0x0F)); //Col addr hi byte
   twi_stop();
}

void oled_cls(int invert)
{
    unsigned int row, col;

    //Just fill the memory with zeros
    for(row = 0; row < S_LCDHEIGHT / 8; row++)
    {
        oled_gotoxy(0, row); //Set OLED address
        twi_start();
        twi_write(OLEDADDR); //Select OLED
        twi_write(OLEDDATA); //Data follows
        for(col = 0; col < S_LCDWIDTH; col++)
        {
            if(!invert)
            {
                twi_write (0); //normal
            }   
            else
            {
                twi_write(255); //inverse
            } 
        }
        twi_stop();
    }
    oled_gotoxy(0, 0); //Return to 0, 0
}

//Write number of bitmaps to one row of screen
void oled_write_section(int x1, int x2, int row, int number)
{
    int t1;
    oled_gotoxy(x1, row);
    	
    twi_start();
    twi_write(OLEDADDR); //Device address
    twi_write(OLEDDATA); //Data follows
   
    for(t1 = x1; t1 < x2; t1++)
    {
       twi_write(number); //send the byte(s)
    }    
    twi_stop ();   
}


//Initialize OLED
void oled_init(void)
{
    oled_command(0xAE); // Display OFF
	oled_command(0x20); // Set Memory Addressing Mode
    oled_command(0x00); // HOR
    
    oled_command(0xB0);    // Set Page Start Address for Page Addressing Mode, 0-7
    oled_command(0xC8);    // Set COM Output Scan Direction
    oled_command(0x00);    // --set low column address
    oled_command(0x10);    // --set high column address
    oled_command(0x40);    // --set start line address
    oled_command(0x81);
    oled_command(0xFF);    // Set contrast control register
    oled_command(0xA1);    // Set Segment Re-map. A0=address mapped; A1=address 127 mapped.
    oled_command(0xA6);    // Set display mode. A6=Normal; A7=Inverse
    oled_command(0xA8);
    oled_command(0x3F);    // Set multiplex ratio(1 to 64)
    oled_command(0xA4);    // Output RAM to Display
					       // 0xA4=Output follows RAM content; 0xA5,Output ignores RAM content
    oled_command(0xD3);
    oled_command(0x00);    // Set display offset. 00 = no offset
    oled_command(0xD5);    // --set display clock divide ratio/oscillator frequency
    oled_command(0xF0);    // --set divide ratio
    oled_command(0xD9); 
    oled_command(0x22);    // Set pre-charge period
    oled_command(0xDA);
    oled_command(0x12);    // Set com pins hardware configuration
    oled_command(0xDB);    // --set vcomh
    oled_command(0x20);    // 0x20,0.77xVcc
    oled_command(0x8D);
    oled_command(0x14);    // Set DC-DC enabl
    oled_command(0xAF);    //Display ON
   
} 

//Write 1 byte pattern to screen using vertical orientation 
void oled_byte(unsigned char value)
{
   twi_start();
   twi_write(OLEDADDR); //Device address
   twi_write(OLEDDATA); //Data follows
   twi_write(value);
   twi_stop ();   
}

//Write character to screen (normal size);
void oled_putchar1(unsigned int x, unsigned int y, unsigned char ch, int invert)
{
	int t0;
		
	oled_gotoxy(x, y);
	for(t0 = 0; t0 < FONTW; t0++)
	{
		if(!invert)
		{
            oled_byte(pgm_read_byte(&font[ch - 32][t0]));
        }
        else    
        {
            oled_byte(~pgm_read_byte(&font[ch - 32][t0]));
        }
        
	}
}		

//Write character to screen (DOUBLE size);
void oled_putchar2(unsigned int x, unsigned int y, unsigned char ch, int invert)
{
	int t0, t1;
	char c;
	int i[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	
	for(t0 = 0; t0 < FONTW; t0++)
	{
		for(t1 = 0; t1 < 8; t1++)
		{
			if(!invert)
			{
				c = pgm_read_byte(&font[ch - 32][t0]);
			}
			else
			{
				c = ~pgm_read_byte(&font[ch - 32][t0]);
			}
				
		    if(c & (1 << t1))
		    {
			    i[t0] += (1 << (t1 * 2));
			    i[t0] += (1 << (t1 * 2 + 1));
		    }	
	    }
	}
	
	oled_gotoxy(x, y);
	for(t0 = 0; t0 < FONTW; t0++)
	{		
	    oled_byte(i[t0] & 0xFF);
	    oled_byte(i[t0] & 0xFF);
	}
	
	oled_gotoxy(x, y + 1);
	for(t0 = 0; t0 < FONTW; t0++)
	{		
	    oled_byte((i[t0] & 0xFF00) >> 8);
	    oled_byte((i[t0] & 0xFF00) >> 8);
	}
}		

//Print string in given size
//lsize=0 => normal height, lsize=1 => double height
void oled_putstring(int col, int row, char *s, char lsize, int inv)
{
    int c = col * 6;
	
	while(*s)
	{
	    if(!lsize)
		{
	        oled_putchar1(c, row, *s++, inv);
		}
        else
        {
            oled_putchar2(c, row, *s++, inv);
		}	
		c += (lsize + 1) * FONTW;
	}
}

//Print an integer/long to OLED
void oled_putnumber(int col, int row, long num, int dec, int lsize, int inv)
{
    char *s = malloc(16);
	if(s != NULL)
	{
	    int2asc(num, dec, s, 16);
	    oled_putstring(col, row, s, lsize, inv);
	    free(s);
	}	
}


/////////////////////////////////
//
// STRING FUNCTIONS
//
////////////////////////////////
//INT 2 ASC
int int2asc(long num, int dec, char *buf, int buflen)
{
    int i, c, xp = 0, neg = 0;
    long n, dd = 1E09;

    if(!num)
	{
	    *buf++ = '0';
		*buf = 0;
		return 1;
	}	
		
    if(num < 0)
    {
     	neg = 1;
	    n = num * -1;
    }
    else
    {
	    n = num;
    }

    //Fill buffer with \0
    for(i = 0; i < 12; i++)
    {
	    *(buf + i) = 0;
    }

    c = 9; //Max. number of displayable digits
    while(dd)
    {
	    i = n / dd;
	    n = n - i * dd;
	
	    *(buf + 9 - c + xp) = i + 48;
	    dd /= 10;
	    if(c == dec && dec)
	    {
	        *(buf + 9 - c + ++xp) = '.';
	    }
	    c--;
    }

    //Search for 1st char different from '0'
    i = 0;
    while(*(buf + i) == 48)
    {
	    *(buf + i++) = 32;
    }

    //Add minus-sign if neccessary
    if(neg)
    {
	    *(buf + --i) = '-';
    }

    //Eleminate leading spaces
    c = 0;
    while(*(buf + i))
    {
	    *(buf + c++) = *(buf + i++);
    }
    *(buf + c) = 0;
	
	return c;
}

//STRLEN
int strlen(char *s)  
{
   int t1 = 0;

   while(*(s + t1++));

   return (t1 - 1);
}

ISR (TIMER2_COMP_vect)
{
    ms++; 
}

//DCF77 decode routine
int get_bits(int b[], int sta, int end)
{
    int t0, n = 0;
    
    for(t0 = sta; t0 < end + 1; t0++)
    {
        n += b[t0] << (t0 - sta); //Change bit order! 0100 bin. = 2 dec.!!!
    }
    return n;
    
}

int get_parity(int b[], int sta, int end)
{
    int t0, n = 0;
    
    for(t0 = sta; t0 < end + 1; t0++)
    {
        if(b[t0])
        {
			n++;
		}	
    }
    
    if((n / 2) * 2 == n)
    {
		return 0; //even parity
	}
	else	
	{
		return 1; //odd parity
	}
}
int main(void)
{
	unsigned long ms0 = 0, ms1 = 0;
	int b[60];
	int bc = 0;
	int t1;
	char weekday[7][4] = {"MON", "TUE", "WEN", "THU", "FRI", "SAT", "SUN"};
		
	//OUTPUT
	DDRD = (1 << PD0);	
	//INPUT
    PORTC = 0x30;//PC0: Pull-up for key switches with various resistors against GND 
	             //I²C-Bus lines: PC4=SDA, PC5=SCL
	             
	//ADC
	ADMUX = (1<<REFS0);                          // Set Reference to AVCC and input to ADC0
    ADCSRA = (1<<ADFR)|(1<<ADEN)                 // Enable ADC, set prescaler to 16
            |(1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2);   // Fadc=Fcpu/prescaler=16000000/128=125kHz
                                                 // Fadc should be between 50kHz and 200kHz

    ADCSRA |= (1<<ADSC);              // Start the first conversion
    ADCSRA |= (1<<ADSC);              // Start the first conversion
    
    //Timer 2 as counter for 1 millisecond fclock = 8MHz
    OCR2 = 62;
    TCCR2 |= (1 << WGM21); // Set to CTC Mode
    TIMSK |= (1 << OCIE2); //Set interrupt on compare match
    TCCR2 |= (1 << CS21)|(1 << CS22);  // set prescaler to 256
                 
	//TWI
	twi_init();
	_delay_ms(20);
	
	//OLED
	oled_init();
	_delay_ms(20);
	
	oled_cls(0);	
	
	oled_putstring(0, 0, " DK7IH DCF77 CLOCK ", 0, 1);
 
    //Init DCF77 bit array;
    for(t1 = 0; t1 < 59; t1++)
    { 
		b[t1] = 0;
	}	
		
    sei();	
		  
    for(;;)
    {
	    //DCF77
	    //Wait till pause ends
	    ms0 = ms;
	    while(!(PINB & (1 << PB0)))
	    {
			PORTD |= (1 << PD0);
	    }
	    ms1 = ms;
	    
	    if((ms1 - ms0) > 1200) //Pause longer than 1.2 seconds? YES: New minute starts
	    {
			bc = 0;
						
			//Parity check  date
			if(get_parity(b, 36, 57) != get_bits(b, 58, 58))
			{
				oled_putstring(0, 2, "-------", 1, 0);
			}
			else
			{
			    //Day
			    oled_putnumber(0, 2, get_bits(b, 40, 41), -1, 1, 0);
			    oled_putnumber(2, 2, get_bits(b, 36, 39), -1, 1, 0);
			    oled_putchar2(4 * 6, 2, '.', 0); 
			
			    //Month
			    oled_putnumber(5, 2, get_bits(b, 49, 49), -1, 1, 0);
			    oled_putnumber(7, 2, get_bits(b, 45, 48), -1, 1, 0);
			    oled_putchar2(9 * 6, 2, '.', 0); 
			
			    //Year
			    oled_putnumber(10, 2, get_bits(b, 54, 57), -1, 1, 0);
			    oled_putnumber(12, 2, get_bits(b, 50, 53), -1, 1, 0);
			}    
					
			//Day of week			
			if((get_bits(b, 42, 44) >= 0) && (get_bits(b, 42, 44) <= 6))
			{
			    oled_putstring(15, 3, weekday[get_bits(b, 42, 44) - 1], 0, 0); 
			}
			else
			{
				oled_putstring(12, 3, "---", 0, 0);     
			}	
			
			//parity check hour
			if(get_parity(b, 29, 34) != get_bits(b, 35, 35))
			{
				oled_putstring(0, 5, "--", 1, 0);
			}
			else
			{
			    //Hour
			    oled_putnumber(0, 5, get_bits(b, 33, 34), -1, 1, 0);
			    oled_putnumber(2, 5, get_bits(b, 29, 32), -1, 1, 0);
			    oled_putchar2(4 * 6, 5, ':', 0); 
			}
			
			//parity check minute
			if(get_parity(b, 21, 27) != get_bits(b, 28, 28))
			{
				oled_putstring(5, 5, "--", 1, 0);
			}
			else
			{
				//Minute
			    oled_putnumber(5, 5, get_bits(b, 25, 27), -1, 1, 0);
			    oled_putnumber(7, 5, get_bits(b, 21, 24), -1, 1, 0);
			}    
			
			//MEZ or MESZ?
			if(!b[17] && b[18])
			{
				oled_putstring(15, 6, "MEZ ", 0, 0);
			}
			
			if(b[17] && !b[18])
			{
				oled_putstring(15, 6, "MESZ", 0, 0);
			}		
		}
		
		//Info	
	    PORTD |= (1 << PD0);
	    ms0 = ms;
	    while(PINB & (1 << PB0))
	    {
			PORTD &= ~(1 << PD0);
	    }   
	    ms1 = ms; 
	    PORTD |= (1 << PD0);

        //Is bit 0 or 1?
	    if((ms1 - ms0) > 120)
	    {
			b[bc] = 1;
		}
		else	
		{
			b[bc] = 0;
		}
		
		//Bitcounter still in range?
		if(bc < 60)
		{
			bc++;
		}	
    }
	return 0;
}
