
#include <stdint.h>
#include <ctype.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include <stdlib.h>
#include <math.h>

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define PUSH_BUTTON  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))
#define MAX_CHAR 0x50
#define PI 3.14159625

#define CS_NOT       (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 1*4)))
#define A0           (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 6*4)))



    char stri [MAX_CHAR + 1];
	char str_new1[MAX_CHAR + 1];
	char str_new2[MAX_CHAR + 1];
	uint8_t field_position[20];
	uint8_t field_type[20];
	uint8_t field_count=0;
	char first_str[40];
	char second_str[40];
	uint32_t data;
	uint32_t phase;
	uint32_t delta_phase;
	uint32_t table[4096];
	uint16_t n;
	uint16_t index;
	   float frequency,freq2;
	   float amplitude;
	   uint32_t value;
	   float arrived_data;


	   void waitMicrosecond(uint32_t us)
	   {
	   	                                            // Approx clocks per us
	   	__asm("WMS_LOOP0:   MOV  R1, #6");          // 1
	       __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
	       __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
	       __asm("             NOP");                  // 5
	       __asm("             B    WMS_LOOP1");       // 5*3
	       __asm("WMS_DONE1:   SUB  R0, #1");          // 1
	       __asm("             CBZ  R0, WMS_DONE0");   // 1
	       __asm("             B    WMS_LOOP0");       // 1*3
	       __asm("WMS_DONE0:");                        // ---
	                                                   // 40 clocks/us + error
	   }

// Initialize Hardware
void initHw()
{
	// Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A and F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOB| SYSCTL_RCGC2_GPIOF|SYSCTL_RCGC2_GPIOB;

    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R = 0x0A;  // bits 1 and 3 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R = 0x0A; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x1A;  // enable LEDs and pushbuttons
    GPIO_PORTF_PUR_R = 0x10;  // enable internal pull-up for push button

    // Configure UART0 pins
	SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
	GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

   	// Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
	UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    		  // Configure A0 and ~CS
              GPIO_PORTB_DIR_R |= 0x42;  // make bits 1 and 6 outputs
              GPIO_PORTB_DR2R_R |= 0x42; // set drive strength to 2mA
              GPIO_PORTB_DEN_R |= 0x42;  // enable bits 1 and 6 for digital*/

              // Configure SSI2 pins for SPI configuration
              SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2;           // turn-on SSI2 clocking
              GPIO_PORTB_DIR_R |= 0xB0;                        // make bits 4, 5 and 7 outputs
              GPIO_PORTB_DR2R_R |= 0xB0;                       // set drive strength to 2mA
              GPIO_PORTB_AFSEL_R |= 0xB0;                      // select alternative functions for MOSI, SCLK, FSS pins
              GPIO_PORTB_PCTL_R = GPIO_PCTL_PB7_SSI2TX | GPIO_PCTL_PB4_SSI2CLK | GPIO_PCTL_PB5_SSI2FSS; // map alt fns to SSI2
              GPIO_PORTB_DEN_R |= 0xB0;                        // enable digital operation on TX, CLK and FSS pins
              GPIO_PORTB_PUR_R |= 0x10;                        // must be enabled when SPO=1

              // Configure the SSI2 as a SPI master, mode 3, 16bit operation, 1 MHz bit rate
              SSI2_CR1_R &= ~SSI_CR1_SSE;                      // turn off SSI2 to allow re-configuration
              SSI2_CR1_R = 0;                                  // select master mode
              SSI2_CC_R = 0;                                   // select system clock as the clock source
              SSI2_CPSR_R = 20;                                // set bit rate to 1 MHz (if SR=0 in CR0)
              SSI2_CR0_R = SSI_CR0_SPH | SSI_CR0_SPO | SSI_CR0_FRF_MOTO | SSI_CR0_DSS_16; // set SR=0, mode 3 (SPH=1, SPO=1), 8-bit
              SSI2_CR1_R |= SSI_CR1_SSE;                       // turn on SSI2


             // Configure Timer 1 as the time base
                 SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
                 TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
                 TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
                 TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
                 TIMER1_TAILR_R = 0x190;                          // set load value to 400 for 100 kHz interrupt rate
                 TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
                 NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
                // TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
	while (UART0_FR_R & UART_FR_TXFF);
	UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
	uint8_t i;
    for (i = 0; i < strlen(str); i++)
	  putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
	while (UART0_FR_R & UART_FR_RXFE);
	return UART0_DR_R & 0xFF;
}

//Step 2 Taking a string
char getstring(char *stri)
{
	uint8_t char_count = 0;
	char c;

	while(1)
	{
		c=getcUart0();
		if (c == 8 && char_count == 0)
		{
			continue;
		}
		if( char_count != MAX_CHAR )
		{
			if (c == 0x0D)
			{
				break;
			}
			if (c == 8)
			{
				char_count --;
			}
			if (c >= 0x20)
			{
				stri [char_count] = c;
				char_count++;
			}

		}
		else
		{

			break;
		}
	}
	stri [char_count+1] = 0;
	 putsUart0(stri);

}



// Step3 Checking string for the string values and arguments
	 void check_string(char stri[],uint8_t *field_count, char *str_new1,uint8_t *field_position)
	 {
		            //char str_new1[81] = {0};
		 			char str_new2[81] = {0};
		 			char field_type[20]={0};
		 			uint8_t i=0,j,l=0;
		 			char first_str[40] = {0};
		 			char second_str[40] = {0};
		 			(*field_count) = 0;

	 while(i< 20)
	     {

	 	   if (stri[i]>=0x41 && stri[i]<= 0x5A || stri[i]>=0x61 && stri[i]<= 0x7A)
	 	   {
	 		   str_new1[i]=stri[i];
	 	   }
	 	   else if ((stri[i]>=0x30 && stri[i]<=0x39) || stri[i]==0x2E || stri[i]== 0x2A || stri[i] == 0x2D)
	 	   {
	 		   str_new1[i]=stri[i];
	 	   }
	 	   else str_new1[i]='\0';

	 	   i++;
	    }
	 while(l< 20)

    {

	   if (stri[l]>=0x41 && stri[l]<= 0x5A || stri[l]>=0x61 && stri[l]<= 0x7A)
	   {
		   str_new2[l]='c';
	   }
	   else if ((stri[l]>=0x30 && stri[i]<=0x39) || stri[l]==0x2E || stri[l]== 0x2A || stri[l] == 0x2D)
	   {
		   str_new2[l]='n';
	   }
	   else str_new2[l]='\0';

	   l++;
   }



	 int x=0,y=0;

for(j=0; j<40; j++)
{
	first_str[x] = str_new2[j];
	second_str[x] = str_new2[j+1];

	if (second_str[x] != first_str[x] && second_str[x] != '\0')
	{
		field_position[y]=j+1;
		field_type[y]=second_str[x];
	    (*field_count)++;
		y++;
	}
	x++;
}
putsUart0(str_new1);
putsUart0(str_new2);
}


_Bool iscommand(char *entered_field_name, int no_of_arguments)
	 {
	 	int m = strcmp (&str_new1[field_position[0]],entered_field_name);
	 	if (m == 0 && (field_count > no_of_arguments))
	 	{
	 		return 1;
	 	}
	 	else
	 	{

	 		return 0;

	 	}
	 }


//Look - Up table
void lookuptable()
{
/*if (amplitude < 0)
	 	{
	 		for (n=0; n < 4096 ; n++)

	 		{
	 		 	table[n] = 0x3000 + 1982 + (( 2113*amplitude)/5) *( sin((2*PI*n)/ 4096));
	 		}
	 	}*/
        {
		 	for (n=0; n < 4096 ; n++)

		 		{
		 		 	table[n] = 0x3000 + 1982 +  ((1982*(amplitude))/5) *( sin((2*PI*n)/ 4096));
		 		}


        }
}

float get_number(int field_index)
	{
		float number = 0;
		number = atof(&str_new1[field_position [field_index]]);
		return number;

	}


	 	void getcommands()
	 	{
	 		if (iscommand("sine",2))
	 	{

	 		frequency = get_number(1);

	 		amplitude = get_number(2);
	 		if (frequency < 0 || amplitude > 5 || amplitude < -5)
	 			 			 		{
	 			 			 			putsUart0("\r\n Please Enter a valid command\r\n");
	 			 			 		}
	 		else
	 		{
	 		lookuptable();
	 		delta_phase = (frequency * 4294967296)/100000 ;
	 		TIMER1_CTL_R |= TIMER_CTL_TAEN;
	 		}
	 	}
	 		else if (iscommand("sawtooth",2))
	 	{
	 		frequency = get_number(1);
	 		amplitude = get_number(2);
	 	}
	 		else if (iscommand("square",2))
	 	{
	 		frequency = get_number(1);
	 		amplitude = get_number(2);
	 	}
	 		else if (iscommand("sweep",2))
		{
	 		frequency = get_number(1);
	 		freq2 = get_number(2);
		}
	 		else if (iscommand("dc",1))
	 	{
	 		arrived_data = get_number(1);
	 		if (arrived_data > 5 || arrived_data < -5)
	 		{
	 			putsUart0("\r\n Please Enter a valid command\r\n");
	 		}
	 		else if (iscommand("volatge"))
	 		{

	 		}

	 		else

	 		SSI2_DR_R = 0x3000 + (1982 - (( 1982*(arrived_data) / 5)));

	 	}


	 		/*}
	 		else
	 		{
	 		SSI2_DR_R = 0x3000 + (1982 + (( 2113* (arrived_data) + 5)/10));
	 		}*/

	 		//while (SSI2_SR_R & SSI_SR_BSY);


	 	else
	 	{
	 		putsUart0("\r\n Please Enter a valid command\r\n");
	 	}
	 	}


	 	void Timer1Isr()
	 	{

	 		phase += delta_phase;
	 		index = phase >> 20;
	 		value = table[index];
	 		SSI2_DR_R= value;
	 		TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
	 	}


int main(void)


{

	initHw();
	GREEN_LED ^= 1;
	waitMicrosecond(50000);


	while(1)
	{

		putsUart0("\r\nPlease Enter the commands\r\n");
		getstring(stri);
		check_string(stri, &field_count, str_new1, field_position);
		getcommands();

	}

}









