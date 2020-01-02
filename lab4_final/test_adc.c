//LAB 4 - Amine Gaizi  

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "hd44780.h"
#include <avr/interrupt.h>

//***********************************************************************
//                    Global variables
//***********************************************************************

#define CW 1 
#define CCW 2

uint8_t A_old = 1; //Past value of the A switch of the encoder
uint8_t B_old = 1; //Past value of the B switch of the encoder
uint8_t A2_old = 1; //Past value of the A switch of the encoder
uint8_t B2_old = 1; //Past value of the B switch of the encoder


uint8_t direction = 0; //Direction the encoder is turning
volatile uint8_t second = 0; 
volatile uint8_t minute_clk = 0;
volatile uint8_t hour_clk = 0; 
volatile uint8_t minute_alarm = 1;
volatile uint8_t hour_alarm = 0;
uint8_t dummy_counter = 0; 
uint8_t second_flag = 0; 
uint8_t segment_data[5]; 
//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[12]={0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90};
uint16_t adc_result;     //holds adc result 
bool set_clock_ena = 0;
bool set_alarm_ena = 0;
volatile bool alarm_set = 0;
bool trigger_alarm = 0;
volatile bool snooze = 0;
bool display_WR = 0;
volatile int TENsec_counter = 0;
bool hour_format = 0; 
bool hour_am = 1;
//***********************************************************************
//                     Function prototypes
//***********************************************************************
void TIM0_RTC_init();
void TimeProcessing();
void InitLEDdisplay();
void SelectDigit(int DigitNumber);
void SeperateDigits();
void TIM2_PWM_init(void);
void ADC_init();
void TIM1_init();
void TIM3_PWM_init();
int chk_buttons(uint8_t PinNumber);
uint8_t Encoder1_handling(int data_encoder, uint8_t counter);
uint8_t Encoder2_handling(int data_encoder, uint8_t counter, uint8_t bound);
void spi_init(void);
int spi_read();
void spi_write(uint8_t display_count);
void InitPushButtons();
void Set_Clock();
void Set_Alarm();
void Alarm();
void TIM3_set_PWM(int duty_cycle);

//void InitPushButtons()
//This function initializes the IO to read the state of the push buttons
void InitPushButtons()
{
        DDRA = 0x00; //Set all PortA to input
        DDRB |= 0xF0; //Set PortB pin fom 4 to 7 as output
        asm("nop"); //IO Synchronization necessary delay
        asm("nop");
        PORTA = 0xFF; //Pull up resistor on PORTA input pins
        PORTB |= (1<<PORTB4) | (1<<PORTB5) | (1<<PORTB6) | (1<<PORTB7); //Enable Tri-State buffer and cutoff Transistor
}

void spi_init(void){

	 // Run this code before attempting to write to the LCD.*/
	DDRF  |= 0x08;  //port F bit 3 is enable for LCD
	asm("nop");
	asm("nop");
	PORTF &= 0xF7;  //port F bit 3 is initially low

        DDRB |= 0b00000111; //Set PB0, PB1 and PB2 as output (resp. SS_N, CLK, MOSI) 
        DDRB &= 0b11110111; //Set PB3 as input (MISO)
        asm("nop"); //IO sync delay
        asm("nop");

        SPCR   = (1<<SPE) | (1<<MSTR); //master mode, clk low on idle, leading edge sample
        SPSR   = (1<<SPI2X); //choose double speed operation
        DDRE   = (1<<PE6) | (1<<PE7); //config PORTE6 and 7 as output
        asm("nop"); //IO sync delay
        asm("nop");
        PORTE |= (1<<PORTE6) | (1<<PORTE7); //SH/LD_N to high : no shifting, clkinh high
 }//spi_init

int spi_read()
{

        PORTE |= (1<<PE6);            //send rising edge to SH/LD_N on HC165
        PORTE = 0b10111111;             //send falling edge to  on HC165
        PORTE |= (1<<PE6);            //send rising edge to SH/LD_N on HC165
        PORTE &= (1<<PE6) | (0<<PE7); //Clk inhibit low, send data through miso
        SPDR = 0x00; //reset data value
        while (bit_is_clear(SPSR,SPIF)){} //spin till SPI data has been received (interrupt flag cleared)
        PORTE |= (1<<PE7);
        return SPDR;
}

//This function sends data via the MOSI port of the SPI 
//The data is sent to the bargraph
void spi_write(uint8_t display_count)
{

        SPDR = display_count;//send display_count to the display 
        while (bit_is_clear(SPSR,SPIF)){} //spin till SPI data has been sent 

        PORTB |= 0b00000001; //Rising edge on PORTB0 (SRCLK of the  bargraph)
        PORTB &= 0b11111110;  //falling edge on PORTB0 (SRCLK of the bargraph)
}


/*
*This function processes the increments in seconds to compute the time past
* Called in Timer 0 RTC interrupt
*/
void TimeProcessing() 
{
  if(dummy_counter == 128)//No prescaler, 128Hz
  {
    second++;
    dummy_counter = 0;
  }
  if(dummy_counter == 127)
  {second_flag = 0;}

  if(dummy_counter == 63)
  {second_flag = 1;}

  if(second >= 60)
  { minute_clk++; 
    second = 0;}
  if(minute_clk == 60)
  { hour_clk++;
    minute_clk = 0;}
  if(hour_clk == 24)
  { hour_clk = 0;} 
  dummy_counter++;  
}

//***********************************************************************
//                     ISR for timer counter zero
//***********************************************************************

ISR( TIMER0_OVF_vect ) 
{
	TimeProcessing();
	if(snooze)
		TENsec_counter++; //Count 10 seconds for Snooze
	
	return; 
}

//***********************************************************************
//                     Timer 0 RTC mode initialization
//***********************************************************************
void TIM0_RTC_init()
{
	ASSR |= (1<<AS0); //Timer 0 clocked from oscillator clkTOS = 32768Hz
	TCNT0 = 0x00; //Reset start value for the timer
	TCCR0 = (0<<CS2) | (0<<CS1) | (1<<CS0); //Select prescaler value of 1-> 128 interrupt every second
	while(!bit_is_clear(ASSR, TCN0UB)) //Wait for ASSR to be updated
  	{}
  	TIMSK |= (1<<TOIE0); //Enable Timer 0 Overflow interrupt
  	sei(); //Enable global interrupt
}


//***********************************************************************
//                     ISR for timer one
//***********************************************************************
ISR(TIMER1_COMPA_vect)
{
	if(trigger_alarm)
 	{	
		PORTD ^= (1<<PD2); //Toggle PORTD pin 2 with XOR operation
		if(!display_WR)
		{	string2lcd("Alarm!");
			display_WR = 1;}
			
	}
	else if(TENsec_counter == 1280)
	{	snooze = 0;
		trigger_alarm = 1;
		TENsec_counter = 0;
		clear_display();
		display_WR = 0;
	}
	if((display_WR == 1) && (trigger_alarm == 0))
	{
		display_WR = 0; 
		clear_display();
	}
	if(!trigger_alarm)
		PORTD &= (0<<PD2);
}

//***********************************************************************
//                     Timer 1 initialization
//***********************************************************************

void TIM1_init()
{
  	TCCR1A = 0x00;                  //Normal mode operation
  	TCCR1B = (1<<WGM12) | (1<<CS10) | (1<<CS11);   //use OCR1A as source for TOP, use clk/1024
  	TCCR1C = 0x00;                          //no forced compare 
  	OCR1A = 0x0269; //top value
  	TIMSK |= (1<<OCIE1A); //enable interrupt

  	DDRD |= (1<<PORTD2); // Port D bit 2 output
  	PORTD |= (0<<PD2); //high state
}


//void InitLEDdisplay()
//This function initializes the IO to use the 4Digit LED display
void InitLEDdisplay()
{
  	DDRA = 0xFF; //Set PORTA to all output
  	DDRB |= 0XF0; //Set PORTB pins 4 to 7 to output
  	asm("nop"); //IO Synchronization delay
  	asm("nop"); //IO Synchronization delay
  	PORTB |= 0b01100000; //Set PB5 and PB6 to 1 (no digit selected on Display)
  	PORTB &= 0b01101111; //Set PB4 and PB7 to 0 (transistor saturated) 
}

//This function take in parameter the DigitNumber to select
//It chooses the correct parameters to turn on that digit
void SelectDigit(int DigitNumber)
{
  	switch (DigitNumber)
        {
                case 0: //Choose digit number 1 (from the right)
                        PORTB = (0<<PORTB4) | (0<<PORTB5) | (0<<PORTB6);
                        break;
                case 1:  //Choose digit number 2 
                        PORTB = (1<<PORTB4) | (0<<PORTB5) | (0<<PORTB6);
                        break;
                case 2: //Choose the colon 
                    	PORTB = (0<<PORTB4) | (1<<PORTB5) | (0<<PORTB7);
                    break; 
                case 3:  //Choose digit number 3
                        PORTB = (1<<PORTB4) | (1<<PORTB5) | (0<<PORTB6);
                        break;
                case 4: //Choose digit number 4
            		PORTB = (0<<PORTB4) | (0<<PORTB5) | (1<<PORTB6);
                        break;
                default:
                        break;
        }
}

/*void SeperateDigits(int counter)
*This function separates the counter value into 4 different digits to be displayed
*The value of the number to be displayed by each digit is a modulo of 10
*/
void SeperateDigits()
{
   //if set alarm ena = 1, minute_alarm and hour_alarm; else minute_clk hour_clk

	uint8_t minute = 0, hour = 0;
	if(set_alarm_ena == 1)
	{	minute = minute_alarm;
		hour = hour_alarm;
	}
	else
	{	minute = minute_clk;
		hour = hour_clk;
	}

	segment_data[0] = dec_to_7seg[minute%10]; //The ones for minutes
	segment_data[1] = dec_to_7seg[(minute/10)%10]; //The tens for minutes
	segment_data[2] = dec_to_7seg[hour%10]; //The ones for hours
	segment_data[3] = dec_to_7seg[(hour/10)%10]; //The tens for hour
	segment_data[4] = 0xFC; //Colon 

	//uint8_t colon = 0xFC; 

	SelectDigit(0); //Select digit 0
	PORTA = segment_data[0]; //Display the "ones" value
	_delay_ms(1); 

	SelectDigit(1); //Select Digit 1
	PORTA = segment_data[1]; //Dispay "tens" value
	_delay_ms(1);

	if((set_clock_ena == 0) && ( set_alarm_ena == 0))
	{	SelectDigit(2); //Select Digit 2: colon
		if(second_flag == 0) //Flag for a half second
		{	if(alarm_set)
				PORTA = 0xFB;
			else 	
				PORTA = 0xFF; //Off
		}
		if(second_flag == 1)
		{ 	if(alarm_set)
				PORTA = 0xF8;
			else
				PORTA = 0xFC; //On
		}
	}
	else if((set_clock_ena || set_alarm_ena) == 1)
	{	SelectDigit(2); //If time or alarm are being set, no blinking of colon
		PORTA = 0xFC;
	}
	
	
	_delay_ms(1);

	SelectDigit(3); //Select Digit 3 
	PORTA = segment_data[2]; //Display hundreds valus
	_delay_ms(1);

	SelectDigit(4); //Select Digit 4
	PORTA = segment_data[3]; //Display thousands value
	_delay_ms(1);

}

void TIM2_PWM_init(void)
{
    /* Timer clock = I/O clock */    
    TCCR2 = 0x69; 
    /* Set the compare value to control duty cycle */    
    OCR2  = 0x80;    
    /* Set OC2A pin as output */
    DDRB |= 0b10000000;
}

void TIM3_PWM_init()
{
	DDRE |= (1<<PE3);
	OCR3A = 0x1FF; //50% duty cycle
	TCCR3A = (1<<COM3A1) | (1<<COM3A0) | (1<<WGM31) | (1<<WGM30); //Fast PWM 10bit 
	TCCR3B = (1<<WGM32) | (1<<CS30); //Set OC3A on compare match
}

void TIM3_set_PWM(int duty_cycle)
{
	uint16_t conversion = (duty_cycle*10.23); // duty*1023/100 = duty*10.23 
	OCR3A = 1023 - ((duty_cycle*1023)/100); //set the duty cycle value
}

void ADC_init()
{
  //Initalize ADC and its ports
  	DDRF  &= ~(_BV(DDF7)); //make port F bit 7 is ADC input  
  	PORTF &= ~(_BV(PF7));  //port F bit 7 pullups must be off
  	ADMUX = 0x47; //writes 00111 to ADMUX (4:0) for single-ended, input PORTF bit 7, right adjusted, 10 bits
  	ADCSRA |= (1<<ADEN) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); //ADC enabled, don't start yet, single shot mode 
                             //division factor is 128 (125khz)
}

ISR(ADC_vect)
{
  	int brightness_lvl = -1;
  	adc_result = ADC; //read the ADC output as 16 bits
  	brightness_lvl = (5*adc_result)/1023; 

  	switch(brightness_lvl)
	{
		case(1): //Resistance very high, dark environment  
			OCR2 = 0xCC; //80% duty cycle, LED display is active low
			break;
		case(2): 
			OCR2 = 0x9A; //60% duty cycle
			break; 
		case(3):
			OCR2 = 0x66; //40% duty cycle
			break;
		case(4):
			OCR2 = 0x33; //20% duty cycle 
			break; 
		case(0):
			OCR2 = 0x1A; //10% duty cycle
			break;
		default: 
			OCR2 = 0x80; //Default duty cycle is 50% 
			break;
	}//switch case

  	ADCSRA |= (1<<ADSC); //Start next ADC conversion
}


//******************************************************************************
//                            chk_buttons                                      
//Checks the state of the button number passed to it. 
//It calls the debounce switch function for each pin of port A
int chk_buttons(uint8_t PinNumber)
{
  	static uint16_t state[8] ={0}; //holds present state
  	state[PinNumber] = (state[PinNumber] << 1) | (! bit_is_clear(PINA, PinNumber)) | 0xE000;
  	if (state[PinNumber] == 0xF000) return 1;
  	_delay_ms(1);
  	return 0;
}


//This function takes in input the data received by the encoder via SPI
//Data is being processed to increment the counter properly 
uint8_t Encoder1_handling(int data_encoder, uint8_t counter)
{
        uint8_t A_value = data_encoder & 0x01; //isolate bit0 of the data sent by the encoder
        uint8_t B_value = (data_encoder>>1) & 0x01; //isolate bit1 of the data sent by the encoder  

        if(((A_old == 1) && (A_value == 0)) && ((B_old == 1) && (B_value == 1))) //If the encoder switches were toggled clockwise
        {
                direction = CW;  // set the direction 
                if((direction == 1) &((A_old != A_value) | (B_old !=  B_value)))
                //make sure to decrement by 1 from resting state to resting state, if the 2 buttons were pushed, no decrementation
        	{
                	if(counter >= 0  && (counter - 1)>=0) // Make sure to respect the boundaries 
                	{       //minute_clk = minute_clk +(inc_sign*pow(2,x)); //decrementation of the counter
                		counter--;
			}
                	else
                	{       counter = 59; //reset the counter value 
                	}

        	}
        }
        else if ((A_old == 1) && (A_value == 1)&& ((B_old == 1) && (B_value == 0))) //if the encoder switches were toggled counter clockwise
        {
                direction = CCW; //set the direction
                if ((direction == 2) & ((A_old != A_value) | (B_old != B_value)))
                //make sure to increment by 1 from resting state to resting state, if the 2 buttons were pushed, no incrementation 
                {       if(counter < 59 && (counter + 1) < 59) //Make sure to respect the boundaries
			{	counter++;} //incrementation of the counter
                        else
                        {	counter = 0;} //reset the counter value 
                }
        }

        A_old = A_value; //Update the past A switch value
        B_old = B_value; //Update the past B switch value
	return counter;
}

//This function takes in input the data received by the encoder via SPI
//Data is being processed to increment the counter properly 
uint8_t Encoder2_handling(int data_encoder, uint8_t counter, uint8_t bound)
{
        uint8_t A_value = (data_encoder>>2) & 0x01; //isolate bit0 of the data sent by the encoder
        uint8_t B_value = (data_encoder>>3) & 0x01; //isolate bit1 of the data sent by the encoder  

        if(((A2_old == 1) && (A_value == 0)) && ((B2_old == 1) && (B_value == 1))) //If the encoder switches were toggled clockwise
        {
                direction = CW;  // set the direction 
                if((direction == 1) &((A2_old != A_value) | (B2_old !=  B_value)))
                //make sure to decrement by 1 from resting state to resting state, if the 2 buttons were pushed, no decrementation
                {
                        if(counter >= 0  && (counter - 1)>=0) // Make sure to respect the boundaries 
                        {       //minute_clk = minute_clk +(inc_sign*pow(2,x)); //decrementation of the counter
                                counter--;
                        }
                        else
                        {       counter = bound; //reset the counter value 
				//hour_am = !hour_am;
                        }

                }
        }
        else if ((A2_old == 1) && (A_value == 1)&& ((B2_old == 1) && (B_value == 0))) //if the encoder switches were toggled counter clockwise
        {
                direction = CCW; //set the direction
                if ((direction == 2) & ((A2_old != A_value) | (B2_old != B_value)))
                //make sure to increment by 1 from resting state to resting state, if the 2 buttons were pushed, no incrementation 
                {       if(counter < bound && (counter + 1) < bound) //Make sure to respect the boundaries
                        {       counter++;} //incrementation of the counter
                        else
                        {       counter = 0; //reset the counter value 
				//hour_am = !hour_am;
			}
                }
        }

        A2_old = A_value; //Update the past A switch value
        B2_old = B_value; //Update the past B switch value
        return counter;
}


void Set_Clock()
{
	TCCR0 = 0x00; //Disable timer 0

	int bound = 0;
	if(!hour_format)
	{	bound = 23;
		spi_write(0xFF);
		//if((hour_am == 0) && (hour_clk>12))
		//	hour_clk = hour_clk + 12;
	}
	else
	{
		bound = 12;
		spi_write(0xF); 
		if(hour_clk>=12)
			hour_clk = hour_clk - 12;
	}
	minute_clk = Encoder1_handling(spi_read(), minute_clk);//Read encoder 1
	hour_clk = Encoder2_handling(spi_read(), hour_clk, bound); //Read encoder 2
}

void Set_Alarm()
{
	minute_alarm = Encoder1_handling(spi_read(), minute_alarm); //Read encoder 1
	hour_alarm = Encoder2_handling(spi_read(), hour_alarm, 23);  //Read encoder 2
//	if(chk_buttons(1))
	alarm_set = 1;
}

void Alarm()
{
	if((hour_clk == hour_alarm) && (snooze == 0) && (alarm_set == 1)) 
	{	if(minute_clk == minute_alarm)
		{	//spi_write(0xFF);
			trigger_alarm = 1; 
		}
	}
}
int main()
{
  	spi_init();
	TIM2_PWM_init(); 
	//ADC_init();
	TIM0_RTC_init();
	TIM1_init();
	TIM3_PWM_init();
	lcd_init();
	clear_display();
	spi_write(0x00);
        TIM3_set_PWM(40);
	ADC_init();

	ADCSRA |= (1<<ADSC); //Start next ADC conversion

	while(1)
	{	
		InitLEDdisplay();
		SeperateDigits();
		InitPushButtons();
		//asm("nop");
		//asm("nop");

                if(set_clock_ena == 1)
                {       Set_Clock();
                        //spi_write(0x01);
                }

                if(set_alarm_ena == 1)
                {       Set_Alarm();
                        //spi_write(0x02);
                }

		if(chk_buttons(0)) //Button 0 controls the clock setting
		{	if(set_clock_ena == 1)
				TIM0_RTC_init();
			set_clock_ena = !set_clock_ena; //!set_clock_ena;
		}
    
 		if(chk_buttons(1)) //Button 1 controls the alarm setting 
			set_alarm_ena = !set_alarm_ena; 

                if(alarm_set)
                        Alarm();

		if(chk_buttons(2))
		{	alarm_set = 0; //Button 2 pushed, stop completely the alarm
			trigger_alarm = 0; 
			//spi_write(0);
			clear_display();
		}	
		if(chk_buttons(3))
		{	snooze = 1;	
			trigger_alarm = 0;
			//spi_write(1);
			clear_display();	
		}
		if(chk_buttons(4))
			hour_format = !hour_format;

			
	} //while

}//main
