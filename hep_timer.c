/*
   Copyright 2009-present Keith Daigle

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
//outputs
//PORTA pin 7 is OCR0B used for map PWM, low pass filter does DAC
#define MAP_OUT 7
//PORTA pin 5 is VSS output pin
#define VSS_OUT 5

//PORTB pin 1 is 1st hep channel output pin, should be ref pickup, runs behind 2nd channel sync
#define HEP1_OUT 1
//PORTB pin 3 is 2nd hep channel output pin, should be sync pickup
#define HEP2_OUT 3
//PORTB pin 2 is OCR0A used for TPS with PWM
#define TPS_OUT 2


//inputs, also used for indexs in array of outputs and adc channels to convert
//PORTA pin 4 is input for MAP signal from pot
#define MAP_IN 4
//PORTA pin 1 is input for HEP signal from pot
#define HEP_IN 1
//PORTA pin 2 is input for VSS signal from pot
#define VSS_IN 2
//PORTA pin 3 is input for TPS signal from pot
#define TPS_IN 3
//start/stop button on PORTA pin 6
#define START_IN 6
//2bar/3bar baro selector
#define BARNESS_IN 0
//baro read on PORTB pin0
#define BARO_IN 0

//Indexes into adc outputs array
#define MAP_IDX 0
#define HEP_IDX 1
#define VSS_IDX 2
#define TPS_IDX 3

//Number of adc channels in use, needs to be set in the adc_inputs array
#define NUM_ADC 4

//This is for the baro duty cycle, this is 2 bar, slightly above sea level
//This should eventually be a variable so 3bar/2bar can be changed on fly
#define BARO_DC_TWO_BAR 0x7C
#define BARO_DC_THREE_BAR 0x52

//This is the minimum TPS, anything below a certian value the smec will reject as out of range
#define MIN_TPS 0x25

//This is the maximum TPS, anything above a certian value the smec will reject as out of range
#define MAX_TPS ( 0xff - MIN_TPS )

//This is offset of the 2 pickups in time ticks, as (degrees of offset/360)*1000
#define OFFSET 20

//set debounce timer
#define DEBOUNCE_TICKS 10000

//set vss base number of ticks amount
#define BASE_VSS_TICKS 4


//Array below is is used to setup the hep signal times, both pickups are run from the same
//array, and which pickup is changing state is given by the ch_to_toggle array
//Only the differences between each event are clocked.  These events are based on a total of
//1000, and the values therein are calculated by taking (degrees of rotation for edge/360)*1000
//offset is used to space the 2 pickups beyond the 180*  
static const unsigned int differences[20] = {OFFSET, 150-OFFSET, OFFSET, 250-(150+OFFSET), 
											OFFSET, 400-(250+OFFSET),OFFSET, 29, 42, 29-OFFSET, 
											OFFSET, 650-(500+OFFSET), OFFSET, 750-(650+OFFSET), 
											OFFSET, 900-(750+OFFSET), OFFSET, 29-OFFSET, 42, 29};

//this is which channel to toggle on each ISRs
//static const unsigned char ch_to_toggle[20] = {1,0,1,0,1,0,1,1,1,0,1,0,1,0,1,0,1,0,0,0};
static const unsigned char pin_to_toggle[20] = {0x02,0x08,0x02,0x08,0x02,0x08,0x02,0x02,0x02,0x08,
												0x02,0x08,0x02,0x08,0x02,0x08,0x02,0x08,0x08,0x08};

//Array of adc values to be used in loading timers, this is the initialization
volatile unsigned char adc_outputs[NUM_ADC] = {BARO_DC_THREE_BAR,0x00,0x00, MIN_TPS};

//This is the ADC channel to read for the input of each of the values
//this matches pin nubmer on port A
static unsigned char adc_inputs[NUM_ADC] = {(0<<MUX0)|(0<<MUX1)|(1<<MUX2),(1<<MUX0)|(0<<MUX1)|(0<<MUX2),
											(0<<MUX0)|(1<<MUX1)|(0<<MUX2),(1<<MUX0)|(1<<MUX1)|(0<<MUX2)};

volatile unsigned char map_jittered = BARO_DC_THREE_BAR;

//Flag for baro reading
volatile unsigned char reading_baro = 0;

//This is the baro value, gets set in main after 2 bar
//3 bar selection
static unsigned char barodc = 0;

//counter for start/stop switch debounce
volatile unsigned long int ignore_bounces = 0;

//Counter for vss toggle, used on overflow of timer0
volatile unsigned long int vss_overflows_left = BASE_VSS_TICKS;
/*
//Isr is run when timer 1 is reset to bottom
//At that point one of the pins is toggled (based upon ch_to_toggle)
//the timer is reloaded with next sleep amount
//and the position in the rotation is updated
ISR(TIM1_COMPA_vect)
{
	//This is the HEP position in the differences and channel array
	static unsigned char hep_position=0;

	//toggle appropriate pin
	if(ch_to_toggle[hep_position++])
		PORTB ^= (1<<HEP1_OUT);
	else
		PORTB ^= (1<<HEP2_OUT);
	//make sure our position in the rotation 
	//stays within the array boundaries
	hep_position %= 20;

	//load timer with next toggle point
	//the 5+ part ensures maximum rpm is not exceeded, then scales by whatever the adc value is
	OCR1A=(5 + adc_outputs[HEP_IDX]) * differences[hep_position];

return;
}
*/
//Faster, more compact version of ISR for HEP
//Isr is run when timer 1 is reset to bottom
//At that point one of the pins is toggled (based upon ch_to_toggle)
//the timer is reloaded with next sleep amount
//and the position in the rotation is updated
ISR(TIM1_COMPA_vect)
{
	//This is the HEP position in the differences and channel array
	static unsigned char hep_position=0;

	//toggle appropriate pin
	PINB = pin_to_toggle[hep_position++];
	//make sure our position in the rotation 
	//stays within the array boundaries
	if (hep_position >= 20)
		hep_position = 0;

	//load timer with next toggle point
	//the 5+ part ensures maximum rpm is not exceeded, then scales by whatever the adc value is
	OCR1A=(5 + adc_outputs[HEP_IDX]) * differences[hep_position];

return;
}

//This isr is run when timer0 overflows, should be used to
//reset the pwm values for the MAP and TPS, also used to constrain the 2
//and load baro value if baro currently being read
ISR(TIM0_OVF_vect)
{
//ocr0x is double buffered, so loading the timer value isn't as critical as timer1's mode
//Need code to check adc value and scale duty cycle
//Raw adc is taken here


//Range check the TPS values

if(adc_outputs[TPS_IDX] > MAX_TPS)
	OCR0A=MAX_TPS;

else if(adc_outputs[TPS_IDX] < MIN_TPS)
	OCR0A=MIN_TPS;

else
	OCR0A=adc_outputs[TPS_IDX];

//always use baro reading while 
//the ecu has that relay grounded

if(reading_baro)
	OCR0B=barodc;
else
	OCR0B=adc_outputs[MAP_IDX];


//Use timer overflows as time to ignore switch bounces
if(ignore_bounces)
	ignore_bounces--;

//handle vss if last overflow is reached
if(!(--vss_overflows_left))
	{
		//PORTA ^= (1<<VSS_OUT);
		PINA = (1<<VSS_OUT);
		vss_overflows_left = 5 +  adc_outputs[VSS_IDX];
	}
return;
}

//This is run each time a a2d conversion completes
//The output is only taken as 8 bits and loaded into
//the outputs array, and the next channel in line has it's
//adc conversion started
ISR(ADC_vect)
{

	//Position in adc array, main starts from 0
	static unsigned char adc_position = 0;

	//Position in adc counting, main starts from 0
	adc_outputs[adc_position++]=ADCH;

//	adc_position %= NUM_ADC;
	if(adc_position >= NUM_ADC)
		adc_position = 0;

	//This will reset the to the next channel

	ADMUX=(0<<REFS0)|(0<<REFS1)|(adc_inputs[adc_position]);

	//Start the next conversion
	ADCSRA|=(1<<ADSC);

return;
}
//Interrupt handler for start/stop
ISR(PCINT0_vect)
{
	//The pushbutton switches bounce, causing this ISR
	//to run when it shouldn't, this causes the isr
	//not to be run for 10000 overflows of timer 0
	//which ticks at 8mhz; total about 1/3rd of a second
	if(!ignore_bounces)
	{
		TCCR1B ^= (1<<CS11);
		ignore_bounces = DEBOUNCE_TICKS;
	}

return;
}

//Interrupt handler for baro
ISR(PCINT1_vect)
{
	//toggle the baro reading flag
	//this shouldn't bounce since it's done by logic
	reading_baro^=1;
	return;
}

int main(void)
{
//Set port and pin state/direction

	// Set Port A pins input/output
	DDRA = (1<<MAP_OUT)|(1<<VSS_OUT)|(0<<MAP_IN)|(0<<VSS_IN)|(0<<HEP_IN)|(0<<TPS_IN)|(0<<START_IN)|(0<<BARNESS_IN);

	//Set port b pins as input/output
	DDRB = (1<<TPS_OUT)|(0<<BARO_IN)|(1<<HEP1_OUT)|(1<<HEP2_OUT);


	//set baro pin to be low, change and hold in high state to set baro DC
	//start button pulls pin to gnd
	PORTA = (1<<START_IN)|(0<<BARNESS_IN)|(0<<VSS_OUT);

	//set  hep1 low, and 2 high (will be inverted by npn)
	//Set b pins to all be low
	PORTB = (0<<HEP1_OUT)|(1<<HEP2_OUT)|(0<<BARO_IN);

//Setup Timer1, 16 bit timer for HEP signal, ISR does most of work

	//Disable toggling for the output compares on timer1
	TCCR1A=0x00;

	//Set waveform generation mode to CTC scaler is not set until started
	TCCR1B = (1<<WGM12);

	//set initial toggle point
	OCR1A=((5 + adc_outputs[HEP_IN]) * differences[0]);

	//Enable timer interrupts for the OCR1A compare register
	TIMSK1 = (1<<OCIE1A);

//Timer 0 initiliaztion, used for PWM Map output signal, VSS output

	//Set to non-inverted PWM for ocr0a and ocr0b pins, and set wave generation to fast pwm
	TCCR0A = (1<<COM0A1)|(1<<COM0B1)|(1<<WGM00)|(1<<WGM01);

	//Set the clock scaler for timer0, runs = to system clock for fastest pwm freq possible
	TCCR0B = (1<<CS00);

	//Set initial duty cycle to minimum TPS and Baro press
	OCR0A=MIN_TPS;
	OCR0B=BARO_DC_THREE_BAR;

	//Set the interrupt mask to run interrupt on Overflow (at top)
	//this ensures it's always run, match compare may not run interrupt under certian cirmcustances
	TIMSK0 = (1<<TOIE0);

//ADC initilization
	//disable digital inputs for the 4 adc channels
	DIDR0 = (1<<MAP_IN)|(1<<VSS_IN)|(1<<HEP_IN)|(1<<TPS_IN);

	//set admux to use vref, and the initial channel is the map input
	//ADMUX= (0<<REFS0)|(0<<REFS1)|adc_inputs[0];
	ADMUX=(0<<REFS0)|(0<<REFS1)|(adc_inputs[0]);

	//Left justify result to take 8 bit resolution only
	ADCSRB = (1<<ADLAR);
	//Enable adc, start conversion, enable interrupt, and set prescaler to 64
	ADCSRA= (1<<ADEN)|(1<<ADSC)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(0<<ADPS0);

//Pin change interrupt settings

	//enable interrupt for PCINT0 on port a pin 6 (start/stop input)
	PCMSK0 = (1<<PCINT6);
	//enable interrupt for PCINT1 on port b pin 0 (baro input)
	PCMSK1 = (1<<PCINT8);

	// enable the interrupts for the 2 pin changes
	GIMSK = (1<<PCIE0)|(1<<PCIE1);
//	GIMSK = (1<<PCIE0);
	//if the 3 bar jumper is installed use the 3 bar dc
	if(PORTA & (1<<BARNESS_IN))
		barodc=BARO_DC_THREE_BAR;
	else
		barodc=BARO_DC_TWO_BAR;

	//enable interrupts
	sei();

	//to infinity.....
	while(1){
	//Make the map jitter a little on start so smec thinks it's valid
/*	if(ignore_bounces % 2)
		map_jittered=adc_outputs[MAP_IDX]+1;
	else
		map_jittered=adc_outputs[MAP_IDX]-2;	
*/
	}
	return 1;
}
