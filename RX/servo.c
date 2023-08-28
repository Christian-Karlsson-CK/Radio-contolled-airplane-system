#include <avr/io.h>
#include <avr/interrupt.h>
#include  "servo.h "
 
#define TIME_VALUE	(40000) 
#define RESET_VALUE	(65636ul-TIME_VALUE)
#define STOP_VALUE	(TIME_VALUE*0.076)
//(TIME_VALUE*0.075)
#define RANGE		(TIME_VALUE*0.0482)
// (TIME_VALUE*0.025)

/*
#define TIME_VALUE1   (200)   // Assuming an 8-bit timer
#define RESET_VALUE1  (256 - TIME_VALUE1)
#define STOP_VALUE1	(TIME_VALUE1*0.076)
#define RANGE1		(TIME_VALUE1*0.0482)
*/


ISR(TIMER1_OVF_vect)
{
	TCNT1 = RESET_VALUE;
 
	PORT_1 |= (1<<PIN_1);
	PORT_2 |= (1<<PIN_2);
}

ISR(TIMER1_COMPA_vect)
{
	PORT_1 &= ~(1<<PIN_1);
}
 
ISR(TIMER1_COMPB_vect)
{
	PORT_2 &= ~(1<<PIN_2);
}


/*
ISR(TIMER2_OVF_vect)
{
	//TCNT2 = RESET_VALUE1;
 
	PORT_3 |= (1<<PIN_3);
}


ISR(TIMER2_COMPA_vect)
{
	PORT_3 &= ~(1<<PIN_3);
}
 
ISR(TIMER2_COMPB_vect)
{
	PORT_4 &= ~(1<<PIN_4);
}
*/

 
void init_servo(void)
{
	
	// Config pins as output
	DDR_1 |= (1<<PIN_1);
	DDR_2 |= (1<<PIN_2);
 
	// Use mode 0, clkdiv = 8
	TCCR1A = 0;
	TCCR1B = (0<<CS12) | (1<<CS11) | (0<<CS10);
	// Interrupts on OCA, OCB and OVF
	TIMSK1 = (1<<OCIE1B) | (1<<OCIE1A) | (1<<TOIE1);
 
	TCNT1 = RESET_VALUE;
	
	servo1_set_percentage(0);
	servo2_set_percentage(0);

	//servo3_set_percentage(0);
	//servo4_set_percentage(0);
	
}
 
void servo1_set_percentage(signed char percentage)
{
	if (percentage >= -100 && percentage <= 100)
	{
		OCR1A = RESET_VALUE+STOP_VALUE+(RANGE/100*percentage);
	}
}
 
void servo2_set_percentage(signed char percentage)
{
	if (percentage >= -100 && percentage <= 100)
	{
		OCR1B = RESET_VALUE+STOP_VALUE+(RANGE/100*percentage);
	}
}

/*
void servo3_set_percentage(signed char percentage)
{
	if (percentage >= -100 && percentage <= 100)
	{
		OCR2B = RESET_VALUE1+STOP_VALUE1+(RANGE1/100*percentage);
	}
}
 
void servo4_set_percentage(signed char percentage)
{
	if (percentage >= -100 && percentage <= 100)
	{
		OCR2B = RESET_VALUE1+STOP_VALUE1+(RANGE1/100*percentage);
	}
}
*/

