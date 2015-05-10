#include <avr/io.h>
#include <util/delay.h>

int main (void)
	{	
		
		//output
	
		DDRB |=(1<<1);		//PWM A
		DDRB |=(1<<2);		//PWM B
		
		DDRD |=(1<<4);		//DIR A1
		DDRD |=(1<<5);		//DIR A2
		DDRD |=(1<<6);		//DIR B1
		DDRD |=(1<<7);		//DIR B2
		
		DDRD |=(1<<0);		//Diode 0
		DDRD |=(1<<1);		//Diode 1
		DDRD |=(1<<2);		//Diode 2
		
		//input
		
		DDRC &=~(1<<0);	//C1
		/*
		DDRC &=~(1<<1);	//C2
		DDRC &=~(1<<2);	//C3
		DDRC &=~(1<<3);	//C4
		DDRC &=~(1<<4);	//C5
		DDRC &=~(1<<5);	//C6
		DDRD &=~(1<<3);	//C7*/
		
		//direction constant
		
		//magic
		PORTD |=(1<<4);		//DIR A1
		PORTD &=~(1<<5);		//DIR A2
		PORTD |=(1<<6);		//DIR B1
		PORTD &=~(1<<7);		//DIR B2
		
		//PWM settings
		
		ICR1  = 399;
		OCR1A = 300;
		OCR1B = 300;
		
		TCCR1A=(1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);
		TCCR1B=(1<<WGM13)|(1<WGM12)|(1<<CS10);
				
		int i; //iterator
		
		while(1)
		{
			for(i=0;i<=100;i++)
			 {
			   OCR1A=i;
			   OCR1B=399-i;  
			   _delay_ms(10);
			 }
			 
			 PORTD |=(1<<0);
			 
			 for(i=100;i<=200;i++)
			 {
			   OCR1A=i;
			   OCR1B=399-i;  
			   _delay_ms(10);
			 }
			 
			 PORTD |=(1<<1); 
			 
			 for(i=200;i<=300;i++)
			 {
			   OCR1A=i;
			   OCR1B=399-i;  
			   _delay_ms(10);
			 }
			 
			 PORTD |=(1<<2);
			 
			 for(i=300;i<=399;i++)
			 {
			   OCR1A=i;
			   OCR1B=399-i;  
			   _delay_ms(10);
			 }
			 
		/*	 
			for(i=399;i>=0;i--)
			 {
			   OCR1A=i;
			   OCR1B=399-i;  
			   _delay_ms(10);
			 }
		
			if (!(PIND & (1<<3)))
			{
				PORTD |=(1<<0);
			}
			else
			{
				PORTD &=~(1<<0);
			}
			
			if (!(PINC & (1<<0)))
			{
				PORTD |=(1<<1);
			}
			else
			{
				PORTD &=~(1<<1);
			}
			
			if (!(PINC & (1<<1)))
			{
				PORTD |=(1<<2);
			}
			else
			{
				PORTD &=~(1<<2);
			}
*/

		}
	}