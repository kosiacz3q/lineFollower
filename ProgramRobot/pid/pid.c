
#include<avr/io.h>
#include<util/delay.h>
//Define WHITE if the robot is supposed to follow the white line.
//#define WHITE


/*
 * Sets all SFRs.
*/
void initialize(void);

/*
 * Updates global sensors variable.
 */
void readSensors(void);
int  getDifference(void);
int k[7] = {-50, -25, -18, 0, 18, 25, 50};
int kP = 30;
int kD = 10;
int kI = 20;
// last working 16 10 15
void diodesDiagnose(void);

inline int max(int a, int b){ return a>b?a:b;}
inline int min(int a, int b){ return a>b?b:a;}
int round(float number){ return (number >= 0) ? (int)(number + 0.5) : (int)(number - 0.5);}
/**
 * Changes the speed of the motor.
 * Value is between 0 and 1000 (1000 is a max speed)
 *
 */
void setLeftMotorPwm(int value);
void setRightMotorPwm(int value);
enum motors{left,right};
void setMotorPwm(enum motors which, int value);



/**
 * Global variable for storing the binary input from sensors.
 * The bits are as following:
 * 2^7 = none
 * 2^6 = most left sensor
 * 2^5 = second most left
 * ...
 * 2^2 currently not working
 * ...
 * 2^0 = most right
 *
 * bit = 1 -> Sensor is above the line
 * bit = 0 -> Sensor is not above the line
 */
int sensors = 0;






int main(void)
{
	initialize();
	readSensors();
	int current_read = getDifference();
	int previous_read;

	float diffPart = 0;
	int intPart = 0;
	int propPart = 0;
	int steeringPart = 0;


	while(1)
	{

		readSensors();
		diodesDiagnose();
		previous_read = current_read;
		current_read = getDifference();
		diffPart = diffPart/1.05 + (current_read - previous_read) * kD;
		intPart += current_read * kI;
		intPart = (intPart<-1000)?(-1000):((intPart>1000)?1000:intPart);
		propPart = current_read * kP;
		steeringPart = round(diffPart) + intPart + propPart;
		if(steeringPart > 0)
		{
			setRightMotorPwm(1000 - steeringPart);
			setLeftMotorPwm(1000);

		}
		else
		{
			setRightMotorPwm(1000);
			setLeftMotorPwm(1000 + steeringPart);

		}



	}
}


void initialize(){
	// Enable output pins
		DDRB |=(1<<1); //PWM A
		DDRB |=(1<<2); //PWM B
		DDRD |=(1<<4); //DIR A1
		DDRD |=(1<<5); //DIR A2
		DDRD |=(1<<6); //DIR B1
		DDRD |=(1<<7); //DIR B2
		DDRD |=(1<<0); //Diode 0
		DDRD |=(1<<1); //Diode 1
		DDRD |=(1<<2); //Diode 2


		//Enable input pins
		DDRC &=~(1<<0); //C1
		DDRC &=~(1<<1); //C2
		DDRC &=~(1<<2); //C3
		DDRC &=~(1<<3); //C4
		DDRC &=~(1<<4); //C5
		DDRC &=~(1<<5); //C6
		DDRD &=~(1<<3); //C7

		//direction constant
		PORTD |=(1<<4); //DIR A1
		PORTD &=~(1<<5); //DIR A2
		PORTD |=(1<<6); //DIR B1
		PORTD &=~(1<<7); //DIR B2

		//PWM settings

		ICR1  = 400;
		//OCR1A = 300;
		//OCR1B = 300;

//		//FastPwm 8 bit
//		TCCR1A=(1<<COM1A1)|(1<<COM1B1)|(1<<COM1A0)|(1<<COM1B0) | (1<<WGM10);
//		TCCR1B=(1<WGM12)|(1<<CS11);
//
		TCCR1A=(1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);
		TCCR1B=(1<<WGM13)|(1<WGM12)|(1<<CS10);
}

/**
 * Set global sensor variable.
 * Each bit is responsible for one sensor.
 * 1 = sensor detected line, 0 = otherwise.
 * 2^6 - most left
 * 2^5 - second most left
 * ...
 * 2^2 - not working currently
 * 2^1 - second most right
 * 2^0 - most right
 */
void readSensors(){
	sensors = (PINC & 0b00111111);
	if(PIND & 8) sensors |= 0b01000000;
	#ifdef WHITE
	sensors = ~(sensors & 0b01111111);
	#endif

}

void setLeftMotorPwm(int value){setMotorPwm(left, value);}
void setRightMotorPwm(int value){setMotorPwm(right, value);}
void setMotorPwm(enum motors which, int value){
	// set maxes to 320 xD
	//def lmin : 210 rmin 185 lrmax 320
static const int lmin = 170;
static const int lmax = 260;
static const int rmin = 130;
static const int rmax = 255;
	switch(which){
	case left:
// min: 215
		OCR1B = max(min(lmin + round(value*((float)lmax-lmin)/1000), lmax),lmin);
		break;
	case right:
		OCR1A = max(min(rmin + round(value*((float)rmax-rmin)/1000), rmax),rmin);
		break;

	}
}

/**
 * Look at int k[]
 * This function returns k[0] if only most left sensor is above the line,
 * k[1] if only second most left sensor is above the line
 * and so on
 * If more than one sensor is above the line the return value is sum of those k's.
 *
 */
int getDifference(){
	int toReturn = 0;
	for(int i = 0; i<7; i++)
		toReturn += (sensors & (1<<( 6- i)))?k[i]:0;
	return toReturn;
}

void diodesDiagnose(void){
	// light the middle diode if middle sensor is above the line
	if((sensors & (1<<3)) > 0)
		PORTD |= 2;
	else
		PORTD &= ~(2);
	// same for right sensor and bottom diode
	if((sensors & (1<<4)) > 0)
		PORTD |= 1;
	else
		PORTD &= ~(1);
	//same for left sensor and top diode
	if((sensors & (1<<2)) > 0)
		PORTD |= 4;
	else
		PORTD &= ~(4);

}


