#include<avr/io.h>
#include<util/delay.h>

//Define WHITE if the robot is supposed to follow the white line.
//#define WHITE

/**********************************************************/
//				Global variables
/**********************************************************/
int k[7] 					= {1000, 300, 150, 0, -150, -300, -1000};
int intPartBoundaries[7]	= {1000, 300, 150, 0, -150, -300, -1000};

/*
Kp = Proptional Constant.
Ki = Integral Constant.
Kd = Derivative Constant.
err = Expected Output - Actual Output ie. error;
int  = int from previous loop + err; ( i.e. integral error )
der  = err - err from previous loop; ( i.e. differential error)
dt = execution time of loop.
*/

float kP = 1.5; 
float kD = 0.006;
float kI = 0.5;

float err;
float integralError;
float differentialError;

//experimentaly measured 
const float dt = 0.0003; 


int diffPart = 0;
float intPart = 0;
int propPart = 0;

int current_read = 0;
int previous_read = 0;

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
int activeSensor = 3;


/**********************************************************/
//				Functions declarations
/**********************************************************/

void initialize(void);
inline void updateSensors(void);
inline int  getSteeringValue(void);

// indicates values from choosed sensors (1-7) 
// di : i-diode
void diodesDiagnose(int d1, int d2, int d3);

void indicateValue(int val, int max);

inline void computeP(void); 
inline void computeI(void);
inline void computeD(void);

/**
 * Changes the speed of the motor.
 * Value is between 0 and 1000 (1000 is a max speed)
 *
 */
void setLeftMotorPwm(int value);
void setRightMotorPwm(int value);


/**********************************************************/
//						Main
/*********************************************************/

int main(void)
{
	initialize();
	
	int steeringPart = 0;

	while(1)
	{
		updateSensors();
		//diodesDiagnose(7,4,1);

		
		current_read = getSteeringValue();
		differentialError = previous_read - current_read;
		
		computeP(); 
		computeI();
		computeD();
		
		steeringPart = diffPart + (int)intPart + propPart;
		
		if (steeringPart > 1000)
			steeringPart = 1000;
		else if (steeringPart < -1000)
			steeringPart = -1000;
		
		if(steeringPart > 0)
		{
			//indicateValue(steeringPart, 1000);
			indicateValue((int)intPart, 500);
			
			setRightMotorPwm(1000 - steeringPart);
			setLeftMotorPwm(1000);
		}
		else
		{
			//indicateValue(-steeringPart, 1000);
			indicateValue((int)-intPart, 500);
			
			setRightMotorPwm(1000);
			setLeftMotorPwm(1000 + steeringPart);
		}	
		
		previous_read = current_read;
	}
}


/**********************************************************/
//				Functions definitions
/*********************************************************/

inline void computeP()
{
	propPart = current_read * kP;
}

inline void computeI()
{
	intPart += current_read * kI * dt;
	
	if (intPart < 0 && intPart < intPartBoundaries[activeSensor] )
		intPart = intPartBoundaries[activeSensor];
	else if (intPart > intPartBoundaries[activeSensor] )
		intPart = intPartBoundaries[activeSensor];
		
	
}

inline void computeD()
{
	diffPart = (differentialError * kD) / dt;
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
void updateSensors()
{
	sensors = (PINC & 0b00111111) | ((PIND & 8) ? 0b01000000 : 0);
	
	#ifdef WHITE
	sensors = ~sensors;
	#endif
	
	int leftSensor;
	int rightSensor;
	
	//selecting active sensor
	if (sensors)
	{
		for (int i = 0; i < 3; ++i)
		{
			leftSensor = (sensors & (1 << i));
			rightSensor = (sensors & (1 << (6 - i)));
		
			if (leftSensor && rightSensor)
			{
				activeSensor = 3;
				return;
			}
		
			if (leftSensor)
			{
				activeSensor = i;
				return;
			}
			
			if (rightSensor)
			{
				activeSensor = 6 - i;
				return;
			}
		}
		
		//intPart = 0; //if only central sensor is active, We want to reset integral part  
		activeSensor = 3;
	}
}

const int lmin = 150;
const int rmin = 130;

//test
const int rmax = 200;
const int lmax = 220;

//max
//const int rmax = 280;
//const int lmax = 300;

const int maxSpeed = 1000;

void setLeftMotorPwm(int value)
{
	OCR1B = lmin + (int)(value * ((float)lmax-lmin) / maxSpeed);
}

void setRightMotorPwm(int value)
{
	OCR1A = rmin + (int)(value * ((float)rmax-rmin) / maxSpeed);
}

int getSteeringValue()
{
	return k[activeSensor];
}

void diodesDiagnose(int d1, int d2, int d3)
{
	//d1
	if((sensors & (1 << (d1 - 1))) > 0)
		PORTD |= 1;
	else
		PORTD &= ~(1);
	
	//d2
	if((sensors & (1 << (d2 - 1))) > 0)
		PORTD |= 2;
	else
		PORTD &= ~(2);
		
	//d3
	if((sensors & (1 << (d3 - 1))) > 0)
		PORTD |= 4;
	else
		PORTD &= ~(4);
}

void indicateValue(int val, int max)
{
	PORTD &= ~(1);
	PORTD &= ~(2);
	PORTD &= ~(4);

	if (val <= max / 4)
	{
		//nothing to do
	}
	else if (val <= 2 * max / 4)
	{
		PORTD |= 1;
	}
	else if (val <= 3 * max / 4)
	{
		PORTD |= 2;
	}
	else
	{
		PORTD |= 4;
	}
}

void initialize()
{
	// Enable output pins
	DDRB |= (1 << 1); //PWM A
	DDRB |= (1 << 2); //PWM B
	DDRD |= (1 << 4); //DIR A1
	DDRD |= (1 << 5); //DIR A2
	DDRD |= (1 << 6); //DIR B1
	DDRD |= (1 << 7); //DIR B2
	DDRD |= (1 << 0); //Diode 0
	DDRD |= (1 << 1); //Diode 1
	DDRD |= (1 << 2); //Diode 2


	//Enable input pins
	DDRC &= ~(1 << 0); //C1
	DDRC &= ~(1 << 1); //C2
	DDRC &= ~(1 << 2); //C3
	DDRC &= ~(1 << 3); //C4
	DDRC &= ~(1 << 4); //C5
	DDRC &= ~(1 << 5); //C6
	DDRD &= ~(1 << 3); //C7

	//direction constant
	PORTD |= (1 << 4); //DIR A1
	PORTD &= ~(1 << 5); //DIR A2
	PORTD |= (1 << 6); //DIR B1
	PORTD &= ~(1 << 7); //DIR B2

	//PWM settings

	ICR1  = 400;
	//OCR1A = 300;
	//OCR1B = 300;

	TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);
}

