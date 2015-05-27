#include<avr/io.h>
#include<util/delay.h>

//Define WHITE if the robot is supposed to follow the white line.
//#define WHITE

/**********************************************************/
//				Global variables
/**********************************************************/
int k[7] 					= {1000, 400, 200, 0, -200, -400, -1000};
int intPartBoundaries[7]	= {450, 350, 250, 0, -250, -350, -450};

/*
Kp = Proptional Constant.
Ki = Integral Constant.
Kd = Derivative Constant.
err = Expected Output - Actual Output ie. error;
int  = int from previous loop + err; ( i.e. integral error )
der  = err - err from previous loop; ( i.e. differential error)
dt = execution time of loop.
*/

float kP = 2; 
float kD = 1;
float kI = 4;

float err;
float integralError;
float differentialError;

//experimentaly measured 
const float dt = 0.0003; 
const int brakeForce = 600;

float diffPart = 0;
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
 * 2^0 = most right
 *
 * bit = 1 -> Sensor is above the line
 * bit = 0 -> Sensor is not above the line
 */
int sensors = 0;
int activeSensor = 3;
int lastKnownNonCenterSensor = 3;

int isSensorDetected;

/**********************************************************/
//				Functions declarations
/**********************************************************/

void initialize(void);
inline void updateSensors(void);
inline int  getSteeringValue(void);

// indicates values from choosed sensors (1-7) 
// di : i-diode
void diodesDiagnose(int d1, int d2, int d3);
void binaryOutput(int val);
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
	
	int steeringValue = 0;
	activeSensor = 3;
	isSensorDetected=1;
	
	while(1)
	{
		updateSensors();
		
		current_read = getSteeringValue();
		differentialError = current_read - previous_read;
		
		computeP(); 
		computeI();
		computeD();
		
		steeringValue = diffPart + (int)intPart + propPart;
		
		if (steeringValue > 1000)
			steeringValue = 1000;
		else if (steeringValue < -1000)
			steeringValue = -1000;
		
		if(!isSensorDetected && lastKnownNonCenterSensor != 3)
		{
			if (lastKnownNonCenterSensor > 3)
			{
				intPart = diffPart - 0;
				setRightMotorPwm(1000);
				PORTD &= ~(1 << 6); //DIR B1
				PORTD |= (1 << 7); //DIR B2
				setLeftMotorPwm(brakeForce);			
			}
			else if (lastKnownNonCenterSensor < 3)
			{
				intPart = diffPart = 0;
				PORTD &= ~(1 << 4); //DIR A1
				PORTD |= (1 << 5); //DIR A2
				setRightMotorPwm(brakeForce);
				setLeftMotorPwm(1000);
			}
		
		}
		else if(steeringValue > 0)
		{
			PORTD |= (1 << 4); //DIR A1
			PORTD &= ~(1 << 5); //DIR A2	
			PORTD |= (1 << 6); //DIR B1
			PORTD &= ~(1 << 7); //DIR B2		
			setRightMotorPwm(1000 - steeringValue);
			setLeftMotorPwm(1000);
		}
		else
		{
			PORTD |= (1 << 4); //DIR A1
			PORTD &= ~(1 << 5); //DIR A2	
			PORTD |= (1 << 6); //DIR B1
			PORTD &= ~(1 << 7); //DIR B2				
			setRightMotorPwm(1000);
			setLeftMotorPwm(1000 + steeringValue);
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
	diffPart = diffPart * (1 - 0.002) + (differentialError * kD);
}


/**
 * Set global sensor variable.
 * Each bit is responsible for one sensor.
 * 1 = sensor detected line, 0 = otherwise.
 * 2^6 - most left
 * 2^5 - second most left
 * ...
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

	for (int i = 0; i < 3; ++i)
	{
		leftSensor = (sensors & (1 << i));
		rightSensor = (sensors & (1 << (6 - i)));
	
		if (leftSensor && rightSensor)
		{
			activeSensor = 3;
			isSensorDetected = 1;
			return;
		}
	
		if (leftSensor)
		{
			activeSensor = i;
			lastKnownNonCenterSensor = i;
			isSensorDetected = 1;
			return;
		}
		
		if (rightSensor)
		{
			isSensorDetected = 1;
			lastKnownNonCenterSensor = 6-i;
			activeSensor = 6 - i;
			return;
		}
	}
	
	if(sensors & (1 << 3))
	{
		activeSensor = 3;
		isSensorDetected = 1;
		return;
	} 
	
	isSensorDetected = 0;
}

const int lmin = 0;
const int rmin = 0;

const int rmax = 380;
const int lmax = 400;

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

void binaryOutput(int val)
{
	PORTD &= ~(1);
	PORTD &= ~(2);
	PORTD &= ~(4);
	if(val & 4) PORTD |= 4;
	if(val & 2) PORTD |= 2;
	if(val & 1) PORTD |= 1;
}
