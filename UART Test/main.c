#include "hardwareprofile.h"
#include <stdio.h>
#include <stdlib.h>
#include <dsp.h>

#define NEXT_SPEED_BUFFER_SIZE		25

//Set up processor registers
_FOSCSEL(FNOSC_FRCPLL)									//Use built in oscillator
_FOSC(POSCMD_NONE & OSCIOFNC_ON)					//Use Oscillator pins as I/O pins is on
_FWDT(FWDTEN_OFF)									//Turn watchdog timer off
_FICD(ICS_PGD1 & JTAGEN_OFF)						//Set to programming interface 1 and no JTAG

//UART Functions
unsigned char UART1DataReady(void);					//Check for available data in RX buffer
unsigned char UART1Data(void);						//Get byte from RX buffer
void UART1AddByte(char c);							//Add byte to TX buffer, pause if it's full
void InitUART1(void);								//Initialize UART
void UART1Println(char string[]);					//Custom print line function (because the built in one sucks...)

//Motor Control Functions
void InitQEI1(void);								//Initialize the Quadrature Encoder Interface for motor A
void InitTMR1(void);								//Initialize the timer for the PID calculation interrupt
void InitPid();										//Initialize PID
void setMotorACoeffs(float kP, float kI, float kD);	//Set PID Gains for motor A
void setMotorBCoeffs(float kP, float kI, float kD);	//Set PID Gains for motor B
float calcNextTrap(float maxSpeed, float * currentSpeed, float accel, float decel, float distance, float * distanceTraveled);	// Calculates next speed values for a trapezoidal movement profile
void setup(float diameterA, float diameterB, float spacing, int countsA, int countsB);
int TrapezoidalMovement(float maxSpeed, float accel, float decel, float distance, int motors, int dir);	// Calculate the next appropriate value in the trapezoidal motion profile and try to place it in the buffer
int ConstantVelocity(float velocity, int dir, int motor);	// Place the next CV value in the buffer
int stickThisInTheBuffer(float bufferValue, int motors);	// Places a floating point value in nextSpeed for A,B or both
int TurnOnAxis(float velocity, int dir, int degrees);
void Stop(int coast, int motors);

//Legacy Functions
void PositionCalculation(void);						// !!Legacy!!

//Begin motor control variables
// Note: "Bool" -> 0 = False, 1 = True
int pidCalcA = 0;			//Perform PID algorithm calculations in the interupt? (BOOL)
int pidCalcB = 0;

//Setup variables
float wheelDiameterA;							//Wheel diameter for wheel attached to motor A
float wheelDiameterB;							//Wheel diameter for wheel attached to motor B
float wheelSpacing;								//Spacing between wheels
int quadCountsA;								//Number of counts per revolution for motor A quadrature encoder
int quadCountsB;								//Number of counts per revolution for motor B quadrature encoder
float distancePerCountA = 0.12;
float distancePerCountB = 0.12;

//PID Variables
tPID motorAPid;									//PID data structure for Motor A
tPID motorBPid;									//PID data structure for Motor B
float measuredSpeedA;
float measuredSpeedB;
int motorActiveA;
int motorActiveB;

float nextSpeedA[NEXT_SPEED_BUFFER_SIZE];		//MTRA Ring buffer for storing speeds that the PID algorithm consumes as the control reference
volatile int nextSpeedAWriteCounter = 0;		//Indicates the next index to be written to in the nextSpeedA buffer
volatile int nextSpeedAReadCounter = -1;			//Indicates the next index to be read from in the nextSpeedA buffer

float nextSpeedB[NEXT_SPEED_BUFFER_SIZE];		//MTRA Ring buffer for storing speeds that the PID algorithm consumes as the control reference
volatile int nextSpeedBWriteCounter = 0;		//Indicates the next index to be written to in the nextSpeedB buffer
volatile int nextSpeedBReadCounter = -1;		//Indicates the next index to be read from in the nextSpeedB buffer

volatile int velocity;							//velocity measurement
int PosA[2] = {0,0};							//last and present position
int PosB[2] = {0,0};

int stayCVA = 0;					//Indicates that motor A should remain at a constant velocity
float CVA = 0;						//The constant velocity that A should remain at
int stayCVB = 0;					//Indicates that motor B shoudl remain at a constant velocity
float CVB = 0;						//The constant velocity that B should remain at

// Movement calculation Variables
float currentSpeedA = 0;				//Last speed calculated, used for iterative calculations of velocity
float currentSpeedB = 0;
float distanceTraveledA= 0;			//Distance traveled for the current function
float distanceTraveledB = 0;
int ConstantVelocityCounter = 0;	// Used by constantVelocity to determine if the CV function has generated enough points
float timeSlice = .0085;
int neededDataPoints;				// The number of data points needed for a movement command, trucated
int numberOfGeneratedPoints = 0;	// The number of data points that have been generated so far by a movement command

volatile unsigned char c[4];
volatile int testNum = 0;
volatile int switchTest = 0;

int neededDataPoints;				// The number of data points needed for a movement command, trucated
int numberOfGeneratedPoints = 0;	// The number of data points that have been generated so far by a movement command

//**************************************//
// The variables below will be        //
//      provided by various              //
//      packets from the Arduino     //
//**************************************//
float userProvidedTime;						// How long the user wants the motor(s) to move
float wheelDiameterA;						//Wheel diameter for wheel attached to motor A
float wheelDiameterB;						//Wheel diameter for wheel attached to motor B
float wheelSpacing;							//Spacing between wheels
int quadCountsA;								//Number of counts per revolution for motor A quadrature encoder
int quadCountsB;								//Number of counts per revolution for motor B quadrature encoder
float velocity;										// The max speed provided by the user for various functions
float accel;
float decel;
int direction;										// 0 = clockwise, 1 = counterclockwise
int motors;										// 0 = A, 1 = B, 2 = A and B
float distance;									// The distance the user wishes to travel
int degrees;										// Degrees by which to rotate for turn on Axis and possibly turn on radius
int coast;											// 0 = coast, 1 = brake
//*****End of provided globals*******//

//PID variable definitions
fractional mAabcCoeffs[3] __attribute__ ((space(xmemory)));		//((section (".xbss, bss, xmemory")));
fractional mBabcCoeffs[3] __attribute__ ((space(xmemory)));		//((section (".xbss, bss, xmemory")));
fractional mAcontHist[3] __attribute__ 	((space(ymemory)));		//((section (".ybss, bss, ymemory")));
fractional mBcontHist[3] __attribute__ 	((space(ymemory)));		//((section (".ybss, bss, ymemory")));
fractional kCoeffsA[3];// = {0.2,0,0};
fractional kCoeffsB[3];// = {0.2,0,0};

int main(void)
{
	AD1PCFGL = 0xFFFF;  //make all pins digital

	//setup internal clock for 80MHz/40MIPS
	//7.37/2=3.685*43=158.455/2=79.2275

	CLKDIVbits.PLLPRE=0; 		// PLLPRE (N2) 0=/2 
	PLLFBD=41; 			//pll multiplier (M) = +2
	CLKDIVbits.PLLPOST=0;		// PLLPOST (N1) 0=/2
	 	
	while(!OSCCONbits.LOCK);	//wait for PLL to stabilize


	//Set up UART
	InitUART1();
	
	//Set up Quadrature Encoder Interface A
	InitQEI1();

	//Set up PID
	InitPid();

	//Set up timer 1 (Interrupts for velocity calculations)
	InitTMR1();

	// Setup status flags as inputs
	MTR_A_SF_TRIS = 1;
	MTR_B_SF_TRIS = 1;
	
	//*****Initialize PWM******
	//Set pwm pins to outputs
	MTR_AH_PWMTRIS = 0;		
	MTR_AL_PWMTRIS = 0;
	MTR_BH_PWMTRIS = 0;
	MTR_BL_PWMTRIS = 0;
	
	//Clear any data from pwm pins
	MTR_AH_PWM_O = 0;		
	MTR_AL_PWM_O = 0;
	MTR_BH_PWM_O = 0;
	MTR_BL_PWM_O = 0;
	
	PWM_POSTSCALE = 0;			//No pwm timer scaling
	PWM_PRESCALE = 0;
	PWM_TIMEBASEMODE = 0;		//set to free running
	
	PWM_TMRSTART = 0;			//start counter at 0
	
	PWM_TIMEBASE_PER = TIMEBASE_PERIOD;	//set the timebase period for the pwm module
	
	PWM_CONFbits.PMOD3 = 1; 	// PWM in independent mode
	PWM_CONFbits.PMOD2 = 1; 	// PWM in independent mode
	PWM_CONFbits.PMOD1 = 1; 	// PWM in independent mode
	PWM_CONFbits.PEN3H = 0; 	// PWM High pin is disabled
	PWM_CONFbits.PEN2H = 1; 	// PWM High pin is enabled
	PWM_CONFbits.PEN1H = 1; 	// PWM High pin is enabled
	PWM_CONFbits.PEN3L = 0; 	// PWM Low pin disabled 
	PWM_CONFbits.PEN2L = 0; 	// PWM Low pin disabled 
	PWM_CONFbits.PEN1L = 0; 	// PWM Low pin disabled

	MTR_A_DUTY_CYCLE = 0; //TIMEBASE_PERIOD;		//Set duty cycle to 50%
	MTR_B_DUTY_CYCLE = 0; //TIMEBASE_PERIOD;
	
	PWM_TMR_ENABLE = 1;			//Enable the PWM Timerbase
	
	//Set up PID gains
	setMotorACoeffs(0.5, 0.49, 0.01);
	setMotorBCoeffs(0.5, 0.49, 0.01);
	
	//Set the control reference and measured output
	//(This is just for testing purposes)
	//motorAPid.controlReference = Q15(0.74);
	///motorBPid.controlReference = Q15(0.74);
	//motorAPid.measuredOutput = Q15(0.453);
	//motorBPid.measuredOutput = Q15(0.453);

	// Test Variables
	velocity = 2.5;	// Velocity of 2 m/s
	accel = 1;
	decel = 1;
	direction= 0;		// Spinning clockwise
	motors = 0;		// Testing only motor A
	distance = 5;
	degrees = 15;
	userProvidedTime = 0;
	wheelSpacing = 8;
	coast = 1;
	//userProvidedTime = 5*timeSlice;

	setup(0.12, 0.12, 1, 200, 200);

	//stayCVA = 1;
	//CVA = 5;
	//pidCalcA = 1;
	//stayCVB = 1;
	//CVB = 2.5;
	//pidCalcB = 1;
	int test = 1;
	while(test == 1){
	
		test = TrapezoidalMovement(5, 1, 1, 30, 2, 1);
		
	}

	while(1){}
/*
		// Trapezoidal movement 
		while (TrapezoidalMovement(velocity, accel, decel, distance, motors, dir) == 1)
		{
			nextSpeedAReadCounter++;
			//nextSpeedBReadCounter++;
		}
*/
		while (TurnOnAxis(velocity, direction, degrees) == 1)
		{
			nextSpeedAReadCounter++;
			nextSpeedBReadCounter++;
		}

		Stop(coast, motors);
		nextSpeedAReadCounter++;
		
	return 0;
}

//Initialize PID
void InitPid()
{
	motorAPid.abcCoefficients = &mAabcCoeffs[0];
	motorBPid.abcCoefficients = &mBabcCoeffs[0];

	motorAPid.controlHistory = &mAcontHist[0];
	motorBPid.controlHistory = &mBcontHist[0];

	PIDInit(&motorAPid);
	PIDInit(&motorBPid);

	PIDCoeffCalc(&kCoeffsA[0], &motorAPid);
	PIDCoeffCalc(&kCoeffsB[0], &motorBPid);
}

//Speed Caclulation ISR
void __attribute__((__interrupt__)) _T1Interrupt(void)
{
	int outputA;
	int outputB;
	int dirA;
	int dirB;
	
	//First copy the position count to get a snapshot
	int POS1CNTcopy = (int)POS1CNT;
	int POS2CNTcopy = (int)POS2CNT;

	//If the count happens to be less than zero, make it positive
	//(I need to check this, because i'm not sure it makes sense...)
	//if (POSCNTcopy < 0)
	//	POSCNTcopy = -POSCNTcopy;

	//Cycle the old position into the lower array and place the copy
	//in the new spot
	PosA[1] = PosA[0];
	PosA[0] = POS1CNTcopy;
	
	PosB[1] = PosB[0];
	PosB[0] = POS2CNTcopy;

	//Calculate velocity
	//The decimal value 0.0085 needs to be changed based on
	//the interrupt interval
	if(QEI1CONbits.UPDN == 1){
		if(PosA[0] >= PosA[1]){
			measuredSpeedA = ((float)(PosA[0] - PosA[1])/timeSlice) * distancePerCountA;
		}
		else{
			measuredSpeedA = ((float)(PosA[0] - PosA[1] + quadCountsA)/timeSlice) * distancePerCountA;
		}
	}
	else
	{
		if(PosA[0] <= PosA[1]){
			measuredSpeedA = ((float)(PosA[0] - PosA[1])/timeSlice) * distancePerCountA;
		}
		else{
			measuredSpeedA = ((float)(PosA[0] - PosA[1] - quadCountsA)/timeSlice) * distancePerCountA;
		}
	}

	if(QEI2CONbits.UPDN == 1){
		if(PosB[0] >= PosB[1]){
			measuredSpeedB = ((float)(PosB[0] - PosB[1])/timeSlice) * distancePerCountB;
		}
		else{
			measuredSpeedB = ((float)(PosB[0] - PosB[1] + quadCountsB)/timeSlice) * distancePerCountB;
		}
	}
	else
	{
		if(PosB[0] <= PosB[1]){
			measuredSpeedB = ((float)(PosB[0] - PosB[1])/timeSlice) * distancePerCountB;
		}
		else{
			measuredSpeedB = ((float)(PosB[0] - PosB[1] - quadCountsB)/timeSlice) * distancePerCountB;
		}
	}
	
	if(pidCalcA == 1){
		if(stayCVA == 1)
		{
			motorAPid.controlReference = Q15(CVA / 20);
		}
		else
		{
			int lookAhead = nextSpeedAReadCounter + 1;
			if(lookAhead == NEXT_SPEED_BUFFER_SIZE)
			{
				lookAhead = 0;
			}
			
			if(lookAhead != nextSpeedAWriteCounter)
			{
				nextSpeedAReadCounter = lookAhead;
			}
			
			motorAPid.controlReference = Q15(nextSpeedA[nextSpeedAReadCounter] / 20);
		}
		
		motorAPid.measuredOutput = Q15(measuredSpeedA / 20);
		PID(&motorAPid);
		outputA = motorAPid.controlOutput;
		
		if(outputA < 0)
		{
			if(outputA < -FRACTIONAL_MAX)
			{
				outputA = -FRACTIONAL_MAX;
			}
			
			outputA = -outputA;
			dirA = 0;
		}
		else
		{
			dirA = 1;
		}
		
		outputA = ((double)outputA/(double)FRACTIONAL_MAX) * DUTY_CYCLE_MAX;
	}
	
	if(pidCalcB == 1){
		if(stayCVB == 1)
		{
			motorBPid.controlReference = Q15(CVB / 20);
		}
		else
		{
			int lookAhead = nextSpeedBReadCounter + 1;
			if(lookAhead == NEXT_SPEED_BUFFER_SIZE)
			{
				lookAhead = 0;
			}
			
			if(lookAhead != nextSpeedBWriteCounter)
			{
				nextSpeedBReadCounter = lookAhead;
			}
			
			motorBPid.controlReference = Q15(nextSpeedB[nextSpeedBReadCounter] / 20);
		}
		
		motorBPid.measuredOutput = Q15(measuredSpeedB / 20);
		PID(&motorBPid);
		outputB = motorBPid.controlOutput;
		
		if(outputB < 0)
		{
			if(outputB < -FRACTIONAL_MAX)
			{
				outputB = -FRACTIONAL_MAX;
			}
			
			outputB = -outputB;
			dirB = 0;
		}
		else
		{
			dirB = 1;
		}
		
		outputB = ((double)outputB/(double)FRACTIONAL_MAX) * DUTY_CYCLE_MAX;
	}
	
	if(pidCalcA == 1)
	{
		MTR_A_DUTY_CYCLE = outputA;
		
		if(dirA == 1)
		{
			MTR_A_HIGH_CHANNEL = 1;
			MTR_A_LO_CHANNEL = 0;
		}
		else
		{
			MTR_A_LO_CHANNEL = 1;
			MTR_A_HIGH_CHANNEL = 0;
		}		
	}
	else
	{
		//MTR_A_HIGH_CHANNEL = 0;
		//MTR_A_LO_CHANNEL = 0;
	}
	
	if(pidCalcB == 1)
	{
		MTR_B_DUTY_CYCLE = outputB;
		
		if(dirB == 1)
		{
			MTR_B_HIGH_CHANNEL = 1;
			MTR_B_LO_CHANNEL = 0;
		}
		else
		{
			MTR_B_LO_CHANNEL = 1;
			MTR_B_HIGH_CHANNEL = 0;
		}
	}
	else
	{
		//MTR_B_HIGH_CHANNEL = 0;
		//MTR_B_LO_CHANNEL = 0;
	}
		
	IFS0bits.T1IF = 0; 			// Clear timer 1 interrupt flag	
}

//UART RX ISR
void __attribute__((__interrupt__)) _U1RXInterrupt(void)
{
	//This is test code for initial testing of the recieve
	//It WILL need to be changed to accomodate
	union {
		float f;
		unsigned long ul;
	} floatInterpret;

	c[3] = UART1Data();
	c[2] = UART1Data();
	c[1] = UART1Data();
	c[0] = UART1Data();

	floatInterpret.ul = ((unsigned long)c[0] << 24) | ((unsigned long)c[1] << 16) | ((unsigned long)c[2] << 8) | (unsigned long)c[3];

	float test = floatInterpret.f;

	testNum++;
	/*
	char veloString[20];
	itoa(veloString,velocity,10);
	
	if(c == '0'){
		MTR_A_DUTY_CYCLE = 0;
		UART1Println("0%");
		//printf("%d%c%",0,'%');
		UART1Println(veloString);
	}
	else if(c == '1'){
		MTR_A_DUTY_CYCLE = 204;
		UART1Println("25%");
		//printf("%d%c%",25,'%');
	}
	else if(c == '2'){
		MTR_A_DUTY_CYCLE = 408;
		UART1Println("50%");
		//printf("%d%c%",50,'%');
	}
	else if(c == '3'){
		MTR_A_DUTY_CYCLE = 612;
		UART1Println("75%");
		//printf("%d%c%",75,'%');
	}
	*/

	IFS0bits.U1RXIF = 0;	//clear the recieve flag
}

//Set PID Gains for motor A
void setMotorACoeffs(float kP, float kI, float kD){
	kCoeffsA[0] = Q15(kP);
	kCoeffsA[1] = Q15(kI);
	kCoeffsA[2] = Q15(kD);

	PIDCoeffCalc(&kCoeffsA[0], &motorAPid);
}

//Set PID Gains for motor B
void setMotorBCoeffs(float kP, float kI, float kD){
	kCoeffsB[0] = Q15(kP);
	kCoeffsB[1] = Q15(kI);
	kCoeffsB[2] = Q15(kD);

	PIDCoeffCalc(&kCoeffsB[0], &motorBPid);
}

//Check for available data in RX buffer
unsigned char UART1DataReady(void)
{
	return U1STAbits.URXDA;
}

//Get byte from RX buffer
unsigned char UART1Data(void)
{
	while(U1STAbits.URXDA == 0);
	return U1RXREG;
}

//Add byte to TX buffer, pause if it's full
void UART1AddByte(char c)
{
	//int i;

	//Wait for space in buffer
	while(U1STAbits.UTXBF == 1);

	/*
	//Delay at least 104 usec (1/9600) before sending first char
	for(i = 0; i < 384; i++)
	{
		Nop();
	}
	*/

	U1TXREG = c;
	return;
}

//Custom print line function (because the built in one sucks...)
void UART1Println(char string[])
{
	int l = sizeof(string);
	int i = 0;
	while((i < (l + 1)) && (string[i] != '\0'))
	{
		UART1AddByte(string[i]);
		i++;
	}
	UART1AddByte('\n');
	return;
}

//Initialize UART
void InitUART1(void)
{
	//Assign UART pins
	//RX assigned to RP9, TX assigned to RP8
	U1RXR_I = 9;
	RP8_O = U1TX_O;

	//Set up UART
	//U1BRG = 95;				//95@7.37MHz = 9600 Baud
	U1BRG = 1030;			//1031 @ 79.2MHz = approx. 9600 Baud
	U1MODE = 0;				//Clear mode to disable UART
	U1MODEbits.BRGH = 1;	//Set to high precision baud rate generator
	U1STA = 0;				//Clear the status register
	U1STAbits.URXISEL = 3;	//Set UART1 to interrupt on each byte RX'd
	IEC0bits.U1RXIE = 1;	//Enable UART1 RX interrupt
	U1MODEbits.UARTEN = 1;	//Enable UART RX pin
	U1STAbits.UTXEN = 1;	//Enable UART TX
	IFS0bits.U1RXIF = 0;	//clear the recieve flag
	return;	
}

//Initialize the Quadrature Encoder Interface for motor A
void InitQEI1(void)
{
	//Assign QEI1 pins
	QEI1A_I = QEIA_A;
	QEI1B_I = QEIA_B;
	
	//Set up QEI1
	QEI1CONbits.QEIM = 0; 		// Disable QEI Module
	QEI1CONbits.CNTERR = 0; 	// Clear any count errors
	QEI1CONbits.QEISIDL = 0; 	// Continue operation during sleep
	QEI1CONbits.SWPAB = 0; 		// QEA and QEB not swapped
	QEI1CONbits.PCDOUT = 0; 	// Normal I/O pin operation
	DFLT1CONbits.CEID = 1; 		// Count error interrupts disabled
	DFLT1CONbits.QEOUT = 1; 	// Digital filters output enabled for QEn pins
	DFLT1CONbits.QECK = 4; 		// 1:32 clock divide for digital filter for QEn
	POS1CNT = 0; 				// Reset position counter
	MAX1CNT = 800;				//Set maximum number of counts to the 4x the total encoder counts
	QEI1CONbits.QEIM = 7; 		// X4 mode with position counter reset by MAX1CNT

	//Assign QEI2 pins
	QEI2A_I = QEIB_A;
	QEI2B_I = QEIB_B;
	
	//Set up QEI2
	QEI2CONbits.QEIM = 0; 		// Disable QEI Module
	QEI2CONbits.CNTERR = 0; 	// Clear any count errors
	QEI2CONbits.QEISIDL = 0; 	// Continue operation during sleep
	QEI2CONbits.SWPAB = 0; 		// QEA and QEB not swapped
	QEI2CONbits.PCDOUT = 0; 	// Normal I/O pin operation
	DFLT2CONbits.CEID = 1; 		// Count error interrupts disabled
	DFLT2CONbits.QEOUT = 1; 	// Digital filters output enabled for QEn pins
	DFLT2CONbits.QECK = 4; 		// 1:32 clock divide for digital filter for QEn
	POS2CNT = 0; 				// Reset position counter
	MAX2CNT = 800;				//Set maximum number of counts to the 4x the total encoder counts
	QEI2CONbits.QEIM = 7; 		// X4 mode with position counter reset by MAX1CNT
	
	return;
}

//Initialize the timer for the PID calculation interrupt
void InitTMR1(void)
{
	TMR1 = 0; 					// Reset timer counter
	T1CONbits.TON = 0; 			// Turn off timer 1
	T1CONbits.TSIDL = 0; 		// Continue operation during sleep
	T1CONbits.TGATE = 0; 		// Gated timer accumulation disabled
	T1CONbits.TCS = 0; 			// use Tcy as source clock
	T1CONbits.TCKPS = 2; 		// Tcy / 64 as input clock
	PR1 = 5260; 				// Interrupt period = 0.0085 sec with a 64 prescaler
	IFS0bits.T1IF = 0; 			// Clear timer 1 interrupt flag
	IEC0bits.T1IE = 1; 			// Enable timer 1 interrupts
	T1CONbits.TON = 1; 			// Turn on timer 1
	return;
}

//Setup function
void setup(float diameterA, float diameterB, float spacing, int countsA, int countsB)
{
	wheelDiameterA = diameterA;
	wheelDiameterB = diameterB;
	wheelSpacing = spacing;
	quadCountsA = countsA * 4;
	quadCountsB = countsB * 4;

	POS1CNT = 0; 				// Reset position counter
	MAX1CNT = quadCountsA;

	POS2CNT = 0;
	MAX2CNT = quadCountsB;
	
	float pi = 3.14159;
	
	distancePerCountA = (pi * diameterA)/quadCountsA;
	distancePerCountB = (pi * diameterB)/quadCountsB; 
}

//**************************************************Movement Functions************************************************************//

//**************************************************************
// Purpose: Calculates values for a trapezoidal movement profile and places them in the buffer
// Returns: Completion status
//					1 = incomplete
//					0 = done
//**************************************************************
int TrapezoidalMovement(float maxSpeed, float accel, float decel, float distance, int motors, int dir)
{
	float nextSpeedA = 0;
	float nextSpeedB = 0;
	int distanceMetA = -1;
	int distanceMetB = -1;
	
	if (motors != 1)	// Motor A is active
	{
		pidCalcA = 1;
		nextSpeedA = calcNextTrap(maxSpeed, &currentSpeedA, accel, decel, distance, &distanceTraveledA);	// The next speed that the PID algorithm should aim for
		
		// Place nextSpeed into the buffer, and if it happened successfully then update currentSpeed
		if (stickThisInTheBuffer((nextSpeedA * dir), 0) == 0)
		{			
			if (nextSpeedA == 0)
			{
				distanceTraveledA = distance;
			}
			else
			{
				// Calculate the ammount of positive distance we have traveled regaurdless of dirrection
				distanceTraveledA = distanceTraveledA + nextSpeedA*timeSlice;
			}
			
			currentSpeedA = nextSpeedA;
			
			// If we have met our distance requirement for this motor then indicate so
			if(distanceTraveledA < distance)
			{
				distanceMetA  = 1;	// We have not met our goal
			}
			else
			{
				distanceMetA = 0;	// We have met our goal
			}
		}
		else
		{
			distanceMetA = 1;	// We have not met our goal
		}
	}
	
	if (motors != 0)	// Motor B is active
	{
		pidCalcB = 1;
		nextSpeedB = calcNextTrap(maxSpeed, &currentSpeedB, accel, decel, distance, &distanceTraveledB);	// The next speed that the PID algorithm should aim for
		
		// Place nextSpeed into the buffer, and if it happened successfully then update currentSpeed
		if (stickThisInTheBuffer((nextSpeedB * dir), 1) == 0)
		{
			if (nextSpeedB == 0)
			{
				distanceTraveledB = distance;
			}
			else
			{
				// Calculate the ammount of positive distance we have traveled regaurdless of dirrection
				distanceTraveledB= distanceTraveledB + nextSpeedB*timeSlice;
			}
			
			currentSpeedB = nextSpeedB;
			
			// If we have met our distance requirement for this motor then indicate so
			if(distanceTraveledB < distance)
			{
				distanceMetB = 1;	// We have not met our goal
			}
			else
			{
				distanceMetB = 0;	// We have met our goal
			}
		}
		else
		{
			distanceMetB = 1; // We have not met our goal
		}
	}

	// Determine if the rapezoidal movement function is done
	if (motors == 0)	// motor A only
	{
		if (distanceMetA == 0)
		{
			//pidCalcA = 0;
			return 0;	// Status = DONE
		}
		else
		{
			return 1;	// Status == INCOMPLETE
		}
	}
	else if (motors == 1)	// Motor B only
	{
		if (distanceMetB == 0)
		{
			//pidCalcB = 0;
			return 0;	// Status = DONE
		}
		else
		{
			return 1;	// Status = INCOMPLETE
		}
	}
	else	// Motors A and B
	{
		if ((distanceMetA == 0) && (distanceMetB == 0))
		{
			//pidCalcA = 0;
			//pidCalcB = 0;
			return 0;	// Status = DONE
		}
		else
		{
			return 1;	// Status = INCOMPLETE
		}
	}	
}

//**************************************************************
// Purpose: Calculate the next volocity point on the trapezoid
// Returns: The next speed to be placed in the buffer
//**************************************************************
float calcNextTrap(float maxSpeed, float* currentSpeed, float accel, float decel, float distance, float* distanceTraveled)
{
	// timeSlice is the time interval at which we are measuing points on the trapezoid
	float nextSpeed; // The next speed that the PID algorithm should aim for

	// Check to see if we are hitting the maxSpeed
	if(*currentSpeed >= maxSpeed)
	{
		nextSpeed = maxSpeed;
	}
	else // If not, assume we are accelerating and calculate nextSpeed
	{
		nextSpeed = *currentSpeed + (accel * timeSlice);
	}
	
	// Determine how many time slices it will take to decelerate
	int numberOfDecelSlices = 0;		// The number of time slices needed to decelerate to 0
	float speedTemp = nextSpeed;
	
	while (speedTemp > 0)
	{
		speedTemp = speedTemp - (decel * timeSlice);
		numberOfDecelSlices++;
	}

	// If we are going to overshoot our target then begin to decelerate
	if ((*distanceTraveled + (nextSpeed *(numberOfDecelSlices * timeSlice))) >= distance)
	{
		nextSpeed = *currentSpeed - (decel * timeSlice);
	}
	
	*distanceTraveled = *distanceTraveled + nextSpeed * timeSlice;
	*currentSpeed = nextSpeed;
	
	if (nextSpeed < 0)
	{
		nextSpeed = 0;
	}

	return nextSpeed;
}

//********************************************************
// TURN ON AXIS FUNCTION
// Purpose: fill the nexSpeed buffer with values for turn on axis
// Returns: Completion status
//					1 = incomplete
//					0 = done
// Notes: For this funciton, motor A is left and motor B is right
//			  for dir, 0 = clockwise and 1 = counterclockwise
//********************************************************
int TurnOnAxis(float velocity, int dir, int degrees)
{
	float velocityA;
	float velocityB;

	// Disable pid calculations
	pidCalcA = 1;
	pidCalcB = 1;
	
	// Determine which direction to rotate
	if (dir == 0)	// Clockwise
	{
		velocityA = velocity;
		velocityB = -1 * velocity;
	}
	else			// Counterclockwise
	{
		velocityA = -1 * velocity;
		velocityB = velocity;
	}
	
	if ((userProvidedTime <= 0) && (degrees == 0))	// Spin for an infinite amount of time
	{
		stayCVA = 1;
		stayCVB = 1;
		
		CVA = velocityA;
		CVB = velocityB;

		// Enable pid calculations
		pidCalcA = 0;
		pidCalcB = 0;
		
		return 0;	// Status = DONE;
	}
	else	
	{
		if((userProvidedTime > 0) && (degrees == 0)) // Spin for a defined ammount of time
		{
			neededDataPoints = userProvidedTime/timeSlice;
		}
		else	// Spin for a defined angle
		{
			float angleInRadians = degrees * (3.14/180);
			float arcLength = angleInRadians * (wheelSpacing / 2);
			neededDataPoints = (arcLength / velocity) / timeSlice;
		}
	
	
		if((stickThisInTheBuffer(velocityA,0) == 0) && (stickThisInTheBuffer(velocityB,1) == 0))	//  If the data was successfully put into the buffer A, move on the buffer B
		{
			numberOfGeneratedPoints++;
			
			if(numberOfGeneratedPoints < neededDataPoints)
			{
				return 1;	// Status = INCOMPLETE. Run again to generate another point
			}
			else
			{
				pidCalcA = 0;
				pidCalcB = 0;
				return 0;	// Status = DONE
			}
		}
		else
		{
			return 1;	// Status = INCOMPLETE. Run again to generate another point
		}
	}
}

//********************************************************
// CONSTANT VELOCITY FUNCTION
// Purpose: fill the nexSpeed buffer with values for constant velocity values
// Returns: Completion status
//					1 = incomplete
//					0 = done
//********************************************************
int ConstantVelocity(float velocity, int dir, int motor)
{
	if (motor != 1)
	{
		pidCalcA = 1;
	}
	if (motor != 0)
	{
		pidCalcB = 1;
	}
	
	if (userProvidedTime <= 0)	// Constant velocity for an infinite amount of time
	{
		if (motor != 1)		// Motor A
		{
			stayCVA = 1;
			CVA = dir * velocity;
		}
		if (motor != 0)		// Motor B
		{
			stayCVB = 1;
			CVB = dir * velocity;
		}

		pidCalcA = 0;
		pidCalcB = 0;
		return 0;	// Status = DONE;
	}
	else	// Constant velocity for a defined ammount of time
	{
		if(stickThisInTheBuffer((velocity*dir),motor) == 0)	//  If the data was successfully put into the buffer, increment the counter
		{
			numberOfGeneratedPoints++;
			neededDataPoints = userProvidedTime / timeSlice;
			
			if(numberOfGeneratedPoints < neededDataPoints)
			{
				return 1;	// Status = INCOMPLETE. Run again to generate another point
			}
			else
			{
				pidCalcA = 0;
				pidCalcB = 0;
				return 0;	// Status = DONE
			}
		}
		else
		{
			return 1;	// Status = INCOMPLETE. Run again to generate another point
		}
	}
}

//************************************************
// Purpose: Places a floating point value in the nextSpeed buffer fo A, B or both
// Returns: success or failure
//************************************************
int stickThisInTheBuffer(float bufferValue, int motors)	// don't forget the function prototype
{
	if (motors == 0)	// Motor A
	{
		if ((nextSpeedAReadCounter == nextSpeedAWriteCounter) || ((nextSpeedAReadCounter == -1) && (nextSpeedAWriteCounter == (NEXT_SPEED_BUFFER_SIZE-1))))	// We must wait for the read counter if its being slow
		{
			return 1;	// Failure to place the value in the buffer
		}
		else
		{
			nextSpeedA[nextSpeedAWriteCounter] = bufferValue;	// Place the value in the buffer
			nextSpeedAWriteCounter++;	// increment the write counter
			
			if (nextSpeedAWriteCounter >= NEXT_SPEED_BUFFER_SIZE)	// Have we exceeded our biffer size?
			{
				nextSpeedAWriteCounter = 0;		// restart the buffer count
			}
			
			return 0;	// Successfully placed the value in the buffer
		}
	}
	if (motors == 1)	// Motor B
	{
		if ((nextSpeedBReadCounter == nextSpeedBWriteCounter) || ((nextSpeedBReadCounter == -1) && (nextSpeedBWriteCounter == (NEXT_SPEED_BUFFER_SIZE-1))))	// We must wait for the read counter if its being slow
		{
			return 1;	// Failure to place the value in the buffer
		}
		else
		{
			nextSpeedB[nextSpeedBWriteCounter] = bufferValue;	// Place the value in the buffer
			nextSpeedBWriteCounter++;	// increment the write counter
			
			if (nextSpeedBWriteCounter >= NEXT_SPEED_BUFFER_SIZE)	// Have we exceeded our biffer size?
			{
				nextSpeedBWriteCounter = 0;		// restart the buffer count
			}
			
			return 0;	// successfully replaced the value in the buffer	
		}
	}
	else	// both motors
	{
		// For both motors its all or nothing...either both values get placed into the buffer or neither do
		if (((nextSpeedAReadCounter != nextSpeedAWriteCounter) && (nextSpeedBReadCounter != nextSpeedBWriteCounter)) && (((nextSpeedAWriteCounter != (NEXT_SPEED_BUFFER_SIZE-1)) || (nextSpeedAReadCounter != -1)) && ((nextSpeedBWriteCounter != (NEXT_SPEED_BUFFER_SIZE-1)) || (nextSpeedBReadCounter != -1))))// We must wait for the read counters if they're being slow
		{
			nextSpeedA[nextSpeedAWriteCounter] = bufferValue;
			nextSpeedB[nextSpeedBWriteCounter] = bufferValue;
			nextSpeedAWriteCounter++;
			nextSpeedBWriteCounter++;
			
			return 0;	// Successfully placed both values in the buffer
		}
		else
		{
			return 1;	// Failure to place both values in the buffer
		}
	}
}

//********************************************************
// STOP FUNCTION
// Purpose: Stop the motor(s)
//********************************************************
void Stop(int coast, int motors)
{
	if (coast == 0)	// Coasting
	{
		if (motors != 1)
		{
			pidCalcA = 0;
		}
		if (motors != 0)
		{
			pidCalcB = 0;
		}
	}
	else	// braking
	{
		int counter = 0;
		
		if (motors != 1)	// Motor A
		{
			nextSpeedAReadCounter = -1;
			nextSpeedAWriteCounter = 0;
			
			while (counter < NEXT_SPEED_BUFFER_SIZE)
			{
				if (stickThisInTheBuffer(0, 0) == 0)
				{
					counter++;
				}
			}
			
			counter = 0;
		}
		if (motors != 0)	// Motor B
		{
			nextSpeedBReadCounter = -1;
			nextSpeedBWriteCounter = 0;
			
			while (counter < NEXT_SPEED_BUFFER_SIZE)
			{
				if (stickThisInTheBuffer(0, 1) == 0)
				{
					counter++;
				}
			}
		}
	}
}
