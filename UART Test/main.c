//Test code, first run
#include "hardwareprofile.h"
#include <stdio.h>
#include <stdlib.h>
#include <dsp.h>

//Set up processor registers
_FOSCSEL(FNOSC_FRC)					// Use built in oscillator crystal
_FOSC(POSCMD_NONE & OSCIOFNC_ON)	// "Use Oscillator pins as I/O pins" is on
_FWDT(FWDTEN_OFF)					// Turn watchdog timer off
_FICD(ICS_PGD1 & JTAGEN_OFF)		// Set to programming interface 1 and no JTAG

//Function definitions
unsigned char UART1DataReady(void);	//Check for available data in RX buffer
unsigned char UART1Data(void);				//Get byte from RX buffer
void UART1AddByte(char c);					//Add byte to TX buffer, pause if it's full
void InitUART1(void);								//Initialize UART
void InitQEI1(void);									//Initialize the Quadrature Encoder Interface for motor A
void InitTMR1(void);								//Initialize the timer for the PID calculation interrupt
void UART1Println(char string[]);				//Custom print line function (because the built in one sucks...)
void PositionCalculation(void);					// !!Legacy!!
															// This is likely not the answer we need, but i'll leave it
															//just in case
void setMotorACoeffs(float kP, float kI, float kD);	//Set PID Gains for motor A
void setMotorBCoeffs(float kP, float kI, float kD);	//Set PID Gains for motor B
int calcNextTrap(int timeSlice, float maxSpeed, float accel, float decel, int distance, int* distanceTraveled, int* currentSpeed);	// get the next value in the trapezoidal function


//Begin motor control variables
volatile int velocity;				//velocity measurement
int Pos[2] = {0,0};					//last and present position
tPID motorAPid;						//PID data structure for Motor A
tPID motorBPid;						//PID data structure for Motor B

//PID variable definitions
fractional mAabcCoeffs[3] __attribute__ ((space(xmemory)));		//((section (".xbss, bss, xmemory")));
fractional mBabcCoeffs[3] __attribute__ ((space(xmemory)));		//((section (".xbss, bss, xmemory")));
fractional mAcontHist[3] __attribute__ 	((space(ymemory)));		//((section (".ybss, bss, ymemory")));
fractional mBcontHist[3] __attribute__ 	((space(ymemory)));		//((section (".ybss, bss, ymemory")));
fractional kCoeffsA[] = {0.7,0.2,0};
fractional kCoeffsB[] = {0.7,0.2,0};

int main(void)
{
	AD1PCFGL = 0xFFFF;  //make all pins digital

	//Initialize PID
	motorAPid.abcCoefficients = &mAabcCoeffs[0];
	motorBPid.abcCoefficients = &mBabcCoeffs[0];

	motorAPid.controlHistory = &mAcontHist[0];
	motorBPid.controlHistory = &mBcontHist[0];

	PIDInit(&motorAPid);
	PIDInit(&motorBPid);

	PIDCoeffCalc(&kCoeffsA[0], &motorAPid);
	PIDCoeffCalc(&kCoeffsB[0], &motorBPid);
	
	/*
		We may use this commented out section later to increase the speed, leave it here!!
	*/
	//setup internal clock for 80MHz/40MIPS
	//7.37/2=3.685*43=158.455/2=79.2275
	//CLKDIVbits.PLLPRE=0; 	// PLLPRE (N2) 0=/2 
	//PLLFBD=41; 		//pll multiplier (M) = +2
	//CLKDIVbits.PLLPOST=0;	// PLLPOST (N1) 0=/2
	 	
	//while(!OSCCONbits.LOCK);	//wait for PLL to stabilize

	//Set up UART
	InitUART1();
	
	//Set up Quadrature Encoder Interface A
	InitQEI1();

	//Set up timer 1 (Interrupts for velocity calculations)
	InitTMR1();

	//Set up motor A control pins as inputs
	MOTOR_A_1TRIS = 1;
	MOTOR_A_2TRIS = 1;

	//Set up motor B control pins as inputs
	MOTOR_B_1TRIS = 1;
	MOTOR_B_2TRIS = 1;

	//Initialize PWM
	MTR_A_PWMTRIS = 0;		//sets pwm pins to outputs
	MTR_B_PWMTRIS = 0;
	MTR_A_PWM_O = 0;		//clears any data from pwm pins
	MTR_B_PWM_O = 0;
	
	PWM_POSTSCALE = 0;		//No pwm timer scaling
	PWM_PRESCALE = 0;
	PWM_TIMEBASEMODE = 0;	//set to free running
	
	PWM_TMRSTART = 0;		//start counter at 0
	
	PWM_TIMEBASE_PER = 408;	//set the timebase period for the pwm module
	
	PWM_CONFbits.PMOD3 = 1; 	// PWM in independent mode
	PWM_CONFbits.PMOD2 = 1; 	// PWM in independent mode
	PWM_CONFbits.PMOD1 = 1; 	// PWM in independent mode
	PWM_CONFbits.PEN3H = 0; 	// PWM High pin is disabled
	PWM_CONFbits.PEN2H = 1; 	// PWM High pin is disabled
	PWM_CONFbits.PEN1H = 1; 	// PWM High pin is enabled
	PWM_CONFbits.PEN3L = 0; 	// PWM Low pin disabled 
	PWM_CONFbits.PEN2L = 0; 	// PWM Low pin disabled 
	PWM_CONFbits.PEN1L = 0; 	// PWM Low pin disabled

	MTR_A_DUTY_CYCLE = 408;		//Set duty cycle to 50%
	MTR_B_DUTY_CYCLE = 408;
	
	PWM_TMR_ENABLE = 1;			//Enable the PWM Timerbase
	
	//Set up PID gains
	setMotorACoeffs(0.7, 0.2, 0.02);
	setMotorBCoeffs(0.7, 0.2, 0.02);
	
	//Set the control reference and measured output
	//(This is just for testing purposes)
	motorAPid.controlReference = Q15(0.74);
	motorBPid.controlReference = Q15(0.74);
	motorAPid.measuredOutput = Q15(0.453);
	motorBPid.measuredOutput = Q15(0.453);

	while(1){
		
		//Perform PID functions (For testing)
		PID(&motorAPid);
		PID(&motorBPid);
	
	}	
}

//Speed Caclulation ISR
void __attribute__((__interrupt__)) _T1Interrupt(void)
{
	//First copy the position count to get a snapshot
	int POSCNTcopy = (int)POSCNT;

	//If the count happens to be less than zero, make it positive
	//(I need to check this, because i'm not sure it makes sense...)
	if (POSCNTcopy < 0)
		POSCNTcopy = -POSCNTcopy;

	//Cycle the old position into the lower array and place the copy
	//in the new spot
	Pos[1] = Pos[0];
	Pos[0] = POSCNTcopy;

	//Calculate velocity
	//The decimal value 0.0085 needs to be changed based on
	//the interrupt interval
	velocity = (Pos[0]-Pos[1])/0.0085;

	IFS0bits.T1IF = 0; 			// Clear timer 1 interrupt flag	
}

//UART RX ISR
void __attribute__((__interrupt__)) _U1RXInterrupt(void)
{
	//This is test code for initial teseting of the recieve
	//It WILL need to be changed to accomodate
	char c;
	c = UART1Data();
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

//!!Legacy!!
//This is likely not the answer we need, but i'll leave it
//just in case
void PositionCalculation(void)
{
	int POSCNTcopy = (int)POSCNT;
	if (POSCNTcopy < 0)
		POSCNTcopy = -POSCNTcopy;
	Pos[1] = Pos[0];
	Pos[0] = POSCNTcopy;
	
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
	U1BRG = 95;				//95@7.37MHz = 9600 Baud
	U1MODE = 0;				//Clear mode to disable UART
	U1MODEbits.BRGH = 1;	//Set to high precision baud rate generator
	U1STA = 0;				//Clear the status register
	U1STAbits.URXISEL = 0;	//Set UART1 to interrupt on each byte RX'd
	U1STAbits.URXISEL = 0;
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
	QEI1A_I = 6;
	QEI1B_I = 5;
	
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
	PR1 = 488; 					// Interrupt period = 0.0085 sec with a 64 prescaler
	IFS0bits.T1IF = 0; 			// Clear timer 1 interrupt flag
	IEC0bits.T1IE = 1; 			// Enable timer 1 interrupts
	T1CONbits.TON = 1; 			// Turn on timer 1
	return;
}

//
// Trapezoidal Movement Functions
//
// Calculate velocity given distance
int calcNextTrap(int timeSlice, float maxSpeed, float accel, float decel, int distance, int* distanceTraveled, int* currentSpeed)
{
	// timeSlice is the time interval at which we are measuing points on the trapezoid
	int nextSpeed	// The next speed that the PID algorithm should aim for
	
	// Check to see if we are hitting the maxSpeed
	if(currentSpeed >= maxSpeed)
	{
		nextSpeed = maxSpeed;
	}
	else	// If not, assume we are accelerating and calculate nextSpeed
	{
		nextSpeed = currentSpeed + (accel * timeSlice);
	}
	
	// If we are going to overshoot our target b
	if ((distanceTraveled + (nextSpeed * timeSlice)) > distance)
	{
		nextSpeed = currentSpeed - (decel * timeSlice);
	}
	
	distanceTraveled = distanceTraveled + nextSpeed * timeSlice;
	curSpeed = nextSpeed;
	
	return nextSpeed;
}

/* Greg's obsolete code
float velocity(float maxSpeed, float accel, float decel, int time)
{
	float accelVelocity = accel * time;
	float decelVelocity = decel * time;
	
	if (accelVelocity < maxSpeed)
		return accelVelocity;
	else if (decelVelocity < maxSpeed
		return decelVelocity;
	else
		return maxSpeed;
}

	//float accelTime = maxSpeed / accel;
	//float decelTime = maxSpeed / decel;
	
	float accelDistance = .5 * accel * time * time; // How far I have gone so far
	float projectedDistance = accelDistance + ((accel * distance) * (accel * distance) / 2 * decel);	// How far I would go if I slow down now

	// slow down if we can't reach our amx speed
	if (projectedDistance > distance) 
	{
		
	}
*/