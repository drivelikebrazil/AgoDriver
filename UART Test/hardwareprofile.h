#include <p33FJ128MC802.h>
/* Used for testing the dsPIC on the breadboard
//Shortcut names for the motor input pins
#define	MOTOR_A_1TRIS	(TRISAbits.TRISA0)
#define	MOTOR_A_2TRIS	(TRISAbits.TRISA1)
#define	MOTOR_A_1I	(PORTAbits.RA0)
#define	MOTOR_A_2I	(PORTAbits.RA1)

#define	MOTOR_B_1TRIS	(TRISAbits.TRISA2)
#define	MOTOR_B_2TRIS	(TRISAbits.TRISA3)
#define	MOTOR_B_1I	(PORTAbits.RA2)
#define	MOTOR_B_2I	(PORTAbits.RA3)
*/

//Shortcut names for PWM
#define TIMEBASE_PERIOD 4400
#define DUTY_CYCLE_MAX 8800
#define FRACTIONAL_MAX 32767

#define	MTR_AH_PWMTRIS	(TRISBbits.TRISB12)
#define	MTR_AH_PWM_O	(PORTBbits.RB12)
#define MTR_AL_PWMTRIS	(TRISBbits.TRISB13)
#define MTR_AL_PWM_O	(PORTBbits.RB13)
#define	MTR_A_DUTY_CYCLE	(PDC1)
#define MTR_A_HIGH_CHANNEL (PWM_CONFbits.PEN1L)
#define MTR_A_LO_CHANNEL (PWM_CONFbits.PEN1H)

#define	MTR_BH_PWMTRIS	(TRISBbits.TRISB14)
#define	MTR_BH_PWM_O	(PORTBbits.RB14)
#define MTR_BL_PWMTRIS	(TRISBbits.TRISB15)
#define MTR_BL_PWM_O	(PORTBbits.RB15)
#define	MTR_B_DUTY_CYCLE	(PDC2)
#define MTR_B_HIGH_CHANNEL (PWM_CONFbits.PEN2H)
#define MTR_B_LO_CHANNEL (PWM_CONFbits.PEN2L)

#define	PWM_POSTSCALE	(PTCONbits.PTOPS)
#define	PWM_PRESCALE	(PTCONbits.PTCKPS)
#define	PWM_TIMEBASEMODE	(PTCONbits.PTMOD)
#define	PWM_TMRSTART	(PTMR)
#define	PWM_TIMEBASE_PER	(PTPER)
#define	PWM_CONFbits	(PWMCON1bits)
#define	PWM_TMR_ENABLE	(PTCONbits.PTEN)

//Status flag shortcut names
#define MTR_A_SF_TRIS	TRISBbits.TRISB2
#define MTR_B_SF_TRIS	TRISBbits.TRISB3
#define MTR_A_SF_I	PORTBbits.RB2
#define MTR_B_SF_I	PORTBbits.RB3

//PPS Inputs
#define QEIA_A	5
#define QEIA_B	6
#define QEIB_A	10
#define QEIB_B	11

//PPS Input registers
#define U1RXR_I RPINR18bits.U1RXR
#define U1CTSR_I RPINR18bits.U1CTSR
#define QEI1A_I	RPINR14bits.QEA1R
#define QEI1B_I	RPINR14bits.QEB1R
#define QEI2A_I	RPINR16bits.QEA2R
#define QEI2B_I RPINR16bits.QEB2R

//PPS Outputs
#define U1TX_O	3
#define U1RTS_O	4

//PPS pin output registers
#define RP0_O	RPOR0bits.RP0R
#define RP1_O	RPOR0bits.RP1R
#define RP2_O	RPOR1bits.RP2R
#define RP3_O	RPOR1bits.RP3R
#define RP4_O	RPOR2bits.RP4R
#define RP5_O	RPOR2bits.RP5R
#define RP6_O	RPOR3bits.RP6R
#define RP7_O	RPOR3bits.RP7R
#define RP8_O	RPOR4bits.RP8R
#define RP9_O	RPOR4bits.RP9R
#define RP10_O	RPOR5bits.RP10R
#define RP11_O	RPOR5bits.RP11R
#define RP12_O	RPOR6bits.RP12R
#define RP13_O	RPOR6bits.RP13R
#define RP14_O	RPOR7bits.RP14R
#define RP15_O	RPOR7bits.RP15R
