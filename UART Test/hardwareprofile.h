#include <p33FJ128MC802.h>

//Shortcut names for the motor input pins
#define	MOTOR_A_1TRIS	(TRISAbits.TRISA0)
#define	MOTOR_A_2TRIS	(TRISAbits.TRISA1)
#define	MOTOR_A_1I	(PORTAbits.RA0)
#define	MOTOR_A_2I	(PORTAbits.RA1)

#define	MOTOR_B_1TRIS	(TRISAbits.TRISA2)
#define	MOTOR_B_2TRIS	(TRISAbits.TRISA3)
#define	MOTOR_B_1I	(PORTAbits.RA2)
#define	MOTOR_B_2I	(PORTAbits.RA3)

//Shortcut names for PWM
#define	MTR_A_PWMTRIS	(TRISBbits.TRISB12)
#define	MTR_A_PWM_O	(PORTBbits.RB12)
#define	PWM_POSTSCALE	(PTCONbits.PTOPS)
#define	PWM_PRESCALE	(PTCONbits.PTCKPS)
#define	PWM_TIMEBASEMODE	(PTCONbits.PTMOD)
#define	PWM_TMRSTART	(PTMR)
#define	PWM_TIMEBASE_PER	(PTPER)
#define	MTR_A_DUTY_CYCLE	(PDC1)
#define	PWM_CONFbits	(PWMCON1bits)
#define	PWM_TMR_ENABLE	(PTCONbits.PTEN)

#define	MTR_B_PWMTRIS	(TRISBbits.TRISB14)
#define	MTR_B_PWM_O	(PORTBbits.RB14)
#define	MTR_B_DUTY_CYCLE	(PDC2)

//PPS Input registers
#define U1RXR_I RPINR18bits.U1RXR
#define U1CTSR_I RPINR18bits.U1CTSR
#define QEI1A_I	RPINR14bits.QEA1R
#define QEI1B_I	RPINR14bits.QEB1R

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
