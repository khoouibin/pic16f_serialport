#ifndef init_all_H
#define init_all_H

// tmr0 - 1ms.
#define TMR0_cnt 131
//	IO define.
// #define TEST3_O		LATDbits.LATD3
// #define TEST2_O		LATEbits.LATE1
// #define TEST1_O       PORTBbits.RB2       //temperate.
#define TEST1_O PORTBbits.RB5

void InitAll(void);
void Initial_GPIO(void);
void Initial_Tmr0(void);

// void 	Initial_PWM(void);
void Initial_Interrupt(void);
void Initial_ADC(void);
// void    Startup_ioCondition(void);

#endif