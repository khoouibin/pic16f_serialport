#include <xc.h>
#include "init_all.h"
#include "uart_modbus.h"

void InitAll(void)
{
    //	System Clock.
    OSCCON = 0x70;
    NOP();
    NOP();
    NOP();

    // Initial_OSC();			// this mcu only setting by configuration bits.
    Initial_GPIO();
    Initial_Interrupt();
    Initial_ADC();
    Initial_Tmr0();
    UART1_modbus_init();
}

void Initial_GPIO(void)
{
    //	PORT A.
    TRISA = 0x00;
    PORTA = 0x00;

    //	PORT B.
    TRISBbits.TRISB0 = 0; // input no use. [1]
    TRISBbits.TRISB1 = 0; // input no use. [2]
    TRISBbits.TRISB2 = 0; // input no use. [3]
    TRISBbits.TRISB3 = 0; // input no use. [4]    // for PWM output debugger.
    TRISBbits.TRISB4 = 0; // LED OUTPUT. LED3
    TRISBbits.TRISB5 = 0; // cancelled, 5v output for internal input.
    TRISBbits.TRISB6 = 1; // U3.12 ICS-CLK, and coupling with AC digital input (spared)
    TRISBbits.TRISB7 = 1; // U3.13 ICS-DAT, and coupling with AC digital input
    PORTB = 0x00;

    //	PORT C.
    TRISCbits.TRISC0 = 0; //
    TRISCbits.TRISC1 = 0; //
    TRISCbits.TRISC2 = 0; //
    TRISCbits.TRISC3 = 0; //
    TRISCbits.TRISC4 = 0; //
    TRISCbits.TRISC5 = 0; //
    TRISCbits.TRISC6 = 0; // Tx
    TRISCbits.TRISC7 = 1; // Rx
    PORTB = 0x00;
}

void Initial_Interrupt(void)
{
    INTCON = 0x00;
    // Disable
    PIE1 = 0x00;
    // Flag
    PIR1 = 0x00;
    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1;
}

void Initial_ADC(void)
{
    // ANC Reg
    /*
    ADCON0bits.CHS=4;
    ADCON1bits.VCFG=0;
    ADCON1bits.VNCFG=0;
    ADCON1bits.CHSN=0;
    ADCON2bits.ADFM=1;
    ADCON2bits.ACQT=0b100;	//8Tad
    ADCON2bits.ADCS=0b101;

    PIE1bits.ADIE=1;
//	IPR1bits.ADIP=0;
    ADCON0bits.ADON=1;	//adc turn on.
    */

    ADCON1bits.ADCS = 0b001; // fosc/8.
    // ADCON0bits.CHS  = 0b000;    // an0
    ADCON0bits.CHS = 0;

    // ADCON1bits.PCFG = 0b100; // ad port configuration control bits.
    // // ADCON1bits.PCFG = 0b101;    // ng setting.
    // // ADCON1bits.PCFG = 0b000;    // ng setting.
    // //  because this MCU is a tiny one,
    // //  so, there is no ansel SFR something likes that,
    // //  'ADCON1' setting RA2-digital, RA3-analog.
    // //  so that RA3 pin should not to be use in this OVP board.

    ADCON0bits.ADON = 0;
}

void Initial_Tmr0(void)
{
    TMR0 = 0;
    OPTION_REGbits.TMR0CS = 0; // Tmr0 clock source:Fosc/4.
    OPTION_REGbits.PSA = 0;    // Tmr0 assigned by Prescaler.
    OPTION_REGbits.PS2 = 1;    // Tmr0 1:2	--> PS2:PS0=000.	Tmr0 1:4	-->	PS2:PS0=001.
    OPTION_REGbits.PS1 = 0;    // Tmr0 1:8	--> PS2:PS0=010.	Tmr0 1:16	--> PS2:PS0=011.
    OPTION_REGbits.PS0 = 1;    // Tmr0 1:32	--> PS2:PS0=100.	Tmr0 1:64	--> PS2:PS0=101.
    TMR0 = TMR0_cnt;
    INTCONbits.TMR0IE = 1;
}

