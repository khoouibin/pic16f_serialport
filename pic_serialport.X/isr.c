#include <xc.h>

#include "stdint.h"  // Integer definition
#include "stdbool.h" // Boolean (true/false) definition
#include "init_all.h"

#define Test_1 LATAbits.LATA0
#define Test_2 LATAbits.LATA1


void __interrupt() isr_routine(void)
{
    //--Tmr0 check.
    if (INTCONbits.TMR0IF == 1)
    {
        INTCONbits.TMR0IF = 0;
        TMR0 = TMR0_cnt;
        Test_1 = ~Test_1;

    }

    if (PIR1bits.TMR2IF == 1)
    {
        PIR1bits.TMR2IF = 0;



    }
}
