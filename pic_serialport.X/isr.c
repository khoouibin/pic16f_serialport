#include <xc.h>

#include "stdint.h"  // Integer definition
#include "stdbool.h" // Boolean (true/false) definition
#include "init_all.h"
#include "uart_modbus.h"

#define TMR0_8ms 0x07
char tmr0_ms = 0;
char tmr1_delay = 1;

void __interrupt() isr_routine(void)
{
    if (INTCONbits.TMR0IF == 1)
    {
        INTCONbits.TMR0IF = 0;
        TMR0 = TMR0_cnt;

        tmr0_ms += 1;
        tmr0_ms &= TMR0_8ms;
        if (tmr0_ms == 0)
        {
            tmr1_delay += 1;
            if (tmr1_delay > 7)
                tmr1_delay = 0;
        }
    }

    if (PIR1bits.TMR1IF == 1)
    {
        PIR1bits.TMR1IF = 0;
        Tmr1_Off();
        Tmr1_Process();
    }

    if (PIR1bits.RCIF == 1)
    {
        PIR1bits.RCIF = 0;
        UART_Rx(RCREG);
        Test_1 = ~Test_1;
    }

    if (PIE1bits.TXIE == 1 && PIR1bits.TXIF == 1)
    {
        txBuf_send();
        Test_2 = ~Test_2;
    }
}
