// ---------------------------------------------------------------------------
//  Filename: UART_modbus.c
//  Created by: Khoo UiBin
//  Date:  July05, 2021.
//  Orisol Taiwan(c) 
// ---------------------------------------------------------------------------
#include <xc.h>
#include "uart_modbus.h"

uart_modbus_status_t UART1_modbus;

void UART1_modbus_init(void)
{
    UART1_module_init();
    Tmr2_module_init();
    set_modbus_mode( loopback_mode );
    //set_modbus_mode( normal_mode );
    set_modbus_slave_addr(SLAVE_ADDR_DEF);
    set_modbus_loopback_filter( no_filter );
    set_modbus_resp_delay_ms( 5 );
}


void Tmr2_module_init(void)
{
    // PR2 = 93
    // tmr2 prescaler = 64
    // Fosc/4 = 32 Mhz / 4 = 8 Mhz.
    // 1 instruction cycle = 0.125 usec.
    // 1 cnt = 0.125 usec * 64 = 8 usec.
    TMR2 = 0;
    T2CON = 0;
    T2CONbits.T2CKPS = 0b11;
    PR2 = modbus_SilentInt;
    PIE1bits.TMR2IE = 1;
    T2CONbits.TMR2ON = 0;
}

void UART1_module_init(void)
{
    TXSTAbits.CSRC = 1; //clock source select: master mode
    TXSTAbits.TX9 = 0; // 8-bit transmission
    TXSTAbits.TXEN = 1;
    TXSTAbits.SYNC = 0;
    TXSTAbits.SENDB = 0;
    TXSTAbits.BRGH = 0;
    TXSTAbits.TX9D = 0;

    RCSTAbits.SPEN = 1;
    RCSTAbits.RX9 = 0;
    RCSTAbits.CREN = 1;

    BAUDCONbits.SCKP = 0;
    BAUDCONbits.BRG16 = 0;
    BAUDCONbits.WUE = 0;
    BAUDCONbits.ABDEN = 0;

    SPBRGL = 51;
    SPBRGH = 0;

    PIE1bits.RCIE = 1;
    PIR1bits.RCIF = 1;
}

int set_modbus_mode( modbus_mode_t mode )
{
//    char mode_max = (char)maxnum_mode;
//    char mode_input=(char)mode;

    if( mode < maxnum_mode )
    {
        UART1_modbus.mode   = mode;
        UART1_modbus.status = status_rx;
        UART1_modbus.wait_ctrl.wait_type  = silent_wait;
        UART1_modbus.tx_ctrl.txSize       = 0;
        UART1_modbus.rx_ctrl.rxSize       = 0;
        UART1_modbus.tx_ctrl.send_num     = 0;

        UART1_modbus.rx_ctrl.ptr_rxBuf    = &UART1_modbus.rx_ctrl.rxBuf[0];
        UART1_modbus.tx_ctrl.ptr_txBuf    = &UART1_modbus.tx_ctrl.txBuf[0];

        if( UART1_modbus.mode == periodic_mode )
        {
            if( UART1_modbus.wait_ctrl.delay_ms < REPLY_PERIOD_MIN )
                UART1_modbus.wait_ctrl.delay_ms = REPLY_PERIOD_MIN;
        }
        return 0;
    }
    else
        return -1; 
}

modbus_mode_t get_modbus_mode(void)
{
    return UART1_modbus.mode;
}

int set_modbus_slave_addr( char addr )
{
//  broadcast addr: 00h.   ( not support )
//  slave addr:     01~7fh.
    unsigned char addr_temp = (unsigned char)addr;

    if( addr_temp == 0x00 )
        return -1;
    else if( addr_temp > 0x7f )
        return -2;
    else
        UART1_modbus.slave_addr = addr_temp;

    return 0;
}

char get_modbus_resp_delay_ms(void)
{
    return UART1_modbus.wait_ctrl.delay_ms;
}

void set_modbus_resp_delay_ms( char delay_ms )
{
    UART1_modbus.wait_ctrl.delay_ms   = delay_ms;
}

char get_modbus_slave_addr(void)
{
    return UART1_modbus.slave_addr;
}

int set_modbus_loopback_filter( modbus_loopback_t filter_typ )
{
    if( filter_typ < maxnum_filter )
    {
        UART1_modbus.loopback_filter = filter_typ;
        return 0;
    }
    else
    {
        UART1_modbus.loopback_filter = slave_addr_filter;
        return -1;
    }
}

char get_modbus_loopback_filter(void)
{
    return (char)UART1_modbus.loopback_filter;
}

void tmr2_setting( waiting_typ_t type )
{
    if( type == silent_wait )
    {
        PR2  = modbus_SilentInt;
    }
    else if( type == response_delay )
    {
        PR2  = resp_delay_ms_trans_tmr2( UART1_modbus.wait_ctrl.delay_ms );
    }

    UART1_modbus.wait_ctrl.wait_type = type;
    TMR2= 0;
    T2CONbits.TMR2ON = 1;
}

unsigned char resp_delay_ms_trans_tmr2( char delay_ms )
{
    // tmr4: 1cnt = 8usec.
    // e.g. 1ms = 1000/8 = 125cnt.

    unsigned char tmr4_cnt;

    if( delay_ms == 0 )
        tmr4_cnt = modbus_resp_0ms;
    else
        tmr4_cnt = modbus_resp_1ms * (unsigned char) delay_ms;

    return tmr4_cnt;
}