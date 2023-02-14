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
    Tmr1_module_init();
    set_modbus_mode( loopback_mode );
    //set_modbus_mode( normal_mode );
    set_modbus_slave_addr(SLAVE_ADDR_DEF);
    set_modbus_loopback_filter( no_filter );
    set_modbus_resp_delay_ms( 5 );
}


void Tmr1_module_init(void)
{
    // PR2 = 93
    // tmr2 prescaler = 64
    // Fosc/4 = 32 Mhz / 4 = 8 Mhz.
    // 1 instruction cycle = 0.125 usec.
    // 1 cnt = 0.125 usec * 64 = 8 usec.
    // TMR2 = 0;
    // T2CON = 0;
    // T2CONbits.T2CKPS = 0b11;
    // PR2 = modbus_SilentInt;
    // PIE1bits.TMR2IE = 1;
    // T2CONbits.TMR2ON = 0;

    T1CONbits.TMR1CS = 0b00; //clk source = Fosc/4
    T1CONbits.T1CKPS = 0b11; //prescale =1:8
    T1CONbits.T1OSCEN = 0;
    T1CONbits.TMR1ON = 0;

    T1GCONbits.TMR1GE = 0;
    PIE1bits.TMR1IE = 1;

    TMR1H = 0xff;
    TMR1L = 0xf0;
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

char set_modbus_mode( modbus_mode_t mode )
{
//    char mode_max = (char)maxnum_mode;
//    char mode_input=(char)mode;
    char res = 0;
    
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
    }
    else
    {
        res -= 1; 
    }
    return res;
}

modbus_mode_t get_modbus_mode(void)
{
    return UART1_modbus.mode;
}

char set_modbus_slave_addr( char addr )
{
//  broadcast addr: 00h.   ( not support )
//  slave addr:     01~7fh.
    unsigned char addr_temp = (unsigned char)addr;
    char res = 0;

    if( addr_temp == 0x00 )
        res -= 1;
    else if( addr_temp > 0x7f )
        res -= 2;
    else
        UART1_modbus.slave_addr = addr_temp;

    return res;
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

char set_modbus_loopback_filter( modbus_loopback_t filter_typ )
{
    char res = 0;
    if( filter_typ < maxnum_filter )
    {
        UART1_modbus.loopback_filter = filter_typ;
    }
    else
    {
        UART1_modbus.loopback_filter = slave_addr_filter;
        res -= 1;
    }
    return res;
}

modbus_loopback_t get_modbus_loopback_filter(void)
{
    return UART1_modbus.loopback_filter;
}

void tmr1_setting( waiting_typ_t type )
{
    unsigned char hibyte,lobyte;
    if( type == silent_wait )
    {
        //PR2  = modbus_SilentInt;
        TMR1H = modbus_SilentH;
        TMR1L = modbus_SilentL;
    }
    else if( type == response_delay )
    {
        resp_delay_ms_trans_tmr1( UART1_modbus.wait_ctrl.delay_ms, &hibyte,&lobyte);
        TMR1H = hibyte;
        TMR1L = lobyte;
        
    }

    UART1_modbus.wait_ctrl.wait_type = type;
    //T2CONbits.TMR2ON = 1;
    T1CONbits.TMR1ON = 1;
}

void resp_delay_ms_trans_tmr1( char delay_ms,unsigned char* hb,unsigned char* lb)
{
    // tmr1: 1cnt = 1usec.
    // e.g. 1ms = 1000 cnt.

    int delay_cnt;
    char delay_val = 0;
    unsigned char temp_val;

    if( delay_ms == 0 )
    {
        *hb = 0Xff;
        *lb = 0xff-modbus_resp_0ms;
    }
    else
    {
        if (delay_ms > 65)
            delay_val = 65;
        else
            delay_val = delay_ms;

        delay_cnt = modbus_resp_1ms * (unsigned char) delay_val;
        temp_val = (delay_cnt >> 8) & 0xff ;
        *hb = 0xff - temp_val ;
        
        temp_val = (delay_cnt) & 0xff;
        *lb = 0xff - temp_val ;
    }
}