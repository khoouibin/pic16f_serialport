// ---------------------------------------------------------------------------
//  Filename: UART_modbus.c
//  Created by: Khoo UiBin
//  Date:  July05, 2021.
//  Orisol Taiwan(c)
// ---------------------------------------------------------------------------
#include <xc.h>
#include "uart_modbus.h"

uart_modbus_status_t UARTStatus;
unsigned char tx_testStr[9] = {0xaa, 0xbb, 0xcc, 0xdd, 0x12, 0x34, 0x56, 0x78, 0xff};

void UART1_modbus_init(void)
{
    UART1_module_init();
    Tmr1_module_init();
    set_modbus_mode(echo_mode);
    // set_modbus_mode( normal_mode );
    set_modbus_slave_addr(SLAVE_ADDR_DEF);
    set_modbus_loopback_filter(no_filter);
    set_modbus_resp_delay_ms(5);
}

void Tmr1_module_init(void)
{
    T1CONbits.TMR1CS = 0b00; // clk source = Fosc/4
    T1CONbits.T1CKPS = 0b11; // prescale =1:8
    T1CONbits.T1OSCEN = 0;
    T1CONbits.TMR1ON = 0;

    T1GCONbits.TMR1GE = 0;
    PIE1bits.TMR1IE = 1;
    TMR1H = 0xff;
    TMR1L = 0xf0;
}

void UART1_module_init(void)
{
    TXSTAbits.CSRC = 1; // clock source select: master mode
    TXSTAbits.TX9 = 0;  // 8-bit transmission
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
    PIR1bits.RCIF = 0;
}

char set_modbus_mode(modbus_mode_t mode)
{
    char res = 0;

    if (mode < maxnum_mode)
    {
        UARTStatus.mode = mode;
        UARTStatus.status = status_rx;
        UARTStatus.wait_ctrl.wait_type = silent_wait;
        UARTStatus.tx_ctrl.txSize = 0;
        UARTStatus.rx_ctrl.rxSize = 0;
        UARTStatus.tx_ctrl.send_num = 0;

        UARTStatus.rx_ctrl.ptr_rxBuf = &UARTStatus.rx_ctrl.rxBuf[0];
        UARTStatus.tx_ctrl.ptr_txBuf = &UARTStatus.tx_ctrl.txBuf[0];

        if (UARTStatus.mode == periodic_mode)
        {
            if (UARTStatus.wait_ctrl.delay_ms < REPLY_PERIOD_MIN)
                UARTStatus.wait_ctrl.delay_ms = REPLY_PERIOD_MIN;
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
    return UARTStatus.mode;
}

char set_modbus_slave_addr(char addr)
{
    //  broadcast addr: 00h.   ( not support )
    //  slave addr:     01~7fh.
    unsigned char addr_temp = (unsigned char)addr;
    char res = 0;

    if (addr_temp == 0x00)
        res -= 1;
    else if (addr_temp > 0x7f)
        res -= 2;
    else
        UARTStatus.slave_addr = addr_temp;

    return res;
}

char get_modbus_resp_delay_ms(void)
{
    return UARTStatus.wait_ctrl.delay_ms;
}

void set_modbus_resp_delay_ms(char delay_ms)
{
    UARTStatus.wait_ctrl.delay_ms = delay_ms;
}

char get_modbus_slave_addr(void)
{
    return UARTStatus.slave_addr;
}

char set_modbus_loopback_filter(modbus_loopback_t filter_typ)
{
    char res = 0;
    if (filter_typ < maxnum_filter)
    {
        UARTStatus.loopback_filter = filter_typ;
    }
    else
    {
        UARTStatus.loopback_filter = slave_addr_filter;
        res -= 1;
    }
    return res;
}

modbus_loopback_t get_modbus_loopback_filter(void)
{
    return UARTStatus.loopback_filter;
}

waiting_typ_t get_Tmr1_WaitType(void)
{
    return UARTStatus.wait_ctrl.wait_type;
}

void Tmr1_TypeSetting(waiting_typ_t type)
{
    unsigned char hibyte, lobyte;
    if (type == silent_wait)
    {
        // PR2  = modbus_SilentInt;
        TMR1H = modbus_SilentH;
        TMR1L = modbus_SilentL;
    }
    else if (type == response_delay)
    {
        delay_ms_to_Tmr1Count(UARTStatus.wait_ctrl.delay_ms, &hibyte, &lobyte);
        TMR1H = hibyte;
        TMR1L = lobyte;
    }
    UARTStatus.wait_ctrl.wait_type = type;
    T1CONbits.TMR1ON = 1;
}

void Tmr1_Off(void)
{
    T1CONbits.TMR1ON = 0;
}

void delay_ms_to_Tmr1Count(char delay_ms, unsigned char *hb, unsigned char *lb)
{
    // tmr1: 1cnt = 1usec.
    // e.g. 1ms = 1000 cnt.

    int delay_cnt;
    char delay_val = 0;
    unsigned char temp_val;

    if (delay_ms == 0)
    {
        *hb = 0Xff;
        *lb = 0xff - modbus_resp_0ms;
    }
    else
    {
        if (delay_ms > 65)
            delay_val = 65;
        else
            delay_val = delay_ms;

        delay_cnt = modbus_resp_1ms * (unsigned char)delay_val;
        temp_val = (delay_cnt >> 8) & 0xff;
        *hb = 0xff - temp_val;

        temp_val = (delay_cnt)&0xff;
        *lb = 0xff - temp_val;
    }
}

void UART_Rx(unsigned char uart_byte)
{
    (*(UARTStatus.rx_ctrl.ptr_rxBuf)++) = uart_byte;
    UARTStatus.rx_ctrl.rxSize++;

    Tmr1_Off();
    Tmr1_TypeSetting(silent_wait);
}

void UART_Tx(unsigned char data)
{
    NOP();
    TXREG = data;
}

void UART_TxInterruptEnable(void)
{
    PIE1bits.TXIE = 1;
}

void UART_TxInterruptDisable(void)
{
    PIE1bits.TXIE = 0;
}

char txBuf_send(void)
{
    unsigned char send_temp;

    if (UARTStatus.tx_ctrl.send_num < UARTStatus.tx_ctrl.txSize)
    {
        send_temp = *UARTStatus.tx_ctrl.ptr_txBuf++;
        UART_Tx(send_temp);
        UARTStatus.tx_ctrl.send_num++;
        if (UARTStatus.tx_ctrl.txSize > 1)
        {
            UART_TxInterruptEnable();
        }
        return 0;
    }
    else
    {
        UART_TxInterruptDisable();
        return 255;
    }
}

modbus_idle_t isReceiveIdle(void)
{
    if (BAUDCONbits.RCIDL == 0)
        return state_receiving;
    else
        return state_idle;
}

void Tmr1_Process(void)
{
    char res;
    char execute_status;

    if (get_Tmr1_WaitType() == silent_wait)
    {
        if (get_modbus_mode() == echo_mode)
        {
            if (rxBuf_LoopbackMode_check(&UARTStatus.rx_ctrl.rxBuf[0]) == 0)
            {
                txBuf_load(&UARTStatus.rx_ctrl.rxBuf[0], UARTStatus.rx_ctrl.rxSize);
                // txBuf_load(&tx_testStr[0], 5);

                Tmr1_TypeSetting(response_delay);
            }

            if (isReceiveIdle() == state_idle)
                reset_modbus_status_ptr(&UARTStatus);
        }
        else if (get_modbus_mode() == normal_mode)
        {
            NOP();
        }
    }
    else if (get_Tmr1_WaitType() == response_delay)
    {
        UARTStatus.tx_ctrl.send_num = 0;
        if (get_modbus_mode() == echo_mode)
        {
            txBuf_send();
        }
    }
}

char rxBuf_LoopbackMode_check(unsigned char *data_buf)
{
    char res = 0;
    if (UARTStatus.loopback_filter == no_filter)
        res = 0;
    else if (UARTStatus.loopback_filter == slave_addr_filter)
        res = rxBuf_check_SlaveAddr(data_buf);
    else
        res = 255;
    return res;
}

char rxBuf_check_SlaveAddr(unsigned char *data_buf)
{
    if (*data_buf == UARTStatus.slave_addr)
        return 0;
    else
        return 255;
}

void txBuf_load(unsigned char *data_buf, unsigned char leng)
{
    int i;
    for (i = 0; i < leng; i++)
    {
        UARTStatus.tx_ctrl.txBuf[i] = *data_buf;
        data_buf++;
    }
    UARTStatus.tx_ctrl.txSize = leng;
}

void reset_modbus_status_ptr(uart_modbus_status_t *status)
{
    status->rx_ctrl.ptr_rxBuf = &status->rx_ctrl.rxBuf[0];
    status->tx_ctrl.ptr_txBuf = &status->tx_ctrl.txBuf[0];
    status->rx_ctrl.rxSize = 0;
}