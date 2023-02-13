/*
 * File:   main.c
 * Author: khoo uibin
 *
 * Created on February 13, 2023, 2:19 PM
 */

#include <xc.h>
#include "init_all.h"

#define _XTAL_FREQ 32000000

// __CONFIG(FOSC_INTOSC & WDTE_OFF & LVP_OFF & PLLEN_ON);
//-----------------------------------------------
#pragma config FOSC = INTOSC
#pragma config WDTE = OFF
#pragma config LVP = OFF
#pragma config PLLEN = ON

void main(void)
{
    NOP();
    NOP();
    InitAll();


    while(1)
    {
        
    }
    return;
}

