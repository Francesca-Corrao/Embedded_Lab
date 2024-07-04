/*
 * File:   newmainXC16.c
 * Author: Giulia
 *
 * Created on 10 ottobre 2023, 11.50
 */

// DSPIC30F4011 Configuration Bit Settings

// 'C' source line config statements

// FOSC
#pragma config FPR = XT           // Primary Oscillator Mode (XTL)
#pragma config FOS = PRI           // Oscillator Source (Primary Oscillator)
#pragma config FCKSMEN = CSW_FSCM_OFF// Clock Switching and Monitor (Sw Disabled, Mon Disabled)

// FWDT
#pragma config FWPSB = WDTPSB_16   // WDT Prescaler B (1:16)
#pragma config FWPSA = WDTPSA_512  // WDT Prescaler A (1:512)
#pragma config WDT = WDT_OFF       // Watchdog Timer (Disabled)

// FBORPOR
#pragma config FPWRT = PWRT_64  // POR Timer Value (64ms)
#pragma config BODENV = BORV20  // Brown Out Voltage (Reserved)
#pragma config BOREN = PBOR_ON  // PBOR Enable (Enabled)
#pragma config LPOL = PWMxL_ACT_HI// Low-side PWM Output Polarity (Active High)
#pragma config HPOL = PWMxH_ACT_HI// High-side PWM Output Polarity (Active High)
#pragma config PWMPIN = RST_IOPIN// PWM Output Pin Reset (Control with PORT/TRIS regs)
#pragma config MCLRE = MCLR_EN  // Master Clear Enable (Enabled)

// FGS
#pragma config GWRP = GWRP_OFF      // General Code Segment Write Protect (Disabled)
#pragma config GCP = CODE_PROT_OFF  // General Segment Code Protection (Disabled)

// FICD
#pragma config ICS = ICS_PGD       // Comm Channel Select (Use PGC/EMUC and PGD/EMUD)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <string.h>
#include <stdio.h>
#define TIMER1 1 //timer 1 
#define TIMER2 2 //timer 2 

void tmr_setup_period(int timer, int ms){
    //set up the timer to count for  the specific amount of milliseconds
    
    //frequenza 7.3728 MHz
    long int Fcy = 7372800/4;
    long int clk_steps = (Fcy/256)*(ms/1000.0);
    switch(timer){
        case 1:{
            TMR1 = 0; //reset timer
            PR1 = clk_steps; // set value to count up to
            T1CONbits.TCKPS = 3; //set prescaler
            T1CONbits.TON = 1;//start timer
            break;
        }
        case 2:{
            TMR2 = 0; //reset timer
            PR2 = clk_steps; // set value to count up to
            T2CONbits.TCKPS = 3; //set prescaler
            T2CONbits.TON = 1;//start timer
            break;
        }
        case 3:{
            TMR3 = 0 ;//reset timer
            PR3 = clk_steps; // set value to count up to
            T3CONbits.TCKPS = 3; //set prescaler
            T3CONbits.TON = 1;//start timer
            break;
        }
    }
}

void tmr_wait_period(int timer){
    switch(timer){
        case 1:{
            while(1){
                if(IFS0bits.T1IF){
                    IFS0bits.T1IF = 0;
                    break;
                }
            }break;
        }
        case 2:{
            while(1){
                if(IFS0bits.T2IF){
                    IFS0bits.T2IF = 0;
                    break;
                }
            }break;
        }
        case 3:{
            while(1){
                if(IFS0bits.T3IF){
                    IFS0bits.T3IF = 0;
                    break;
                }
            }break;
        }
    }    
}

void tmr_wait_ms(int timer, int ms){
    tmr_setup_period(timer, ms);
    tmr_wait_period(timer);
}

void clean_lcd(){
    while(SPI1STATbits.SPITBF == 1);
    SPI1BUF = 0x80;
    for(int j = 0; j<2; j++){
        for(int i = 0; i<16 ;i++){
            while(SPI1STATbits.SPITBF == 1);
            SPI1BUF = ' ';
        }
        while(SPI1STATbits.SPITBF == 1);
        SPI1BUF = 0xC0;
    }
    while(SPI1STATbits.SPITBF == 1);
    SPI1BUF = 0x80;
}

void print_lcd(char* str){
    clean_lcd();
    int len=strlen(str);
    for(int i = 0; i<len;i++){
        if(str[i]== '\n'){
            while(SPI1STATbits.SPITBF == 1);
            SPI1BUF = 0xC0;
            i++;
        };
        while(SPI1STATbits.SPITBF == 1);
        SPI1BUF = str[i];
    }
}

int main(void) { 
    //LCD init
    SPI1CONbits.MSTEN = 1;
    SPI1CONbits.MODE16 = 0;
    SPI1CONbits.PPRE = 3;
    SPI1CONbits.SPRE = 3;
    SPI1STATbits.SPIEN = 1;
    
    //Button init
    TRISEbits.TRISE8 = 1; //set RE8(button S5) as input
    int cycle = 0; //variable to know in which cycle of 100ms are we
    int button_info; //variable to read the button
    int button_state = 1; //variable to kept track of how the button was in the previous iteration
   
    int c = 0;
    char buf[32];
    
    tmr_setup_period(TIMER1,100);
    
    while(1){
        button_info = PORTEbits.RE8;
        
        if (button_info == 0 && button_state!=0){
            c=0;
            sprintf(buf,"%d",c);
            print_lcd(buf);
        }
        
        if(cycle==9){
            c++;
            sprintf(buf,"%d",c);
            print_lcd(buf);
            cycle = 0;
        }
        
        button_state = button_info;
        tmr_wait_period(TIMER1);
        cycle++;
    }
    
}