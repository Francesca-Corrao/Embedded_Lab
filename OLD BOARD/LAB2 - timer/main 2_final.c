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
    //function that wait for the timer to end a period
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
    //function that initialize a timer of a fixed period and wait for it to finish, similar to sleep or wait.
    tmr_setup_period(timer, ms);
    tmr_wait_period(timer);
}
int main(void) {    
    TRISBbits.TRISB0 = 0; //set up RB0 register(D3 led) as output
    TRISEbits.TRISE8 = 1; //set RE8(button S5) as input
    LATBbits.LATB0=0;//turn off the led
    int pulse = 0; //variable to know how many pulse the led should do
    int button_info; //variable to read the button
    int cycle = 0; //variable to know in which cycle of 100ms are we
    int button_state = 1; //variable to kept track of how the button was in the previous iteration
    tmr_setup_period(TIMER1,100); //set up and start a timer of 100 ms
    
    while(1){
        button_info = PORTEbits.RE8 ;
        
        if (button_info == 0 && button_state!=0){
            //if the button is press increment the pulse and start a timer to know if the user is pressing for 3s
            tmr_setup_period(TIMER2,3000);
                        
            if(pulse<3){
                pulse++;
                
            }
            else{
                pulse = 1;
            }
            
            cycle = 1;
        
        }    
        
        if(button_info == 1 && button_state != 1){
            //when the button is release if the timer of 3s is expired stop the LED D3 pulse.
            if(IFS0bits.T2IF){
                IFS0bits.T2IF = 0;
                pulse = 0;
                LATBbits.LATB0=0;
                T2CONbits.TON = 0;//stop timer
            }
            T2CONbits.TON = 0;//stop timer
        }
        
        button_state=button_info;
        
        if((cycle%2)!= 0 && cycle < (2*pulse)){
            //if we are in the cycle in which the LED D3 should be on turn it on
            LATBbits.LATB0=1;            
        }
        else{
            LATBbits.LATB0=0; //turn off the LED D3
        }
        
        cycle++; 
        
        if(cycle == 11){
            cycle = 1; //reset the cycle if it execedes 10
        }
        
        tmr_wait_period(TIMER1);
           
    }
    
    return 0;
}