
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
    long int Fcy = 70000000;
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

int main(void) {
    
    float ADCValue;
    float d;
    float lvl = 3.3/1024.0;
    float v;
    char buf[16];
    
    
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0;
    
    // enable 
    TRISBbits.TRISB10 = 0;
    LATBbits.LATB10=1;
    
    ANSELBbits.ANSB14 = 1;
    
    //remap
    RPOR0bits.RP64R = 0x01;
    RPINR18bits.U1RXR = 0x4b;
     
    //Define UART2
    U1BRG = 468; // (72000000) / (16 * 9600) - 1 = 467.75
    U1MODEbits.UARTEN = 1; //enable he UART mode
    U1STAbits.UTXEN = 1; //enable transmission 
    
    
    
    // Configure ADC
    AD1CON3bits.ADCS = 11; //select Tad: TCY=1/FCY; TAD = at least 154/TCY -1???
    AD1CON1bits.ASAM = 0; //select how sampling begins MANUAL
    AD1CON1bits.SSRC = 7; //select how sampling end AUTO
    AD1CON3bits.SAMC = 16; //sampling time
    AD1CON2bits.CHPS = 0; //how many channel
    //AD1CHS0bits.CH0SA = 01110; //An14 in binary
    
    AD1CHS0bits.CH0SA = 14; //An14
    AD1CON1bits.ADON = 1; //turn it on
    
    while(1){
        AD1CON1bits.SAMP = 1; //start sampling
        while(!AD1CON1bits.DONE){}
        ADCValue=ADC1BUF0;
        v = ADCValue*lvl;
        d = 2.34 - 4.74*v + 4.06*v*v - 1.60*v*v*v + 0.24*v*v*v*v;
        sprintf(buf, "%f", d);
        U1TXREG = 'd';
        U1TXREG = ':';
        for(int i = 0; i<strlen(buf); i++){
            //write on UART
            U1TXREG = buf[i];
            }
        U1TXREG = ' ';
        LATGbits.LATG9=0;
        for(int i = 0; i<5; i++){
            tmr_wait_ms(TIMER1,200);
        }
    }
}