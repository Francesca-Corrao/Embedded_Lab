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
#define TIMER3 3 //timer 3 
#define TIMER4 4 //timer 4

#define BUFFER_SIZE 12 
/*
 BUFFER_SIZE has been assigned to 12 for the following reasons:
  - the baud rate of the UART is 9600 bps
  - 1 byte is 10 baud
  - in 10ms 10 byte can be sent 
  
  9600/10 = 960 byte in 1 second 
  960 / 1000 = 0,96 byte in 1 ms 
  0,96 * 10 = 9,6 byte in 10 ms
  rounding = 10 byte in 10 ms which is at the limit then
  buffer_size = 12 
 */

typedef struct {
    int chars2read;
    int read_p;
    int write_p;
    char cbuf[BUFFER_SIZE];
}circularBuf;

volatile circularBuf buf;
volatile int send = 0; // Bool to notify if S5 has been pressed
volatile int reset = 0;// Bool to notify if S6 has been pressed

void tmr_setup_period(int timer, int ms){
    //set up the timer to count for  the specific amount of milliseconds
    //frequency 7.3728 MHz
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
        case 4:{
            TMR4 = 0 ;//reset timer
            PR4 = clk_steps; // set value to count up to
            T4CONbits.TCKPS = 3; //set prescaler
            T4CONbits.TON = 1;//start timer
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
        case 4:{
            while(1){
                if(IFS1bits.T4IF){
                    IFS1bits.T4IF = 0;
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
    //clean LCD by writing 2 rows of blank spaces
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

void clean_first_row(){
    while(SPI1STATbits.SPITBF == 1);
    SPI1BUF = 0x80;
    for(int i = 0; i<16 ;i++){
        while(SPI1STATbits.SPITBF == 1);
            SPI1BUF = ' ';
    }
    while(SPI1STATbits.SPITBF == 1);
    SPI1BUF = 0x80;
}

void print_lcd(char* str){
    //write on the LCD
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

void algorithm() {
    tmr_wait_ms(TIMER2, 7);
}

void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt(){
    IFS1bits.U2RXIF = 0; // reset if flag when arrives at least 1 byte
    buf.cbuf[buf.write_p] = U2RXREG; //write
    buf.write_p ++; //increment the pointer
    buf.chars2read ++; //increment characters to be red
    if(buf.write_p == BUFFER_SIZE){
        buf.write_p = 0; // back to 0 if oversized
    }
}

void __attribute__((__interrupt__,__auto_psv__)) _INT0Interrupt(){
    //Interrupt service Routine associated to INT0 flag 
    //(external signal associated to the high edge detection of S5)
    //reset interrupt flag
    IFS0bits.INT0IF = 0;
    //start timer3 to avoid bouncing:
    //wait 20ms and check button flag
    tmr_setup_period(TIMER3,20);   
    
}

void __attribute__((__interrupt__,__auto_psv__)) _T3Interrupt(){
    //Interrupt service routine triggered by timer3 expiration.
    //reset interrupt flag
    IFS0bits.T3IF = 0;
    //stop timer3
    T3CONbits.TON = 0;
    //check button S5 info 
    if(PORTEbits.RE8 == 1){
        send = 1;        
    }       
}

void __attribute__((__interrupt__,__auto_psv__)) _INT1Interrupt(){
    //Interrupt service Routine associated to INT1 flag 
    //(external signal associated to the high edge detection of S6)
    //reset interrupt flag
    IFS1bits.INT1IF = 0;
    //start timer3 to avoid bouncing:
    //wait 20ms and check button flag 
    tmr_setup_period(TIMER4,20);   
}

void __attribute__((__interrupt__,__auto_psv__)) _T4Interrupt(){
    //Interrupt service routine triggered by timer4 expiration.
    //reset interrupt flag
    IFS1bits.T4IF = 0;
    //stop timer4
    T4CONbits.TON = 0;
    //check button S6 info 
    if(PORTDbits.RD0 == 1){
        reset = 1;
    }      
}
void write_second_line(int c){
    char str[13];
    sprintf(str,"Char Recv: %d", c );
    while(SPI1STATbits.SPITBF == 1);
    SPI1BUF = 0xC0;
    print_lcd(str);   
}

int main(void) {
    
    //Define UART2
    U2BRG = 11; // (7372800/4) / (16 * 9600) - 1
    U2MODEbits.UARTEN = 1; //enable he UART mode
    U2STAbits.UTXEN = 1; //enable transmission 
    
    //Interrupt 
    U2STAbits.URXISEL = 01; 
    
    //Define LCD
    SPI1CONbits.MSTEN = 1;
    SPI1CONbits.MODE16 = 0;
    SPI1CONbits.PPRE = 3;
    SPI1CONbits.SPRE = 6;
    SPI1STATbits.SPIEN = 1;
    
    //Button init
    TRISEbits.TRISE8 = 1; //set RE8(button S5) as input
    TRISDbits.TRISD0 = 1; //set RD0(button S6) as input
    //enable interrupt
    IEC0bits.INT0IE = 1; //button S5
    IEC0bits.T3IE = 1; //TIMER3
    IEC1bits.INT1IE = 1; //button S6
    IEC1bits.T4IE = 1; //TIMER4

    //wait 1 sec
    tmr_wait_ms(TIMER1,1000);
    
    //clean_lcd
    clean_lcd();
    
    //tmr
    tmr_setup_period(TIMER1, 10);
   
    //enable interrupt
    IEC1bits.U2RXIE = 1; 
    
    //counter 
    int count = 0 ; //chars received
    int place = 0; //cursor position
    
    while(1) {
        
        algorithm();
        // code to handle the assignment
        //read_cbuf()
        while (buf.chars2read > 0){
            
            int s = 0x80;
            s += place % 16;
            SPI1BUF = (unsigned char)s;
            
            if (place % 16 == 0){
                //when line ends start back
                clean_first_row();
            }
            if(buf.cbuf[buf.read_p] == '\n' || buf.cbuf[buf.read_p] =='\r'){
                clean_first_row();
                place = 0;
            }
            else{
                place++;
                while(SPI1STATbits.SPITBF == 1);
                SPI1BUF = buf.cbuf[buf.read_p];
            }
            buf.read_p ++;
            buf.chars2read --;
            if(buf.read_p == BUFFER_SIZE){
                buf.read_p = 0;
            }
            
            count++;
        }
        
        if(reset){
            count = 0;
            clean_lcd();
            reset = 0;
            place = 0;
        }
        
        if(send){
            char send_str[5];
            sprintf(send_str,"%d",count);
            for(int i = 0; i<strlen(send_str); i++){
            //write on UART
            U2TXREG = send_str[i];
            }
            send = 0;
        }
        
        write_second_line(count);
        tmr_wait_period(TIMER1);
    }
}