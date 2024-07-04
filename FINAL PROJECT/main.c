#include <xc.h>
#include <string.h>
#include <stdio.h>

// UART function
#define VOLT (1) // send voltage
#define DIST (2) // send distnce in cm
#define PWM (3) // send DC

// Parse fuction
#define STATE_DOLLAR (1) // discard everything until a dollar is found
#define STATE_TYPE (2) // read the type of msg until a comma is found
#define STATE_PAYLOAD (3) // read the payload until an asterix is found
#define NO_MESSAGE (-1) // In case of no message recived 
#define NEW_MESSAGE (4) // Everything wet fine

// ADC function
#define IR_SENSOR (1) // read distance 
#define BATTERY_SENSOR (0) // read voltage

// Timer
#define TIMER1 1 // timer 1 
#define TIMER2 2 // timer 2


#define BUFFER_SIZE 15 // size circular buffer
/*
 BUFFER_SIZE has been assigned to 15 for the following reasons:
  - the baud rate of the UART is 9600 bps
  - 1 byte is 10 baud
  - in 1ms 1 byte can be sent by the UART 
  
  9600/10 = 960 byte in 1 second 
  960 / 1000 = 0,96 byte in 1 ms 
  rounding = 1 byte in 1 ms
  theoretically we do not even need the circular buffer 
  but it is more convenient to store information 
*/

// struct yaw-surge
typedef struct{
    float surge;
    float yaw_rate;
    int flag;
}pwm_percentage;

// struct threshold
typedef struct{
    int minth;
    int maxth;
}threshold;

// struct buffer
typedef struct {
    int chars2read;
    int read_p;
    int write_p;
    char cbuf[BUFFER_SIZE];
}circularBuf;

// struct parse
typedef struct {
    int state;
    char msg_type[6]; // type is 5 chars + string terminator
    char msg_payload[100]; // assume payload cannot be longer than 100 chars
    int index_type;
    int index_payload;
}parser_state;

// global variable 
volatile int state; // state variable 0 - wait 1 - moving
volatile circularBuf buf; // circular buffer

// timer functions
void tmr_setup_period(int timer, int ms){
    // set up the timer to count for  the specific amount of milliseconds
    // frequency 72 MHz
    long int Fcy = 72000000;
    long int clk_steps = (Fcy/256)*(ms/1000.0);
    switch(timer){
        case 1:{
            TMR1 = 0; // reset timer
            PR1 = clk_steps; // set value to count up to
            T1CONbits.TCKPS = 3; // set prescaler
            T1CONbits.TON = 1; // start timer
            break;
        }
        case 2:{
            TMR2 = 0; // reset timer
            PR2 = clk_steps; // set value to count up to
            T2CONbits.TCKPS = 3; // set prescaler
            T2CONbits.TON = 1; // start timer
            break;
        }
        case 3:{
            TMR3 = 0 ; // reset timer
            PR3 = clk_steps; // set value to count up to
            T3CONbits.TCKPS = 3; // set prescaler
            T3CONbits.TON = 1; // start timer
            break;
        }
    }
}

void tmr_wait_period(int timer){
    // wait the timer to expire
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

// Interrupt RE8 for changing state back and fort
void __attribute__((__interrupt__,__auto_psv__)) _INT1Interrupt(){
    // ISR associated to INT1 flag associated to RE8 switch
    IFS1bits.INT1IF = 0; // reset interrupt flag
    // start timer2 to avoid bouncing
    tmr_setup_period(TIMER2,10);   
}

void __attribute__((__interrupt__,__auto_psv__)) _T2Interrupt(){
    // ISR triggered by timer 2 expiration.
    IFS0bits.T2IF = 0; // reset interrupt flag
    T2CONbits.TON = 0; // stop timer2
    //check button info if still close switch state 
    if(PORTEbits.RE8 == 1){
        state = !state;
    }
}

// Calculate yaw & surge from threshold  
void set_pwm_percentage(pwm_percentage*p, threshold*t, float sensed_d){
    // setting proportional parameter
    int s = 2 ;
    int y = 500;
    
    if(sensed_d < t->minth){
        // turn right
        p->surge = 0;
        p->yaw_rate = 100;
        p->flag = 1; //flag to wait
    }
    else if(sensed_d > t->maxth){
        // go forward
        p->surge = 100;
        p->yaw_rate = 0;
        p->flag = 0;
    }
    else{
        // combined movement - proportional law
        p->surge = sensed_d * s; // surge directly proportional
        p->yaw_rate = y / sensed_d; // yaw inv pro
    }
}

// Set PWM output compare
void init_pwm(int Fpwm){    
    // RD1(RP65) --> OC1
    RPOR0bits.RP65R = 0x10;
    // RD2(RP66) --> OC2
    RPOR1bits.RP66R = 0x11;
    // RD3(RP67) --> OC3
    RPOR1bits.RP67R = 0x12;
    // RD4(RP68) --> OC4
    RPOR2bits.RP68R = 0x13;
    
    // set OC1
    OC1CON1bits.OCTSEL = 7;
    OC1CON1bits.OCM = 6;
    OC1CON2bits.SYNCSEL = 0x1F;
    // set OC2
    OC2CON1bits.OCTSEL = 7;
    OC2CON1bits.OCM = 6;
    OC2CON2bits.SYNCSEL = 0x1F;
    // set OC3
    OC3CON1bits.OCTSEL = 7;
    OC3CON1bits.OCM = 6;
    OC3CON2bits.SYNCSEL = 0x1F; 
    // set OC4
    OC4CON1bits.OCTSEL = 7;
    OC4CON1bits.OCM = 6;
    OC4CON2bits.SYNCSEL = 0x1F;
    
    if(Fpwm < 1100) return; // return if number out of range
    
    // Fcy = 72 MHz
    int Fcy = 72000000;
    OC1RS = Fcy/Fpwm;
    OC2RS = Fcy/Fpwm;
    OC3RS = Fcy/Fpwm;
    OC4RS = Fcy/Fpwm;
    
    OC1R = 0;
    OC2R = 0;
    OC3R = 0;
    OC4R = 0;
}

void stop_pwm(){
    OC1R = 0;
    OC2R = 0;
    OC3R = 0;
    OC4R = 0;
}

// set PWM DC according to yaw & surge
void motor_ctrl(pwm_percentage*p){
    
    // sum left pwm 
    float left_pwm = p->surge + p->yaw_rate;
    // if exceed the 100 rescale at 90%
    if(left_pwm > 100){
        left_pwm = 0.9 * left_pwm;
    }
    // subtract right pwm
    float right_pwm = p->surge - p->yaw_rate;
    // same at -100
    if(right_pwm < -100){
        right_pwm = 0.9 * right_pwm;
    }
    
    // control the wheels according to Table 1
    if (left_pwm > 0){
        OC1R = 0;
        OC2R = (left_pwm/100) * OC2RS; 
    }
    else if (left_pwm < 0){
        OC2R = 0;
        OC1R = -(left_pwm/100) * OC1RS;
    }
    if (right_pwm > 0){
        OC3R = 0;
        OC4R = (right_pwm/100) * OC4RS;
    }
    else if (right_pwm < 0){
        OC4R = 0;
        OC3R = -(right_pwm/100) * OC3RS;
    }
}

// Read IR or Battery sensor tension 
float analog_measure(int flag){
    float lvl = 3.3/1024.0; // 3.3 max Volt mapped in 10 bit
    float ADCValue; 
    float v;
    do{
        if (AD1CON1bits.SAMP == 0)
            AD1CON1bits.SAMP = 1; // trigger the sampling
    }while(!AD1CON1bits.DONE);
    // Read IR sensor
    if(flag){      
        ADCValue = ADC1BUF1;
        v = ADCValue*lvl;
        float dist = (2.34 - 4.74*v + 4.06*v*v - 1.60*v*v*v + 0.24*v*v*v*v)*100;
        return dist;
    }
    // Read Battery sensor
    else{
        ADCValue = ADC1BUF0;
        v = ADCValue*lvl;
        float tens = 3*v; // computation on circuit
        return tens;        
    }
}

// send to UART
void transmit_uart(int type){
    char buf[25]; 
    int dim = 0;
    // formatting string to transmit
    switch(type){
        case VOLT:{
            float volt = analog_measure(BATTERY_SENSOR);
            sprintf(buf,"$MBATT,%.2f*\n",volt);
            dim = strlen(buf);
            break;
        }
        case DIST:{
            int dist = (int)analog_measure(IR_SENSOR);
            sprintf(buf,"$MDIST,%d*\n",dist);
            dim = strlen(buf);
            break;
        }
        case PWM:{
            int dc1 = ((OC1R*1.0)/(OC1RS*1.0))*100;
            int dc2 = ((OC2R*1.0)/(OC2RS*1.0))*100;
            int dc3 = ((OC3R*1.0)/(OC3RS*1.0))*100;
            int dc4 = ((OC4R*1.0)/(OC4RS*1.0))*100;
            sprintf(buf,"$MPWM,%d,%d,%d,%d*\n",dc1,dc2,dc3,dc4);
            dim = strlen(buf);
            break;
        }
        default: break;
    }
    // effective transmission
    for(int i = 0; i < dim; i++){
        while(!U1STAbits.TRMT);
        U1TXREG = buf[i];
    }
}

// parsing functions
int parse_byte(parser_state* ps, char byte) {
    switch (ps->state) {
        case STATE_DOLLAR:
            if (byte == '$' ) {
                ps->state = STATE_TYPE;
                ps->index_type = 0; 
            }
        break;
        case STATE_TYPE:
            if (byte == ',' ) {
                ps->state = STATE_PAYLOAD;
                ps->msg_type[ps->index_type] = '\0';
                ps->index_payload = 0; 
            }else if (ps->index_type == 6){
                ps->state = STATE_DOLLAR;
                ps->index_type = 0;
            }else {
                ps->msg_type[ps->index_type] = byte; 
                ps->index_type++; 
            }
            break;
        case STATE_PAYLOAD:
            if(byte == '*') {
                ps->state = STATE_DOLLAR; 
                ps->msg_payload[ps->index_payload] = '\0';
                return NEW_MESSAGE;
            } else if (ps->index_payload == 100) { 
                ps->state = STATE_DOLLAR;
                ps->index_payload = 0;
            } else {
                ps->msg_payload[ps->index_payload] = byte; 
                ps->index_payload++; 
            }
        break;
    }
    return NO_MESSAGE;
}

int extract_integer(const char* str) {
    int i=0;
    int number = 0;
    int sign = 1;
    if (str[i] == '-') {
        sign = -1;
        i++;
    } else if (str[i] == '+') {
        sign = 1;
        i++;
    }
    while (str[i] != ',' && str[i] != '\0') {
        number *= 10; 
        number += str[i] - '0';
        i++;
    }
    return sign*number;
}
 
int next_value(const char* msg, int i) {
    while (msg[i] != ',' && msg[i] != '\0') {i++;}
    if (msg[i] == ',') {i++;}
    return i;
}
 
void parse_mcsen(const char* msg, threshold* t) {
    int i = 0;
    t->minth = extract_integer(msg);
    i = next_value(msg, i);
    t->maxth = extract_integer(msg + i);
    }

// Interrupt to write on circular buffer
void __attribute__((__interrupt__, __auto_psv__)) _U1RXInterrupt(){
    IFS0bits.U1RXIF = 0; // reset if flag when arrives at least 1 byte
    while(U1STAbits.URXDA){
        buf.cbuf[buf.write_p] = U1RXREG; // write
        buf.write_p ++; // increment the pointer
        buf.chars2read ++; // increment characters to be red
        if(buf.write_p == BUFFER_SIZE){
            buf.write_p = 0; // back to 0 if oversized
        }
    }
}

int main(void) {
    // set analog input to 0 
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0;
    
    // init interrupt RE8 + remap (INT1)
    TRISEbits.TRISE8 = 1; // RE8 input mode
    RPINR0bits.INT1R = 0x58; // remap INT1 to RE8 pin (RPI88)
    INTCON2bits.INT1EP = 0; // set edge
    IFS1bits.INT1IF = 0; // reset flag
    
    // init LEDA0
    TRISAbits.TRISA0 = 0;
    LATAbits.LATA0 = 0;
    
    init_pwm(10000); // to have 10KHz
    
    // init pwm_percentage
    pwm_percentage p;
    p.surge = 0.0;
    p.yaw_rate = 0.0;
    p.flag = 0;
    
    // init IR sensor
    float d;    
    TRISBbits.TRISB15 = 1; // sensor analog configuratiom AN15
    ANSELBbits.ANSB15 = 1; // set AN15 as analog pin
    TRISBbits.TRISB11 = 1; // battery sensing analog configuration AN11
    ANSELBbits.ANSB11 = 1; // set AN11 as analog pin
    
    // configure ADC
    AD1CON3bits.ADCS = 14; // set Tad
    AD1CON1bits.ASAM = 0; // select how sampling begins MANUAL
    AD1CON1bits.SSRC = 7; // select how sampling end AUTO
    AD1CON3bits.SAMC = 16; // sampling time
    AD1CON2bits.CHPS = 0; // how many channel = 1    
    AD1CON1bits.SIMSAM = 0; // sequential sampling
    
    // scan mode specific configuration
	AD1CON2bits.CSCNA = 1; // scan mode enabled
    AD1CSSLbits.CSS11 = 1;   // scan for AN11 battery
    AD1CSSLbits.CSS15 = 1;   // scan for AN15 ir sensor
	AD1CON2bits.SMPI = 1; // N-1 channels
    
    // IR distance sensor enable line
    TRISAbits.TRISA3 = 0; // set as output 
    LATAbits.LATA3 = 1; // enable
    AD1CON1bits.ADON = 1; // turn it on
    
    // UART
    RPOR0bits.RP64R = 0x01; // remap transmission
    RPINR18bits.U1RXR = 0x4b;  // remap reception
    U1BRG = 468; // (72000000) / (16 * 9600) - 1 = 467.75
    U1MODEbits.UARTEN = 1; //enable the UART mode
    U1STAbits.UTXEN = 1; //enable transmission
    U1STAbits.URXISEL = 1;
    // init var 
    buf.chars2read = 0;
    buf.read_p = 0;
    buf.write_p = 0;
        
    // init parser
    parser_state pstate;
    pstate.state = STATE_DOLLAR;
    pstate.index_type = 0;
    pstate.index_payload = 0;
    char byte;
    // init threshold
    threshold t;
    // set threshold at reliable value - can be changed by UART
    t.minth = 25;
    t.maxth = 70;
    
    // define counters 
    int blink_count = 0; // 1 Hz for always blinking led & sending volt 
    int lights_count = 0; // for all other lights
    int uart_count = 0; // 10Hz for sending distance & pwm 
    int turn_count = 0; // to keep turning for 0.5s
    
    // set all the lights
    TRISBbits.TRISB8 = 0; // left
    TRISFbits.TRISF1 = 0; // right
    TRISFbits.TRISF0 = 0; // brakes
    TRISGbits.TRISG1 = 0; // low intensity
    TRISAbits.TRISA7 = 0; // beam lights

    //set initial state as waiting
    state = 0;

    // enable interrupt
    INTCON2bits.GIE = 1; // enable global interrupt
    IEC1bits.INT1IE = 1; 
    IEC0bits.T2IE = 1; 
    IEC0bits.U1RXIE = 1; // enable interrupt for UART    

    //control loop 1KHz
    tmr_setup_period(TIMER1,1);
         
    while(1){
        // read distance
        d = analog_measure(IR_SENSOR);
        
        int ret = 0; 
        // read from cbuf and parse 
        if (buf.chars2read > 0){
            byte = buf.cbuf[buf.read_p];
            IEC0bits.U1RXIE = 0; // disable UART interrupt to avoid problems
            buf.chars2read --;
            IEC0bits.U1RXIE = 1; // enable back UART
            buf.read_p ++;
            if(buf.read_p == BUFFER_SIZE){
                buf.read_p = 0;
            }
            ret = parse_byte(&pstate, byte);
        }
        if (ret == NEW_MESSAGE) {
            if (strcmp(pstate.msg_type, "PCTH") == 0) {
                parse_mcsen(pstate.msg_payload, &t);
            }
        }
        
        // state 1 - Moving
        if(state){
            // to keep turning for 0.5s after seeing over max_th
            if(p.flag && d > t.maxth && turn_count < 500){
                turn_count++;
            }
            else{
                turn_count = 0;
                set_pwm_percentage(&p,&t,d);
            }
            // effective move
            motor_ctrl(&p);
            // lights settings according to Table2
            if(p.yaw_rate > 15){
                LATBbits.LATB8 = 0;
                if( lights_count == 1000){
                    LATFbits.LATF1 = !LATFbits.LATF1;
                    lights_count = 0;
                }
            lights_count++;
            }
            else{
                LATBbits.LATB8 = 0;
                LATFbits.LATF1 = 0;
                lights_count = 0;               
            }
            if(p.surge > 50){
                LATAbits.LATA7 = 1;
                LATGbits.LATG1 = 0;
                LATFbits.LATF0 = 0;
            }
            else{
                LATAbits.LATA7 = 0;
                LATGbits.LATG1 = 1;
                LATFbits.LATF0 = 1;
            }
        }
        // state 0 - Waiting
        else{
            // stop moving
            stop_pwm();
            // lights settings according to Table2
            LATAbits.LATA7 = 0;
            LATGbits.LATG1 = 0;
            LATFbits.LATF0 = 0;
            if( lights_count == 1000){
                LATBbits.LATB8 = !LATBbits.LATB8;
                LATFbits.LATF1 = !LATFbits.LATF1;
                lights_count = 0;
            }
            lights_count++;
        }
        
        // check 1Hz counter for blinking & UART transmission
        if( blink_count == 1000){
            LATAbits.LATA0 = !LATAbits.LATA0;
            transmit_uart(VOLT);
            blink_count = 0;
        }
        // check 10Hz counter for UART transmission
        if( uart_count == 10000){
            transmit_uart(DIST);
            transmit_uart(PWM);
            uart_count = 0;
        }
        
        blink_count++;
        uart_count++;
        
        // timer
        tmr_wait_period(TIMER1);    
    }
}