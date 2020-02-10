/* 
 * File:   try2.c
 * Author: amrsh
 *
 * Created on January 20, 2020, 12:44 PM
 */

// PIC16F18877 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/
/************************************************ Components to Buy **************************************************************/
/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

/*LG HG2 BATTERIES*/ /**** Buy enough to use for this project and the electric Long board Project*/
/*H-BRIDGE MOTOR*/
/*SERIAL COMMUNICATION Circuit*/ /* This system should be 2.4GHz and capable of far transmission */
/*Battery management system*/
/*See if the motors you currently own suffice or get 12v Motors with high RPM*/

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/
/************************************************ Additional Add ons *************************************************************/
/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

/* Live feed, This is where you will be able to see your car on your phone */
/* GPS automation, using GPS and pinpoints, the car makes its way to those points while avoiding obstacles(LADAR SENSORS?) */

/* Digital Speed display*/


/*********************************************************************************************************************************/
/*********************************************************************************************************************************/
/************************************************** Configuration Bits ***********************************************************/
/*********************************************************************************************************************************/
/*********************************************************************************************************************************/


#pragma config FEXTOSC = OFF    // External Oscillator mode selection bits (Oscillator not enabled)

//#pragma config RSTOSC = HFINTPLL// Power-up default value for COSC bits (HFINTOSC with 2x PLL, with OSCFRQ = 16 MHz and CDIV = 1:1 (FOSC = 32 MHz))
#pragma config RSTOSC = HFINT32// Power-up default value for COSC bits (HFINTOSC with OSCFRQ= 32 MHz and CDIV = 1:1)
        
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; i/o or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (FSCM timer disabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR pin is Master Clear function)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config LPBOREN = OFF    // Low-Power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = OFF      // Brown-out reset enable bits (Brown-out reset disabled)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (VBOR) set to 1.9V on LF, and 2.45V on F Devices)
#pragma config ZCD = OFF        // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR.)
#pragma config PPS1WAY = ON     // Peripheral Pin Select one-way control (The PPSLOCK bit can be cleared and set only once in software)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a reset)

// CONFIG3
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled, SWDTEN is ignored)
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4
#pragma config WRT = OFF        // UserNVM self-write protection bits (Write protection off)
#pragma config SCANE = available// Scanner Enable bit (Scanner module is available for use)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/Vpp pin function is MCLR.)

// CONFIG5
#pragma config CP = OFF         // UserNVM Program memory code protection bit (Program Memory code protection disabled)
#pragma config CPD = OFF        // DataNVM code protection bit (Data EEPROM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.


/*********************************************************************************************************************************/
/*********************************************************************************************************************************/
/************************************************** CODE START *******************************************************************/
/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>

//#define _XTAL_FREQ 32000000


void speeds(void);
void delayby_65ms(void);
void delayby_130ms(void);
void delayby_260ms(void);
void delayby_500ms(void);
void delayby_s(void);
int delayms=0;
int delays=0;
int delay=0;
void __interrupt(high_priority) tcInt(void);
int i=0;
int counter=0;
int ct2=0;
int devider=0;
//Variables for Button Interrupt
int speed=0;
//Variables for PWM
int channel1=0;
int channel2=0;
int horiz=0;
int vert=0;
//This will be used for brushed motors
int M1=0;
int M2=0;
//This will be used for brushless motors
int M1HOR=0;
int M2HOR=0;
int M1VERT=0;
int M2VERT=0;
unsigned char M1T=0;
int M2T=0;
//Variables for Serial Communication
int S1=0;
int S2=0;
int read;

//Defining clock and TMRPrescale
//#define _XTAL_FREQ 16000000
#define _XTAL_FREQ 4000000
#define TMR2PRESCALE 128



long PWM_freq = 50;
/*******************************************************************************************************************************************************/
/**************************************************************PULSE WIDTH MODULATION*******************************************************************/
/*******************************************************************************************************************************************************/
PWM_Initialize()
{
  PR2 = (_XTAL_FREQ/(PWM_freq*4*TMR2PRESCALE)) - 1; //Setting the PR2 formulae using Datasheet // Makes the PWM work in 5KHZ
    // Set up CCP1
    RC2PPS=0x09;//this sets it to RC2
    TRISC2=0;//This sets port c2 as an output
    CCP1CONbits.EN=1;
    CCP1CONbits.FMT=0;
    CCP1CONbits.MODE=0b1111;
    CCPR1L=0;
    CCPR1H=0;
    CCPTMRS0bits.C1TSEL=0b01;//choose TMR2(PWM)
    //CCPTMRS1bits.P6TSEL=0b01;//choose TMR2(PWM)
    
    //Set up TMR2
    PIR4bits.TMR2IF=0;// This clears the Timer 2 interrupt flag
    PIE4bits.TMR2IE=1;// This enables timer 2 interrupts
    T2CLKCONbits.CS=0b0001; T2CONbits.CKPS = 0b111; TMR2ON = 1; //Configure the Timer module
    //TRISB0 = 0; // make port pin on b as output
    
    // set up CCP2
    RC1PPS=0x0A;//this sets it to RC2
    TRISC1=0;//This sets port c1 as an output
    CCP2CONbits.EN=1;
    CCP2CONbits.FMT=0;
    CCP2CONbits.MODE=0b1111;
    CCPR2L=0;
    CCPR2H=0;
    CCPTMRS0bits.C2TSEL=0b11;//choose TMR6(PWM)
    //CCPTMRS1bits.P6TSEL=0b01;//choose TMR2(PWM)
    
    //Set up TMR6
    PIR4bits.TMR6IF=0;// This clears the Timer 6 interrupt flag
    PIE4bits.TMR6IE=1;// This enables timer 6 interrupts
    T6CLKCONbits.CS=0b0001; T6CONbits.CKPS = 0b111; TMR6ON = 1; //Configure the Timer module
    //TRISB0 = 0; // make port pin on b as output
    
    //set up CCP4
    RB0PPS=0x0C;
    TRISB0=0;
    CCP4CONbits.EN=1;
    CCP4CONbits.FMT=0;
    CCP4CONbits.MODE=0b1111;
    CCPR4L=0;
    CCPR4H=0;
    CCPTMRS0bits.C4TSEL=0b11;//choose TMR6(PWM)
    
    //set up CCP3
    RB5PPS=0x0B;
    TRISB5=0;
    CCP3CONbits.EN=1;
    CCP3CONbits.FMT=0;
    CCP3CONbits.MODE=0b1111;
    CCPR3L=0;
    CCPR3H=0;
    CCPTMRS0bits.C3TSEL=0b11;//choose TMR6(PWM)
}

PWM1_Duty(unsigned int horizontal,unsigned int vertical)// This is for Brushed Motors
{     //red
    M1=0;
    M2=0;

    /********************************************Horizontal Control*********************************************************************************/
    if(horizontal<1025 && horizontal>500)
  {
        //duty=duty-510;
        horiz = (((float)horizontal-500)/1023)*(_XTAL_FREQ/(PWM_freq*TMR2PRESCALE)); // On reducing //duty = (((float)duty/1023)*(1/PWM_freq)) / ((1/_XTAL_FREQ) * TMR2PRESCALE);
        M1=M1-horiz;
        M2=M2+horiz;
        
        /*
        CCPR1H = horiz>>8; //Store the first 2 MSB bits
        CCPR1L = horiz;// Store the remining 8 bit
        CCPR4H = 0; //Store the first 2 MSB bits
        CCPR4L = 0;// Store the remining 8 bit
        */
  }
      else if(horizontal<500 && horizontal>=0){
        horiz = (((((float)horizontal)-500)*-1)/1023)*(_XTAL_FREQ/(PWM_freq*TMR2PRESCALE)); // On reducing //duty = (((float)duty/1023)*(1/PWM_freq)) / ((1/_XTAL_FREQ) * TMR2PRESCALE);

        M1=M1+horiz;
        M2=M2-horiz;
        /*
        CCPR4H = horiz>>8; //Store the first 2 MSB bits
        CCPR4L = horiz;// Store the remining 8 bit
        CCPR1H = 0; //Store the first 2 MSB bits
        CCPR1L = 0;// Store the remining 8 bit
        */ 
      }
      else{
          horiz=0;
          M1=M1-horiz;
          M2=M2+horiz;
          /*
          CCPR2H = horiz>>8; //Store the first 2 MSB bits
          CCPR2L = horiz;// Store the remining 8 bit
          CCPR4H = horiz>>8; //Store the first 2 MSB bits
          CCPR4L = horiz;// Store the remining 8 bit
          */ 
      }
        /*********************************************Vertical Control*********************************************************************************/
      if(vertical<1025&&vertical>500)
  {
        //duty=duty-510;
        vert = (((float)vertical-500)/1023)*(_XTAL_FREQ/(PWM_freq*TMR2PRESCALE)); // On reducing //duty = (((float)duty/1023)*(1/PWM_freq)) / ((1/_XTAL_FREQ) * TMR2PRESCALE);
        M1=M1+vert;
        M2=M2+vert;
        
        if(M1<0){
            M1=0;
        }
        if(M1>1023){
            M1=1023;
        }
        if( M2<0){
            M2=0;
        }
        if(M2>1023){
            M2=1023;
        }
        /*Motor 1*/
        CCPR2H = M1>>8; //Store the first 2 MSB bits
        CCPR2L = M1;// Store the remining 8 bit
        /*Motor 2*/
        CCPR1H = M2>>8; //Store the first 2 MSB bits
        CCPR1L = M2;// Store the remining 8 bit
        /*back polarity*/
        
        
        /*Back of Motor 1*/
        CCPR4H = 0; //Store the first 2 MSB bits
        CCPR4L = 0;// Store the remining 8 bit
        
        /*Back of Motor 2*/
        CCPR3H = 0; //Store the first 2 MSB bits
        CCPR3L = 0;// Store the remining 8 bit
  }
      else if(vertical<500 && vertical>=0){
        vert = (((((float)vertical)-500)*-1)/1023)*(_XTAL_FREQ/(PWM_freq*TMR2PRESCALE)); // On reducing //duty = (((float)duty/1023)*(1/PWM_freq)) / ((1/_XTAL_FREQ) * TMR2PRESCALE);
        M1=M1+vert;
        M2=M2+vert;
        if(M1<0){
            M1=0;
        }
        if(M1>1023){
            M1=1023;
        }
        if( M2<0){
            M2=0;
        }
        if(M2>1023){
            M2=1023;
        }
        /*Motor 1*/
        CCPR2H = 0; //Store the first 2 MSB bits
        CCPR2L = 0;// Store the remining 8 bit
        /*Motor 2*/
        CCPR1H = 0; //Store the first 2 MSB bits
        CCPR1L = 0;// Store the remining 8 bit
        
        
        /*back polarity*/
        
        /*Back of Motor 1*/
        CCPR4H = M1>>8; //Store the first 2 MSB bits
        CCPR4L = M1;// Store the remining 8 bit
        
        /*Back of Motor 2*/
        CCPR3H = M2>>8; //Store the first 2 MSB bits
        CCPR3L = M2;// Store the remining 8 bit
      }
      else{
            vert=0;
            M1 = M1+ vert;
            M2 = M2+ vert;
        if(M1<0){
            M1=0;
        }
        if(M1>1023){
            M1=1023;
        }
        if( M2<0){
            M2=0;
        }
        if(M2>1023){
            M2=1023;
        }
            
            /*Motor 1*/
            CCPR2H = 0; //Store the first 2 MSB bits
            CCPR2L = 0; // Store the remining 8 bit
            /*Motor 2*/
            CCPR1H = 0; //Store the first 2 MSB bits
            CCPR1L = 0; // Store the remining 8 bit
            /*back polarity*/
            
            /*Back of Motor 1*/
            CCPR4H = 0; //Store the first 2 MSB bits
            CCPR4L = 0;// Store the remining 8 bit

            /*Back of Motor 2*/
            CCPR3H = 0; //Store the first 2 MSB bits
            CCPR3L = 0;// Store the remining 8 bit
      }
    
    
}
PWM2_Duty()// This is for Brushless Motors
{     //red
    

    /********************************************Horizontal Control*********************************************************************************/

    
        /*Motor 1*/
        CCPR2H = M1T>>8; //Store the first 2 MSB bits
        CCPR2L = M1T;// Store the remining 8 bit
    
        //CCPR2H = channel1>>8; //Store the first 2 MSB bits
        //CCPR2L = channel1;// Store the remining 8 bit
        /*Motor 2*/
        CCPR1H = channel2>>8; //Store the first 2 MSB bits
        CCPR1L = channel2;// Store the remining 8 bit
        /*back polarity*/
    
    
    
}
/*******************************************************Brushless Motor ESC Mapping********************************************************************/

/*******************************************************************************************************************************************************/
/**************************************************************ADC**************************************************************************************/
/*******************************************************************************************************************************************************/
AD_Initialize(){
    ANSB7=0b00011000;//Sets RB4 and RB3 as analog
    TRISB7=0b00011000;// Sets RB4 and RB3 as input
    //ADCLK=0b00011111;//FOSC=16MHz, ADC clock source=FOSC/16= 1us
    ADCLK=0b00000001;//FOSC=4MHz, ADC clock source= FOSC/4= 1us
    ADREF=0x00;//VREF-=VSS VREFF+=VDD
    ADPCH=0b001100;//ANB4
    //ADPCH=0b001011//ANB3
    //ADCON0=0x84;//ADFRM0=1 RIGHT justified
    ADCON0=0x84;
    //ADCON0bits.ADCONT=0;
    //GO_nDONE = 1;
    ADGO=1;
    //ADSOI=1;
    //ADIE=1;//enable ADC Interrupt
    //ADIF=0;//Clear Interrupt flag
    
    
}

/*******************************************************************************************************************************************************/
/**************************************************************Serial Communication*******************************************************************/
/*******************************************************************************************************************************************************/
Serial_comm_init(){
    ANSC4=0;
    TRISC4=1;//SET RC4 AS INPUT check
    SSP1DATPPS=0x14;//Choose RC4 as SDI check
    ANSC3=0;
    TRISC3=1;//SET RC3 AS INPUT check
    SSP1CLKPPS=0x13;//RC3=SCK1/SCL1 check
    
    ANSC5=0;
    TRISC5=0;//SET RC5 AS OUTPUT
    RC5PPS=0x15;//RC4=SD01/SDA1 
    
    ANSA5=0;
    TRISA5=1;
    //SSP1SSPPS=0x05;
    
    SSP1STAT=0b00000000;//SMP=sample at the middle of data output
                        //CKE=from low to high
                        //BF=0; receive not complete
    //use when in slave mode
    SSP1CON1=0b00110101;//WCOL=0 NO COLLISION
                        //SSPOV=0 NO OVERFLOW
                        //SSPM=0100 slave mode with SS ENABLED 
    /*
    SSP1CON1=0b00110000;//WCOL=0 NO COLLISION
                        //SSPOV=0 NO OVERFLOW
                        //SSPM=0000 FOSC/4
    */
    //SSP1IE=1;
    //SSP1IF=0;
}
/*
SPI_READ(){
    
    while ( !SSP1STATbits.BF );
    
    M1T=SSP1IF;
    
    return (M1T);
  
}
*/

unsigned char wait_SPI(void){
    while(!SSP1STATbits.BF);
    return SSP1BUF;
}
void sendSPI(unsigned int byte){
    
    SSP1BUF = byte;
    wait_SPI();

/*RD2=0;
while(!SSP1IF) continue;
RD2=1;
read=SSP1BUF;
        
return(SSP1BUF);
*/
}

unsigned char receiveSPI(){
    unsigned char data;
    SSP1STATbits.BF=0;
    SSP1BUF=0;
    data=wait_SPI();
    return (data);
}

/*******************************************************************************************************************************************************/
/**************************************************************Main Code********************************************************************************/
/*******************************************************************************************************************************************************/
int main(int argc, char** argv) {
    /*Set up Interrupts*/
    INTCON=0xFF;// Interrupts are set on rising edge( active high)
    PIR0bits.TMR0IF=0;// This clears the Timer 0 interrupt flag
    //PIE0bits.TMR0IE=1;// This enables timer interrupts
    
    /*Set up Oscillator*/
    OSCENbits.HFOEN=0;// enables setting the internal oscillator from pins other than OSCFRQ
    //OSCFRQbits.HFFRQ=0b110; // This will set the oscillator to 32MHZ if RSTOSC = HFINT32
    OSCFRQbits.HFFRQ=0b101; // This will set the oscillator to 16MHZ if RSTOSC = HFINT32
    
    /*Set up Timer 4 as I am alive clock*/
    PIR4bits.TMR4IF=0;// This clears the Timer 2 interrupt flag
    PIE4bits.TMR4IE=1;// This enables timer 2 interrupts
    T4CLKCONbits.T4CS=0b00000001;// Input clock to timer 2 is FOSC/4
    T4CONbits.T4ON=1;
    T4CONbits.T4CKPS =0b111;// Pre-scalar =128
    T4CONbits.T4OUTPS=0b0000;
    T4HLTbits.T4PSYNC=0;
    //T2CONbits=0xF0;// Pre-scalar =128
    //Given that timer 2 is 8 bits, 1/(31.25KHz/(128*256))= 1.04s
    
    /*Set up RE0 as external interrupt button (active high)*/
    //INTPPS=0x20;
    PIE0bits.INTE=1;//unable external interrupt
    PIR0bits.INTF=0;//clear interrupt flag
    

    
    PWM_Initialize();
    //AD_Initialize(); We are in Slave device
    
    
    ANSELA = 0x00; //This sets all the pins to digital I/O pins. If it was 1, then it would be analog pin
    TRISA = 0b11110101; // This sets the digital I/O pins as outputs or inputs. 1 is input, 0 is output
    unsigned char d;
    Serial_comm_init();
    //SSP1BUF=M2T;
    while(1){
        //SPI_READ();
        d=receiveSPI();
        M1T=d;
        PWM2_Duty();
      
        }
        


    
    
    return 0;
}
void speeds(void){
    if (speed<4){
        ++speed;
    }
    else{
        speed=0;
    }
}
void delayby_65ms(void){
    PIE0bits.TMR0IE=1;// This enables timer interrupts
    delay=0;
    //delayms=k;
    T0CON0=0b10000000; //T0EN=1 -> enable, T016BIT=0 -> 8-bit timer
    //T0CON1=0b01001010; //pre scalar =1024 if FOSC=16MHz
    T0CON1=0b01001000; //pre scalar =256 if FOSC=4MHz
    //T0CON1=0b01001011; //pre scalar =2048 if FOSC=32MHz
    while (PIR0bits.TMR0IF!=1){;}
}
void delayby_130ms(void){ //uses timer0 to create a 1ms delay
    PIE0bits.TMR0IE=1;// This enables timer interrupts
    delay=0;
    //delayms=k;
    T0CON0=0b10000000; //T0EN=1 -> enable, T016BIT=0 -> 8-bit timer
    //T0CON1=0b01001011; //pre scalar =2048 if FOSC=16MHz
    T0CON1=0b01001001; //pre scalar =512 if FOSC=4MHz
    //T0CON1=0b01001100; //pre scalar =4096 if FOSC=32MHz
    while (PIR0bits.TMR0IF!=1){;}
}
void delayby_260ms(void){ //uses timer0 to create a 1ms delay
    PIE0bits.TMR0IE=1;// This enables timer interrupts
    delay=0;
    //delayms=k;
    T0CON0=0b10000000; //T0EN=1 -> enable, T016BIT=0 -> 8-bit timer
    //T0CON1=0b01001100; //pre scalar =4096 if FOSC=16MHz
    T0CON1=0b01001010; //pre scalar =2048 if FOSC=4MHz
    //T0CON1=0b01001101; //pre scalar =8192 if FOSC=32MHz
    while (PIR0bits.TMR0IF!=1){;}
}
void delayby_500ms(void){ //uses timer0 to create a 1ms delay
    PIE0bits.TMR0IE=1;// This enables timer interrupts
    delay=0;
    //delayms=k;
    T0CON0=0b10000000; //T0EN=1 -> enable, T016BIT=0 -> 8-bit timer
    //T0CON1=0b01001101; //pre scalar =8192 if FOSC=16MHz
    T0CON1=0b01001011; //pre scalar =2048 if FOSC=4MHz
    //T0CON1=0b01001110;//pre scalar =16384 if FOSC=32MHz
    while (PIR0bits.TMR0IF!=1){;}
 
}
void delayby_s(void){ //uses timer0 to create a 1 delay
    PIE0bits.TMR0IE=1;// This enables timer interrupts
    delay=1;
    //delays=k;
    T0CON0=0b10000000; //T0EN=1 -> enable, T016BIT=0 -> 8-bit timer
    T0CON1=0b01001110;//pre scalar =16384 if FOSC=16MHz
    T0CON1=0b01001100; //pre scalar =4096 if FOSC=4MHz
    //T0CON1=0b01001111;//pre scalar =32768 if FOSC=32MHz
    while (PIR0bits.TMR0IF!=1){;}
    
}

void __interrupt(high_priority) tcInt(void){
    if (ADIE && ADIF){
        ADIF=0;
        
        if(ADPCH==0b00001100){//ANB4
            channel1=((ADRESH<<8)+ADRESL);
            ADPCH=0b00001011;//ANB3
            //ADGO=1;
        }
        else{
            channel2=((ADRESH<<8)+ADRESL);
            ADPCH=0b00001100;
            
        }
        ADGO=1;
    }
    else if (TMR4IE && TMR4IF){
        PIR4bits.TMR4IF=0;
        if (ct2==122){//if FOSC=32MHZ then change to ct2=245
            ct2=0;
            if ( PORTAbits.RA1 ==1){
            PORTAbits.RA1=0 ;
            }
            else{
            PORTAbits.RA1 =1;
            } 
        }
        else{++ct2;}

    }
    /*else if (SSP1IE==1 && SSP1IF==1){
        //read=SSP1BUF;
        if (S1==1){
            channel1=SSP1BUF;
            S1=0;
            S2=1;
        }
        else{
            channel2=SSP1BUF;
            S2=0;
            S1=1;
        }
    }*/
    else if (PIE0bits.INTE==1 && PIR0bits.INTF==1){
        delayby_65ms();
        PIR0bits.INTF=0;
        speeds();
        
    }
    else if (TMR1IE && TMR1IF){
        TMR1IF=0;
    }
    else if (TMR0IE && TMR0IF){
        PIR0bits.TMR0IF=0;
        /*if ( PORTAbits.RA3 ==1){
            PORTAbits.RA3=0 ;
            }
            else{
            PORTAbits.RA3 =1;
            }
        PIE0bits.TMR0IE=0;
        */
    }
    else if (TMR2IE && TMR2IF){
        TMR2IF=0;
    }
    else if (TMR6IE && TMR6IF){
        TMR6IF=0;
        /*if (speed==4){
            speed=0;
        }
        else{
            speed++;
        }*/
    }

    
}

/*1/22/2020*/
    /****************************************************************************************************/
    /****************************************************************************************************/
    /* The next steps to this would be to see how to control the CLK and to place a timer based delay****/
    /****************************************************************************************************/
    /****************************************************************************************************/    


    /*1/22/2020*/
    /*Lets create a button based interrupt where if it is pressed then a certain code like turning on the LED,
     * This proof of concept could be used to set the speed control. A global variable will equal the number of times the 
     * button is pressed. This would then go to different if statements that would control the PWM based on the number pressed. */


    /*1/23/2020*/
    /*Building on the above statement, The conditional statement for motor power should be written. A rough idea of the desired 
     * output is to provide power to an H-bridge circuit based on the PWM level. If the speed button has been pressed once, then
     it will be 1/4 the max speed, while the 4th press would be the max speed. (A digital display would be useful 
     to show the current speed level.*/


    /*1/24/2020*/
    /*Lets also create a serial communication definition. This will be used with the 2.4GHz module to transmit data. A separate 
     script for the receiving controller would need to be written.*/

    /************************************************************************************************/
    /************************************************************************************************/
    /******************************** Project Week 2 plan *******************************************/
    /************************************************************************************************/
    /************************************************************************************************/


    //return (EXIT_SUCCESS);
/*********************************************************************************************************************************/
/*********************************************************************************************************************************/
/************************************************** CODE End *******************************************************************/
/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

    /*    if( OSCSTATbits.HFOR==1){
        while (1) {
            PORTAbits.RA1 = 1;
            delayby_s(1);
            PORTAbits.RA1 = 0;
            delayby_s(1);
            counter++;
            printf('counter %d',counter);
        }
    }
   */

    /*
    if( OSCSTATbits.HFOR=1){
        while (1) {
            PORTAbits.RA1 = 1;
            __delay_ms(500);
            PORTAbits.RA1 = 0;
            __delay_ms(500);
        }
    }
    */
    /*
    if( OSCSTATbits.HFOR=1){
        while (1) {
            PORTAbits.RA1 = 1;
            delayby_ms(500);
            PORTAbits.RA1 = 0;
            delayby_ms(500);
        }
    }
    */
/*
PWM2_Duty(unsigned int duti)
{     //blue
      if(duti<1025&&duti>500)
  {
        //duty=duty-510;
        duti = (((float)duti-500)/600)*(_XTAL_FREQ/(PWM_freq*TMR2PRESCALE)); // On reducing //duty = (((float)duty/1023)*(1/PWM_freq)) / ((1/_XTAL_FREQ) * TMR2PRESCALE);

        CCPR2H = duti>>8; //Store the first 2 MSB bits
        CCPR2L = duti;// Store the remining 8 bit
        CCPR4H = 0; //Store the first 2 MSB bits
        CCPR4L = 0;// Store the remining 8 bit
  }
      else if(duti<460){
        duti = (((((float)duti)-460)*-1)/600)*(_XTAL_FREQ/(PWM_freq*TMR2PRESCALE)); // On reducing //duty = (((float)duty/1023)*(1/PWM_freq)) / ((1/_XTAL_FREQ) * TMR2PRESCALE);

        CCPR4H = duti>>8; //Store the first 2 MSB bits
        CCPR4L = duti;// Store the remining 8 bit
        CCPR2H = 0; //Store the first 2 MSB bits
        CCPR2L = 0;// Store the remining 8 bit
      }
      else{
          duti=0;
          CCPR2H = duti>>8; //Store the first 2 MSB bits
          CCPR2L = duti;// Store the remining 8 bit
          CCPR4H = duti>>8; //Store the first 2 MSB bits
          CCPR4L = duti;// Store the remining 8 bit
      }
}
*/