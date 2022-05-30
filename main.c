    /*
 * File:   PWM.c
 * Author: Cristian Catú, Jose Gonzales, Frank Martínez
 * LAB 9
 *
 * Created on 27 april 2022, 19:04
 */

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

#define _XTAL_FREQ 4000000
#define IN_MIN 0           
#define IN_MAX 255        
#define OUT_MIN 15       
#define OUT_MAX 75


/*------------------------------------------------------------------------------
 * VARIABLES
 ------------------------------------------------------------------------------*/
unsigned short CCPR = 0;        // Variable para almacenar ancho de pulso al hacer la interpolación lineal
uint8_t pot2 = 0;
uint8_t pot3 = 0;
uint8_t bandera = 0;
uint8_t bit_envio = 0;
uint8_t entrada = 0;
uint8_t CCPR_3 = 0;
uint8_t CCPR_4 = 0;

/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES
 ------------------------------------------------------------------------------*/
void setup(void);
unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max,
            unsigned short out_min, unsigned short out_max);
/*------------------------------------------------------------------------------
 * INTERRUPCIONES
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){
    if(PIR1bits.RCIF){
        entrada = RCREG;
        bit_envio = 0b00000001 & entrada;
        if (bit_envio == 1){
            CCPR_3 = map(entrada, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
            CCPR2L = (uint8_t)(CCPR_3>>2);    // Guardamos los 8 bits mas significativos en CPR1L
            CCP2CONbits.DC2B0 = CCPR_3 & 0b1; // Guardamos los 2 bits menos significativos en DC1B
            CCP2CONbits.DC2B1 = CCPR_3>>1 & 0b1;
            
        }
        else if(bit_envio == 0){
            CCPR_4 = map(entrada, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
            CCPR1L = (uint8_t)(CCPR_4>>2);    // Guardamos los 8 bits mas significativos en CPR1L
            CCP1CONbits.DC1B = CCPR_4 & 0b11; // Guardamos los 2 bits menos significativos en DC1B
        }
    }
    
    return;
}

/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/
void main(void) {
    setup();
    while(1){
    }
    return;
}

/*------------------------------------------------------------------------------
 * CONFIGURACION
 ------------------------------------------------------------------------------*/
void setup(void){
    ANSEL = 0xF;                // AN0, AN1 y AN2 como entrada analógica
    ANSELH = 0;                 // I/O digitales
    TRISA = 0xF;                // AN0, AN1 y AN2 como entrada
    PORTA = 0;
    TRISB = 0;
    PORTB = 0;
    TRISD = 0;
    PORTD = 0;

    OSCCONbits.IRCF = 0b011;    // 500MHz
    OSCCONbits.SCS = 1;         // Oscilador interno
    
    // Configuraciones de comunicacion serial
    //SYNC = 0, BRGH = 1, BRG16 = 1, SPBRG=25 <- Valores de tabla 12-5
    TXSTAbits.SYNC = 0;         // Comunicación ascincrona (full-duplex)
    TXSTAbits.BRGH = 1;         // Baud rate de alta velocidad 
    BAUDCTLbits.BRG16 = 1;      // 16-bits para generar el baud rate
    
    SPBRG = 12;
    SPBRGH = 0;                 // Baud rate ~9600, error -> 0.16%
    
    RCSTAbits.SPEN = 1;         // Habilitamos comunicación
    TXSTAbits.TX9 = 0;          // Utilizamos solo 8 bits
    TXSTAbits.TXEN = 1;         // Habilitamos transmisor
    RCSTAbits.CREN = 1;         // Habilitamos receptor
    
    // Configuración PWM
    TRISCbits.TRISC2 = 1;       // Deshabilitamos salida de CCP1
    PR2 = 155;                  // periodo de 2ms

    // Configuración CCP
    CCP1CON = 0;                // Apagamos CCP1
    CCP1CONbits.P1M = 0;        // Modo single output
    CCP1CONbits.CCP1M = 0b1100; // PWM

    CCPR1L = 30>>2;
    CCP1CONbits.DC1B = 30 & 0b11;    // 0.5ms ancho de pulso / 25% ciclo de trabajo

    TRISCbits.TRISC1 = 1;       // Deshabilitamos salida de CCP1
    CCP2CON = 0;                // Apagamos CCP1
    CCP2CONbits.CCP2M = 0b1100; // PWM
    CCPR2L = 30>>2;
    CCP2CONbits.DC2B0 = 30 & 0b1; // Guardamos los 2 bits menos significativos en DC2B
    CCP2CONbits.DC2B1 = 30>>1 & 0b1;

    PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2
    T2CONbits.T2CKPS = 0b11;    // prescaler 1:16
    T2CONbits.TMR2ON = 1;       // Encendemos TMR2
    while(!PIR1bits.TMR2IF);    // Esperar un cliclo del TMR2
    PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2 nuevamente

    TRISCbits.TRISC2 = 0;       // Habilitamos salida de PWM
    TRISCbits.TRISC1 = 0;       // Habilitamos salida de PWM
    PORTC = 0;

    // Configuracion interrupciones
    INTCONbits.PEIE = 1;        // Habilitamos int. de perifericos
    INTCONbits.GIE = 1;         // Habilitamos int. globales
    PIE1bits.RCIE = 1;


}
// función para el mapeo
unsigned short map(uint8_t x, uint8_t x0, uint8_t x1,
            unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}