// Archivo:     Proyecto2.c
// Dispositivo:	PIC16F887
// Autor:       Diana Alvarado
// Compilador:	XC8
//
// Programa:	Movimiento Servo Motores
// Hardware:	pots PORTA, motores servo PORTC
// 
// Creado:      27 abr, 2021
// ?litma modificaci?n: 05 junio, 2021

#include <xc.h>
#include <stdint.h>
#include <pic16f887.h>

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


#define _XTAL_FREQ 8000000
//**
//                              VARIABLES
//**

unsigned char   POT0;
unsigned char   POT1;
unsigned char   POT2;
unsigned char   POT3;
unsigned char   ItPOT0;
unsigned char   ItPOT1;
unsigned char   ItPOT2;
unsigned char   ItPOT3;
unsigned char   ENTM0Pot0;
unsigned char   ENTM0Pot1;
unsigned char   ENTM0Pot2;
unsigned char   ENTM0Pot3;
unsigned char   ConTMR1;
unsigned char   ConTEEPROM = 0;
unsigned char   ADD_POT0[5] = {0x04,0x05,0x06,0x07,0x08};
unsigned char   ADD_POT1[5] = {0x09,0x0A,0x0B,0x0C,0x0D};
unsigned char   ADD_POT2[5] = {0x0E,0x0F,0x10,0x11,0x12};
unsigned char   ADD_POT3[5] = {0x13,0x14,0x15,0x16,0x17};
unsigned char   RB0_old = 0;
unsigned char   RB1_old = 0;
unsigned char   RB7_old = 0;
unsigned char   Reading = 0;
unsigned char   WriteDone = 0;

unsigned char   PCenable = 0;

uint8_t opcion_REG;
uint8_t opcion_REG_temp;
uint8_t MAP_TEMP1;
uint8_t MAP_TEMP2;
uint8_t MAP_TEMP3;
uint8_t MAP_TEMP4;

/*char Save [] = "Guardado";

char SaveDone [] = "Guardado listo --> Ya puede leer";

char Read1 [] = "Leyendo 1/5";
char Read2 [] = "Leyendo 2/5";
char Read3 [] = "Leyendo 3/5";
char Read4 [] = "Leyendo 4/5";
char Read5 [] = "Leyendo 5/5";
char ReadDone [] = "Lectura Terminada";*/

//**
//                       PROTOTIPO DE FUNCION
//**
void UART(void);

//**
//                              FUNCIONES
//**
void UART(void){
    opcion_REG = RCREG;
    PORTC = opcion_REG;
    switch(opcion_REG){
        case (1): 
            opcion_REG_temp = 1;
            break;
        case (2):
            opcion_REG_temp = 2;
            break;
        case (3):
            opcion_REG_temp = 3;
            break;
        case (4):
            opcion_REG_temp = 4;
            break;
        }
    
    if (opcion_REG_temp == 1){
        while(!PIR1bits.RCIF){};
        MAP_TEMP1 = RCREG;
        POT0 = MAP_TEMP1;
        opcion_REG = 0;
        }
    if (opcion_REG_temp == 2){
        while(!PIR1bits.RCIF){};
        MAP_TEMP2 = RCREG;
        POT1 = MAP_TEMP2;
        opcion_REG = 0;
        }
    if (opcion_REG_temp == 3){
        while(!PIR1bits.RCIF){};
        MAP_TEMP3 = RCREG;
        POT2 = MAP_TEMP3;
        opcion_REG = 0;
    }
    if (opcion_REG_temp == 4){
        while(!PIR1bits.RCIF){};
        MAP_TEMP4 = RCREG;
        POT3 = MAP_TEMP4;
        opcion_REG = 0;
        }
    while(!PIR1bits.RCIF){};

}

void Envio_caracter (char a)
{
    while (TXSTAbits.TRMT == 0){
       
    }
    if (PIR1bits.TXIF)
        {
            TXREG = a;
        }  
    return;
}

void Envio_Cadena (char* cadena)
{
    while (*cadena != 0)
    {
      Envio_caracter(*cadena);
      cadena++;
    }
    if (PIR1bits.TXIF)
        {
            TXREG = 13;
        }
    return;
}
unsigned char NumIteraciones (unsigned char ValorPot)
{
    if (ValorPot <19)
    {
        return 2;
    }
    else if (ValorPot < 40 && ValorPot > 18)
    {
        return 2;
    }
    else if (ValorPot < 61 && ValorPot > 39)
    {
        return 3;
    }
    else if (ValorPot < 82 && ValorPot > 60)
    {
        return 4;
    }
    else if (ValorPot < 103 && ValorPot > 81)
    {
        return 5;
    }
    else if (ValorPot < 131 && ValorPot > 102)
    {
        return 6;
    }
    else if (ValorPot < 152 && ValorPot > 130)
    {
        return 7;
    }
    else if (ValorPot < 173 && ValorPot > 151)
    {
        return 8;
    }
    else if (ValorPot < 194 && ValorPot > 172)
    {
        return 9;
    }
    else if (ValorPot < 215 && ValorPot > 193)
    {
        return 10;
    }
    else if (ValorPot < 236 && ValorPot > 214)
    {
        return 11;
    }
    else
    {
        return 11;
    }
}


void writeToEEPROM(unsigned char data, unsigned char address)
{
    EEADR = address;
    EEDAT = data;
    
    EECON1bits.EEPGD = 0;   // Escribir a memoria de datos
    EECON1bits.WREN = 1;    // Habilitar escritura a EEPROM (datos)
    
    INTCONbits.GIE = 0;     // Deshabilitar interrupciones
    INTCONbits.PEIE = 0;     // Deshabilitar interrupciones
    
    EECON2 = 0x55;          // Secuencia obligatoria
    EECON2 = 0xAA;
    EECON1bits.WR = 1;      // Escribir
    
    while(PIR2bits.EEIF==0);
    PIR2bits.EEIF = 0;
    
    INTCONbits.GIE = 1;     // Habilitar interrupciones
    INTCONbits.PEIE = 1;     // Habilitar interrupciones
    EECON1bits.WREN = 0;    // Deshabilitar escritura de EEPROM
    
    return;
}

unsigned char readFromEEPROM(unsigned char   address)
{
    EEADR = address;        // direccion a leer
    EECON1bits.EEPGD = 0;   // memoria de datos
    EECON1bits.RD = 1;      // hace una lectura
    unsigned char  data = EEDAT;   // guardar el dato le?do de EEPROM
    return data;
}

void readFromEEPROMPOTS(void)
{
    ConTEEPROM = 0;
    Reading = 1;
    while (ConTEEPROM < 5)
    {
        switch(ConTEEPROM)
        {
            case 0:
                if (PIR1bits.TXIF)
                /*{
                    Envio_Cadena (Read1);
                }*/
                __delay_us(20);
                POT0 = readFromEEPROM(ADD_POT0[0]);
                __delay_us(20);
                POT1 = readFromEEPROM(ADD_POT1[0]);
                __delay_us(20);
                POT2 = readFromEEPROM(ADD_POT2[0]);
                __delay_us(20);
                POT3 = readFromEEPROM(ADD_POT3[0]);
                __delay_us(20);
                PORTBbits.RB2 = 0;
                __delay_ms(250);
                __delay_ms(250);
                ConTEEPROM = 1;
               break;

            case 1:
                if (PIR1bits.TXIF)
                /*{
                    Envio_Cadena (Read2);
                }*/
                __delay_us(20);
                POT0 = readFromEEPROM(ADD_POT0[1]);
                __delay_us(20);
                POT1 = readFromEEPROM(ADD_POT1[1]);
                __delay_us(20);
                POT2 = readFromEEPROM(ADD_POT2[1]);
                __delay_us(20);
                POT3 = readFromEEPROM(ADD_POT3[1]);
                __delay_us(20);
                PORTBbits.RB3 = 0;
                __delay_ms(250);
                __delay_ms(250);
                ConTEEPROM = 2;
               break;

            case 2:
                if (PIR1bits.TXIF)
                /*{
                    Envio_Cadena (Read3);
                }*/
                __delay_us(20);
                POT0 = readFromEEPROM(ADD_POT0[2]);
                __delay_us(20);
                POT1 = readFromEEPROM(ADD_POT1[2]);
                __delay_us(20);
                POT2 = readFromEEPROM(ADD_POT2[2]);
                __delay_us(20);
                POT3 = readFromEEPROM(ADD_POT3[2]);
                __delay_us(20);
                PORTBbits.RB4 = 0;
                __delay_ms(250);
                __delay_ms(250);
                ConTEEPROM = 3;
               break;
               
            case 3:
                if (PIR1bits.TXIF)
                /*{
                    Envio_Cadena (Read4);
                }*/
                __delay_us(20);
                POT0 = readFromEEPROM(ADD_POT0[3]);
                __delay_us(20);
                POT1 = readFromEEPROM(ADD_POT1[3]);
                __delay_us(20);
                POT2 = readFromEEPROM(ADD_POT2[3]);
                __delay_us(20);
                POT3 = readFromEEPROM(ADD_POT3[3]);
                __delay_us(20);
                PORTBbits.RB5 = 0;
                __delay_ms(250);
                __delay_ms(250);
                ConTEEPROM = 4;
               break;

            case 4:
                if (PIR1bits.TXIF)
                /*{
                    Envio_Cadena (Read5);
                }*/
                __delay_us(20);
                POT0 = readFromEEPROM(ADD_POT0[4]);
                __delay_us(20);
                POT1 = readFromEEPROM(ADD_POT1[4]);
                __delay_us(20);
                POT2 = readFromEEPROM(ADD_POT2[4]);
                __delay_us(20);
                POT3 = readFromEEPROM(ADD_POT3[4]);
                __delay_us(20);
                PORTBbits.RB6 = 0;
                __delay_ms(250);
                __delay_ms(250);
                ConTEEPROM = 5;
               break;
        }
    }
    Reading = 0;
    ConTEEPROM = 0;
    return;
}

//**
//                              INTERRUPCI?N
//**
void __interrupt() isr(void)
{    
    
    //Interrupci?n ADC o PC
    if (PCenable == 0)
    {
        if (ADIF == 1) 
        {
            if (Reading == 0)
            {
                switch (ADCON0bits.CHS)
                {
                    case 0:
                        POT0 = NumIteraciones(ADRESH);          //Se guarda el valor del POT1
                        ADCON0bits.CHS = 1;     //cambio para el multiplexeo
                        break;

                    case 1:
                        POT1 = NumIteraciones(ADRESH);         //Se guarda el valor del POT1
                        ADCON0bits.CHS = 2;     //cambio para el multiplexeo
                        break;

                    case 2:
                        POT2 = NumIteraciones(ADRESH);          //Se guarda el valor del POT1
                        ADCON0bits.CHS = 3;     //cambio para el multiplexeo
                        break;

                   case 3:
                        POT3=  NumIteraciones(ADRESH);          //Se guarda el valor del POT1
                        ADCON0bits.CHS = 4;     //cambio para el multiplexeo
                        break;

                    case 4:
                        CCPR1L = (ADRESH>>1)+124;//para que el servo1 pueda girar 180 grados
                        CCP1CONbits.DC1B1 = ADRESH & 0b01; //a?adir precision/resolucion
                        CCP1CONbits.DC1B0 = (ADRESL>>7);
                        ADCON0bits.CHS = 0; //se cambia al canal del segundo pot
                }
            }
            __delay_us(20);             //delay de 20 us
            PIR1bits.ADIF = 0;          //limpieza de bandera
            ADCON0bits.GO = 1;          //inicio de conversi?n
        }
    }
    
    else 
    {
        if (PIR1bits.RCIF == 1){
                    UART();
                }

    }
    /*
    else 
    {   //Interrupciones EEPROM
        if (PIR1bits.RCIF)
        {
            switch (RCREG)
            {
                case 49:
                    POT0 = 4;
                    break;
                case 50:
                    POT0 = 5;
                    break;
                case 51:
                    POT0 = 6;
                    break;
                case 52:
                    POT0 = 7;
                    break;                  
                case 53:
                    POT0 = 8;
                    break;                   
                case 54:
                    POT0 = 9;
                    break;                   
                case 55:
                    POT0 = 10;
                    break;                  
                case 56:
                    POT0 = 11;
                    break;    
                
                    
                case 113:
                    POT1 = 4;
                    break;
                case 119:
                    POT1 = 5;
                    break;
                case 101:
                    POT1 = 6;
                    break;                    
                case 114:
                    POT1 = 7;
                    break;                    
                case 116:
                    POT1 = 8;
                    break;                    
                case 121:
                    POT1 = 9;
                    break;                    
                case 117:
                    POT1 = 10;
                    break;                    
                case 105:
                    POT1 = 11;
                    break;  
                    
                
                case 97:
                    POT2 = 4;
                    break;
                case 115:
                    POT2 = 5;
                    break;
                case 100:
                    POT2 = 6;
                    break;                    
                case 102:
                    POT2 = 7;
                    break;                    
                case 103:
                    POT2 = 8;
                    break;                    
                case 104:
                    POT2 = 9;
                    break;                    
                case 106:
                    POT2 = 10;
                    break;                    
                case 107:
                    POT2 = 11;
                    break;                   
                case 122:
                    POT3 = 4;
                    break;
                case 120:
                    POT3 = 5;
                    break;
                case 99:
                    POT3 = 6;
                    break;                    
                case 118:
                    POT3 = 7;
                    break;                    
                case 98:
                    POT3 = 8;
                    break;                    
                case 110:
                    POT3 = 9;
                    break;                    
                case 109:
                    POT3 = 10;
                    break;                    
                case 44:
                    POT3 = 11;
                    break;                    
                   
            }
            
        }
    }*/

    
   if (T0IF == 1) 
   {                
        TMR0 = 128;                 //reseteo de timer0
        if (ItPOT0 == POT0)
        {
            PORTDbits.RD0 = 0;
            ItPOT0 = 0;
            ENTM0Pot0 = 0;
        }
        if (ItPOT1 == POT1)
        {
            PORTDbits.RD1 = 0;
            ItPOT1 = 0;
            ENTM0Pot1 = 0;
        }
        if (ItPOT2 == POT2)
        {
            PORTDbits.RD2 = 0;
            ItPOT2 = 0;
            ENTM0Pot2 = 0;
        }
        if (ItPOT3 == POT3)
        {
            PORTDbits.RD3 = 0;
            ItPOT3 = 0;
            ENTM0Pot3 = 0;
        }
        if (ENTM0Pot0 == 1)
        {
            ItPOT0++;
        }
        if (ENTM0Pot1 == 1)
        {
            ItPOT1++;
        }
        if (ENTM0Pot2 == 1)
        {
            ItPOT2++;
        }
        if (ENTM0Pot3 == 1)
        {
            ItPOT3++;
        }
        INTCONbits.T0IF = 0;        //limpieza de bandera
    }
    
    if (TMR1IF == 1)
    {
        TMR1H = 255;
        TMR1L = 191;
        if (ConTMR1 == 1)
        {
           ENTM0Pot0 = 1; 
           ENTM0Pot1 = 1; 
           ENTM0Pot2 = 1; 
           ENTM0Pot3 = 1; 
        }   
        
        if (ConTMR1 == 71)
        {
            ConTMR1 = 0;
            PORTDbits.RD0 = 1;
            PORTDbits.RD1 = 1;
            PORTDbits.RD2 = 1;
            PORTDbits.RD3 = 1;
        }
        ConTMR1++;
        PIR1bits.TMR1IF = 0;       //limpieza de TMR1 
    }
}
        
void main(void) 
{
//**
//                              CONFIGURACIONES
//**
    
    //RELOJ
    OSCCONbits.IRCF = 0b0111;       //Reloj a 8 MHz
    OSCCONbits.SCS   = 1;
    
    
    //INPUTS/OUTPUTS
    ANSELH = 0;                     
    ANSELbits.ANS0  = 1;            //RA: 0,1,2,3 entradas analogas
    ANSELbits.ANS1  = 1;
    ANSELbits.ANS2  = 1;
    ANSELbits.ANS3  = 1;
    TRISA  = 0b11111111;            //PORTA inputs
    TRISD  = 0;                     //PORTD salidas
    TRISE  = 0;                     //PORTE salidas
    TRISB  = 0;            //PORTD salidas
    TRISB  = 0b10000011;            //PORTD salidas
    OPTION_REGbits.nRBPU = 0;
    WPUBbits.WPUB0 = 1;             //habilitar pull-ups
    WPUBbits.WPUB1 = 1;
    WPUBbits.WPUB7 = 1;
    PORTA  = 0;                     //limpieza de puertos
    PORTD  = 0;
    PORTB  = 0;
    PORTB  = 0b10000011;
    PORTC  = 0;
    PORTE  = 0;
    ItPOT0 = 0;
    ItPOT1 = 0;
    ItPOT2 = 0;
    ItPOT3 = 0;
    POT0 = 0;
    POT1 = 0;
    POT2 = 0;
    POT3 = 0;
    ConTMR1 = 0;
    
    
    
    //ADC
    ADCON0bits.ADCS = 2;            //Fosc/32 
    ADCON0bits.CHS0 = 0;            //se selecciona el canal AN0
    ADCON1bits.VCFG1 = 0;           //voltajes referencia default
    ADCON1bits.VCFG0 = 0;           
    ADCON1bits.ADFM = 0;            //justificaci?n izquierda
    ADCON0bits.ADON = 1;            
    __delay_us(20);                 //delay de 20 us
    
    
    //PWM
    TRISCbits.TRISC2 = 1;           //CCP1 como entrada;
    PR2 = 255;                      //valor para que el periodo pwm sea 2 ms 
    CCP1CONbits.P1M = 0;       
    CCP1CONbits.CCP1M = 0b1100;
    CCPR1L = 0x01;                  //ciclo de trabajo inicial
    CCP1CONbits.DC1B = 0;
    
    // 9600 BAUDIOS
    //Configuracion de TX y RX / UART
    TXSTAbits.SYNC = 0;
    TXSTAbits.BRGH = 1;
    
    BAUDCTLbits.BRG16 = 1;
    
    SPBRG = 103;
    SPBRGH = 0;
   
    RCSTAbits.SPEN = 1; //enable
    RCSTAbits.RX9 = 0;  //no comunicarme en 9 bits (modo de 8 bits)
    RCSTAbits.CREN = 1; //habilitamos la recepcion
    TXSTAbits.TXEN = 1; //habilitamos la transmision
    TXSTAbits.TX9 = 0; //no comunicarme en 9 bits
    
    //TMR2
    PIR1bits.TMR2IF = 0;            //limpieza de bandera
    T2CONbits.T2CKPS = 0b11;        //prescaler 1:16
    T2CONbits.TMR2ON = 1;           
    while(PIR1bits.TMR2IF == 0);    //esperar un ciclo de tmr2
    PIR1bits.TMR2IF = 0;
    TRISCbits.TRISC2 = 0;           
    TRISCbits.TRISC1 = 0;   
    
    //TMR0
    OPTION_REGbits.T0CS = 0;
    OPTION_REGbits.PSA  = 0;
    OPTION_REGbits.PS2  = 0;
    OPTION_REGbits.PS1  = 0;
    OPTION_REGbits.PS0  = 0;        //prescaler a 2
    TMR0 = 228;                      //reseteo TMR0
    INTCONbits.T0IF = 0;
    
    //TMR1
    T1CONbits.TMR1ON = 1;
    T1CONbits.TMR1CS = 0;
    T1CONbits.TMR1CS = 0;
    T1CONbits.T1CKPS1 = 1;
    T1CONbits.T1CKPS0 = 1;          //prescaler a 8
    TMR1H = 255;
    TMR1L = 16;
    PIR1bits.TMR1IF = 0;            //limpieza de la bandera
    
    //configuraci?n recepci?n-transmisi?n
    TXSTAbits.SYNC = 0;
    TXSTAbits.BRGH = 1;
    BAUDCTLbits.BRG16 = 1;
    SPBRG = 207;
    SPBRGH = 0;
    RCSTAbits.SPEN = 1;
    RCSTAbits.RX9 = 0;
    RCSTAbits.CREN = 1;
    TXSTAbits.TXEN = 1;
    
    //INTERRUPCIONES
    PIR1bits.ADIF = 0;
    PIE1bits.ADIE = 1;              //interrupci?n ADC
    INTCONbits.PEIE = 1;            
    INTCONbits.GIE  = 1; 
    INTCONbits.T0IE  = 1;           //interrupci?n del TMR0
    PIE1bits.TMR1IE = 1;            //interrupcion del TMR1
    ADCON0bits.GO = 1;              
    PIE1bits.RCIE = 1;
    PIR1bits.RCIF = 1;
    
    
    while (1)
    {
        PORTEbits.RE1 = WriteDone;
        if (RB0 == 0)
            RB0_old = 1;
        
        if (RB1 == 0)
            RB1_old = 1;
        
        if (RB7 == 0)
            RB7_old = 1;
        
        if (RB0 == 1 && RB0_old == 1)
        {
            switch(ConTEEPROM)
            {
                case 0:
                    writeToEEPROM(POT0,ADD_POT0[0]); 
                    writeToEEPROM(POT1,ADD_POT1[0]); 
                    writeToEEPROM(POT2,ADD_POT2[0]); 
                    writeToEEPROM(POT3,ADD_POT3[0]);
                    ConTEEPROM = 1;
                    PORTBbits.RB2 = 1;
                    /*if (PIR1bits.TXIF)
                    {
                        Envio_Cadena (Save);
                    }*/
                    if (PIR1bits.TXIF)
                    {
                        TXREG = 49;
                    }
                    __delay_ms(1);
                    if (PIR1bits.TXIF)
                    {
                        TXREG = 13;
                    }
                    break;
                
                case 1:
                    writeToEEPROM(POT0,ADD_POT0[1]); 
                    writeToEEPROM(POT1,ADD_POT1[1]); 
                    writeToEEPROM(POT2,ADD_POT2[1]); 
                    writeToEEPROM(POT3,ADD_POT3[1]);
                    ConTEEPROM = 2;
                    PORTBbits.RB3 = 1;
                    /*if (PIR1bits.TXIF)
                    {
                        Envio_Cadena (Save);
                    }*/
                    if (PIR1bits.TXIF)
                    {
                        TXREG = 50;
                    }
                    __delay_ms(1);
                    if (PIR1bits.TXIF)
                    {
                        TXREG = 13;
                    }
                    break;
                
                case 2:
                    writeToEEPROM(POT0,ADD_POT0[2]); 
                    writeToEEPROM(POT1,ADD_POT1[2]); 
                    writeToEEPROM(POT2,ADD_POT2[2]); 
                    writeToEEPROM(POT3,ADD_POT3[2]);
                    ConTEEPROM = 3;
                    PORTBbits.RB4 = 1;
                    /*if (PIR1bits.TXIF)
                    {
                        Envio_Cadena (Save);
                    }*/
                    if (PIR1bits.TXIF)
                    {
                        TXREG = 51;
                    }
                    __delay_ms(1);
                    if (PIR1bits.TXIF)
                    {
                        TXREG = 13;
                    }
                    break;
                
                case 3:
                    writeToEEPROM(POT0,ADD_POT0[3]); 
                    writeToEEPROM(POT1,ADD_POT1[3]); 
                    writeToEEPROM(POT2,ADD_POT2[3]); 
                    writeToEEPROM(POT3,ADD_POT3[3]);
                    ConTEEPROM = 4;
                    PORTBbits.RB5 = 1;
                    /*if (PIR1bits.TXIF)
                    {
                        Envio_Cadena (Save);
                    }*/
                    if (PIR1bits.TXIF)
                    {
                        TXREG = 52;
                    }
                    __delay_ms(1);
                    if (PIR1bits.TXIF)
                    {
                        TXREG = 13;
                    }
                    break;
                
                case 4:
                    writeToEEPROM(POT0,ADD_POT0[4]); 
                    writeToEEPROM(POT1,ADD_POT1[4]); 
                    writeToEEPROM(POT2,ADD_POT2[4]); 
                    writeToEEPROM(POT3,ADD_POT3[4]);
                    ConTEEPROM = 5;
                    PORTBbits.RB6 = 1;
                    /*if (PIR1bits.TXIF)
                    {
                        Envio_Cadena (Save);
                    }*/
                    if (PIR1bits.TXIF)
                    {
                        TXREG = 53;
                    }
                    __delay_ms(1);
                    if (PIR1bits.TXIF)
                    {
                        TXREG = 13;
                    }
                    WriteDone = 1;
                    __delay_ms(1);
                    /*if (PIR1bits.TXIF)
                    {
                        Envio_Cadena (SaveDone);
                    }*/
                    break;  
            }
            RB0_old = 0;
        }
        
        if (RB1 == 1 && RB1_old == 1)
        {
            if (WriteDone == 1)
            {
                WriteDone = 0;
                ConTEEPROM = 0;
                PORTEbits.RE1 = 0;
                PORTEbits.RE2 = 1;
                readFromEEPROMPOTS();
                /*if (PIR1bits.TXIF)
                {
                    Envio_Cadena (ReadDone);
                }*/
                PORTEbits.RE2 = 0;
                RB1_old = 0;
            }
        }
        
        if (RB7 == 1 && RB7_old == 1)
        {
            if (PCenable == 0)
            {
                PORTEbits.RE0 = 1;
                PCenable = 1;
            }
            RB7_old = 0;
        }
    }      
}
