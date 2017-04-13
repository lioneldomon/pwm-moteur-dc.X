#include <xc.h>
#include <pic18f25k22.h>
#include "test.h"

/**
 * Bits de configuration:
 */
#pragma config FOSC = INTIO67   // Osc. interne, A6 et A7 comme IO.
#pragma config IESO = OFF       // Pas d'osc. au démarrage.
#pragma config FCMEN = OFF      // Pas de monitorage de l'oscillateur.

// Nécessaires pour ICSP / ICD:
#pragma config MCLRE = EXTMCLR  // RE3 est actif comme master reset.
#pragma config WDTEN = OFF      // Watchdog inactif.
#pragma config LVP = OFF        // Single Supply Enable bits off.

typedef enum {
    AVANT = 0b01,
    ARRIERE = 0b10
} Direction;

#define PR2value  249           // (249+1)*4*Tosc*TMRPrescaler => 250*4us = 1ms (1khz))


/**
 * Indique la direction correspondante à la valeur du potentiomètre.
 * @param v Valeur du potentiomètre.
 * @return AVANT ou ARRIERE.
 */
Direction conversionDirection(unsigned char v) {
    // À implémenter.
    if(v > 127){
        return AVANT;
    }
    return ARRIERE;
//    if(v < 127){
//       return AVANT;     
//    }
//    return 0b00;
}

/**
 * Indique le cycle de travail PWM correspondant à la valeur du potentiomètre.
 * @param v Valeur du potentiomètre.
 * @return Cycle de travail du PWM.
 */
unsigned char conversionMagnitude(unsigned char v) {
    // À implémenter.
    
    int mag;
    if(v == 127 || v == 128){
        mag = 0;
    }
    else if(v < 127){
       mag = (127-v)*2;
       // return (PR2value-(PR2value-v));
    }
    else if(v > 128){
        mag = (v-128)*2;
        //return (PR2value-(v-PR2value));
    }
    return mag;
           
}

#ifndef TEST

/**
 * Initialise le hardware.
 */
static void hardwareInitialise() {
    // À implémenter.
    TRISCbits.RC0 = 0;      // RC0 en sortie
    TRISCbits.RC1 = 0;      // RC1 en sortie
    TRISCbits.RC2 = 0;      // RC2 en sortie (PWM))

    ANSELBbits.ANSB3 = 1;   // RB3 en analogique
    
    // Configuration du convertisseur AD  
    ADCON0 = 0b00100101;    // Active le convertisseur AD et AD9
    ADCON2bits.ADFM = 0;    // Resultat dans registre ADRESH (ignore les 2 bits de poids faible)
    ADCON2bits.ACQT = 000;  // Temps d'aquisition 000
    ADCON2bits.ADCS = 011;  // Selection du clock choix : Clock de l'oscillateur interne
    
    
    // Activer le temporisateur 2:    
    T2CONbits.T2OUTPS = 0b00001001;     // Postscaler de 1:10
    T2CONbits.T2CKPS = 00;              // Prescaler is 1
    T2CONbits.TMR2ON =1;                // Active le temporisateur2.

    
    // PWM
    CCPTMRS0bits.C1TSEL = 00;           // CCP1 utilise TMR2
    CCP1CONbits.CCP1M = 0b00001111;     // PWM mode
    PR2 = PR2value;
       
    
 
    // Active les interruptions
    RCONbits.IPEN = 1;          // Active les niveaux d'interruptions.
    INTCONbits.GIEH = 1;        // Active les interruptions de haute priorité.
    INTCONbits.GIEL = 1;        // Active les interruptions de basse priorité.
    PIE1bits.ADIE = 1;          // Active les interruptions de conv AD
    PIE1bits.TMR2IE = 1;        // Active les interruption du TMR2             
    IPR1bits.ADIP = 1;
}

/**
 * Point d'entrée des interruptions.
 */
void low_priority interrupt interruptionsBassePriorite() {
    if (PIR1bits.TMR2IF) {
        PIR1bits.TMR2IF = 0;            // Clear du bit d'interruption du TMR2
        ADCON0bits.GO = 1;              // Demarre la conversion AD
    }
    
    if (PIR1bits.ADIF) {
        PIR1bits.ADIF = 0;              // Clear du bit d'interruption du convertisseur AD
        PORTC = conversionDirection(ADRESH);    // Ecrit la direction de rotation
        CCPR1L = conversionMagnitude(ADRESH);   // Ecrit la valeur du temps à l'état haut du PWM
        CCP1CONbits.CCP1M = 0b00001111;   // PWM mode (sinon lorsque l'on est à 50% du pot, le système ne redémarre pas...)
    }
}

/**
 * Point d'entrée pour l'émetteur de radio contrôle.
 */
void main(void) {
    hardwareInitialise();       // Appel la fonction d'init harware
    
    while(1);
}
#endif

#ifdef TEST
void testConversionMagnitude() {
    testeEgaliteEntiers("CM01", conversionMagnitude(0),   254);
    testeEgaliteEntiers("CM02", conversionMagnitude(1),   252);
    testeEgaliteEntiers("CM03", conversionMagnitude(2),   250);
    
    testeEgaliteEntiers("CM04", conversionMagnitude(125),   4);
    testeEgaliteEntiers("CM05", conversionMagnitude(126),   2);
    
    testeEgaliteEntiers("CM06", conversionMagnitude(127),   0);
    testeEgaliteEntiers("CM07", conversionMagnitude(128),   0);

    testeEgaliteEntiers("CM08", conversionMagnitude(129),   2);
    testeEgaliteEntiers("CM09", conversionMagnitude(130),   4);
    
    testeEgaliteEntiers("CM10", conversionMagnitude(253), 250);
    testeEgaliteEntiers("CM11", conversionMagnitude(254), 252);
    testeEgaliteEntiers("CM12", conversionMagnitude(255), 254);
}
void testConversionDirection() {
    testeEgaliteEntiers("CD01", conversionDirection(  0), ARRIERE);    
    testeEgaliteEntiers("CD02", conversionDirection(  1), ARRIERE);    
    testeEgaliteEntiers("CD03", conversionDirection(127), ARRIERE);    
    testeEgaliteEntiers("CD04", conversionDirection(128), AVANT);
    testeEgaliteEntiers("CD05", conversionDirection(129), AVANT);
    testeEgaliteEntiers("CD06", conversionDirection(255), AVANT);    
}
void main() {
    initialiseTests();
    testConversionMagnitude();
    testConversionDirection();
    finaliseTests();
    while(1);
}
#endif
