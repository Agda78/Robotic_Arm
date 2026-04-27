#ifndef __ARM_CONTROL_H__
#define __ARM_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// Tale librerie viene implementata per effettuare le seguente operazioni
// 1. Configurazione del sistema di gestione dei segnali dell'accellerometro
// 2. Sistema di implementazione di sistemi di pulizia del segnale/filtraggio digitale
// 3. Implementazione di un sistema ricunfigurabile di "guadagni" di velocità

/* [ Defines ] */
// Tolleranze
// Tali tolleranze fanno riferimento alla lunghezza dei vettori che vengono proiettati
// lungo lo specifico asse. Superata una certa "tolleranza" permette di avere un certo tipo di azionamento
#define TOL_SPEED (float)6.2
#define TOL_LOW (float)3.4

// Decremento
// Percentuale di "movimento" applicata ai motori lenti rispetto a quelli veloci
#define DECREMENTO (float) 0.4

// Angolo di applicazione (diversificato per la movimentazione della base e dell'altezza)
#define DELTA_BASE 20 // [1°]
#define DELTA_ALTEZZA 20 // [1°]

// Configurazione in base al posizionamento dell'accellerometro
// 0 -> ax, 1 -> ay, 2 -> az
#define CONTROL_BASE 0
#define CONTROL_ALTEZZA 1

// Numero di motori da controllare
#define NUM_CONTROLLED_MOTOR 3

// Posizione rispetto al vettore di stato, dell'angolo che la base sta assumendo
#define POS_BASE 0
#define MOTOR_ID_BASE 1

// Posizione dei motori di controllo dell'altezza rispetto al vettore
#define POS_ALTEZZA_FIRST 1
#define MOTOR_ID_FIRST 2

#define POS_ALTEZZA_SECOND 2
#define MOTOR_ID_SECOND 3

/* [ FUNCTIONS ] */
void posizionamento_iniziale(uint8_t position_zero[NUM_CONTROLLED_MOTOR]);

// L'azionamento del braccio va a mappare i movimenti del braccio
// in base ai dati dell'accellerometro (Componenti del vettore g sugli assi)
void azionamento_braccio(float accellerometro[3]);

// [ OUT_OF_LIBRARY ]
// Questa funzione è specifica per la nostra architettura
uint8_t controllo_sincrono(uint8_t incement, uint8_t positive);
// void incremento_sincrono(uint8_t result, uint8_t incremento, uint8_t positive);

#ifdef __cplusplus
}
#endif
#endif /*__ARM_CONTROL_H__ */
