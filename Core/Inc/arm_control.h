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


// Tolleranze
// Tali tolleranze fanno riferimento alla lunghezza dei vettori che vengono proiettati
// lungo lo specifico asse. Superata una certa "tolleranza" permette di avere un certo tipo di azionamento
#define TOL_LOW 3.4f

// Configurazione in base al posizionamento dell'accellerometro
// 0 -> ax, 1 -> ay, 2 -> az
#define CONTROL_BASE 0
#define CONTROL_ALTEZZA 1

// Numero di motori da controllare
#define NUM_CONTROLLED_MOTOR 3

/* [ CONTROLLO FILTRATO ] */
#define GUADAGNO 20.0f
#define ALFA 0.1f
#define SOGLIA_JITTER 0.6f // Dipende dalla "sensibilità" del controllo

#define TAU 0.02f
void posizionamento_iniziale(uint8_t position_zero[NUM_CONTROLLED_MOTOR]);

// Controllo sviluppato secondo uno specifico filtraggio
void azionamento_braccio_filtrato(float accellerometro[3]);

#ifdef __cplusplus
}
#endif
#endif /*__ARM_CONTROL_H__ */
