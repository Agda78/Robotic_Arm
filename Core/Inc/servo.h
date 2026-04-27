#ifndef __SERVO_H__
#define __SERVO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

// Numero di motori complessivo
#define MOTOR_NUM 5

// Valori limite dei motori

// Motori dei giunti
#define MAX_DEGREE_NORMAL 120
#define MIN_DEGREE_NORMAL 0

// Motore della Pinza
#define MAX_DEGREE_PINZA 90
#define MIN_DEGREE_PINZA 60

// Struttura dati per rendere più modulare il set dei gradi per i motori
typedef struct{
	TIM_HandleTypeDef* Tim; // Puntatore al timer di riferimento per il motore
	uint32_t Channel; // Riferimento diretto alla macro del canale
}Motor_t;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

// Dichiarazione delle funzioni di gestione dei servomotori
void Motor_Init();

// Oltre ad eseguire l'azionamento, ritorna un intero che permette di capire
// se il movimento è andato a buon fine o meno
uint8_t set_degrees(uint16_t degrees, uint8_t motor_id);

// Funzioni di alto livello per migliorare la comprensione del codice
void close_pinza();
void open_pinza();

#ifdef __cplusplus
}
#endif
#endif /*__ SERVO_H__ */
