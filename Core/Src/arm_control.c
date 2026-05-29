#include "arm_control.h"
#include "servo.h"

#include <stdio.h>

// Struttura dati di contenimento della posizione del braccio
uint8_t position [NUM_CONTROLLED_MOTOR];
float position_float [NUM_CONTROLLED_MOTOR];

float p_filt [NUM_CONTROLLED_MOTOR];
float p_inv [NUM_CONTROLLED_MOTOR];

volatile uint8_t movimento_braccio = 0; // Di base non si muove
volatile uint8_t nTimes = 0;

/*
 * Funzione di inizializzazione dello stato dei motori
 * @param position_zero: Vettore che contiene i dati caorrispondenti ad una posizione
 * di riferimento da cui poi andare a partire (espressa nello spazio dei giunti)
 * Nel caso si passi un puntatore a NULL, la funzione inizializzerà a 0
 */
void posizionamento_iniziale(uint8_t position_zero[NUM_CONTROLLED_MOTOR]){
	if (position_zero != NULL){
		for(uint8_t i = 0;i<NUM_CONTROLLED_MOTOR;i++){
			position[i] = position_zero[i];
			position_float[i] = position_zero[i] * 0.5f;
			p_filt[i] = position_zero[i] * 0.5f;
			p_inv[i] = position_zero[i] * 0.5f;

			set_degrees(position[i], i + 1);
		}
	}else{
		for(uint8_t i = 0;i<NUM_CONTROLLED_MOTOR;i++){
			position[i] = 0;
			position_float[i] = 0.0f;
			p_filt[i] = 0.0f;
			p_inv[i] = 0.0f;

			set_degrees(position[i], i + 1);
		}
	}

	open_pinza();
}

/*
 * Funzione di azionamento del braccio robotico mediante un controllo più graduale.
 * Il suo funzionamento è costituito da un integratore + un filtro EMA che ha permesso
 * di rendere i movimenti più fluidi e meno scattosi
 * @param accelerometro_taked: Parametro che deve contenere i dati dell'accelerometro secondo
 * cui poi saranno valutati gli eventuali forzamenti da date ai motori
 */
void azionamento_braccio_filtrato(float accelerometro_taked[3]){

	// Bufferizzazione preliminare dei dati, per evitare race conditions
	float accelerometro[3];
	for (uint8_t i = 0; i < 3; i++){
		accelerometro[i] = accelerometro_taked[i];
	}

	// Variabile di raccolta del tick per andare a valutare il dt
	static uint32_t last_tick_ms = 0;

	if (safety_variable == 0){ // Controllo di sicurezza

		// Print di debug
		printf("CB %f, %f, %f\r\n", accelerometro[0], accelerometro[1], accelerometro[2]);

		// Variabili di gestione del movimento
		uint32_t now = HAL_GetTick();
		float dt = (now - last_tick_ms)/1000.0f;
		last_tick_ms = now;

		if (dt <= 0.0f || dt >= 0.5f){
			dt = 0.020f;
		}

		uint8_t stato_movimento = 0;

		// 1. Gestione dell'azionamento (Scelta del movimento mediante una proporzione) [INTEGRAZIONE]
		if (accelerometro[CONTROL_BASE] > TOL_LOW){
			stato_movimento = 0;
			position_float[CONTROL_BASE] += GUADAGNO * dt * (accelerometro[CONTROL_BASE] - TOL_LOW);

		}else if(accelerometro[CONTROL_BASE] < -TOL_LOW){
			stato_movimento = 0;
			position_float[CONTROL_BASE] += GUADAGNO * dt * (accelerometro[CONTROL_BASE] + TOL_LOW);
		}else{
			stato_movimento = 1;
		}

		if (accelerometro[CONTROL_ALTEZZA] > TOL_LOW){
			stato_movimento = 0;
			position_float[CONTROL_ALTEZZA] += GUADAGNO * dt * (accelerometro[CONTROL_ALTEZZA] - TOL_LOW);

		}else if(accelerometro[CONTROL_ALTEZZA] < -TOL_LOW){
			stato_movimento = 0;
			position_float[CONTROL_ALTEZZA] += GUADAGNO * dt * (accelerometro[CONTROL_ALTEZZA] + TOL_LOW);
		}

		// stato_movimento serve a controllare che il movimento sia nullo per ambo i gradi di libertà
		// quindi sia per la base che per l'altezza, per poi essere implementato come un sistema che va a gestire
		// il movimento effettivo
		if (stato_movimento == 1){ // Se effettivamente non si muove
			nTimes++;
			if (nTimes >= TIMES_DELAY){
				movimento_braccio = 0;
			}
		}else{
			nTimes = 0; // Questo permette di evitare che ci siano delle intenzioni che magari sono solo un rumore
			movimento_braccio = 1;
		}

		// Print di debug
		printf("CB %f, %f\r\n", position_float[0], position_float[1]);

		float alfa = dt/TAU;

		// 2. Filtro EMA per smussare la curva del movimento
		for (uint8_t i=0; i < 2; i++){
			p_filt[i] += alfa * (position_float[i] - p_filt[i]);
		}

		// Tulino Theorem
		p_filt[2] = p_filt[1];

		for (uint8_t i=0; i < NUM_CONTROLLED_MOTOR; i++){
			float diff = p_filt[i] - p_inv[i];
			if (diff >= SOGLIA_JITTER || diff <= -SOGLIA_JITTER){
				p_inv[i] = p_filt[i];
			}
		}

		// 3. Gestione dell'azionamento ben sogliato
		for (uint8_t i=0; i < NUM_CONTROLLED_MOTOR; i++){
			// Valutazione dei passi da effettuare per poter sopraggiungere all'angolo desiderato
			uint16_t position_16 = (uint8_t)(p_inv[i] / SOGLIA_JITTER);

			if (position_16 >= 195){ // Soglia per evitare accumuli inutili fuori dal range dei motori
				position_float[i] = 100.0f;
				p_filt[i] = 100.0f;
				p_inv[i] = 100.0f;
				position[i] = 200;
				set_degrees(position[i], i + 1);

			}else if (position_16 <= 5){ // Soglia ma per l'angolo inferiore
				position_float[i] = 0.0f;
				p_filt[i] = 0.0f;
				p_inv[i] = 0.0f;
				position[i] = 0;
				set_degrees(position[i], i + 1);

			}else{ // Controllo Base senza problemi di sogliatura
				position[i] = (uint8_t) position_16;
				set_degrees(position[i], i + 1);
			}
		}
	}
}


