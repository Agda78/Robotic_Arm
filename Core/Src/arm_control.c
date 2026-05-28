#include "arm_control.h"
#include "servo.h"

#include <stdio.h>

// Struttura dati di contenimento della posizione del braccio
uint8_t position [NUM_CONTROLLED_MOTOR];
float position_float [NUM_CONTROLLED_MOTOR];

float p_filt [NUM_CONTROLLED_MOTOR];
float p_inv [NUM_CONTROLLED_MOTOR];

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
}

void azionamento_braccio_filtrato(float accelerometro_taked[3]){
	float accelerometro[3];
	for (uint8_t i = 0; i < 3; i++){
		accelerometro[i] = accelerometro_taked[i];
	}

	//HAL_GPIO_TogglePin(Test_PIN_GPIO_Port, Test_PIN_Pin);
	static uint32_t last_tick_ms = 0;

	if (safety_variable == 0){ // Controllo di sicurezza
		// Variabili di gestione del movimento
		printf("CB %f, %f, %f\r\n", accelerometro[0], accelerometro[1], accelerometro[2]);
		uint8_t mov_result = 0;
		uint32_t now = HAL_GetTick();
		float dt = (now - last_tick_ms)/1000.0f;
		last_tick_ms = now;

		if (dt <= 0.0f || dt >= 0.5f){
			dt = 0.020f;
		}

		// 1. Gestione dell'azionamento (Scelta del movimento mediante una proporzione)
		if (accelerometro[CONTROL_BASE] > TOL_LOW){
			position_float[CONTROL_BASE] += GUADAGNO * dt * (accelerometro[CONTROL_BASE] - TOL_LOW);

		}else if(accelerometro[CONTROL_BASE] < -TOL_LOW){
			position_float[CONTROL_BASE] += GUADAGNO * dt * (accelerometro[CONTROL_BASE] + TOL_LOW);
		}

		if (accelerometro[CONTROL_ALTEZZA] > TOL_LOW){
			position_float[CONTROL_ALTEZZA] += GUADAGNO * dt * (accelerometro[CONTROL_ALTEZZA] - TOL_LOW);

		}else if(accelerometro[CONTROL_ALTEZZA] < -TOL_LOW){
			position_float[CONTROL_ALTEZZA] += GUADAGNO * dt * (accelerometro[CONTROL_ALTEZZA] + TOL_LOW);
		}

		printf("CB %f, %f\r\n", position_float[0], position_float[1]);

		float alfa = dt/TAU;

		// 2. Gestione del movimento mediante ISR su Timer
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

		// Angolo Normale -> tick-and-set
		for (uint8_t i=0; i < NUM_CONTROLLED_MOTOR; i++){
			// Passaggio da angoli a Tick da contare
			uint16_t position_16 = (uint8_t)(p_inv[i] / SOGLIA_JITTER);
			if (position_16 >= 200){
				position_float[i] = 100.0f;
				p_filt[i] = 100.0f;
				p_inv[i] = 100.0f;
				position[i] = 200;
				mov_result = set_degrees(position[i], i + 1);
			}else if (position_16 <= 0){
				position_float[i] = 0.0f;
				p_filt[i] = 0.0f;
				p_inv[i] = 0.0f;
				position[i] = 0;
				mov_result = set_degrees(position[i], i + 1);
			}else{
				position[i] = (uint8_t) position_16;
				mov_result = set_degrees(position[i], i + 1);
			}
		}
	}
}


