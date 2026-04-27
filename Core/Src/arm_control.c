#include "arm_control.h"
#include "servo.h"

// Struttura dati di contenimento della posizione del braccio
uint8_t position [NUM_CONTROLLED_MOTOR];

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
			set_degrees(position[i], i + 1);
		}
	}else{
		for(uint8_t i = 0;i<NUM_CONTROLLED_MOTOR;i++){
			position[i] = 0;
			set_degrees(position[i], i + 1);
		}
	}
}

/*
 * @param accellerometro: Vettore contenente i dati di accellerazione rispetto a xyz
 * Questo parametro verrà interpretato nel seguente modo: [ax, ay, az]
 *
 */
void azionamento_braccio(float accellerometro[3]){
	// Corpo di azionamento
	uint8_t mov_result = 0;
	// 1. Parte di azionamento della base
	if (accellerometro[CONTROL_BASE] > TOL_LOW){
		if (accellerometro[CONTROL_BASE] > TOL_SPEED){
			mov_result = set_degrees(position[POS_BASE] + DELTA_BASE, MOTOR_ID_BASE);
			if (mov_result == 0){ // Se non fosse 0, non si potrebbe forzare
				position[POS_BASE] += DELTA_BASE;
			}else{
				set_degrees(MAX_DEGREE_NORMAL, MOTOR_ID_BASE);
				position[POS_BASE] = MAX_DEGREE_NORMAL;
			}
		}else{
			uint8_t incremento = (uint8_t)(DECREMENTO * DELTA_BASE);
			mov_result = set_degrees(position[POS_BASE] + incremento, MOTOR_ID_BASE);
			if (mov_result == 0){ // Se non fosse 0, non si potrebbe forzare
				position[POS_BASE] += incremento;
			}else{
				set_degrees(MAX_DEGREE_NORMAL, MOTOR_ID_BASE);
				position[POS_BASE] = MAX_DEGREE_NORMAL;
			}
		}
	}else if(accellerometro[CONTROL_BASE] < -TOL_LOW){
		if (accellerometro[CONTROL_BASE] < -TOL_SPEED){
			mov_result = set_degrees(position[POS_BASE] - DELTA_BASE, MOTOR_ID_BASE);
			if (mov_result == 0){ // Se non fosse 0, non si potrebbe forzare
				position[POS_BASE] -= DELTA_BASE;
			}else{
				set_degrees(MIN_DEGREE_NORMAL, MOTOR_ID_BASE);
				position[POS_BASE] = MIN_DEGREE_NORMAL;
			}
		}else{
			uint8_t incremento = (uint8_t)(DECREMENTO * DELTA_BASE);
			mov_result = set_degrees(position[POS_BASE] - incremento, MOTOR_ID_BASE);
			if (mov_result == 0){ // Se non fosse 0, non si potrebbe forzare
				position[POS_BASE] -= incremento;
			}else{
				set_degrees(MIN_DEGREE_NORMAL, MOTOR_ID_BASE);
				position[POS_BASE] = MIN_DEGREE_NORMAL;
			}
		}
	}

	// 2. Parte di azionamento della lunghezza
	if (accellerometro[CONTROL_ALTEZZA] > TOL_LOW){
		if (accellerometro[CONTROL_ALTEZZA] > TOL_SPEED){
			mov_result = controllo_sincrono(DELTA_ALTEZZA, 1);
			if (mov_result == 0){
				position[POS_ALTEZZA_FIRST] += DELTA_ALTEZZA;
				position[POS_ALTEZZA_SECOND] += DELTA_ALTEZZA;
			}
			// Capire cosa fare quando i motori escono dai bound (potremmo perdere il parallelismo con il suolo
//			else{
//				incremento_sincrono(mov_result, DELTA_ALTEZZA, 1);
//				set_degrees(position[POS_ALTEZZA_FIRST], MOTOR_ID_FIRST);
//				set_degrees(position[POS_ALTEZZA_SECOND], MOTOR_ID_SECOND);
//			}
		}else{
			uint8_t incremento = (uint8_t)(DECREMENTO * DELTA_ALTEZZA);
			mov_result = controllo_sincrono(incremento, 1);
			if (mov_result == 0){
				position[POS_ALTEZZA_FIRST] += incremento;
				position[POS_ALTEZZA_SECOND] += incremento;
			}
		}
	}else if (accellerometro[CONTROL_ALTEZZA] < -TOL_LOW){
		if (accellerometro[CONTROL_ALTEZZA] < -TOL_SPEED){
			mov_result = controllo_sincrono(DELTA_ALTEZZA, 0); // Lo 0 indica sottrazione
			if (mov_result == 0){
				position[POS_ALTEZZA_FIRST] -= DELTA_ALTEZZA;
				position[POS_ALTEZZA_SECOND] -= DELTA_ALTEZZA;
			}
		}else{
			uint8_t incremento = (uint8_t)(DECREMENTO * DELTA_ALTEZZA);
			mov_result = controllo_sincrono(incremento, 0);
			if (mov_result == 0){
				position[POS_ALTEZZA_FIRST] -= incremento;
				position[POS_ALTEZZA_SECOND] -= incremento;
			}
		}
	}
}

uint8_t controllo_sincrono(uint8_t increment, uint8_t positive){
	uint8_t risultato_1 = 0;
	uint8_t risultato_2 = 0;
	if (positive == 1){
		risultato_1 = set_degrees(position[POS_ALTEZZA_FIRST] + increment, MOTOR_ID_FIRST);
		risultato_2 = set_degrees(position[POS_ALTEZZA_SECOND] + increment, MOTOR_ID_SECOND);
	}else{
		risultato_1 = set_degrees(position[POS_ALTEZZA_FIRST] - increment, MOTOR_ID_FIRST);
		risultato_2 = set_degrees(position[POS_ALTEZZA_SECOND] - increment, MOTOR_ID_SECOND);
	}
	// 0 -> entrambi 0 (tutto ok)
	// 1 -> il primo è 1 (esclusivamente)
	// 2 -> il secondo è 1 (esclusivamente)
	// 3 -> entrambi sono a 1

	return (uint8_t) (risultato_1 + risultato_2 * 2);
}

//// Nel caso di eventuale controllo limite, decommentare
//void incremento_sincrono(uint8_t result, uint8_t incremento, uint8_t positive){
//	switch(mov_result){
//		case 1:
//			if (incremento == 1){
//				position[POS_ALTEZZA_FIRST] = MAX_DEGREE_NORMAL;
//				position[POS_ALTEZZA_SECOND] += incremento;
//			}else{
//				position[POS_ALTEZZA_FIRST] = MIN_DEGREE_NORMAL;
//				position[POS_ALTEZZA_SECOND] -= incremento;
//			}
//			break;
//		case 2:
//			if (incremento == 1){
//				position[POS_ALTEZZA_SECOND] = MAX_DEGREE_NORMAL;
//				position[POS_ALTEZZA_FIRST] += incremento;
//			}else{
//				position[POS_ALTEZZA_SECOND] = MIN_DEGREE_NORMAL;
//				position[POS_ALTEZZA_FIRST] -= incremento;
//			}
//			break;
//		case 3:
//			if (incremento == 1){
//				position[POS_ALTEZZA_FIRST] = MAX_DEGREE_NORMAL;
//				position[POS_ALTEZZA_SECOND] = MAX_DEGREE_NORMAL;
//			}else{
//				position[POS_ALTEZZA_FIRST] = MIN_DEGREE_NORMAL;
//				position[POS_ALTEZZA_SECOND] = MIN_DEGREE_NORMAL;
//
//			}
//			break;
//	}
//}
//
