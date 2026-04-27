#include "servo.h"

// Array di raccolta dei motori
Motor_t motors[MOTOR_NUM];

/* -----> [ DISPOSIZIONE MOTORI ] <----- */
// Tale disposizione è da prendere in considerazione dopo l'attivazione dei
// pin per la specifica SampleApp
// [ TIMER 2 ]
// Channel_1 -> Motor_id = 1 (MOT_1)
// Channel_3 -> Motor_id = 2 (MOT_2)
// [ TIMER 3 ]
// Channel_1 -> Motor_id = 3 (MOT_3)
// Channel_2 -> Motor_id = 4 (MOT_4)
// Channel_3 -> Motor_id = 5 (PINZA)

void Motor_Init(){
	// Definizione manuale dei motori

	// Motor_id = 1
	motors[0].Tim = &htim2;
	motors[0].Channel = TIM_CHANNEL_1;

	// Motor_id = 2
	motors[1].Tim = &htim2;
	motors[1].Channel = TIM_CHANNEL_3;

	// Motor_id = 3
	motors[2].Tim = &htim3;
	motors[2].Channel = TIM_CHANNEL_1;

	// Motor_id = 4
	motors[3].Tim = &htim3;
	motors[3].Channel = TIM_CHANNEL_2;

	// Motor_id = 5 (PINZA)
	motors[4].Tim = &htim3;
	motors[4].Channel = TIM_CHANNEL_3;
}

/*
 * @brief Funzione di "basso livello" per il controllo dei Servo Motori
 * @param value: Valore che indica l'angolo a cui impostare il servo-motore
 * @param motor_id: Valore che indica l'id del motore di cui andare a modificare l'angolo
 * @retval movement_error: 0 -> ok, 1 -> out_of_degree, 2 -> other
 *
 */
uint8_t set_degrees(uint16_t degrees, uint8_t motor_id)
{
	// Variabile di restituzione del corretto funzionamento
	// dei motori
	uint8_t movement_error = 0;

	// Andando a considerare il conteggio dei motori con un insieme N
	// da cui partiamo dal valore 1 a seguire

	// Consideriamo una prima configurazione della struttura dati che andrà
	// poi a configurare il sistema + il valore pulse. Tale valore verrà
	// assegnato direttamente al registro CCR del timer, che andrà a generare il PWM
	// in maniera del tutto parallela alle operazioni presenti sul microprocessore

	// Vado ad aggiungere lo sfasamento per rendere le funzioni più facilmenti utilizzabili

	// Sì esclude l'ID della pinza per evitare il setting di valori anomali
	if (motor_id > 0 && motor_id < MOTOR_NUM){
		if (degrees >= MIN_DEGREE_NORMAL && degrees <= MAX_DEGREE_NORMAL){
			TIM_OC_InitTypeDef sConfigOC;
			sConfigOC.OCMode = TIM_OCMODE_PWM1;
			sConfigOC.Pulse = degrees + 120;
			sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
			sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

			HAL_TIM_PWM_ConfigChannel(motors[motor_id - 1].Tim, &sConfigOC, motors[motor_id - 1].Channel);
			HAL_TIM_PWM_Start(motors[motor_id - 1].Tim, motors[motor_id - 1].Channel);
		}else{
			movement_error = 1;
		}
	}else if(motor_id == MOTOR_NUM){
		// Sì effettuano controlli diversi, per evitare la rottura del componente
		if (degrees >= MIN_DEGREE_PINZA && degrees <= MAX_DEGREE_PINZA){
			TIM_OC_InitTypeDef sConfigOC;
			sConfigOC.OCMode = TIM_OCMODE_PWM1;
			sConfigOC.Pulse = degrees + 120;
			sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
			sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

			HAL_TIM_PWM_ConfigChannel(motors[motor_id - 1].Tim, &sConfigOC, motors[motor_id - 1].Channel);
			HAL_TIM_PWM_Start(motors[motor_id - 1].Tim, motors[motor_id - 1].Channel);
		}else{
			movement_error = 1;
		}
	}

	return movement_error;
}

void close_pinza(){
	set_degrees(MAX_DEGREE_PINZA, MOTOR_NUM);
}

void open_pinza(){
	set_degrees(MIN_DEGREE_PINZA, MOTOR_NUM);
}
