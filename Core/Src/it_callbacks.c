#include "app_bluenrg_2.h"
#include "servo.h"
#include "arm_control.h"

/*
 * ISR per la gestione della sicurezza
 * Tale ISR viene implementata per fermare il movimento del braccio qualora questo riceva una segnalazione
 * in falling edge dalla parte di controllo ambientale e sicurezza operatore. Tale interrupt ha lo scopo di
 * modificare la variabile safety_variable che andrà a bloccare il movimento del braccio
*/
volatile uint8_t safety_variable = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// Dato che l'handler è lo stesso di altri pin, la callback va a verificare se sia
	// effettivo il cambiamento del segnale
	if (GPIO_Pin == Safety_Set_Pin){
		// Caso in cui il segnale di segnalazione si è abbassato e bisogna fermare tutto
		safety_variable = 1;
		HAL_GPIO_WritePin(Test_PIN_GPIO_Port, Test_PIN_Pin, GPIO_PIN_SET);
	}else if (GPIO_Pin == Safety_Reset_Pin){
		// Caso in cui arriva un segnale per la ripresa dell'operatività
		if (HAL_GPIO_ReadPin(Safety_Set_GPIO_Port, Safety_Set_Pin) == GPIO_PIN_SET && HAL_GPIO_ReadPin(BCI_Set_GPIO_Port, BCI_Set_Pin) == GPIO_PIN_SET){
			// Check per notare se il segnale di sicurezza sia funzionante o meno, prima di ri-attivare il braccio
			safety_variable = 0;
			HAL_GPIO_WritePin(Test_PIN_GPIO_Port, Test_PIN_Pin, GPIO_PIN_RESET);
		}
	}else if (GPIO_Pin == BCI_Set_Pin){
		// Caso in cui il BCI passivo da segnale di non adeguatezza
		// per segnalazione di non adeguatezza dello stato dell'operatore
		safety_variable = 1;
	}else if (GPIO_Pin == BCI_Reset_Pin){
		// Caso di arrivo di una recovery
		if (HAL_GPIO_ReadPin(BCI_Set_GPIO_Port, BCI_Set_Pin) == GPIO_PIN_SET && HAL_GPIO_ReadPin(Safety_Set_GPIO_Port, Safety_Set_Pin) == GPIO_PIN_SET){
			safety_variable = 0;
		}
	}else if (GPIO_Pin == EF_Control_Pin){
		// Implementazione del Toggle effettivo della pinza

		// Primo If di sicurezza, anche se con la safety_variable dovrebbe essere fixed, il problam rimane il caso estremo
		// di segnalazione durante un primo arrivo di tale ISR
		if (HAL_GPIO_ReadPin(BCI_Set_GPIO_Port, BCI_Set_Pin) == GPIO_PIN_SET && HAL_GPIO_ReadPin(Safety_Set_GPIO_Port, Safety_Set_Pin) == GPIO_PIN_SET){
			if (movimento_braccio == 0){
				// Toggle dell'Intenzione
				// Problema di gestione del movimento del braccio
				toggle_pinza();
			}
		}
	}
	// HAL_GPIO_EXTI_IRQHandler(GPIO_Pin);
}

uint8_t n_executions = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	static float buffer_acc[3] = {0.0f, 0.0f, 0.0f};

	if (htim->Instance == TIM4){
		if (ready_data == 1) {
			n_executions = 0;
			for (uint8_t i = 0; i < 3; i++){
				buffer_acc[i] = valori_ricevuti[i];
			}
			ready_data = 0;
		}else{
			n_executions++;
		}

		// Tale if permette di evitare che nel caso di disconnessione della sensor tile, il braccio continui
		// a muoversi
		if (n_executions <= TIMES_TIMEOUT){
			azionamento_braccio_filtrato(buffer_acc);
		}

		// HAL_GPIO_TogglePin(Test_PIN_GPIO_Port, Test_PIN_Pin);

		htim->State = HAL_TIM_STATE_READY;
		HAL_TIM_Base_Start_IT(htim);
	}
}
