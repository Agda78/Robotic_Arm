/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "app_bluenrg_2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "servo.h"
#include "arm_control.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
typedef union {
    float values[3];
    uint8_t bytes[12];
} SerialPacket;

SerialPacket packet;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char rx_buffer[64];
int rx_index = 0;
char temp_char;
float valori_ricevuti[3];

void parse_serial_data(char* str)
{
    // Esegue l'eco sulla seriale per debug (Python lo leggerà)
    char debug_msg[80];

    char *token = strtok(str, ",");
    int count = 0;

    while (token != NULL && count < 3) {
        valori_ricevuti[count++] = atof(token);
        token = strtok(NULL, ",");
    }

    if (count == 3) {
        // Conferma ricezione corretta
        //sprintf(debug_msg, "Ricevuti: %.2f, %.2f, %.2f\r\n", valori_ricevuti[0], valori_ricevuti[1], valori_ricevuti[2]);
        HAL_UART_Transmit(&huart2, (uint8_t*)debug_msg, strlen(debug_msg), 100);

        // Esegui il movimento
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        azionamento_braccio(valori_ricevuti);
    } else {
        char err_msg[] = "Errore: Dati incompleti\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)err_msg, strlen(err_msg), 100);
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  //MX_BlueNRG_2_Init();
  /* USER CODE BEGIN 2 */
  Motor_Init();
  posizionamento_iniziale(NULL);

//  static const float dataset_test[10][3] = {
//      { 0.12f, -3.05f,  9.82f}, // Campione 0
//      { 8.50f,  0.22f,  0.60f}, // Campione 1
//      { 5.44f, -8.10f,  7.80f}, // Campione 2
//      {-8.12f,  0.12f,  3.55f}, // Campione 3
//      { 0.01f,  9.78f,  0.15f}, // Campione 4
//      { 2.33f, -4.55f,  8.11f}, // Campione 5
//      {-1.20f,  0.00f, -9.81f}, // Campione 6
//      { 0.55f,  0.55f,  0.55f}, // Campione 7
//      { 9.00f,  1.20f,  1.10f}, // Campione 8
//      {-0.44f, -0.12f,  9.84f}  // Campione 9
//  };
//
//  uint8_t i = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

 // MX_BlueNRG_2_Process();
    /* USER CODE BEGIN 3 */

	  // Codice di esempio per motori
//	  HAL_StatusTypeDef status = HAL_UART_Receive(&huart2, packet.bytes, 12, 100);
//
//	  if (status == HAL_OK)
//	  {
//		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
//		  // Ora hai i dati convertiti automaticamente!
//		  float x = packet.values[0];
//		  float y = packet.values[1];
//		  float z = packet.values[2];
//
//		  // Passaggio dei dati dell'accellerometro
//		  azionamento_braccio(dataset_test[i++]);
//		  if (i == 10){
//			  i = 0;
//		  }
//		  // Qui puoi inserire la tua logica (es. controllo_sincrono)
//	  }
//	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//	  azionamento_braccio(dataset_test[i++]);
//	  if (i == 10){
//		  i = 0;
//	  }
//	  HAL_Delay(20000);

	  if (HAL_UART_Receive(&huart2, (uint8_t *)&temp_char, 1, 10) == HAL_OK)
	      {
	          // Se ricevo il terminatore di riga
	          if (temp_char == '\n' || temp_char == '\r')
	          {
	              if (rx_index > 0)
	              {
	                  rx_buffer[rx_index] = '\0'; // Chiudo la stringa
	                  parse_serial_data(rx_buffer);
	                  rx_index = 0; // Reset per la prossima riga
	              }
	          }
	          else
	          {
	              // Aggiungo il carattere al buffer se c'è spazio
	              if (rx_index < sizeof(rx_buffer) - 1)
	              {
	                  rx_buffer[rx_index++] = temp_char;
	              }
	          }
	      }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
