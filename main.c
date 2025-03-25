/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
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
uint8_t counterA = 0;  // Contador para BT1 
uint8_t counterB = 0;  // Contador para BT2 
uint8_t estado = 1;

const uint8_t tabla_segmentos[10] = {
    0b0111111,  // 0
    0b0000110,  // 1
    0b1011011,  // 2
    0b1001111,  // 3
    0b1100110,  // 4
    0b1101101,  // 5
    0b1111101,  // 6
    0b0000111,  // 7
    0b1111111,  // 8
    0b1101111   // 9
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  	 {

	  if (estado == 1) {
		  // Mostrar cuentra regresiva en el Display
		  mostrar_numero(5);
		  HAL_Delay(1000);

		  mostrar_numero(4);
		  HAL_Delay(1000);

		  mostrar_numero(3);
		  HAL_Delay(1000);

		  mostrar_numero(2);
		  HAL_Delay(1000);

		  mostrar_numero(1);
		  HAL_Delay(1000);

		  mostrar_numero(0);

		  estado = 0;
	  } else {
		  //HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);

	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin|LED3_Pin|LED_Pin
                          |LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED5_Pin|LED6_Pin|LED7_Pin|SEG_G_Pin
                          |LED8_Pin|SEG_A_Pin|SEG_B_Pin|SEG_C_Pin
                          |SEG_D_Pin|SEG_E_Pin|SEG_F_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BTN_Pin BT1_Pin BT2_Pin */
  GPIO_InitStruct.Pin = BTN_Pin|BT1_Pin|BT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED_Pin
                           LED4_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|LED_Pin
                          |LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED5_Pin LED6_Pin LED7_Pin SEG_G_Pin
                           LED8_Pin SEG_A_Pin SEG_B_Pin SEG_C_Pin
                           SEG_D_Pin SEG_E_Pin SEG_F_Pin */
  GPIO_InitStruct.Pin = LED5_Pin|LED6_Pin|LED7_Pin|SEG_G_Pin
                          |LED8_Pin|SEG_A_Pin|SEG_B_Pin|SEG_C_Pin
                          |SEG_D_Pin|SEG_E_Pin|SEG_F_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* Interrupt callback function */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    static uint8_t winner = 0; // 0 = Ninguno ha ganado, 1 = BT1 ganó, 2 = BT2 ganó

    if (GPIO_Pin == BT2_Pin) {
        	// RESET
    		winner = 0;
        	counterA = 0;
        	counterB = 0;
        	GPIOA->ODR &= ~(LED1_Pin | LED2_Pin | LED3_Pin | LED4_Pin); // Apagar LEDs de J1
        	GPIOB->ODR &= ~(LED5_Pin | LED6_Pin | LED7_Pin | LED8_Pin); // Apagar LEDs de J2

        	// Iniciar cuenta regresiva en el Display
        	estado = 1; // Se iniciar en el While
        }


    if (winner == 0 && estado == 0) { // Solo se permite avanzar si nadie ha ganado y si ya acabó la cuenta regesiva
        if (GPIO_Pin == BT1_Pin) {  // Botón 1
            counterA = (counterA + 1) % 5; // Ciclo de 0 a 4

            // Limpiar LEDs antes de actualizar
            GPIOA->ODR &= ~(LED1_Pin | LED2_Pin | LED3_Pin | LED4_Pin);

            // Encender LEDs según counterA
            switch (counterA) {
                case 1: GPIOA->ODR |= LED1_Pin; break;
                case 2: GPIOA->ODR |= LED2_Pin; break;
                case 3: GPIOA->ODR |= LED3_Pin; break;
                case 4: GPIOA->ODR |= LED4_Pin; break;
                case 0: // Si llega a 5, gana BT1
                    GPIOA->ODR |= (LED1_Pin | LED2_Pin | LED3_Pin | LED4_Pin);
                    GPIOB->ODR &= ~(LED5_Pin | LED6_Pin | LED7_Pin | LED8_Pin); // Apagar LEDs de BT2
                    // Muestra en el Display el 1
                    mostrar_numero(1);
                    winner = 1;
                    break;
            }
        }
        else if (GPIO_Pin == BTN_Pin) {  // Botón 2
            counterB = (counterB + 1) % 5; // Ciclo de 0 a 4

            // Limpiar LEDs antes de actualizar
            GPIOB->ODR &= ~(LED5_Pin | LED6_Pin | LED7_Pin | LED8_Pin);

            // Encender LEDs según counterB
            switch (counterB) {
                case 1: GPIOB->ODR |= LED5_Pin; break;
                case 2: GPIOB->ODR |= LED6_Pin; break;
                case 3: GPIOB->ODR |= LED7_Pin; break;
                case 4: GPIOB->ODR |= LED8_Pin; break;
                case 0: // Si llega a 5, gana BT2
                    GPIOB->ODR |= (LED5_Pin | LED6_Pin | LED7_Pin | LED8_Pin);
                    GPIOA->ODR &= ~(LED1_Pin | LED2_Pin | LED3_Pin | LED4_Pin); // Apagar LEDs de BT1
                    // Muestra en el Display el 2
                    mostrar_numero(2);
                    winner = 2;
                    break;
            }
        }
    }

}

void mostrar_numero(uint8_t numero) {

	// Limpiar el puerto
	GPIOB->ODR |= (SEG_A_Pin | SEG_B_Pin | SEG_C_Pin | SEG_D_Pin | SEG_E_Pin | SEG_F_Pin | SEG_G_Pin);

	// Cargar el valor correspondiente al número en los pines PB4-PB10
	GPIOB->ODR &= ~(tabla_segmentos[numero] << 4);

}

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

#ifdef  USE_FULL_ASSERT
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
