/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPI_BUFFER_SIZE 8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

uint8_t SPI_RX_Buffer[SPI_BUFFER_SIZE] = {127, 127, 127, 127, 127, 127, 127, 127};
uint8_t SPI_TX_Buffer[SPI_BUFFER_SIZE] = {0, 0, 0, 0, 0, 0, 0, 0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

void EnablePWMOutput(TIM_HandleTypeDef *_htim);

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
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /**
    * @brief  Receive an amount of data in non-blocking mode with Interrupt.
    * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
    *               the configuration information for SPI module.
    * @param  pData pointer to data buffer
    * @param  Size amount of data to be sent
    * @retval HAL status

  HAL_StatusTypeDef HAL_SPI_Receive_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size)
  */
  //HAL_SPI_Receive_IT(&hspi1, SPI_RX_Buffer, SPI_BUFFER_SIZE);
  //HAL_TIM_PWM_Start
  /*
  HAL_TIM_PWM_Start(&htim3, 1);
  HAL_TIM_PWM_Start(&htim3, 2);
  HAL_TIM_PWM_Start(&htim3, 3);
  HAL_TIM_PWM_Start(&htim3, 4);

  HAL_TIM_PWM_Start(&htim2, 1);
  HAL_TIM_PWM_Start(&htim2, 2);
  HAL_TIM_PWM_Start(&htim2, 3);
  HAL_TIM_PWM_Start(&htim2, 4);
*/

  EnablePWMOutput(&htim3);
  EnablePWMOutput(&htim2);
  HAL_Delay(10000);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //HAL_SPI_Receive_IT(&hspi1, SPI_RX_Buffer, SPI_BUFFER_SIZE);
  while (1)
  {
	  for (int i = 0; i < 250; i++)
	  {
		  htim2.Instance->CCR1 = (uint32_t) (i + 250);
		  htim2.Instance->CCR2 = (uint32_t) (i + 250);
		  htim2.Instance->CCR3 = (uint32_t) (i + 250);
		  htim2.Instance->CCR4 = (uint32_t) (i + 250);
		  HAL_Delay(50);
	  }

	  for (int i = 250; i > 0; i--)
	  {
	  	  htim2.Instance->CCR1 = (uint32_t) (i + 250);
	  	  htim2.Instance->CCR2 = (uint32_t) (i + 250);
	  	  htim2.Instance->CCR3 = (uint32_t) (i + 250);
	  	  htim2.Instance->CCR4 = (uint32_t) (i + 250);
	  	  HAL_Delay(50);
	  }
	  //HAL_IWDG_Refresh(&hiwdg);
	  //HAL_SPI_Receive_IT(&hspi1, SPI_RX_Buffer, SPI_BUFFER_SIZE);
	  //HAL_SPI_Receive(&hspi1, SPI_RX_Buffer, SPI_BUFFER_SIZE, 50000);
	  //HAL_NVIC_DisableIRQ(SPI1_IRQn);
	  //Data handling goes here

//	  htim3.Instance->CCR1 = (uint32_t) SPI_RX_Buffer[0] + 250;
//	  htim3.Instance->CCR2 = (uint32_t) SPI_RX_Buffer[1] + 250;
//	  htim3.Instance->CCR3 = (uint32_t) SPI_RX_Buffer[2] + 250;
//	  htim3.Instance->CCR4 = (uint32_t) SPI_RX_Buffer[3] + 250;
//
//	  htim2.Instance->CCR1 = (uint32_t) SPI_RX_Buffer[4] + 250;
//	  htim2.Instance->CCR2 = (uint32_t) SPI_RX_Buffer[5] + 250;
//	  htim2.Instance->CCR3 = (uint32_t) SPI_RX_Buffer[6] + 250;
//	  htim2.Instance->CCR4 = (uint32_t) SPI_RX_Buffer[7] + 250;

	  //HAL_NVIC_EnableIRQ(SPI1_IRQn);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 32-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 500-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 375;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 32-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 500-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 375;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

void EnablePWMOutput(TIM_HandleTypeDef *_htim) {
	//  Set HAL Timer Channel Status
	TIM_CHANNEL_STATE_SET_ALL(_htim, HAL_TIM_CHANNEL_STATE_BUSY);

	//  Enable outputs for all 4 PWM Channels
	_htim->Instance->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);

	//  Enable Timer Counter
	_htim->Instance->CR1 |= TIM_CR1_CEN;
}

#define NUM_THRUSTERS 8

enum tctp_message_type {
    ACK  = 0,
    NACK = 1,
    FULL_THRUST_CONTROL = 2,
    TOOLS_THRUST_CONTROL = 3
    /* Future expansion, may need a type for each "tool" */
};

uint16_t expected_msg_id = 0;

struct tctp_message {
    enum tctp_message_type type;

    uint16_t message_id;

    union {
        uint8_t full_thrust_values[NUM_THRUSTERS];

        /* TBD: Tools thrust values */

        uint8_t padding;
    } data;

    uint16_t crc;
};

/* CRC INIT */
// CRC_HandleTypeDef hcrc;
// hcrc.Instance = CRC;
// HAL_CRC_Init(&hcrc);

// Configure the CRC polynomial (e.g., CRC-CCITT)
// hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
// hcrc.Init.GeneratingPolynomial = 0x1021;

// uint8_t CRC_compare(struct tctp_message received_msg)
// {
//     HAL_CRC_Reset(&hcrc);
//     /* I think this will calculate the CRC of the message including the CRC in it, needs
//      * to just be calculating on everything before the CRC. Does the -2 fix it? */
//     HAL_CRC_Accumulate(&hcrc, (uint32_t*)&received_msg, ((sizeof(received_msg) - 2) / 4));
//
//     uint16_t received_crc = received_msg.crc;
//     uint16_t calculated_crc = HAL_CRC_Calculate(&hcrc);
//
//     return received_crc == calculated_crc;
// }

uint8_t tctp_handler(struct tctp_message received)
{
    /* Once we recieve this we need to calculate its CRC to determine if it is valid */
    struct tctp_message message = {
        .message_id = received.message_id,
        .data = {
            .padding = 0,
        },
        .crc = 0, /* Placeholder, we may have hardware calculate this for us */
    };

    // uint8_t message_correct = CRC_compare(received);
    uint8_t message_correct = 1;

    /* We need to make sure we are receiving the correct message */
    // message_correct = (expected_msg_id == received.message_id);

    /* maybe add back error codes to say what is wrong with the message,
     * ex: CRC wrong, unmatching msg ids etc */

    if (message_correct) {
        message.type = ACK;
    } else {
        message.type = NACK;
    }

    /* NOTE: Should we transmit the ACK after sending the data to the PWMs?
     * I don't think it would make that much of a difference either way but
     * maybe there is a chance that we get new data in before we end up 
     * sending it back out? Even if so it probably would not be a problem
     * because of the transmit rate */

    /* Transmit response */
    HAL_SPI_Transmit_IT(&hspi1, (uint8_t*)&message, sizeof(message));

    expected_msg_id++;

    return message_correct;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi)
{
    /* Pull SPI message into RX buffer */
    HAL_SPI_Receive_IT(&hspi1, SPI_RX_Buffer, SPI_BUFFER_SIZE);
    struct tctp_message* received_msg = (struct tctp_message*) SPI_RX_Buffer;
    uint8_t message_correct = tctp_handler(*received_msg);
    /* Need to confirm the type is FULL_THRUST_CONTROL before we do this */
    // uint8_t received_payload[NUM_THRUSTERS] = received_msg->data.full_thrust_values;
    uint8_t received_payload[NUM_THRUSTERS];
    memcpy(received_payload, received_msg->data.full_thrust_values, NUM_THRUSTERS);

    /* NOTE: Cannot do a guard clause here because this is an interrupt handler */
    /* Send data to PWMs */
    if (message_correct) {
        htim3.Instance->CCR1 = (uint32_t) received_payload[0] + 250;
        htim3.Instance->CCR2 = (uint32_t) received_payload[1] + 250;
        htim3.Instance->CCR3 = (uint32_t) received_payload[2] + 250;
        htim3.Instance->CCR4 = (uint32_t) received_payload[3] + 250;

        htim2.Instance->CCR1 = (uint32_t) received_payload[4] + 250;
        htim2.Instance->CCR2 = (uint32_t) received_payload[5] + 250;
        htim2.Instance->CCR3 = (uint32_t) received_payload[6] + 250;
    }

	// htim3.Instance->CCR1 = (uint32_t) SPI_RX_Buffer[0] + 250;
	// htim3.Instance->CCR2 = (uint32_t) SPI_RX_Buffer[1] + 250;
	// htim3.Instance->CCR3 = (uint32_t) SPI_RX_Buffer[2] + 250;
	// htim3.Instance->CCR4 = (uint32_t) SPI_RX_Buffer[3] + 250;
	//
	// htim2.Instance->CCR1 = (uint32_t) SPI_RX_Buffer[4] + 250;
	// htim2.Instance->CCR2 = (uint32_t) SPI_RX_Buffer[5] + 250;
	// htim2.Instance->CCR3 = (uint32_t) SPI_RX_Buffer[6] + 250;
	// htim2.Instance->CCR4 = (uint32_t) SPI_RX_Buffer[7] + 250;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
