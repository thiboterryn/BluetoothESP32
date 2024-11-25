/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stdlib.h"
#include "string.h"
#include "../../MDK-ARM/usart1.h"
#include "../../MDK-ARM/usart2.h"
#include "stdio.h"
#include "../../MDK-ARM/circularBuffer.h"
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
IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static volatile enum UsartState usart1State =	idle;
// Verwijzing maken naar een circulair buffer dat alle binnenkomende data van de ESP-module opslaat.
extern volatile CircularBuffer circularBuffer;
extern enum CircularBufferActionResult circularBufferActionResult;
static uint8_t leds = 0, numberOfReceivedBytes = 0;

static volatile uint8_t receivedUsart1Data = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
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
  MX_IWDG_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	// Laten weten dat we opgestart zijn, via de USART2 (USB).
	printf("Reboot\r\n");	
	// Circulaire buffer initialiseren.
	if(InitCircularBuffer(&circularBuffer) != initSucceeded)
		printf("Initialization of CircularBuffer failed!\r\n");
	
	// WiFi-module tijd geven om op te starten.
	HAL_Delay(1000);
	
	// Ontvangst van één byte via interrupts starten (voor de ESP32-module).
	HAL_UART_Receive_IT(&huart1, (uint8_t*)&receivedUsart1Data, 1);
	
	
	
	/* 
	https://docs.espressif.com/projects/esp-at/en/latest/esp32c3/AT_Command_Set/BLE_AT_Commands.html#cmd-badvstart
	*/
	
		// Reset watchdog
    HAL_IWDG_Refresh(&hiwdg);
	
		// zet esp module in AT mode en test communicatie
    StringToUsart1("AT\r\n");
    while(!LookForString1(&circularBuffer, "OK\r\n"));
    HAL_Delay(100);
    
    // Reset esp module
    StringToUsart1("AT+RST\r\n");
    while(!LookForString1(&circularBuffer, "ready\r\n"));
    HAL_Delay(1000); 
		HAL_IWDG_Refresh(&hiwdg);
		
		// Initialiseer BLE als GATT-server
		StringToUsart1("AT+BLEINIT=2\r\n");
    while(!LookForString1(&circularBuffer, "OK\r\n")); 
		HAL_Delay(100);

    // GATT-server aanmaken
    StringToUsart1("AT+BLEGATTSSRVCRE\r\n");
    while(!LookForString1(&circularBuffer, "OK\r\n"));
		HAL_Delay(100);

	  // GATT-server starten
    StringToUsart1("AT+BLEGATTSSRVSTART\r\n");
    while(!LookForString1(&circularBuffer, "OK\r\n"));
		HAL_Delay(100);

		// BLENAME instellen werkt niet... naam ingesteld in BLEADVDATA
//    StringToUsart1("AT+BLENAME=\"GlucoseMeter\"\r\n");
//    while(!LookForString1(&circularBuffer, "OK\r\n"));
//		HAL_Delay(100);

		//AT+BLEADVDATA: Set Bluetooth LE Advertising Data
		/*AT+BLEADVDATAEX=<dev_name>,<uuid>,<manufacturer_data>,<include_power>
		dev_name = GlucoseMeter  				# dit is de naam die je ziet om het toestel te vinden
		uuid= 00A2 											# Ziet er in bluetooth app anders uit?
		manufacturer_data= 5669766573 	# heximaal Vives
		include_power= 1 								# zendkracht mag ook 0 zijn
		*/
		StringToUsart1("AT+BLEADVDATAEX=\"GlucoseMeter\",\"00A2\",\"5669766573\",1\r\n");
    while(!LookForString1(&circularBuffer, "OK\r\n"));
		HAL_Delay(100);

    // BLEADVDATA opvragen
//    StringToUsart1("AT+BLEADVDATAEX?\r\n");
//    while(!LookForString1(&circularBuffer, "OK\r\n"));
//		HAL_Delay(100);
		
    // Advertentie starten
    StringToUsart1("AT+BLEADVSTART\r\n");
    while(!LookForString1(&circularBuffer, "OK\r\n"));
		HAL_Delay(100);
		
		// List services on GATT Server:
    StringToUsart1("AT+BLEGATTSCHAR?\r\n");
    //while(!LookForString1(&circularBuffer, "OK\r\n"));
		HAL_Delay(100);
		
		// List characteristics on GATT Server:
    StringToUsart1("AT+BLEGATTSSRV?\r\n");
    //while(!LookForString1(&circularBuffer, "OK\r\n"));
		HAL_Delay(100);
		
		// Set one byte of first characteristic at first service
    StringToUsart1("AT+BLEGATTSSETATTR=1,1,,1\r\n");
    //while(!LookForString1(&circularBuffer, "OK\r\n"));
		HAL_Delay(100);
		
		// Karakteristiek toevoegen aan de GATT-service
//		StringToUsart1("AT+BLEGATTSCHAR:0x0001,0xFFF1,0x02,0x01,10\r\n");
//		while(!LookForString1(&circularBuffer, "OK\r\n"));
//		HAL_Delay(1000);
 


 
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//watchdog timer resetten.
		HAL_IWDG_Refresh(&hiwdg);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ESP32_RST_GPIO_Port, ESP32_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ESP32_RST_Pin LD3_Pin */
  GPIO_InitStruct.Pin = ESP32_RST_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Code hieronder wordt opgeroepen als alle data van HAL_UART_Receive_IT(() ontvangen is...
// Deze functie wordt opgeroepen vanuit HAL_UART_IRQHandler().
// De data is afkomstig van de ESP01-C3-module.
// Alle data wordt opgeslaan in de globale variabele 'circularBuffer', die gedefinieerd is in circularBuffer.c.
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	// Is het data van UART1?
	if(UartHandle->Instance == USART1)
	{		
		// Byte versturen op USART2 (USB) zodat je ook op de computer kan meevolgen. Dit is niet verplicht, maar vergemakkelijkt het debuggen.
		// Let op: zorg voor een timeout van 0, anders kan de code hier te traag verlopen en problemen bezorgen op UART1.
		HAL_UART_Transmit(&huart2, (uint8_t *)&receivedUsart1Data, 1, 0);
//		// Indien timing problemen, zou onderstaande kunnen helpen...
//		huart2.Instance->TDR = (uint8_t)receivedUsart1Data;
		
		// Ontvangen data opslaan in een circulair buffer. 
		// Het circulair buffer zorgt ervoor dat er geen bytes misgelopen raken. Je kan daarna in de hoofdlus 'rustig aan' alle
		// ontvangen data gaan verwerken...
		if(usart1State == busyReceiving)
		{
			// Is er nog plaats in het buffer?
			if(!IsCircularBufferFull(&circularBuffer))
			{
				// Karakter opslaan.
				PushCharToCircularBuffer(&circularBuffer, receivedUsart1Data);

				// Nieuwe lijn ontvangen. Geef aan dat je klaar bent voor een volgende lijn...
				if(receivedUsart1Data == '\n')
						usart1State = idle;
			}
		}
		else
		{
			// Start van de ontvangst/lijn.
			if(usart1State == idle)
			{
				// Is er nog plaats in het buffer?
				if(!IsCircularBufferFull(&circularBuffer))
				{
					// Buffer nog niet vol. Begin met opslaan in de circulaire buffer.
					usart1State = busyReceiving;
 
					// Karakter opslaan.
					PushCharToCircularBuffer(&circularBuffer, receivedUsart1Data);
					
					// Nieuwe lijn ontvangen. Geef aan dat je klaar bent voor een volgende lijn...
					if(receivedUsart1Data == '\n')
						usart1State = idle;
					else
					{					
						// Na het versturen van het 'AT+CIPSEND'- of een 'MQTTPUBRAW'-command, moet er gewacht worden op het '>'-karakter. Detecteer
						// de ontvangst van dat karakter hier. Voeg automatisch een "\r\n" toe om zo geforceerd een nieuwe lijn in het circulair buffer
						// aan te maken. Hierdoor kan de data uit dat buffer, gemakkelijker verwerkt worden.
						if(receivedUsart1Data == '>')
						{							
							// Geforceerd extra lijn opslaan met ">\r\n".
							PushCharToCircularBuffer(&circularBuffer, '\r');
							PushCharToCircularBuffer(&circularBuffer, '\n');
							
							usart1State = idle;
						}
					}
				}
			}				
		}		
		
		// Ontvangst via interrupts opnieuw starten. Want RXNEIE wordt automatisch uitgeschakeld.
		HAL_UART_Receive_IT(&huart1, (uint8_t*)&receivedUsart1Data, 1);
	}
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
