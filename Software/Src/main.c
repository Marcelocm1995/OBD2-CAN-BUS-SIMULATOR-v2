/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stdbool.h"
#include "PIDs.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define NUMBER_OF_RX_PIDs canRX[0]
#define NUMBER_OF_TX_PIDs canTX_msg[1]

#define FIRST_RX_PID canRX[2]
#define SECOND_RX_PID canRX[3]
#define THIRD_RX_PID canRX[4]

#define ONE_PID 0x02
#define TWO_PID 0x03
#define THREE_PID 0x04

#define RX_ID 0x7DF
#define TX_ID 0x7E8

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

CAN_RxHeaderTypeDef rxHeader;
CAN_TxHeaderTypeDef txHeader;

CAN_FilterTypeDef canfil;

uint32_t canMailbox;

uint8_t canRX[8];
uint8_t canTX[8];

uint8_t canTX_msg[8]; //create CAN TX data message

uint8_t Rx_Nextion[20];
bool NextionRxMsg = 0;

uint8_t PAGE_NEXTION;

uint8_t INTAKE_PRESSURE, 
				INTAKE_TEMPERATURE,
				ENGINE_COOLANT,
				LAMBDA_NARROW,
				LAMBDA_WIDE_1,
				LAMBDA_WIDE_2,
				LAMBDA_WIDE_3,
				IGNITION_TIMING,
				THROTTLE_POS;
				
uint16_t	FUEL_RAIL;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

float Map (float inVal, float inMin, float inMax, float outMin, float outMax);
void filter_can_config(void);
//void send_canmsg(unsigned char pid);
void send_canmsg(void);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void Parse_Nextion(void);
void parse_data(unsigned char pid, uint8_t *A, uint8_t *B, uint8_t *C, uint8_t *D, uint8_t *size);
void Page_0(void);
void Page_1(void);
void Page_2(void);
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
	HAL_Delay(5000);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_UART_Receive_IT(&huart3, Rx_Nextion, 11);
	
	/*----Config filter and start CAN interface------*/
	filter_can_config();
	HAL_CAN_ConfigFilter(&hcan, &canfil);
	HAL_CAN_Start(&hcan);
	//HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	
	/*----Configure CAN TX messages------------*/
	txHeader.DLC = 8;
	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.StdId = TX_ID;
	txHeader.ExtId = 0x02;
	txHeader.TransmitGlobalTime = DISABLE;
	
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(NextionRxMsg == 1)
		{
			Parse_Nextion();
			NextionRxMsg = 0;
		}

    if(rxHeader.StdId == RX_ID) //Verify if RX ID is 0x7DF
    {
				//Create_CAN_Msg();
				//send_canmsg(canRX[2]);  //Call function that send CAN message
				send_canmsg();					//Call function that send CAN message
				rxHeader.StdId = 0x00;	//Clear RX ID for don't enter in this "if" on next loop
				HAL_Delay(50);					//Some delay for my application not go to fast, we can comment this line for faster messages
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	NextionRxMsg = 1;
	HAL_UART_Receive_IT(&huart3, Rx_Nextion, 11);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, canRX); //Receive CAN bus message to canRX buffer in INTERRUPT MODE
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);// toggle PC13 LED
}

void Page_0()
{
	switch (Rx_Nextion[2])
	{
		case 1: INTAKE_PRESSURE = Rx_Nextion[7]; break;
		case 2: INTAKE_TEMPERATURE = Rx_Nextion[7]; break;
		case 3: ENGINE_COOLANT = Rx_Nextion[7]; break;
		case 4: LAMBDA_NARROW = Rx_Nextion[7]; break;
		case 5: FUEL_RAIL = Rx_Nextion[7] + (Rx_Nextion[8] * 256); break;
	}
}

void Page_1()
{
	switch (Rx_Nextion[2])
	{
		case 1: IGNITION_TIMING = Rx_Nextion[7]; break;
		case 2: THROTTLE_POS = Rx_Nextion[7]; break;
	}
}

void Page_2()
{
	switch (Rx_Nextion[2])
	{
		case 1: ; break;
		case 2: ; break;
	}
}

void Parse_Nextion()
{
	switch (Rx_Nextion[1])
	{
		case 0: Page_0(); break;
		case 1: Page_1(); break;
		case 2: Page_2(); break;
	}
}
	
void send_canmsg()
{
		uint8_t A, B, C, D, SIZE_OF_FIRST_PID, SIZE_OF_SECOND_PID, SIZE_OF_THIRD_PID;
		canTX_msg[0] = 0x41;
	
		if(NUMBER_OF_RX_PIDs == ONE_PID)
		{
			NUMBER_OF_TX_PIDs = NUMBER_OF_RX_PIDs;
			parse_data(FIRST_RX_PID, &A, &B, &C, &D, &SIZE_OF_FIRST_PID);
			canTX_msg[2] = FIRST_RX_PID;
			canTX_msg[3] = A;
			canTX_msg[4] = B;
			canTX_msg[5] = C;
			canTX_msg[6] = D;		
			canTX_msg[7] = NULL;			
		}
		
		if(NUMBER_OF_RX_PIDs == TWO_PID)
		{
			NUMBER_OF_TX_PIDs = NUMBER_OF_RX_PIDs;
			canTX_msg[2] = FIRST_RX_PID;
			parse_data(FIRST_RX_PID, &A, &B, &C, &D, &SIZE_OF_FIRST_PID);
			if(SIZE_OF_FIRST_PID == 2)
			{
				canTX_msg[3] = A;
				canTX_msg[4] = B;
				canTX_msg[5] = SECOND_RX_PID;
				parse_data(SECOND_RX_PID, &A, &B, &C, &D, &SIZE_OF_SECOND_PID);
				canTX_msg[6] = A;
				canTX_msg[7] = B;	
			}
			else
			{
				canTX_msg[3] = A;
				canTX_msg[4] = SECOND_RX_PID;
				parse_data(SECOND_RX_PID, &A, &B, &C, &D, &SIZE_OF_SECOND_PID);
				canTX_msg[5] = A;
				canTX_msg[6] = B;	
				canTX_msg[7] = NULL;	
			}
		}
		
		if(NUMBER_OF_RX_PIDs == THREE_PID)
		{
			NUMBER_OF_TX_PIDs = NUMBER_OF_RX_PIDs;
			canTX_msg[2] = FIRST_RX_PID;
			parse_data(FIRST_RX_PID, &A, &B, &C, &D, &SIZE_OF_FIRST_PID);
			if(SIZE_OF_FIRST_PID == 2)
			{
				canTX_msg[3] = A;
				canTX_msg[4] = B;
				canTX_msg[5] = SECOND_RX_PID;
				parse_data(SECOND_RX_PID, &A, &B, &C, &D, &SIZE_OF_SECOND_PID);
				canTX_msg[6] = A;
				canTX_msg[7] = B;	
			}
			else
			{
				canTX_msg[3] = A;
				canTX_msg[4] = SECOND_RX_PID;
				parse_data(SECOND_RX_PID, &A, &B, &C, &D, &SIZE_OF_SECOND_PID);
				if(SIZE_OF_SECOND_PID == 2)
				{
					canTX_msg[5] = A;
					canTX_msg[6] = B;	
					canTX_msg[7] = NULL;	
				}
				else
				{
					canTX_msg[5] = A;
					canTX_msg[6] = THIRD_RX_PID;
					parse_data(THIRD_RX_PID, &A, &B, &C, &D, &SIZE_OF_THIRD_PID);
					canTX_msg[7] = A;
				}
			}
		}
		
		HAL_CAN_AddTxMessage(&hcan, &txHeader, canTX_msg, &canMailbox);//send TX CAN message
	
		memset(canTX_msg, 0x00, 8);
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);// toggle PC13 LED
}

void parse_data(unsigned char pid, uint8_t *A, uint8_t *B, uint8_t *C, uint8_t *D, uint8_t *size)
{
    switch (pid) 
    {
        case PID_RPM:
        case PID_EVAP_SYS_VAPOR_PRESSURE: // kPa
            break;
        case PID_FUEL_PRESSURE: // kPa
            break;
        case PID_COOLANT_TEMP: //celsius
            *A = ENGINE_COOLANT; *B = 0x00; *C = 0x00; *D = 0x00; *size = 1;
            break;
        case PID_OXYGEN_SENSOR: //milivolts
            *A = LAMBDA_NARROW;  *B = 0x00; *C = 0x00; *D = 0x00; *size = 2;
            break;
        case PID_INTAKE_TEMP: //celsius
            *A = INTAKE_TEMPERATURE; *B = 0x00; *C = 0x00; *D = 0x00; *size = 1;
            break;
        case PID_INTAKE_MAP: //kpa
            *A = INTAKE_PRESSURE; *B = 0x00; *C = 0x00; *D = 0x00; *size = 1;
            break;
        case PID_AMBIENT_TEMP:
        case PID_ENGINE_OIL_TEMP://celsius
            break;
        case PID_THROTTLE: //percent
            *A = THROTTLE_POS; *B = 0x00; *C = 0x00; *D = 0x00; *size = 1;
            break;
        case PID_SPEED: //km/h
            break;
        case PID_COMMANDED_EGR:
        case PID_COMMANDED_EVAPORATIVE_PURGE:
        case PID_FUEL_LEVEL:
        case PID_RELATIVE_THROTTLE_POS:
        case PID_ABSOLUTE_THROTTLE_POS_B:
        case PID_ABSOLUTE_THROTTLE_POS_C:
        case PID_ACC_PEDAL_POS_D:
        case PID_ACC_PEDAL_POS_E:
        case PID_ACC_PEDAL_POS_F:
        case PID_COMMANDED_THROTTLE_ACTUATOR:
        case PID_ENGINE_LOAD://percent
            break;
        case PID_ABSOLUTE_ENGINE_LOAD:
        case PID_ETHANOL_FUEL:
        case PID_HYBRID_BATTERY_PERCENTAGE:
        case PID_MAF_FLOW: // grams/sec
            break;
        case PID_TIMING_ADVANCE: //degree
            *A = IGNITION_TIMING; *B = 0x00; *C = 0x00; *D = 0x00; *size = 1;
            break;
        case PID_DISTANCE: // km
        case PID_DISTANCE_WITH_MIL: // km
        case PID_TIME_WITH_MIL: // minute
        case PID_TIME_SINCE_CODES_CLEARED: // minute
        case PID_RUNTIME: // second
        case PID_FUEL_RAIL_PRESSURE:// kPa
            *A = (FUEL_RAIL >> 8) & 0xFF; *B = (FUEL_RAIL >> 0) & 0xFF; *C = 0x00; *D = 0x00; *size = 2;
            break;
        case PID_ENGINE_REF_TORQUE: // Nm
            break;
        case PID_CONTROL_MODULE_VOLTAGE: // V
            break;
        case PID_ENGINE_FUEL_RATE: // L/h
            break;
        case PID_ENGINE_TORQUE_DEMANDED: // %
        case PID_ENGINE_TORQUE_PERCENTAGE: // %
            break;
        case PID_SHORT_TERM_FUEL_TRIM_1:
        case PID_LONG_TERM_FUEL_TRIM_1:
        case PID_SHORT_TERM_FUEL_TRIM_2:
        case PID_LONG_TERM_FUEL_TRIM_2:
        case PID_EGR_ERROR:
            break;
        case PID_FUEL_INJECTION_TIMING:
            break;
        case PID_CATALYST_TEMP_B1S1:
        case PID_CATALYST_TEMP_B2S1:
        case PID_CATALYST_TEMP_B1S2:
        case PID_CATALYST_TEMP_B2S2:
            break;
        case PID_AIR_FUEL_EQUIV_RATIO_1: // 0~200
            break;
				case PID_AIR_FUEL_EQUIV_RATIO_2: // 0~200
            break;
				case PID_AIR_FUEL_EQUIV_RATIO_3: // 0~200
            break;
        default:
            *A = 0x00; *B = 0x00; *C = 0x00; *D = 0x00; 
    }//fim do case
}

void filter_can_config()
{
	//https://www.esacademy.com/en/library/calculators/can-best-and-worst-case-calculator.html
	//https://schulz-m.github.io/2017/03/23/stm32-can-id-filter/
	canfil.FilterBank = 0;
	canfil.FilterMode = CAN_FILTERMODE_IDMASK;
	canfil.FilterFIFOAssignment = CAN_RX_FIFO0;

	canfil.FilterIdHigh = ((RX_ID << 5)  | (RX_ID >> (32 - 5))) & 0xFFFF; // STID[10:0] & EXTID[17:13]
	canfil.FilterIdLow = (RX_ID >> (11 - 3)) & 0xFFF8; // EXID[12:5] & 3 Reserved bits
	
	canfil.FilterMaskIdHigh = ((0x1FFFFFFF << 5)  | (0x1FFFFFFF >> (32 - 5))) & 0xFFFF;
  canfil.FilterMaskIdLow = (0x1FFFFFFF >> (11 - 3)) & 0xFFF8;
	
	canfil.FilterScale = CAN_FILTERSCALE_32BIT;
	canfil.FilterActivation = ENABLE;
	canfil.SlaveStartFilterBank = 14;
}

float Map (float inVal, float inMin, float inMax, float outMin, float outMax)
{
	return ( (inVal - inMin)*(outMax - outMin)/(inMax - inMin) + outMin );
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
