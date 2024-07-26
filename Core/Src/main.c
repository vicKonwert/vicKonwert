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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#include "File_Handling_RTOS.h"
#include "can.h"
//#include "int_rtc.h"
#include "timestamp.h"

#include "string.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint16_t data0[90]={0};

#define WHEEL_RPM                       data0[0]
#define WHEEL_R                         data0[1]
#define BATTERY_VOLTAGE                 data0[2]
#define BATTERY_CURRENT                 data0[3]
#define THROTTLE_POSITION               data0[4]
#define MODE_AND_LOCK_STATE             data0[5]
#define PARKING_INDICATION              data0[6]
#define REVERSE                         data0[7]
#define OVER_CURRENT                    data0[8]
#define OVERLOAD_ERROR                  data0[9]
#define LOCK1                           data0[10]
#define VEHICLE_ANTI_THEFT_SYSTEM       data0[11]
#define E_ABS                           data0[12]
#define CHARGING                        data0[13]
#define REGENERATIVE_BRAKING1           data0[14]
#define THROTTLE_ERROR                  data0[15]
#define MCU_ERROR                       data0[16]
#define MOTOR_ERROR                     data0[17]
#define BRAKE_ANY                       data0[18]
#define TEMPERATURE                     data0[19]
#define LOCK2                           data0[20]
#define FWD_GEAR                        data0[21]
#define NEUTRAL_GEAR                    data0[22]
#define SPEED_CONTROL_STATUS            data0[23]
#define CURRENT_CONTROL_STATUS          data0[24]
#define UNDER_VOLTAGE_STATUS            data0[25]
#define OVER_VOLTAGE_STATUS             data0[26]
#define REGENERATIVE_BRAKING_STATUS     data0[27]
#define REGENERATIVE_DURING_BRAKING_STATUS data0[28]
#define TORQUE_CONTROL_STATUS           data0[29]
#define MODE_AND_STATE_CONTROL_SECTION  data0[30]
#define E_ABS2                          data0[31]
#define REGENERATIVE_BRAKING2           data0[32]
#define LOCK3                           data0[33]
#define MAXIMUM_SPEED_CONTROL           data0[34]
#define CURRENT_LIMIT                   data0[35]
#define UNDERVOLTAGE_LIMIT              data0[36]
#define OVERVOLTAGE_LIMIT               data0[37]
#define REGEN_BRA_AT_ZERO_THROTTLE      data0[38]
#define RE_BR_AT_ZERO_THROTTLE          data0[39]

// RevxxData mappings
#define SOC                 data0[40]
#define STATE_OF_HEALTH     data0[41]
#define CURRENT_STATE       data0[42]
#define PREV_STATE          data0[43]
#define AVAILABLECAPACITY   data0[44]
#define BMSID               data0[45]
#define H_VERSION           data0[46]
#define S_VERSION           data0[47]
#define CELLVOL1            data0[48]
#define CELLVOL2            data0[49]
#define CELLVOL3            data0[50]
#define CELLVOL4            data0[51]
#define CELLVOL5            data0[52]
#define CELLVOL6            data0[53]
#define CELLVOL7            data0[54]
#define CELLVOL8            data0[55]
#define CELLVOL9            data0[56]
#define CELLVOL10           data0[57]
#define CELLVOL11           data0[58]
#define CELLVOL12           data0[59]
#define CELLVOL13           data0[60]
#define CELLVOL14           data0[61]
#define CELLVOL15           data0[62]
#define CELLVOL16           data0[63]
#define CELLVOL17           data0[64]
#define CELLVOL18           data0[65]
#define CELLVOL19           data0[66]
#define CELLVOL20           data0[67]
#define CELLTEMP1           data0[68]
#define CELLTEMP2           data0[69]
#define CELLTEMP3           data0[70]
#define CELLTEMP4           data0[71]
#define CELLTEMP5           data0[72]
#define CELLTEMP6           data0[73]
#define CELLTEMP7           data0[74]
#define CELLTEMP8           data0[75]
#define CELLTEMP9           data0[76]
#define CELLTEMP10          data0[77]
#define CELLTEMP11          data0[78]
#define CELLTEMP12          data0[79]
#define CELLTEMP13          data0[80]
#define CELLTEMP14          data0[81]
#define CELLTEMP15          data0[82]
#define CELLTEMP16          data0[83]
#define CELLTEMP17          data0[84]
#define CELLTEMP18          data0[85]
#define TOTAL_VOLTAGE       data0[86]
#define CURRENT             data0[87]
#define POWER               data0[88]
#define BATTERY_FAILURE     data0[89]
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

CAN_TxHeaderTypeDef TxHeader;

CAN_RxHeaderTypeDef RxHeader;

uint8_t txData[8];

uint8_t rxData[8];

uint32_t mailbox0;

extern char time[30];
extern char date[30];

extern RTC_DateTypeDef gDate;
extern RTC_TimeTypeDef gTime;

extern uint32_t seconds;
extern uint32_t milliseconds;
extern uint32_t milliseconds_display;
extern uint8_t running;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
extern void Start_Stopwatch(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
xTaskHandle SD_TASK_Handler;
xTaskHandle UART_TASK_Handler;

//void UART_TASK(void *arguments){
//	while(1){
////	char *buffer = pvPortMalloc(2000*sizeof(char));
//
////	sprintf(buffer,
////	    "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
////	    data0[0], data0[1], data0[2], data0[3], data0[4], data0[5], data0[6], data0[7], data0[8], data0[9], data0[10], data0[11],
////	    data0[12], data0[13], data0[14], data0[15], data0[16], data0[17], data0[18], data0[19], data0[20], data0[21], data0[22], data0[23],
////	    data0[24], data0[25], data0[26], data0[27], data0[28], data0[29], data0[30], data0[31], data0[32], data0[33], data0[34], data0[35],
////	    data0[36], data0[37], data0[38], data0[39], data0[40], data0[41], data0[42], data0[43], data0[44], data0[45], data0[46], data0[47],
////	    data0[48], data0[49], data0[50], data0[51], data0[52], data0[53], data0[54], data0[55], data0[56], data0[57], data0[58], data0[59],
////	    data0[60], data0[61], data0[62], data0[63], data0[64], data0[65], data0[66], data0[67], data0[68], data0[69], data0[70], data0[71],
////	    data0[72], data0[73], data0[74], data0[75], data0[76], data0[77], data0[78], data0[79], data0[80], data0[81], data0[82], data0[83],
////	    data0[84], data0[85], data0[86], data0[87], data0[88], data0[89]
////	);
////	sprintf(buffer, "%02d-%02d-%04d_%02d-%02d_KonwertLog.txt\r\n",
////	               gDate.Date, gDate.Month, 2000 + gDate.Year, gTime.Hours, gTime.Minutes);
//
//	HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
////	HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
//
//	vPortFree(buffer);
//	vTaskDelay(pdMS_TO_TICKS(1000)); // Delay 10 milliseconds
//
//	}
//}

//char *buffer;  // Declare the pointer variable
//char buffer[2000];  // Static allocation

void SD_TASK(void *arguments){

		int indx=1;
		int flag=1;

//		char *buffer = pvPortMalloc(3000*sizeof(char));

		char buffer[1500];
		while (1)
		{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
		if(flag==1){
			sprintf(buffer, " INDEX,TIMESTAMP, WHEEL_RPM, WHEEL_R, BATTERY_VOLTAGE, BATTERY_CURRENT, THROTTLE_POSITION, MODE_AND_LOCK_STATE, PARKING_INDICATION, REVERSE, OVER_CURRENT, OVERLOAD_ERROR, LOCK1, VEHICLE_ANTI_THEFT_SYSTEM, E_ABS, CHARGING, REGENERATIVE_BRAKING1, THROTTLE_ERROR, MCU_ERROR, MOTOR_ERROR, BRAKE_ANY, TEMPERATURE, LOCK2, FWD_GEAR, NEUTRAL_GEAR, SPEED_CONTROL_STATUS, CURRENT_CONTROL_STATUS, UNDER_VOLTAGE_STATUS, OVER_VOLTAGE_STATUS, REGENERATIVE_BRAKING_STATUS, REGENERATIVE_DURING_BRAKING_STATUS, TORQUE_CONTROL_STATUS, MODE_AND_STATE_CONTROL_SECTION, E_ABS2, REGENERATIVE_BRAKING2, LOCK3, MAXIMUM_SPEED_CONTROL, CURRENT_LIMIT, UNDERVOLTAGE_LIMIT, OVERVOLTAGE_LIMIT, REGEN_BRA_AT_ZERO_THROTTLE, RE_BR_AT_ZERO_THROTTLE, SOC, STATE_OF_HEALTH, CURRENT_STATE, PREV_STATE, AVAILABLECAPACITY, BMSID, H_VERSION, S_VERSION, CELLVOL1, CELLVOL2, CELLVOL3, CELLVOL4, CELLVOL5, CELLVOL6, CELLVOL7, CELLVOL8, CELLVOL9, CELLVOL10, CELLVOL11, CELLVOL12, CELLVOL13, CELLVOL14, CELLVOL15, CELLVOL16, CELLVOL17, CELLVOL18, CELLVOL19, CELLVOL20, CELLTEMP1, CELLTEMP2, CELLTEMP3, CELLTEMP4, CELLTEMP5, CELLTEMP6, CELLTEMP7, CELLTEMP8, CELLTEMP9, CELLTEMP10, CELLTEMP11, CELLTEMP12, CELLTEMP13, CELLTEMP14, CELLTEMP15, CELLTEMP16, CELLTEMP17, CELLTEMP18, TOTAL_VOLTAGE, CURRENT, POWER, BATTERY_FAILURE\n");
			HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
			flag=0;
		}else{
			Start_Stopwatch();
			sprintf(buffer,
			    "%d,%lu.%03lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
			    indx,seconds,milliseconds_display,data0[0], data0[1], data0[2], data0[3], data0[4], data0[5], data0[6], data0[7], data0[8], data0[9], data0[10], data0[11],
			    data0[12], data0[13], data0[14], data0[15], data0[16], data0[17], data0[18], data0[19], data0[20], data0[21], data0[22], data0[23],
			    data0[24], data0[25], data0[26], data0[27], data0[28], data0[29], data0[30], data0[31], data0[32], data0[33], data0[34], data0[35],
			    data0[36], data0[37], data0[38], data0[39], data0[40], data0[41], data0[42], data0[43], data0[44], data0[45], data0[46], data0[47],
			    data0[48], data0[49], data0[50], data0[51], data0[52], data0[53], data0[54], data0[55], data0[56], data0[57], data0[58], data0[59],
			    data0[60], data0[61], data0[62], data0[63], data0[64], data0[65], data0[66], data0[67], data0[68], data0[69], data0[70], data0[71],
			    data0[72], data0[73], data0[74], data0[75], data0[76], data0[77], data0[78], data0[79], data0[80], data0[81], data0[82], data0[83],
			    data0[84], data0[85], data0[86], data0[87], data0[88], data0[89]
			);

			HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
			indx++;
		}

//			Mount_SD("/");
			Update_File("konwert.TXT", buffer);
//			vPortFree(buffer);
//			HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
//			Unmount_SD("/");
//			HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
//			HAL_Delay(500);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
//			HAL_Delay(500);
//			HAL_GPIO_WritePin(LD_GPIO_Port, LD_Pin, 0);
//			vTaskDelay(pdMS_TO_TICKS(1000)); // Delay 10 milliseconds
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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  MX_CAN1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
//  if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0x32F2)
//    {
//      //   Set the time
//        set_time();
//    }
//  get_time();
//  sprintf((char*)fileName, "%02d-%02d-%04d_%02d-%02d_KonwertLog.txt",
//               gDate.Date, gDate.Month, 2000 + gDate.Year, gTime.Hours, gTime.Minutes);

  Mount_SD("/");
  Format_SD();
  Create_File("konwert.TXT");
//  Unmount_SD("/");

//  filter_config();
//
//  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
//
//  if(HAL_CAN_Start(&hcan1)!=HAL_OK)
//  Error_Handler();

  xTaskCreate(SD_TASK, "SD_CARD", 4000, NULL, 5, &SD_TASK_Handler); //2048
//  xTaskCreate(UART_TASK, "UART_CARD", 256, NULL, 5, &UART_TASK_Handler);
//
//
  vTaskStartScheduler();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 176;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 8;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD_GPIO_Port, LD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD_Pin */
  GPIO_InitStruct.Pin = LD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcann){

		if(HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&RxHeader, rxData) != HAL_OK){
				Error_Handler();
		}
		else{
			 HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		}
		HAL_GPIO_WritePin(LD_GPIO_Port, LD_Pin, 1);

		if (RxHeader.ExtId == 0x14520902) {
		    WHEEL_RPM = (rxData[0] | (rxData[1] << 8));
		    WHEEL_R = (rxData[2] | (rxData[3] << 8));
		    BATTERY_VOLTAGE = (rxData[4] | (rxData[5] << 8)) * 0.1;
		    BATTERY_CURRENT = (rxData[6] | (rxData[7] << 8)) * 0.01;
		}

		if (RxHeader.ExtId == 0x18530902) {
		    THROTTLE_POSITION = rxData[0];
		    MODE_AND_LOCK_STATE = (rxData[1] >> 6) & ((1 << 2) - 1);
		    PARKING_INDICATION = (rxData[1] >> 2) & 1;
		    REVERSE = (rxData[1] >> 3) & 1;
		    OVER_CURRENT = (rxData[1] >> 4) & 1;
		    OVERLOAD_ERROR = (rxData[1] >> 5) & 1;
		    LOCK1 = (rxData[1] >> 7) & 1;
		    VEHICLE_ANTI_THEFT_SYSTEM = (rxData[2] >> 0) & 1;
		    E_ABS = (rxData[2] >> 1) & 1;
		    CHARGING = (rxData[2] >> 2) & 1;
		    REGENERATIVE_BRAKING1 = (rxData[2] >> 3) & 1;
		    THROTTLE_ERROR = (rxData[2] >> 5) & 1;
		    MCU_ERROR = (rxData[2] >> 4) & 1;
		    MOTOR_ERROR = (rxData[2] >> 6) & 1;
		    BRAKE_ANY = (rxData[2] >> 7) & 1;
		    TEMPERATURE = rxData[3];
		    LOCK2 = (rxData[4] >> 0) & 1;
		    FWD_GEAR = (rxData[4] >> 1) & 1;
		    NEUTRAL_GEAR = (rxData[4] >> 2) & 1;
		}

		if (RxHeader.ExtId == 0x1C530902) {
		    SPEED_CONTROL_STATUS = rxData[0];
		    CURRENT_CONTROL_STATUS = rxData[1];
		    UNDER_VOLTAGE_STATUS = rxData[2];
		    OVER_VOLTAGE_STATUS = rxData[3];
		    REGENERATIVE_BRAKING_STATUS = rxData[4];
		    REGENERATIVE_DURING_BRAKING_STATUS = rxData[5];
		    TORQUE_CONTROL_STATUS = rxData[6];
		}

		if (RxHeader.ExtId == 0x18430209) {
		    MODE_AND_STATE_CONTROL_SECTION = (rxData[0] >> 6) & ((1 << 2) - 1);
		    E_ABS2 = (rxData[0] >> 2) & 1;
		    REGENERATIVE_BRAKING2 = (rxData[0] >> 3) & 1;
		    LOCK3 = (rxData[0] >> 6) & 1;
		    MAXIMUM_SPEED_CONTROL = rxData[1];
		    CURRENT_LIMIT = rxData[2];
		    UNDERVOLTAGE_LIMIT = rxData[3];
		    OVERVOLTAGE_LIMIT = rxData[4];
		    REGEN_BRA_AT_ZERO_THROTTLE = rxData[5];
		    RE_BR_AT_ZERO_THROTTLE = rxData[6];
		}

		if (RxHeader.ExtId == 0x19FF01D9) {
		    SOC = rxData[0];
		    STATE_OF_HEALTH = rxData[1];
		    CURRENT_STATE = rxData[2];
		    PREV_STATE = rxData[3];
		        // AVAILABLECAPACITY not assigned, spans bytes 4-7
		    }

		    if (RxHeader.ExtId == 0x19FF01D0) {
		        H_VERSION = rxData[0];
		        S_VERSION = rxData[1];
		        BMSID = rxData[6];
		    }

		    if (RxHeader.ExtId == 0x19FF01D1) {
		        CELLVOL1 = ((rxData[0] << 8) | rxData[1]) * 0.001;
		        CELLVOL2 = ((rxData[2] << 8) | rxData[3]) * 0.001;
		        CELLVOL3 = ((rxData[4] << 8) | rxData[5]) * 0.001;
		        CELLVOL4 = ((rxData[6] << 8) | rxData[7]) * 0.001;
		    }

		    if (RxHeader.ExtId == 0x19FF01D2) {
		        CELLVOL5 = ((rxData[0] << 8) | rxData[1]) * 0.001;
		        CELLVOL6 = ((rxData[2] << 8) | rxData[3]) * 0.001;
		        CELLVOL7 = ((rxData[4] << 8) | rxData[5]) * 0.001;
		        CELLVOL8 = ((rxData[6] << 8) | rxData[7]) * 0.001;
		    }

		    if (RxHeader.ExtId == 0x19FF01D3) {
		        CELLVOL9 = ((rxData[0] << 8) | rxData[1]) * 0.001;
		        CELLVOL10 = ((rxData[2] << 8) | rxData[3]) * 0.001;
		        CELLVOL11 = ((rxData[4] << 8) | rxData[5]) * 0.001;
		        CELLVOL12 = ((rxData[6] << 8) | rxData[7]) * 0.001;
		    }

		    if (RxHeader.ExtId == 0x19FF01D4) {
		        CELLVOL13 = ((rxData[0] << 8) | rxData[1]) * 0.001;
		        CELLVOL14 = ((rxData[2] << 8) | rxData[3]) * 0.001;
		        CELLVOL15 = ((rxData[4] << 8) | rxData[5]) * 0.001;
		        CELLVOL16 = ((rxData[6] << 8) | rxData[7]) * 0.001;
		    }

		    if (RxHeader.ExtId == 0x19FF01D5) {
		        CELLVOL17 = ((rxData[0] << 8) | rxData[1]) * 0.001;
		        CELLVOL18 = ((rxData[2] << 8) | rxData[3]) * 0.001;
		        CELLVOL19 = ((rxData[4] << 8) | rxData[5]) * 0.001;
		        CELLVOL20 = ((rxData[6] << 8) | rxData[7]) * 0.001;
		    }

		    if (RxHeader.ExtId == 0x19FF01D7) {
		        CELLTEMP1 = rxData[0];
		        CELLTEMP2 = rxData[1];
		        CELLTEMP3 = rxData[2];
		        CELLTEMP4 = rxData[3];
		        CELLTEMP5 = rxData[4];
		        CELLTEMP6 = rxData[5];
		        CELLTEMP7 = rxData[6];
		        CELLTEMP8 = rxData[7];
		    }

		    if (RxHeader.ExtId == 0x19FF01D8) {
		        CELLTEMP9 = rxData[0];
		        CELLTEMP10 = rxData[1];
		        TOTAL_VOLTAGE = ((rxData[2] << 8) | rxData[3]) * 0.01;
		        CURRENT = ((rxData[6] << 8) | rxData[7]) * 0.001;
		        POWER = TOTAL_VOLTAGE * CURRENT;
		    }

		    if (RxHeader.ExtId == 0x19FF01ED) {
		        CELLTEMP11 = rxData[0];
		        CELLTEMP12 = rxData[1];
		        CELLTEMP13 = rxData[2];
		        CELLTEMP14 = rxData[3];
		        CELLTEMP15 = rxData[4];
		        CELLTEMP16 = rxData[5];
		        CELLTEMP17 = rxData[6];
		        CELLTEMP18 = rxData[7];
		    }
//		if(RxHeader.ExtId == 0x19FF01DA){
//				revxx_data.battery_failure;
//		}
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
//if(==14){
//	  xTaskCreate(UART_TASK, "UART_CARD", 256, NULL, 5, &UART_TASK_Handler);
//	  vTaskStartScheduler();
//	  //			HAL_GPIO_WritePin(LD_GPIO_Port, LD_Pin, 0);
//
//}

}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
	           if (running) {
	               milliseconds++;
	               if (milliseconds >= 1000) {
	                   milliseconds = 0;
	                   seconds++;
	               }
	           }
	       milliseconds_display = milliseconds % 1000;
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
