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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	I2C_HandleTypeDef *i2c;
	uint8_t i2c_addr;
	float temp;
	float hum;
	float press;
} Bme280_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BME280_ADDR (0x76 << 1 )

#define BME280_REG_CHIP_ID 0xD0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

osThreadId DefaultTaskHandle;
osThreadId CANTaskHandle;
osThreadId SensorTaskHandle;
osSemaphoreId UARTSemHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_CAN_Init(void);
void StartDefaultTask(void const * argument);
void StartCANTask(void const * argument);
void StartSensorTask(void const * argument);

/* USER CODE BEGIN PFP */
void BME280_Init(Bme280_t *bme280, I2C_HandleTypeDef *i2c, uint8_t address);
float BME280_ReadTemperature(Bme280_t *bme280);
float BME280_ReadHumidity(Bme280_t *bme280);
void BME280_ReadCalibration(Bme280_t *bme280);
void Send_Sensor_Data_CAN(float p, float t, float h);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t dig_T1;
int16_t dig_T2, dig_T3;

uint8_t dig_H1, dig_H3;
int16_t dig_H2, dig_H4, dig_H5;
int8_t dig_H6;

int32_t t_fine;
uint8_t chip_id = 0;
HAL_StatusTypeDef test_status;
float temp, hum;
Bme280_t myBME280;


uint8_t rx_byte;
uint8_t buf[32];
uint8_t rx_index = 0;

volatile uint16_t pm25 = 0;
volatile uint8_t frame_ready = 0;
float pm25_stable = 0;

volatile uint32_t last_update = 0;
volatile uint32_t pm_sum = 0;
volatile uint32_t pm_count = 0;

CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
uint32_t TxMailbox;
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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
	
	BME280_Init(&myBME280, &hi2c1, BME280_ADDR);
	
	HAL_CAN_Start(&hcan);
	
	TxHeader.StdId = 0x123;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = DISABLE;
	
	// Th? d?a ch? 0x76 (ph? bi?n nh?t cho module t�m)
test_status = HAL_I2C_Mem_Read(&hi2c1, (0x76 << 1), 0xD0, I2C_MEMADD_SIZE_8BIT, &chip_id, 1, 100);

if (test_status != HAL_OK) {
    // N?u l?i, th? ti?p d?a ch? 0x77
    test_status = HAL_I2C_Mem_Read(&hi2c1, (0x77 << 1), 0xD0, I2C_MEMADD_SIZE_8BIT, &chip_id, 1, 100);
}
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of UARTSem */
  osSemaphoreDef(UARTSem);
  UARTSemHandle = osSemaphoreCreate(osSemaphore(UARTSem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
	osSemaphoreWait(UARTSemHandle, 0);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of DefaultTask */
  osThreadDef(DefaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  DefaultTaskHandle = osThreadCreate(osThread(DefaultTask), NULL);

  /* definition and creation of CANTask */
  osThreadDef(CANTask, StartCANTask, osPriorityAboveNormal, 0, 128);
  CANTaskHandle = osThreadCreate(osThread(CANTask), NULL);

  /* definition and creation of SensorTask */
  osThreadDef(SensorTask, StartSensorTask, osPriorityNormal, 0, 256);
  SensorTaskHandle = osThreadCreate(osThread(SensorTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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
  hcan.Init.Prescaler = 2;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void BME280_ReadCalibration(Bme280_t *bme280)
{
		uint8_t data[6];
	
		HAL_I2C_Mem_Read(bme280->i2c, bme280->i2c_addr, 0x88, 1, data, 6, 100);

		dig_T1 = (data[1] << 8)|data[0];
		dig_T2 = (data[3] << 8)|data[2];
		dig_T3 = (data[5] << 8)|data[4];

		HAL_I2C_Mem_Read(bme280->i2c, bme280->i2c_addr, 0xA1, 1, &dig_H1, 1, 100);
		uint8_t hdata[7];

		HAL_I2C_Mem_Read(bme280->i2c, bme280->i2c_addr, 0xE1, 1, hdata, 7, 100);

		dig_H2 = (hdata[1] << 8)|hdata[0];
		dig_H3 = hdata[2];
		dig_H4 = (hdata[3]<<4)|(hdata[4]&0x0F);
		dig_H5 = (hdata[5]<<4)|(hdata[4]>>4);
		dig_H6 = hdata[6];
}

void BME280_Init(Bme280_t *bme280, I2C_HandleTypeDef *i2c, uint8_t address)
{
	bme280->i2c = i2c;
	bme280->i2c_addr = address;
		uint8_t data;

		data = 0x01;
		HAL_I2C_Mem_Write(bme280->i2c, bme280->i2c_addr, 0xF2, 1, &data, 1, 100);

		data = 0x23;
		HAL_I2C_Mem_Write(bme280->i2c, bme280->i2c_addr, 0xF4, 1, &data, 1, 100);

		BME280_ReadCalibration(bme280);

}


float BME280_ReadTemperature(Bme280_t *bme280)
{
		uint8_t data[3];
		int32_t adc_T;
		int32_t var1, var2;
		float T;

		HAL_I2C_Mem_Read(bme280->i2c, bme280->i2c_addr, 0xFA, 1, data, 3, 100);

		adc_T = ((uint32_t)data[0]<<12)|((uint32_t)data[1]<<4)|(data[2]>>4);

		var1 = ((((adc_T>>3)-((int32_t)dig_T1<<1)))*((int32_t)dig_T2))>>11;

		var2 = (((((adc_T>>4)-((int32_t)dig_T1))*((adc_T>>4)-((int32_t)dig_T1)))>>12)*((int32_t)dig_T3))>>14;

		t_fine = var1+var2;

		T = (t_fine*5+128)>>8;
		T = T/100;

		return T;
}



float BME280_ReadHumidity(Bme280_t *bme280)
{
		uint8_t data[2];
		int32_t adc_H;
		int32_t v_x1;
		float h;
		
		HAL_I2C_Mem_Read(bme280->i2c, bme280->i2c_addr, 0xFD, 1, data, 2, 100);
	
		adc_H = (data[0]<<8)|data[1];
	
		v_x1 = t_fine - 76800;
		
		v_x1 = (((((adc_H<<14)-((int32_t)dig_H4<<20)-((int32_t)dig_H5*v_x1))+16384)>>15)*(((((((v_x1*(int32_t)dig_H6)>>10)*(((v_x1*(int32_t)dig_H3)>>11)+32768))>>10)+2097152)*(int32_t)dig_H2+8192)>>14));
		
		v_x1 = v_x1 - (((((v_x1>>15)*(v_x1>>15))>>7)*(int32_t)dig_H1)>>4);
		
		if(v_x1<0)	v_x1 = 0;
		if(v_x1>419430400) v_x1 = 419430400;
		
		h = (v_x1>>12);
		h = h/1024.0;
		
		return h;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef*huart)
{
		if(huart->Instance == USART1)
		{
				if(rx_index == 0 && rx_byte != 0x42)
				{
						HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
						return;
				}
				
				if(rx_index == 1 && rx_byte != 0x4D)
				{
						rx_index = 0;
						HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
						return;
				}
				
				buf[rx_index++] = rx_byte;
				
				if(rx_index >= 32)
				{
						rx_index = 0;
					
						uint16_t sum = 0;
						for(int i = 0; i<30; i++)	sum+= buf[i];
					
						uint16_t checksum = (buf[30] << 8) | buf[31];
					
						if(sum == checksum)
						{
								pm25 = (buf[12] << 8) | buf[13];
								frame_ready = 1;
						}
				}
				
				HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
		}
}

void Send_Sensor_Data_CAN(float p, float t, float h) 
{
		if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0)
		{
				int16_t p_send = (int16_t)(p*10);
				int16_t t_send = (int16_t)(t*100);
				int16_t h_send = (int16_t)(h*100);
			
				TxData[0] = (p_send >> 8) & 0xFF;
				TxData[1] = p_send & 0xFF;
				TxData[2] = (t_send >> 8) & 0xFF;
				TxData[3] = t_send & 0xFF;
				TxData[4] = (h_send >> 8) & 0xFF;
				TxData[5] = h_send & 0xFF;
				TxData[6] = 0x00;
				TxData[7] = 0x00;
			
				HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
		}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the DefaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartCANTask */
/**
* @brief Function implementing the CANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCANTask */
void StartCANTask(void const * argument)
{
  /* USER CODE BEGIN StartCANTask */
  /* Infinite loop */
  for(;;)
  {
			if(osSemaphoreWait(UARTSemHandle, osWaitForever) == osOK)
			{
					Send_Sensor_Data_CAN(pm25_stable, temp, hum);
			}
  }
  /* USER CODE END StartCANTask */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the SensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void const * argument)
{
  /* USER CODE BEGIN StartSensorTask */
  /* Infinite loop */
	static uint32_t timer_count = 0;
  for(;;)
  {
			if(frame_ready)
			{
					pm_count++;
					pm_sum += pm25;
					frame_ready = 0;
			}
			
			timer_count += 100;
			
			if(timer_count >= 10000)
			{
					if(pm_count > 0)
					{
							pm25_stable = (float)pm_sum / pm_count;
					}
					
					temp = BME280_ReadTemperature(&myBME280);
					hum = BME280_ReadHumidity(&myBME280);
					
					taskENTER_CRITICAL();
					pm_sum = 0;
					pm_count = 0;
					taskEXIT_CRITICAL();
					
					timer_count = 0;
					
					osSemaphoreRelease(UARTSemHandle);
			}
			
			osDelay(100);
  }
  /* USER CODE END StartSensorTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
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
