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
#include <main.h>
#include <stdio.h>
#include <iostream>
#include "stdint.h"
#include "string.h"
//#include "C:\Users\suoud\OneDrive\Desktop\C Program list\Custom Messages\common\mavlink.h"
#include "C:\Users\suoud\OneDrive\Desktop\C Program list\Custom Messages\your_custom_dialect\mavlink.h"
#include <time.h>
#include <sys/time.h>
//#include <pthread.h>
//#include <unistd.h>
//#include <mutex>
#include "Autopilot.h"
//#include "Time_Stamps.h"
#include "stm32g4xx_hal_conf.h"
#include "stm32g4xx_it.h"
//#include "Time_Stamps.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define READCHAR_PROTOTYPE int __io_getchar(void)
//#define virtual int read_message(mavlink_message_t &message);
//#define virtual int write_command(const mavlink_message_t &message);

#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION     0b0000110111111000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     0b0000110111000111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION 0b0000110000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FORCE        0b0000111000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE    0b0000100111111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE     0b0000010111111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ROLL         0b0010111111111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_PITCH        0b0001111111111111

#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_TAKEOFF      0x1000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_LAND         0x2000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_LOITER       0x3000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_IDLE         0x4000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
//uint8_t receive_buffer[10000];
//uint8_t* cp = receive_buffer;
bool debug=true;
bool autotakeoff = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
//void enable_offboard_control();
//int  toggle_offboard_control( bool flag );
//void disable_offboard_control();
//int arm_disarm( bool flag );
//void write_thread(void);
//void read_thread();
//bool autotakeoff = false;
//void commands(Autopilot_Interface &api, bool autotakeoff);
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
	std::cout<<"I'm Working";
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
  MX_LPUART1_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint8_t rx_buffer[1000];
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	  HAL_Delay(1000);
	  Autopilot_Interface autopilot_interface;
	  autopilot_interface.start();
	  Commands(autopilot_interface, autotakeoff);
//	  HAL_UART_Receive(&huart1,rx_buffer,8,HAL_MAX_DELAY);
//	  for (int i=0; i<1000; i++)
//	  {
//		HAL_UART_Transmit(&hlpuart1, &rx_buffer[i], 1, HAL_MAX_DELAY);
//	  }
  }
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */
  /* USER CODE END LPUART1_Init 2 */
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
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if (huart->Instance == USART1)
//    {
//        rx_buffer[rx_buffer_index++] = /* received byte */;
//
//        // Check if a complete MAVLink message is received
//        if (rx_buffer_index >= MAVLINK_MIN_LEN && rx_buffer_index >= rx_buffer[1] + 8)
//        {
//            // Process MAVLink message (e.g., parse and handle)
//            // Example:
//            mavlink_message_t msg;
//            mavlink_status_t status;
//            mavlink_parse_char(MAVLINK_COMM_0, rx_buffer[rx_buffer_index - 1], &msg, &status);
//
//            // Reset buffer index for next message
//            rx_buffer_index = 0;
//        }
//
//        // Continue reception for next byte
//        HAL_UART_Receive_IT(&huart1, &rx_buffer[rx_buffer_index], 1);
//    }
//}
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}
/* USER CODE BEGIN 4 */

void Autopilot_Interface::start()
{
	Autopilot_Interface::write_thread();
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)("Commands Entered"), 16, HAL_MAX_DELAY);
	Autopilot_Interface::read_thread();
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)("Found"), 5, HAL_MAX_DELAY);
	if (not system_id)
	{
		system_id = current_messages.sysid;
		HAL_UART_Transmit(&hlpuart1, (uint8_t*)("GOT VEHICLE SYSTEM ID:"), 22, HAL_MAX_DELAY);
		HAL_UART_Transmit(&hlpuart1, (uint8_t*)(system_id), 100, HAL_MAX_DELAY);
	}
	if (not autopilot_id)
	{
		autopilot_id = current_messages.compid;
		HAL_UART_Transmit(&hlpuart1, (uint8_t*)("GOT AUTOPILOT COMPONENT ID:"), 27, HAL_MAX_DELAY);
		HAL_UART_Transmit(&hlpuart1, (uint8_t*)(autopilot_id), 100, HAL_MAX_DELAY);
	}
	Mavlink_Messages local_data = current_messages;
	initial_position.x        = local_data.control.x;
	initial_position.y        = local_data.control.y;
	initial_position.z        = local_data.control.z;
	initial_position.vx       = local_data.local_position_ned.vx;
	initial_position.vy       = local_data.local_position_ned.vy;
	initial_position.vz       = local_data.local_position_ned.vz;
	initial_position.yaw      = local_data.control.r;
	initial_position.yaw_rate = local_data.attitude.yawspeed;
	//intial_position.roll      = local_data.attittude.roll;
	//initial_position.pitch    = local_data.attitude.roll;

	float float_value = initial_position.x;
	uint8_t tx_buffer[sizeof(float)];
	memcpy(tx_buffer, &float_value, sizeof(float));

	float float_value1 = initial_position.y;
	uint8_t tx_buffer1[sizeof(float)];
	memcpy(tx_buffer1, &float_value1, sizeof(float));

	float float_value2 = initial_position.z;
	uint8_t tx_buffer2[sizeof(float)];
	memcpy(tx_buffer2, &float_value2, sizeof(float));

	float float_value3 = initial_position.yaw;
	uint8_t tx_buffer3[sizeof(float)];
	memcpy(tx_buffer3, &float_value3, sizeof(float));

	HAL_UART_Transmit(&hlpuart1, (uint8_t*)("INITIAL POSITION XYZ:"), 24, HAL_MAX_DELAY);
	HAL_UART_Transmit(&hlpuart1, tx_buffer, sizeof(float), HAL_MAX_DELAY);
	HAL_UART_Transmit(&hlpuart1, tx_buffer1, sizeof(float), HAL_MAX_DELAY);
	HAL_UART_Transmit(&hlpuart1, tx_buffer2, sizeof(float), HAL_MAX_DELAY);
	HAL_UART_Transmit(&hlpuart1, tx_buffer3, sizeof(float), HAL_MAX_DELAY);
}

void Commands(Autopilot_Interface &api, bool autotakeoff)
{
	api.enable_offboard_control();
	//usleep(100);
	if(autotakeoff)
	{
		api.arm_disarm(true);
		HAL_Delay(10);
	}
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)("Send Offboard Commands\n"),24, HAL_MAX_DELAY);
	mavlink_set_position_target_local_ned_t sp;
	mavlink_set_position_target_local_ned_t ip = api.initial_position;
	api.set_position(ip.x, ip.y, ip.z, sp);

	api.set_yaw((ip.yaw + 90.0/180.0*M_PI), sp);
	if(autotakeoff)
	{
		sp.type_mask |= MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_TAKEOFF;
	}
	api.update_setpoint(sp);
	while (true)
	{
		mavlink_local_position_ned_t pos = api.current_messages.local_position_ned;

		float float_value4 = pos.x;
		uint8_t tx_buffer4[sizeof(float)];
		memcpy(tx_buffer4, &float_value4, sizeof(float));

		float float_value5 = pos.y;
		uint8_t tx_buffer5[sizeof(float)];
		memcpy(tx_buffer5, &float_value5, sizeof(float));

		float float_value6 = pos.z;
		uint8_t tx_buffer6[sizeof(float)];
		memcpy(tx_buffer6, &float_value6, sizeof(float));

		HAL_UART_Transmit(&hlpuart1, tx_buffer4, sizeof(float), HAL_MAX_DELAY);
		HAL_UART_Transmit(&hlpuart1, tx_buffer5, sizeof(float), HAL_MAX_DELAY);
		HAL_UART_Transmit(&hlpuart1, tx_buffer6, sizeof(float), HAL_MAX_DELAY);

		Mavlink_Messages messages = api.current_messages;
	    mavlink_attitude_t attitude = messages.attitude;
	    mavlink_get_attitude_battery_t custom = messages.custom;
	    mavlink_battery_status_t battery_status = messages.battery_status;
	    custom.roll=attitude.roll;
	    custom.pitch=attitude.pitch;
	    custom.yaw=attitude.yaw;
	    custom.current_battery= battery_status.current_battery;
	    custom.energy_consumed= battery_status.energy_consumed;
	    custom.battery_remaining= battery_status.battery_remaining;

	    float float_value7 = custom.time_boot_ms;
	    uint8_t tx_buffer7[sizeof(float)];
	    memcpy(tx_buffer7, &float_value7, sizeof(float));

	    float float_value8 = custom.roll;
	    uint8_t tx_buffer8[sizeof(float)];
	    memcpy(tx_buffer8, &float_value8, sizeof(float));

	    float float_value9 = custom.yaw;
	    uint8_t tx_buffer9[sizeof(float)];
	    memcpy(tx_buffer9, &float_value9, sizeof(float));

	    float float_value10 = custom.pitch;
	    uint8_t tx_buffer10[sizeof(float)];
	    memcpy(tx_buffer10, &float_value10, sizeof(float));

	    float float_value11 = custom.current_battery;
	    uint8_t tx_buffer11[sizeof(float)];
	    memcpy(tx_buffer11, &float_value11, sizeof(float));

	    float float_value13 = custom.energy_consumed;
	    uint8_t tx_buffer13[sizeof(float)];
	    memcpy(tx_buffer13, &float_value13, sizeof(float));

	    float float_value14 = custom.battery_remaining;
	    uint8_t tx_buffer14[sizeof(float)];
	    memcpy(tx_buffer14, &float_value14, sizeof(float));

	    HAL_UART_Transmit(&hlpuart1, (uint8_t*)("Time (Since Boot):"), 18, HAL_MAX_DELAY);
	    HAL_UART_Transmit(&hlpuart1, tx_buffer7, sizeof(float), HAL_MAX_DELAY);

	    HAL_UART_Transmit(&hlpuart1, (uint8_t*)("Roll:"), 5, HAL_MAX_DELAY);
	    HAL_UART_Transmit(&hlpuart1, tx_buffer8, sizeof(float), HAL_MAX_DELAY);

	    HAL_UART_Transmit(&hlpuart1, (uint8_t*)("Yaw:"), 4, HAL_MAX_DELAY);
	    HAL_UART_Transmit(&hlpuart1, tx_buffer9, sizeof(float), HAL_MAX_DELAY);

	    HAL_UART_Transmit(&hlpuart1, (uint8_t*)("Pitch:"), 6, HAL_MAX_DELAY);
	    HAL_UART_Transmit(&hlpuart1, tx_buffer10, sizeof(float), HAL_MAX_DELAY);

	    HAL_UART_Transmit(&hlpuart1, (uint8_t*)("Current Battery:"), 16, HAL_MAX_DELAY);
	    HAL_UART_Transmit(&hlpuart1, tx_buffer11, sizeof(float), HAL_MAX_DELAY);

	    HAL_UART_Transmit(&hlpuart1, (uint8_t*)("Energy Consumed:"), 18, HAL_MAX_DELAY);
	    HAL_UART_Transmit(&hlpuart1, tx_buffer13, sizeof(float), HAL_MAX_DELAY);

	    HAL_UART_Transmit(&hlpuart1, (uint8_t*)("Battery Remaining:"), 19, HAL_MAX_DELAY);
	    HAL_UART_Transmit(&hlpuart1, tx_buffer14, sizeof(float), HAL_MAX_DELAY);

	    HAL_Delay(100);
	}
	api.arm_disarm(false);
}

void Autopilot_Interface::write_thread(void)
{
	mavlink_set_position_target_local_ned_t sp;
	sp.type_mask = (MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY & MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE & MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ROLL & MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_PITCH);
	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
	sp.vx       = 0.0;
	sp.vy       = 0.0;
	sp.vz       = 0.0;
	sp.yaw_rate = 0.0;
	current_setpoint.data = sp;
	Autopilot_Interface::write_setpoint();
	bool writing_status = true;
	while ( !time_to_exit )
	{
		//usleep(250000);   // Stream at 4Hz
		HAL_Delay(10);
		Autopilot_Interface::write_setpoint();
		time_to_exit=true;
	}
	writing_status = false;
	return;
}

int Autopilot_Interface::write_message(mavlink_message_t &message)
{
	char buf[MAVLINK_MAX_PACKET_LEN];
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);
	HAL_UART_Transmit(&huart1, (uint8_t *)&buf, len, HAL_MAX_DELAY);
	return len;
}
void Autopilot_Interface::write_setpoint()
{
	mavlink_set_position_target_local_ned_t sp;
	sp = current_setpoint.data;
	if ( not sp.time_boot_ms)
	    sp.time_boot_ms = (uint32_t) (get_time_usec()/1000);
	sp.target_system    = system_id;
	sp.target_component = autopilot_id;
	mavlink_message_t message;
	mavlink_msg_set_position_target_local_ned_encode(system_id, companion_id, &message, &sp);
	int len = Autopilot_Interface::write_message(message);
	//return len;
}

void Autopilot_Interface::enable_offboard_control()
{
	if ( control_status == false )
	{
		HAL_UART_Transmit(&hlpuart1, (uint8_t*)("ENABLE OFFBOARD MODE"), 20, HAL_MAX_DELAY);
		int success = Autopilot_Interface::toggle_offboard_control( true );
		if ( success )
			control_status = true;
		else
		{
			HAL_UART_Transmit(&hlpuart1, (uint8_t*)(stderr), 100, HAL_MAX_DELAY);
			HAL_UART_Transmit(&hlpuart1, (uint8_t*)("Error: off-board mode not set, could not write message\n"), 53, HAL_MAX_DELAY);
		}
	}
}
void Autopilot_Interface::disable_offboard_control()
{
    if ( control_status == true )
	{
		HAL_UART_Transmit(&hlpuart1, (uint8_t*)("DISABLE OFFBOARD MODE\n"), 22, HAL_MAX_DELAY);
		int success = Autopilot_Interface::toggle_offboard_control( false );
		if ( success )
			control_status = false;
		else
		{
			HAL_UART_Transmit(&hlpuart1, (uint8_t*)("Error: off-board mode not set, could not write message\n"), 53, HAL_MAX_DELAY);
		}
	}
	else
	{
		HAL_UART_Transmit(&hlpuart1, (uint8_t*)("Cannot Enter\n"), 14, HAL_MAX_DELAY);
	}

}
int Autopilot_Interface::arm_disarm( bool flag )
{
	if(flag)
	{
		HAL_UART_Transmit(&hlpuart1, (uint8_t*)("ARM MOTORS"), 10, HAL_MAX_DELAY);
	}
	else
	{
		HAL_UART_Transmit(&hlpuart1, (uint8_t*)("DISARM MOTORS"), 10, HAL_MAX_DELAY);
	}
	mavlink_command_long_t com = { 0 };
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	com.command          = MAV_CMD_COMPONENT_ARM_DISARM;
	com.confirmation     = true;
	com.param1           = (float) flag;
	com.param2           = 21196;
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);
    int len = Autopilot_Interface::write_message(message);
    return len;
}
int Autopilot_Interface::move_control(uint16_t x, uint16_t y, uint16_t z, uint16_t r, uint16_t buttons, bool flag)
{
	if(flag)
	{
		HAL_UART_Transmit(&hlpuart1, (uint8_t*)("MANUAL CONTROL ENABLED\n"), 24, HAL_MAX_DELAY);
		mavlink_command_long_t com = {0};
	    com.target_system    = system_id;
	    com.target_component = autopilot_id;
	    com.command          = MAV_CMD_DO_SET_MODE;
	    com.confirmation     = true;
	    com.param1           = autopilot_id;
	    com.param2           = x;
	    com.param3           = y;
	    com.param4           = z;
		com.param5           = r;
	    com.param6           = buttons;
	    mavlink_message_t message;
	    mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);
        int len = Autopilot_Interface::write_message(message);
        return len;
	}
	else
	{
		return 0;
	}
}
int Autopilot_Interface::toggle_offboard_control( bool flag )
{
	mavlink_command_long_t com = { 0 };
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	com.command          = MAV_CMD_NAV_GUIDED_ENABLE;
	com.confirmation     = true;
	com.param1           = (float) flag; // flag >0.5 => start, <0.5 => stop
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);
    int len = Autopilot_Interface::write_message(message);
    return len;
}
void Autopilot_Interface::update_setpoint(mavlink_set_position_target_local_ned_t setpoint)
{
	//std::lock_guard<std::mutex> lock(current_setpoint.mutex);
	current_setpoint.data = setpoint;
}









void Autopilot_Interface::read_thread()
{
	reading_status = false;
//	const char* str = reading_status ? "True" : "False";
//	HAL_UART_Transmit(&hlpuart1, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);

	while (!reading_status)
	{

		Autopilot_Interface::read_messages();
		HAL_Delay(100); // Read batches at 10Hz
		reading_status = true;
	}
	return;
}


int Autopilot_Interface::read_message(mavlink_message_t &message)
{
	    uint8_t receive_buffer[MAVLINK_MAX_PACKET_LEN];
		mavlink_status_t status;
		uint8_t msgReceived = false;
		int bytesReceived=HAL_UART_Receive(&huart1,receive_buffer,MAVLINK_MAX_PACKET_LEN,HAL_MAX_DELAY);
		uint8_t totransmit1[] = {'q'};
		HAL_UART_Transmit(&hlpuart1, totransmit1, 1, HAL_MAX_DELAY);
		for (int i = 0; i < MAVLINK_MAX_PACKET_LEN; i++)
		    {
		        msgReceived = mavlink_parse_char(MAVLINK_COMM_1, receive_buffer[i], &message, &status);
		        if (msgReceived)
		        {
		            break;  // Exit loop if a message is successfully parsed
		        }
		    }
		//HAL_UART_Transmit(&hlpuart1, &(msgReceived), 10, HAL_MAX_DELAY);
		if ( (lastStatus.packet_rx_drop_count != status.packet_rx_drop_count) && debug )
		{
			HAL_UART_Transmit(&hlpuart1, (uint8_t*)"Error: DROPPED PACKETS", 23, HAL_MAX_DELAY);
			HAL_UART_Transmit(&hlpuart1, (uint8_t*)(status.packet_rx_drop_count), 20, HAL_MAX_DELAY);
		}
			lastStatus = status;
		if(msgReceived && debug)
		{
			// Report info
			HAL_UART_Transmit(&hlpuart1, (uint8_t*)"Received message from serial with ID", 36, HAL_MAX_DELAY);
			HAL_UART_Transmit(&hlpuart1, (uint8_t*)(message.msgid), MAVLINK_MAX_PACKET_LEN, HAL_MAX_DELAY);
			HAL_UART_Transmit(&hlpuart1, (uint8_t*)(message.sysid), MAVLINK_MAX_PACKET_LEN, HAL_MAX_DELAY);
			HAL_UART_Transmit(&hlpuart1, (uint8_t*)(message.compid), MAVLINK_MAX_PACKET_LEN, HAL_MAX_DELAY);

			//fprintf(stderr,"Received serial data: ");
			unsigned int i;
			uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
			unsigned int messageLength = mavlink_msg_to_send_buffer(buffer, &message);
			if (messageLength > MAVLINK_MAX_PACKET_LEN)
			{
				HAL_UART_Transmit(&hlpuart1, (uint8_t*)"FATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE", 54, HAL_MAX_DELAY);
			}
			else
			{
				for (i=0; i<messageLength; i++)
				{
					unsigned char v=buffer[i];
					HAL_UART_Transmit(&hlpuart1, &v, sizeof(v), HAL_MAX_DELAY);
					//fprintf(stderr,"%02x ", v);
				}
				//fprintf(stderr,"\n");
			}

		}
		return msgReceived;
}
void Autopilot_Interface::read_messages()
{
	bool success;
	bool received_all = false;
	Time_Stamps this_timestamps;
	while (!(received_all) and (!reading_status) )
	{
	    mavlink_message_t message;
		success = Autopilot_Interface::read_message(message);
	    if (success==true)
	    {
	        current_messages.sysid  = message.sysid;
	        current_messages.compid = message.compid;
	        switch (message.msgid)
			{
				case MAVLINK_MSG_ID_HEARTBEAT:
				{
	                //printf("MAVLINK_MSG_ID_HEARTBEAT\n");
					mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
					current_messages.time_stamps.heartbeat = get_time_usec();
					this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
					break;
				}
				case MAVLINK_MSG_ID_SYS_STATUS:
				{
	                //printf("MAVLINK_MSG_ID_SYS_STATUS\n");
					mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
					current_messages.time_stamps.sys_status = get_time_usec();
					this_timestamps.sys_status = current_messages.time_stamps.sys_status;
					break;
	            }
				case MAVLINK_MSG_ID_BATTERY_STATUS:
				{
					//printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
					mavlink_msg_battery_status_decode(&message, &(current_messages.battery_status));
					current_messages.time_stamps.battery_status = get_time_usec();
					this_timestamps.battery_status = current_messages.time_stamps.battery_status;
					break;
				}
				case MAVLINK_MSG_ID_RADIO_STATUS:
				{
					//printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
					mavlink_msg_radio_status_decode(&message, &(current_messages.radio_status));
					current_messages.time_stamps.radio_status = get_time_usec();
					this_timestamps.radio_status = current_messages.time_stamps.radio_status;
					break;
				}
				case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
				{
					//printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
					mavlink_msg_local_position_ned_decode(&message, &(current_messages.local_position_ned));
					current_messages.time_stamps.local_position_ned = get_time_usec();
					this_timestamps.local_position_ned = current_messages.time_stamps.local_position_ned;
					break;
				}
				case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
				{
					//printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
					mavlink_msg_global_position_int_decode(&message, &(current_messages.global_position_int));
					current_messages.time_stamps.global_position_int = get_time_usec();
					this_timestamps.global_position_int = current_messages.time_stamps.global_position_int;
					break;
				}
				case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
				{
					//printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
					mavlink_msg_position_target_local_ned_decode(&message, &(current_messages.position_target_local_ned));
					current_messages.time_stamps.position_target_local_ned = get_time_usec();
					this_timestamps.position_target_local_ned = current_messages.time_stamps.position_target_local_ned;
					break;
				}
				case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
				{
					//printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
					mavlink_msg_position_target_global_int_decode(&message, &(current_messages.position_target_global_int));
					current_messages.time_stamps.position_target_global_int = get_time_usec();
					this_timestamps.position_target_global_int = current_messages.time_stamps.position_target_global_int;
					break;
				}
				case MAVLINK_MSG_ID_HIGHRES_IMU:
				{
					//printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
					mavlink_msg_highres_imu_decode(&message, &(current_messages.highres_imu));
					current_messages.time_stamps.highres_imu = get_time_usec();
					this_timestamps.highres_imu = current_messages.time_stamps.highres_imu;
					break;
				}
				case MAVLINK_MSG_ID_ATTITUDE:
				{
					//printf("MAVLINK_MSG_ID_ATTITUDE\n");
					mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
					current_messages.time_stamps.attitude = get_time_usec();
					this_timestamps.attitude = current_messages.time_stamps.attitude;
					break;
				}
				case MAVLINK_MSG_ID_MANUAL_CONTROL:
				{
					//printf("MAVLINK_MSG_ID_MANUAL_CONTROL\n");
					mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
					current_messages.time_stamps.attitude = get_time_usec();
					this_timestamps.attitude = current_messages.time_stamps.attitude;
					break;
				}
				case MAVLINK_MSG_ID_GET_ATTITUDE_BATTERY:
				{
					//printf("MAVLINK_MSG_ID_GET_ATTITUDE_BATTERY\n");
					mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
					current_messages.time_stamps.custom = get_time_usec();
					this_timestamps.custom = current_messages.time_stamps.custom;
					break;
				}
				default:
				{
					//printf("No message id %i\n",message.msgid);
					break;
				}
			}
	        HAL_UART_Transmit(&hlpuart1, (uint8_t*)this_timestamps.heartbeat, MAVLINK_MAX_PACKET_LEN, HAL_MAX_DELAY);
	        HAL_UART_Transmit(&hlpuart1, (uint8_t*)this_timestamps.sys_status, MAVLINK_MAX_PACKET_LEN, HAL_MAX_DELAY);

		}
		received_all =this_timestamps.heartbeat && this_timestamps.sys_status;
	//				this_timestamps.battery_status             &&
	//				this_timestamps.radio_status               &&
	//				this_timestamps.local_position_ned         &&
	//				this_timestamps.global_position_int        &&
	//				this_timestamps.position_target_local_ned  &&
	//				this_timestamps.position_target_global_int &&
	//				this_timestamps.highres_imu                &&
	//				this_timestamps.attitude                   &&
        HAL_UART_Transmit(&hlpuart1, (uint8_t*)received_all, MAVLINK_MAX_PACKET_LEN, HAL_MAX_DELAY);

		if (writing_status > false)
	    {
		    HAL_Delay(100);
	    }
		HAL_UART_Transmit(&hlpuart1, (uint8_t*)("No Error"), 8, HAL_MAX_DELAY);
	}
	return;
}


uint64_t get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return ((_time_stamp.tv_sec*1000000) +_time_stamp.tv_usec);
}

void Autopilot_Interface::set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp)
{
	mavlink_message_t msg;
	sp.type_mask=MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;
	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
//	printf("Give x:");
//	scanf("%f",&x);
//	printf("Give y:");
//	scanf("%f",&y);
//	printf("Give z:");
//	scanf("%f",&z);
	x=1;
	y=1;
	z=1;
	sp.x   = x;
	sp.y   = y;
	sp.z   = z;
	//sp.r   = r;
	uint8_t tx_buffer4[sizeof(mavlink_set_position_target_local_ned_t)];
    mavlink_msg_set_position_target_local_ned_encode(system_id, companion_id, &msg, &sp);
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)("Current Setpoint:\n"), 18, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1, tx_buffer4, sizeof(tx_buffer4), HAL_MAX_DELAY);

	float float_value15 = sp.x;
	uint8_t tx_buffer15[sizeof(float)];
	memcpy(tx_buffer15, &float_value15, sizeof(float));

	float float_value16 = sp.y;
	uint8_t tx_buffer16[sizeof(float)];
	memcpy(tx_buffer16, &float_value16, sizeof(float));

	float float_value18 = sp.z;
	uint8_t tx_buffer18[sizeof(float)];
	memcpy(tx_buffer18, &float_value18, sizeof(float));

	HAL_UART_Transmit(&hlpuart1, (uint8_t*)("POSITION SETPOINT XYZ:"), 22, HAL_MAX_DELAY);
	HAL_UART_Transmit(&hlpuart1, tx_buffer15, sizeof(float), HAL_MAX_DELAY);
	HAL_UART_Transmit(&hlpuart1, tx_buffer16, sizeof(float), HAL_MAX_DELAY);
	HAL_UART_Transmit(&hlpuart1, tx_buffer18, sizeof(float), HAL_MAX_DELAY);
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)("\n"), 1, HAL_MAX_DELAY);
}
//void set_velocity(float vx, float vy, float vz,mavlink_set_position_target_local_ned_t &sp)
//{
//	sp.type_mask=MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY;
//	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
//	sp.vx   = vx;
//	sp.vy   = vy;
//	sp.vz   = vz;
//	printf("VELOCITY SETPOINT XYZ = [ %.4f , %.4f , %.4f ] \n", sp.vx, sp.vy, sp.vz);
//}
//void set_acceleration(float ax, float ay, float az, mavlink_set_position_target_local_ned_t &sp)
//{
//	sp.type_mask=MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION & MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY;
//	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
//	sp.afx   = ax;
//	sp.afy   = ay;
//	sp.afz   = az;
//	//printf("ACCELERATION SETPOINT XYZ = [ %.4f , %.4f , %.4f ] \n", sp.ax, sp.ay, sp.az);
//}
void Autopilot_Interface::set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp)
{
	mavlink_message_t msg;
	sp.type_mask=sp.type_mask & MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE;
    HAL_UART_Transmit(&hlpuart1, (uint8_t*)("Give yaw:\n"), 11, HAL_MAX_DELAY);
//	printf("Give yaw:");
//	scanf("%f",&yaw);
	yaw=1;
	sp.yaw=yaw;
	uint8_t tx_buffer19[sizeof(mavlink_set_position_target_local_ned_t)];
//	memcpy(tx_buffer5, &sp, sizeof(mavlink_set_position_target_local_ned_t));
	mavlink_msg_set_position_target_local_ned_encode(system_id, companion_id, &msg, &sp);
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)("POSITION SETPOINT YAW:\n"), 23, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1, tx_buffer19, sizeof(huart1), HAL_MAX_DELAY);
	//printf("POSITION SETPOINT YAW = %.4f\n",sp.yaw);
}

//void set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp)
//{
//	sp.type_mask = sp.type_mask & MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE ;
//	sp.yaw_rate = yaw_rate;
//}


PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}
READCHAR_PROTOTYPE
{
	uint8_t data;
	HAL_UART_Receive(&hlpuart1, &data, 1, HAL_MAX_DELAY);
	return (int)data;
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


