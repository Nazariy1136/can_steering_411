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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h> 
#include "stm32f4xx_hal.h"
#include "CANSPI.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_RX_BUF_SIZE 64
#define PACKET_START_1 0x57
#define PACKET_START_2 0xAB
#define PACKET_SIZE	26
#define CONTROL_MODE 0x20
#define CENETER_ANGLE 0x00
#define ANGULAR_VELOCITY 250
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t counter = 0;
uint16_t counter1 = 0;
uint16_t frame_type;

uint8_t uart_rx_byte;
uint8_t buffer[UART_RX_BUF_SIZE];
uint8_t packet[UART_RX_BUF_SIZE];
uint8_t packet_pos = 0;
uint8_t in_packet = 0;  

int16_t angle;
uint8_t angle_low;
uint8_t angle_high;

struct CAN_Rx_Message Rx_Msg;
struct Channels ch;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ProcessUARTByte(uint8_t byte);
void SetAngle(uint16_t angle);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

struct CAN_Rx_Message
{
	int8_t data0;
	int8_t data1;
	int8_t data2;
	int16_t data3;
	int16_t data4;
	int8_t data5;
	int8_t data6;
	int8_t data7;
	float angle;
};

struct Channels
{
	int16_t channel1;
	int16_t channel2;
	int16_t channel3;
	int16_t channel4;
	int16_t channel5;
	int16_t channel6;
	int16_t channel7;
	int16_t channel8;
};

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	counter++;
    if (huart->Instance == USART1) {
        ProcessUARTByte(uart_rx_byte);
        HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
    }
}

void ProcessUARTByte(uint8_t byte) {
	if (!in_packet) {
		if (byte == PACKET_START_1) {
			packet_pos = 0;
			buffer[packet_pos++] = byte;
		}
		else{
		buffer[packet_pos++] = byte;
		frame_type = buffer[2];
		if (packet_pos >= PACKET_SIZE && buffer[2] == 0x88 && buffer[0] == PACKET_START_1) {
				counter1++;
				memcpy(packet, buffer, PACKET_SIZE);
				ch.channel1 = (((packet[9] << 8) | packet[8]));
				ch.channel2 = (((packet[11] << 8) | packet[10]));
				ch.channel3 = (((packet[13] << 8) | packet[12]));
				ch.channel4 = (((packet[15] << 8) | packet[14]));
			  ch.channel5 = (((packet[17] << 8) | packet[16]));
				ch.channel6 = (((packet[19] << 8) | packet[18]));
				ch.channel7 = (((packet[21] << 8) | packet[20]));
				ch.channel8 = (((packet[23] << 8) | packet[22]));
				in_packet = 0;
	}}}}

void SetAngle(uint16_t angle){
	uCAN_MSG txMessage;
	angle_low = (angle + 1024) & 0xFF;
	angle_high =  (angle + 1024) >> 8;
	txMessage.frame.id = 0x469;
	txMessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
  txMessage.frame.dlc = 8;
  txMessage.frame.data0 = CONTROL_MODE;
	txMessage.frame.data1 = 0x00;
	txMessage.frame.data2 = 0x00;
  txMessage.frame.data3 = angle_high;
  txMessage.frame.data4 = angle_low;
  txMessage.frame.data5 = 0x00;
	txMessage.frame.data6 = ANGULAR_VELOCITY;
  txMessage.frame.data7 = txMessage.frame.data0 ^ txMessage.frame.data1 ^ txMessage.frame.data2 
	^ txMessage.frame.data3 ^ txMessage.frame.data4 ^ txMessage.frame.data5 ^ txMessage.frame.data6;
  CANSPI_Transmit(&txMessage);
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
	
  //uCAN_MSG txMessage;
	uCAN_MSG rxMessage;
  CANSPI_Initialize();
	
  HAL_Delay(1000);
	SetAngle(0);
	HAL_Delay(1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	angle = 0.3515 * ch.channel2 - 360;
	SetAngle(angle);
	
	if(CANSPI_Receive(&rxMessage)){
		Rx_Msg.data0 = rxMessage.frame.data0;
		Rx_Msg.data1 = rxMessage.frame.data1;
		Rx_Msg.data2 = rxMessage.frame.data2;
		Rx_Msg.data3 = rxMessage.frame.data3;
		Rx_Msg.data4 = rxMessage.frame.data4;
		Rx_Msg.data5 = rxMessage.frame.data5;
		Rx_Msg.data6 = rxMessage.frame.data6;
		Rx_Msg.data7 = rxMessage.frame.data7;
		Rx_Msg.angle = ((Rx_Msg.data3 << 8) + Rx_Msg.data4) - 1024;
  }}
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
