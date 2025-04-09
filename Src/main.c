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
#include "dma.h"
#include "i2c.h"
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
#define SYNC 0xC8;
#define RC 0x16;
#define UART_RX_BUF_SIZE 64
#define PACKET_START_1 0x57
#define PACKET_START_2 0xAB
#define PACKET_SIZE	26
#define CONTROL_MODE 0x20
#define CENETER_ANGLE 0x00
#define ANGULAR_VELOCITY 250
#define MSP_SET_RAW_RC 200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t counter = 0;
uint16_t counter1 = 0;
uint16_t counterrx = 0;
uint16_t counter1rx = 0;
uint16_t frame_type;

uint8_t uart_rx_byte;
uint8_t buffer[UART_RX_BUF_SIZE];
uint8_t packet[UART_RX_BUF_SIZE];
uint8_t packet_pos = 0;
uint8_t in_packet = 0;  

int16_t angle;
uint8_t angle_low;
uint8_t angle_high;

const uint8_t numOfChannels = 16;
const uint8_t srcBits = 11;
const uint16_t inputChannelMask = (1 << srcBits) - 1;

uint8_t bitsMerged = 0;
uint32_t readValue = 0;
uint8_t readByteIndex = 0;

uint8_t data_type;
uint8_t command_type;
uint8_t DMA_buffer[64];
uint8_t SPI_buffer[64];
uint8_t RxData_rc[26];
uint8_t SPIData_rc[26];
uint16_t rcData[16];
uint16_t SPIrcData[16];

struct CAN_Rx_Message Rx_Msg;
struct Channels ch;

uint8_t rxData[3];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void unpackChannelData(uint8_t *RxDataRC, uint16_t *RCData);
void ProcessUARTByte(uint8_t byte);
void SetAngle(uint16_t angle);


void reset_near_control(void);
void reset_far_control(void);
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
				reset_far_control();
    }
		if(huart->Instance == USART2){
		counterrx++;
		for(int i = 0; i<26; i++){
			if(DMA_buffer[i] == 0xC8){
				data_type = DMA_buffer[i+2];
				if(data_type == 0x16){
				memcpy(RxData_rc, &DMA_buffer[i], 26 * sizeof(RxData_rc[0]));
					}}}
		unpackChannelData(RxData_rc, rcData);
		HAL_UART_Receive_DMA(&huart2, DMA_buffer, sizeof(DMA_buffer));
		reset_near_control();					
	}
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi == &hspi2) {
			for(int i = 0; i<26; i++){
			if(SPI_buffer[i] == '$' && SPI_buffer[i+1] == 'M' && SPI_buffer[i+2] == '>'){}
				command_type = DMA_buffer[i+3];
				if(command_type == MSP_SET_RAW_RC){
					memcpy(SPIData_rc, &SPI_buffer[i], 26 * sizeof(SPIData_rc[0]));
					SPIrcData[0] = ((SPIData_rc[1] << 8) | SPIData_rc[0]);
					SPIrcData[1] = ((SPIData_rc[3] << 8) | SPIData_rc[2]);
					SPIrcData[2] = ((SPIData_rc[5] << 8) | SPIData_rc[4]);
					SPIrcData[3] = ((SPIData_rc[7] << 8) | SPIData_rc[6]);
					SPIrcData[4] = ((SPIData_rc[9] << 8) | SPIData_rc[8]);
					SPIrcData[5] = ((SPIData_rc[11] << 8) | SPIData_rc[10]);
					SPIrcData[6] = ((SPIData_rc[13] << 8) | SPIData_rc[12]);
					SPIrcData[7] = ((SPIData_rc[15] << 8) | SPIData_rc[14]);
				}}}
		HAL_SPI_Receive_IT(&hspi2, SPI_buffer, sizeof(SPI_buffer));
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

void unpackChannelData(uint8_t *RxDataRC, uint16_t *RCData)
{
	bitsMerged = 0;
	readValue = 0;
	readByteIndex = 3;

	for (uint8_t n = 0; n < numOfChannels; n++) {
		while (bitsMerged < srcBits) {
			uint8_t readByte = RxDataRC[readByteIndex++];
      readValue |= ((uint32_t) readByte) << bitsMerged;
      bitsMerged += 8;
			}
			RCData[n] = (readValue & inputChannelMask);
			readValue >>= srcBits;
			bitsMerged -= srcBits;
	}
}
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

void reset_near_control(void){ch.channel2 = 0;}
void reset_far_control(void){rcData[1] = 0;}

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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_I2C1_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
	//HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
  HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
	HAL_UART_Receive_DMA(&huart2, DMA_buffer, sizeof(DMA_buffer));
	
	
	HAL_SPI_Receive_IT(&hspi2, SPI_buffer, sizeof(SPI_buffer));	
	//HAL_SPI_Transmit_IT(&hspi2, txData, 3);
	
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
	if (SPIrcData[1]>0)
	{angle = (((rcData[1]-172)*0.4395)-360);}
	if (ch.channel2>0)
	{angle = ((0.3515 * ch.channel2) - 360);}
	if (rcData[1]>0)
	{angle = (((rcData[1]-172)*0.4395)-360);}
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
  }
	HAL_UART_Receive_DMA(&huart2, DMA_buffer, sizeof(DMA_buffer));
	
	HAL_Delay(50);
	//HAL_SPI_Transmit_IT(&hspi2, txData, 3);
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
