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
#define WHEEL_CHANNEL 1

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
int16_t angle;
uint8_t angle_low;
uint8_t angle_high;

const uint8_t numOfChannels = 16;
const uint8_t srcBits = 11;
const uint16_t inputChannelMask = (1 << srcBits) - 1;

uint8_t bitsMerged = 0;
uint32_t readValue = 0;
uint8_t readByteIndex = 0;

uint8_t UART_buffer[64];
uint8_t DMA_buffer2[64];
uint8_t DMA_buffer6[64];
uint8_t SPI_buffer[64];
uint8_t RxDataUart1[26];
uint8_t RxDataUart2[26];
uint8_t RxDataUart6[26];
uint8_t RxDataSPI[26];
uint8_t crc_buffer[26];

uint16_t RC_HID[16];
uint16_t RC_TBS[16];
uint16_t RC_ELRS[16];
uint16_t RC_SPI[16];

struct CAN_Rx_Message Rx_Msg;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void unpack_hid(uint8_t *array1);
void unpack_lrs(uint8_t *array1, uint16_t *array2);
void SetAngle(uint16_t angle);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

struct CAN_Rx_Message{
	int8_t data0;
	int8_t data1;
	int8_t data2;
	int16_t data3;
	int16_t data4;
	int8_t data5;
	int8_t data6;
	int8_t data7;
	float angle;};

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		for(int i = 0; i<26; i++){
			if(UART_buffer[i] == 0x57){
				if(UART_buffer[i+2] == 0x88){
				memcpy(RxDataUart1, &UART_buffer[i], 26 * sizeof(RxDataUart1[0]));
				unpack_hid(RxDataUart1);}}}}
		
	if(huart->Instance == USART2){
		for(int i = 0; i<26; i++){
			if(DMA_buffer2[i] == 0xC8){
				if(DMA_buffer2[i+2] == 0x16){
				memcpy(RxDataUart2, &DMA_buffer2[i], 26 * sizeof(RxDataUart2[0]));}}}
				unpack_lrs(RxDataUart2, RC_TBS);}

	if (huart->Instance == USART6){
		for(int i = 0; i<26; i++){
			if(DMA_buffer6[i] == 0xC8){
				if(DMA_buffer6[i+2] == 0x16){
				memcpy(RxDataUart6, &DMA_buffer6[i], 26 * sizeof(RxDataUart6[0]));}}}
				unpack_lrs(RxDataUart6, RC_ELRS);}
	
				HAL_UART_Receive_DMA(&huart2, DMA_buffer2, sizeof(DMA_buffer2));
				HAL_UART_Receive_DMA(&huart6, DMA_buffer6, sizeof(DMA_buffer6));
				HAL_UART_Receive_IT(&huart1, UART_buffer, sizeof(UART_buffer));}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi == &hspi2) {
		for(int i = 0; i<26; i++){
			if(SPI_buffer[i] == '$' && SPI_buffer[i+1] == 'M' && SPI_buffer[i+2] == '>' && SPI_buffer[i+4] == MSP_SET_RAW_RC){
				memcpy(RxDataSPI, &SPI_buffer[i+5], 26 * sizeof(RxDataSPI[0]));
				RC_SPI[0] = ((RxDataSPI[1] << 8) | RxDataSPI[0]);
				RC_SPI[1] = ((RxDataSPI[3] << 8) | RxDataSPI[2]);
				RC_SPI[2] = ((RxDataSPI[5] << 8) | RxDataSPI[4]);
				RC_SPI[3] = ((RxDataSPI[7] << 8) | RxDataSPI[6]);
				RC_SPI[4] = ((RxDataSPI[9] << 8) | RxDataSPI[8]);
				RC_SPI[5] = ((RxDataSPI[11] << 8) | RxDataSPI[10]);
				RC_SPI[6] = ((RxDataSPI[13] << 8) | RxDataSPI[12]);
				RC_SPI[7] = ((RxDataSPI[15] << 8) | RxDataSPI[14]);}}}
	HAL_SPI_Receive_IT(&hspi2, SPI_buffer, sizeof(SPI_buffer));}

void unpack_hid(uint8_t *array1) {
	RC_HID[0] = (((RxDataUart1[9] << 8) | RxDataUart1[8]));
	RC_HID[1] = (((RxDataUart1[11] << 8) | RxDataUart1[10]));
	RC_HID[2] = (((RxDataUart1[13] << 8) | RxDataUart1[12]));
	RC_HID[3] = (((RxDataUart1[15] << 8) | RxDataUart1[14]));
	RC_HID[4] = (((RxDataUart1[17] << 8) | RxDataUart1[16]));
	RC_HID[5] = (((RxDataUart1[19] << 8) | RxDataUart1[18]));
	RC_HID[6] = (((RxDataUart1[21] << 8) | RxDataUart1[20]));
	RC_HID[7] = (((RxDataUart1[23] << 8) | RxDataUart1[22]));}

void unpack_lrs(uint8_t *array1, uint16_t *array2){
	bitsMerged = 0;
	readValue = 0;
	readByteIndex = 3;
	for (uint8_t n = 0; n < numOfChannels; n++) {
		while (bitsMerged < srcBits) {
			uint8_t readByte = array1[readByteIndex++];
      readValue |= ((uint32_t) readByte) << bitsMerged;
      bitsMerged += 8;}
			array2[n] = (readValue & inputChannelMask);
			readValue >>= srcBits;
			bitsMerged -= srcBits;}}

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
  CANSPI_Transmit(&txMessage);}


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

  HAL_UART_Receive_IT(&huart1, UART_buffer, sizeof(UART_buffer));
	HAL_UART_Receive_DMA(&huart2, DMA_buffer2, sizeof(DMA_buffer2));
	HAL_UART_Receive_DMA(&huart6, DMA_buffer6, sizeof(DMA_buffer6));
	HAL_SPI_Receive_IT(&hspi2, SPI_buffer, sizeof(SPI_buffer));	
	
	
  //uCAN_MSG txMessage;
	uCAN_MSG rxMessage;
  CANSPI_Initialize();
	
  HAL_Delay(1000);
	SetAngle(0);
	HAL_Delay(1000);

  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){ 
	if (RC_SPI[WHEEL_CHANNEL]>0){angle = (((RC_SPI[WHEEL_CHANNEL]-172)*0.4395)-360);}
	else if (RC_HID[WHEEL_CHANNEL]>0){angle = ((0.3515 * RC_HID[WHEEL_CHANNEL]) - 360);}
	else if (RC_TBS[WHEEL_CHANNEL]>0){angle = (((RC_TBS[WHEEL_CHANNEL]-172)*0.4395)-360);}
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
		Rx_Msg.angle = ((Rx_Msg.data3 << 8) + Rx_Msg.data4) - 1024;}
	
	HAL_UART_Receive_IT(&huart1, UART_buffer, sizeof(UART_buffer));
	HAL_UART_Receive_DMA(&huart2, DMA_buffer2, sizeof(DMA_buffer2));
	HAL_UART_Receive_DMA(&huart6, DMA_buffer2, sizeof(DMA_buffer6));
	HAL_SPI_Receive_IT(&hspi2, SPI_buffer, sizeof(SPI_buffer));	}
	
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
