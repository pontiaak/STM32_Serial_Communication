/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************* ***********************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include "string.h"
#include "stdint.h" //Required by nanopb - protocol buffer implementation and For use in converting RecievedDataForPB2 buffer from hex into decimal
#include "stddef.h" //Required by nanopb - protocol buffer implementation
#include "stdbool.h" //Required by nanopb - protocol buffer implementation
#include "limits.h" //Required by nanopb - protocol buffer implementation
#include <stdlib.h>

#include "src/STM.pb.h"
#include "pb_common.h"
#include "pb.h"
#include "pb_encode.h"
#include "pb_decode.h"

#include "usbd_cdc_if.h" // USB user interface  - For communication USB 11
#include "math.h"
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
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */

int DebugIncriment = 0;

//Private variables used for USB Communication
uint8_t DataToSend[64]; // Table with data for sending
uint8_t MessageCounter = 0; // Counter of number of sent messages
uint8_t MessageLength = 0; // The length of messages for sending through USB function

//Private variables used for Receiving Protocol
uint8_t ReceivedData[64]; // Table with received data from USB communication
uint8_t ReceivedDataForPB[64]; // ReceivedData array in suitible for decoding by pbnano (HEX)
uint8_t ReceivedDataForPB2[64]; //!!!!!!here firstly the hex bit array will be written, but then it will be turned into a dec array by hexToDecimal function suitable for decoding!!!!!
uint8_t ReceivedDataFlag = 0; // Flag for receiving data

//Private variables used for ADC temperature
uint16_t MeasurementADC;
float Temperature;
float Vsense;

//nano protocol buffer serialization
uint8_t bufferPB[128];
uint8_t bufferPBread[128];
STMMessage message = STMMessage_init_zero;	//declaring message data structure and initialize it
pb_ostream_t stream;
pb_istream_t istream;

int int1;
int int2;
int int3;
int mainint;
bool status=true; //status of protobuf decoding function decode_message
int functionflag = 0; //used in decode_message
//FOR TEST PURPOSES uint8_t encoded_buffer[] = {0x08, 0x01, 0x10, 0x1A, 0x18, 0xA5, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x3F};
//FOR TEST PURPOSES size_t buffer_length = sizeof(encoded_buffer);

char combined_hex;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */


void decode_message(const uint8_t* buffer, size_t length) {
	functionflag = 1;

    // Create a nanopb input stream from the buffer
    pb_istream_t istream = pb_istream_from_buffer(buffer, length);

    // Decode the message
    status = pb_decode(&istream, STMMessage_fields, &message);
    if (status) {
        // Message decoding successful
        int1 = message.messageId;
        int2 = message.temperatureInt;
        int3 = message.temperatureFraction;
    } else {
    	int1 = message.messageId;
    	int2 = message.temperatureInt;
    	int3 = message.temperatureFraction;
    }
}

int hexCharToInt(char hexChar) {
    if (hexChar >= '0' && hexChar <= '9') {
        return hexChar - '0';
    } else if (hexChar >= 'A' && hexChar <= 'F') {
        return hexChar - 'A' + 10;
    } else if (hexChar >= 'a' && hexChar <= 'f') {
        return hexChar - 'a' + 10;
    }
    return -1;  // Invalid hex character
}


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
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

//nano protocol buffer serialization

  stream = pb_ostream_from_buffer(bufferPB, 15);
  message.messageId = 1;
  message.temperatureInt = 10;
  message.temperatureFraction = 100;
  message.fakeMeasure = 8;
  pb_encode(&stream, STMMessage_fields, &message);

  for(int i = 0; i<stream.bytes_written; i++){
      //Serial.printf("%02X",bufferPB[i]);
    }




  	//reseting all the diods
  	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

    //variables for ADC temperature measurements
    const float V25 = 0.76; // [Volts]
    const float Avg_slope = 0.0025; //[Volts/degree]
    const float SupplyVoltage = 3.0; // [Volts]
    const float ADCResolution = 4095.0;
    //variables for sending ADC temp
    //char *tmpSign;		//char that represents the "-" sign for the process of sending a float variable as 2 ints
    float tmpVal;

    int tmpInt1;	//acts as the integer part of Temperature from float
	float tmpFrac;      //a buffer for getting fraction part of Temperature
	int tmpInt2; 	//acts as the fraction part of Temperature, lates added to int part to look like a float


    //Flags for measurement beginning/end
    uint8_t FlagTemperatureMeasurement = 0;//flag for ADC temperature measurement
    uint8_t FlagDiod = 0;//flag for Diod 12 on/off


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_Delay(100);  //elimination of the contact vibration effect

	  for(int i = 0; i <sizeof(ReceivedData)-1 ; i++){
		  ReceivedDataForPB[i] = hexCharToInt(ReceivedData[i]);
		  if((ReceivedData[i*2]==13)&&(ReceivedData[i*2+1]==10)&&(ReceivedData[i*2+2]==0)){
		  			  ReceivedDataForPB[i*2]=0;
		  			  ReceivedDataForPB[i*2+1]=0;
		  }
		  ReceivedDataForPB2[i] = combined_hex = ((ReceivedDataForPB[i*2] & 0xF) << 4) | (ReceivedDataForPB[i*2+1] & 0xF); //combining two char variables into a single char variable representing a hexadecimal value
	  }

	  istream = pb_istream_from_buffer(ReceivedDataForPB2, sizeof(ReceivedDataForPB2));		//Helper function for creating an input stream that reads data from a memory buffer.
	  decode_message(ReceivedDataForPB2, sizeof(ReceivedDataForPB2)); // Call the decode_message function to decode the message


/* This code remains undeleted for future debug and testing purposes, will we deleted in final version
	  //"Bitwise concatenation" - need to transform received data to an array of bytes(uint8_t HEX)
	  for(int i = 0; i <sizeof(ReceivedData)-1 ; i++){

		  //ReceivedDataForPB[i]=(Recei	vedData[i] << 4) | ReceivedData[i+1];
		  ReceivedDataForPB[i]=(int)ReceivedData[i]-48; //very important - converting chars into ints from ReceivedData (ascii values of the characters are subtracted from each other), for example
		  if(ReceivedDataForPB[i]==221){ReceivedDataForPB[i]=0;}
		  if(ReceivedDataForPB[i]==218){ReceivedDataForPB[i]=0;}
		  if(ReceivedDataForPB[i]==208){ReceivedDataForPB[i]=0;}
		  ReceivedDataForPB2[i]=(ReceivedDataForPB[i*2]*10+ReceivedDataForPB[i*2+1]); //concatination to form struvtures suitable for protocol buffers
	  }
	  for (int i = 30; i <63 ; ++i){ //cleanup, if more than 30 bits will be used, decrese or delete this loop!!!!
		  ReceivedDataForPB2[i]=0;
	  }


	  istream = pb_istream_from_buffer(ReceivedDataForPB2, sizeof(ReceivedDataForPB2));		//Helper function for creating an input stream that reads data from a memory buffer.
	  pb_read(&istream, bufferPBread, 8);		//Read data from input stream. Always use this function, don’t try to call the stream callback directly.
	  decodeflag = read_ints(&istream, STMMessage_fields, &message2);


	  decodeflag = pb_decode(&istream, STMMessage_fields, &message2);		//Read and decode all fields of a structure. Reads until EOF on input stream.

	  PBstorage = message.temperatureInt;

	  pb_decode_varint32(&istream,&PBstorage);


	  // Call the decode_message function to decode the message
*/





// sSwitching function based on user input from android app (from converted and decoded protobuf message)
	  if((int1==1)&&(int2==10)&&(int3==100)&&(FlagTemperatureMeasurement == 0)){
		  FlagTemperatureMeasurement = 1;
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
	  }
	  if((int1==2)&&(int2==20)&&(int3==200)&&(FlagTemperatureMeasurement == 1)){
		  FlagTemperatureMeasurement = 0;
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
	  }
	  if((int1==3)&&(int2==30)&&(int3==300)&&(FlagDiod == 0)){
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
		  FlagDiod = 1;
	  }
	  if((int1==4)&&(int2==40)&&(int3==400)&&(FlagDiod == 1)){
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
		  FlagDiod = 0;
	  }

// Turning on and off measurements based on button press
	  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET){
	  		  			  if (FlagTemperatureMeasurement == 1){
	  		  				  FlagTemperatureMeasurement = 0;
	  		  				  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
	  		  				  ReceivedData[0]=8;
	  		  			  }else{
	  		  				  FlagTemperatureMeasurement = 1;
	  		  				  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
	  		  				  ReceivedData[0]=5;
	  		  			  }
	  }else{



// Gathering measurements if apropriate flag in ON
	  if (FlagTemperatureMeasurement == 1){
		  HAL_Delay(400);  //elimination of the contact vibration effect
		  	  HAL_ADC_Start(&hadc1);
			  if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) { // Waiting for the conversion to complete
				  MeasurementADC = HAL_ADC_GetValue(&hadc1);// Getting the measured value
		  	  	  Vsense = (SupplyVoltage*MeasurementADC)/ADCResolution;// Convert the measured value to voltage
		  	  	  Temperature = ((Vsense-V25)/Avg_slope)+25;// Temperature calculation

		  	  	  HAL_ADC_Start(&hadc1);// Start of new conversion
		  	  	  //tmpSign = (Temperature < 0) ? "-" : "";
		  	  	  tmpVal = (Temperature < 0) ? -Temperature : Temperature;


		  	  }
	  }

	  // Converting the Measurement data into apropriate format, serializing it usenig protocol buffers and sending it over serial communication
	  if (FlagTemperatureMeasurement == 1){
		  	  	  	  	  tmpInt1 = tmpVal;                  // Get the integer part of Temperature
		  		  	  	  tmpFrac = tmpVal - tmpInt1;      // Get fraction part of Temperature
		  		  	  	  tmpInt2 = trunc(tmpFrac * 10000);  // Turn fraction part into integer
		  		  	  	  //MessageLength = sprintf(DataToSend, "{TEMP:%s%d.%d;}\r", tmpSign, tmpInt1, tmpInt2, ReceivedData);
		  		  	  	  //CDC_Transmit_FS(DataToSend, MessageLength);
		  		  	  	  MessageCounter+=1;
		  		  	  	  stream = pb_ostream_from_buffer(bufferPB, sizeof(bufferPB));
		  		  	  	  message.messageId = MessageCounter;
		  		  	  	  message.temperatureInt = 1;//tmpInt1;
		  		  	  	  message.temperatureFraction = 1;//tmpInt2;
		  		  	  	  message.fakeMeasure = 1;//8;
		  		  	  	  pb_encode(&stream, STMMessage_fields, &message);
		  		  	  	  MessageLength = sprintf(DataToSend, "%02x%02x%02x%02x%02x%02x%02x ", bufferPB[0], bufferPB[1], bufferPB[2], bufferPB[3], bufferPB[4], bufferPB[5], bufferPB[6]);
		  		  	  	  CDC_Transmit_FS(DataToSend, MessageLength);
	  	  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
