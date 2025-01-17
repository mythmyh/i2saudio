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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "audio_player.h"
#include "delay.h"
#include "i2s.h"
#include "malloc.h"
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
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
typedef __packed struct {
	uint32_t ChunkID;
	uint32_t ChunkSize;
	uint32_t Format;
} ChunkRIFF;
//fmt驴茅
typedef __packed struct {
	uint32_t ChunkID;
	uint32_t ChunkSize;
	uint16_t AudioFormat;
	uint16_t NumOfChannels;
	uint32_t SampleRate;
	uint32_t ByteRate;
	uint16_t BlockAlign;
	uint16_t BitsPerSample;
//	uint16_t ByteExtraData;
} ChunkFMT;
//fact驴茅
typedef __packed struct {
	uint32_t ChunkID;
	uint32_t ChunkSize;
	uint32_t NumOfSamples;
} ChunkFACT;
//LIST驴茅
typedef __packed struct {
	uint32_t ChunkID;
	uint32_t ChunkSize;
} ChunkLIST;

//data驴茅
typedef __packed struct {
	uint32_t ChunkID;
	uint32_t ChunkSize;
} ChunkDATA;
typedef __packed struct {
	ChunkRIFF riff;
	ChunkFMT fmt;
//	ChunkFACT fact;
	ChunkDATA data;
} __WaveHeader;

void recoder_wav_init(__WaveHeader* wavhead)
{
	wavhead->riff.ChunkID = 0X46464952;
	wavhead->riff.ChunkSize = 0;
	wavhead->riff.Format = 0X45564157;
	wavhead->fmt.ChunkID = 0X20746D66;
	wavhead->fmt.ChunkSize = 16;
	wavhead->fmt.AudioFormat = 0X01;
	wavhead->fmt.NumOfChannels = 2;
	wavhead->fmt.SampleRate = 16000;
	wavhead->fmt.ByteRate = wavhead->fmt.SampleRate * 4;
	wavhead->fmt.BlockAlign = 4;
	wavhead->fmt.BitsPerSample = 16;
	wavhead->data.ChunkID = 0X61746164;
	wavhead->data.ChunkSize = 0;
}
int file_num = 10;

char names[FILE_SIZE][33];
int file_index = 0;

FIL *ftemp=NULL;	  		//脦脛录镁2.

//  uint8_t res=f_open(ftemp,names[file_index],FA_READ);
//  		      if(res==FR_NO_FILE){
//
//  		      }else{
//  		      f_unlink(names[file_index]);
//  		      }
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
FIL *f_rec=NULL;
int count_t = 0;
int end_t = 0;
char lt[40];
char *lt2=NULL;
uint8_t *i2srecbuf1=NULL;
uint8_t *i2srecbuf2=NULL;
uint16_t I2S_Buf0[4096] = { 0 };
uint16_t I2S_Buf1[4096] = { 0 };
uint32_t wavsize;
FILINFO *fno=NULL;
uint16_t bw3;
void rec_i2s_dma_rx_callback(void) {
	if (DMA1_Stream3->CR & (1 << 19)) {
		f_write(f_rec, i2srecbuf1, 4096, (UINT*) &bw3);
		f_sync(f_rec);


		if ((count_t % 40) == 0) {

				HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3);
			}



	} else {
	 f_write(f_rec, i2srecbuf2, 4096, (UINT*) &bw3);		//脨麓脠毛脦脛录镁
		f_sync(f_rec);


	}
	wavsize += I2S_RX_DMA_BUF_SIZE;

	count_t += 1;

	if (count_t == RECORDER_MAX_CIRCLE) {

		end_t = 1;
		count_t=0;

		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3);
	}


}
char * joint3(char*s1,char*s2){

	char*result=malloc(strlen(s1)+strlen(s2)+1+2);
	if(result==NULL)exit(1);
	strcpy(result,s1);
	strcat(result,"\\");
	strcat(result,s2);
	return result;
}

FRESULT delete_files (char* path)
{
    FRESULT res;
    DIR dir;
    UINT i;
    FIL *ftemp2=NULL;	  		//脦脛录镁2.

    static FILINFO fno;
	static uint8_t layerDeeph=0;
	uint8_t j;
	char *file_path;
    res = f_opendir(&dir, path);
    if (res == FR_OK)
		layerDeeph+=4;
        for (;;)
		{
            res = f_readdir(&dir, &fno);
            if (res != FR_OK || fno.fname[0] == 0)
				break;
            if (fno.fattrib & AM_DIR)
			{
                i = strlen(path);
                sprintf(&path[i], "/%s", fno.fname);
				for(j=0;j<layerDeeph;j++)
					printf(" ");
               // res = scan_files(path);
              //  if (res != FR_OK) break;
                path[i] = 0;
            }
			else
			{
//				for(j=0;j<layerDeeph;j++)
//					printf(" ");
                printf("====>%s\r\n",fno.fname);
                file_path=joint3(path,fno.fname);
                uint8_t res2= f_open(ftemp2,file_path,FA_OPEN_ALWAYS | FA_WRITE|FA_READ);


                if(res2==FR_NO_FILE){

                         }else{
                             res2= f_unlink(file_path);

                            printf("delete status %d ",res2);

                         }
                break;

            }
}
            //char *file_path=joint3(path,fno.fname);
            printf("file +++path %s\r\n",file_path);

            f_close(ftemp2);
            free(file_path);
            f_closedir(&dir);

        //}
		layerDeeph-=4;


    return res;
}



FRESULT scan_files (char* path)
{
    FRESULT res;
    DIR dir;
    static FILINFO fno;
	static uint8_t layerDeeph=0;
    res = f_opendir(&dir, path);
    if (res == FR_OK)
	{
		layerDeeph+=4;
        for (;;)
		{
            res = f_readdir(&dir, &fno);
            if (res != FR_OK || fno.fname[0] == 0)
				break;
            if (fno.fattrib & AM_DIR)
			{

            }
			else
			{

                printf("scan file %s\r\n",fno.fname);
            }
        }
        f_closedir(&dir);
		layerDeeph-=4;
    }

    return res;
}











const uint16_t i2splaybuf[2] = { 0X0000, 0X0000 };//2赂枚16脦禄脢媒戮脻,脫脙脫脷脗录脪么脢卤I2S Master路垄脣脥.脩颅禄路路垄脣脥0.
FRESULT set_timestamp(char *obj, /* Pointer to the file name */
int year, int month, int mday, int hour, int min, int sec) {

	fno->fdate = (WORD) (((year - 1980) * 512U) | month * 32U | mday);
	fno->ftime = (WORD) (hour * 2048U | min * 32U | sec / 2U);
	retSD = f_utime(obj, fno);
	return retSD;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_I2C1_Init();
	MX_I2S2_Init();
	MX_USART1_UART_Init();
	MX_SDIO_SD_Init();
	MX_FATFS_Init();
	MX_RTC_Init();
	/* USER CODE BEGIN 2 */

	RTC_DateTypeDef sdatestructure;
	RTC_TimeTypeDef stimestructure;
	HAL_Delay(1000);
	Audio_Player_Init();
	WM8978_HPvol_Set(40, 40);
	WM8978_SPKvol_Set(50);
	HAL_Delay(1000);
	WM8978_ADDA_Cfg(0, 1);
	WM8978_Input_Cfg(1, 1, 0);
	WM8978_Output_Cfg(0, 1);
	WM8978_MIC_Gain(46);
	WM8978_I2S_Cfg(2, 0);
	my_mem_init(SRAMIN);

	FATFS fs;
	f_mount(&fs, "0", 1);
	f_mkdir("0:\\etcdh");
	printf("just a function");
	//delete_files("0:\\etcdh");

	//scan_files("0:\\etcdh");
	int abc[2]={300,400};
	delay_ms(2500);

	i2srecbuf1 = mymalloc(SRAMIN, I2S_RX_DMA_BUF_SIZE);
	i2srecbuf2 = mymalloc(SRAMIN, I2S_RX_DMA_BUF_SIZE);
	I2S2_Init(I2S_STANDARD_PHILIPS, I2S_MODE_MASTER_TX, I2S_CPOL_LOW,
	I2S_DATAFORMAT_16B);
	I2S2_SampleRate_Set(16000);
	I2S2_TX_DMA_Init((uint8_t*) &i2splaybuf[0], (uint8_t*) &i2splaybuf[1], 1);
	DMA1_Stream4->CR &= ~(1 << 4);
	I2S2ext_RX_DMA_Init(i2srecbuf1, i2srecbuf2, 2048);
	i2s_rx_callback = rec_i2s_dma_rx_callback;
	__WaveHeader *wavhead=0;


	uint16_t bw;
	uint8_t res;
	int status = 0;
	ftemp = (FIL*) mymalloc(SRAMIN, sizeof(FIL));

	//f_close(f_rec);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		switch (end_t) {
		case 0:
			if (file_index < FILE_SIZE) {
				if (status == 0) {
					HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3);
					f_rec = (FIL*) mymalloc(SRAMIN, sizeof(FIL));
					I2S_Play_Start();
					I2S_Rec_Start();
					HAL_RTC_GetTime(&hrtc, &stimestructure, RTC_FORMAT_BIN);
					HAL_RTC_GetDate(&hrtc, &sdatestructure, RTC_FORMAT_BIN);
					fno = (FILINFO*) mymalloc(SRAMIN, sizeof(FILINFO));	//脦陋ftemp脡锚脟毛脛脷麓忙
					/* Display date Format : yy/mm/dd */
					lt2=(char*)malloc(40*sizeof(char));
					sprintf(lt2, "0:\\etcdh\\%02d-%02d-%02d_%02d_%02d_%02d.wav",
							2000 + sdatestructure.Year, sdatestructure.Month,
							sdatestructure.Date, stimestructure.Hours,
							stimestructure.Minutes, stimestructure.Seconds);
					/* Display time Format : hh:mm:ss */
					f_open(ftemp, lt2, FA_READ);


					if (res == FR_NO_FILE) {

					} else {
						f_close(ftemp);
						set_timestamp(lt2,
														2000 + sdatestructure.Year, sdatestructure.Month,
														sdatestructure.Date, stimestructure.Hours,
														stimestructure.Minutes, stimestructure.Seconds);
						//res = f_unlink(lt2);

						printf("file_index %d\r\n", file_index);
					}
					wavhead = (__WaveHeader*) mymalloc(SRAMIN,sizeof(__WaveHeader ));
					recoder_wav_init(wavhead);
					f_open(f_rec, lt2,
							FA_CREATE_ALWAYS | FA_WRITE);
					retSD = f_write(f_rec, (const void*) wavhead,
							sizeof(__WaveHeader ),(void *) &bw);
					file_index += 1;
					status = 1;
				}
			} else {
				//delete first file
				scan_files("0:\\etcdh");

				delete_files("0:\\etcdh");

				file_index-=1;


			}
			break;

		case 1:


			wavhead->riff.ChunkSize = wavsize + 36;		//脮没赂枚脦脛录镁碌脛麓贸脨隆-8;
			wavhead->data.ChunkSize = wavsize;		//脢媒戮脻麓贸脨隆
			f_lseek(f_rec, 0);						//脝芦脪脝碌陆脦脛录镁脥路.
			f_write(f_rec, (const void*) wavhead, sizeof(__WaveHeader ), (void *)&bw);//脨麓脠毛脥路脢媒戮脻

			/* Display date Format : yy/mm/dd */
			//   sprintf(lt,"0:\\etcdh\\%02d-%02d-%02d_%02d_%02d_%02d.wav",2000 + sdatestructure.Year, sdatestructure.Month, sdatestructure.Date,stimestructure.Hours, stimestructure.Minutes, stimestructure.Seconds);
			f_close(f_rec);
			set_timestamp(lt2,2000 + sdatestructure.Year, sdatestructure.Month,
															sdatestructure.Date, stimestructure.Hours,
															stimestructure.Minutes, stimestructure.Seconds);
			free(lt2);
			myfree(SRAMIN, f_rec);		//脢脥路脜脛脷麓忙
			myfree(SRAMIN, wavhead);
			myfree(SRAMIN, fno);
			I2S_Rec_Stop();
			end_t = 0;
			count_t = 0;
			status = 0;
			break;

		}

		delay_ms(5);
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE
			| RCC_OSCILLATORTYPE_LSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

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
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief I2S2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2S2_Init(void) {

	/* USER CODE BEGIN I2S2_Init 0 */

	/* USER CODE END I2S2_Init 0 */

	/* USER CODE BEGIN I2S2_Init 1 */

	/* USER CODE END I2S2_Init 1 */
	hi2s2.Instance = SPI2;
	hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
	hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
	hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
	hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
	hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_16K;
	hi2s2.Init.CPOL = I2S_CPOL_LOW;
	hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
	hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
	if (HAL_I2S_Init(&hi2s2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2S2_Init 2 */

	/* USER CODE END I2S2_Init 2 */

}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void) {

	/* USER CODE BEGIN RTC_Init 0 */

	/* USER CODE END RTC_Init 0 */

	RTC_TimeTypeDef sTime = { 0 };
	RTC_DateTypeDef sDate = { 0 };

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
	if (HAL_RTC_Init(&hrtc) != HAL_OK) {
		Error_Handler();
	}

	/* USER CODE BEGIN Check_RTC_BKUP */

	/* USER CODE END Check_RTC_BKUP */

	/** Initialize RTC and set the Time and Date
	 */
	sTime.Hours = 0x8;
	sTime.Minutes = 0x0;
	sTime.Seconds = 0x0;
	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	if (HAL_OK != HAL_OK) {
		Error_Handler();
	}
	sDate.WeekDay = RTC_WEEKDAY_MONDAY;
	sDate.Month = RTC_MONTH_MAY;
	sDate.Date = 0x16;
	sDate.Year = 0x22;

	if (HAL_OK != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN RTC_Init 2 */

	/* USER CODE END RTC_Init 2 */

}

/**
 * @brief SDIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_SDIO_SD_Init(void) {

	/* USER CODE BEGIN SDIO_Init 0 */

	/* USER CODE END SDIO_Init 0 */

	/* USER CODE BEGIN SDIO_Init 1 */

	/* USER CODE END SDIO_Init 1 */
	hsd.Instance = SDIO;
	hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
	hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
	hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
	hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
	hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_ENABLE;
	hsd.Init.ClockDiv = 5;
	/* USER CODE BEGIN SDIO_Init 2 */

	/* USER CODE END SDIO_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

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
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
	/* DMA2_Stream3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
	/* DMA2_Stream6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

	/*Configure GPIO pin : PE3 */
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, 1000);
	return ch;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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

