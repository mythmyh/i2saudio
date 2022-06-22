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
typedef __packed struct
{
    uint32_t ChunkID;		   	//chunk id;脮芒脌茂鹿脤露篓脦陋"RIFF",录麓0X46464952
    uint32_t ChunkSize ;		   	//录炉潞脧麓贸脨隆;脦脛录镁脳脺麓贸脨隆-8
    uint32_t Format;	   			//赂帽脢陆;WAVE,录麓0X45564157
}ChunkRIFF ;
//fmt驴茅
typedef __packed struct
{
    uint32_t ChunkID;		   	//chunk id;脮芒脌茂鹿脤露篓脦陋"fmt ",录麓0X20746D66
    uint32_t ChunkSize ;		   	//脳脫录炉潞脧麓贸脨隆(虏禄掳眉脌篓ID潞脥Size);脮芒脌茂脦陋:20.
    uint16_t AudioFormat;	  	//脪么脝碌赂帽脢陆;0X01,卤铆脢戮脧脽脨脭PCM;0X11卤铆脢戮IMA ADPCM
	uint16_t NumOfChannels;		//脥篓碌脌脢媒脕驴;1,卤铆脢戮碌楼脡霉碌脌;2,卤铆脢戮脣芦脡霉碌脌;
	uint32_t SampleRate;			//虏脡脩霉脗脢;0X1F40,卤铆脢戮8Khz
	uint32_t ByteRate;			//脳脰陆脷脣脵脗脢;
	uint16_t BlockAlign;			//驴茅露脭脝毛(脳脰陆脷);
	uint16_t BitsPerSample;		//碌楼赂枚虏脡脩霉脢媒戮脻麓贸脨隆;4脦禄ADPCM,脡猫脰脙脦陋4
//	uint16_t ByteExtraData;		//赂陆录脫碌脛脢媒戮脻脳脰陆脷;2赂枚; 脧脽脨脭PCM,脙禄脫脨脮芒赂枚虏脦脢媒
}ChunkFMT;
//fact驴茅
typedef __packed struct
{
    uint32_t ChunkID;		   	//chunk id;脮芒脌茂鹿脤露篓脦陋"fact",录麓0X74636166;
    uint32_t ChunkSize ;		   	//脳脫录炉潞脧麓贸脨隆(虏禄掳眉脌篓ID潞脥Size);脮芒脌茂脦陋:4.
    uint32_t NumOfSamples;	  	//虏脡脩霉碌脛脢媒脕驴;
}ChunkFACT;
//LIST驴茅
typedef __packed struct
{
    uint32_t ChunkID;		   	//chunk id;脮芒脌茂鹿脤露篓脦陋"LIST",录麓0X74636166;
    uint32_t ChunkSize ;		   	//脳脫录炉潞脧麓贸脨隆(虏禄掳眉脌篓ID潞脥Size);脮芒脌茂脦陋:4.
}ChunkLIST;

//data驴茅
typedef __packed struct
{
    uint32_t ChunkID;		   	//chunk id;脮芒脌茂鹿脤露篓脦陋"data",录麓0X5453494C
    uint32_t ChunkSize ;		   	//脳脫录炉潞脧麓贸脨隆(虏禄掳眉脌篓ID潞脥Size)
}ChunkDATA;
typedef __packed struct
{
	ChunkRIFF riff;	//riff驴茅
	ChunkFMT fmt;  	//fmt驴茅
//	ChunkFACT fact;	//fact驴茅 脧脽脨脭PCM,脙禄脫脨脮芒赂枚陆谩鹿鹿脤氓
	ChunkDATA data;	//data驴茅
}__WaveHeader;

void recoder_wav_init(__WaveHeader* wavhead) //鲁玫脢录禄炉WAV脥路
{
	wavhead->riff.ChunkID=0X46464952;	//"RIFF"
	wavhead->riff.ChunkSize=0;			//禄鹿脦麓脠路露篓,脳卯潞贸脨猫脪陋录脝脣茫
	wavhead->riff.Format=0X45564157; 	//"WAVE"
	wavhead->fmt.ChunkID=0X20746D66; 	//"fmt "
	wavhead->fmt.ChunkSize=16; 			//麓贸脨隆脦陋16赂枚脳脰陆脷
	wavhead->fmt.AudioFormat=0X01; 		//0X01,卤铆脢戮PCM;0X01,卤铆脢戮IMA ADPCM
 	wavhead->fmt.NumOfChannels=2;		//脣芦脡霉碌脌
 	wavhead->fmt.SampleRate=16000;		//16Khz虏脡脩霉脗脢 虏脡脩霉脣脵脗脢
 	wavhead->fmt.ByteRate=wavhead->fmt.SampleRate*4;//脳脰陆脷脣脵脗脢=虏脡脩霉脗脢*脥篓碌脌脢媒*(ADC脦禄脢媒/8)
 	wavhead->fmt.BlockAlign=4;			//驴茅麓贸脨隆=脥篓碌脌脢媒*(ADC脦禄脢媒/8)
 	wavhead->fmt.BitsPerSample=16;		//16脦禄PCM
   	wavhead->data.ChunkID=0X61746164;	//"data"
 	wavhead->data.ChunkSize=0;			//脢媒戮脻麓贸脨隆,禄鹿脨猫脪陋录脝脣茫
}
int file_num=10;

 char names[FILE_SIZE][100];
	int file_index=0;

	FIL *ftemp;	  		//脦脛录镁2.

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
FIL *f_rec;
int count_t=0;
int end_t=0;
uint8_t wtext[] = "鎴戝氨鏄偅寮燬D!"; // ????
char lt[200];
uint8_t *i2srecbuf1;
uint8_t *i2srecbuf2;
uint16_t I2S_Buf0[4096] = { 0 };
uint16_t I2S_Buf1[4096] = { 0 };
uint32_t wavsize;		//wav脢媒戮脻麓贸脨隆(脳脰陆脷脢媒,虏禄掳眉脌篓脦脛录镁脥路!!)
FILINFO *fno;

void rec_i2s_dma_rx_callback(void)
{
	uint16_t bw;
	uint8_t res;

		if(DMA1_Stream3->CR&(1<<19))
		{
			res=f_write(f_rec,i2srecbuf1,4096,(UINT*)&bw);//脨麓脠毛脦脛录镁
			f_sync(f_rec);
			if(res)
			{
				printf("%dwrite error:%d\r\n",count_t,res);
			}
			else{
				if((count_t%40)==0){

									HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_3);
								}
			}
		}else
		{
	    	res=f_write(f_rec, i2srecbuf2,4096,(UINT*)&bw);//脨麓脠毛脦脛录镁
			f_sync(f_rec);

			if(res)
			{
				printf("%dwrite error:%d\r\n",count_t,res);
			}
		}
		wavsize+=I2S_RX_DMA_BUF_SIZE;

		count_t+=1;
		if(count_t==RECORDER_MAX_CIRCLE){

			end_t=1;


					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
		}

}
const uint16_t i2splaybuf[2]={0X0000,0X0000};//2赂枚16脦禄脢媒戮脻,脫脙脫脷脗录脪么脢卤I2S Master路垄脣脥.脩颅禄路路垄脣脥0.
FRESULT set_timestamp (
    char *obj,     /* Pointer to the file name */
    int year,
    int month,
    int mday,
    int hour,
    int min,
    int sec
)
{

    fno->fdate = (WORD)(((year - 1980) * 512U) | month * 32U | mday);
    fno->ftime = (WORD)(hour * 2048U | min * 32U | sec / 2U);
    retSD=f_utime(obj, fno);
    return retSD;
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

  RTC_DateTypeDef sdatestructure2;
  RTC_TimeTypeDef stimestructure2;
  printf("root user\r\n");
  HAL_Delay(1000);
  Audio_Player_Init();
 	WM8978_HPvol_Set(40,40);	//露煤禄煤脪么脕驴脡猫脰脙
 	WM8978_SPKvol_Set(50);
   HAL_Delay(1000);
 	WM8978_ADDA_Cfg(0,1);		//驴陋脝么ADC
 		WM8978_Input_Cfg(1,1,0);	//驴陋脝么脢盲脠毛脥篓碌脌(MIC&LINE IN)
 		WM8978_Output_Cfg(0,1);		//驴陋脝么BYPASS脢盲鲁枚
 		WM8978_MIC_Gain(46);		//MIC脭枚脪忙脡猫脰脙

 		WM8978_I2S_Cfg(2,0);





 		  HAL_SD_CardInfoTypeDef  SDCardInfo;
 		    void show_sdcard_info(void)
 		    {
 		    	uint64_t CardCap;	//SD鍗�?�锟�???????
 		    	HAL_SD_CardCIDTypeDef SDCard_CID;

 		    	HAL_SD_GetCardCID(&hsd,&SDCard_CID);	//鑾峰彇CID
 		    	HAL_SD_GetCardInfo(&hsd,&SDCardInfo);					//鑾峰彇SD鍗�?�俊锟�???????
 		    	switch(SDCardInfo.CardType)
 		    	{
 		    		case CARD_SDSC:
 		    		{
 		    			if(SDCardInfo.CardVersion == CARD_V1_X)
 		    				printf("Card Type:SDSC V1\r\n");
 		    			else if(SDCardInfo.CardVersion == CARD_V2_X)
 		    				printf("Card Type:SDSC V2\r\n");
 		    		}
 		    		break;
 		    		case CARD_SDHC_SDXC:printf("Card Type:SDHC\r\n");break;
 		    	}
 		    	CardCap=(uint64_t)(SDCardInfo.LogBlockNbr)*(uint64_t)(SDCardInfo.LogBlockSize);	//璁＄畻SD鍗�?�锟�???????
 		      printf("Card ManufacturerID:%d\r\n",SDCard_CID.ManufacturerID);					//鍒讹�?????锟藉晢ID
 		     	printf("Card RCA:%d\r\n",SDCardInfo.RelCardAdd);								//鍗＄浉�?�瑰湴锟�???????????
 		    	printf("LogBlockNbr:%d \r\n",(uint32_t)(SDCardInfo.LogBlockNbr));					//鏄剧ず閫昏緫鍧楁暟锟�???????????
 		    	printf("LogBlockSize:%d \r\n",(uint32_t)(SDCardInfo.LogBlockSize));					//鏄剧ず閫昏緫鍧楀ぇ锟�???????????
 		    	printf("Card Capacity:%d MB\r\n",(uint32_t)(CardCap>>20));							//鏄剧ず�?�归�????
 		     	printf("Card BlockSize:%d\r\n\r\n",SDCardInfo.BlockSize);						//鏄剧ず鍧�????ぇ锟�???????????
 		    }
 		   FATFS fs;

 		f_mount(&fs,"0",1);
    	f_mkdir("0:\\etcdh");
		my_mem_init(SRAMIN);			//鲁玫脢录禄炉脛脷虏驴脛脷麓忙鲁脴

    	//FIL * f_rec;

    	uint8_t wtext[] = "鎴戝氨鏄偅寮燬D!"; // ????
    	i2srecbuf1=mymalloc(SRAMIN,I2S_RX_DMA_BUF_SIZE);//I2S脗录脪么脛脷麓忙1脡锚脟毛
    	i2srecbuf2=mymalloc(SRAMIN,I2S_RX_DMA_BUF_SIZE);
    	//f_write(&f_rec, "aaa",3,(void *)&byteswritten);
    		I2S2_Init(I2S_STANDARD_PHILIPS,I2S_MODE_MASTER_TX,I2S_CPOL_LOW,I2S_DATAFORMAT_16B);	//路脡脌没脝脰卤锚脳录,脰梅禄煤路垄脣脥,脢卤脰脫碌脥碌莽脝陆脫脨脨搂,16脦禄脰隆鲁陇露脠
    			I2S2_SampleRate_Set(16000);	//脡猫脰脙虏脡脩霉脗脢
    		 	I2S2_TX_DMA_Init((uint8_t*)&i2splaybuf[0],(uint8_t*)&i2splaybuf[1],1); 		//脜盲脰脙TX DMA
    			DMA1_Stream4->CR&=~(1<<4);	//鹿脴卤脮麓芦脢盲脥锚鲁脡脰脨露脧(脮芒脌茂虏禄脫脙脰脨露脧脣脥脢媒戮脻)
    			I2S2ext_RX_DMA_Init(i2srecbuf1,i2srecbuf2,2048); 	//脜盲脰脙RX DMA
    		  	i2s_rx_callback=rec_i2s_dma_rx_callback;//禄脴碌梅潞炉脢媒脰赂wav_i2s_dma_callback
    			__WaveHeader *wavhead=0;

    					//鲁玫脢录禄炉wav脢媒戮脻
    	uint16_t bw;
    	uint32_t byteswritten;    // ?????
    	uint8_t res;
    	int status=0;
			ftemp=(FIL*)mymalloc(SRAMIN,sizeof(FIL));		//脦陋ftemp脡锚脟毛脛脷麓忙
			uint8_t resd;
    	//retSD = f_write(f_rec, wtext, sizeof(wtext), (void *)&byteswritten);	//鍦ㄦ枃浠跺唴鍐欏叆wtext鍐呯殑鍐呭

      //f_close(f_rec);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  switch(end_t){
	  case 0:
		  if(file_index<FILE_SIZE){
		  if(status==0){
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
	    	f_rec=(FIL *)mymalloc(SRAMIN,sizeof(FIL));		//驴陋卤脵FIL脳脰陆脷碌脛脛脷麓忙脟酶脫貌

  		 	I2S_Play_Start();	//驴陋脢录I2S脢媒戮脻路垄脣脥(脰梅禄煤)

  			I2S_Rec_Start(); 	//�????

  		   HAL_RTC_GetTime(&hrtc, &stimestructure, RTC_FORMAT_BIN);
  		      /* Get the RTC current Date */
  		    HAL_RTC_GetDate(&hrtc, &sdatestructure, RTC_FORMAT_BIN);
  			fno=(FILINFO*)mymalloc(SRAMIN,sizeof(FILINFO));		//脦陋ftemp脡锚脟毛脛脷麓忙

  		      /* Display date Format : yy/mm/dd */
  		      sprintf(lt,"0:\\etcdh\\%02d-%02d-%02d_%02d_%02d_%02d.wav",2000 + sdatestructure.Year, sdatestructure.Month, sdatestructure.Date,stimestructure.Hours, stimestructure.Minutes, stimestructure.Seconds);
  		      /* Display time Format : hh:mm:ss */

  		     resd=f_open(ftemp,names[file_index],FA_READ);
  	       set_timestamp(names[file_index-1], 2000 + sdatestructure.Year, sdatestructure.Month, sdatestructure.Date, stimestructure.Hours,stimestructure.Minutes,stimestructure.Seconds);

  		   		      if(res==FR_NO_FILE){

  		   		      }else{
  		   		    	  f_close(ftemp);
  		   		      res=f_unlink(names[file_index]);
  		   		      printf("remove%d\r\n",res);
  		   		      }
  		      memcpy(names[file_index],lt,strlen(lt)+1);
  		      printf("%s\n",names[file_index]);
			wavhead=(__WaveHeader*)mymalloc(SRAMIN,sizeof(__WaveHeader));//驴陋卤脵__WaveHeader脳脰陆脷碌脛脛脷麓忙脟酶脫貌
			recoder_wav_init(wavhead);
		    f_open(f_rec, names[file_index], FA_CREATE_ALWAYS | FA_WRITE);
	  		retSD=f_write(f_rec,(const void*)wavhead,sizeof(__WaveHeader),&bw);//脨麓脠毛脥路脢媒戮脻
	  		file_index+=1;
		  status=1;
		  }
		  }else{
			  file_index=0;
		  }
		  break;

	  case 1:

		  wavhead->riff.ChunkSize=wavsize+36;		//脮没赂枚脦脛录镁碌脛麓贸脨隆-8;
		  wavhead->data.ChunkSize=wavsize;		//脢媒戮脻麓贸脨隆
		  f_lseek(f_rec,0);						//脝芦脪脝碌陆脦脛录镁脥路.
		  f_write(f_rec,(const void*)wavhead,sizeof(__WaveHeader),&bw);//脨麓脠毛脥路脢媒戮脻


		  		      /* Display date Format : yy/mm/dd */
		  		  //   sprintf(lt,"0:\\etcdh\\%02d-%02d-%02d_%02d_%02d_%02d.wav",2000 + sdatestructure.Year, sdatestructure.Month, sdatestructure.Date,stimestructure.Hours, stimestructure.Minutes, stimestructure.Seconds);



		  f_close(f_rec);

	      myfree(SRAMIN,f_rec);		//脢脥路脜脛脷麓忙
	      myfree(SRAMIN,wavhead);
	      myfree(SRAMIN,fno);


		  I2S_Rec_Stop();
			//缁撴潫鏍囧織
			end_t=0;

			//缁撴潫鏃堕棿 300绾︾瓑锟�????????20s,
			count_t=0;

			//鏄惁鏄娆＄敓鎴愭柊鏂囦�????
			status=0;
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

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
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
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
  sTime.Hours = 0x8;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if ( HAL_OK != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_MAY;
  sDate.Date = 0x16;
  sDate.Year = 0x22;

  if ( HAL_OK != HAL_OK)
  {
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
static void MX_SDIO_SD_Init(void)
{

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

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
int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 1000);
	return ch;
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

