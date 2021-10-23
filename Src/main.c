
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */
#define cs_set() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, SET)
#define cs_reset() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, RESET)
#define SPI_PORT      SPI1
#define SPI_SCK_PIN   GPIO_PIN_3   // PB13
#define SPI_MISO_PIN  GPIO_PIN_4    // PB14
#define SPI_CS_PIN    GPIO_PIN_0    // PB12
#define SPI_GPIO_PORT GPIOB
#define cs_strob() cs_reset(); cs_set()
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM1_Init(void);
static void MX_CAN_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint32_t v=0;
uint32_t v1=0;
uint32_t v2=0;
uint32_t v3=0;

//Переменные для получения температуры
long internal=0;
long centigrade=0;
long internal1=0;
long centigrade1=0;
long internal3=0;
long centigrade3=0;
long centigrade4=0;
long internal4=0;



//Массивы для получения значения
uint16_t data1[4];
uint16_t data2[4];

uint16_t data3[4];
uint16_t data4[4];

float a[8]={0};

//хуйня ебаная
char TC1_open=0;
char TC1_short_gnd=0;
char TC1_short_vcc=0;
long TC1_internal_temp=0;
long TC1_temp=0;
//сасок
char TC2_open=0;
char TC2_short_gnd=0;
char TC2_short_vcc=0;
long TC2_internal_temp=0;
long TC2_temp=0;

char TC3_open=0;
char TC3_short_gnd=0;
char TC3_short_vcc=0;
long TC3_internal_temp=0;
long TC3_temp=0;

char TC4_open=0;
char TC4_short_gnd=0;
char TC4_short_vcc=0;
long TC4_internal_temp=0;
long TC4_temp=0;
/* USER CODE END PFP */


/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{


	HAL_Init();



  /* Configure the system clock */
  SystemClock_Config();


  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_CAN_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, SET);  //b1
 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, SET); //b2
 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, SET); //b3
 	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, SET); //b4


  while (1)
  {

	  /*
	  		 for(i=0;i<=255;i++)
	  		 {
	  			 a[0]=i;
	  			 HAL_SPI_Transmit(&hspi1, (uint8_t*)a, 1, 5000);
	  			 cs_strob();
	  			 HAL_Delay(1000);

	  		 }
	  	   res();
	  		  sendbit(0);
	  		  sendbit(1);
	  		  sendbit(1);
	  		  sendbit(1);
	  		  sendbit(1);
	  		  sendbit(1);
	  		  sendbit(1);
	  		  sendbit(1);
	  		  latch();
	  		  HAL_Delay(100);
*/
	  		  //---------------ADC1-----------------//
	  		  HAL_ADCEx_InjectedStart(&hadc1); 						   //START ADC1
	  		  HAL_ADCEx_InjectedPollForConversion(&hadc1, 100);		   //Преобразование
	  		  a[0]=((float)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1))*3/4095; //КАНАЛ4 PA0
	  		  a[1]=((float)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2))*3/4095; //КАНАЛ4 PA1
	  		  a[2]=((float)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3))*3/4095; //КАНАЛ4 PA2
	  		  a[3]=((float)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_4))*3/4095; //КАНАЛ4 PA3
	  		  HAL_ADCEx_InjectedStop(&hadc1); //STOP

	  		  //---------------ADC2-----------------//
	  		  HAL_ADCEx_InjectedStart(&hadc2); 						   //START ADC2
	  		  HAL_ADCEx_InjectedPollForConversion(&hadc2, 100);          //Преобразование
	  		  a[4]=((float)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1))*3/4095; //КАНАЛ1 PA4
	  		  a[5]=((float)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2))*3/4095; //КАНАЛ2 PA5
	  		  a[6]=((float)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_3))*3/4095; //КАНАЛ3 PA6
	  		  a[7]=((float)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_4))*3/4095; //КАНАЛ4 PA7
	  		  HAL_ADCEx_InjectedStop(&hadc2); 						   //STOP

	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, RESET);  //b1
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, SET); //b2
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, SET); //b3
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, SET); //b4
	  		HAL_Delay(5);
	  		 HAL_SPI_Receive(&hspi1, &data1[0], 4, 100);
	  		HAL_Delay(5);
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, SET);  //b1
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, RESET); //b2
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, SET); //b3
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, SET); //b4
	  		HAL_Delay(5);
	  		HAL_SPI_Receive(&hspi1, &data2[0], 4, 100);
	  		HAL_Delay(5);
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, SET);  //b1
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, SET); //b2
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, RESET); //b3
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, SET); //b4
	  		HAL_Delay(5);
	  		HAL_SPI_Receive(&hspi1,&data3[0],4,100);
	  		HAL_Delay(5);
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, SET);  //b1
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, SET); //b2
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, SET); //b3
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, RESET); //b4
	  		HAL_Delay(5);
	  		HAL_SPI_Receive(&hspi1,&data4[0],4,100);
	  		HAL_Delay(5);
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, SET);  //b1
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, SET); //b2
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, SET); //b3
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, SET); //b4

/*
	  		TC1_open=(data1[3] & ( 1 << 0 )) >> 0;
	  		TC1_short_gnd=(data1[3] & ( 1 << 1 )) >> 1;
	  		TC1_short_vcc=(data1[3] & ( 1 << 2 )) >> 2;
	  		TC1_internal_temp=data1[2];//(data1[2]<<4)+(data1[3]>>4);
	  		TC1_temp=(data1[0]<<4)+(data1[1]>>4);


	  			TC2_open=(data2[3] & ( 1 << 0 )) >> 0;
	  			TC2_short_gnd=(data2[3] & ( 1 << 1 )) >> 1;
	  			TC2_short_vcc=(data2[3] & ( 1 << 2 )) >> 2;
	  			TC2_internal_temp=data2[2];//(data2[2]<<4)+(data2[3]>>4);
	  			TC2_temp=(data2[0]<<4)+(data2[1]>>4);

	  			TC3_open=(data3[3] & ( 1 << 0 )) >> 0;
	  			TC3_short_gnd=(data3[3] & ( 1 << 1 )) >> 1;
	  			TC3_short_vcc=(data3[3] & ( 1 << 2 )) >> 2;
	  			TC3_internal_temp=data3[2];//(data1[2]<<4)+(data1[3]>>4);
	  			TC3_temp=(data3[0]<<4)+(data3[1]>>4);

	  			TC4_open=(data4[3] & ( 1 << 0 )) >> 0;
	  			TC4_short_gnd=(data4[3] & ( 1 << 1 )) >> 1;
	  			TC4_short_vcc=(data4[3] & ( 1 << 2 )) >> 2;
	  			TC4_internal_temp=data4[2];//(data2[2]<<4)+(data2[3]>>4);
	  			TC4_temp=(data4[0]<<4)+(data4[1]>>4);

*/
	  		//*****DA1  Первая термопара******///
	  		  v1=(data1[0]<<16)|data1[1];
	  		  internal1 = (v1 >> 4) & 0x7FF;
	  		  internal1 *= 0.0625;
	  		  if ((v1 >> 4) & 0x800)
	  			  internal1 *= -1;
	  			  if (v1 & 0x80000000)
	  			  	 {
	  			  	 v1 = 0xFFFFC000 | ((v1 >> 18) & 0x00003FFFF);
	  			  	 }
	  			  else
	  			  	 {
	  			  	 v1 >>= 18;
	  			  	 }
	  		  centigrade1 = v1;
	  		  centigrade1 *= (-0.25);// ХЗ с этим минусом можешь убрать попробывать, если че
	  		//printf("ineternal: %f couple: %f . \r \n",internal,centigrade);

//*****DA2  Вторая термопара******///
	  	    v=(data2[0]<<16)|data2[1];
	  	    internal = (v >> 4) & 0x7FF;
	  	      internal *= 0.0625;
	  	      if ((v >> 4) & 0x800)
	  	    	  internal *= -1;
	  	      if (v & 0x80000000)
	  	      {
	  	        v = 0xFFFFC000 | ((v >> 18) & 0x00003FFFF);
	  	      }
	  	      else
	  	      {
	  	        v >>= 18;
	  	      }
	  	   centigrade = v;
	  	   centigrade *= (-0.25);
	  	      //printf("ineternal: %f couple: %f . \r \n",internal,centigrade);



	  	    //*****DA3  Третья термопара******ВОЗМОЖНО ПЕРЕПУТАЛ :) ХЗ//

		  	    v2=(data3[0]<<16)|data3[1];
		  	    internal3 = (v2 >> 4) & 0x7FF;
		  	      internal3 *= 0.0625;
		  	      if ((v2 >> 4) & 0x800)
		  	      internal3 *= -1;
		  	      if (v2 & 0x80000000)
		  	      {
		  	        v2 = 0xFFFFC000 | ((v2 >> 18) & 0x00003FFFF);
		  	      }
		  	      else
		  	      {
		  	        v2 >>= 18;
		  	      }
		  	    centigrade3 = v2;
		  	    centigrade3 *= (-0.25);
		  	      //printf("ineternal: %f couple: %f . \r \n",internal,centigrade);



			  	    //*****DA4  Четвертая термопара****** ВОЗМОЖНО ПЕРЕПУТАЛ :) ХЗ///

		  	    v3=(data4[0]<<16)|data4[1];
				internal4 = (v3 >> 4) & 0x7FF;
				internal4 *= 0.0625;
				if ((v3 >> 4) & 0x800)
				  internal4 *= -1;
				if (v3 & 0x80000000)
				{
				  v3 = 0xFFFFC000 | ((v3 >> 18) & 0x00003FFFF);
				}
				  else
				{
				  v3 >>= 18;
				}
				  centigrade4 = v3;
				  centigrade4 *= (-0.25);
			//printf("ineternal: %f couple: %f . \r \n",internal,centigrade);
				  HAL_Delay(500);

				  HAL_CAN_Transmit(centigrade1, 100);
				  HAL_CAN_Transmit(centigrade, 100);
				  HAL_CAN_Transmit(centigrade3, 100);
				  HAL_CAN_Transmit(centigrade4, 100);

  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;
  ADC_InjectionConfTypeDef sConfigInjected;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the ADC multi-mode 
    */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Injected Channel 
    */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedNbrOfConversion = 4;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_NONE;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Injected Channel 
    */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Injected Channel 
    */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Injected Channel 
    */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_4;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_4;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* ADC2 init function */
static void MX_ADC2_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;
  ADC_InjectionConfTypeDef sConfigInjected;

    /**Common config 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Injected Channel 
    */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedNbrOfConversion = 4;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_NONE;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Injected Channel 
    */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Injected Channel 
    */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Injected Channel 
    */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_4;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_4;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* CAN init function */
static void MX_CAN_Init(void)
{

  hcan.Instance = CAN;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SJW = CAN_SJW_1TQ;
  hcan.Init.BS1 = CAN_BS1_1TQ;
  hcan.Init.BS2 = CAN_BS2_1TQ;
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = DISABLE;
  hcan.Init.AWUM = DISABLE;
  hcan.Init.NART = DISABLE;
  hcan.Init.RFLM = DISABLE;
  hcan.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
