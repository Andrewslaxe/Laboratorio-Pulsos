/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Start 0x06
#define Stop 0x07
#define Resistor 1 	//Resistencia en Ohms
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
uint8_t Connection = 0, Flag, EstAntx, Estatex, Sentidox, EstAnty, Estatey, Sentidoy;
uint32_t PosDeseadax = 0, PosDeseaday = 0, PosActualx = 0, PosActualy = 0;
int Valor,Valor2;
double rpm = 0, st = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void Poscalibrate();
void Send(int Cmd, int Info);
void Iniciar();
void LeerSensores();
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
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim3);
  uint8_t Current = 0,a = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(Connection == 1){
		  if(__HAL_TIM_GET_COUNTER(&htim3) >= 1000){ //Se envia cada 100ms
			  switch((a%5)+1){
			  	  case 1:
			  		Send(01,rpm);
			  		break;
			  	  case 2:
			  		Send(03, PosActualx);
			  		break;
			  	  case 3:
			  		Send(04, PosActualy);
			  		break;
			  	  case 4:
			  		Send(05, Sentidox);
			  		break;
			  	  case 5:
			  		Send(06, Sentidoy);
			  		break;
			  }
			  a++;
			  __HAL_TIM_SET_COUNTER(&htim3, 0);
		  }
		  else{
			  Poscalibrate();
			  //LeerSensores();
			  //HAL_ADC_Start(&hadc1);
			  //HAL_ADC_PollForConversion(&hadc1, 10);
			  //Current=HAL_ADC_GetValue(&hadc1);
			  //HAL_ADC_Stop(&hadc1);

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
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 96-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9600-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : StartSensor_Pin */
  GPIO_InitStruct.Pin = StartSensor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(StartSensor_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Sensor1_Pin Sensor2_Pin Sensor3_Pin Sensor4_Pin */
  GPIO_InitStruct.Pin = Sensor1_Pin|Sensor2_Pin|Sensor3_Pin|Sensor4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : StartSensor2_Pin Sen1_Pin */
  GPIO_InitStruct.Pin = StartSensor2_Pin|Sen1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
/*void LeerSensores(){
	switch(HAL_GPIO_ReadPin(Sensor1_GPIO_Port, Sensor1_Pin)){		//Lee sensor x
		case 0:
			if(HAL_GPIO_ReadPin(Sensor2_GPIO_Port, Sensor2_Pin) == 0){
				Estatex = 0;
			}
			else{
				Estatex = 1;
			}
			break;
		case 1:
  			if(HAL_GPIO_ReadPin(Sensor2_GPIO_Port, Sensor2_Pin) == 0){
  				Estatex = 2;
  			}
  			else{
  				Estatex = 3;
  			}
  			break;
	}

	switch(HAL_GPIO_ReadPin(Sensor3_GPIO_Port, Sensor3_Pin)){		//Lee sensor y
		case 0:
			if(HAL_GPIO_ReadPin(Sensor4_GPIO_Port, Sensor4_Pin) == 0){
					Estatey = 0;
			}
			else{
				Estatey = 1;
			}
			break;
		case 1:
	  		if(HAL_GPIO_ReadPin(Sensor4_GPIO_Port, Sensor4_Pin) == 0){
	  			Estatey = 2;
	  		}
	  		else{
	  			Estatey = 3;
	  		}
	  		break;
	}
  //Orientacion =1 Horario =2 Antihorario Estado 0 (0,0) Estado 1(0,1) Estado 2(1,0) Estado 3(1,1)
  switch(EstAntx){
    case 0:     
      if(Estatex == 1){    //Se activa solo el sensor 2
        PosActualx--;
      }
      else if(Estatex == 2){ //Se activa solo el sensor 1
        PosActualx++;
      }
    break;
    case 1:     //Se habia activado sensor 2
      if(Estatex == 0){    //Se activa el sensor 1 y 2
        PosActualx++;
      }
      else if(Estatex == 3){   //Se activan los dos sensores
        PosActualx--;
      }
    break;
    case 2:   //Se habia activado sensor 1
      if(Estatex == 0){
        PosActualx--;
      }
      else if(Estatex == 3){
        PosActualx++;
      }
    case 3:   //Se habia activado el sensor 1 y 2
      if(Estatex == 1){
        PosActualx++;
      }
      else if(Estatex == 2){
        PosActualx--;
      }
    break;
  }
  switch(EstAnty){
      case 0:
        if(Estatey == 1){    //Se activa solo el sensor 2
        	PosActualy--;
        }
        else if(Estatey == 2){ //Se activa solo el sensor 1
        	PosActualy++;
        }
      break;
      case 1:     //Se habia activado sensor 2
        if(Estatey == 0){    //Se activa el sensor 1 y 2
        	PosActualy++;
        }
        else if(Estatey == 3){   //Se activan los dos sensores
        	PosActualy--;
        }
      break;
      case 2:   //Se habia activado sensor 1
        if(Estatey == 0){
        	PosActualy--;
        }
        else if(Estatey == 3){
        	PosActualy++;
        }
      case 3:   //Se habia activado el sensor 1 y 2
        if(Estatey == 1){
        	PosActualy++;
        }
        else if(Estatey == 2){
        	PosActualy--;
        }
      break;
    }
  EstAntx = Estatex;
  EstAnty = Estatey;
}*/
void Poscalibrate(){

	if(PosActualx < PosDeseadax){
		TIM1->CCR1 = Valor;
		Sentidox = 1;	//Horario
	}
	else if(PosActualx == PosDeseadax){
		TIM1->CCR1 = Valor;
		Sentidox = 0;	//Detenido
	}
	else{
		TIM1->CCR1 = Valor;
		Sentidox = 2;	//Antihorario
	}
	/*
	if(PosActualy < PosDeseaday){
		TIM4->CCR3 = 99 * 650;
		Sentidoy = 1;
	}
	else if(PosActualy == PosDeseaday){
		TIM4->CCR3 = Valor2 * 650;
		TIM1->CCR2 = Valor2 * 650;
		Sentidoy = 0;
	}
	else{
		TIM4->CCR3 = 1;
		Sentidoy = 2;
	}*/
}
void Iniciar(){
	PosActualx = 12000;
	PosActualy = 12000;
	PosDeseadax = 0;
	PosDeseaday = 0;
}
void Send(int Cmd, int Info){
	int Size=0x00,Parity=0,Contador=0;
	uint8_t *Data;

	if(Info<256){
		Size=0x01;
		Data=(uint8_t*)malloc(6*sizeof(int));
		Data[3]=Info;
	}
	else if(Info < 65536){
		Size=0x02;
		Data=(uint8_t*)malloc(7*sizeof(int));
		Data[3]=0x00FF & (Info >> 8);
		Data[4]=0x00FF & Info;
	}
	else{
		Size=0x03;
		Data=(uint8_t*)malloc(8*sizeof(int));
		Data[3] = 0x00FF & (Info >> 16);
		Data[4] = 0x00FF & (Info >> 8);
		Data[5] = 0x00FF & Info;
	}
	Data[0]=Start;
	Data[1]=Size;
	Data[2]=Cmd;
	for(Contador=0;Contador<Size+3;Contador++){
		Parity^=Data[Contador];
	}
	Data[Size+3]=Parity;
	Data[Size+4]=Stop;

	Contador=sizeof(Data);
	CDC_Transmit_FS(Data,Size+5);
	free(Data);
}

void CDC_ReceiveCallBack(uint8_t* Buf, uint32_t Len){
	int Cmd,Aux,Parity=0,Size;
	double Temp=0;
	for(Aux=0;Aux<=Len;Aux++){
		Temp=Buf[Aux];
	}

	if(Buf[0]==Start && Buf[Len-1]==Stop){ //Protocolo
		Size=Buf[1];
		Cmd=Buf[2];
		if(Size==1){
			Temp=Buf[3];
		}
		else if(Size==2){
			Temp=(Buf[3]<< 8) | Buf[4];
		}
		else if(Size==3){
			Temp=(Buf[3]<< 16) | (Buf[4]<<8) | Buf[5];
		}
		for(Aux=0;Aux<Size+3;Aux++){
			Parity^=Buf[Aux];
		}
		Aux=Buf[Size+3];
		if(Buf[Size+3]==Parity){
			if(Cmd==1){ //Le pide a la STM enviar Info
				Connection = 1;
				Iniciar();
			}
			else if(Cmd == 0){ //Le pide que deje de enviar Info
				Connection = 0;
			}
			else if(Cmd == 2){ //PosDeseada en pulsos X
				PosDeseadax = Temp;
			}
			else if(Cmd == 3){
				PosDeseaday = Temp;
			}
			else if(Cmd == 4){ //Resetea el estado 
				Iniciar();
			}
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
		if(GPIO_Pin==Sensor1_Pin){
			if(Flag==0){
				Flag=1;
				__HAL_TIM_SET_COUNTER(&htim2, 0);
			}
			else{
				st=__HAL_TIM_GET_COUNTER(&htim2);
				if(st==0){
					rpm=0;
				}
				else{
					rpm=1000000/st;//
				}
				Flag=0;
			}
		}
		else if(GPIO_Pin == StartSensor_Pin){
			PosActualx = 0;
		}
		else if(GPIO_Pin == StartSensor2_Pin){
			PosActualy = 0;
		}
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
