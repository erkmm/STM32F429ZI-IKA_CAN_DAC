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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "string.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

#define CAN_ID1 0x101
#define CAN_ID2 0x102
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile int16_t current_effective_speed = 0; // Motora uygulanan anlık hız

uint32_t Actuator_Feedback[2];

uint8_t received_data[8] = {0}; // Gelen veriler
uint8_t TxData[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

uint8_t a1[8] = {0};
uint8_t a2[8] = {0};
uint8_t a3[8] = {0};
uint8_t a4[8] = {0};

volatile uint16_t l_dac_value = 0;
volatile uint16_t r_dac_value = 0;

volatile int16_t r_s_value = 0;
volatile int16_t l_s_value = 0;

volatile int16_t i_direction_value = 0;

volatile uint16_t speed_value = 0;
volatile int16_t i_speed_value = 0;

volatile int16_t i_pivot_value = 0; // i_topuk_value = 0;
volatile int16_t s_pivot_value = 0; // s_topuk_value = 0;

volatile bool left_direction = 0;  // sola_topuk
volatile bool right_direction = 0; // saga_topuk

bool ileri_komutu = 0;

volatile int16_t effective_speed = 0;
volatile int16_t scale_value = 0;

volatile struct
{
    uint16_t speed_value;      // Byte0-1
    int16_t direction_value;  // Byte2-3
    uint8_t auto_speed_value; // Byte4
    // Byte5
    uint8_t brake_status : 1;       // fren_bilgisi
    uint8_t auto_manual_mode : 1;   // auto_manual
    uint8_t headlight_state : 1;    // far_durumu_bilgisi
    uint8_t connection_status : 1;  // baglanti_kontrol

    uint8_t reverse_direction; // Byte6
    uint8_t control_mode; // Byte7
}M100_flags_t;

volatile struct
{
    uint8_t remote_emergency : 1;   // kumanda_acil
    uint8_t mine_emergency : 1;     // mayin_acil
    uint8_t barrier_emergency : 1; // engel_acil
    uint8_t reserved : 5; // kullanılmayan bitler
} M101_data_t;

volatile struct
{
    uint16_t pivot_value; // pivot_value topuk_value
} M103_data_t;

bool m100 = 0;
bool m101 = 0;
bool m102 = 0;
bool m103 = 0;

uint32_t lastCanTick = 0;
bool canTimeout = false;

static uint32_t emergencyReleaseTime = 0;

volatile bool isInEmergencyState = false;

uint8_t break_flag = 0;

uint32_t lastDirectionChangeTime = 0;
uint8_t lastDirectionValue = 0xFF; // İlk değer varsayılan (farklı bir değer)

uint16_t mainTick = 0;
uint16_t main_Timeout = 0;
uint8_t state = 0;

volatile uint32_t last_direction_change_tick = 0;
static int16_t previous_direction_value = 0;
static uint32_t last_change_time = 0;

static uint32_t last_change_time2 = 0;


static int16_t previous_brake_value = 0;
uint32_t TxMailbox;
uint32_t TxMailbox2;

volatile uint32_t last_direction_change_tick_2 = 0;

uint32_t brake_timer = 0;
uint8_t brake_active = 0;

int16_t *TX_Ser_Val;

uint8_t brake_state = 0;
uint8_t brake_flag = 0;
uint32_t brake_timer1 = 0;
uint32_t brake_timer2 = 0;

uint32_t brake_action_start_time = 0; // Fren eyleminin başladığı zaman

static uint8_t R2_triggered = 0;
static uint8_t R2_state = 0;
static uint32_t R2_start_time = 0;

static uint8_t R1_triggered = 0;
static uint8_t R1_state = 0;
static uint32_t R1_start_time = 0;

static uint8_t R1_triggered2 = 0;
static uint8_t R1_state2 = 0;
static uint32_t R1_start_time2 = 0;

uint16_t current_torque = 0;
uint16_t current_rpm = 0;

uint8_t count = 0;
uint16_t comm_timeout = 0;

int16_t pwm_value1=0;
int16_t pwm_value2=0;
uint32_t pTxMailbox;
uint32_t pTxMailbox2;
uint32_t pTxMailbox3;
uint8_t Tx_Data[4];
uint8_t rCount[8];

CAN_TxHeaderTypeDef pTxHeader;
CAN_TxHeaderTypeDef pTxHeader2;
CAN_RxHeaderTypeDef pRxHeader;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

// it is for adc time test
/*
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
}
*/
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &pRxHeader, received_data);
    lastCanTick = HAL_GetTick();

    if (pRxHeader.StdId == 0x100)
    {
        m100 = 1;
        for (int i = 0; i < 8; i++)
            a1[i] = received_data[i];
        // 16-bit alanları LSB-first olarak birleştir
        M100_flags_t.speed_value = ((uint16_t)a1[1] << 8) | (uint16_t)a1[0];
        M100_flags_t.direction_value = ((uint16_t)a1[3] << 8) | (uint16_t)a1[2];
        M100_flags_t.auto_speed_value = a1[4];

        uint8_t flags = a1[5];

        M100_flags_t.brake_status = (flags >> 0) & 0x01;
        M100_flags_t.auto_manual_mode = (flags >> 1) & 0x01;
        M100_flags_t.headlight_state = (flags >> 3) & 0x01;
        M100_flags_t.connection_status = (flags >> 4) & 0x01;

        M100_flags_t.reverse_direction = a1[6];
        M100_flags_t.control_mode = a1[7];

        i_speed_value = M100_flags_t.speed_value;
        i_direction_value = M100_flags_t.direction_value;
        /*printf("Speed=%u Dir=%d AutoSpd=%u Brake=%u RevDir=%X CtrlMode=%X\n",
               M100_flags_t.speed_value,
			   i_direction_value,
               M100_flags_t.auto_speed_value,
               M100_flags_t.brake_status,
               M100_flags_t.reverse_direction,
               M100_flags_t.control_mode);
        HAL_CAN_AddTxMessage(&hcan2, &pTxHeader2, TxData, &TxMailbox2);
        for (uint16_t f = 0; f < 10000; f++) {
        	__asm("NOP");
       	}*/
    }
    else if (pRxHeader.StdId == 0x101)
    {
        m101 = 1;
        M101_data_t.mine_emergency = (received_data[0] >> 0) & 0x01;   //  1. bit
        M101_data_t.remote_emergency = (received_data[0] >> 1) & 0x01; //  2. bit
        M101_data_t.barrier_emergency = (received_data[0] >> 2) & 0x01;   //  3. bit
    }
    else if (pRxHeader.StdId == 0x102)
    {
        m102 = 1;
    }
    else if (pRxHeader.StdId == 0x103)
    {
        for (int i = 0; i < 8; i++)
        {
            a3[i] = received_data[i];
        }
        M103_data_t.pivot_value = ((uint8_t)a3[1] << 8) // MSB’yi üst 8 bita koy
                                  | (uint8_t)a3[0];     // LSB’yi alt 8 bita koy

        i_pivot_value = M103_data_t.pivot_value;

        m103 = 1;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM3)  // TIM2 kesmesi geldiyse
	{
		/*count++;
		if (count > 100) {
			count = 0;
			printf("sending %lu\n", lastCanTick);
		}*/
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	/*if (comm_timeout >= 500) {
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); // Örnek LED yak
		} else {
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); // Normalde kapalı
		}*/
	}
	if (htim->Instance == TIM2)  // TIM3 kesmesi geldiyse
	{
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
		//printf("sending toggle 2");
	}
	if (htim->Instance == TIM4)  // TIM3 kesmesi geldiyse
	{
		TxData[0] = 0x05;
		TxData[1] = 0;
	    TxData[2] = M100_flags_t.reverse_direction;
		TxData[3] = M100_flags_t.control_mode;
		TX_Ser_Val[2] = M100_flags_t.speed_value;
		TX_Ser_Val[3] = M100_flags_t.speed_value;
	    /*TxData[0] = 0x05;
	    TxData[2] = 0x00;
	    TxData[3] = 0xC0;
	    TxData[4] = 0x08;
	    TxData[5] = 0x07;
	    TxData[6] = 0x08;
	    TxData[7] = 0x07;*/
		HAL_GPIO_TogglePin(Status_Led_GPIO_Port, Status_Led_Pin);
		HAL_CAN_AddTxMessage(&hcan2, &pTxHeader2, TxData, &TxMailbox2);
		//printf("sending can message");
	}
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
  MX_CAN1_Init();
  MX_DAC_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_CAN2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);
  TX_Ser_Val = (int16_t *)&TxData[0];

    pTxHeader2.ExtId = 0x10F83807;
    pTxHeader2.IDE = CAN_ID_EXT;
    pTxHeader2.RTR = CAN_RTR_DATA;
    pTxHeader2.DLC = 8;
    pTxHeader2.TransmitGlobalTime = DISABLE;

    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_TX_MAILBOX_EMPTY);

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_CAN_Start(&hcan2) != HAL_OK)
    {
        Error_Handler();
    }
    uint8_t spPage = 0x00;

    if (HAL_CAN_AddTxMessage(&hcan1, &pTxHeader, &spPage, &pTxMailbox) != HAL_OK)
    {
        Error_Handler();
    }
    uint8_t spPager = 0x00;
    if (HAL_CAN_AddTxMessage(&hcan2, &pTxHeader, &spPager, &pTxMailbox2) != HAL_OK)
    {
        Error_Handler();
    }

    TxData[0] = 0x05;
    TxData[2] = 0x00;
    TxData[3] = 0xC0;
    TxData[4] = 0x08;
    TxData[5] = 0x07;
    TxData[6] = 0x08;
    TxData[7] = 0x07;
  /* USER CODE END 2 */

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
  RCC_OscInitStruct.PLL.PLLN = 160;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 10;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_8TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_7TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

    CAN_FilterTypeDef can_filtre_yapi;

    can_filtre_yapi.FilterActivation = CAN_FILTER_ENABLE;
    can_filtre_yapi.FilterBank = 13;
    can_filtre_yapi.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    can_filtre_yapi.FilterIdHigh = 0;
    can_filtre_yapi.FilterIdLow = 0;
    can_filtre_yapi.FilterMaskIdHigh = 0x0000;
    can_filtre_yapi.FilterMaskIdLow = 0x0000;
    can_filtre_yapi.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filtre_yapi.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filtre_yapi.SlaveStartFilterBank = 0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filtre_yapi);

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 10;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_8TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_7TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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
  htim2.Init.Prescaler = 160*20-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  htim3.Init.Prescaler = 160*20-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 160*20-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Break_On_Pin|L_EN_Pin|R_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, left_direction_Pin|right_direction_Pin|IN_3_2_Pin|IN_3_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, Break_Off_Pin|R1_Pin|R2_Pin|R3_Pin
                          |R4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IN_1_1_GPIO_Port, IN_1_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, IN_1_2_Pin|IN_2_1_Pin|LED1_Pin|Status_Led_Pin
                          |LED2_Pin|IN_2_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Break_On_Pin L_EN_Pin R_EN_Pin */
  GPIO_InitStruct.Pin = Break_On_Pin|L_EN_Pin|R_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : left_direction_Pin right_direction_Pin IN_3_2_Pin IN_3_1_Pin */
  GPIO_InitStruct.Pin = left_direction_Pin|right_direction_Pin|IN_3_2_Pin|IN_3_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Break_Off_Pin R1_Pin R2_Pin R3_Pin
                           R4_Pin */
  GPIO_InitStruct.Pin = Break_Off_Pin|R1_Pin|R2_Pin|R3_Pin
                          |R4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : IN_1_1_Pin */
  GPIO_InitStruct.Pin = IN_1_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IN_1_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IN_1_2_Pin IN_2_1_Pin LED1_Pin Status_Led_Pin
                           LED2_Pin IN_2_2_Pin */
  GPIO_InitStruct.Pin = IN_1_2_Pin|IN_2_1_Pin|LED1_Pin|Status_Led_Pin
                          |LED2_Pin|IN_2_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
