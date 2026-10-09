/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_PWM 1000
#define ADC_COMPA 2000
#define SPEED 500
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
// Thêm từ khóa volatile cho các biến dùng chung giữa ngắt (EXTI/DMA) và vòng lặp chính
// để tránh trình biên dịch tối ưu hóa lưu biến vào thanh ghi CPU gây mất phản hồi
volatile uint8_t is_avoiding = 0;
volatile uint16_t adc[8];

float error = 0;
float last_error = 0;
float I = 0;
float Kp = 35.0, Ki = 0.0, Kd = 15.0;
float PID_value = 0;

void delay_us(uint16_t us) {								// Hàm delay theo us để gửi xung cho cảm biến siêu âm
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    while (__HAL_TIM_GET_COUNTER(&htim2) < us);
}

void pulse(GPIO_TypeDef *Port, uint16_t pin, uint8_t t){    // Tạo xung kích hoạt (Trigger) cho cảm biến siêu âm
	HAL_GPIO_WritePin(Port , pin, GPIO_PIN_RESET);
	delay_us(t/10);
	HAL_GPIO_WritePin(Port, pin, GPIO_PIN_SET);
	delay_us(t);
	HAL_GPIO_WritePin(Port, pin, GPIO_PIN_RESET);
}

void Set_Motor_Speed(int16_t speed_left, int16_t speed_right) {
	    uint16_t pwm_left = 0;
	    uint16_t pwm_right = 0;

	    // Giới hạn giá trị PWM trong dải [-MAX_PWM, MAX_PWM]
	    if (speed_left > MAX_PWM) speed_left = MAX_PWM;
	    else if (speed_left < -MAX_PWM) speed_left = -MAX_PWM;
	    if (speed_right > MAX_PWM) speed_right = MAX_PWM;
	    else if (speed_right < -MAX_PWM) speed_right = -MAX_PWM;

	    // Điều khiển chiều quay và độ rộng xung động cơ trái (PB12, PB13, TIM1_CH1)
	    if (speed_left >= 0) {
	        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	        pwm_left = speed_left;
	    } else {
	        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
	        pwm_left = -speed_left;
	    }

	    // Điều khiển chiều quay và độ rộng xung động cơ phải (PB14, PB15, TIM1_CH2)
	    if (speed_right >= 0) {
	        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	        pwm_right = speed_right;
	    } else {
	        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
	        pwm_right = -speed_right;
	    }
	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_left);
	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_right);
}

void pid_calculate(){
	uint8_t weight[8] = {0, 1, 2, 3, 4, 5, 6, 7};
	float setpoint = 3.5;
	uint32_t sum_value = 0;
	uint32_t weighted_sum = 0;
	float position = 0;

	for (int i = 0; i < 8; i++) {
	    if (adc[i] > ADC_COMPA) {
	        weighted_sum += weight[i] * adc[i];
	        sum_value += adc[i];
	    }
	}

	if (sum_value > 0) {
	    // SỬA: Ép kiểu float cho cả tử và mẫu để phép chia giữ lại phần thập phân,
	    // giúp tính toán vị trí vạch line mịn màng thay vì bị cắt thành số nguyên
	    position = (float)weighted_sum / (float)sum_value;
	    error = setpoint - position;
	    // SỬA: ĐÃ XÓA dòng "last_error = error;" ở đây!
	    // Trước đó dòng này làm cho (error - last_error) luôn bằng 0, triệt tiêu khâu vi phân D.
	} else {
	    // Khi mất vạch line hoàn toàn, nhớ hướng lệch trước đó để tiếp tục bẻ lái gắt tìm lại line
	    if (last_error > 0) {
	        error = 4.0;
	    } else {
	        error = -4.0;
	    }
	}

	float P = error;
	// SỬA: Bây giờ last_error vẫn giữ giá trị của chu kỳ trước, khâu D hoạt động chuẩn xác để chống lắc
	float D = error - last_error;
	I += error;

	// Giới hạn chống bão hòa tích phân (Anti-windup)
	if (I > 1000.0f) I = 1000.0f;
	else if (I < -1000.0f) I = -1000.0f;

	PID_value = (Kp * P) + (Ki * I) + (Kd * D);
	last_error = error; // Cập nhật last_error ở cuối hàm cho chu kỳ tiếp theo
}
void avoid(void) {
    static uint8_t state = 0;
    static uint32_t tick = 0;

    switch (state) {
        case 0:
            Set_Motor_Speed(-400, -400);
            tick = HAL_GetTick();
            state = 1;
            break;
        case 1:
            if (HAL_GetTick() - tick >= 300) {
                Set_Motor_Speed(500, -500);
                tick = HAL_GetTick();
                state = 2;
            }
            break;
        case 2:
            if (HAL_GetTick() - tick >= 400) {
                Set_Motor_Speed(400, 400);
                tick = HAL_GetTick();
                state = 3;
            }
            break;
        case 3:
            if (HAL_GetTick() - tick >= 600) {
                Set_Motor_Speed(-500, 500);
                tick = HAL_GetTick();
                state = 4;
            }
            break;
        case 4:
            if (HAL_GetTick() - tick >= 400) {
                Set_Motor_Speed(400, 400);
                tick = HAL_GetTick();
                state = 5;
            }
            break;
        case 5:
            if (HAL_GetTick() - tick >= 800) {
                Set_Motor_Speed(-500, 500);
                tick = HAL_GetTick();
                state = 6;
            }
            break;
        case 6:
            if (HAL_GetTick() - tick >= 350) {
                Set_Motor_Speed(350, 350);
                tick = HAL_GetTick(); // Lưu tick bắt đầu để đếm thời gian cho state 7
                state = 7;
            }
            break;
        case 7:
            // SỬA: Dùng hằng số ADC_COMPA đồng nhất; bổ sung timeout 2000ms để dừng xe nếu xe chạy trượt khỏi vạch line
            if (adc[3] > ADC_COMPA || adc[4] > ADC_COMPA) {
                Set_Motor_Speed(500, -500);
                tick = HAL_GetTick();
                state = 8;
            } else if (HAL_GetTick() - tick >= 2000) {
                // Quá 2 giây không tìm thấy line, dừng xe an toàn
                Set_Motor_Speed(0, 0);
                is_avoiding = 0;
                state = 0;
            }
            break;
        case 8:
            if (HAL_GetTick() - tick >= 150) {
                Set_Motor_Speed(0, 0);
                tick = HAL_GetTick();
                state = 9;
            }
            break;
        case 9:
            if (HAL_GetTick() - tick >= 100) {
                state = 0;
                is_avoiding = 0;
            }
            break;
    }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin == GPIO_PIN_11) {
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_SET) {
			__HAL_TIM_SET_COUNTER(&htim2, 0); // Sườn lên: Reset bộ đếm Timer 2 để bắt đầu đo thời gian
		}
		else {
	        uint32_t time_val = __HAL_TIM_GET_COUNTER(&htim2); // Sườn xuống: Lấy độ rộng xung Echo (micro-giây)
	        // Vận tốc âm thanh: 340 m/s = 0.034 cm/us. Khoảng cách (cm) = time * 0.034 / 2
	        float distance = (float)time_val * 0.034f / 2.0f;

	        // Lọc nhiễu khoảng cách ảo (bỏ qua giá trị <= 2cm do phản xạ gần) và phát hiện vật cản < 15cm
	        if (distance > 2.0f && distance < 15.0f) {
	        	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	        	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);

	            is_avoiding = 1;
	        }
	    }
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  // SỬA: Hiệu chuẩn phần cứng ADC trước khi kích hoạt DMA để loại bỏ sai số offset trên STM32F1
  HAL_ADCEx_Calibration_Start(&hadc1);

  // SỬA: Khởi động ADC đọc liên tục 8 kênh qua DMA vào mảng adc[8]
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc, 8);

  uint32_t pid_loop_tick = 0;
  uint32_t ultrasonic_tick = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (is_avoiding == 1) {
		  avoid();
	  }
	  else {
		  // SỬA: Tách chu kỳ phát xung siêu âm riêng (mỗi 60ms) theo chuẩn cảm biến HC-SR04,
		  // tránh phát dồn dập 20ms làm nhận nhầm sóng phản xạ của lần phát trước
		  if (HAL_GetTick() - ultrasonic_tick >= 60) {
		      ultrasonic_tick = HAL_GetTick();
		      pulse(GPIOB, GPIO_PIN_10, 10);
		  }

		  // Chu kỳ tính toán và điều khiển động cơ PID chạy ổn định mỗi 20ms
		  if (HAL_GetTick() - pid_loop_tick >= 20) {
		      pid_loop_tick = HAL_GetTick();
		      pid_calculate();
		      // Cập nhật tốc độ 2 bánh xe
		      // Lưu ý: Nếu thực tế xe lệch trái nhưng lại rẽ trái (ngược hướng), chỉ cần đổi dấu:
		      // Set_Motor_Speed(SPEED - PID_value, SPEED + PID_value);
		      Set_Motor_Speed(SPEED + PID_value, SPEED - PID_value);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  // SỬA: Đổi từ RCC_HSE_OFF sang RCC_HSE_ON. Nếu HSE tắt mà PLL dùng nguồn HSE,
  // HAL_RCC_OscConfig sẽ trả về HAL_ERROR và làm chip treo vĩnh viễn trong Error_Handler()
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 8;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  // SỬA: Cấu hình tuần tự 8 mắt đọc cảm biến từ PA0 (ADC_CHANNEL_0) đến PA7 (ADC_CHANNEL_7)
  // và tăng SamplingTime lên 55.5 chu kỳ để chống nhiễu xuyên kênh (crosstalk) giữa các mắt đọc

  /** Configure Regular Channel - Rank 1: PA0 (Sensor 0)
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel - Rank 2: PA1 (Sensor 1)
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel - Rank 3: PA2 (Sensor 2)
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel - Rank 4: PA3 (Sensor 3)
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel - Rank 5: PA4 (Sensor 4)
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel - Rank 6: PA5 (Sensor 5)
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel - Rank 7: PA6 (Sensor 6)
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel - Rank 8: PA7 (Sensor 7)
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_8;
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
  // SỬA: Đổi Prescaler = 71 và Period = 999 để tạo tần số PWM 1 kHz (72MHz / (71+1) / (999+1) = 1kHz)
  // Khớp dải điều khiển MAX_PWM = 1000 (100% duty cycle) và SPEED = 500 (50% duty cycle)
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB10 PB12 PB13 PB14
                           PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
