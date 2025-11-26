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
#include "Motor_control.h"
#include "math.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_BUFFER_SIZE 100

uint8_t uart_rx_data;                   // biến nhận từng byte UART
char uart_cmd_buffer[UART_BUFFER_SIZE]; // buffer lưu chuỗi nhận được
uint16_t uart_cmd_index = 0;            // chỉ số lưu vị trí hiện tại trong buffer
volatile uint8_t uart_data_ready = 0;   // cờ báo nhận xong 1 chuỗi dữ liệu
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
float toc_do;
uint8_t uart_flag_tx;
volatile uint32_t pulse_count = 0;
uint8_t flag_pulse;
uint8_t flag_timer_100ms;
uint8_t motor_on = 0;
uint8_t controller_type = 0;
uint8_t input_type = 0;
float amp_max = 0, amp_min = 0;
float duty;
typedef enum {
    STATE_IDLE,     // Động cơ đang nghỉ hoặc chờ
    STATE_RUNNING,  // Động cơ đang chạy ổn định
    STATE_BRAKING   // Động cơ đang trong quá trình giảm tốc về 0
} MotorState_t;

MotorState_t current_state = STATE_IDLE;
float target_set_point = 0; // Biến lưu giá trị mong muốn (nhận từ UART)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* --- CÁC HÀM CHỨC NĂNG (ĐƯỢC TÁCH RA CHO GỌN) --- */
void Process_Encoder_Event(void);
void Process_Speed_Calculation(void);
void Process_UART_Reception(void);
void Update_Setpoint_Logic(void);
void Run_Control_Algorithm(void);
void Send_Telemetry_Data(void);
void Stop_Motor(void);

/* Các hàm tiện ích  */
void send_uart_3float(UART_HandleTypeDef *huart, float var1, float var2, float var3);
void Parse_UART_Command(char* str);
float calculate_motor_speed_rpm(void);
uint16_t percent_to_duty(double input_percent);
float scale_to_0_1(float x, float x_min, float x_max);
float scale_to_50_250(float x);
void Process_Motor_State_Machine(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile float encoder_count = 0; // Dem xung

// --- CÁC HÀM XỬ LÝ LOGIC CHÍNH ---

// 1. Xử lý sự kiện đếm xung (gọi trong vòng lặp chính)
void Process_Encoder_Event(void) {
    if (flag_pulse == 1) {
        flag_pulse = 0;
        pulse_count++;
    }
}

// 2. Tính toán tốc độ mỗi 100ms
void Process_Speed_Calculation(void) {
    if (flag_timer_100ms == 1) {
        encoder_count = pulse_count;
        toc_do = calculate_motor_speed_rpm();
        pulse_count = 0;
        flag_timer_100ms = 0;
    }
}

// 3. Xử lý lệnh UART khi nhận đủ chuỗi
void Process_UART_Reception(void) {
    if (uart_data_ready) {
        Parse_UART_Command(uart_cmd_buffer);
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // Đèn báo
        uart_data_ready = 0;
    }
}

// 4. Cập nhật giá trị đặt (Setpoint) và Chiều quay
void Update_Setpoint_Logic(void) {
    
    // --- PART 1: Determine Input Source ---
    if (input_type == 0) { 
        // Keep UART set_point
    } 
    else if (input_type == 1) {
        set_point = set_point_sine; 
    } 
//    else if (input_type == 2) {
//        static uint16_t pulse_counter = 0;
//        if (pulse_counter < 15000) set_point = amp_max;
//        else set_point = amp_min;
//        pulse_counter++;
//        if (pulse_counter >= 30000) pulse_counter = 0;
//    }
		else if (input_type == 2) {
        // Mode Pulse (Tạo xung vuông)
        // --- DÙNG THỜI GIAN THAY VÌ BIẾN ĐẾM ---
        
        // Lấy thời gian hiện tại của hệ thống (tính bằng ms)
        uint32_t current_time = HAL_GetTick(); 
        
        // Chu kỳ xung: 10000ms (tức là 4 giây: 2s Âm, 2s Dương)
        uint32_t period = 10000; 
        
        // Chia lấy dư để tạo chu kỳ lặp lại
        if ((current_time % period) < (period / 2)) {
            set_point = amp_max; // 2 giây đầu: Chạy Max (ví dụ 100)
        } else {
            set_point = amp_min; // 2 giây sau: Chạy Min (ví dụ -100)
        }
    }
    
    if (set_point >= 0) {
        // Chạy Chiều Thuận (Forward)
        // Ví dụ: IN1 = 1, IN2 = 0
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);   
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    } 
    else {
			
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); 
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
				}
}

// 5. Chạy thuật toán điều khiển và xuất PWM
void Run_Control_Algorithm(void) {
    current_speed = toc_do;
    // Gọi hàm tính toán
    Motor_control_step(); 
    
		// Output PWM
		
		if (controller_type == 0) 
				duty = signal_control;
		else 
				duty = signal_controlPImras;
		htim2.Instance->CCR1 = percent_to_duty(duty);

}

// 6. Gửi dữ liệu lên máy tính
void Send_Telemetry_Data(void) {
    if (uart_flag_tx) {
        if (controller_type == 0) {
            send_uart_3float(&huart1, set_point, current_speed, signal_control);
        }
        uart_flag_tx = 0;
    }
}

// 7. Dừng động cơ an toàn
void Stop_Motor(void) {
    htim2.Instance->CCR1 = 0;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
}

/* --- CÁC HÀM TIỆN ÍCH --- */

float calculate_motor_speed_rpm() {
    const float pulses_per_encoder_rev = 11.0f;
    const float gear_ratio = 21.3f;
    const float sampling_time_sec = 0.1;
    float encoder_rps = encoder_count / pulses_per_encoder_rev / sampling_time_sec;
    float motor_rpm = encoder_rps * 60.0f / gear_ratio;
    encoder_count = 0;
    return motor_rpm;
}

uint16_t percent_to_duty(double input_percent) {
    uint16_t min_duty = 0;
    uint16_t max_duty = 1000;
    double scaled_duty = min_duty + ((double)input_percent * (max_duty - min_duty) / 100.0);
    return (uint16_t)round(scaled_duty);
}

float scale_to_0_1(float x, float x_min, float x_max)
{
    if (x < x_min) x = x_min;   // not negative
    if (x > x_max) x = x_max;
    return (x - x_min) / (x_max - x_min);
}


float scale_to_50_250(float x) {
    if (x < 0.0f) x = 0.0f;
    if (x > 1.0f) x = 1.0f;
    return x * (250.0f - 50.0f) + 50.0f;
}

void send_uart_3float(UART_HandleTypeDef *huart, float var1, float var2, float var3) {
    char uart_buffer[100];
    int len = sprintf(uart_buffer, "%.2f,%.2f,%.2f\r\n", var1, var2, var3);
    HAL_UART_Transmit(huart, (uint8_t*)uart_buffer, len, HAL_MAX_DELAY);
}

void Parse_UART_Command(char* str) {
    char* token;
    uint8_t field = 0;
    token = strtok(str, ",");
    while (token != NULL) {
        switch (field) {
            case 0: motor_on = atoi(token); break;
            case 1: controller_type = atoi(token); break;
            case 2: input_type = atoi(token); break;
            case 3: kp = atof(token); break;
            case 4: ki = atof(token); break;
            case 5: gp = atof(token); break;
            case 6: gi = atof(token); break;
            case 7: amp_max = atof(token); break;
            case 8: amp_min = atof(token); break;
            case 9: target_set_point = atof(token); break;
            default: break;
        }
        token = strtok(NULL, ",");
        field++;
    }
}

/* --- CALLBACK INTERRUPTS --- */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_10) flag_pulse = 1;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1) flag_timer_100ms = 1;
    if (htim->Instance == TIM3) uart_flag_tx = 1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        if (uart_rx_data == 'R') {
            uart_cmd_buffer[uart_cmd_index] = '\0';
            uart_data_ready = 1;
            uart_cmd_index = 0;
        } else {
            if (uart_cmd_index < UART_BUFFER_SIZE - 1) {
                uart_cmd_buffer[uart_cmd_index++] = uart_rx_data;
            } else {
                uart_cmd_index = 0;
            }
        }
        HAL_UART_Receive_IT(huart, &uart_rx_data, 1);
    }
}
void Process_Motor_State_Machine(void) {
    // Ngưỡng coi như đã dừng hẳn (ví dụ dưới 5 RPM)
    float stop_threshold = 5.0f;
		if (input_type != 0) {
        current_state = STATE_RUNNING;
        return; 
    }

    switch (current_state) {
        
        // --- TRẠNG THÁI 1: ĐANG CHẠY ---
        case STATE_RUNNING:
            // Logic: Kiểm tra xem người dùng có muốn đảo chiều không?
            // Nếu Target và Set_point khác dấu (tức là đảo chiều), chuyển sang BRAKING
            if ((target_set_point > 0 && set_point < 0) || 
                (target_set_point < 0 && set_point > 0)) {
                
                current_state = STATE_BRAKING;
            }
            // Nếu người dùng muốn dừng hẳn (target = 0)
            else if (target_set_point == 0) {
                current_state = STATE_BRAKING;
            }
            else {
                // Nếu cùng chiều hoặc tăng giảm tốc bình thường, cập nhật set_point
                set_point = target_set_point; 
            }
            break;

        // --- TRẠNG THÁI 2: ĐANG HÃM (STOPPING) ---
        case STATE_BRAKING:
            // 1. Đặt set_point về 0 để PID hãm động cơ lại
            set_point = 0;
            
            // 2. Kiểm tra xem động cơ đã dừng hẳn chưa?
            // (Lấy trị tuyệt đối tốc độ hiện tại)
            if (fabs(toc_do) < stop_threshold) {
                // Đã dừng hẳn!
                // Reset các bộ tích phân PID (để tránh vọt lố khi khởi động lại)
                // pid_reset(); // <--- Nếu thư viện PID của bạn có hàm này
                
                // Chuyển sang chờ một chút hoặc chạy luôn chiều mới
                current_state = STATE_IDLE; 
            }
            break;

        // --- TRẠNG THÁI 3: CHỜ (IDLE) ---
        case STATE_IDLE:
            // Tại đây động cơ đang đứng yên.
            // Nếu có lệnh mới khác 0, bắt đầu chuyển sang chạy
            if (target_set_point != 0) {
                // Gán chiều mới
                set_point = target_set_point;
                current_state = STATE_RUNNING;
            }
            break;
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  // --- KHỞI TẠO HỆ THỐNG ---
  Motor_control_initialize();
  HAL_TIM_Base_Start_IT(&htim1);            // Timer 1 (Tốc độ)
  HAL_TIM_Base_Start_IT(&htim3);            // Timer 3 (UART)
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Timer 2 (PWM)
  
  // Bắt đầu nhận UART
  HAL_UART_Receive_IT(&huart1, (uint8_t*)&uart_rx_data, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		Process_Encoder_Event();
    Process_Speed_Calculation();
    Process_UART_Reception();

    if (motor_on == 1) 
    {
        // 1. Xử lý logic trạng thái (Quyết định Stop hay Run)
        Process_Motor_State_Machine();

        // 2. Cập nhật phần cứng dựa trên set_point (đã được máy trạng thái xử lý)
        // Lưu ý: Update_Setpoint_Logic lúc này chỉ nhiệm vụ set GPIO chiều quay
        Update_Setpoint_Logic(); 
        
        // 3. Chạy PID
        Run_Control_Algorithm();
        
        Send_Telemetry_Data();
    }
    else 
    {
        Stop_Motor();
        current_state = STATE_IDLE; // Reset trạng thái khi tắt motor
        set_point = 0;
        target_set_point = 0;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 799;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim3.Init.Prescaler = 799;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2499;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
