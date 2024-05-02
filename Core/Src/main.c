/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void move(int speed) {
  if (speed>0) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
    TIM1->CCR4 = speed;
    TIM1->CCR1 = speed;
  } else {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
    TIM1->CCR4 = -speed;
    TIM1->CCR1 = -speed;
  }
}

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t uartdata[8];
int last_tick=0;
// uint16_t last_yaw_angle = 0;
// uint16_t last_roll_angle = 0;
// uint16_t yaw_angle = 0;
// uint16_t roll_angle = 0;
int pitch_angle = 0;
int pitch_corrected = 0;
int pitch_p_diff = 0;
int pitch_offset = 0;
int time_diff = 0;
// int pitch_last_p=0;
// int pitch_p=0;
// int pitch_i=0;
// int pitch_d=0;
// int left_i = 0;
// int left_d = 0;
// int right_i = 0;
// int right_d = 0;
struct motor {
  int encoder_val;
  int last_speed;
  int current_speed;
  int encoder_last_val;
};
struct motor motor_left;
struct motor motor_right;
struct pid {
  int last_p;
  int p;
  float kp;
  int i;
  float ki;
  int d;
  float kd;
};
struct pid pitch = {.last_p = 0, .i = 0, .kp = 0.13, .ki = 0.0, .kd = 275};
struct pid motor_speed =  {.last_p = 0, .i = 0, .kp = 50, .ki = 0.008, .kd = 15};
int velo = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  HAL_TIM_Base_Start_IT(&htim4);
  HAL_Delay(3000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      //Below is the pid
      // move_l(1000);
      // move_r(1000);
      while (pitch_offset == 0)
      {
          HAL_UART_Receive_IT(&huart2, uartdata, 8);
          pitch_offset = pitch_angle;
      }
      while (1) {
        //encode start
        motor_left.encoder_val = __HAL_TIM_GET_COUNTER(&htim2);
        motor_left.current_speed = motor_left.encoder_val - motor_left.encoder_last_val;
        motor_left.encoder_last_val = motor_left.encoder_val;
        motor_right.encoder_val = __HAL_TIM_GET_COUNTER(&htim3);
        motor_right.current_speed = motor_right.encoder_val - motor_right.encoder_last_val;
        motor_right.encoder_last_val = motor_right.encoder_val;
        //encoder end
        HAL_UART_Receive_IT(&huart2, uartdata, 8);
        pitch_corrected = pitch_angle-pitch_offset;
        if (pitch_corrected > (33000)) {pitch_corrected -= 66000;}
        pitch.p = pitch_corrected;
        pitch_p_diff = pitch.p - pitch.last_p;
        time_diff = HAL_GetTick() - last_tick;
        pitch.d = (pitch.p - pitch.last_p)/*/((HAL_GetTick() - last_tick)*0.001)*/;
        if (pitch.i+pitch.p<=5000 && pitch.i+pitch.p>=-5000) {
          pitch.i += pitch.p*0.05;
        } else if (pitch.i+pitch.p>0) {pitch.i = 5000;}
        else {pitch.i = -5000;}
        motor_speed.p = -(motor_left.current_speed + motor_right.current_speed)/2;
        if (motor_speed.i+motor_speed.p<=5000 && motor_speed.i+motor_speed.p>=-5000) {
          motor_speed.i += motor_speed.p;
        } else if (motor_speed.i+motor_speed.p>0) {motor_speed.i = 5000;}
        else {motor_speed.i = -5000;}
        motor_speed.d = (motor_speed.p - motor_speed.last_p)/((HAL_GetTick()-last_tick)*0.001);
        velo = 0.5*(4*(pitch.kp * pitch.p + pitch.ki * pitch.i + pitch.kd * pitch.d) + 1.5* (motor_speed.kp * motor_speed.p + motor_speed.ki * motor_speed.i + motor_speed.kd * motor_speed.d));
        move(velo);
        pitch.last_p = pitch.p;
        motor_speed.last_p = motor_speed.p;
        last_tick = HAL_GetTick();
        HAL_Delay(5);
      }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
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
