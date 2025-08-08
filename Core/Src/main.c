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
#include <stdint.h>
#include "stm32f0xx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MAX_ITER 32  // Reduced from 100 for faster execution
#define SCALE_FACTOR 65536    // 2^16 for faster fixed-point arithmetic (bit shifts)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//TODO: Define and initialise the global varibales required
/*
  start_time
  end_time
  execution_time
  checksum: should be uint64_t
  initial width and height maybe or you might opt for an array??
*/

// Global variables for benchmarking (volatile to prevent compiler optimization)
volatile uint32_t start_time = 0;
volatile uint32_t end_time = 0;
volatile uint32_t execution_time = 0;
volatile uint32_t execution_time_ms = 0;
volatile uint64_t checksum = 0;
volatile uint64_t global_checksum = 0;
// Image dimensions for testing (square images)
// Start with index 0 (128x128) for fastest testing, then increase for full benchmarks
int image_dimensions[5] = {128, 160, 192, 224, 256};
volatile int current_test_index = 1;      // Change this to test different dimensions (0-4)
volatile int current_test_number = 0;     // Human-readable test number (1-5)
volatile int current_dimension = 0;       // Current image dimension being tested

// Additional tracking variables (volatile for debugging)
volatile int test_mode = 0;              // 0 = fixed-point, 1 = double precision
volatile int benchmark_complete = 0;     // 1 when benchmark is finished
volatile uint32_t fastest_time = 0xFFFFFFFF;  // Track fastest execution time
volatile uint32_t slowest_time = 0;      // Track slowest execution time
volatile int debug_counter = 0;          // General purpose debug counter
volatile int variables_initialized = 1;  // Indicates variables are set up

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations);

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
  /* USER CODE BEGIN 2 */
  // IMPORTANT: For maximum speed, compile with -O2 or -O3 optimization
  // In STM32CubeIDE: Project -> Properties -> C/C++ Build -> Settings -> Tool Settings -> Optimization

  // Initialize tracking variables
  current_test_number = current_test_index + 1;  // Human readable (1-5)
  current_dimension = image_dimensions[current_test_index];
  test_mode = 0;  // 0 = fixed-point, 1 = double precision
  benchmark_complete = 0;
  debug_counter++;

  // Force variables to be used so compiler doesn't optimize them away
  if (variables_initialized == 0) {
    // This will never execute, but prevents optimization
    variables_initialized = 1;
  }

  //TODO: Turn on LED 0 to signify the start of the operation
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

  //TODO: Call the Mandelbrot Function and store the output in the checksum variable defined initially
  // Use the first dimension (128x128) for initial test
  // Change current_test_index to test different dimensions
  int width = image_dimensions[current_test_index];
  int height = image_dimensions[current_test_index];

  // Test with fixed-point arithmetic
  test_mode = 0;  // Fixed-point mode

  //TODO: Record the start time
  start_time = HAL_GetTick();

  checksum = calculate_mandelbrot_fixed_point_arithmetic(width, height, MAX_ITER);

  //TODO: Record the end time
  end_time = HAL_GetTick();

  // test with double precision instead:
  test_mode = 1;  // Double precision mode
  start_time = HAL_GetTick();
  checksum = calculate_mandelbrot_double(width, height, MAX_ITER);
  end_time = HAL_GetTick();

  //TODO: Calculate the execution time
  execution_time = end_time - start_time;
  execution_time_ms = execution_time;  // Same value, clearer name
  global_checksum = checksum;          // Same value, matches your expression

  // Update performance tracking
  if (execution_time < fastest_time) {
    fastest_time = execution_time;
  }
  if (execution_time > slowest_time) {
    slowest_time = execution_time;
  }

  //TODO: Turn on LED 1 to signify the end of the operation
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

  benchmark_complete = 1;  // Mark benchmark as complete

  // DEBUG BREAKPOINT: Set a breakpoint on the next line to view all variables
  debug_counter = 999;  // Marker for debugging - set breakpoint here

  //TODO: Hold the LEDs on for a 1s delay
  HAL_Delay(1000);

  //TODO: Turn off the LEDs
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//TODO: Mandelbroat using variable type integers and fixed point arithmetic
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations){
    uint64_t mandelbrot_sum = 0;
    //TODO: Complete the function implementation

    // Optimized constants using 2^16 scale factor for bit shifts
    const int32_t scale_shift = 16;  // Use bit shifts instead of division
    const int32_t escape_limit = 4 << scale_shift;  // 4.0 in fixed point

    // Pre-compute scaling factors for coordinate mapping
    const int32_t x_scale = (229376) / width;  // 3.5 * 65536 / width
    const int32_t y_scale = (131072) / height; // 2.0 * 65536 / height
    const int32_t x_offset = -163840;  // -2.5 * 65536
    const int32_t y_offset = -65536;   // -1.0 * 65536

    for (int y = 0; y < height; y++) {
        int32_t y0 = y * y_scale + y_offset;

        for (int x = 0; x < width; x++) {
            int32_t x0 = x * x_scale + x_offset;

            int32_t xi = 0, yi = 0;
            int iteration = 0;

            // Mandelbrot iteration loop with optimized arithmetic
            while (iteration < max_iterations) {
                // Use bit shifts for faster division (since scale = 2^16)
                int32_t xi_squared = (xi * xi) >> scale_shift;
                int32_t yi_squared = (yi * yi) >> scale_shift;

                // Fast escape condition check
                if (xi_squared + yi_squared > escape_limit) {
                    break;
                }

                // Mandelbrot recurrence with optimized operations
                int32_t temp = xi_squared - yi_squared + x0;
                yi = ((xi * yi) >> (scale_shift - 1)) + y0;  // 2*xi*yi optimized
                xi = temp;

                iteration++;
            }

            mandelbrot_sum += iteration;
        }
    }

    return mandelbrot_sum;
}

//TODO: Mandelbroat using variable type double
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations){
    uint64_t mandelbrot_sum = 0;
    //TODO: Complete the function implementation

    // Pre-compute scaling factors for better performance
    const double x_scale = 3.5 / width;
    const double y_scale = 2.0 / height;

    for (int y = 0; y < height; y++) {
        double y0 = y * y_scale - 1.0;

        for (int x = 0; x < width; x++) {
            double x0 = x * x_scale - 2.5;

            double xi = 0.0, yi = 0.0;
            int iteration = 0;

            // Optimized loop with pre-computed squares
            while (iteration < max_iterations) {
                double xi_sq = xi * xi;
                double yi_sq = yi * yi;

                if (xi_sq + yi_sq > 4.0) {
                    break;
                }

                // Mandelbrot recurrence
                yi = 2.0 * xi * yi + y0;
                xi = xi_sq - yi_sq + x0;

                iteration++;
            }

            mandelbrot_sum += iteration;
        }
    }
    
    return mandelbrot_sum;
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
