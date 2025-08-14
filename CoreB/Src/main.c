/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file : main.c
  * @brief : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_rcc.h"

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

#define MAX_ITER 100

// Image dimensions
uint64_t height[5] = {128, 160, 192, 224, 256};
uint64_t width[5] = {128, 160, 192, 224, 256};

// Global variables for all test results
volatile uint32_t start_time = 0;
volatile uint32_t end_time = 0;
volatile uint32_t execution_time[5];
volatile uint64_t checksum[5];

// Test mode: 0 = fixed-point, 1 = double
volatile int test_mode = 1;  // Change this to switch modes

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations);

/* Private user code ---------------------------------------------------------*/


int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();


    /* USER CODE BEGIN 2 */

    // Run all 5 test cases
    for (int j = 0; j < 5; j++) {

        //TODO: Turn on LED 0 to signify the start of the operation
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

        //TODO: Record the start time
        start_time = HAL_GetTick();

        //TODO: Call the Mandelbrot Function and store the output in the checksum variable
        if (test_mode == 0) {
            // Fixed-point arithmetic
            checksum[j] = calculate_mandelbrot_fixed_point_arithmetic(width[j], height[j], MAX_ITER);
        } else {
            // Double precision
            checksum[j] = calculate_mandelbrot_double(width[j], height[j], MAX_ITER);
        }

        //TODO: Record the end time
        end_time = HAL_GetTick();

        //TODO: Calculate the execution time
        execution_time[j] = end_time - start_time;

        //TODO: Turn on LED 1 to signify the end of the operation
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

        //TODO: Hold the LEDs on for a 1s delay
        HAL_Delay(1000);

        //TODO: Turn off the LEDs
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

        // Small delay between tests
        HAL_Delay(500);
    }

    // Set breakpoint here to examine all results
    volatile int debug_complete = 1; // Breakpoint marker

    /* USER CODE END 2 */

    /* Infinite loop */
    while (1)
    {
        // Results are stored in checksum[0] through checksum[4]
        // and execution_time[0] through execution_time[4]
    }
}

/* System Clock and GPIO Configuration (same as yours) */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

//TODO: Mandelbrot using fixed point arithmetic
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations) {

    uint64_t mandelbrot_checksum = 0;
    int64_t scale_factor = 1000000;


    for (int y = 0; y <= height - 1; y++) {
        for (int x = 0; x <= width - 1; x++) {
        	int64_t x0 = (x * scale_factor / width) * 35 / 10 - 25 * scale_factor / 10;
        	int64_t y0 = (y * scale_factor / height) * 2 - scale_factor;

            int64_t xi = 0;
            int64_t yi = 0;
            int64_t iteration = 0;

            while ((iteration < max_iterations) && (xi * xi + yi * yi <= 4 * scale_factor * scale_factor)) {
            	int64_t xi_temp = (xi * xi - yi * yi) / scale_factor;
                yi = (2 * xi * yi) / scale_factor + y0;
                xi = xi_temp + x0;
                ++iteration;
            }

            mandelbrot_checksum += iteration;
        }
    }
    return mandelbrot_checksum;
}
//TODO: Mandelbrot using double precision
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations) {
    uint64_t mandelbrot_checksum = 0;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            // Your exact coordinate mapping
            double x0 = ((double)x / (double)width) * 3.5 - 2.5;
            double y0 = ((double)y / (double)height) * 2.0 - 1.0;

            double xi = 0.0;
            double yi = 0.0;
            int iteration = 0;

            while (iteration < max_iterations && (xi * xi + yi * yi <= 4.0)) {
                double xi_temp = xi * xi - yi * yi + x0;
                yi = 2.0 * xi * yi + y0;
                xi = xi_temp;
                iteration++;
            }

            mandelbrot_checksum += iteration;
        }
    }
    return mandelbrot_checksum;
}

/* USER CODE END 4 */

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}
