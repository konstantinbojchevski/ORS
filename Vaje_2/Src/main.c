/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stdint.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADDRESS_CLOCK ((uint32_t *) 0x40023830)
#define ADDRESS_BTN_INIT1 ((uint32_t *)  0x40020000)
#define ADDRESS_BTN_INIT2 ((uint32_t *)  0x4002000C)
#define ADDRESS_LED_INIT0 ((uint32_t *)  0x40020C00)
#define ADDRESS_LED_INIT1 ((uint32_t *)  0x40020C04)
#define ADDRESS_LED_INIT2 ((uint32_t *)  0x40020C08)
#define ADDRESS_LED_INIT3 ((uint32_t *)  0x40020C0C)
#define ADDRESS_LED_ON ((uint16_t *)  0x40020C18)
#define ADDRESS_LED_OFF ((uint16_t *)  0x40020C1A)
#define ADDRESS_READ_BTN ((uint16_t *)  0x40020010)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void reset_two_bits(volatile uint32_t *address, uint8_t p);
void set_bit(volatile uint32_t *address, uint8_t p);
void set_bit_16(volatile uint16_t *address, uint8_t p);
void set_two_bits_to(volatile uint32_t *address, uint8_t p, uint8_t n);
void button_port_clock_on();
void led_port_clock_on();
void button_init();
void led_init();
void led_on(uint8_t i);
void led_off(uint8_t i);
uint8_t read_button();
void delay();
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
  /* USER CODE BEGIN 2 */

  // turn on required peripherals
  button_port_clock_on();
  led_port_clock_on();


  // init required periphearls
  button_init();
  led_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	uint8_t count=0;
	if(read_button() && count==0){
	   led_on(0);
	   delay();
	   led_on(1);
	   delay();
	   led_on(2);
	   delay();
	   led_on(3);
	   delay();
	   count++;
	   count %= 2;
	}
	if(read_button() && count==1){
	   led_off(0);
	   delay();
	   led_off(1);
	   delay();
	   led_off(2);
	   delay();
	   led_off(3);
	   delay();
	   count++;
	   count %= 2;
	}
    // TODO READ BUTTON AND START LED SEQUENCE

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
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
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

/* USER CODE BEGIN 4 */

void reset_two_bits(volatile uint32_t *address, uint8_t p)
{
  // reset two bits (p and p+1) in a 32 bit register
  *address = *address & ~(3<<p);
}

void set_bit(volatile uint32_t *address, uint8_t p)
{
  // set bit p in a 32 bit register
  *address = *address | (1<<p);
}

void set_bit_16(volatile uint16_t *address, uint8_t p)
{
  // set bit p in a 16 bit register
  *address = *address | (1<<p);
}

void set_two_bits_to(volatile uint32_t *address, uint8_t p, uint8_t n)
{
  // set two bits (p and p+1) to value n in a 32 bit register
  *address = *address & ~(3<<p);
  *address = *address | (n<<p);
}

void button_port_clock_on()
{
  // turn on the button port clock
  uint32_t *p = ADDRESS_CLOCK;
  *p |= 1;

}

void led_port_clock_on()
{
  // turn on the led port clockADDRESS_READ_BTN
  uint32_t *p = ADDRESS_CLOCK;
  *p |= (1<<3);
}

void button_init()
{
  // init the button
  uint32_t *p = ADDRESS_BTN_INIT1;
  *p &= ~(3);
  uint32_t *q = ADDRESS_BTN_INIT2;
  *q &= ~(3);
}

void led_init()
{
  // init leds
  uint32_t *p = ADDRESS_LED_INIT0;
  *p &= ~((0xFF)<<24);
  *p |= (0x55) << 24;
  uint32_t *q = ADDRESS_LED_INIT1;
  *q &= ~((0xFF)<<24);
  uint32_t *r = ADDRESS_LED_INIT2;
  *r &= ~((0xFF)<<24);
  uint32_t *s = ADDRESS_LED_INIT3;
  *s &= ~((0xFF)<<24);
}

void led_on(uint8_t i)
{
  // turn led number i on
  uint16_t *p = ADDRESS_LED_ON;
  *p |= (1<<(i+12));
}

void led_off(uint8_t i)
{
  // turn led number i off
  uint16_t *p = ADDRESS_LED_OFF;
  *p |= (1<<(i+12));
}

uint8_t read_button()
{
  // get button state
  uint16_t *p = ADDRESS_READ_BTN;
  return (*p & 1);
}

void delay()
{
  // hardcoded delay
  int volatile  i = 0;
  while(i<100000){
	  i++;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/d
