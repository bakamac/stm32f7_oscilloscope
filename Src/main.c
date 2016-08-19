/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "i2c.h"
#include "ltdc.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "interfejs.h"
//#include "RGB565_480x272.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

uint8_t flaga_wyswietl = 0;

uint8_t tryb = 0;

uint8_t bufor[480];

uint8_t podstawa_czasu = 3;
uint8_t podstawa_czasu_zmiana = 0;

uint8_t ch1 = 0, ch1_poprzednie = 0;
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LTDC_Init();
  MX_TIM6_Init();
  MX_I2C1_Init();
  MX_TIM7_Init();

  /* USER CODE BEGIN 2 */
 // HAL_I2C_IsDeviceReady(&hi2c1, LCD_ADDR, 10, 1);

  lcd_init();

  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);

  tryb = 0;

  wyswietl_interfejs(0, 9, 0b1111111111100000, 1);
  wyswietl_ikony();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  //if(flaga_wyswietl == 1)
	  //{
		//  HAL_LTDC_ConfigLayer(&hltdc, &hltdc.LayerCfg[1], 1);
		//  flaga_wyswietl = 0;
	  //}
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  HAL_PWREx_EnableOverDrive();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_I2C1;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 50;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)
{
	static uint32_t czas = 0;
	uint16_t i;
	if(htim ->Instance == TIM6)
	{
		if(tryb == 0)
		{
			if(ch1 != ch1_poprzednie) lcd_drawVline(1, 21, czas+30, 0b1111111111100000);
			ch1_poprzednie = ch1;
			if(ch1 == 1)
				lcd_setpixel(czas+30, 1, 0b1111111111100000);
			else
				lcd_setpixel(czas+30, 21, 0b1111111111100000);

			for(i = czas+31; i < czas+30+20; i++)
			{
				if(i < IKONY_KRAWEDZ)
				{
					lcd_drawVline(1, 21, i, 0);
				}
				else
				{
					lcd_drawVline(1, 21, INTERFEJS_KRAWEDZ + i%IKONY_KRAWEDZ+1, 0);
				}
			}
			if(czas + 30 == IKONY_KRAWEDZ - 1) czas = 0;
		}
		else
		{

		}

		czas++;

		//if(czas == 100) flaga_wyswietl = 1;
	}
	else if(htim -> Instance == TIM7)
	{
		HAL_LTDC_ConfigLayer(&hltdc, &hltdc.LayerCfg[1], 1);
	}
}

void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_7)
	{
		ch1 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);
	}
	else if(GPIO_Pin == GPIO_PIN_11)
	{
		podstawa_czasu++;
		if(podstawa_czasu >= 5) podstawa_czasu = 1;
		  if(podstawa_czasu == 0)
		  {
			  TIM6->PSC = 10-1;
			  TIM6->ARR = 108;
		  }
		  else if(podstawa_czasu == 1)
		  {
			  TIM6->PSC = 10-1;
			  TIM6->ARR = 1080;
		  }
		  else if(podstawa_czasu == 2)
		  {
			  TIM6->PSC = 10-1;
			  TIM6->ARR = 10800;
		  }
		  else if(podstawa_czasu == 3)
		  {
			  TIM6->PSC = 100-1;
			  TIM6->ARR = 10800;
		  }
		  else if(podstawa_czasu == 4)
		  {
			  TIM6->PSC = 1000-1;
			  TIM6->ARR = 10800;
		  }
		  else if(podstawa_czasu == 5)
		  {
			  TIM6->PSC = 10000-1;
			  TIM6->ARR = 10800;
		  }
	}
	else if(GPIO_Pin == GPIO_PIN_13)
	{
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
	}
}
/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
