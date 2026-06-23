#include <Arduino.h>
#include "config.h"

/**
 * System clock configuration.
 *
 * This always overrides the Arduino core's (weak) SystemClock_Config, so the
 * SYSCLK_HZ chosen in config.h is honored on BOTH the HSI and HSE paths.
 *
 * Strategy: divide the oscillator down to a fixed 4 MHz PLL input (PLLM), then
 * multiply back up to the target SYSCLK (PLLN) with PLLR = /2.
 *
 *   PLL input = source / PLLM = 4 MHz
 *   VCO       = 4 MHz * PLLN            (must be 96..344 MHz)
 *   SYSCLK    = VCO / 2
 *   => PLLN   = SYSCLK / 1 MHz / 2
 *
 * Flash wait-states use voltage range 1 BOOST mode (valid to 170 MHz), per the
 * G4 reference table: 0WS<=34, 1WS<=68, 2WS<=102, 3WS<=136, 4WS<=170 MHz.
 *
 * USB keeps sourcing its 48 MHz from HSI48, independent of SYSCLK.
 */

/* --- PLLM: divide the chosen source down to 4 MHz --- */
#if CLOCK_SOURCE == CLOCK_SOURCE_HSI
  /* HSI = 16 MHz */
  #define PLL_M RCC_PLLM_DIV4
#else /* HSE */
  #if   HSE_FREQUENCY_HZ == 24000000UL
    #define PLL_M RCC_PLLM_DIV6
  #elif HSE_FREQUENCY_HZ == 16000000UL
    #define PLL_M RCC_PLLM_DIV4
  #elif HSE_FREQUENCY_HZ == 8000000UL
    #define PLL_M RCC_PLLM_DIV2
  #else
    #error "Unsupported HSE_FREQUENCY_HZ: add a PLLM case in src/clock_config.cpp"
  #endif
#endif

/* --- PLLN + flash latency: derived from the target SYSCLK --- */
#if   SYSCLK_HZ == SYSCLK_170MHZ
  #define PLL_N 85   /* VCO 340 */
  #define SYSCLK_FLASH_LATENCY FLASH_LATENCY_4
#elif SYSCLK_HZ == SYSCLK_144MHZ
  #define PLL_N 72   /* VCO 288 */
  #define SYSCLK_FLASH_LATENCY FLASH_LATENCY_4
#elif SYSCLK_HZ == SYSCLK_128MHZ
  #define PLL_N 64   /* VCO 256 */
  #define SYSCLK_FLASH_LATENCY FLASH_LATENCY_3
#elif SYSCLK_HZ == SYSCLK_64MHZ
  #define PLL_N 32   /* VCO 128 */
  #define SYSCLK_FLASH_LATENCY FLASH_LATENCY_1
#else
  #error "Unsupported SYSCLK_HZ: add a PLL_N / latency case in src/clock_config.cpp"
#endif

extern "C" void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {};

  /* Boost mode: valid for the full 0..170 MHz range we support. */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

#if CLOCK_SOURCE == CLOCK_SOURCE_HSI
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
#else
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
#endif
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;   /* USB 48 MHz source */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLM = PLL_M;            /* source -> 4 MHz */
  RCC_OscInitStruct.PLL.PLLN = PLL_N;            /* -> target VCO */
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;    /* SYSCLK = VCO / 2 */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, SYSCLK_FLASH_LATENCY) != HAL_OK) {
    Error_Handler();
  }

  /* USB keeps its own 48 MHz from HSI48, independent of SYSCLK. */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}
