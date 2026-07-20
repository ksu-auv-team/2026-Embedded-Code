#include "main.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_opamp.h"

/* HAL handles ---------------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
I2C_HandleTypeDef hi2c2;
OPAMP_HandleTypeDef hopamp3;
UART_HandleTypeDef huart1;

/* Function prototypes from CubeMX ------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C2_Init(void);
static void MX_OPAMP1_Init(void);
static void MX_OPAMP2_Init(void);
static void MX_OPAMP3_Init(void);
static void MX_USART1_UART_Init(void);
void Error_Handler(void);

/* ===================== User safety configuration ========================== */

const float SHUNT_RESISTANCE_OHMS = 0.0005f;
const float OPAMP_GAIN            = 50.0f;
const float ADC_REF_VOLTAGE       = 3.3f;
const uint16_t ADC_MAX_COUNTS     = 4095;

const float CURRENT_SPIKE_LIMIT_A      = 60.0f;
const float CURRENT_SUSTAIN_LIMIT_A    = 50.0f;
const uint32_t CURRENT_SUSTAIN_TIME_MS = 50;

const uint8_t I2C_SLAVE_ADDRESS = 0x4A;
const float BATTERY_DIVIDER_1 = 10.0f;
const float BATTERY_DIVIDER_2 = 10.0f;
const float BATTERY_DIVIDER_3 = 10.0f;
const bool DEBUG_FORCE_MOSFETS_ON = true;

bool faultLatched = false;
uint32_t overCurrentStartMs = 0;

/* ===================== Helper functions =================================== */

float adcToVoltage(uint16_t raw)
{
  return (ADC_REF_VOLTAGE * raw) / ADC_MAX_COUNTS;
}

float readCurrentA(void)
{
  HAL_ADC_Start(&hadc2);
  HAL_ADC_PollForConversion(&hadc2, 10);
  uint16_t raw = HAL_ADC_GetValue(&hadc2);
  HAL_ADC_Stop(&hadc2);

  float vOut   = adcToVoltage(raw);
  float vShunt = vOut / OPAMP_GAIN;
  return vShunt / SHUNT_RESISTANCE_OHMS;
}

float readBatteryVoltage(ADC_HandleTypeDef *hadc, uint32_t channel, float dividerRatio)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  HAL_ADC_ConfigChannel(hadc, &sConfig);

  HAL_ADC_Start(hadc);
  HAL_ADC_PollForConversion(hadc, 10);
  uint16_t raw = HAL_ADC_GetValue(hadc);
  HAL_ADC_Stop(hadc);

  return adcToVoltage(raw) * dividerRatio;
}

void setMosfets(bool on)
{
  GPIO_PinState s = on ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(Q3Gate_GPIO_Port, Q3Gate_Pin, s);
  HAL_GPIO_WritePin(Q6Gate_GPIO_Port, Q6Gate_Pin, s);
}

void latchFault(void)
{
  faultLatched = true;
  setMosfets(false);
  HAL_GPIO_WritePin(SIGNAL_GPIO_Port, SIGNAL_Pin, GPIO_PIN_SET);
}

void updateProtection(void)
{
  if (faultLatched)
  {
    return;
  }

  float currentA = readCurrentA();
  uint32_t now = HAL_GetTick();

  if (currentA >= CURRENT_SPIKE_LIMIT_A)
  {
    latchFault();
    return;
  }

  if (currentA >= CURRENT_SUSTAIN_LIMIT_A)
  {
    if (overCurrentStartMs == 0)
    {
      overCurrentStartMs = now;
    }
    else if (now - overCurrentStartMs >= CURRENT_SUSTAIN_TIME_MS)
    {
      latchFault();
    }
  }
  else
  {
    overCurrentStartMs = 0;
  }
}

void checkManualKill(void)
{
  float currentA = readCurrentA();
  if (currentA < -5.0f || currentA > 200.0f)
  {
    latchFault();
  }
}

/* ===================== I2C slave to Orin ================================== */

uint8_t orinPowerCommand = 1;
bool killSwitchMessageShown = false;

void updateOrinPowerCommand(void)
{
  GPIO_PinState switchState = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
  orinPowerCommand = (switchState == GPIO_PIN_SET) ? 1u : 0u;
}

void logKillSwitchState(void)
{
  if (orinPowerCommand == 1u)
  {
    if (!killSwitchMessageShown)
    {
      const char msg[] = "Killswitch closed: sending 1\r\n";
      HAL_UART_Transmit(&huart1, (uint8_t *)msg, sizeof(msg) - 1, 100);
      killSwitchMessageShown = true;
    }
  }
  else
  {
    if (killSwitchMessageShown)
    {
      const char msg[] = "Killswitch open: sending 0\r\n";
      HAL_UART_Transmit(&huart1, (uint8_t *)msg, sizeof(msg) - 1, 100);
    }
    killSwitchMessageShown = false;
  }
}

#ifdef __cplusplus
extern "C"
{
#endif

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
  (void)AddrMatchCode;

  if (hi2c->Instance != I2C2)
  {
    return;
  }

  if (TransferDirection == I2C_DIRECTION_RECEIVE)
  {
    updateOrinPowerCommand();
    HAL_I2C_Slave_Transmit_IT(hi2c, &orinPowerCommand, 1);
  }
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C2)
  {
    HAL_I2C_EnableListen_IT(hi2c);
  }
}

#ifdef __cplusplus
}
#endif

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_I2C2_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  MX_USART1_UART_Init();

  updateOrinPowerCommand();
  logKillSwitchState();
  setMosfets(true);
  HAL_GPIO_WritePin(SIGNAL_GPIO_Port, SIGNAL_Pin, GPIO_PIN_RESET);
  HAL_I2C_EnableListen_IT(&hi2c2);

  while (1)
  {
    if (DEBUG_FORCE_MOSFETS_ON)
    {
      faultLatched = false;
      setMosfets(true);
      HAL_GPIO_WritePin(SIGNAL_GPIO_Port, SIGNAL_Pin, GPIO_PIN_RESET);
      updateOrinPowerCommand();
      logKillSwitchState();
      HAL_Delay(10);
      continue;
    }

    updateProtection();
    checkManualKill();
    setMosfets(!faultLatched);
    HAL_GPIO_WritePin(SIGNAL_GPIO_Port, SIGNAL_Pin, faultLatched ? GPIO_PIN_SET : GPIO_PIN_RESET);
    updateOrinPowerCommand();
    logKillSwitchState();
    HAL_Delay(10);
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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

static void MX_ADC1_Init(void)
{
  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_ADC2_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_17;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C2_Init(void)
{
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00503D58;
  hi2c2.Init.OwnAddress1 = (I2C_SLAVE_ADDRESS << 1);
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_OPAMP1_Init(void)
{
}

static void MX_OPAMP2_Init(void)
{
}

static void MX_OPAMP3_Init(void)
{
  hopamp3.Instance = OPAMP3;
  hopamp3.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp3.Init.Mode = OPAMP_PGA_MODE;
  hopamp3.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp3.Init.InternalOutput = ENABLE;
  hopamp3.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp3.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_NO;
  hopamp3.Init.PgaGain = OPAMP_PGA_GAIN_2_OR_MINUS_1;
  hopamp3.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp3) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, SIGNAL_Pin|Q6Gate_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, Q3Gate_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = OPAMP1_P_Pin|OPAMP1_N_Pin|OPAMP2_N_Pin|OPAMP2_P_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SIGNAL_Pin|Q6Gate_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = Q3Gate_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = Q4Gate_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}