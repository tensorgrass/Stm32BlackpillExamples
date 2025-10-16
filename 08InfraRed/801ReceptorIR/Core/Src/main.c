/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
#define NEC_BIT_COUNT 32
#define NEC_BIT_COUNT_BUFFER 99
volatile uint32_t captured_edges[NEC_BIT_COUNT * 2] = {0}; // Almacena los tiempos de cada flanco
volatile uint32_t captured_edges_difference[NEC_BIT_COUNT_BUFFER] = {0}; // Almacena los tiempos de cada flanco
volatile uint8_t edge_count = 0; // Contador de flancos
volatile uint8_t ir_data_ready = 0; // Flag para indicar que se recibió una trama completa

uint32_t decoded_data = 0; // Aquí guardaremos el código decodificado

char resultado[99];

typedef enum {
   START_PULSE_BRUST,
   SPACE_SELECTOR,
   FULL_VALUE_PULSE_BRUST,
   FULL_VALUE,
   REPETITION_VALUE_PULSE_BRUST,
   END_FULL_VALUE,
   END_REPETITION_VALUE,
   ERROR_VALUE
} enum_ir_protocol; // <-- El nuevo nombre de tipo se define aquí al final

volatile enum_ir_protocol ir_step;
volatile uint32_t ir_previous_value_ms = 0;
volatile uint32_t ir_current_value_ms = 0;
volatile uint32_t ir_repetitions = 0;
volatile uint32_t decoded_data_tmp = 0;
volatile uint8_t ir_value_position;
volatile uint8_t ir_is_valid_value;

volatile uint8_t ir_address;
volatile uint8_t ir_address_inverse;
volatile uint8_t ir_command;
volatile uint8_t ir_command_inverse;

volatile uint8_t ir_address_array[8];
volatile uint8_t ir_address_inverse_array[8];
volatile uint8_t ir_command_array[8];
volatile uint8_t ir_command_inverse_array[8];




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      if (ir_data_ready)
      {

          // Extraer la dirección y el comando (en el protocolo NEC)
          uint8_t address = (decoded_data >> 24) & 0xFF;
          uint8_t command = (decoded_data >> 8) & 0xFF;

          // Imprime por SWV o UART para depurar
//          printf("Datos recibidos: 0x%08lX -> Comando: 0x%02X\n", decoded_data, command);
          sprintf(resultado, "Datos recibidos: 0x%08lX -> Comando: 0x%02X\n", decoded_data, command);

          // Limpiamos y reiniciamos para la siguiente captura
          ir_data_ready = 0;
          HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffffffff-1;
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
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 4;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  uint32_t ir_diference;
  if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
  {

    ir_current_value_ms = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
    ir_diference = ir_current_value_ms - ir_previous_value_ms;
    captured_edges_difference[edge_count++] = ir_diference;
    if (edge_count >= NEC_BIT_COUNT_BUFFER) {
      edge_count = 0;
    }
    ir_previous_value_ms = ir_current_value_ms;

    if (ir_diference > 10000) {
      __HAL_TIM_SET_COUNTER(htim, 0);
      ir_previous_value_ms = 0;
      ir_step = START_PULSE_BRUST;

    }
    else {
      switch(ir_step){
      case (START_PULSE_BRUST):
        if (8750 < ir_diference && ir_diference < 9250) { //9 ms
          ir_step = SPACE_SELECTOR;
        }
        else {
          ir_step = ERROR_VALUE;
          ir_is_valid_value = 0;
        }
        break;
      case (SPACE_SELECTOR):
        if (4250 < ir_diference && ir_diference < 4750) {// 4.5ms full value
          ir_step = FULL_VALUE_PULSE_BRUST;
          ir_value_position = 0;
          decoded_data_tmp = 0;
          ir_repetitions = 0;

          ir_address = 0;
          ir_address_inverse = 0;
          ir_command = 0;
          ir_command_inverse = 0;

          memset((void*)ir_address_array, 0, 8); // Se pone (void*) para que no salga el warning
          memset((void*)ir_address_inverse_array, 0, 8);
          memset((void*)ir_command_array, 0, 8);
          memset((void*)ir_command_inverse_array, 0, 8);


        }
        else if (2000 < ir_diference && ir_diference < 2500) {// 2.25ms repeat code
          if (ir_is_valid_value == 1) {
            ir_step = REPETITION_VALUE_PULSE_BRUST;
          }
          else {
            ir_step = ERROR_VALUE;
            ir_is_valid_value = 0;
          }
        }
        else {
          ir_step = ERROR_VALUE;
          ir_is_valid_value = 0;
        }
        break;
      case (FULL_VALUE_PULSE_BRUST):
        if (320 < ir_diference && ir_diference < 812) { // 0.5625ms
          if (ir_value_position == 32){
            ir_step = END_FULL_VALUE;
            decoded_data = decoded_data_tmp;
            ir_is_valid_value = 1;
          }
          else {
            ir_step = FULL_VALUE;
          }
        }
        else {
          ir_step = ERROR_VALUE;
          ir_is_valid_value = 0;
        }
        break;
      case (FULL_VALUE):
        if (1437 < ir_diference && ir_diference < 1937) { //1.6875ms para los 1
          decoded_data_tmp |= (1UL << (NUM_BITS_VALUE - ir_value_position - 1));

          if (ir_value_position < 8) {
            ir_address_array[ir_value_position] = 1;
            ir_address |= (1 << ir_value_position);
            ir_address_inverse |= (1 << (7 - ir_value_position));
          }
          else if (ir_value_position < 16) {
            ir_address_inverse_array[ir_value_position - 8] = 1;
          }
          else if (ir_value_position < 24) {
            ir_command_array[ir_value_position - 16] = 1;
            ir_command |= (1 << (ir_value_position - 16));
            ir_command_inverse |= (1 << (7 - (ir_value_position - 16)));
          }
          else if (ir_value_position < 32) {
            ir_command_inverse_array[ir_value_position - 24] = 1;
          }

          ir_value_position++;
          ir_step = FULL_VALUE_PULSE_BRUST;
        }
        else if (312 < ir_diference && ir_diference < 812) { //0.5625ms para los 0
          ir_value_position++;
          ir_step = FULL_VALUE_PULSE_BRUST;
          //no hace falta poner el valor ya que está a cero
        }
        else {
          ir_step = ERROR_VALUE;
          ir_is_valid_value = 0;
        }
        break;
      case (REPETITION_VALUE_PULSE_BRUST):
        if (320 < ir_diference && ir_diference < 812) { // 0.5625ms
          ir_step = END_REPETITION_VALUE;
          ir_repetitions++;
        }
        else {
          ir_step = ERROR_VALUE;
          ir_is_valid_value = 0;
        }
        break;
      case (END_FULL_VALUE):
        ir_step = ERROR_VALUE;
        ir_is_valid_value = 0;
      case (END_REPETITION_VALUE):
        ir_step = ERROR_VALUE;
        ir_is_valid_value = 0;
      default:
        ir_step = ERROR_VALUE;
        ir_is_valid_value = 0;
        break;
      }
    }


    if (ir_step == END_FULL_VALUE || ir_step == END_REPETITION_VALUE) // Si ya capturamos todos los bits
    {
      ir_data_ready = 1;
      HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_3); // Detenemos para procesar
    }
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
