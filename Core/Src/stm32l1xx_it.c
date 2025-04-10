/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l1xx_it.h"
#include "FreeRTOS.h"
#include "task.h"
#include "robot_control.h"


/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern UltrasonicCapture_t ultrasonic;
extern QueueHandle_t sensorDataQueue;


static uint32_t risingEdge = 0;
static uint32_t fallingEdge = 0;
static uint8_t captureState = 0;



void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
#endif /* INCLUDE_xTaskGetSchedulerState */
  xPortSysTickHandler();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  }
#endif /* INCLUDE_xTaskGetSchedulerState */
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}


void TIM2_IRQHandler(void)
{
	if(__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC1)) {
	    __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC1);

	    if(ultrasonic.capture_state == 0) {
	      ultrasonic.rising_edge = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
	      __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
	      ultrasonic.capture_state = 1;
	    } else {
	      ultrasonic.falling_edge = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
	      __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
	      ultrasonic.capture_state = 0;

	      uint32_t pulse_width = (ultrasonic.falling_edge > ultrasonic.rising_edge) ?
	        (ultrasonic.falling_edge - ultrasonic.rising_edge) :
	        (0xFFFF - ultrasonic.rising_edge + ultrasonic.falling_edge);

	      uint32_t distance_cm = (uint32_t)(pulse_width * 0.01715f);
	      xQueueSendFromISR(sensorDataQueue, &distance_cm, NULL);
	    }
	  }
}

