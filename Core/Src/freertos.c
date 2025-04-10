
/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "queue.h"
#include "robot_control.h"
#include "dwt_delay.h"
#include "gpio.h"


/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId MotorControlTasHandle;
osThreadId SensorTaskHandle;
osThreadId DecisionTaskHandle;
osMessageQId motorCommandQueueHandle;
osMessageQId sensorDataQueueHandle;

/* External variables */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
UltrasonicCapture_t ultrasonic = {0};

void StartDefaultTask(void const * argument);
void StartMotorControlTask(void * argument);
void StartSensorTask(void * argument);
void StartDecisionTask(void * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {

  /* Create the queue(s) */
  /* definition and creation of motorCommandQueue */
	osThreadDef(MotorControlTas, StartMotorControlTask, osPriorityHigh, 0, 256);
	osThreadDef(SensorTask, StartSensorTask, osPriorityAboveNormal, 0, 256);
	osThreadDef(DecisionTask, StartDecisionTask, osPriorityHigh, 0, 256);


}


void StartDefaultTask(void const * argument)
{
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }

}


void StartMotorControlTask(void * argument) {
  MotorCommand_t cmd;

  for(;;) {
    if(xQueueReceive(motorCommandQueue, &cmd, portMAX_DELAY)) {
      switch(cmd.direction) {
        case FORWARD:
          HAL_GPIO_WritePin(MOTOR1_IN1_GPIO_Port, MOTOR1_IN1_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(MOTOR1_IN2_GPIO_Port, MOTOR1_IN2_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(MOTOR2_IN1_GPIO_Port, MOTOR2_IN1_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(MOTOR2_IN2_GPIO_Port, MOTOR2_IN2_Pin, GPIO_PIN_RESET);
          break;
        case BACKWARD:
          HAL_GPIO_WritePin(MOTOR1_IN1_GPIO_Port, MOTOR1_IN1_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(MOTOR1_IN2_GPIO_Port, MOTOR1_IN2_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(MOTOR2_IN1_GPIO_Port, MOTOR2_IN1_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(MOTOR2_IN2_GPIO_Port, MOTOR2_IN2_Pin, GPIO_PIN_SET);
          break;
        case LEFT:
          HAL_GPIO_WritePin(MOTOR1_IN1_GPIO_Port, MOTOR1_IN1_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(MOTOR1_IN2_GPIO_Port, MOTOR1_IN2_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(MOTOR2_IN1_GPIO_Port, MOTOR2_IN1_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(MOTOR2_IN2_GPIO_Port, MOTOR2_IN2_Pin, GPIO_PIN_RESET);
          break;
        case RIGHT:
          HAL_GPIO_WritePin(MOTOR1_IN1_GPIO_Port, MOTOR1_IN1_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(MOTOR1_IN2_GPIO_Port, MOTOR1_IN2_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(MOTOR2_IN1_GPIO_Port, MOTOR2_IN1_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(MOTOR2_IN2_GPIO_Port, MOTOR2_IN2_Pin, GPIO_PIN_SET);
          break;
        case STOP:
          HAL_GPIO_WritePin(MOTOR1_IN1_GPIO_Port, MOTOR1_IN1_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(MOTOR1_IN2_GPIO_Port, MOTOR1_IN2_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(MOTOR2_IN1_GPIO_Port, MOTOR2_IN1_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(MOTOR2_IN2_GPIO_Port, MOTOR2_IN2_Pin, GPIO_PIN_RESET);
          break;
      }
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, cmd.leftSpeed);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, cmd.rightSpeed);
    }
    osDelay(10);
}


void StartSensorTask(void * argument)
{
	const TickType_t xDelay = pdMS_TO_TICKS(100);
	for(;;) {
	    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
	    DWT_Delay_us(10);
	    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
	    vTaskDelay(xDelay);
	 }
}

void StartDecisionTask(void * argument)
{
	uint32_t distance = 0;
	  MotorCommand_t cmd = {.leftSpeed = 800, .rightSpeed = 800};
	  const uint32_t OBSTACLE_THRESHOLD = 20; //20cm

	  for(;;) {
	    if(xQueueReceive(sensorDataQueue, &distance, portMAX_DELAY)) {
	      if(distance < OBSTACLE_THRESHOLD) {
	    	  cmd.direction = STOP;
	    	  xQueueOverwrite(motorCommandQueue, &cmd);
	    	  vTaskDelay(pdMS_TO_TICKS(2000));

	    	  cmd.direction = BACKWARD;
	    	  xQueueOverwrite(motorCommandQueue, &cmd);
	    	  vTaskDelay(pdMS_TO_TICKS(1000));

	    	  cmd.direction = RIGHT;
	    	  xQueueOverwrite(motorCommandQueue, &cmd);
	    	  vTaskDelay(pdMS_TO_TICKS(500));
	    	} else {
	    	  cmd.direction = FORWARD;
	    	  xQueueOverwrite(motorCommandQueue, &cmd);
	      }
	    }
	    osDelay(50);
	 }
}

void DWT_Delay_us(volatile uint32_t microseconds) {
  uint32_t clk_cycle_start = DWT->CYCCNT;
  microseconds *= (SystemCoreClock / 1000000);
  while ((DWT->CYCCNT - clk_cycle_start) < microseconds);
}
}
