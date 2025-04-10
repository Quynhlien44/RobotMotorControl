
#ifndef __GPIO_H__
#define __GPIO_H__
#define MOTOR1_IN1_Pin GPIO_PIN_0
#define MOTOR1_IN1_GPIO_Port GPIOA
#define MOTOR1_IN2_Pin GPIO_PIN_1
#define MOTOR1_IN2_GPIO_Port GPIOA
#define MOTOR2_IN1_Pin GPIO_PIN_4
#define MOTOR2_IN1_GPIO_Port GPIOA
#define MOTOR2_IN2_Pin GPIO_PIN_5
#define MOTOR2_IN2_GPIO_Port GPIOA
#define TRIG_Pin GPIO_PIN_10
#define TRIG_GPIO_Port GPIOB

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

