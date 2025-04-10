#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "main.h"
#include "cmsis_os.h"

#define MOTOR1_IN1_GPIO_Port GPIOA
#define MOTOR1_IN1_Pin GPIO_PIN_0
#define MOTOR1_IN2_GPIO_Port GPIOA
#define MOTOR1_IN2_Pin GPIO_PIN_1
#define MOTOR2_IN1_GPIO_Port GPIOA
#define MOTOR2_IN1_Pin GPIO_PIN_4
#define MOTOR2_IN2_GPIO_Port GPIOA
#define MOTOR2_IN2_Pin GPIO_PIN_5


typedef enum {
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT,
  STOP
} Direction_t;

typedef struct {
  Direction_t direction;
  uint16_t leftSpeed;
  uint16_t rightSpeed;
} MotorCommand_t;

extern QueueHandle_t motorCommandQueue;
extern QueueHandle_t sensorDataQueue;

// Ultrasonic measurement structure
typedef struct {
  uint32_t rising_edge;
  uint32_t falling_edge;
  uint8_t capture_state;
} UltrasonicCapture_t;


void Error_Handler(void);

#endif
