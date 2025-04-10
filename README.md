# Robotics Project: Motor Control with Obstacle Avoidance using STM32L152-RE and FreeRTOS

This project implements a motor-controlled robot with obstacle avoidance using an STM32L152RE microcontroller, FreeRTOS for task scheduling, and an ultrasonic sensor for distance measurement.

---

## **Features**
- **Motor Control**: Drive two DC motors with PWM for speed control and GPIO for direction.
- **Obstacle Avoidance**: Ultrasonic sensor (HC-SR04) detects obstacles and triggers evasive maneuvers.
- **RTOS Tasks**:
  - **MotorControlTask**: Handles motor commands from a queue.
  - **SensorTask**: Triggers ultrasonic sensor and calculates distance.
  - **DecisionTask**: Processes sensor data to adjust robot movement.
- **Precision Delays**: Uses Cortex-M3 DWT cycle counter for µs-level delays.

---

## **Hardware Setup**
### **Components**
- STM32L152RE Nucleo Board
- Dual H-Bridge Motor Driver (e.g., L298N)
- 2x DC Motors
- HC-SR04 Ultrasonic Sensor
- Power Supply (e.g., 6V Battery)

### **Pin Connections**
| STM32 Pin | Component       | Function              |
|-----------|-----------------|-----------------------|
| PA0       | Motor1 IN1      | Direction Control     |
| PA1       | Motor1 IN2      | Direction Control     |
| PA4       | Motor2 IN1      | Direction Control     |
| PA5       | Motor2 IN2      | Direction Control     |
| PB10      | HC-SR04 TRIG    | Trigger Signal        |
| TIM2 CH1  | HC-SR04 ECHO    | Echo Pulse Measurement|
| TIM3 CH1  | Motor1 PWM      | Speed Control         |
| TIM3 CH2  | Motor2 PWM      | Speed Control         |

---

## **Software Dependencies**
- STM32CubeMX (for code generation)
- STM32L1xx HAL Library
- FreeRTOS (CMSIS-RTOS v1 API)
- ARM CMSIS Core (for DWT delay)

---

## **Project Structure**
RobotMotorControl/
├── Core/
│ ├── Inc/ # Header files
│ │ ├── dwt_delay.h
│ │ ├── gpio.h
│ │ ├── robot_control.h
│ ├── Src/ # Source files
│ │ ├── dwt_delay.c # DWT-based µs delay
│ │ ├── freertos.c # FreeRTOS task implementations
│ │ ├── gpio.c # GPIO initialization
│ │ ├── main.c # System setup and RTOS start
│ │ ├── stm32l1xx_it.c # TIM2 interrupt for ultrasonic
├── Drivers/ # STM32 HAL drivers
├── Middlewares/ # FreeRTOS components
├── STM32CubeMX/ # Configuration files


---

## **Configuration Steps**
1. **STM32CubeMX Setup**:
    - Configure TIM3 CH1/CH2 as PWM outputs.
    - Configure TIM2 CH1 for input capture (ultrasonic echo).
    - Enable FreeRTOS with CMSIS-V1 API.
    - Assign GPIO pins for motor direction control (PA0, PA1, PA4, PA5) and TRIG (PB10).
2. **Code Generation**:
    - Generate code with STM32CubeMX and replace/merge with provided source files.
3. **Build and Flash**:
    - Compile using STM32CubeIDE or Makefile.
    - Flash to STM32L152RE via ST-Link.
