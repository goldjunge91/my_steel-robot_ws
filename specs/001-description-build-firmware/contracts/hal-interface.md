# Hardware Abstraction Layer (HAL) Interface Contract

**Feature**: STM32 to Pico Compatibility Layer  
**Date**: 2025-09-23  
**Version**: 1.0  

## Purpose

This HAL provides exact function name compatibility between STM32 firmware and Pico SDK, enabling direct porting of legacy STM32 code with minimal changes.

## GPIO Interface

### Pin Control Functions
```c
// STM32 HAL Compatible Functions
void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

// Pico SDK Implementation
#define HAL_GPIO_WritePin(port, pin, state) gpio_put(pin, state)
#define HAL_GPIO_ReadPin(port, pin) gpio_get(pin)
#define HAL_GPIO_TogglePin(port, pin) gpio_xor_mask(1u << pin)
```

### Pin State Constants
```c
typedef enum {
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET = 1
} GPIO_PinState;

// Legacy pin definitions (ignored but maintained for compatibility)
typedef void GPIO_TypeDef;
#define GPIOA ((GPIO_TypeDef*)0)
#define GPIOB ((GPIO_TypeDef*)0)
#define GPIOC ((GPIO_TypeDef*)0)
```

## PWM Interface

### Timer/PWM Functions
```c
// STM32 HAL Compatible Functions
void __HAL_TIM_SET_COMPARE(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t Compare);
void HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
void HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);

// Pico SDK Implementation Mapping
void pico_pwm_set_duty(uint8_t gpio_pin, uint16_t duty_cycle);
void pico_pwm_enable(uint8_t gpio_pin, bool enabled);

#define __HAL_TIM_SET_COMPARE(htim, channel, value) \
    pico_pwm_set_duty(channel, value)

#define HAL_TIM_PWM_Start(htim, channel) \
    pico_pwm_enable(channel, true)

#define HAL_TIM_PWM_Stop(htim, channel) \
    pico_pwm_enable(channel, false)
```

### Timer Handle Compatibility
```c
typedef struct {
    uint8_t gpio_pin;       // Pico GPIO pin number
    uint8_t slice_num;      // PWM slice number
    uint8_t channel;        // PWM channel (A or B)
} TIM_HandleTypeDef;

// Motor PWM handles (initialized at startup)
extern TIM_HandleTypeDef htim_fl;  // Front-left motor
extern TIM_HandleTypeDef htim_fr;  // Front-right motor  
extern TIM_HandleTypeDef htim_rl;  // Rear-left motor
extern TIM_HandleTypeDef htim_rr;  // Rear-right motor
```

## System Functions

### Delay Functions
```c
// STM32 HAL Compatible Functions
void HAL_Delay(uint32_t Delay);
uint32_t HAL_GetTick(void);

// Pico SDK Implementation
#define HAL_Delay(ms) sleep_ms(ms)
#define HAL_GetTick() to_ms_since_boot(get_absolute_time())
```

### System Initialization
```c
// STM32 HAL Compatible Functions
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_TIM_Init(void);

// Pico SDK Implementation
void pico_system_init(void);
void pico_gpio_init_all(void);
void pico_pwm_init_all(void);

#define SystemClock_Config() pico_system_init()
#define MX_GPIO_Init() pico_gpio_init_all()  
#define MX_TIM_Init() pico_pwm_init_all()
```

## Interrupt Handling

### GPIO Interrupts (Encoders)
```c
// STM32 HAL Compatible Functions
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

// Pico SDK Implementation
void gpio_interrupt_callback(uint gpio, uint32_t events);

// Mapping encoder interrupts to legacy callback
void setup_encoder_interrupts(void);
```

### Timer Interrupts
```c
// STM32 HAL Compatible Functions  
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

// Pico SDK Implementation
bool control_loop_timer_callback(struct repeating_timer *t);

// Main control loop timer (replaces TIM interrupt)
extern struct repeating_timer control_timer;
```

## Memory and Constants

### Pin Definitions
```c
// STM32-style pin definitions mapped to Pico GPIO numbers
#define GPIO_PIN_0   0
#define GPIO_PIN_1   1
#define GPIO_PIN_2   2
// ... up to GPIO_PIN_28

// Motor control pin mappings
#define MOTOR_FL_PWM_PIN    FL_PWM     // GPIO 2
#define MOTOR_FR_PWM_PIN    FR_PWM     // GPIO 3
#define MOTOR_RL_PWM_PIN    RL_PWM     // GPIO 4
#define MOTOR_RR_PWM_PIN    RR_PWM     // GPIO 5
```

### Configuration Constants
```c
// Preserve STM32 timing constants
#define TIM_FREQUENCY       1000000    // 1MHz timer frequency (emulated)
#define PWM_PERIOD          255        // 8-bit PWM period
#define PWM_PRESCALER       1          // No prescaling on Pico

// System configuration
#define HSE_VALUE           12000000   // External crystal (not used on Pico)
#define HSI_VALUE           16000000   // Internal RC (mapped to Pico's 12MHz)
#define SYSCLK_FREQ         133000000  // Pico system clock (133MHz)
```

## Error Handling

### HAL Status Compatibility
```c
typedef enum {
    HAL_OK       = 0x00,
    HAL_ERROR    = 0x01,
    HAL_BUSY     = 0x02,
    HAL_TIMEOUT  = 0x03
} HAL_StatusTypeDef;

// All HAL functions return HAL_OK in Pico implementation
// unless actual error conditions occur
```

### Error Callbacks
```c
// STM32 HAL Compatible Functions
void Error_Handler(void);

// Pico SDK Implementation
void pico_error_handler(const char* error_msg);

#define Error_Handler() pico_error_handler(__func__)
```

## Implementation Requirements

### Initialization Sequence
1. Call `pico_system_init()` - sets up clocks, stdio
2. Call `pico_gpio_init_all()` - configures all GPIO pins
3. Call `pico_pwm_init_all()` - initializes PWM channels
4. Call `setup_encoder_interrupts()` - configures encoder callbacks
5. Start control loop timer

### Performance Guarantees
- GPIO operations: <1μs latency
- PWM updates: <10μs latency  
- Timer callbacks: <200μs execution time
- Interrupt response: <5μs

### Memory Usage
- HAL overhead: <1KB RAM
- Function call overhead: <10% vs direct SDK calls
- No dynamic allocation required

---
*HAL Interface Contract v1.0 - Preserves STM32 API compatibility*