#include "main.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* Private variables */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// PID控制器结构体
typedef struct {
    volatile float kp, ki, kd;
    float target, current;
    float error, last_error, integral;
    float derivative;
    float p_term, i_term, d_term;
    float output;
    float output_max, output_min;
} PID_t;

// 电机控制结构体
typedef struct {
    volatile float target_speed;    // 目标速度：车轮转速（圈/秒）
    float current_speed;            // 当前速度：车轮转速（圈/秒）
    int32_t encoder_count;
    int32_t last_encoder;
    float encoder_rps;              // 编码器转速（圈/秒）
    float wheel_rps;                // 车轮转速（圈/秒）
    float pwm_output;
    uint8_t direction;
    uint8_t speed_mode;
    uint8_t enable;
    volatile uint8_t control_mode;  // 0=按钮控制, 1=VOFA+控制
} Motor_t;

// 系统状态结构体
typedef struct {
    uint8_t pid_tuning_mode;
    uint8_t data_send_mode;
    uint32_t system_time;
} System_t;

// 全局变量
Motor_t motor = {0};
PID_t speed_pid = {0};
System_t sys_status = {0};

volatile uint8_t control_flag = 0;
volatile uint8_t button_check_flag = 0;

// 参数定义
#define ENCODER_PPR 500.0f          // 编码器每圈脉冲数
#define CONTROL_FREQUENCY 1000      // 控制频率
#define WHEEL_CONVERSION 0.0085f    // 换算系数：车轮转速 = 编码器转速 × 0.0085

// 按键状态结构体
typedef struct {
    uint8_t current_state;
    uint8_t last_state;
    uint32_t press_time;
} Button_t;

Button_t button_mode = {0};
Button_t button_forward = {0};
Button_t button_reverse = {0};

// 串口接收变量
volatile char uart_cmd[64];
volatile uint8_t uart_index = 0;
volatile uint8_t cmd_ready = 0;
uint8_t rx_byte;

volatile uint32_t uart_int_count = 0;
volatile uint32_t cmd_process_count = 0;

// 重定向printf到UART
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 1000);
    return len;
}

// 串口中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        uart_int_count++;

        if (rx_byte == '\n') {
            uart_cmd[uart_index] = '\0';
            cmd_ready = 1;
            uart_index = 0;
        } else if (uart_index < 63) {
            uart_cmd[uart_index] = rx_byte;
            uart_index++;
        }

        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}

// 命令处理函数
void TestCommandProcess(void) {
    cmd_process_count++;

    char *cmd = (char*)uart_cmd;
    printf("CMD:[%s]\n", cmd);

    if (strstr(cmd, "pid:kp,") != NULL) {
        char *start = strstr(cmd, "pid:kp,") + 7;
        float val = atof(start);
        speed_pid.kp = val;
        printf("NEW_KP=%.1f\n", val);
    }
    else if (strstr(cmd, "pid:ki,") != NULL) {
        char *start = strstr(cmd, "pid:ki,") + 7;
        float val = atof(start);
        speed_pid.ki = val;
        printf("NEW_KI=%.1f\n", val);
    }
    else if (strstr(cmd, "pid:kd,") != NULL) {
        char *start = strstr(cmd, "pid:kd,") + 7;
        float val = atof(start);
        speed_pid.kd = val;
        printf("NEW_KD=%.1f\n", val);
    }
    else if (strstr(cmd, "speed:") != NULL) {
        char *start = strstr(cmd, "speed:") + 6;
        float val = atof(start);
        motor.target_speed = val;  // 直接设置车轮转速（圈/秒）
        motor.control_mode = 1;
        printf("NEW_SPEED=%.2f WHEEL_RPS\n", val);
    }
    else if (strstr(cmd, "dir:") != NULL) {
        char *start = strstr(cmd, "dir:") + 4;
        int val = atoi(start);
        motor.direction = (val > 0) ? 1 : 0;
        printf("NEW_DIR=%d\n", motor.direction);
    }
    else if (strstr(cmd, "enable:") != NULL) {
        char *start = strstr(cmd, "enable:") + 7;
        int val = atoi(start);
        motor.enable = val;
        printf("MOTOR_ENABLE=%d\n", val);
    }
    else if (strstr(cmd, "mode:button") != NULL) {
        motor.control_mode = 0;
        printf("BUTTON_CONTROL_MODE\n");
    }
    else if (strstr(cmd, "mode:vofa") != NULL) {
        motor.control_mode = 1;
        printf("VOFA_CONTROL_MODE\n");
    }
    else if (strstr(cmd, "reset") != NULL) {
        speed_pid.integral = 0;
        speed_pid.last_error = 0;
        printf("PID_RESET\n");
    }
    else if (strstr(cmd, "info") != NULL) {
        printf("ENCODER_PPR=%.0f\n", ENCODER_PPR);
        printf("WHEEL_CONVERSION=%.4f\n", WHEEL_CONVERSION);
        printf("CONTROL_MODE=%s\n", motor.control_mode ? "VOFA+" : "BUTTON");
    }
}

// 发送详细的调试数据
void SendDetailedData(void) {
    printf("debug:%.3f,%.3f,%.1f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d\n",
           motor.target_speed,      // 目标速度（车轮圈/秒）
           motor.current_speed,     // 当前速度（车轮圈/秒）
           motor.pwm_output,        // PWM输出
           speed_pid.error,         // PID误差（编码器单位）
           speed_pid.kp,            // Kp参数
           speed_pid.ki,            // Ki参数
           speed_pid.kd,            // Kd参数
           speed_pid.p_term,        // PID P项
           speed_pid.i_term,        // PID I项
           speed_pid.d_term,        // PID D项
           motor.speed_mode,        // 速度模式
           motor.direction);        // 方向
}

// 发送系统状态信息
void SendStatusInfo(void) {
    printf("status:PID[Kp=%.2f Ki=%.2f Kd=%.2f] Target=%.3f Current=%.3f PWM=%.0f\n",
           speed_pid.kp, speed_pid.ki, speed_pid.kd,
           motor.target_speed, motor.current_speed, motor.pwm_output);
    printf("encoder_rps=%.1f, wheel_rps=%.3f, conversion=%.4f\n",
           motor.encoder_rps, motor.wheel_rps, WHEEL_CONVERSION);
}

// 发送调试计数器信息
void SendDebugCounters(void) {
    printf("COUNTERS:UART_INT=%lu,CMD_PROC=%lu\n", uart_int_count, cmd_process_count);
}

// 按键检测函数
uint8_t ReadButton(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    return (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET) ? 1 : 0;
}

// 按键处理函数
void ProcessButtons(void) {
    static uint32_t last_check_time = 0;
    uint32_t current_time = HAL_GetTick();

    if (current_time - last_check_time < 20) {
        return;
    }
    last_check_time = current_time;

    button_mode.current_state = ReadButton(GPIOC, GPIO_PIN_13);
    button_forward.current_state = ReadButton(GPIOC, GPIO_PIN_14);
    button_reverse.current_state = ReadButton(GPIOC, GPIO_PIN_15);

    // 按钮模式下才响应按键
    if (motor.control_mode == 0) {
        if (button_mode.current_state && !button_mode.last_state) {
            motor.speed_mode = (motor.speed_mode + 1) % 3;
            switch(motor.speed_mode) {
                case 0: motor.target_speed = 0.5f; break;  // 车轮0.5圈/秒
                case 1: motor.target_speed = 1.0f; break;  // 车轮1圈/秒
                case 2: motor.target_speed = 2.0f; break;  // 车轮2圈/秒
            }
            printf("BUTTON_SPEED_MODE_%d: %.1f WHEEL_RPS\n", motor.speed_mode, motor.target_speed);
        }

        if (button_forward.current_state && !button_forward.last_state) {
            motor.direction = 0;
            printf("BUTTON_FORWARD\n");
        }

        if (button_reverse.current_state && !button_reverse.last_state) {
            motor.direction = 1;
            printf("BUTTON_REVERSE\n");
        }
    }

    button_mode.last_state = button_mode.current_state;
    button_forward.last_state = button_forward.current_state;
    button_reverse.last_state = button_reverse.current_state;
}

/* USER CODE END PV */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    /* USER CODE BEGIN 2 */
    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_USART2_UART_Init();

    // 初始化TB6612
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

    // 初始化PID参数（针对编码器转速）
    speed_pid.kp = 110f;           // 比例增益
    speed_pid.ki = 0.5f;           // 积分增益
    speed_pid.kd = 0.1f;           // 微分增益
    speed_pid.output_max = PWM_MAX_VALUE * 0.9f;
    speed_pid.output_min = -PWM_MAX_VALUE * 0.9f;

    // 初始化电机参数
    motor.target_speed = 1.0f;     // 目标：车轮1圈/秒
    motor.enable = 1;
    motor.speed_mode = 1;
    motor.direction = 0;
    motor.control_mode = 1;        // 默认VOFA+控制模式

    // 初始化系统状态
    sys_status.data_send_mode = 1;
    sys_status.pid_tuning_mode = 1;

    // 启动定时器
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Base_Start_IT(&htim3);

    // 启动串口接收中断
    printf("STARTING UART...\n");
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    printf("UART STARTED\n");

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);

    printf("WHEEL-BASED MOTOR CONTROL\n");
    printf("ENCODER_PPR=%.0f, CONVERSION=%.4f\n", ENCODER_PPR, WHEEL_CONVERSION);
    printf("TARGET: %.2f WHEEL_RPS (wheel revolutions per second)\n", motor.target_speed);
    /* USER CODE END 2 */

    while (1)
    {
        /* USER CODE BEGIN WHILE */

        // 处理串口命令
        if (cmd_ready) {
            cmd_ready = 0;
            TestCommandProcess();
        }

        // PID控制逻辑
        if (control_flag) {
            control_flag = 0;

            // 读取编码器
            motor.encoder_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
            int32_t encoder_diff = motor.encoder_count - motor.last_encoder;
            motor.last_encoder = motor.encoder_count;

            // 处理编码器溢出
            if (encoder_diff > 32768) encoder_diff -= 65536;
            if (encoder_diff < -32768) encoder_diff += 65536;

            // 计算编码器转速（圈/秒）
            motor.encoder_rps = (float)encoder_diff * CONTROL_FREQUENCY / ENCODER_PPR;

            // **关键：转换为车轮转速**
            motor.wheel_rps = motor.encoder_rps * WHEEL_CONVERSION;
            motor.current_speed = fabs(motor.wheel_rps);  // 当前车轮转速（圈/秒）

            // 只有使能时才进行PID控制
            if (motor.enable) {
                // **PID计算：将车轮目标转速转换为编码器目标转速进行控制**
                float target_encoder_rps = motor.target_speed / WHEEL_CONVERSION;

                if (motor.direction == 0) {
                    // 正转
                    speed_pid.error = target_encoder_rps - motor.encoder_rps;
                } else {
                    // 反转
                    speed_pid.error = -target_encoder_rps - motor.encoder_rps;
                }

                // PID各项计算
                speed_pid.p_term = speed_pid.kp * speed_pid.error;

                speed_pid.integral += speed_pid.error / CONTROL_FREQUENCY;
                // 积分限幅
                float integral_limit = 200.0f / (speed_pid.ki + 0.1f);
                if (speed_pid.integral > integral_limit) speed_pid.integral = integral_limit;
                if (speed_pid.integral < -integral_limit) speed_pid.integral = -integral_limit;
                speed_pid.i_term = speed_pid.ki * speed_pid.integral;

                speed_pid.derivative = (speed_pid.error - speed_pid.last_error) * CONTROL_FREQUENCY;
                speed_pid.d_term = speed_pid.kd * speed_pid.derivative;

                speed_pid.output = speed_pid.p_term + speed_pid.i_term + speed_pid.d_term;

                // 输出限幅
                if (speed_pid.output > speed_pid.output_max) speed_pid.output = speed_pid.output_max;
                if (speed_pid.output < speed_pid.output_min) speed_pid.output = speed_pid.output_min;

                speed_pid.last_error = speed_pid.error;
            } else {
                speed_pid.output = 0;  // 禁用时输出为0
            }

            motor.pwm_output = speed_pid.output;

            // 设置电机方向和PWM
            float pwm_abs = fabs(speed_pid.output);

            // 死区补偿
            if (pwm_abs > 5.0f && pwm_abs < 100.0f) {
                pwm_abs = 100.0f;
            }

            if (speed_pid.output > 0) {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
            } else if (speed_pid.output < 0) {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
            } else {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
            }

            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)pwm_abs);
        }

        // 处理按键
        if (button_check_flag) {
            button_check_flag = 0;
            ProcessButtons();
        }

        /* USER CODE END WHILE */
    }
}

/* USER CODE BEGIN 4 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

// 定时器中断
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        control_flag = 1;
        sys_status.system_time++;

        // 数据发送（10Hz）
        static uint32_t data_counter = 0;
        data_counter++;
        if (data_counter >= 100) {
            data_counter = 0;
            SendDetailedData();
        }

        // 状态信息（每3秒）
        static uint32_t status_counter = 0;
        status_counter++;
        if (status_counter >= 3000) {
            status_counter = 0;
            SendStatusInfo();
        }

        // 按键检测（100Hz）
        static uint32_t button_counter = 0;
        button_counter++;
        if (button_counter >= 10) {
            button_counter = 0;
            button_check_flag = 1;
        }
    }
}
/* USER CODE END 4 */