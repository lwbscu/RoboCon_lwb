#include "main.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* Private variables */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;  // 新增：主电机编码器
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

// 主电机结构体（新增）
typedef struct {
    int32_t encoder_count;
    int32_t last_encoder;
    int32_t position_encoder_total;     // 位置累计编码器计数
    float encoder_rps;                  // 编码器转速（圈/秒）
    float wheel_rps;                    // 车轮转速（圈/秒）
    float wheel_position;               // 车轮位置（圈）
    float current_position;             // 当前位置（圈数）
} MasterMotor_t;

// 从电机控制结构体
typedef struct {
    volatile float target_speed;        // 目标速度：车轮转速（圈/秒）
    volatile float target_position;     // 目标位置：车轮圈数
    float current_speed;                // 当前速度：车轮转速（圈/秒）
    float current_position;             // 当前位置：车轮圈数
    int32_t encoder_count;
    int32_t last_encoder;
    int32_t position_encoder_total;     // 位置累计编码器计数
    float encoder_rps;                  // 编码器转速（圈/秒）
    float wheel_rps;                    // 车轮转速（圈/秒）
    float wheel_position;               // 车轮位置（圈）
    float pwm_output;
    uint8_t direction;
    uint8_t speed_mode;                 // 速度模式：0,1,2
    uint8_t position_mode;              // 位置模式：0,1,2
    uint8_t enable;
    volatile uint8_t control_mode;      // 0=按键控制, 1=VOFA+控制
    volatile uint8_t control_type;      // 0=速度控制, 1=位置控制, 2=串级控制, 3=位置跟随控制
} SlaveMotor_t;

// 系统状态结构体
typedef struct {
    uint8_t pid_tuning_mode;
    uint8_t data_send_mode;
    uint32_t system_time;
} System_t;

// 全局变量
MasterMotor_t master_motor = {0};    // 新增：主电机
SlaveMotor_t slave_motor = {0};      // 重命名：从电机
PID_t speed_pid = {0};               // 速度PID
PID_t position_pid = {0};            // 位置PID
PID_t follow_pid = {0};              // 新增：跟随PID
System_t sys_status = {0};

volatile uint8_t control_flag = 0;
volatile uint8_t button_check_flag = 0;

// 参数定义
#define ENCODER_PPR 500.0f          // 编码器每圈脉冲数
#define CONTROL_FREQUENCY 1000      // 控制频率
#define WHEEL_CONVERSION 0.0085f    // 换算系数：车轮转速 = 编码器转速 × 0.0085
#define PWM_MAX_VALUE 1000          // PWM最大值

// 预设的速度和位置档位
const float SPEED_LEVELS[3] = {0.5f, 1.0f, 2.0f};      // 车轮转速（圈/秒）
const float POSITION_LEVELS[3] = {1.0f, 10.0f, 50.0f}; // 车轮位置（圈）

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

    // 跟随PID参数（新增）
    if (strstr(cmd, "follow_pid:kp,") != NULL) {
        char *start = strstr(cmd, "follow_pid:kp,") + 14;
        float val = atof(start);
        float old_val = follow_pid.kp;
        follow_pid.kp = val;
        printf("FOLLOW_KP: %.2f -> %.2f (SUCCESS)\n", old_val, follow_pid.kp);
    }
    else if (strstr(cmd, "follow_pid:ki,") != NULL) {
        char *start = strstr(cmd, "follow_pid:ki,") + 14;
        float val = atof(start);
        float old_val = follow_pid.ki;
        follow_pid.ki = val;
        printf("FOLLOW_KI: %.2f -> %.2f (SUCCESS)\n", old_val, follow_pid.ki);
    }
    else if (strstr(cmd, "follow_pid:kd,") != NULL) {
        char *start = strstr(cmd, "follow_pid:kd,") + 14;
        float val = atof(start);
        float old_val = follow_pid.kd;
        follow_pid.kd = val;
        printf("FOLLOW_KD: %.2f -> %.2f (SUCCESS)\n", old_val, follow_pid.kd);
    }
        // 位置PID参数 - 必须放在前面，优先匹配更具体的命令
    else if (strstr(cmd, "pos_pid:kp,") != NULL) {
        char *start = strstr(cmd, "pos_pid:kp,") + 11;
        float val = atof(start);
        float old_val = position_pid.kp;
        position_pid.kp = val;
        printf("POSITION_KP: %.2f -> %.2f (SUCCESS)\n", old_val, position_pid.kp);
    }
    else if (strstr(cmd, "pos_pid:ki,") != NULL) {
        char *start = strstr(cmd, "pos_pid:ki,") + 11;
        float val = atof(start);
        float old_val = position_pid.ki;
        position_pid.ki = val;
        printf("POSITION_KI: %.2f -> %.2f (SUCCESS)\n", old_val, position_pid.ki);
    }
    else if (strstr(cmd, "pos_pid:kd,") != NULL) {
        char *start = strstr(cmd, "pos_pid:kd,") + 11;
        float val = atof(start);
        float old_val = position_pid.kd;
        position_pid.kd = val;
        printf("POSITION_KD: %.2f -> %.2f (SUCCESS)\n", old_val, position_pid.kd);
    }
        // 速度PID参数 - 放在后面，只有位置PID不匹配时才检查
    else if (strstr(cmd, "pid:kp,") != NULL) {
        char *start = strstr(cmd, "pid:kp,") + 7;
        float val = atof(start);
        float old_val = speed_pid.kp;
        speed_pid.kp = val;
        printf("SPEED_KP: %.2f -> %.2f (SUCCESS)\n", old_val, speed_pid.kp);
    }
    else if (strstr(cmd, "pid:ki,") != NULL) {
        char *start = strstr(cmd, "pid:ki,") + 7;
        float val = atof(start);
        float old_val = speed_pid.ki;
        speed_pid.ki = val;
        printf("SPEED_KI: %.2f -> %.2f (SUCCESS)\n", old_val, speed_pid.ki);
    }
    else if (strstr(cmd, "pid:kd,") != NULL) {
        char *start = strstr(cmd, "pid:kd,") + 7;
        float val = atof(start);
        float old_val = speed_pid.kd;
        speed_pid.kd = val;
        printf("SPEED_KD: %.2f -> %.2f (SUCCESS)\n", old_val, speed_pid.kd);
    }
        // 速度控制
    else if (strstr(cmd, "speed:") != NULL) {
        char *start = strstr(cmd, "speed:") + 6;
        float val = atof(start);
        slave_motor.target_speed = val;
        slave_motor.control_mode = 1;
        slave_motor.control_type = 0;  // 速度控制模式
        printf("SPEED_MODE: TARGET=%.2f WHEEL_RPS\n", val);
    }
        // 位置控制
    else if (strstr(cmd, "position:") != NULL) {
        char *start = strstr(cmd, "position:") + 9;
        float val = atof(start);
        slave_motor.target_position = val;
        slave_motor.control_mode = 1;
        slave_motor.control_type = 1;  // 位置控制模式
        // 重置位置累计
        slave_motor.position_encoder_total = 0;
        slave_motor.current_position = 0;
        printf("POSITION_MODE: TARGET=%.2f WHEEL_TURNS\n", val);
    }
        // 串级控制
    else if (strstr(cmd, "cascade:") != NULL) {
        char *start = strstr(cmd, "cascade:") + 8;
        char temp_str[32];
        strncpy(temp_str, start, 31);
        temp_str[31] = '\0';

        char *comma_pos = strchr(temp_str, ',');
        if (comma_pos != NULL) {
            *comma_pos = '\0';
            float pos_val = atof(temp_str);
            float speed_val = atof(comma_pos + 1);

            slave_motor.target_position = pos_val;
            slave_motor.target_speed = speed_val;
            slave_motor.control_mode = 1;
            slave_motor.control_type = 2;  // 串级控制模式
            // 重置位置累计
            slave_motor.position_encoder_total = 0;
            slave_motor.current_position = 0;
            printf("CASCADE_MODE: POS=%.2f, SPEED=%.2f\n", pos_val, speed_val);
        }
    }
        // 位置跟随控制（新增）
    else if (strstr(cmd, "follow") != NULL) {
        slave_motor.control_mode = 1;
        slave_motor.control_type = 3;  // 位置跟随控制模式
        // 重置位置累计
        slave_motor.position_encoder_total = 0;
        slave_motor.current_position = 0;
        master_motor.position_encoder_total = 0;
        master_motor.current_position = 0;
        printf("FOLLOW_MODE: SLAVE FOLLOWS MASTER\n");
    }
        // 方向控制
    else if (strstr(cmd, "dir:") != NULL) {
        char *start = strstr(cmd, "dir:") + 4;
        int val = atoi(start);
        slave_motor.direction = (val > 0) ? 1 : 0;
        printf("DIRECTION=%s\n", slave_motor.direction ? "REVERSE" : "FORWARD");
    }
        // 使能控制
    else if (strstr(cmd, "enable:") != NULL) {
        char *start = strstr(cmd, "enable:") + 7;
        int val = atoi(start);
        slave_motor.enable = val;
        printf("MOTOR_ENABLE=%s\n", val ? "ON" : "OFF");
    }
        // 控制模式 (0=按键控制, 1=VOFA+控制)
    else if (strstr(cmd, "control_mode:") != NULL) {
        char *start = strstr(cmd, "control_mode:") + 13;
        int val = atoi(start);
        slave_motor.control_mode = (val > 0) ? 1 : 0;
        printf("CONTROL_MODE=%s\n", slave_motor.control_mode ? "VOFA+" : "BUTTON");
    }
        // 控制类型 (0=速度, 1=位置, 2=串级, 3=跟随)
    else if (strstr(cmd, "control_type:") != NULL) {
        char *start = strstr(cmd, "control_type:") + 13;
        int val = atoi(start);
        if (val >= 0 && val <= 3) {
            slave_motor.control_type = val;
            // 重置位置累计
            if (slave_motor.control_type != 0) {
                slave_motor.position_encoder_total = 0;
                slave_motor.current_position = 0;
                if (slave_motor.control_type == 3) {
                    master_motor.position_encoder_total = 0;
                    master_motor.current_position = 0;
                }
            }
            printf("CONTROL_TYPE=%s\n",
                   slave_motor.control_type == 0 ? "SPEED" :
                   slave_motor.control_type == 1 ? "POSITION" :
                   slave_motor.control_type == 2 ? "CASCADE" : "FOLLOW");
        }
    }
        // 速度模式切换 (0=0.5rps, 1=1.0rps, 2=2.0rps)
    else if (strstr(cmd, "speed_mode:") != NULL) {
        char *start = strstr(cmd, "speed_mode:") + 11;
        int val = atoi(start);
        if (val >= 0 && val <= 2) {
            slave_motor.speed_mode = val;
            slave_motor.target_speed = SPEED_LEVELS[slave_motor.speed_mode];
            slave_motor.control_type = 0;  // 自动切换到速度控制
            printf("SPEED_MODE_%d: %.1f RPS\n", slave_motor.speed_mode, slave_motor.target_speed);
        }
    }
        // 位置模式切换 (0=1圈, 1=10圈, 2=50圈)
    else if (strstr(cmd, "position_mode:") != NULL) {
        char *start = strstr(cmd, "position_mode:") + 14;
        int val = atoi(start);
        if (val >= 0 && val <= 2) {
            slave_motor.position_mode = val;
            slave_motor.target_position = POSITION_LEVELS[slave_motor.position_mode];
            slave_motor.control_type = 1;  // 自动切换到位置控制
            slave_motor.position_encoder_total = 0;
            slave_motor.current_position = 0;
            printf("POSITION_MODE_%d: %.1f TURNS\n", slave_motor.position_mode, slave_motor.target_position);
        }
    }
        // 串级模式切换 (0=低速短距, 1=中速中距, 2=高速长距)
    else if (strstr(cmd, "cascade_mode:") != NULL) {
        char *start = strstr(cmd, "cascade_mode:") + 13;
        int val = atoi(start);
        if (val >= 0 && val <= 2) {
            slave_motor.speed_mode = val;
            slave_motor.position_mode = val;
            slave_motor.target_speed = SPEED_LEVELS[slave_motor.speed_mode];
            slave_motor.target_position = POSITION_LEVELS[slave_motor.position_mode];
            slave_motor.control_type = 2;  // 自动切换到串级控制
            slave_motor.position_encoder_total = 0;
            slave_motor.current_position = 0;
            printf("CASCADE_MODE_%d: SPEED=%.1f, POS=%.1f\n",
                   val, slave_motor.target_speed, slave_motor.target_position);
        }
    }
        // 复位
    else if (strstr(cmd, "reset") != NULL) {
        speed_pid.integral = 0;
        speed_pid.last_error = 0;
        position_pid.integral = 0;
        position_pid.last_error = 0;
        follow_pid.integral = 0;
        follow_pid.last_error = 0;
        slave_motor.position_encoder_total = 0;
        slave_motor.current_position = 0;
        master_motor.position_encoder_total = 0;
        master_motor.current_position = 0;
        printf("PID_RESET_AND_POSITION_RESET\n");
    }
        // 信息查询
    else if (strstr(cmd, "info") != NULL) {
        printf("=== SYSTEM INFO ===\n");
        printf("ENCODER_PPR=%.0f\n", ENCODER_PPR);
        printf("WHEEL_CONVERSION=%.4f\n", WHEEL_CONVERSION);
        printf("CONTROL_MODE=%s (%d)\n", slave_motor.control_mode ? "VOFA+" : "BUTTON", slave_motor.control_mode);
        printf("CONTROL_TYPE=%s (%d)\n",
               slave_motor.control_type == 0 ? "SPEED" :
               slave_motor.control_type == 1 ? "POSITION" :
               slave_motor.control_type == 2 ? "CASCADE" : "FOLLOW", slave_motor.control_type);
        printf("SPEED_MODE=%d (%.1f rps)\n", slave_motor.speed_mode, SPEED_LEVELS[slave_motor.speed_mode]);
        printf("POSITION_MODE=%d (%.1f turns)\n", slave_motor.position_mode, POSITION_LEVELS[slave_motor.position_mode]);
        printf("DIRECTION=%s (%d)\n", slave_motor.direction ? "REVERSE" : "FORWARD", slave_motor.direction);
        printf("ENABLE=%s (%d)\n", slave_motor.enable ? "ON" : "OFF", slave_motor.enable);
        printf("SLAVE_SPEED=%.3f, SLAVE_POSITION=%.3f\n",
               slave_motor.current_speed, slave_motor.current_position);
        printf("MASTER_POSITION=%.3f\n", master_motor.current_position);
        printf("SPEED_PID: Kp=%.2f Ki=%.2f Kd=%.2f\n", speed_pid.kp, speed_pid.ki, speed_pid.kd);
        printf("POS_PID: Kp=%.2f Ki=%.2f Kd=%.2f\n", position_pid.kp, position_pid.ki, position_pid.kd);
        printf("FOLLOW_PID: Kp=%.2f Ki=%.2f Kd=%.2f\n", follow_pid.kp, follow_pid.ki, follow_pid.kd);
    }
}

// 发送详细的调试数据 (VOFA+兼容格式) - 16通道
void SendDetailedData(void) {
    printf("%.3f,%.3f,%.3f,%.3f,%.1f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%.3f\n",
           slave_motor.target_speed,      // CH0: 目标速度（车轮圈/秒）
           slave_motor.current_speed,     // CH1: 当前速度（车轮圈/秒）
           slave_motor.target_position,   // CH2: 目标位置（车轮圈）
           slave_motor.current_position,  // CH3: 当前位置（车轮圈）
           slave_motor.pwm_output,        // CH4: PWM输出
           speed_pid.error,               // CH5: 速度PID误差
           position_pid.error,            // CH6: 位置PID误差
           speed_pid.kp,                  // CH7: 速度Kp参数
           speed_pid.ki,                  // CH8: 速度Ki参数
           speed_pid.kd,                  // CH9: 速度Kd参数
           position_pid.kp,               // CH10: 位置Kp参数
           position_pid.ki,               // CH11: 位置Ki参数
           position_pid.kd,               // CH12: 位置Kd参数
           slave_motor.control_type,      // CH13: 控制类型
           slave_motor.direction,         // CH14: 方向
           master_motor.current_position); // CH15: 主电机位置（新增）
}

// 发送系统状态信息
void SendStatusInfo(void) {
    const char* control_types[] = {"SPEED", "POSITION", "CASCADE", "FOLLOW"};
    printf("=== STATUS ===\n");
    printf("CONTROL: %s | TARGET_SPEED: %.3f | CURRENT_SPEED: %.3f\n",
           control_types[slave_motor.control_type], slave_motor.target_speed, slave_motor.current_speed);
    printf("TARGET_POS: %.3f | SLAVE_POS: %.3f | MASTER_POS: %.3f | PWM: %.0f\n",
           slave_motor.target_position, slave_motor.current_position, master_motor.current_position, slave_motor.pwm_output);
    printf("SPEED_PID[Kp=%.2f Ki=%.2f Kd=%.2f]\n",
           speed_pid.kp, speed_pid.ki, speed_pid.kd);
    printf("POSITION_PID[Kp=%.2f Ki=%.2f Kd=%.2f]\n",
           position_pid.kp, position_pid.ki, position_pid.kd);
    printf("FOLLOW_PID[Kp=%.2f Ki=%.2f Kd=%.2f]\n",
           follow_pid.kp, follow_pid.ki, follow_pid.kd);
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

    // 按键模式下才响应按键
    if (slave_motor.control_mode == 0) {
        // 模式按键 - 双击切换控制类型，单击切换档位
        static uint32_t last_mode_press = 0;
        static uint8_t click_count = 0;

        if (button_mode.current_state && !button_mode.last_state) {
            uint32_t time_since_last = current_time - last_mode_press;
            if (time_since_last < 500) {
                click_count++;
            } else {
                click_count = 1;
            }
            last_mode_press = current_time;
        }

        // 检查双击
        if (click_count >= 2 && (current_time - last_mode_press) > 100) {
            slave_motor.control_type = (slave_motor.control_type + 1) % 4;  // 修改：支持4种控制类型
            click_count = 0;

            // 重置位置累计
            if (slave_motor.control_type != 0) {
                slave_motor.position_encoder_total = 0;
                slave_motor.current_position = 0;
                if (slave_motor.control_type == 3) {
                    master_motor.position_encoder_total = 0;
                    master_motor.current_position = 0;
                }
            }

            printf("BUTTON_CONTROL_TYPE: %s\n",
                   slave_motor.control_type == 0 ? "SPEED" :
                   slave_motor.control_type == 1 ? "POSITION" :
                   slave_motor.control_type == 2 ? "CASCADE" : "FOLLOW");
        }
            // 检查单击
        else if (click_count == 1 && (current_time - last_mode_press) > 500) {
            if (slave_motor.control_type == 0) {
                // 速度控制模式
                slave_motor.speed_mode = (slave_motor.speed_mode + 1) % 3;
                slave_motor.target_speed = SPEED_LEVELS[slave_motor.speed_mode];
                printf("SPEED_MODE_%d: %.1f RPS\n", slave_motor.speed_mode, slave_motor.target_speed);
            } else if (slave_motor.control_type == 1) {
                // 位置控制模式
                slave_motor.position_mode = (slave_motor.position_mode + 1) % 3;
                slave_motor.target_position = POSITION_LEVELS[slave_motor.position_mode];
                slave_motor.position_encoder_total = 0;
                slave_motor.current_position = 0;
                printf("POSITION_MODE_%d: %.1f TURNS\n", slave_motor.position_mode, slave_motor.target_position);
            } else if (slave_motor.control_type == 2) {
                // 串级控制模式
                slave_motor.speed_mode = (slave_motor.speed_mode + 1) % 3;
                slave_motor.position_mode = (slave_motor.position_mode + 1) % 3;
                slave_motor.target_speed = SPEED_LEVELS[slave_motor.speed_mode];
                slave_motor.target_position = POSITION_LEVELS[slave_motor.position_mode];
                slave_motor.position_encoder_total = 0;
                slave_motor.current_position = 0;
                printf("CASCADE_MODE_%d: SPEED=%.1f, POS=%.1f\n",
                       slave_motor.speed_mode, slave_motor.target_speed, slave_motor.target_position);
            } else if (slave_motor.control_type == 3) {
                // 跟随控制模式 - 重置位置
                slave_motor.position_encoder_total = 0;
                slave_motor.current_position = 0;
                master_motor.position_encoder_total = 0;
                master_motor.current_position = 0;
                printf("FOLLOW_MODE_RESET\n");
            }
            click_count = 0;
        }

        // 正转按键
        if (button_forward.current_state && !button_forward.last_state) {
            slave_motor.direction = 0;
            printf("FORWARD\n");
        }

        // 反转按键
        if (button_reverse.current_state && !button_reverse.last_state) {
            slave_motor.direction = 1;
            printf("REVERSE\n");
        }
    }

    button_mode.last_state = button_mode.current_state;
    button_forward.last_state = button_forward.current_state;
    button_reverse.last_state = button_reverse.current_state;
}

// 速度PID控制
float SpeedPIDControl(float target_speed, float current_speed) {
    speed_pid.error = target_speed - current_speed;

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

    return speed_pid.output;
}

// 位置PID控制
float PositionPIDControl(float target_position, float current_position) {
    position_pid.error = target_position - current_position;

    position_pid.p_term = position_pid.kp * position_pid.error;

    position_pid.integral += position_pid.error / CONTROL_FREQUENCY;
    // 积分限幅
    float integral_limit = 50.0f / (position_pid.ki + 0.1f);
    if (position_pid.integral > integral_limit) position_pid.integral = integral_limit;
    if (position_pid.integral < -integral_limit) position_pid.integral = -integral_limit;
    position_pid.i_term = position_pid.ki * position_pid.integral;

    position_pid.derivative = (position_pid.error - position_pid.last_error) * CONTROL_FREQUENCY;
    position_pid.d_term = position_pid.kd * position_pid.derivative;

    position_pid.output = position_pid.p_term + position_pid.i_term + position_pid.d_term;

    // 位置PID输出限制在合理的速度范围
    if (position_pid.output > 3.0f) position_pid.output = 3.0f;
    if (position_pid.output < -3.0f) position_pid.output = -3.0f;

    position_pid.last_error = position_pid.error;

    return position_pid.output;
}

// 跟随PID控制（新增）
float FollowPIDControl(float master_position, float slave_position) {
    follow_pid.error = master_position - slave_position;

    follow_pid.p_term = follow_pid.kp * follow_pid.error;

    follow_pid.integral += follow_pid.error / CONTROL_FREQUENCY;
    // 积分限幅
    float integral_limit = 50.0f / (follow_pid.ki + 0.1f);
    if (follow_pid.integral > integral_limit) follow_pid.integral = integral_limit;
    if (follow_pid.integral < -integral_limit) follow_pid.integral = -integral_limit;
    follow_pid.i_term = follow_pid.ki * follow_pid.integral;

    follow_pid.derivative = (follow_pid.error - follow_pid.last_error) * CONTROL_FREQUENCY;
    follow_pid.d_term = follow_pid.kd * follow_pid.derivative;

    follow_pid.output = follow_pid.p_term + follow_pid.i_term + follow_pid.d_term;

    // 跟随PID输出限制在合理的速度范围
    if (follow_pid.output > 3.0f) follow_pid.output = 3.0f;
    if (follow_pid.output < -3.0f) follow_pid.output = -3.0f;

    follow_pid.last_error = follow_pid.error;

    return follow_pid.output;
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
    MX_TIM4_Init();  // 新增：主电机编码器
    MX_USART2_UART_Init();

    // 初始化TB6612
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);  // 修正：使能TB6612（PB0）
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

    // 初始化速度PID参数
    speed_pid.kp = 10.0f;
    speed_pid.ki = 0.5f;
    speed_pid.kd = 0.1f;
    speed_pid.output_max = PWM_MAX_VALUE * 0.9f;
    speed_pid.output_min = -PWM_MAX_VALUE * 0.9f;

    // 初始化位置PID参数
    position_pid.kp = 2.0f;
    position_pid.ki = 0.1f;
    position_pid.kd = 0.05f;
    position_pid.output_max = 3.0f;
    position_pid.output_min = -3.0f;

    // 初始化跟随PID参数（新增）
    follow_pid.kp = 5.0f;
    follow_pid.ki = 0.2f;
    follow_pid.kd = 0.1f;
    follow_pid.output_max = 3.0f;
    follow_pid.output_min = -3.0f;

    // 初始化电机参数
    slave_motor.target_speed = SPEED_LEVELS[1];        // 默认1圈/秒
    slave_motor.target_position = POSITION_LEVELS[1];  // 默认10圈
    slave_motor.enable = 1;
    slave_motor.speed_mode = 1;
    slave_motor.position_mode = 1;
    slave_motor.direction = 0;
    slave_motor.control_mode = 1;  // 默认VOFA+控制模式
    slave_motor.control_type = 0;  // 默认速度控制

    // 初始化系统状态
    sys_status.data_send_mode = 1;
    sys_status.pid_tuning_mode = 1;

    // 启动定时器
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);  // 从电机编码器
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);  // 新增：主电机编码器
    HAL_TIM_Base_Start_IT(&htim3);

    // 启动串口接收中断
    printf("=== OPTIMIZED MOTOR CONTROL SYSTEM WITH FOLLOW MODE ===\n");
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    printf("UART_READY\n");

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);

    printf("ENCODER_PPR=%.0f, CONVERSION=%.4f\n", ENCODER_PPR, WHEEL_CONVERSION);
    printf("CONTROL_MODES: 0=SPEED, 1=POSITION, 2=CASCADE, 3=FOLLOW\n");
    printf("SPEED_LEVELS: 0=%.1frps, 1=%.1frps, 2=%.1frps\n",
           SPEED_LEVELS[0], SPEED_LEVELS[1], SPEED_LEVELS[2]);
    printf("POSITION_LEVELS: 0=%.0fturns, 1=%.0fturns, 2=%.0fturns\n",
           POSITION_LEVELS[0], POSITION_LEVELS[1], POSITION_LEVELS[2]);
    printf("DEFAULT: SPEED_MODE_%d=%.2f RPS\n", slave_motor.speed_mode, slave_motor.target_speed);
    printf("NEW: FOLLOW_MODE ADDED - SLAVE FOLLOWS MASTER POSITION\n");
    printf("USE INTEGER PARAMETERS FOR MODE CONTROL!\n");
    /* USER CODE END 2 */

    while (1)
    {
        /* USER CODE BEGIN WHILE */

        // 处理串口命令
        if (cmd_ready) {
            cmd_ready = 0;
            TestCommandProcess();
        }

        // 主控制逻辑
        if (control_flag) {
            control_flag = 0;

            // 读取从电机编码器
            slave_motor.encoder_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
            int32_t slave_encoder_diff = slave_motor.encoder_count - slave_motor.last_encoder;
            slave_motor.last_encoder = slave_motor.encoder_count;

            // 处理从电机编码器溢出
            if (slave_encoder_diff > 32768) slave_encoder_diff -= 65536;
            if (slave_encoder_diff < -32768) slave_encoder_diff += 65536;

            // 累计从电机位置编码器计数
            slave_motor.position_encoder_total += slave_encoder_diff;

            // 计算从电机编码器转速（圈/秒）
            slave_motor.encoder_rps = (float)slave_encoder_diff * CONTROL_FREQUENCY / ENCODER_PPR;

            // 转换为从电机车轮转速和位置
            slave_motor.wheel_rps = slave_motor.encoder_rps * WHEEL_CONVERSION;
            slave_motor.current_speed = fabs(slave_motor.wheel_rps);
            slave_motor.wheel_position = (float)slave_motor.position_encoder_total * WHEEL_CONVERSION / ENCODER_PPR;
            slave_motor.current_position = fabs(slave_motor.wheel_position);

            // 读取主电机编码器（新增）
            master_motor.encoder_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
            int32_t master_encoder_diff = master_motor.encoder_count - master_motor.last_encoder;
            master_motor.last_encoder = master_motor.encoder_count;

            // 处理主电机编码器溢出
            if (master_encoder_diff > 32768) master_encoder_diff -= 65536;
            if (master_encoder_diff < -32768) master_encoder_diff += 65536;

            // 累计主电机位置编码器计数
            master_motor.position_encoder_total += master_encoder_diff;

            // 计算主电机编码器转速（圈/秒）
            master_motor.encoder_rps = (float)master_encoder_diff * CONTROL_FREQUENCY / ENCODER_PPR;

            // 转换为主电机车轮转速和位置
            master_motor.wheel_rps = master_motor.encoder_rps * WHEEL_CONVERSION;
            master_motor.wheel_position = (float)master_motor.position_encoder_total * WHEEL_CONVERSION / ENCODER_PPR;
            master_motor.current_position = fabs(master_motor.wheel_position);

            // 控制逻辑
            if (slave_motor.enable) {
                float target_encoder_rps = 0;

                switch (slave_motor.control_type) {
                    case 0: // 速度控制模式
                        target_encoder_rps = slave_motor.target_speed / WHEEL_CONVERSION;
                        if (slave_motor.direction == 1) target_encoder_rps = -target_encoder_rps;
                        slave_motor.pwm_output = SpeedPIDControl(target_encoder_rps, slave_motor.encoder_rps);
                        break;

                    case 1: // 位置控制模式
                    {
                        float target_wheel_position = slave_motor.target_position;
                        if (slave_motor.direction == 1) target_wheel_position = -target_wheel_position;
                        float position_speed_output = PositionPIDControl(target_wheel_position, slave_motor.wheel_position);
                        target_encoder_rps = position_speed_output / WHEEL_CONVERSION;
                        slave_motor.pwm_output = SpeedPIDControl(target_encoder_rps, slave_motor.encoder_rps);
                    }
                        break;

                    case 2: // 串级控制模式
                    {
                        float target_wheel_position = slave_motor.target_position;
                        if (slave_motor.direction == 1) target_wheel_position = -target_wheel_position;
                        float position_speed_output = PositionPIDControl(target_wheel_position, slave_motor.wheel_position);

                        // 限制位置环输出的速度不超过设定最大速度
                        if (fabs(position_speed_output) > slave_motor.target_speed) {
                            position_speed_output = (position_speed_output > 0) ?
                                                    slave_motor.target_speed : -slave_motor.target_speed;
                        }

                        target_encoder_rps = position_speed_output / WHEEL_CONVERSION;
                        slave_motor.pwm_output = SpeedPIDControl(target_encoder_rps, slave_motor.encoder_rps);
                    }
                        break;

                    case 3: // 位置跟随控制模式（新增）
                    {
                        // 使用跟随PID控制器，从电机跟随主电机位置
                        float follow_speed_output = FollowPIDControl(master_motor.wheel_position, slave_motor.wheel_position);
                        target_encoder_rps = follow_speed_output / WHEEL_CONVERSION;
                        slave_motor.pwm_output = SpeedPIDControl(target_encoder_rps, slave_motor.encoder_rps);

                        // 更新目标位置为主电机当前位置（用于显示）
                        slave_motor.target_position = master_motor.current_position;
                    }
                        break;
                }
            } else {
                slave_motor.pwm_output = 0;
            }

            // 设置电机方向和PWM
            float pwm_abs = fabs(slave_motor.pwm_output);

            // 死区补偿
            if (pwm_abs > 5.0f && pwm_abs < 100.0f) {
                pwm_abs = 100.0f;
            }

            if (slave_motor.pwm_output > 0) {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
            } else if (slave_motor.pwm_output < 0) {
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

        // 状态信息（每5秒）
        static uint32_t status_counter = 0;
        status_counter++;
        if (status_counter >= 5000) {
            status_counter = 0;
            SendStatusInfo();
        }

        // 按键检测（50Hz）
        static uint32_t button_counter = 0;
        button_counter++;
        if (button_counter >= 20) {
            button_counter = 0;
            button_check_flag = 1;
        }
    }
}
/* USER CODE END 4 */