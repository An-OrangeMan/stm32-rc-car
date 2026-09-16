#include "pid.h"
#include "encoder.h"
#include "motor.h"

/* ================= 二轮遥控小车控制 =================
 * 控制周期 10ms，由 TIM3 更新中断(100Hz)调用 Control()
 * 结构：蓝牙指令 -> 目标速度/转向 -> 速度环PI + 差速 -> 电机
 * ================================================== */

// 编码器计数（每个控制周期读取一次）
int Encoder_Left, Encoder_Right;

// 目标量与输出量
int Target_Speed = 0;    // 目标速度（左+右编码器计数之和 / 周期）
int Target_turn  = 0;    // 目标转向（差速 PWM 偏移）
int Speed_out, MOTO1, MOTO2;

/* ---- 速度环 PI 参数（需按实车调试）---- */
float Velocity_Kp = 40.0f;   // 比例：PWM / 编码器计数
float Velocity_Ki = 2.0f;    // 积分：PWM / (编码器计数·周期)
#define INTEGRAL_MAX  4000   // 速度环积分限幅

/* ---- 设定值 ---- */
#define SPEED_SET  60        // 前进/后退目标速度（左右编码器计数和/10ms）
#define TURN_DIFF  2500      // 转向差速 PWM 偏移（0~7200）

#define PWM_MAX 7200

extern TIM_HandleTypeDef htim2, htim4;
extern uint8_t Fore, Back, Left, Right;

/* 速度环 PI：目标速度 -> 平均 PWM */
static int Velocity_PI(int Target, int encoder_L, int encoder_R)
{
    static int Encoder_S = 0;               // 积分项
    int Err = Target - (encoder_L + encoder_R);   // 误差 = 目标 - 测量（负反馈）
    int out;

    Encoder_S += Err;
    if (Encoder_S >  INTEGRAL_MAX) Encoder_S =  INTEGRAL_MAX;
    if (Encoder_S < -INTEGRAL_MAX) Encoder_S = -INTEGRAL_MAX;

    out = (int)(Velocity_Kp * Err + Velocity_Ki * Encoder_S);
    if (out >  PWM_MAX) out =  PWM_MAX;
    if (out < -PWM_MAX) out = -PWM_MAX;
    return out;
}

/* 控制主函数：TIM3 每 10ms 调用一次 */
void Control(void)
{
    // 1. 读编码器（右轮镜像安装，取反对齐符号）
    Encoder_Left  = Read_Speed(&htim2);
    Encoder_Right = -Read_Speed(&htim4);

    // 2. 解析指令 -> 目标速度 / 转向
    if      (Fore == 1) Target_Speed =  SPEED_SET;
    else if (Back == 1) Target_Speed = -SPEED_SET;
    else                Target_Speed = 0;        // 无指令：PI 制动到 0

    if      (Left  == 1) Target_turn =  TURN_DIFF;
    else if (Right == 1) Target_turn = -TURN_DIFF;
    else                 Target_turn = 0;

    // 4. 速度环 PI -> 平均 PWM
    Speed_out = Velocity_PI(Target_Speed, Encoder_Left, Encoder_Right);

    // 5. 差速叠加：左轮减、右轮加（向左转 = 左慢右快）
    MOTO1 = Speed_out - Target_turn;
    MOTO2 = Speed_out + Target_turn;

    // 6. 限幅并输出到电机
    Limit(&MOTO1, &MOTO2);
    Load(MOTO1, MOTO2);
}
