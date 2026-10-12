#include "app/app.hpp"

#include <algorithm>
#include <cmath>

/* app */
#include "app/incremental_encoder.hpp"
#include "app/vesc_can.hpp"
/* gn10-can*/
#include "gn10_can/devices/launcher_server.hpp"
/* fdcan driver*/
#include "gn10_stm32_fdcan_driver/can_callback_helper.hpp"
#include "gn10_stm32_fdcan_driver/can_driver.hpp"
#include "gn10_stm32_fdcan_driver/fdcan_driver.hpp"
/* stm */
#include "adc.h"
#include "tim.h"

#define VESC_ID 45  // 43 or 45

// タイマー使用
volatile bool timer_1khz_triggered;
volatile bool timer_100hz_triggered;

// メイン基板との通信に使用
gn10_can::drivers::FDCANDriver fdcan1_driver(&hfdcan1);
gn10_can::FDCANBus fdcan1_bus(fdcan1_driver);
gn10_can::devices::LauncherServer launcher(fdcan1_bus, 0);

float target_vel_from_mainboard{};
volatile float feedback_angular_velocity{};
float feedback_velocity{};
bool start = false;

// VESCとのCAN通信
gn10_can::drivers::CANDriver can2_driver(&hfdcan2, FDCAN_RX_FIFO1, true);
VescCAN vesc(can2_driver);

// constants
constexpr float TARGET_DUTY_INIT        = 0.04f;
constexpr float KE                      = 0.0202f;  // 逆起電力定数[V*s/rad]
constexpr float OFFSET                  = 0.0f;     // 摩擦や抵抗の損失offset
constexpr float RELEASE_POINT_ROTATIONS = 11.5f;
constexpr float INITIAL_POINT_ROTATIONS = 5.8f;
constexpr float PULLEY_RADIUS           = 0.019f;  //[m]
constexpr float ENCODER_SAMPLE_PERIOD   = 0.001f;  // [s]
constexpr float ROTATION_DIRECTION      = -1.0f;

// 安全関係 (デバッグ中は小さめに。慣れてから上げる)
constexpr float VBUS_MIN          = 18.5f;  // これ以下は撃たない [V]
constexpr float DUTY_MAX          = 0.2f;   // デューティの上限
constexpr uint32_t RUN_TIMEOUT_MS = 3000;   // 指令後、この時間で自動停止

// VESC関係
float target_duty = 0.0f;

// エンコーダー関係
gn10_motor::IncrementalEncoder encoder(4095, &htim3);
volatile float total_encoder_rad = 0.0f;

// ホールセンサ
bool magnet_near             = false;
float voltage_threshold_high = 2.0f;
float voltage_threshold_low  = 1.8f;

// LED点滅
constexpr uint32_t HEARTBEAT_TOGGLE_INTERCAL_MS = 500;
uint32_t heartbeat_last_toggle_time_ms          = 0;

// setting function
void update_heartbeat_led();
void send_anglar_data(std::array<float, 4> send_data);

// duty control
volatile float vbus{};           // 電源電圧[V]。デバッグ時はデバッガから書き込む (将来CANから取得)
volatile bool debug_run = true;  // デバッガから true にした間だけ動く。false で常に停止
uint32_t run_start_ms   = 0;     // 指令を出した時刻 (自動停止用)

float v{};

/**
 * @brief 回転数からradに変換
 */
float rotate_to_rad(float rotate)
{
    return rotate * M_PI * 2;
}

/**
 * @brief 速度から電圧に変換
 */
float velocity_to_voltage(float velocity)
{
    return KE / PULLEY_RADIUS * velocity + OFFSET;
}

/**
 * @brief 電圧からdutyに変換。vbusが不正なら0を返す (0除算・全開の防止)
 */
float voltage_to_duty(float voltage, float vb)
{
    return (vb > VBUS_MIN) ? (voltage / vb) : 0.0f;
}

/**
 * @brief 角速度から速度に変換
 */
float angular_velocity_to_velocity(float angular_velocity)
{
    return angular_velocity * PULLEY_RADIUS;
}

void timer_1khz_process()
{
    int16_t encoder_count = encoder.read_and_reset_count();
    feedback_angular_velocity =
        encoder.count_to_angular_velocity(encoder_count, ENCODER_SAMPLE_PERIOD);
    feedback_velocity = angular_velocity_to_velocity(feedback_angular_velocity);
    total_encoder_rad = encoder.accumulate_angle_rad(encoder_count);
}

void timer_100hz_process()
{
    launcher.send_velocity_feedback(v);  // 校正中は混ざるのでOFF
    // 0でも送り続ける (VESC側のタイムアウトで止まるのを防ぎ、確実に0を指令する)
    // vesc.comm_can_set_duty(VESC_ID, target_duty * ROTATION_DIRECTION);
}

void setup()
{
    // CAN通信の開始
    fdcan1_driver.set_tx_timeout(2);
    fdcan1_driver.init();
    can2_driver.init();
    // Encoderの初期化
    encoder.hardware_init();
    // ADCのキャリブレーション
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    // タイマーを有効化
    HAL_TIM_Base_Start_IT(&htim7);
    HAL_TIM_Base_Start_IT(&htim6);
    while (!launcher.get_init()) {
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
    }

    // Tickを初期化
    heartbeat_last_toggle_time_ms = HAL_GetTick();
}

void loop()
{
    if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO1) > 0) {
        vesc.update();
        HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET);
        vbus = vesc.get_voltage();
    }
    float target_voltage = velocity_to_voltage(12.0f);
    target_duty          = voltage_to_duty(target_voltage, vbus);
    vesc.comm_can_set_duty(VESC_ID, target_duty);
    launcher.send_velocity_feedback(vbus);

    update_heartbeat_led();
}

// CAN Receive CAllback
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
    (void)RxFifo0ITs;
    if (process_fdcan_fifo(hfdcan, &hfdcan1, fdcan1_bus, FDCAN_RX_FIFO0)) return;
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo1ITs)
{
    (void)RxFifo1ITs;
    if (process_fdcan_fifo(hfdcan, &hfdcan2, vesc, FDCAN_RX_FIFO1)) return;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    // 100Hz
    if (htim->Instance == TIM6) {
        timer_100hz_triggered = true;
    }

    // 1kHz
    if (htim->Instance == TIM7) {
        timer_1khz_triggered = true;
    }
}

/**
 * @brief Toggle heartbeat LED
 */
void update_heartbeat_led()
{
    const uint32_t now_ms = HAL_GetTick();
    if ((now_ms - heartbeat_last_toggle_time_ms) >= HEARTBEAT_TOGGLE_INTERCAL_MS) {
        heartbeat_last_toggle_time_ms = now_ms;
        HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
    }
}