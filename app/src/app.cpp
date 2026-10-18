#include "app/app.hpp"

#include "app/target_erpm.hpp"

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

#define VESC_ID 43  // 43 or 45

// タイマー使用
volatile bool timer_1khz_triggered;
volatile bool timer_100hz_triggered;

// 状態管理用
enum class InitState {
    WaitForInit,
    ZeroPointInitializing,
    InitializingPosition,
    Ready,
    GoToInitPlace,
    Reset_rad,
} app_state;

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
constexpr float TARGET_ERPM_INIT        = 2500.0f;
constexpr float RELEASE_POINT_ROTATIONS = 5.9f;
constexpr float TUEN_ROTATIONS          = 12.0f;
constexpr float INITIAL_POINT_ROTATIONS = 5.6f;
constexpr float BELT_PITCH              = 0.005f;  // 5 mm -> m
constexpr float PULLEY_TEETH            = 24.0f;
constexpr float MOTOR_POLES             = 14.0f;
constexpr float ENCODER_SAMPLE_PERIOD   = 0.001f;  // [s]
constexpr float ROTATION_DIRECTION      = -1.0f;

// VESC関係
float target_erpm = 0.0f;
float feedback_rpm{};
// エンコーダー関係
gn10_motor::IncrementalEncoder encoder(2000, &htim3);
volatile float total_encoder_rad = 0.0f;

// ホールセンサ
bool magnet_near             = false;
float voltage_threshold_high = 2.1f;
float voltage_threshold_low  = 1.8f;

// LED点滅
constexpr uint32_t HEARTBEAT_TOGGLE_INTERCAL_MS = 500;
uint32_t heartbeat_last_toggle_time_ms          = 0;

// setting function
void update_heartbeat_led();
void send_anglar_data(std::array<float, 4> send_data);

/**
 * @brief 回転数からradに変換
 */
float rotate_to_rad(float rotate)
{
    return rotate * M_PI * 2;
}

/**
 * @brief 速度からrpmに変換
 */
float velocity_to_rpm(float velocity)
{
    const float distance_per_rotation = BELT_PITCH * PULLEY_TEETH;
    return (velocity / distance_per_rotation) * 60.0f;
}

/**
 * @brief 角速度から速度に変換
 */
float angular_velocity_to_velocity(float angular_velocity)
{
    const float distance_per_rotation = BELT_PITCH * PULLEY_TEETH;
    return angular_velocity * distance_per_rotation / (2.0f * M_PI);
}

/**
 * @brief RPMから速度に変換
 */
float rpm_to_velocity(float rpm)
{
    const float distance_per_rotation = BELT_PITCH * PULLEY_TEETH;
    return (rpm / 60.0f) * distance_per_rotation;
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
    // init処理が終わったら送信
    if (app_state != InitState::WaitForInit) {
        // launcher.send_velocity_feedback(feedback_velocity);
        vesc.comm_can_set_rpm(VESC_ID, target_erpm * ROTATION_DIRECTION);
    }
}

/**
 * @brief 初期動作の目標ERPMを計算する
 * @param target_init_s 目標速度に到達するまでの時間 [s]
 * @param dt 制御周期 [s]
 * @return 目標ERPM
 */
float send_init_erpm(float target_init_s, float dt)
{
    const float distance_per_rotation = BELT_PITCH * PULLEY_TEETH;
    const float init_distance = (TUEN_ROTATIONS - RELEASE_POINT_ROTATIONS) * distance_per_rotation;
    const float target_velocity = init_distance / target_init_s;
    const float now_velocity    = rpm_to_velocity(feedback_rpm);
    const float acceleration    = (target_velocity - now_velocity) / target_init_s;
    const float next_velocity   = now_velocity + acceleration * dt;
    const float target_rpm      = velocity_to_rpm(next_velocity);
    const float target_erpm     = target_rpm * (MOTOR_POLES / 2.0f);
    return std::clamp(target_erpm, 2000.0f, 10000.0f);
}

void setup()
{
    // 初期化待ちに設定
    app_state = InitState::WaitForInit;
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
    // Tickを初期化
    heartbeat_last_toggle_time_ms = HAL_GetTick();
}

void loop()
{
    // vesc受信処理
    for (int i = 0; i < 8 && HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO1) > 0; i++) {
        vesc.update();
        feedback_rpm = vesc.get_rpm();
    }

    // ホールセンサーの設定
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    int32_t adc_val = HAL_ADC_GetValue(&hadc1);
    float voltage   = (float)adc_val / 4095.0f * 3.3f;

    // ホールセンサー反応処理
    if (voltage > voltage_threshold_high) {
        magnet_near = true;
    }
    if (voltage < voltage_threshold_low) {
        magnet_near = false;
    }

    // 司令を受信　＆　射出OKな場合のみtarget_rpmに目標値を代入。それ以外は0
    if (app_state == InitState::Ready && launcher.get_fire_command(target_vel_from_mainboard)) {
        target_erpm = target_erpm_field::velocity_to_erpm(target_vel_from_mainboard);
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
    }

    if (launcher.get_init()) {
        app_state = InitState::ZeroPointInitializing;
    }

    // ホールセンサーまでのinit処理
    if (app_state == InitState::ZeroPointInitializing) {
        if (magnet_near) {
            encoder.reset();
            total_encoder_rad = 0.0f;
            app_state         = InitState::InitializingPosition;
            HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
            target_erpm = TARGET_ERPM_INIT;
        }
    }

    // ホールセンサーから初期位置までのinit処理
    if (app_state == InitState::InitializingPosition) {
        if (total_encoder_rad > rotate_to_rad(INITIAL_POINT_ROTATIONS)) {
            app_state = InitState::Ready;
            launcher.send_initial_point(total_encoder_rad);
            target_erpm = 0.0f;
            encoder.reset();
            total_encoder_rad = 0.0f;
            HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
            target_erpm = TARGET_ERPM_INIT;
        }
    }

    if (app_state == InitState::Ready) {
        // しきい値を超えたらモーターを止めて、終速を送る処理
        if (total_encoder_rad > rotate_to_rad(RELEASE_POINT_ROTATIONS)) {
            launcher.send_release_point(feedback_velocity);

            float release_erpm = feedback_rpm;
            launcher.send_velocity_feedback(release_erpm);
            app_state = InitState::GoToInitPlace;
            HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);
        }
        HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
    }

    if (app_state == InitState::GoToInitPlace) {
        if (total_encoder_rad > rotate_to_rad(TUEN_ROTATIONS)) {
            target_erpm = 0.0f;
            encoder.reset();
            total_encoder_rad = 0.0f;
            launcher.send_initial_point(total_encoder_rad);
            app_state = InitState::Ready;
            HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
        } else {
            target_erpm = send_init_erpm(2, 0.01f);
        }
    }
    // launcher.send_velocity_feedback(static_cast<float>(app_state));
    launcher.send_velocity_feedback(total_encoder_rad);

    // send target
    if (timer_100hz_triggered) {
        timer_100hz_triggered = false;
        timer_100hz_process();
    }

    // send feedback
    if (timer_1khz_triggered) {
        timer_1khz_triggered = false;
        timer_1khz_process();
    }

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
        HAL_GPIO_TogglePin(LED_4_GPIO_Port, LED_4_Pin);
    }
}