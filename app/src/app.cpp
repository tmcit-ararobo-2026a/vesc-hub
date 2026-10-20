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

#define VESC_ID 45  // 43 or 45

// タイマー使用
volatile bool timer_1khz_triggered;
volatile bool timer_100hz_triggered;

// 状態管理用
enum class InitState {
    WaitForInit,
    FirstSetup,
    GetRotationRad,
    ZeroPointInitializing,
    InitializingPosition,
    Ready,
    GoToInitPoint,
} app_state;

// メイン基板との通信に使用
gn10_can::drivers::FDCANDriver fdcan1_driver(&hfdcan1);
gn10_can::FDCANBus fdcan1_bus(fdcan1_driver);
gn10_can::devices::LauncherServer launcher(fdcan1_bus, 0);

float target_vel_from_mainboard{};
float feedback_angular_velocity{};
float feedback_velocity{};
// VESCとのCAN通信
gn10_can::drivers::CANDriver can2_driver(&hfdcan2, FDCAN_RX_FIFO1, true);
VescCAN vesc(can2_driver);

// constants
constexpr float TARGET_ERPM_INIT        = 2500.0f;
constexpr float RELEASE_POINT_ROTATIONS = 9.0f;
constexpr float INITIAL_POINT_ROTATIONS = 5.85f;
constexpr float BELT_PITCH              = 0.005f;  // 5 mm -> m
constexpr float PULLEY_TEETH            = 24.0f;
constexpr float MOTOR_POLES             = 14.0f;
constexpr float ENCODER_SAMPLE_PERIOD   = 0.001f;  // [s]
constexpr float ROTATION_DIRECTION      = -1.0f;
constexpr float INIT_RETURN_MIN_ERPM    = 1000.0f;
constexpr float INIT_RETURN_MAX_ERPM    = 10000.0f;

// VESC関係
float target_erpm{};
float feedback_rpm{};

// エンコーダー関係
gn10_motor::IncrementalEncoder encoder(4000, &htim3);
float encoder_rad{};
float last_encoder_rad{};
float total_encoder_rad{};
float rotation_rad{};  // 一番最初のinit時のみ記録する一周分のrad

// ホールセンサ
bool magnet_near                   = false;
const float voltage_threshold_high = 2.1f;
const float voltage_threshold_low  = 1.8f;

// LED点滅
constexpr uint32_t HEARTBEAT_TOGGLE_INTERVAL_MS = 500;
uint32_t heartbeat_last_toggle_time_ms          = 0;

// その他動作補助
bool get_fire_command = false;
float process_count{};

/**  変換関数 **/

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

/**
 * @brief radから距離に変換
 */
float rad_to_distance(float rad)
{
    const float distance_per_rotation = BELT_PITCH * PULLEY_TEETH;
    return rad / (2.0f * M_PI) * distance_per_rotation;
}

/** コールバック関数 **/

void timer_1khz_process()
{
    int16_t encoder_count = encoder.read_and_reset_count();
    feedback_angular_velocity =
        encoder.count_to_angular_velocity(encoder_count, ENCODER_SAMPLE_PERIOD);
    feedback_velocity = angular_velocity_to_velocity(feedback_angular_velocity);
    encoder_rad       = encoder.accumulate_angle_rad(encoder_count);
    total_encoder_rad += (encoder_rad - last_encoder_rad);
    last_encoder_rad = encoder_rad;
}

void timer_100hz_process()
{
    // init処理が終わったら送信
    if (app_state != InitState::WaitForInit) {
        // launcher.send_velocity_feedback(feedback_velocity);
        vesc.comm_can_set_rpm(VESC_ID, target_erpm * ROTATION_DIRECTION);
    }
}

/** 他関数 **/

/**
 * @brief 初期位置へ戻る際の目標ERPMを計算する
 *        残り距離が大きいほど速く、近づくと最小ERPMに収束する
 * @param approach_time_s 時定数 [s]。大きいほどゆっくり減速する
 * @return 目標ERPM
 */
float send_init_erpm(float approach_time_s)
{
    const float pole_pairs = MOTOR_POLES / 2.0f;

    // 残り距離 [m](行き過ぎても負にならないようにする)
    const float remaining_rad = rotation_rad * process_count - total_encoder_rad;
    const float remaining     = std::max(rad_to_distance(remaining_rad), 0.0f);

    // 最小ERPMに相当する速度 [m/s]
    const float v_min = rpm_to_velocity(INIT_RETURN_MIN_ERPM / pole_pairs);

    const float velocity = v_min + remaining / approach_time_s;
    const float erpm     = velocity_to_rpm(velocity) * pole_pairs;
    return std::min(erpm, INIT_RETURN_MAX_ERPM);
}

/**
 * @brief Toggle heartbeat LED
 */
void update_heartbeat_led()
{
    const uint32_t now_ms = HAL_GetTick();
    if ((now_ms - heartbeat_last_toggle_time_ms) >= HEARTBEAT_TOGGLE_INTERVAL_MS) {
        heartbeat_last_toggle_time_ms = now_ms;
        HAL_GPIO_TogglePin(LED_4_GPIO_Port, LED_4_Pin);
    }
}

void setup()
{
    // 初期化待ちに設定
    app_state = InitState::WaitForInit;
    // CAN通信の開始
    fdcan1_driver.set_tx_timeout(5);
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
        if (!get_fire_command) {
            get_fire_command = true;
            process_count += 1.0f;
        }
        target_erpm = target_erpm_field::velocity_to_erpm(target_vel_from_mainboard);
    }

    if (launcher.get_init()) {
        if (app_state == InitState::WaitForInit) {
            // 一番最初のinitのみ
            app_state = InitState::FirstSetup;
        } else {
            app_state     = InitState::ZeroPointInitializing;
            process_count = 0.0f;
        }
    }

    if (app_state == InitState::FirstSetup) {
        // hole sensorまでsetup
        if (magnet_near) {
            target_erpm       = 0.0f;
            total_encoder_rad = 0.0f;
            app_state         = InitState::GetRotationRad;
            HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
        } else {
            target_erpm = TARGET_ERPM_INIT;
            HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
        }
    }

    if (app_state == InitState::GetRotationRad) {
        // 即時にホールセンサー反応しないように
        if (total_encoder_rad > rotate_to_rad(INITIAL_POINT_ROTATIONS)) {
            if (magnet_near) {
                rotation_rad      = total_encoder_rad;  // 一周分のradを記録
                target_erpm       = 0.0f;
                total_encoder_rad = 0.0f;
                HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
                app_state = InitState::InitializingPosition;
            }
        } else {
            target_erpm = TARGET_ERPM_INIT;
            HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
        }
    }

    // ホールセンサーまでのinit処理
    if (app_state == InitState::ZeroPointInitializing) {
        if (magnet_near) {
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
            HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
            app_state         = InitState::Ready;
            total_encoder_rad = 0.0f;
            target_erpm       = 0.0f;
            launcher.send_initial_point(total_encoder_rad);
            get_fire_command = false;
        } else {
            target_erpm = TARGET_ERPM_INIT;
            HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
        }
    }

    if (app_state == InitState::Ready) {
        // しきい値を超えたらモーターを止めて、終速を送る処理
        float add_rotation_rad;
        if (get_fire_command) {
            add_rotation_rad = abs(rotation_rad * (process_count - 1.0f));
        } else {
            add_rotation_rad = abs(rotation_rad * (process_count));
        }
        float release_rad = rotate_to_rad(RELEASE_POINT_ROTATIONS) + add_rotation_rad;
        if (total_encoder_rad > release_rad) {
            launcher.send_release_point(feedback_velocity);
            float release_erpm = feedback_rpm;
            launcher.send_velocity_feedback(release_erpm);
            HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);
            app_state = InitState::GoToInitPoint;
        } else {
            HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
        }
    }

    if (app_state == InitState::GoToInitPoint) {
        if (total_encoder_rad > (rotation_rad * static_cast<float>(process_count))) {
            target_erpm = 0.0f;
            launcher.send_initial_point(total_encoder_rad);
            get_fire_command = false;
            HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
            app_state = InitState::Ready;
        } else {
            HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
            target_erpm = send_init_erpm(1);
        }
    }

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
