#include "app/app.hpp"

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
    ZeroPointInitializing,
    InitializingPosition,
    Ready,
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
constexpr float TARGET_DUTY_INIT        = 0.04f;
constexpr float KE                      = 0.0202f;  // 逆起電力定数[V*s/rad]
constexpr float OFFSET                  = 0.0f;     // 摩擦や抵抗の損失offset
constexpr float RELEASE_POINT_ROTATIONS = 11.5f;
constexpr float INITIAL_POINT_ROTATIONS = 5.8f;
constexpr float PULLEY_RADIUS           = 0.019f;  //[m]
constexpr float ENCODER_SAMPLE_PERIOD   = 0.001f;  // [s]
constexpr float ROTATION_DIRECTION      = -1.0f;

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
float vbus{};

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
 * @brief 電圧からdutyに変換
 */
float voltage_to_duty(float voltage)
{
    return voltage / vbus;
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
    // init処理が終わったら送信
    if (app_state != InitState::WaitForInit) {
        launcher.send_velocity_feedback(feedback_velocity);
        vesc.comm_can_set_duty(VESC_ID, target_duty * ROTATION_DIRECTION);
    }
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
        vbus = vesc.get_voltage();
    }

    //  ホールセンサーの設定
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
    if (launcher.get_fire_command(target_vel_from_mainboard) && app_state == InitState::Ready) {
        if (vbus > 18.5) {
            float target_voltage = velocity_to_voltage(target_vel_from_mainboard);
            target_duty          = voltage_to_duty(target_voltage);
            target_duty          = std::clamp(target_duty, 0.0f, 0.4f);
        } else {
            target_duty = 0.0f;
        }

        launcher.send_velocity_feedback(target_duty);  // duty
        launcher.send_velocity_feedback(vbus);
    }

    if (launcher.get_init()) {
        app_state = InitState::ZeroPointInitializing;
    }

    // ホールセンサーまでのinit処理
    if (app_state == InitState::ZeroPointInitializing) {
        if (magnet_near) {
            encoder.reset();
            if (!start) {
                total_encoder_rad = 0.0f;
                start             = true;
            }
            app_state = InitState::InitializingPosition;
            HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
            target_duty = TARGET_DUTY_INIT;
        }
    }

    // ホールセンサーから初期位置までのinit処理
    if (app_state == InitState::InitializingPosition) {
        if (total_encoder_rad > rotate_to_rad(INITIAL_POINT_ROTATIONS)) {
            app_state = InitState::Ready;
            launcher.send_initial_point(total_encoder_rad);
            target_duty = 0.0f;
            HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
            target_duty = TARGET_DUTY_INIT;
        }
    }

    if (app_state == InitState::Ready) {
        // しきい値を超えたらモーターを止めて、終速を送る処理
        if (total_encoder_rad > rotate_to_rad(RELEASE_POINT_ROTATIONS)) {
            launcher.send_release_point(feedback_velocity);
            app_state = InitState::ZeroPointInitializing;
            HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);
        }
        HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
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