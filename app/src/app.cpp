#include "app/app.hpp"

#include "adc.h"
#include "app/incremental_encoder.hpp"
#include "app/vesc_can.hpp"
#include "gn10_can/devices/esc_hub_server.hpp"
#include "gn10_can/devices/motor_driver_types.hpp"
#include "gn10_stm32_fdcan_driver/can_callback_helper.hpp"
#include "gn10_stm32_fdcan_driver/can_driver.hpp"
#include "gn10_stm32_fdcan_driver/fdcan_driver.hpp"
#include "tim.h"

#define VESC_ID 43  // 43 or 45

// タイマー使用
volatile bool timer_1khz_triggered;
volatile bool timer_100hz_triggered;

// 状態管理用
enum class InitState {
    GetInit,
    Initializing,
    FinishInit,
} init;

// メイン基板との通信に使用
gn10_can::drivers::FDCANDriver fdcan1_driver(&hfdcan1);
gn10_can::FDCANBus fdcan1_bus(fdcan1_driver);
gn10_can::devices::ESCHubServer esc_hub(fdcan1_bus, 0);
gn10_can::devices::MotorConfig motor_config_belt;
uint8_t motor_id                               = 0;
std::array<float, 4> target_vel_from_mainboard = {};
std::array<float, 4> feedback_data             = {};

// VESCとのCAN通信
gn10_can::drivers::CANDriver can2_driver(&hfdcan2, FDCAN_RX_FIFO0, true);
VescCAN vesc(can2_driver);

// constants
constexpr float RPM_CONVERSION_CONSTANT = -46000.0f;
constexpr float TARGET_RPM_INIT         = -2500.0f;
constexpr float RELEASE_POINT_ROTATIONS = 11.5f;
constexpr float INITIAL_POINT_ROTATIONS = 5.8f;
constexpr float ENCODER_SAMPLE_PERIOD   = 0.001f;  // [s]

// VESC関係
float target_rpm = 0.0f;
// エンコーダー関係
gn10_motor::IncrementalEncoder encoder(4095, &htim3, TIM3);
float total_encoder_rad = 0.0f;

// ホールセンサ
bool movement                = false;
bool magnet_near             = false;
float voltage_threshold_high = 2.0f;
float voltage_threshold_low  = 1.8f;

// LED点滅
constexpr uint32_t HEARTBEAT_TOGGLE_INTERCAL_MS = 500;
uint32_t heartbeat_last_toggle_time_ms          = 0;

// setting function
void update_heartbeat_led();
void send_anglar_data(std::array<float, 4> send_data);

float rotate_to_rad(float rotate)
{
    return rotate * M_PI * 2;
}

void timer_1khz_process()
{
    int16_t encoder_count = encoder.read_and_reset_count();
    feedback_data[0]      = encoder.count_to_angular_velocity(encoder_count, ENCODER_SAMPLE_PERIOD);
    total_encoder_rad     = encoder.accumulate_angle_rad(encoder_count);
}

void timer_100hz_process()
{
    if (init != InitState::FinishInit) {
        vesc.comm_can_set_rpm(VESC_ID, target_rpm);
        esc_hub.set_feedbacks(feedback_data.data());
    }
}

void setup()
{
    // encoder settings
    encoder.hardware_init();

    // init
    fdcan1_driver.init();
    can2_driver.init();

    // set tick
    heartbeat_last_toggle_time_ms = HAL_GetTick();

    // ADC
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

    // Wait until a command "belt_init" arrives
    while (!esc_hub.get_init(motor_id, motor_config_belt)) {
        HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
    }
    HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);

    // init
    init = InitState::FinishInit;

    // タイマーは最後に有効化
    HAL_TIM_Base_Start_IT(&htim7);
    HAL_TIM_Base_Start_IT(&htim6);
}

void loop()
{
    // ホールセンサーの設定
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);
    int32_t adc_val = HAL_ADC_GetValue(&hadc1);
    float voltage   = (float)adc_val / 4095.0f * 3.3f;

    // ホールセンサー反応処理
    if (voltage > voltage_threshold_high && !magnet_near) {
        magnet_near = true;
    } else if (magnet_near && voltage < voltage_threshold_low) {
        magnet_near = false;
    }

    // 司令を受信
    esc_hub.get_targets(target_vel_from_mainboard.data());
    target_vel_from_mainboard[0] = std::clamp(target_vel_from_mainboard[0], 0.0f, 1.0f);

    if (esc_hub.get_init(motor_id, motor_config_belt) && init == InitState::FinishInit) {
        movement = false;
        init     = InitState::GetInit;
    }

    // Control motor moving rpm
    target_rpm = target_vel_from_mainboard[0] * RPM_CONVERSION_CONSTANT;

    // ホールセンサーまでのinit処理
    if (init == InitState::GetInit) {
        if (magnet_near) {
            encoder.read_and_reset_count();
            init = InitState::Initializing;
        } else {
            target_rpm = TARGET_RPM_INIT;
        }
    }

    // ホールセンサーから初期位置までのinit処理
    if (init == InitState::Initializing) {
        if (total_encoder_rad > rotate_to_rad(INITIAL_POINT_ROTATIONS)) {
            init       = InitState::FinishInit;
            target_rpm = 0.0f;
            movement   = true;
        } else {
            target_rpm = TARGET_RPM_INIT;
        }
    }

    // REREASE POINTを超えたら、動かないようにする。
    if (total_encoder_rad > rotate_to_rad(RELEASE_POINT_ROTATIONS)) {
        movement = false;
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
    if (process_fdcan_fifo(hfdcan, &hfdcan2, vesc, FDCAN_RX_FIFO0)) return;
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo1ITs)
{
    (void)RxFifo1ITs;
    if (process_fdcan_fifo(hfdcan, &hfdcan1, fdcan1_bus, FDCAN_RX_FIFO1)) return;
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
 * @brief Toggle heartbeat LED at    float angle_now;
    float delta_angle;
    float initial_speed; a fixed interval.
 */
void update_heartbeat_led()
{
    const uint32_t now_ms = HAL_GetTick();
    if ((now_ms - heartbeat_last_toggle_time_ms) >= HEARTBEAT_TOGGLE_INTERCAL_MS) {
        heartbeat_last_toggle_time_ms = now_ms;
        HAL_GPIO_TogglePin(LED_4_GPIO_Port, LED_4_Pin);
    }
}