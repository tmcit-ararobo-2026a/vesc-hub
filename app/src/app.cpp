#include "app/app.hpp"

#include "adc.h"
#include "app/vesc_can.hpp"
#include "gn10_can/core/can_bus.hpp"
#include "gn10_can/core/fdcan_bus.hpp"
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
bool init         = false;
bool init_command = false;

// メイン基板との通信に使用
gn10_can::drivers::FDCANDriver fdcan1_driver(&hfdcan1);
gn10_can::FDCANBus fdcan1_bus(fdcan1_driver);
gn10_can::devices::ESCHubServer esc_hub(fdcan1_bus, 0);
gn10_can::devices::MotorConfig motor_config_belt;
uint8_t motor_id                   = 0;
float target_vel_from_mainboard[4] = {0.0f, 0.0f, 0.0f, 0.0f};

// VESCとのCAN通信
gn10_can::drivers::CANDriver can2_driver(&hfdcan2, FDCAN_RX_FIFO0, true);
VescCAN vesc(can2_driver);

// constants
constexpr float RPM_CONVERSION_CONSTANT  = -46000.0f;
constexpr float TARGET_RPM_INIT          = -2500.0f;
constexpr float ENCODER_COUNT_PER_ROTATE = 4096.0f;
constexpr float A_ROTATE_ANGLE           = 360.0f;
constexpr float DISTANCE_PER_ROTATION    = 0.12f;

// VESC関係
float target_rpm = 0.0f;
// エンコーダー関係
float rotate_count = 0.0f;
float angle_last   = 0.0f;
float absolute_angle;
// ホールセンサ
bool movement                = false;
bool magnet_near             = false;
float voltage_threshold_high = 2.0f;
float voltage_threshold_low  = 1.8f;

// LED点滅
constexpr uint32_t HEARTBEAT_TOGGLE_INTERCAL_MS = 500;
uint32_t heartbeat_last_toggle_time_ms          = 0;

// Can send data config
constexpr uint32_t SEND_ANGLAR_DATA_INTERCAL_MS = 100;
uint32_t send_anglar_data_last_time_ms          = 0;

// setting function
void update_heartbeat_led();
void send_anglar_data(float angular_data[4]);
void timer_1khz_process()
{
    float angle_now;
    float delta_angle;
    float initial_speed;

    angle_now     = absolute_angle;
    delta_angle   = angle_now - angle_last;
    initial_speed = ((delta_angle / A_ROTATE_ANGLE) * DISTANCE_PER_ROTATION) / 0.001f;

    angle_last = angle_now;

    HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
    float speed_data[4] = {initial_speed, 0.0f, 0.0f, 0.0f};

    // send
    if (rotate_count > 11.4f && movement) {
        send_anglar_data(speed_data);
    }
}

void timer_100hz_process()
{
    vesc.comm_can_set_rpm(VESC_ID, target_rpm);
}
void setup()
{
    // encoder settings
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    // init
    fdcan1_driver.init();
    can2_driver.init();

    // set tick
    send_anglar_data_last_time_ms = HAL_GetTick();
    heartbeat_last_toggle_time_ms = HAL_GetTick();

    // ADC
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

    // Wait until a command "belt_init" arrives
    while (!esc_hub.get_init(motor_id, motor_config_belt)) {
        HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
    }
    HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);

    absolute_angle = 0;

    // タイマーは最後に有効化
    HAL_TIM_Base_Start_IT(&htim7);
}

void loop()
{
    // 司令を受信
    esc_hub.get_targets(target_vel_from_mainboard);
    target_vel_from_mainboard[0] = std::clamp(target_vel_from_mainboard[0], 0.0f, 1.0f);

    if (esc_hub.get_init(motor_id, motor_config_belt) && init_command) {
        movement     = false;
        init         = false;
        init_command = false;
    }

    // エンコーダーのパルスカウントを取得
    int16_t encoder_count = static_cast<int16_t>(__HAL_TIM_GET_COUNTER(&htim3));
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    // １回転360度として正規化
    float encoder_angle = encoder_count * (A_ROTATE_ANGLE / ENCODER_COUNT_PER_ROTATE);
    // belt initで定めたゼロ点からの絶対角度[deg]
    absolute_angle += encoder_angle;
    rotate_count = absolute_angle / A_ROTATE_ANGLE;  // 回転回数[回]

    // Hall sensor settings
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);
    int32_t adc_val = HAL_ADC_GetValue(&hadc1);
    float voltage   = (float)adc_val / 4095.0f * 3.3f;

    // Control Hall sensor
    if (voltage > voltage_threshold_high && !magnet_near) {
        magnet_near = true;
    } else if (magnet_near && voltage < voltage_threshold_low) {
        magnet_near = false;
    }

    // Control motor moving rpm
    target_rpm = target_vel_from_mainboard[0] * RPM_CONVERSION_CONSTANT;

    if (rotate_count > 11.5) {
        movement = false;
    }

    if (movement && !init) {
    } else {
        if (!magnet_near) {
            target_rpm = TARGET_RPM_INIT;
        } else if (!init) {
            target_rpm     = 0.0f;
            absolute_angle = 0.0f;
            angle_last     = 0.0f;
            rotate_count   = 0.0f;
            movement       = true;
            init           = true;
        }
    }

    if (init) {
        if (rotate_count > 5.8) {
            init         = false;
            init_command = true;
        } else {
            target_rpm = TARGET_RPM_INIT;
        }
    }
    if (timer_100hz_triggered) {
        timer_100hz_triggered = false;
        timer_100hz_process();
    }
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
 * @brief Toggle heartbeat LED at a fixed interval.
 */
void update_heartbeat_led()
{
    const uint32_t now_ms = HAL_GetTick();
    if ((now_ms - heartbeat_last_toggle_time_ms) >= HEARTBEAT_TOGGLE_INTERCAL_MS) {
        heartbeat_last_toggle_time_ms = now_ms;
        HAL_GPIO_TogglePin(LED_4_GPIO_Port, LED_4_Pin);
    }
}

/**
 * @brief Toggle send at a fixed interval.
 */
void send_anglar_data(float angular_data[4])
{
    const uint32_t now_ms = HAL_GetTick();
    if ((now_ms - send_anglar_data_last_time_ms) >= SEND_ANGLAR_DATA_INTERCAL_MS) {
        send_anglar_data_last_time_ms = now_ms;
        esc_hub.set_feedbacks(angular_data);
        HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);
    }
}
