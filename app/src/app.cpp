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
#define VESC_ID 45

// VESCとのCAN通信
gn10_can::drivers::CANDriver can2_driver(&hfdcan2, FDCAN_RX_FIFO1, true);
VescCAN vesc(can2_driver);

gn10_can::drivers::FDCANDriver fdcan1_driver(&hfdcan1);
gn10_can::FDCANBus fdcan1_bus(fdcan1_driver);
gn10_can::devices::LauncherServer launcher(fdcan1_bus, 0);

// LED点滅
constexpr uint32_t HEARTBEAT_TOGGLE_INTERCAL_MS = 500;
uint32_t heartbeat_last_toggle_time_ms          = 0;

constexpr uint32_t k_send_encoder_data_interval_ms = 100;
uint32_t send_encoder_data_last_time_ms            = 0;

constexpr float ENCODER_COUNT_PER_ROTATE = 4096.0f;
constexpr float A_ROTATE_ANGLE           = 360.0f;

// ホールセンサ
bool magnet_near             = false;
float voltage_threshold_high = 2.0f;
float voltage_threshold_low  = 1.8f;

float rotate_count{};
float absolute_angle{};
float encoder_angle = 0;

void update_heartbeat_led()
{
    const uint32_t now_ms = HAL_GetTick();
    if ((now_ms - heartbeat_last_toggle_time_ms) >= HEARTBEAT_TOGGLE_INTERCAL_MS) {
        heartbeat_last_toggle_time_ms = now_ms;
        HAL_GPIO_TogglePin(LED_4_GPIO_Port, LED_4_Pin);
    }
}

void send_encoder_data(float value)
{
    const uint32_t now_ms = HAL_GetTick();
    if ((now_ms - send_encoder_data_last_time_ms) >= k_send_encoder_data_interval_ms) {
        send_encoder_data_last_time_ms = now_ms;
        launcher.send_velocity_feedback(value);
        HAL_GPIO_TogglePin(LED_3_GPIO_Port, LED_3_Pin);
    }
}

void setup()
{
    // CAN通信の開始
    fdcan1_driver.init();
    fdcan1_driver.set_tx_timeout(2);
    can2_driver.init();

    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    send_encoder_data_last_time_ms = HAL_GetTick();
    while (!launcher.get_init()) {
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
        vesc.comm_can_set_rpm(VESC_ID, 0);
    }
    HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
}

bool get_init = false;
void loop()
{
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

    int16_t encoder_count = static_cast<int16_t>(__HAL_TIM_GET_COUNTER(&htim3));
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    float raw_encoder_value = static_cast<float>(encoder_count);

    if (launcher.get_init()) {
        get_init = !get_init;
    }
    //    vesc.comm_can_set_rpm(VESC_ID, 35000);

    if (magnet_near) {
        get_init = false;
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
    }

    if (get_init) {
        vesc.comm_can_set_rpm(VESC_ID, -5000);

        // vesc.comm_can_set_current(VESC_ID, 10.0f);
    } else {
        vesc.comm_can_set_rpm(VESC_ID, 0);
        // vesc.comm_can_set_current(VESC_ID, 0.0f);
    }

    absolute_angle += raw_encoder_value;
    send_encoder_data(absolute_angle / 4096.0f);
    update_heartbeat_led();
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
    (void)RxFifo0ITs;
    if (process_fdcan_fifo(hfdcan, &hfdcan1, fdcan1_bus, FDCAN_RX_FIFO0)) return;
}
