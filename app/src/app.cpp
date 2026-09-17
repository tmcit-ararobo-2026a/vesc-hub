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

float encoder_angle = 0;

void update_heartbeat_led()
{
    const uint32_t now_ms = HAL_GetTick();
    if ((now_ms - heartbeat_last_toggle_time_ms) >= HEARTBEAT_TOGGLE_INTERCAL_MS) {
        heartbeat_last_toggle_time_ms = now_ms;
        HAL_GPIO_TogglePin(LED_4_GPIO_Port, LED_4_Pin);
    }
}

void send_encoder_data()
{
    const uint32_t now_ms = HAL_GetTick();
    if ((now_ms - send_encoder_data_last_time_ms) >= k_send_encoder_data_interval_ms) {
        send_encoder_data_last_time_ms = now_ms;
        launcher.send_velocity_feedback(encoder_angle);
        HAL_GPIO_TogglePin(LED_4_GPIO_Port, LED_4_Pin);
    }
}

void setup()
{
    // CAN通信の開始
    fdcan1_driver.init();
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    send_encoder_data_last_time_ms = HAL_GetTick();
    while (launcher.get_init()) {
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
    }
    HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
}

void loop()
{
    // encoder
    int16_t encoder_count = static_cast<int16_t>(__HAL_TIM_GET_COUNTER(&htim3));
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    encoder_angle = encoder_count * (A_ROTATE_ANGLE / ENCODER_COUNT_PER_ROTATE);

    update_heartbeat_led();
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
    (void)RxFifo0ITs;
    if (process_fdcan_fifo(hfdcan, &hfdcan1, fdcan1_bus, FDCAN_RX_FIFO0)) return;
}
