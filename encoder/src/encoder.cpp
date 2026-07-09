#include "encoder/encoder.hpp"

#include "adc.h"
#include "drivers/stm32_fdcan/driver_stm32_fdcan.hpp"
#include "encoder/fdcan_driver.hpp"
#include "encoder/vesc_can.hpp"
#include "gn10_can/core/can_bus.hpp"
#include "gn10_can/core/fdcan_bus.hpp"
#include "gn10_can/devices/esc_hub_server.hpp"
#include "gn10_can/devices/motor_driver_types.hpp"
#include "stdio.h"
#include "tim.h"

gn10_can::drivers::FDCANDriver fdcan1_driver(&hfdcan1);
VescCAN vesc(&hfdcan2);

gn10_can::FDCANBus fdcan1_bus(fdcan1_driver);
gn10_can::devices::ESCHubServer esc_hub(fdcan1_bus, 0);

// These use get init
gn10_can::devices::MotorConfig motor_config_belt;
uint8_t motor_num;

constexpr int16_t noise                 = 1;
float vesc_velo[4]                      = {0.0f, 0.0f, 0.0f, 0.0f};
constexpr float rpm_conversion_constant = -40000.0f;
float target_rpm                        = 0.0f;

// Control flag
bool magnet_near = false;
bool homing      = false;

// Voltage threshold for hall sensor
float voltage_threshold_high = 1.8f;
float voltage_threshold_low  = 1.4f;

// LED config
constexpr uint32_t k_heartbeat_toggle_interval_ms = 500;
uint32_t heartbeat_last_toggle_time_ms            = 0;

// Can send data config
constexpr uint32_t k_send_anglar_data_interval_ms = 100;
uint32_t send_anglar_data_last_time_ms            = 0;

int32_t absolute_value = 0;

void update_heartbeat_led();

void send_anglar_data(float angular_data[4]);

void do_homing();

void setup()
{
    // encoder settings
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    fdcan1_driver.init();
    send_anglar_data_last_time_ms = HAL_GetTick();
    vesc.init();
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

    // Wait until a command "belt_init" arrives
    while (!esc_hub.get_init(motor_num, motor_config_belt)) {
        HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
    }
    HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);

    homing = true;
}

void loop()
{
    // 2~ init command
    if (esc_hub.get_init(motor_num, motor_config_belt)) {
        homing = true;
    }

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

    int16_t motor_point = static_cast<int16_t>(__HAL_TIM_GET_COUNTER(&htim3));
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    absolute_value += motor_point;

    if (absolute_value > 17000) {
        homing = true;
    }

    // Control motor moving
    target_rpm = vesc_velo[0] * rpm_conversion_constant;

    if (!homing) {
        vesc.comm_can_set_rpm(45, target_rpm);
        vesc.comm_can_set_rpm(43, target_rpm);
    }

    float rotates_test[4] = {
        static_cast<float>(motor_point),
        static_cast<float>(absolute_value),
        static_cast<float>(homing),
        0.0f
    };

    send_anglar_data(rotates_test);
}

// Callback processing
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan->Instance == hfdcan1.Instance) {
        fdcan1_bus.update();
    }
    if (hfdcan->Instance == hfdcan2.Instance) {
        FDCAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
            vesc.receive_data(rx_header.Identifier, rx_data, 8);
        }
    }
}

/**
 * @brief Toggle heartbeat LED at a fixed interval.
 */
void update_heartbeat_led()
{
    const uint32_t now_ms = HAL_GetTick();
    if ((now_ms - heartbeat_last_toggle_time_ms) >= k_heartbeat_toggle_interval_ms) {
        heartbeat_last_toggle_time_ms = now_ms;
        HAL_GPIO_TogglePin(LED_4_GPIO_Port, LED_4_Pin);
    }
}

void send_anglar_data(float angular_data[4])
{
    const uint32_t now_ms = HAL_GetTick();
    if ((now_ms - send_anglar_data_last_time_ms) >= k_send_anglar_data_interval_ms) {
        send_anglar_data_last_time_ms = now_ms;
        esc_hub.set_angular_velocity_feedbacks(angular_data);
        HAL_GPIO_TogglePin(LED_4_GPIO_Port, LED_4_Pin);
    }
}

/**
 * @brief Set the initial position of the motor.
 */
void do_homing()
{
    if (!magnet_near) {
        vesc.comm_can_set_rpm(45, -2500);
        vesc.comm_can_set_rpm(43, -2500);
        absolute_value = 0;

    } else {
        vesc.comm_can_set_rpm(45, 0);
        vesc.comm_can_set_rpm(43, 0);
        homing         = false;
        absolute_value = 0;
    }

    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
}
