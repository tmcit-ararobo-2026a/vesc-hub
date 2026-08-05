#include "encoder/encoder.hpp"

#include "adc.h"
#include "drivers/stm32_fdcan/driver_stm32_fdcan.hpp"
#include "encoder/fdcan_driver.hpp"
#include "encoder/vesc_can.hpp"
#include "gn10_can/core/can_bus.hpp"
#include "gn10_can/core/fdcan_bus.hpp"
#include "gn10_can/devices/esc_hub_server.hpp"
#include "gn10_can/devices/motor_driver_types.hpp"
#include "tim.h"

gn10_can::drivers::FDCANDriver fdcan1_driver(&hfdcan1);
gn10_can::FDCANBus fdcan1_bus(fdcan1_driver);
gn10_can::devices::ESCHubServer esc_hub(fdcan1_bus, 0);

// These use get init
gn10_can::devices::MotorConfig motor_config_belt;
uint8_t motor_id = 0;

// vesc
VescCAN vesc(&hfdcan2);
float absolute_angle;

// constants
constexpr float RPM_CONVERSION_CONSTANT  = -46000.0f;
constexpr float TARGET_RPM_INIT          = -2500.0f;
constexpr float ENCODER_COUNT_PER_ROTATE = 4096.0f;
constexpr float A_ROTATE_ANGLE           = 360.0f;

// definitions
float vesc_vel[4]  = {0.0f, 0.0f, 0.0f, 0.0f};
bool movement      = true;
bool magnet_near   = false;
float rotate_count = 0.0f;
float target_rpm   = 0.0f;

// Voltage threshold for hall sensor
float voltage_threshold_high = 0.5f;
float voltage_threshold_low  = 0.3f;

// LED config
constexpr uint32_t k_heartbeat_toggle_interval_ms = 500;
uint32_t heartbeat_last_toggle_time_ms            = 0;

// Can send data config
constexpr uint32_t k_send_anglar_data_interval_ms = 100;
uint32_t send_anglar_data_last_time_ms            = 0;

// setting function
void update_heartbeat_led();
void send_anglar_data(float angular_data[4]);

void setup()
{
    // encoder settings
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    // init
    fdcan1_driver.init();
    vesc.init();

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
}

void loop()
{
    // get command
    esc_hub.get_angular_velocities(vesc_vel);

    // encoder
    int16_t encoder_count = static_cast<int16_t>(__HAL_TIM_GET_COUNTER(&htim3));
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    float encoder_angle = encoder_count * (A_ROTATE_ANGLE / ENCODER_COUNT_PER_ROTATE);

    // absolute
    absolute_angle += encoder_angle;

    rotate_count = absolute_angle / A_ROTATE_ANGLE;

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
    target_rpm = vesc_vel[0] * RPM_CONVERSION_CONSTANT;

    if (rotate_count > 6) {
        movement = false;
    }

    if (movement) {
        vesc.comm_can_set_rpm(45, target_rpm);
        vesc.comm_can_set_rpm(43, target_rpm);
    } else {
        vesc.comm_can_set_rpm(45, TARGET_RPM_INIT);
        vesc.comm_can_set_rpm(43, TARGET_RPM_INIT);
    }

    // set anglar data for gn10 main
    float anglar_data[4] = {rotate_count, 0.0f, encoder_angle, absolute_angle};

    send_anglar_data(anglar_data);

    update_heartbeat_led();
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

/**
 * @brief Toggle send at a fixed interval.
 */
void send_anglar_data(float angular_data[4])
{
    const uint32_t now_ms = HAL_GetTick();
    if ((now_ms - send_anglar_data_last_time_ms) >= k_send_anglar_data_interval_ms) {
        send_anglar_data_last_time_ms = now_ms;
        esc_hub.set_angular_velocity_feedbacks(angular_data);
        HAL_GPIO_TogglePin(LED_4_GPIO_Port, LED_4_Pin);
    }
}
