#include "belt/belt.hpp"

#include "adc.h"
#include "belt/fdcan_driver.hpp"
#include "belt/vesc_can.hpp"
#include "drivers/stm32_fdcan/driver_stm32_fdcan.hpp"
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
constexpr float DISTANCE_PER_ROTATION    = 0.12f;

// definitions
float vesc_vel[4]  = {0.0f, 0.0f, 0.0f, 0.0f};
float send_data[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float rotate_count = 0.0f;
float target_rpm   = 0.0f;
float angle_last   = 0.0f;

bool movement     = false;
bool magnet_near  = false;
bool init         = false;
bool init_command = false;

// Voltage threshold for hall sensor
float voltage_threshold_high = 2.0f;
float voltage_threshold_low  = 1.8f;

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

    HAL_TIM_Base_Start_IT(&htim7);
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
    esc_hub.get_targets(vesc_vel);

    if (vesc_vel[0] > 1.0f) {
        vesc_vel[0] = 1.0f;
    } else if (vesc_vel[0] < 0.0f) {
        vesc_vel[0] = 0;
    }

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

    if (rotate_count > 11.5) {
        movement = false;
    }

        if (movement && !init) {
        vesc.comm_can_set_rpm(45, target_rpm);
        vesc.comm_can_set_rpm(43, target_rpm);
    } else {
        if (!magnet_near) {
            vesc.comm_can_set_rpm(45, TARGET_RPM_INIT);
            vesc.comm_can_set_rpm(43, TARGET_RPM_INIT);
        } else if (!init) {
            vesc.comm_can_set_rpm(45, 0);
            vesc.comm_can_set_rpm(43, 0);
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
            vesc.comm_can_set_rpm(43, TARGET_RPM_INIT);
            vesc.comm_can_set_rpm(45, TARGET_RPM_INIT);
        }
    }

    update_heartbeat_led();
}

// CAN Receive CAllback
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

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo1ITs)
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

// send_data
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM7) {
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

    if (esc_hub.get_init(motor_id, motor_config_belt) && init_command) {
        movement     = false;
        init         = false;
        init_command = false;
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
        esc_hub.set_feedbacks(angular_data);
        HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);
    }
}
