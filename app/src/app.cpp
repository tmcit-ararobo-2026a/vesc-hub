#include "app/app.hpp"

#include <algorithm>
#include <cmath>

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
    GoToInitPoint,
} app_state;

// メイン基板との通信に使用
gn10_can::drivers::FDCANDriver fdcan1_driver(&hfdcan1);
gn10_can::FDCANBus fdcan1_bus(fdcan1_driver);
gn10_can::devices::LauncherServer launcher(fdcan1_bus, 0);

float target_erpm_from_mainboard{};
float feedback_angular_velocity{};  // [rad/s]

// VESCとのCAN通信
gn10_can::drivers::CANDriver can2_driver(&hfdcan2, FDCAN_RX_FIFO1, true);
VescCAN vesc(can2_driver);

// constants
constexpr float TARGET_ERPM_INIT        = 2000.0f;
constexpr float INITIAL_POINT_ROTATIONS = 5.8f;
constexpr float NUMBER_OF_ROTATION      = 12.0f;
constexpr float MOTOR_POLES             = 14.0f;
constexpr float POLE_PAIRS              = MOTOR_POLES / 2.0f;
constexpr float ENCODER_SAMPLE_PERIOD   = 0.001f;  // [s]
constexpr float ROTATION_DIRECTION      = -1.0f;
constexpr float INIT_RETURN_MIN_ERPM    = 1500.0f;
constexpr float INIT_RETURN_MAX_ERPM    = 10000.0f;
constexpr float CONTROL_PERIOD_S        = 0.01f;  // timer_100hz の周期 [s]
constexpr float RAMP_GAIN               = 0.7f;
constexpr float RAMP_MAX_PER_S          = 1000000.0f;
constexpr float RETURN_APPROACH_TIME_S  = 1.0f;

// VESC関係
float target_erpm{};    // 最終目標 [erpm] (正の値。向きは送信時に反転)
float revision_erpm{};  // VESCへ実際に送る値 [erpm]

// フィードバック
int32_t feedback_erpm{};
float feedback_voltage{};
float feedback_current{};

// エンコーダー関係
gn10_motor::IncrementalEncoder encoder(4000, &htim3);
float encoder_rad{};

// ホールセンサ
bool magnet_near                   = false;
const float voltage_threshold_high = 2.1f;
const float voltage_threshold_low  = 1.8f;

// LED点滅
constexpr uint32_t HEARTBEAT_TOGGLE_INTERVAL_MS = 500;
uint32_t heartbeat_last_toggle_time_ms          = 0;

// その他動作補助
uint32_t process_count{};

/**  変換関数 **/

/**
 * @brief 回転数からradに変換
 */
float rotate_to_rad(float rotate)
{
    return rotate * M_PI * 2;
}

/**
 * @brief radから回転数に変換
 */
float rad_to_rotate(float rad)
{
    return rad / (M_PI * 2);
}

/**
 * @brief エンコーダ速度から求めた現在のERPM (絶対値)
 */
float encoder_erpm()
{
    float rpm = std::fabs(feedback_angular_velocity) / (M_PI * 2) * 60.0f;
    return rpm * POLE_PAIRS;  // erpm
}

/**
 * @brief リリース位置までの残り角度 [rad]
 * @param release_point リリース位置 [rev]
 */
float remaining_rad_to_release(float release_point)
{
    return rotate_to_rad(release_point * static_cast<float>(process_count)) - encoder_rad;
}

/**
 * @brief 初期位置までの残り角度 [rad] (行き過ぎても負にならない)
 */
float remaining_rad_to_init()
{
    return std::max<float>(
        rotate_to_rad(INITIAL_POINT_ROTATIONS) +
            rotate_to_rad(NUMBER_OF_ROTATION * static_cast<float>(process_count)) - encoder_rad,
        0.0f
    );
}

/**
 * @brief 残り距離 remaining_rad [rad] の間に now_erpm から target へ到達するための加速レート
 * [erpm/s] R = RAMP_GAIN * (E_target^2 - E_now^2) / (2 * 60 * 極対数 * 残り回転数[rev]) ※
 * 式は回転数[rev]基準なので、rad を rev に直してから使う
 */
float ramp_up_rate_dynamic(float target, float now_erpm, float remaining_rad)
{
    if (now_erpm >= target) return 0.0f;
    const float remaining_rev = std::max(rad_to_rotate(remaining_rad), 0.05f);  // 0割り・発散防止
    const float rate          = RAMP_GAIN * (target * target - now_erpm * now_erpm) /
                                (2.0f * 60.0f * POLE_PAIRS * remaining_rev);
    return std::min(rate, RAMP_MAX_PER_S);
}

/**
 * @brief VESCへ送る指令ERPMを更新する
 * @param use_dynamic true = 残り距離から加速レートを計算 / false = 上限レート
 */
float update_cmd_erpm(
    float current_cmd, float target, float now_erpm, float remaining_rad, bool use_dynamic
)
{
    float rate{};
    if (now_erpm >= target) return target;

    if (use_dynamic) {
        rate = ramp_up_rate_dynamic(target, now_erpm, remaining_rad);
    } else {
        rate = RAMP_MAX_PER_S;  // 最高加速度
    }
    return std::min(current_cmd + rate * CONTROL_PERIOD_S, target);
}

/** コールバック関数 **/

void timer_1khz_process()
{
    int16_t encoder_count = encoder.read_and_reset_count();
    feedback_angular_velocity =
        encoder.count_to_angular_velocity(encoder_count, ENCODER_SAMPLE_PERIOD);
    encoder_rad = encoder.accumulate_angle_rad(encoder_count);
}

void timer_100hz_process()
{
    if (app_state != InitState::WaitForInit) {
        launcher.send_feedbacks(
            static_cast<float>(feedback_erpm), feedback_current, feedback_voltage
        );

        // 射出(Ready)中だけ、リリース位置までの残り距離から加速レートを動的に決める
        revision_erpm = update_cmd_erpm(
            revision_erpm,
            target_erpm,
            encoder_erpm(),
            remaining_rad_to_release(NUMBER_OF_ROTATION),
            app_state == InitState::Ready
        );

        vesc.comm_can_set_erpm(VESC_ID, revision_erpm * ROTATION_DIRECTION);
    }
}

/** 他関数 **/

/**
 * @brief 初期位置へ戻る際の目標ERPMを計算する (従来方式)
 *        残り回転数が大きいほど速く、近づくと最小ERPMに収束する
 * @param approach_time_s 時定数 [s]。大きいほどゆっくり減速する
 * @return 目標ERPM
 */
float send_init_erpm(float approach_time_s)
{
    // 残り回転数 [rev] を approach_time_s で走り切る回転速度 [rpm] -> ERPM
    const float remaining_rev = rad_to_rotate(remaining_rad_to_init());
    const float approach_rpm  = remaining_rev / approach_time_s * 60.0f;
    const float erpm          = INIT_RETURN_MIN_ERPM + approach_rpm * POLE_PAIRS;

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
        feedback_erpm    = vesc.get_erpm();
        feedback_current = vesc.get_current();
        feedback_voltage = vesc.get_voltage();
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
    if (app_state == InitState::Ready && launcher.get_fire_command(target_erpm_from_mainboard)) {
        target_erpm = target_erpm_from_mainboard;
    }

    if (launcher.get_init()) {
        app_state     = InitState::ZeroPointInitializing;
        process_count = 0;
    }

    // ホールセンサーまでのinit処理
    if (app_state == InitState::ZeroPointInitializing) {
        if (magnet_near) {
            encoder.reset();
            encoder_rad = 0.0f;
            app_state   = InitState::InitializingPosition;
            HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
            target_erpm = TARGET_ERPM_INIT;
        }
    }

    // ホールセンサーから初期位置までのinit処理
    if (app_state == InitState::InitializingPosition) {
        if (encoder_rad > rotate_to_rad(INITIAL_POINT_ROTATIONS)) {
            target_erpm = 0.0f;
            app_state   = InitState::Ready;
            process_count++;
            launcher.send_initial_point(encoder_rad);
            HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);

        } else {
            target_erpm = TARGET_ERPM_INIT;
            HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
        }
    }

    if (app_state == InitState::Ready) {
        // しきい値を超えたらモーターを止めて、終速を送る処理
        if (encoder_rad > (rotate_to_rad(NUMBER_OF_ROTATION) * static_cast<float>(process_count))) {
            launcher.send_release_point(feedback_erpm);
            HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);
            app_state = InitState::GoToInitPoint;
        } else {
            HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
        }
    }

    if (app_state == InitState::GoToInitPoint) {
        if (encoder_rad > (rotate_to_rad(INITIAL_POINT_ROTATIONS) +
                           rotate_to_rad(NUMBER_OF_ROTATION) * static_cast<float>(process_count))) {
            target_erpm = 0.0f;
            app_state   = InitState::Ready;
            process_count++;
            launcher.send_initial_point(encoder_rad);
            HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
            target_erpm = send_init_erpm(RETURN_APPROACH_TIME_S);
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