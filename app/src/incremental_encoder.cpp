/**
 * @file incremental_encoder.cpp
 * @author Gento Aiba (aiba-gento)
 * @author Watanabe-Koichiro
 * @brief インクリメンタルエンコーダの具象クラス
 * @version 0.2.0
 * @date 2026-02-23
 *
 * @copyright Copyright (c) 2026 ararobo
 * SPDX-License-Identifier: Apache-2.0
 *
 */
#include "app/incremental_encoder.hpp"

#include <cmath>
#include <cstdint>

namespace gn10_motor {

// 2π 定数 (M_PI は POSIX 拡張のため constexpr で定義)
static constexpr float TWO_PI = 6.28318530f;

IncrementalEncoder::IncrementalEncoder(uint16_t max_count, TIM_HandleTypeDef* htim)
    : max_count_(max_count), enc_total_(0.0f), htim_(htim)
{
}

void IncrementalEncoder::hardware_init()
{
    HAL_TIM_Encoder_Start(htim_, TIM_CHANNEL_ALL);
}

int16_t IncrementalEncoder::read_and_reset_count()
{
    // 読み取りとリセットの間に割り込みが入ると最大 1 ティックの誤差が生じる
    // 許容誤差範囲内であるため対策しない
    uint16_t raw = __HAL_TIM_GET_COUNTER(htim_);
    __HAL_TIM_SET_COUNTER(htim_, 0);
    return static_cast<int16_t>(raw);
}

float IncrementalEncoder::count_to_rad(int16_t count) const
{
    return static_cast<float>(count) / static_cast<float>(max_count_) * TWO_PI;
}

float IncrementalEncoder::count_to_angular_velocity(int16_t count, float period_s)
{
    return count_to_rad(count) / period_s;
}

float IncrementalEncoder::accumulate_angle_rad(int16_t count)
{
    enc_total_ += count_to_rad(count);
    return enc_total_;
}

void IncrementalEncoder::reset()
{
    enc_total_ = 0.0f;
    __HAL_TIM_SET_COUNTER(htim_, 0);
}

}  // namespace gn10_motor