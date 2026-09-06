/**
 * @file i_encoder.hpp
 * @author Gento Aiba (aiba-gento)
 * @brief インクリメンタルエンコーダの抽象インターフェース
 * @version 0.2.0
 * @date 2026-02-23
 *
 * @copyright Copyright (c) 2026 ararobo
 * SPDX-License-Identifier: Apache-2.0
 *
 */
#pragma once

#include <cstdint>

#include "tim.h"

namespace gn10_motor {

/**
 * @brief インクリメンタルエンコーダの抽象インターフェース
 *
 * ハードウェアタイマーのカウンタを読み取り、角速度・積算角度を提供する。
 * read_and_reset_count() はカウンタ読み取りと同時にリセットを行うため、
 * 呼び出し側は毎制御周期に1回だけ呼び出すこと。
 */
class IncrementalEncoder
{
public:
    IncrementalEncoder(uint16_t max_count, TIM_HandleTypeDef* htim, TIM_TypeDef* htim_channel);
    ~IncrementalEncoder() = default;

    /**
     * @brief ハードウェアの初期化
     */
    void hardware_init();

    /**
     * @brief カウンタ値を読み取り、同時にリセットする
     * @return int16_t 前回呼び出しからの差分カウント
     */
    int16_t read_and_reset_count();

    /**
     * @brief カウント差分を角速度 [rad/s] に変換する
     * @param count  read_and_reset_count() の戻り値
     * @param period_s 制御周期 [s]
     * @return float 角速度 [rad/s]
     */
    float count_to_angular_velocity(int16_t count, float period_s);

    /**
     * @brief カウント差分を積算し、積算角度 [rad] を返す
     * @param count read_and_reset_count() の戻り値
     * @return float 積算角度 [rad]
     */
    float accumulate_angle_rad(int16_t count);

    /**
     * @brief 積算角度とカウンタをリセットする
     */
    void reset();

private:
    /**
     * @brief カウント値をラジアンに変換する内部ユーティリティ
     * @param count カウント値
     * @return float ラジアン値
     */
    float count_to_rad(int16_t count) const;

    const uint16_t max_count_;  ///< 1回転あたりのカウント数 (分解能)
    float enc_total_;           ///< 積算角度 [rad]
    TIM_HandleTypeDef* htim_;
    TIM_TypeDef* htim_channel_;
};

}  // namespace gn10_motor
