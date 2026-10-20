#pragma once
/**
 * 欲しいリリース速度[m/s] -> VESC への ERPM 指令 (実測データから作成した対応表)
 *
 * 元データ: ERPM制御で目標 2.0〜11.5 m/s を各5発ずつ撃ったときの平均から求めた
 *           エンコーダのリリース速度(release velocity)。
 */
#include <array>
#include <cstddef>

namespace target_erpm_field {

struct Node {
    float v;     // 欲しいリリース速度 [m/s] (昇順)
    float erpm;  // そのために指令する ERPM (大きさ)
};

constexpr std::array<Node, 8> NODES{
    {
     {2.000f, 5286.0f},    // 低速側の回帰直線 v = 1.0029*cmd + 0.4933 から逆算 (外挿を含む)
        {4.460f, 13916.0f},   // 同上 (2.0〜4.0 の回帰の上端)
        {4.470f, 17816.0f},   // 谷をまたいで、立ち上がり区間 (5.0→5.5相当) に移る
        {6.204f, 19350.0f},   // 実測: 目標 5.5 相当 (19350 ERPM)
        {8.705f, 21109.0f},   // 実測: 目標 6.0 相当
        {10.594f, 22868.0f},  // 実測: 目標 6.5 相当
        {11.597f, 24627.0f},  // 実測: 目標 7.0 相当
        {11.935f, 26386.0f},  // 実測: 目標 7.5 相当 (これ以上は速度が頭打ち)
    }
};

/**
 * @brief 欲しいリリース速度から、指令 ERPM を求める (区間ごとの線形補間)
 * @param v 欲しいリリース速度 [m/s]
 * @return 指令 ERPM (正の値。回転方向は呼び出し側で掛ける)
 */
inline float velocity_to_erpm(float v)
{
    if (v <= NODES.front().v) return NODES.front().erpm;
    if (v >= NODES.back().v) return NODES.back().erpm;
    for (std::size_t i = 1; i < NODES.size(); i++) {
        if (v <= NODES[i].v) {
            const float t = (v - NODES[i - 1].v) / (NODES[i].v - NODES[i - 1].v);
            return NODES[i - 1].erpm + t * (NODES[i].erpm - NODES[i - 1].erpm);
        }
    }
    return NODES.back().erpm;
}

}  // namespace target_erpm_field