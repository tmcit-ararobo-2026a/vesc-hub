#pragma once
#include <cstdint>

#include "fdcan.h"
#include "gn10_can/drivers/can_driver_interface.hpp"

typedef enum {
    CAN_PACKET_SET_DUTY = 0,
    CAN_PACKET_SET_CURRENT,
    CAN_PACKET_SET_CURRENT_BRAKE,
    CAN_PACKET_SET_RPM,
    CAN_PACKET_SET_POS,
    CAN_PACKET_STATUS          = 9,
    CAN_PACKET_SET_CURRENT_REL = 10,
    CAN_PACKET_SET_CURRENT_BRAKE_REL,
    CAN_PACKET_SET_CURRENT_HANDBRAKE,
    CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
    CAN_PACKET_STATUS_5          = 27,
    CAN_PACKET_MAKE_ENUM_32_BITS = 0xFFFFFFFF,
} CAN_PACKET_ID;

class VescCAN
{
private:
    struct VescStatus1 {
        int32_t rpm;
        float current;
        float duty;
    };
    struct VescStatus5 {  // 追加
        int32_t taco;
        float voltage;
    };
    VescStatus1 status1_;
    VescStatus5 status5_;

public:
    void parse_status1(uint8_t* data, VescStatus1& status);

    void parse_status5(uint8_t* data, VescStatus5& status);  // 追加

    explicit VescCAN(gn10_can::drivers::ICANDriver& can_driver);
    void update();
    void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t* index);
    void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t* index);
    ;

    void buffer_append_float16(uint8_t* buffer, float number, float scale, int32_t* index);

    void buffer_append_float32(uint8_t* buffer, float number, float scale, int32_t* index);

    void comm_can_set_duty(uint8_t controller_id, float duty);

    void comm_can_set_current(uint8_t controller_id, float current);

    void comm_can_set_current_off_delay(uint8_t controller_id, float current, float off_delay);

    void comm_can_set_current_brake(uint8_t controller_id, float current);

    void comm_can_set_rpm(uint8_t controller_id, float rpm);

    void comm_can_set_pos(uint8_t controller_id, float pos);

    void comm_can_set_current_rel(uint8_t controller_id, float current_rel);

    void comm_can_set_current_rel_off_delay(
        uint8_t controller_id, float current_rel, float off_delay
    );
    void comm_can_set_current_brake_rel(uint8_t controller_id, float current_rel);

    void comm_can_set_handbrake(uint8_t controller_id, float current);

    int32_t get_rpm() const
    {
        return -(status1_.rpm / 7);
    }

    int32_t get_taco() const
    {
        return status5_.taco;
    }

    void comm_can_set_handbrake_rel(uint8_t controller_id, float current_rel);

    gn10_can::drivers::ICANDriver& can_driver_;
};