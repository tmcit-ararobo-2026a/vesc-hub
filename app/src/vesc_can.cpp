#include "app/vesc_can.hpp"

VescCAN::VescCAN(gn10_can::drivers::ICANDriver& can_driver) : can_driver_(can_driver) {}

void VescCAN::parse_status1(uint8_t* data, VescStatus1& status)
{
    status.rpm     = (int32_t)((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]);
    status.current = (int16_t)((data[4] << 8) | data[5]) / 10.0f;
    status.duty    = (int16_t)((data[6] << 8) | data[7]) / 1000.0f;
}

void VescCAN::parse_status5(uint8_t* data, VescStatus5& status)
{
    status.taco    = (int32_t)((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]);
    status.voltage = (int16_t)((data[4] << 8) | data[5]) / 10.0f;
}

void VescCAN::update()
{
    gn10_can::CANFrame out_frame;
    can_driver_.receive(out_frame);
    uint8_t packet_id = (out_frame.id >> 8) & 0xFF;

    // rpm feedback
    if (packet_id == CAN_PACKET_STATUS) {
        parse_status1(out_frame.data.data(), status1_);
    }
    // taco feedback
    if (packet_id == CAN_PACKET_STATUS_5) {  // 追加
        parse_status5(out_frame.data.data(), status5_);
    }
}

void VescCAN::buffer_append_int16(uint8_t* buffer, int16_t number, int32_t* index)
{
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void VescCAN::buffer_append_int32(uint8_t* buffer, int32_t number, int32_t* index)
{
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void VescCAN::buffer_append_float16(uint8_t* buffer, float number, float scale, int32_t* index)
{
    buffer_append_int16(buffer, (int16_t)(number * scale), index);
}

void VescCAN::buffer_append_float32(uint8_t* buffer, float number, float scale, int32_t* index)
{
    buffer_append_int32(buffer, (int32_t)(number * scale), index);
}

// set
// controller_id is can_id of VESC
void VescCAN::comm_can_set_duty(uint8_t controller_id, float duty)
{
    int32_t send_index = 0;
    gn10_can::CANFrame frame;
    buffer_append_int32(frame.data.data(), (int32_t)(duty * 100000.0), &send_index);

    frame.id = controller_id | ((uint32_t)CAN_PACKET_SET_DUTY << 8);
    frame.set_data_length(sizeof(int32_t));
    can_driver_.send(frame);
}

void VescCAN::comm_can_set_current(uint8_t controller_id, float current)
{
    int32_t send_index = 0;
    gn10_can::CANFrame frame;
    buffer_append_int32(frame.data.data(), (int32_t)(current * 1000.0), &send_index);

    frame.id = controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8);
    frame.set_data_length(sizeof(int32_t));
    can_driver_.send(frame);
}

void VescCAN::comm_can_set_current_off_delay(uint8_t controller_id, float current, float off_delay)
{
    int32_t send_index = 0;
    gn10_can::CANFrame frame;
    buffer_append_int32(frame.data.data(), (int32_t)(current * 1000.0), &send_index);
    buffer_append_float16(frame.data.data(), off_delay, 1e3, &send_index);
    frame.id = controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8);
    frame.set_data_length(6);
    can_driver_.send(frame);
}

void VescCAN::comm_can_set_current_brake(uint8_t controller_id, float current)
{
    int32_t send_index = 0;
    gn10_can::CANFrame frame;
    buffer_append_int32(frame.data.data(), (int32_t)(current * 1000.0), &send_index);
    frame.id = controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8);
    frame.set_data_length(sizeof(int32_t));
    can_driver_.send(frame);
}

void VescCAN::comm_can_set_rpm(uint8_t controller_id, float rpm)
{
    int32_t send_index = 0;
    gn10_can::CANFrame frame;
    buffer_append_int32(frame.data.data(), (int32_t)rpm, &send_index);
    frame.id = controller_id | ((uint32_t)CAN_PACKET_SET_RPM << 8);
    frame.set_data_length(sizeof(int32_t));
    can_driver_.send(frame);
}

void VescCAN::comm_can_set_pos(uint8_t controller_id, float pos)
{
    int32_t send_index = 0;
    gn10_can::CANFrame frame;
    buffer_append_int32(frame.data.data(), (int32_t)(pos * 1000000.0), &send_index);
    frame.id = controller_id | ((uint32_t)CAN_PACKET_SET_POS << 8);
    frame.set_data_length(sizeof(int32_t));
    can_driver_.send(frame);
}

void VescCAN::comm_can_set_current_rel(uint8_t controller_id, float current_rel)
{
    int32_t send_index = 0;
    gn10_can::CANFrame frame;
    buffer_append_float32(frame.data.data(), current_rel, 1e5, &send_index);
    frame.id = controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_REL << 8);
    frame.set_data_length(sizeof(float));
    can_driver_.send(frame);
}

/**
 * Same as above, but also sets the off delay. Note that this command uses 6 bytes now. The off
 * delay is useful to set to keep the current controller running for a while even after setting
 * currents below the minimum current.
 */
void VescCAN::comm_can_set_current_rel_off_delay(
    uint8_t controller_id, float current_rel, float off_delay
)
{
    int32_t send_index = 0;
    gn10_can::CANFrame frame;
    buffer_append_float32(frame.data.data(), current_rel, 1e5, &send_index);
    buffer_append_float16(frame.data.data(), off_delay, 1e3, &send_index);
    frame.id = controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_REL << 8);
    frame.set_data_length(6);
    can_driver_.send(frame);
}

void VescCAN::comm_can_set_current_brake_rel(uint8_t controller_id, float current_rel)
{
    int32_t send_index = 0;
    gn10_can::CANFrame frame;
    buffer_append_float32(frame.data.data(), current_rel, 1e5, &send_index);
    frame.id = controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE_REL << 8);
    frame.set_data_length(sizeof(float));
    can_driver_.send(frame);
}

void VescCAN::comm_can_set_handbrake(uint8_t controller_id, float current)
{
    int32_t send_index = 0;
    gn10_can::CANFrame frame;
    buffer_append_float32(frame.data.data(), current, 1e3, &send_index);
    frame.id = controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE << 8);
    frame.set_data_length(sizeof(float));
    can_driver_.send(frame);
}

void VescCAN::comm_can_set_handbrake_rel(uint8_t controller_id, float current_rel)
{
    int32_t send_index = 0;
    gn10_can::CANFrame frame;
    buffer_append_float32(frame.data.data(), current_rel, 1e5, &send_index);
    frame.id = controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE_REL << 8);
    frame.set_data_length(sizeof(float));
    can_driver_.send(frame);
}
// 0x1FFFFFFF