#include "app/vesc_can.hpp"

VescCAN::VescCAN(gn10_can::drivers::ICANDriver& can_driver) : can_driver_(can_driver) {}

void VescCAN::update()
{
    gn10_can::CANFrame out_frame;
    can_driver_.receive(out_frame);
    uint8_t packet_id = (out_frame.id >> 8) & 0xFF;

    // rpm feedback
    if (packet_id == CAN_PACKET_STATUS_1) {
        parse_status1(out_frame.data.data(), status1_);
    }
    // taco feedback
    if (packet_id == CAN_PACKET_STATUS_5) {  // 追加
        parse_status5(out_frame.data.data(), status5_);
    }
}

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

void VescCAN::buffer_append_int32(uint8_t* buffer, int32_t number, int32_t* index)
{
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void VescCAN::comm_can_set_erpm(uint8_t controller_id, float rpm)
{
    int32_t data_length_code = 0;
    gn10_can::CANFrame frame;
    buffer_append_int32(frame.data.data(), (int32_t)rpm, &data_length_code);
    frame.id = controller_id | ((uint32_t)CAN_PACKET_SET_RPM << 8);
    frame.set_data_length(data_length_code);
    frame.is_extended = true;
    can_driver_.send(frame);
}
