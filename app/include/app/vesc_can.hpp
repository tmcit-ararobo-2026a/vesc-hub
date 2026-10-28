#pragma once
#include <cstdint>

#include "fdcan.h"
#include "gn10_can/drivers/can_driver_interface.hpp"

typedef enum {
    CAN_PACKET_SET_ERPM = 3,
    CAN_PACKET_STATUS_1 = 9,
    CAN_PACKET_STATUS_5 = 27,
} CAN_PACKET_ID;

class VescCAN
{
private:
    struct VescStatus1 {
        int32_t erpm;   // [rpm]
        float current;  //[A]
        float duty;     //[%]
    };

    struct VescStatus5 {
        int32_t taco;   // [rpm]
        float voltage;  //[V]
    };

    VescStatus1 status1_;
    VescStatus5 status5_;

    /**
     * @details this function is used to send commands to the VESC.
     */
    void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t* index);

    /**
     * @brief assign the vesc feedback value (rpm,current,duty)
     */
    void parse_status1(uint8_t* data, VescStatus1& status);

    /**
     * @brief assign the vesc feedback value (tacometer,voltage)
     */
    void parse_status5(uint8_t* data, VescStatus5& status);

public:
    explicit VescCAN(gn10_can::drivers::ICANDriver& can_driver);

    void update();

    /**
     * @brief send erpm to vesc
     * @param controller_id vesc controller id
     * @param erpm erpm  ※ erpm = rpm * (motor poles / 2)
     */
    void comm_can_set_erpm(uint8_t controller_id, float erpm);

    /** vesc feedback get function**/
    int32_t get_erpm() const
    {
        return status1_.erpm;
    }

    float get_current() const
    {
        return status1_.current;
    }

    float get_voltage() const
    {
        return status5_.voltage;
    }

    gn10_can::drivers::ICANDriver& can_driver_;
};