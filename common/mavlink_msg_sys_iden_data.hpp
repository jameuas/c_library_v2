// MESSAGE SYS_IDEN_DATA support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief SYS_IDEN_DATA message
 *
 * System Identification Data message.
 */
struct SYS_IDEN_DATA : mavlink::Message {
    static constexpr msgid_t MSG_ID = 700;
    static constexpr size_t LENGTH = 48;
    static constexpr size_t MIN_LENGTH = 48;
    static constexpr uint8_t CRC_EXTRA = 165;
    static constexpr auto NAME = "SYS_IDEN_DATA";


    float p; /*< [deg/s] Angular rates about the body frame X-axis (roll) */
    float q; /*< [deg/s] Angular rates about the body frame Y-axis (pitch) */
    float r; /*< [deg/s] Angular rates about the body frame Z-axis (yaw) */
    float ax; /*< [m/s/s] Acceleration (including gravity) in the body frame X-axis */
    float ay; /*< [m/s/s] Acceleration (including gravity) in the body frame Y-axis */
    float az; /*< [m/s/s] Acceleration (including gravity) in the body frame Z-axis */
    float v; /*< [m/s] Airspeed */
    float alpha; /*< [deg] Angle of Attack */
    float beta; /*< [deg] Angle of Side Slip */
    float ail; /*< [deg] Control input (roll) */
    float ele; /*< [deg] Control input (pitch) */
    float rud; /*< [deg] Control input (yaw) */


    inline std::string get_name(void) const override
    {
            return NAME;
    }

    inline Info get_message_info(void) const override
    {
            return { MSG_ID, LENGTH, MIN_LENGTH, CRC_EXTRA };
    }

    inline std::string to_yaml(void) const override
    {
        std::stringstream ss;

        ss << NAME << ":" << std::endl;
        ss << "  p: " << p << std::endl;
        ss << "  q: " << q << std::endl;
        ss << "  r: " << r << std::endl;
        ss << "  ax: " << ax << std::endl;
        ss << "  ay: " << ay << std::endl;
        ss << "  az: " << az << std::endl;
        ss << "  v: " << v << std::endl;
        ss << "  alpha: " << alpha << std::endl;
        ss << "  beta: " << beta << std::endl;
        ss << "  ail: " << ail << std::endl;
        ss << "  ele: " << ele << std::endl;
        ss << "  rud: " << rud << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << p;                             // offset: 0
        map << q;                             // offset: 4
        map << r;                             // offset: 8
        map << ax;                            // offset: 12
        map << ay;                            // offset: 16
        map << az;                            // offset: 20
        map << v;                             // offset: 24
        map << alpha;                         // offset: 28
        map << beta;                          // offset: 32
        map << ail;                           // offset: 36
        map << ele;                           // offset: 40
        map << rud;                           // offset: 44
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> p;                             // offset: 0
        map >> q;                             // offset: 4
        map >> r;                             // offset: 8
        map >> ax;                            // offset: 12
        map >> ay;                            // offset: 16
        map >> az;                            // offset: 20
        map >> v;                             // offset: 24
        map >> alpha;                         // offset: 28
        map >> beta;                          // offset: 32
        map >> ail;                           // offset: 36
        map >> ele;                           // offset: 40
        map >> rud;                           // offset: 44
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
