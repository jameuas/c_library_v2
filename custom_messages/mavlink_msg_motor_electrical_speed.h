#pragma once
// MESSAGE MOTOR_ELECTRICAL_SPEED PACKING

#define MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED 701


typedef struct __mavlink_motor_electrical_speed_t {
 uint16_t front_right_motor; /*< [rpm] Indicated motor1 RPM from ESC.*/
 uint16_t back_left_motor; /*< [rpm] Indicated motor2 RPM from ESC.*/
 uint16_t front_left_motor; /*< [rpm] Indicated motor3 RPM from ESC.*/
 uint16_t back_right_motor; /*< [rpm] Indicated motor4 RPM from ESC.*/
 uint16_t fixed_wing_motor; /*< [rpm] Indicated motor5 RPM from ESC.*/
} mavlink_motor_electrical_speed_t;

#define MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED_LEN 10
#define MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED_MIN_LEN 10
#define MAVLINK_MSG_ID_701_LEN 10
#define MAVLINK_MSG_ID_701_MIN_LEN 10

#define MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED_CRC 206
#define MAVLINK_MSG_ID_701_CRC 206



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MOTOR_ELECTRICAL_SPEED { \
    701, \
    "MOTOR_ELECTRICAL_SPEED", \
    5, \
    {  { "front_right_motor", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_motor_electrical_speed_t, front_right_motor) }, \
         { "back_left_motor", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_motor_electrical_speed_t, back_left_motor) }, \
         { "front_left_motor", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_motor_electrical_speed_t, front_left_motor) }, \
         { "back_right_motor", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_motor_electrical_speed_t, back_right_motor) }, \
         { "fixed_wing_motor", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_motor_electrical_speed_t, fixed_wing_motor) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MOTOR_ELECTRICAL_SPEED { \
    "MOTOR_ELECTRICAL_SPEED", \
    5, \
    {  { "front_right_motor", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_motor_electrical_speed_t, front_right_motor) }, \
         { "back_left_motor", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_motor_electrical_speed_t, back_left_motor) }, \
         { "front_left_motor", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_motor_electrical_speed_t, front_left_motor) }, \
         { "back_right_motor", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_motor_electrical_speed_t, back_right_motor) }, \
         { "fixed_wing_motor", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_motor_electrical_speed_t, fixed_wing_motor) }, \
         } \
}
#endif

/**
 * @brief Pack a motor_electrical_speed message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param front_right_motor [rpm] Indicated motor1 RPM from ESC.
 * @param back_left_motor [rpm] Indicated motor2 RPM from ESC.
 * @param front_left_motor [rpm] Indicated motor3 RPM from ESC.
 * @param back_right_motor [rpm] Indicated motor4 RPM from ESC.
 * @param fixed_wing_motor [rpm] Indicated motor5 RPM from ESC.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motor_electrical_speed_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t front_right_motor, uint16_t back_left_motor, uint16_t front_left_motor, uint16_t back_right_motor, uint16_t fixed_wing_motor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED_LEN];
    _mav_put_uint16_t(buf, 0, front_right_motor);
    _mav_put_uint16_t(buf, 2, back_left_motor);
    _mav_put_uint16_t(buf, 4, front_left_motor);
    _mav_put_uint16_t(buf, 6, back_right_motor);
    _mav_put_uint16_t(buf, 8, fixed_wing_motor);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED_LEN);
#else
    mavlink_motor_electrical_speed_t packet;
    packet.front_right_motor = front_right_motor;
    packet.back_left_motor = back_left_motor;
    packet.front_left_motor = front_left_motor;
    packet.back_right_motor = back_right_motor;
    packet.fixed_wing_motor = fixed_wing_motor;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED_MIN_LEN, MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED_LEN, MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED_CRC);
}

/**
 * @brief Pack a motor_electrical_speed message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param front_right_motor [rpm] Indicated motor1 RPM from ESC.
 * @param back_left_motor [rpm] Indicated motor2 RPM from ESC.
 * @param front_left_motor [rpm] Indicated motor3 RPM from ESC.
 * @param back_right_motor [rpm] Indicated motor4 RPM from ESC.
 * @param fixed_wing_motor [rpm] Indicated motor5 RPM from ESC.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motor_electrical_speed_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t front_right_motor,uint16_t back_left_motor,uint16_t front_left_motor,uint16_t back_right_motor,uint16_t fixed_wing_motor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED_LEN];
    _mav_put_uint16_t(buf, 0, front_right_motor);
    _mav_put_uint16_t(buf, 2, back_left_motor);
    _mav_put_uint16_t(buf, 4, front_left_motor);
    _mav_put_uint16_t(buf, 6, back_right_motor);
    _mav_put_uint16_t(buf, 8, fixed_wing_motor);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED_LEN);
#else
    mavlink_motor_electrical_speed_t packet;
    packet.front_right_motor = front_right_motor;
    packet.back_left_motor = back_left_motor;
    packet.front_left_motor = front_left_motor;
    packet.back_right_motor = back_right_motor;
    packet.fixed_wing_motor = fixed_wing_motor;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED_MIN_LEN, MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED_LEN, MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED_CRC);
}

/**
 * @brief Encode a motor_electrical_speed struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param motor_electrical_speed C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_motor_electrical_speed_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_motor_electrical_speed_t* motor_electrical_speed)
{
    return mavlink_msg_motor_electrical_speed_pack(system_id, component_id, msg, motor_electrical_speed->front_right_motor, motor_electrical_speed->back_left_motor, motor_electrical_speed->front_left_motor, motor_electrical_speed->back_right_motor, motor_electrical_speed->fixed_wing_motor);
}

/**
 * @brief Encode a motor_electrical_speed struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param motor_electrical_speed C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_motor_electrical_speed_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_motor_electrical_speed_t* motor_electrical_speed)
{
    return mavlink_msg_motor_electrical_speed_pack_chan(system_id, component_id, chan, msg, motor_electrical_speed->front_right_motor, motor_electrical_speed->back_left_motor, motor_electrical_speed->front_left_motor, motor_electrical_speed->back_right_motor, motor_electrical_speed->fixed_wing_motor);
}

/**
 * @brief Send a motor_electrical_speed message
 * @param chan MAVLink channel to send the message
 *
 * @param front_right_motor [rpm] Indicated motor1 RPM from ESC.
 * @param back_left_motor [rpm] Indicated motor2 RPM from ESC.
 * @param front_left_motor [rpm] Indicated motor3 RPM from ESC.
 * @param back_right_motor [rpm] Indicated motor4 RPM from ESC.
 * @param fixed_wing_motor [rpm] Indicated motor5 RPM from ESC.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_motor_electrical_speed_send(mavlink_channel_t chan, uint16_t front_right_motor, uint16_t back_left_motor, uint16_t front_left_motor, uint16_t back_right_motor, uint16_t fixed_wing_motor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED_LEN];
    _mav_put_uint16_t(buf, 0, front_right_motor);
    _mav_put_uint16_t(buf, 2, back_left_motor);
    _mav_put_uint16_t(buf, 4, front_left_motor);
    _mav_put_uint16_t(buf, 6, back_right_motor);
    _mav_put_uint16_t(buf, 8, fixed_wing_motor);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED, buf, MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED_MIN_LEN, MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED_LEN, MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED_CRC);
#else
    mavlink_motor_electrical_speed_t packet;
    packet.front_right_motor = front_right_motor;
    packet.back_left_motor = back_left_motor;
    packet.front_left_motor = front_left_motor;
    packet.back_right_motor = back_right_motor;
    packet.fixed_wing_motor = fixed_wing_motor;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED, (const char *)&packet, MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED_MIN_LEN, MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED_LEN, MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED_CRC);
#endif
}

/**
 * @brief Send a motor_electrical_speed message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_motor_electrical_speed_send_struct(mavlink_channel_t chan, const mavlink_motor_electrical_speed_t* motor_electrical_speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_motor_electrical_speed_send(chan, motor_electrical_speed->front_right_motor, motor_electrical_speed->back_left_motor, motor_electrical_speed->front_left_motor, motor_electrical_speed->back_right_motor, motor_electrical_speed->fixed_wing_motor);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED, (const char *)motor_electrical_speed, MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED_MIN_LEN, MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED_LEN, MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED_CRC);
#endif
}

#if MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_motor_electrical_speed_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t front_right_motor, uint16_t back_left_motor, uint16_t front_left_motor, uint16_t back_right_motor, uint16_t fixed_wing_motor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, front_right_motor);
    _mav_put_uint16_t(buf, 2, back_left_motor);
    _mav_put_uint16_t(buf, 4, front_left_motor);
    _mav_put_uint16_t(buf, 6, back_right_motor);
    _mav_put_uint16_t(buf, 8, fixed_wing_motor);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED, buf, MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED_MIN_LEN, MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED_LEN, MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED_CRC);
#else
    mavlink_motor_electrical_speed_t *packet = (mavlink_motor_electrical_speed_t *)msgbuf;
    packet->front_right_motor = front_right_motor;
    packet->back_left_motor = back_left_motor;
    packet->front_left_motor = front_left_motor;
    packet->back_right_motor = back_right_motor;
    packet->fixed_wing_motor = fixed_wing_motor;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED, (const char *)packet, MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED_MIN_LEN, MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED_LEN, MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED_CRC);
#endif
}
#endif

#endif

// MESSAGE MOTOR_ELECTRICAL_SPEED UNPACKING


/**
 * @brief Get field front_right_motor from motor_electrical_speed message
 *
 * @return [rpm] Indicated motor1 RPM from ESC.
 */
static inline uint16_t mavlink_msg_motor_electrical_speed_get_front_right_motor(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field back_left_motor from motor_electrical_speed message
 *
 * @return [rpm] Indicated motor2 RPM from ESC.
 */
static inline uint16_t mavlink_msg_motor_electrical_speed_get_back_left_motor(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field front_left_motor from motor_electrical_speed message
 *
 * @return [rpm] Indicated motor3 RPM from ESC.
 */
static inline uint16_t mavlink_msg_motor_electrical_speed_get_front_left_motor(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field back_right_motor from motor_electrical_speed message
 *
 * @return [rpm] Indicated motor4 RPM from ESC.
 */
static inline uint16_t mavlink_msg_motor_electrical_speed_get_back_right_motor(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field fixed_wing_motor from motor_electrical_speed message
 *
 * @return [rpm] Indicated motor5 RPM from ESC.
 */
static inline uint16_t mavlink_msg_motor_electrical_speed_get_fixed_wing_motor(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Decode a motor_electrical_speed message into a struct
 *
 * @param msg The message to decode
 * @param motor_electrical_speed C-struct to decode the message contents into
 */
static inline void mavlink_msg_motor_electrical_speed_decode(const mavlink_message_t* msg, mavlink_motor_electrical_speed_t* motor_electrical_speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    motor_electrical_speed->front_right_motor = mavlink_msg_motor_electrical_speed_get_front_right_motor(msg);
    motor_electrical_speed->back_left_motor = mavlink_msg_motor_electrical_speed_get_back_left_motor(msg);
    motor_electrical_speed->front_left_motor = mavlink_msg_motor_electrical_speed_get_front_left_motor(msg);
    motor_electrical_speed->back_right_motor = mavlink_msg_motor_electrical_speed_get_back_right_motor(msg);
    motor_electrical_speed->fixed_wing_motor = mavlink_msg_motor_electrical_speed_get_fixed_wing_motor(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED_LEN? msg->len : MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED_LEN;
        memset(motor_electrical_speed, 0, MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED_LEN);
    memcpy(motor_electrical_speed, _MAV_PAYLOAD(msg), len);
#endif
}
