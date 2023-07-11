#pragma once
// MESSAGE AIRDATA_BOOM PACKING

#define MAVLINK_MSG_ID_AIRDATA_BOOM 700


typedef struct __mavlink_airdata_boom_t {
 float indicated_airspeed; /*< [m/s] Indicated Airspeed.*/
 float calibrated_airspeed; /*< [m/s] Calibrated Airspeed.*/
 float true_airspeed; /*< [m/s] True Airspeed.*/
 float aoa; /*< [deg] Angle of Attack.*/
 float aos; /*< [deg] Angle of Side Slip.*/
 float air_temperature; /*< [degC] Air temperature (degrees C) from airspeed sensor.*/
} mavlink_airdata_boom_t;

#define MAVLINK_MSG_ID_AIRDATA_BOOM_LEN 24
#define MAVLINK_MSG_ID_AIRDATA_BOOM_MIN_LEN 24
#define MAVLINK_MSG_ID_700_LEN 24
#define MAVLINK_MSG_ID_700_MIN_LEN 24

#define MAVLINK_MSG_ID_AIRDATA_BOOM_CRC 0
#define MAVLINK_MSG_ID_700_CRC 0



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_AIRDATA_BOOM { \
    700, \
    "AIRDATA_BOOM", \
    6, \
    {  { "indicated_airspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_airdata_boom_t, indicated_airspeed) }, \
         { "calibrated_airspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_airdata_boom_t, calibrated_airspeed) }, \
         { "true_airspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_airdata_boom_t, true_airspeed) }, \
         { "aoa", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_airdata_boom_t, aoa) }, \
         { "aos", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_airdata_boom_t, aos) }, \
         { "air_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_airdata_boom_t, air_temperature) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_AIRDATA_BOOM { \
    "AIRDATA_BOOM", \
    6, \
    {  { "indicated_airspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_airdata_boom_t, indicated_airspeed) }, \
         { "calibrated_airspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_airdata_boom_t, calibrated_airspeed) }, \
         { "true_airspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_airdata_boom_t, true_airspeed) }, \
         { "aoa", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_airdata_boom_t, aoa) }, \
         { "aos", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_airdata_boom_t, aos) }, \
         { "air_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_airdata_boom_t, air_temperature) }, \
         } \
}
#endif

/**
 * @brief Pack a airdata_boom message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param indicated_airspeed [m/s] Indicated Airspeed.
 * @param calibrated_airspeed [m/s] Calibrated Airspeed.
 * @param true_airspeed [m/s] True Airspeed.
 * @param aoa [deg] Angle of Attack.
 * @param aos [deg] Angle of Side Slip.
 * @param air_temperature [degC] Air temperature (degrees C) from airspeed sensor.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_airdata_boom_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float indicated_airspeed, float calibrated_airspeed, float true_airspeed, float aoa, float aos, float air_temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AIRDATA_BOOM_LEN];
    _mav_put_float(buf, 0, indicated_airspeed);
    _mav_put_float(buf, 4, calibrated_airspeed);
    _mav_put_float(buf, 8, true_airspeed);
    _mav_put_float(buf, 12, aoa);
    _mav_put_float(buf, 16, aos);
    _mav_put_float(buf, 20, air_temperature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AIRDATA_BOOM_LEN);
#else
    mavlink_airdata_boom_t packet;
    packet.indicated_airspeed = indicated_airspeed;
    packet.calibrated_airspeed = calibrated_airspeed;
    packet.true_airspeed = true_airspeed;
    packet.aoa = aoa;
    packet.aos = aos;
    packet.air_temperature = air_temperature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AIRDATA_BOOM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AIRDATA_BOOM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AIRDATA_BOOM_MIN_LEN, MAVLINK_MSG_ID_AIRDATA_BOOM_LEN, MAVLINK_MSG_ID_AIRDATA_BOOM_CRC);
}

/**
 * @brief Pack a airdata_boom message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param indicated_airspeed [m/s] Indicated Airspeed.
 * @param calibrated_airspeed [m/s] Calibrated Airspeed.
 * @param true_airspeed [m/s] True Airspeed.
 * @param aoa [deg] Angle of Attack.
 * @param aos [deg] Angle of Side Slip.
 * @param air_temperature [degC] Air temperature (degrees C) from airspeed sensor.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_airdata_boom_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float indicated_airspeed,float calibrated_airspeed,float true_airspeed,float aoa,float aos,float air_temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AIRDATA_BOOM_LEN];
    _mav_put_float(buf, 0, indicated_airspeed);
    _mav_put_float(buf, 4, calibrated_airspeed);
    _mav_put_float(buf, 8, true_airspeed);
    _mav_put_float(buf, 12, aoa);
    _mav_put_float(buf, 16, aos);
    _mav_put_float(buf, 20, air_temperature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AIRDATA_BOOM_LEN);
#else
    mavlink_airdata_boom_t packet;
    packet.indicated_airspeed = indicated_airspeed;
    packet.calibrated_airspeed = calibrated_airspeed;
    packet.true_airspeed = true_airspeed;
    packet.aoa = aoa;
    packet.aos = aos;
    packet.air_temperature = air_temperature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AIRDATA_BOOM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AIRDATA_BOOM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AIRDATA_BOOM_MIN_LEN, MAVLINK_MSG_ID_AIRDATA_BOOM_LEN, MAVLINK_MSG_ID_AIRDATA_BOOM_CRC);
}

/**
 * @brief Encode a airdata_boom struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param airdata_boom C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_airdata_boom_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_airdata_boom_t* airdata_boom)
{
    return mavlink_msg_airdata_boom_pack(system_id, component_id, msg, airdata_boom->indicated_airspeed, airdata_boom->calibrated_airspeed, airdata_boom->true_airspeed, airdata_boom->aoa, airdata_boom->aos, airdata_boom->air_temperature);
}

/**
 * @brief Encode a airdata_boom struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param airdata_boom C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_airdata_boom_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_airdata_boom_t* airdata_boom)
{
    return mavlink_msg_airdata_boom_pack_chan(system_id, component_id, chan, msg, airdata_boom->indicated_airspeed, airdata_boom->calibrated_airspeed, airdata_boom->true_airspeed, airdata_boom->aoa, airdata_boom->aos, airdata_boom->air_temperature);
}

/**
 * @brief Send a airdata_boom message
 * @param chan MAVLink channel to send the message
 *
 * @param indicated_airspeed [m/s] Indicated Airspeed.
 * @param calibrated_airspeed [m/s] Calibrated Airspeed.
 * @param true_airspeed [m/s] True Airspeed.
 * @param aoa [deg] Angle of Attack.
 * @param aos [deg] Angle of Side Slip.
 * @param air_temperature [degC] Air temperature (degrees C) from airspeed sensor.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_airdata_boom_send(mavlink_channel_t chan, float indicated_airspeed, float calibrated_airspeed, float true_airspeed, float aoa, float aos, float air_temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AIRDATA_BOOM_LEN];
    _mav_put_float(buf, 0, indicated_airspeed);
    _mav_put_float(buf, 4, calibrated_airspeed);
    _mav_put_float(buf, 8, true_airspeed);
    _mav_put_float(buf, 12, aoa);
    _mav_put_float(buf, 16, aos);
    _mav_put_float(buf, 20, air_temperature);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRDATA_BOOM, buf, MAVLINK_MSG_ID_AIRDATA_BOOM_MIN_LEN, MAVLINK_MSG_ID_AIRDATA_BOOM_LEN, MAVLINK_MSG_ID_AIRDATA_BOOM_CRC);
#else
    mavlink_airdata_boom_t packet;
    packet.indicated_airspeed = indicated_airspeed;
    packet.calibrated_airspeed = calibrated_airspeed;
    packet.true_airspeed = true_airspeed;
    packet.aoa = aoa;
    packet.aos = aos;
    packet.air_temperature = air_temperature;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRDATA_BOOM, (const char *)&packet, MAVLINK_MSG_ID_AIRDATA_BOOM_MIN_LEN, MAVLINK_MSG_ID_AIRDATA_BOOM_LEN, MAVLINK_MSG_ID_AIRDATA_BOOM_CRC);
#endif
}

/**
 * @brief Send a airdata_boom message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_airdata_boom_send_struct(mavlink_channel_t chan, const mavlink_airdata_boom_t* airdata_boom)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_airdata_boom_send(chan, airdata_boom->indicated_airspeed, airdata_boom->calibrated_airspeed, airdata_boom->true_airspeed, airdata_boom->aoa, airdata_boom->aos, airdata_boom->air_temperature);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRDATA_BOOM, (const char *)airdata_boom, MAVLINK_MSG_ID_AIRDATA_BOOM_MIN_LEN, MAVLINK_MSG_ID_AIRDATA_BOOM_LEN, MAVLINK_MSG_ID_AIRDATA_BOOM_CRC);
#endif
}

#if MAVLINK_MSG_ID_AIRDATA_BOOM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_airdata_boom_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float indicated_airspeed, float calibrated_airspeed, float true_airspeed, float aoa, float aos, float air_temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, indicated_airspeed);
    _mav_put_float(buf, 4, calibrated_airspeed);
    _mav_put_float(buf, 8, true_airspeed);
    _mav_put_float(buf, 12, aoa);
    _mav_put_float(buf, 16, aos);
    _mav_put_float(buf, 20, air_temperature);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRDATA_BOOM, buf, MAVLINK_MSG_ID_AIRDATA_BOOM_MIN_LEN, MAVLINK_MSG_ID_AIRDATA_BOOM_LEN, MAVLINK_MSG_ID_AIRDATA_BOOM_CRC);
#else
    mavlink_airdata_boom_t *packet = (mavlink_airdata_boom_t *)msgbuf;
    packet->indicated_airspeed = indicated_airspeed;
    packet->calibrated_airspeed = calibrated_airspeed;
    packet->true_airspeed = true_airspeed;
    packet->aoa = aoa;
    packet->aos = aos;
    packet->air_temperature = air_temperature;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRDATA_BOOM, (const char *)packet, MAVLINK_MSG_ID_AIRDATA_BOOM_MIN_LEN, MAVLINK_MSG_ID_AIRDATA_BOOM_LEN, MAVLINK_MSG_ID_AIRDATA_BOOM_CRC);
#endif
}
#endif

#endif

// MESSAGE AIRDATA_BOOM UNPACKING


/**
 * @brief Get field indicated_airspeed from airdata_boom message
 *
 * @return [m/s] Indicated Airspeed.
 */
static inline float mavlink_msg_airdata_boom_get_indicated_airspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field calibrated_airspeed from airdata_boom message
 *
 * @return [m/s] Calibrated Airspeed.
 */
static inline float mavlink_msg_airdata_boom_get_calibrated_airspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field true_airspeed from airdata_boom message
 *
 * @return [m/s] True Airspeed.
 */
static inline float mavlink_msg_airdata_boom_get_true_airspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field aoa from airdata_boom message
 *
 * @return [deg] Angle of Attack.
 */
static inline float mavlink_msg_airdata_boom_get_aoa(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field aos from airdata_boom message
 *
 * @return [deg] Angle of Side Slip.
 */
static inline float mavlink_msg_airdata_boom_get_aos(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field air_temperature from airdata_boom message
 *
 * @return [degC] Air temperature (degrees C) from airspeed sensor.
 */
static inline float mavlink_msg_airdata_boom_get_air_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a airdata_boom message into a struct
 *
 * @param msg The message to decode
 * @param airdata_boom C-struct to decode the message contents into
 */
static inline void mavlink_msg_airdata_boom_decode(const mavlink_message_t* msg, mavlink_airdata_boom_t* airdata_boom)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    airdata_boom->indicated_airspeed = mavlink_msg_airdata_boom_get_indicated_airspeed(msg);
    airdata_boom->calibrated_airspeed = mavlink_msg_airdata_boom_get_calibrated_airspeed(msg);
    airdata_boom->true_airspeed = mavlink_msg_airdata_boom_get_true_airspeed(msg);
    airdata_boom->aoa = mavlink_msg_airdata_boom_get_aoa(msg);
    airdata_boom->aos = mavlink_msg_airdata_boom_get_aos(msg);
    airdata_boom->air_temperature = mavlink_msg_airdata_boom_get_air_temperature(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_AIRDATA_BOOM_LEN? msg->len : MAVLINK_MSG_ID_AIRDATA_BOOM_LEN;
        memset(airdata_boom, 0, MAVLINK_MSG_ID_AIRDATA_BOOM_LEN);
    memcpy(airdata_boom, _MAV_PAYLOAD(msg), len);
#endif
}
