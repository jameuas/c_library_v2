#pragma once
// MESSAGE SYS_IDEN_DATA PACKING

#define MAVLINK_MSG_ID_SYS_IDEN_DATA 700


typedef struct __mavlink_sys_iden_data_t {
 float p; /*< [deg/s] Angular rates about the body frame X-axis (roll)*/
 float q; /*< [deg/s] Angular rates about the body frame Y-axis (pitch)*/
 float r; /*< [deg/s] Angular rates about the body frame Z-axis (yaw)*/
 float ax; /*< [m/s/s] Acceleration (including gravity) in the body frame X-axis*/
 float ay; /*< [m/s/s] Acceleration (including gravity) in the body frame Y-axis*/
 float az; /*< [m/s/s] Acceleration (including gravity) in the body frame Z-axis*/
 float v; /*< [m/s] Airspeed*/
 float alpha; /*< [deg] Angle of Attack*/
 float beta; /*< [deg] Angle of Side Slip*/
 float ail; /*< [deg] Control input (roll)*/
 float ele; /*< [deg] Control input (pitch)*/
 float rud; /*< [deg] Control input (yaw)*/
} mavlink_sys_iden_data_t;

#define MAVLINK_MSG_ID_SYS_IDEN_DATA_LEN 48
#define MAVLINK_MSG_ID_SYS_IDEN_DATA_MIN_LEN 48
#define MAVLINK_MSG_ID_700_LEN 48
#define MAVLINK_MSG_ID_700_MIN_LEN 48

#define MAVLINK_MSG_ID_SYS_IDEN_DATA_CRC 165
#define MAVLINK_MSG_ID_700_CRC 165



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SYS_IDEN_DATA { \
    700, \
    "SYS_IDEN_DATA", \
    12, \
    {  { "p", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_sys_iden_data_t, p) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_sys_iden_data_t, q) }, \
         { "r", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_sys_iden_data_t, r) }, \
         { "ax", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_sys_iden_data_t, ax) }, \
         { "ay", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_sys_iden_data_t, ay) }, \
         { "az", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_sys_iden_data_t, az) }, \
         { "v", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_sys_iden_data_t, v) }, \
         { "alpha", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_sys_iden_data_t, alpha) }, \
         { "beta", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_sys_iden_data_t, beta) }, \
         { "ail", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_sys_iden_data_t, ail) }, \
         { "ele", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_sys_iden_data_t, ele) }, \
         { "rud", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_sys_iden_data_t, rud) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SYS_IDEN_DATA { \
    "SYS_IDEN_DATA", \
    12, \
    {  { "p", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_sys_iden_data_t, p) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_sys_iden_data_t, q) }, \
         { "r", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_sys_iden_data_t, r) }, \
         { "ax", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_sys_iden_data_t, ax) }, \
         { "ay", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_sys_iden_data_t, ay) }, \
         { "az", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_sys_iden_data_t, az) }, \
         { "v", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_sys_iden_data_t, v) }, \
         { "alpha", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_sys_iden_data_t, alpha) }, \
         { "beta", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_sys_iden_data_t, beta) }, \
         { "ail", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_sys_iden_data_t, ail) }, \
         { "ele", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_sys_iden_data_t, ele) }, \
         { "rud", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_sys_iden_data_t, rud) }, \
         } \
}
#endif

/**
 * @brief Pack a sys_iden_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param p [deg/s] Angular rates about the body frame X-axis (roll)
 * @param q [deg/s] Angular rates about the body frame Y-axis (pitch)
 * @param r [deg/s] Angular rates about the body frame Z-axis (yaw)
 * @param ax [m/s/s] Acceleration (including gravity) in the body frame X-axis
 * @param ay [m/s/s] Acceleration (including gravity) in the body frame Y-axis
 * @param az [m/s/s] Acceleration (including gravity) in the body frame Z-axis
 * @param v [m/s] Airspeed
 * @param alpha [deg] Angle of Attack
 * @param beta [deg] Angle of Side Slip
 * @param ail [deg] Control input (roll)
 * @param ele [deg] Control input (pitch)
 * @param rud [deg] Control input (yaw)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sys_iden_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float p, float q, float r, float ax, float ay, float az, float v, float alpha, float beta, float ail, float ele, float rud)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SYS_IDEN_DATA_LEN];
    _mav_put_float(buf, 0, p);
    _mav_put_float(buf, 4, q);
    _mav_put_float(buf, 8, r);
    _mav_put_float(buf, 12, ax);
    _mav_put_float(buf, 16, ay);
    _mav_put_float(buf, 20, az);
    _mav_put_float(buf, 24, v);
    _mav_put_float(buf, 28, alpha);
    _mav_put_float(buf, 32, beta);
    _mav_put_float(buf, 36, ail);
    _mav_put_float(buf, 40, ele);
    _mav_put_float(buf, 44, rud);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SYS_IDEN_DATA_LEN);
#else
    mavlink_sys_iden_data_t packet;
    packet.p = p;
    packet.q = q;
    packet.r = r;
    packet.ax = ax;
    packet.ay = ay;
    packet.az = az;
    packet.v = v;
    packet.alpha = alpha;
    packet.beta = beta;
    packet.ail = ail;
    packet.ele = ele;
    packet.rud = rud;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SYS_IDEN_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SYS_IDEN_DATA;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SYS_IDEN_DATA_MIN_LEN, MAVLINK_MSG_ID_SYS_IDEN_DATA_LEN, MAVLINK_MSG_ID_SYS_IDEN_DATA_CRC);
}

/**
 * @brief Pack a sys_iden_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param p [deg/s] Angular rates about the body frame X-axis (roll)
 * @param q [deg/s] Angular rates about the body frame Y-axis (pitch)
 * @param r [deg/s] Angular rates about the body frame Z-axis (yaw)
 * @param ax [m/s/s] Acceleration (including gravity) in the body frame X-axis
 * @param ay [m/s/s] Acceleration (including gravity) in the body frame Y-axis
 * @param az [m/s/s] Acceleration (including gravity) in the body frame Z-axis
 * @param v [m/s] Airspeed
 * @param alpha [deg] Angle of Attack
 * @param beta [deg] Angle of Side Slip
 * @param ail [deg] Control input (roll)
 * @param ele [deg] Control input (pitch)
 * @param rud [deg] Control input (yaw)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sys_iden_data_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               float p, float q, float r, float ax, float ay, float az, float v, float alpha, float beta, float ail, float ele, float rud)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SYS_IDEN_DATA_LEN];
    _mav_put_float(buf, 0, p);
    _mav_put_float(buf, 4, q);
    _mav_put_float(buf, 8, r);
    _mav_put_float(buf, 12, ax);
    _mav_put_float(buf, 16, ay);
    _mav_put_float(buf, 20, az);
    _mav_put_float(buf, 24, v);
    _mav_put_float(buf, 28, alpha);
    _mav_put_float(buf, 32, beta);
    _mav_put_float(buf, 36, ail);
    _mav_put_float(buf, 40, ele);
    _mav_put_float(buf, 44, rud);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SYS_IDEN_DATA_LEN);
#else
    mavlink_sys_iden_data_t packet;
    packet.p = p;
    packet.q = q;
    packet.r = r;
    packet.ax = ax;
    packet.ay = ay;
    packet.az = az;
    packet.v = v;
    packet.alpha = alpha;
    packet.beta = beta;
    packet.ail = ail;
    packet.ele = ele;
    packet.rud = rud;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SYS_IDEN_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SYS_IDEN_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SYS_IDEN_DATA_MIN_LEN, MAVLINK_MSG_ID_SYS_IDEN_DATA_LEN, MAVLINK_MSG_ID_SYS_IDEN_DATA_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SYS_IDEN_DATA_MIN_LEN, MAVLINK_MSG_ID_SYS_IDEN_DATA_LEN);
#endif
}

/**
 * @brief Pack a sys_iden_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param p [deg/s] Angular rates about the body frame X-axis (roll)
 * @param q [deg/s] Angular rates about the body frame Y-axis (pitch)
 * @param r [deg/s] Angular rates about the body frame Z-axis (yaw)
 * @param ax [m/s/s] Acceleration (including gravity) in the body frame X-axis
 * @param ay [m/s/s] Acceleration (including gravity) in the body frame Y-axis
 * @param az [m/s/s] Acceleration (including gravity) in the body frame Z-axis
 * @param v [m/s] Airspeed
 * @param alpha [deg] Angle of Attack
 * @param beta [deg] Angle of Side Slip
 * @param ail [deg] Control input (roll)
 * @param ele [deg] Control input (pitch)
 * @param rud [deg] Control input (yaw)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sys_iden_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float p,float q,float r,float ax,float ay,float az,float v,float alpha,float beta,float ail,float ele,float rud)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SYS_IDEN_DATA_LEN];
    _mav_put_float(buf, 0, p);
    _mav_put_float(buf, 4, q);
    _mav_put_float(buf, 8, r);
    _mav_put_float(buf, 12, ax);
    _mav_put_float(buf, 16, ay);
    _mav_put_float(buf, 20, az);
    _mav_put_float(buf, 24, v);
    _mav_put_float(buf, 28, alpha);
    _mav_put_float(buf, 32, beta);
    _mav_put_float(buf, 36, ail);
    _mav_put_float(buf, 40, ele);
    _mav_put_float(buf, 44, rud);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SYS_IDEN_DATA_LEN);
#else
    mavlink_sys_iden_data_t packet;
    packet.p = p;
    packet.q = q;
    packet.r = r;
    packet.ax = ax;
    packet.ay = ay;
    packet.az = az;
    packet.v = v;
    packet.alpha = alpha;
    packet.beta = beta;
    packet.ail = ail;
    packet.ele = ele;
    packet.rud = rud;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SYS_IDEN_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SYS_IDEN_DATA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SYS_IDEN_DATA_MIN_LEN, MAVLINK_MSG_ID_SYS_IDEN_DATA_LEN, MAVLINK_MSG_ID_SYS_IDEN_DATA_CRC);
}

/**
 * @brief Encode a sys_iden_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sys_iden_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sys_iden_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sys_iden_data_t* sys_iden_data)
{
    return mavlink_msg_sys_iden_data_pack(system_id, component_id, msg, sys_iden_data->p, sys_iden_data->q, sys_iden_data->r, sys_iden_data->ax, sys_iden_data->ay, sys_iden_data->az, sys_iden_data->v, sys_iden_data->alpha, sys_iden_data->beta, sys_iden_data->ail, sys_iden_data->ele, sys_iden_data->rud);
}

/**
 * @brief Encode a sys_iden_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sys_iden_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sys_iden_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sys_iden_data_t* sys_iden_data)
{
    return mavlink_msg_sys_iden_data_pack_chan(system_id, component_id, chan, msg, sys_iden_data->p, sys_iden_data->q, sys_iden_data->r, sys_iden_data->ax, sys_iden_data->ay, sys_iden_data->az, sys_iden_data->v, sys_iden_data->alpha, sys_iden_data->beta, sys_iden_data->ail, sys_iden_data->ele, sys_iden_data->rud);
}

/**
 * @brief Encode a sys_iden_data struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param sys_iden_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sys_iden_data_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_sys_iden_data_t* sys_iden_data)
{
    return mavlink_msg_sys_iden_data_pack_status(system_id, component_id, _status, msg,  sys_iden_data->p, sys_iden_data->q, sys_iden_data->r, sys_iden_data->ax, sys_iden_data->ay, sys_iden_data->az, sys_iden_data->v, sys_iden_data->alpha, sys_iden_data->beta, sys_iden_data->ail, sys_iden_data->ele, sys_iden_data->rud);
}

/**
 * @brief Send a sys_iden_data message
 * @param chan MAVLink channel to send the message
 *
 * @param p [deg/s] Angular rates about the body frame X-axis (roll)
 * @param q [deg/s] Angular rates about the body frame Y-axis (pitch)
 * @param r [deg/s] Angular rates about the body frame Z-axis (yaw)
 * @param ax [m/s/s] Acceleration (including gravity) in the body frame X-axis
 * @param ay [m/s/s] Acceleration (including gravity) in the body frame Y-axis
 * @param az [m/s/s] Acceleration (including gravity) in the body frame Z-axis
 * @param v [m/s] Airspeed
 * @param alpha [deg] Angle of Attack
 * @param beta [deg] Angle of Side Slip
 * @param ail [deg] Control input (roll)
 * @param ele [deg] Control input (pitch)
 * @param rud [deg] Control input (yaw)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sys_iden_data_send(mavlink_channel_t chan, float p, float q, float r, float ax, float ay, float az, float v, float alpha, float beta, float ail, float ele, float rud)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SYS_IDEN_DATA_LEN];
    _mav_put_float(buf, 0, p);
    _mav_put_float(buf, 4, q);
    _mav_put_float(buf, 8, r);
    _mav_put_float(buf, 12, ax);
    _mav_put_float(buf, 16, ay);
    _mav_put_float(buf, 20, az);
    _mav_put_float(buf, 24, v);
    _mav_put_float(buf, 28, alpha);
    _mav_put_float(buf, 32, beta);
    _mav_put_float(buf, 36, ail);
    _mav_put_float(buf, 40, ele);
    _mav_put_float(buf, 44, rud);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYS_IDEN_DATA, buf, MAVLINK_MSG_ID_SYS_IDEN_DATA_MIN_LEN, MAVLINK_MSG_ID_SYS_IDEN_DATA_LEN, MAVLINK_MSG_ID_SYS_IDEN_DATA_CRC);
#else
    mavlink_sys_iden_data_t packet;
    packet.p = p;
    packet.q = q;
    packet.r = r;
    packet.ax = ax;
    packet.ay = ay;
    packet.az = az;
    packet.v = v;
    packet.alpha = alpha;
    packet.beta = beta;
    packet.ail = ail;
    packet.ele = ele;
    packet.rud = rud;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYS_IDEN_DATA, (const char *)&packet, MAVLINK_MSG_ID_SYS_IDEN_DATA_MIN_LEN, MAVLINK_MSG_ID_SYS_IDEN_DATA_LEN, MAVLINK_MSG_ID_SYS_IDEN_DATA_CRC);
#endif
}

/**
 * @brief Send a sys_iden_data message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sys_iden_data_send_struct(mavlink_channel_t chan, const mavlink_sys_iden_data_t* sys_iden_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sys_iden_data_send(chan, sys_iden_data->p, sys_iden_data->q, sys_iden_data->r, sys_iden_data->ax, sys_iden_data->ay, sys_iden_data->az, sys_iden_data->v, sys_iden_data->alpha, sys_iden_data->beta, sys_iden_data->ail, sys_iden_data->ele, sys_iden_data->rud);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYS_IDEN_DATA, (const char *)sys_iden_data, MAVLINK_MSG_ID_SYS_IDEN_DATA_MIN_LEN, MAVLINK_MSG_ID_SYS_IDEN_DATA_LEN, MAVLINK_MSG_ID_SYS_IDEN_DATA_CRC);
#endif
}

#if MAVLINK_MSG_ID_SYS_IDEN_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sys_iden_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float p, float q, float r, float ax, float ay, float az, float v, float alpha, float beta, float ail, float ele, float rud)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, p);
    _mav_put_float(buf, 4, q);
    _mav_put_float(buf, 8, r);
    _mav_put_float(buf, 12, ax);
    _mav_put_float(buf, 16, ay);
    _mav_put_float(buf, 20, az);
    _mav_put_float(buf, 24, v);
    _mav_put_float(buf, 28, alpha);
    _mav_put_float(buf, 32, beta);
    _mav_put_float(buf, 36, ail);
    _mav_put_float(buf, 40, ele);
    _mav_put_float(buf, 44, rud);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYS_IDEN_DATA, buf, MAVLINK_MSG_ID_SYS_IDEN_DATA_MIN_LEN, MAVLINK_MSG_ID_SYS_IDEN_DATA_LEN, MAVLINK_MSG_ID_SYS_IDEN_DATA_CRC);
#else
    mavlink_sys_iden_data_t *packet = (mavlink_sys_iden_data_t *)msgbuf;
    packet->p = p;
    packet->q = q;
    packet->r = r;
    packet->ax = ax;
    packet->ay = ay;
    packet->az = az;
    packet->v = v;
    packet->alpha = alpha;
    packet->beta = beta;
    packet->ail = ail;
    packet->ele = ele;
    packet->rud = rud;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYS_IDEN_DATA, (const char *)packet, MAVLINK_MSG_ID_SYS_IDEN_DATA_MIN_LEN, MAVLINK_MSG_ID_SYS_IDEN_DATA_LEN, MAVLINK_MSG_ID_SYS_IDEN_DATA_CRC);
#endif
}
#endif

#endif

// MESSAGE SYS_IDEN_DATA UNPACKING


/**
 * @brief Get field p from sys_iden_data message
 *
 * @return [deg/s] Angular rates about the body frame X-axis (roll)
 */
static inline float mavlink_msg_sys_iden_data_get_p(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field q from sys_iden_data message
 *
 * @return [deg/s] Angular rates about the body frame Y-axis (pitch)
 */
static inline float mavlink_msg_sys_iden_data_get_q(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field r from sys_iden_data message
 *
 * @return [deg/s] Angular rates about the body frame Z-axis (yaw)
 */
static inline float mavlink_msg_sys_iden_data_get_r(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field ax from sys_iden_data message
 *
 * @return [m/s/s] Acceleration (including gravity) in the body frame X-axis
 */
static inline float mavlink_msg_sys_iden_data_get_ax(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field ay from sys_iden_data message
 *
 * @return [m/s/s] Acceleration (including gravity) in the body frame Y-axis
 */
static inline float mavlink_msg_sys_iden_data_get_ay(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field az from sys_iden_data message
 *
 * @return [m/s/s] Acceleration (including gravity) in the body frame Z-axis
 */
static inline float mavlink_msg_sys_iden_data_get_az(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field v from sys_iden_data message
 *
 * @return [m/s] Airspeed
 */
static inline float mavlink_msg_sys_iden_data_get_v(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field alpha from sys_iden_data message
 *
 * @return [deg] Angle of Attack
 */
static inline float mavlink_msg_sys_iden_data_get_alpha(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field beta from sys_iden_data message
 *
 * @return [deg] Angle of Side Slip
 */
static inline float mavlink_msg_sys_iden_data_get_beta(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field ail from sys_iden_data message
 *
 * @return [deg] Control input (roll)
 */
static inline float mavlink_msg_sys_iden_data_get_ail(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field ele from sys_iden_data message
 *
 * @return [deg] Control input (pitch)
 */
static inline float mavlink_msg_sys_iden_data_get_ele(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field rud from sys_iden_data message
 *
 * @return [deg] Control input (yaw)
 */
static inline float mavlink_msg_sys_iden_data_get_rud(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Decode a sys_iden_data message into a struct
 *
 * @param msg The message to decode
 * @param sys_iden_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_sys_iden_data_decode(const mavlink_message_t* msg, mavlink_sys_iden_data_t* sys_iden_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    sys_iden_data->p = mavlink_msg_sys_iden_data_get_p(msg);
    sys_iden_data->q = mavlink_msg_sys_iden_data_get_q(msg);
    sys_iden_data->r = mavlink_msg_sys_iden_data_get_r(msg);
    sys_iden_data->ax = mavlink_msg_sys_iden_data_get_ax(msg);
    sys_iden_data->ay = mavlink_msg_sys_iden_data_get_ay(msg);
    sys_iden_data->az = mavlink_msg_sys_iden_data_get_az(msg);
    sys_iden_data->v = mavlink_msg_sys_iden_data_get_v(msg);
    sys_iden_data->alpha = mavlink_msg_sys_iden_data_get_alpha(msg);
    sys_iden_data->beta = mavlink_msg_sys_iden_data_get_beta(msg);
    sys_iden_data->ail = mavlink_msg_sys_iden_data_get_ail(msg);
    sys_iden_data->ele = mavlink_msg_sys_iden_data_get_ele(msg);
    sys_iden_data->rud = mavlink_msg_sys_iden_data_get_rud(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SYS_IDEN_DATA_LEN? msg->len : MAVLINK_MSG_ID_SYS_IDEN_DATA_LEN;
        memset(sys_iden_data, 0, MAVLINK_MSG_ID_SYS_IDEN_DATA_LEN);
    memcpy(sys_iden_data, _MAV_PAYLOAD(msg), len);
#endif
}
