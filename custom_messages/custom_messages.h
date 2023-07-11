/** @file
 *  @brief MAVLink comm protocol generated from custom_messages.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_CUSTOM_MESSAGES_H
#define MAVLINK_CUSTOM_MESSAGES_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_CUSTOM_MESSAGES.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#define MAVLINK_CUSTOM_MESSAGES_XML_HASH -6082012244640393391

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{700, 0, 24, 24, 0, 0, 0}, {701, 206, 10, 10, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_CUSTOM_MESSAGES

// ENUM DEFINITIONS



// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_airdata_boom.h"
#include "./mavlink_msg_motor_electrical_speed.h"

// base include



#if MAVLINK_CUSTOM_MESSAGES_XML_HASH == MAVLINK_PRIMARY_XML_HASH
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_AIRDATA_BOOM, MAVLINK_MESSAGE_INFO_MOTOR_ELECTRICAL_SPEED}
# define MAVLINK_MESSAGE_NAMES {{ "AIRDATA_BOOM", 700 }, { "MOTOR_ELECTRICAL_SPEED", 701 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_CUSTOM_MESSAGES_H
