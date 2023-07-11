/** @file
 *    @brief MAVLink comm protocol testsuite generated from custom_messages.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef CUSTOM_MESSAGES_TESTSUITE_H
#define CUSTOM_MESSAGES_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_custom_messages(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_custom_messages(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_airdata_boom(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_AIRDATA_BOOM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_airdata_boom_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0
    };
    mavlink_airdata_boom_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.indicated_airspeed = packet_in.indicated_airspeed;
        packet1.calibrated_airspeed = packet_in.calibrated_airspeed;
        packet1.true_airspeed = packet_in.true_airspeed;
        packet1.aoa = packet_in.aoa;
        packet1.aos = packet_in.aos;
        packet1.air_temperature = packet_in.air_temperature;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_AIRDATA_BOOM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_AIRDATA_BOOM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_airdata_boom_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_airdata_boom_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_airdata_boom_pack(system_id, component_id, &msg , packet1.indicated_airspeed , packet1.calibrated_airspeed , packet1.true_airspeed , packet1.aoa , packet1.aos , packet1.air_temperature );
    mavlink_msg_airdata_boom_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_airdata_boom_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.indicated_airspeed , packet1.calibrated_airspeed , packet1.true_airspeed , packet1.aoa , packet1.aos , packet1.air_temperature );
    mavlink_msg_airdata_boom_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_airdata_boom_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_airdata_boom_send(MAVLINK_COMM_1 , packet1.indicated_airspeed , packet1.calibrated_airspeed , packet1.true_airspeed , packet1.aoa , packet1.aos , packet1.air_temperature );
    mavlink_msg_airdata_boom_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("AIRDATA_BOOM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_AIRDATA_BOOM) != NULL);
#endif
}

static void mavlink_test_motor_electrical_speed(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_motor_electrical_speed_t packet_in = {
        17235,17339,17443,17547,17651
    };
    mavlink_motor_electrical_speed_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.front_right_motor = packet_in.front_right_motor;
        packet1.back_left_motor = packet_in.back_left_motor;
        packet1.front_left_motor = packet_in.front_left_motor;
        packet1.back_right_motor = packet_in.back_right_motor;
        packet1.fixed_wing_motor = packet_in.fixed_wing_motor;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_electrical_speed_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_motor_electrical_speed_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_electrical_speed_pack(system_id, component_id, &msg , packet1.front_right_motor , packet1.back_left_motor , packet1.front_left_motor , packet1.back_right_motor , packet1.fixed_wing_motor );
    mavlink_msg_motor_electrical_speed_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_electrical_speed_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.front_right_motor , packet1.back_left_motor , packet1.front_left_motor , packet1.back_right_motor , packet1.fixed_wing_motor );
    mavlink_msg_motor_electrical_speed_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_motor_electrical_speed_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_electrical_speed_send(MAVLINK_COMM_1 , packet1.front_right_motor , packet1.back_left_motor , packet1.front_left_motor , packet1.back_right_motor , packet1.fixed_wing_motor );
    mavlink_msg_motor_electrical_speed_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MOTOR_ELECTRICAL_SPEED") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MOTOR_ELECTRICAL_SPEED) != NULL);
#endif
}

static void mavlink_test_custom_messages(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_airdata_boom(system_id, component_id, last_msg);
    mavlink_test_motor_electrical_speed(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // CUSTOM_MESSAGES_TESTSUITE_H
