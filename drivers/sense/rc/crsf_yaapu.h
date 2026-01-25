/*
 * Copyright (c) 2026 CogniPilot Foundation
 * Copyright (c) 2026 NXP Semiconductors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CRSF_YAAPU_H_
#define CRSF_YAAPU_H_

#include <stdint.h>

#define CRSF_YAAPU_MESSAGES_APPID   0x5000
#define CRSF_YAAPU_AP_STATUS_APPID  0x5001
#define CRSF_YAAPU_GPS_STATUS_APPID 0x5002
#define CRSF_YAAPU_BATTERY_APPID    0x5003
#define CRSF_YAAPU_HOME_APPID       0x5004
#define CRSF_YAAPU_VELANDYAW_APPID  0x5005
#define CRSF_YAAPU_ROLLPITCH_APPID  0x5006
#define CRSF_YAAPU_PARAMS_APPID     0x5007
#define CRSF_YAAPU_BATTERY2_APPID   0x5008
#define CRSF_YAAPU_WAYPOINTS_APPID  0x5009
#define CRSF_YAAPU_RPM_APPID        0x500A
#define CRSF_YAAPU_TERRAIN_APPID    0x500B
#define CRSF_YAAPU_VFR_APPID        0x50F2

// 0x5006: ROLLPITCH
typedef struct __packed {
	uint32_t roll_bits: 11;      // [Deg] Range [-180, 180]. Encode: (deg / 0.2) + 900
	uint32_t pitch_bits: 10;     // [Deg] Range [-90, 90]. Encode: (deg / 0.2) + 450
	uint32_t range_exp: 1;       // Power of 10 for range
	uint32_t range_mantissa: 10; // [cm] Rangefinder. Value = mantissa * 10^exp
} CRSF_YAAPU_ROLLPITCH;

// 0x5005: VELANDYAW
typedef struct __packed {
	uint32_t vspeed_exp: 1;       // Power of 10 for VSpeed
	uint32_t vspeed_mantissa: 7;  // [m/s] Vertical Speed. Value = mantissa * 10^exp
	uint32_t vspeed_sign: 1;      // 1 = negative VSpeed
	uint32_t speed_exp: 1;        // Power of 10 for HSpeed/Airspeed
	uint32_t speed_mantissa: 7;   // [dm/s] Ground/Air speed. Value = mantissa * 10^exp
	uint32_t yaw_bits: 11;        // [Deg] Heading. Encode: deg / 0.2
	uint32_t airspeed_enabled: 1; // 1 = Speed field is Airspeed, 0 = Groundspeed
	uint32_t reserved: 3;         // Unused
} CRSF_YAAPU_VELANDYAW;

// 0x5001: AP STATUS
typedef struct __packed {
	uint32_t flight_mode: 5;    // ArduPilot Flight Mode ID
	uint32_t simple_mode: 2;    // Simple/SuperSimple flags
	uint32_t land_complete: 1;  // 1 = Landed
	uint32_t armed: 1;          // 1 = Armed
	uint32_t batt_failsafe: 1;  // 1 = Failsafe active
	uint32_t ekf_failsafe: 2;   // EKF status flags
	uint32_t failsafe: 1;       // General failsafe
	uint32_t fence_present: 1;  // 1 = Geofence enabled
	uint32_t fence_breached: 1; // 1 = Geofence breached
	uint32_t reserved: 4;       // Unused
	uint32_t throttle_val: 6;   // [%] Throttle. Encode: percent / 1.58
	uint32_t throttle_sign: 1;  // 1 = negative throttle (reverse)
	uint32_t imu_temp: 6;       // [°C] Temperature. Encode: temp_c - 19
} CRSF_YAAPU_AP_STATUS;

// 0x5002: GPS STATUS
typedef struct __packed {
	uint32_t num_sats: 4;      // Sat count
	uint32_t gps_status_lo: 2; // Fix type low bits (0=No GPS, 2=2D, 3=3D+)
	uint32_t hdop_exp: 1;      // Power of 10 for HDOP
	uint32_t hdop_mantissa: 7; // [dm] HDOP. Value = mantissa * 10^exp
	uint32_t gps_status_hi: 2; // Fix type high bits (DGPS/RTK flags)
	uint32_t reserved: 6;      // Unused
	uint32_t alt_exp: 2;       // Power of 10 for Altitude
	uint32_t alt_mantissa: 7;  // [dm] GPS Altitude. Value = mantissa * 10^exp
	uint32_t alt_sign: 1;      // 1 = Negative altitude
} CRSF_YAAPU_GPS_STATUS;

// 0x5003 & 0x5008: BATTERY
typedef struct __packed {
	uint32_t voltage: 9;          // [dV] Decivolts (0.1V). Range 0-51.1V
	uint32_t current_exp: 1;      // Power of 10 for Current
	uint32_t current_mantissa: 7; // [dA] Deciamps (0.1A). Value = mantissa * 10^exp
	uint32_t mah: 15;             // [mAh] Consumed capacity
} CRSF_YAAPU_BATTERY;

// 0x5004: HOME
typedef struct __packed {
	uint32_t dist_exp: 2;       // Power of 10 for Distance
	uint32_t dist_mantissa: 10; // [m] Dist to Home. Value = mantissa * 10^exp
	uint32_t alt_exp: 2;        // Power of 10 for Altitude
	uint32_t alt_mantissa: 10;  // [m] Alt relative to Home. Value = mantissa * 10^exp * 0.1
	uint32_t alt_sign: 1;       // 1 = Negative altitude
	uint32_t angle: 7;          // [Deg] Direction to home. Encode: deg / 3
} CRSF_YAAPU_HOME;

// 0x5000: MESSAGES
typedef struct __packed {
	uint32_t char_0: 7;    // ASCII char 1
	uint32_t sev_bit_0: 1; // Severity bit 0 (LSB)
	uint32_t char_1: 7;    // ASCII char 2
	uint32_t sev_bit_1: 1; // Severity bit 1
	uint32_t char_2: 7;    // ASCII char 3
	uint32_t sev_bit_2: 1; // Severity bit 2 (MSB)
	uint32_t char_3: 7;    // ASCII char 4
	uint32_t reserved: 1;  // Unused
} CRSF_YAAPU_MESSAGES;

// 0x5007: PARAMS
typedef struct __packed {
	uint32_t param_value: 24; // Raw parameter value (int)
	uint32_t param_id: 4;     // 1=FrameType, 4=Batt1Cap, 5=Batt2Cap
	uint32_t reserved: 4;     // Unused
} CRSF_YAAPU_PARAMS;

// 0x5009: WAYPOINTS
typedef struct __packed {
	uint32_t wp_index: 10;        // Current Waypoint Index
	uint32_t dist_exp: 2;         // Power of 10 for Distance
	uint32_t dist_mantissa: 10;   // [m] Distance to WP. Value = mantissa * 10^exp
	uint32_t xterror_exp: 1;      // Power of 10 for Crosstrack Error
	uint32_t xterror_mantissa: 4; // [m] Crosstrack Error. Value = mantissa * 10^exp
	uint32_t xterror_sign: 1;     // 1 = Negative Error
	uint32_t reserved: 1;         // Unused
	uint32_t bearing: 3;          // [Deg] Bearing to WP. Encode: deg / 45
} CRSF_YAAPU_WAYPOINTS;

// 0x500A: RPM
typedef struct __packed {
	int16_t rpm1; // [RPM] Sensor 1 (Raw int16)
	int16_t rpm2; // [RPM] Sensor 2 (Raw int16)
} CRSF_YAAPU_RPM;

// 0x500B: TERRAIN
typedef struct __packed {
	uint32_t height_exp: 2;       // Power of 10 for Height
	uint32_t height_mantissa: 10; // [m] Height above terrain. Value = mantissa * 10^exp * 0.1
	uint32_t height_sign: 1;      // 1 = Negative height
	uint32_t unhealthy_flag: 1;   // 1 = Terrain data unhealthy
	uint32_t reserved: 18;        // Unused
} CRSF_YAAPU_TERRAIN;

// 0x50F2: VFR HUD
typedef struct __packed {
	uint32_t airspeed_exp: 1;      // Power of 10 for Airspeed
	uint32_t airspeed_mantissa: 7; // [dm/s] Airspeed. Value = mantissa * 10^exp
	uint32_t throttle: 7;          // [%] Throttle (0-100)
	uint32_t baro_exp: 2;          // Power of 10 for Baro Alt
	uint32_t baro_mantissa: 10;    // [m] Baro Altitude. Value = mantissa * 10^exp * 0.1
	uint32_t baro_sign: 1;         // 1 = Negative Altitude
	uint32_t reserved: 4;          // Unused
} CRSF_YAAPU_VFR;

typedef struct __packed {
	uint16_t appid;
	union {
		uint32_t raw;
		CRSF_YAAPU_ROLLPITCH roll_pitch;
		CRSF_YAAPU_VELANDYAW vel_yaw;
		CRSF_YAAPU_AP_STATUS ap_status;
		CRSF_YAAPU_GPS_STATUS gps_status;
		CRSF_YAAPU_BATTERY battery;
		CRSF_YAAPU_HOME home;
		CRSF_YAAPU_MESSAGES messages;
		CRSF_YAAPU_PARAMS params;
		CRSF_YAAPU_WAYPOINTS waypoints;
		CRSF_YAAPU_RPM rpm;
		CRSF_YAAPU_TERRAIN terrain;
		CRSF_YAAPU_VFR vfr;
	} data;
} CRSF_YAAPU_PACKET;

/* Type 0x50 - AP Custom telem */
/* Subheader 1 byte (SubID) + 1 byte (Severity) = 2 bytes */
#define CRSF_AP_CUSTOM_TELEM_SINGLE_PACKET_PASSTHROUGH 0xF0 // Sub-ID for Single Packet Passthrough
#define CRSF_AP_CUSTOM_TELEM_STATUS_TEXT               0xF1 // Sub-ID for Status Text
#define CRSF_AP_CUSTOM_TELEM_MULTI_PACKET_PASSTHROUGH  0xF2 // Sub-ID for Multi Packet Passthrough

/* Severity */
#define CRSF_AP_CUSTOM_SEVERITY_EMERGENCY 0 // System is unusable
#define CRSF_AP_CUSTOM_SEVERITY_ALERT     1 // Action must be taken immediately
#define CRSF_AP_CUSTOM_SEVERITY_CRITICAL  2 // System is in a critical condition
#define CRSF_AP_CUSTOM_SEVERITY_ERROR     3 // Generic error condition
#define CRSF_AP_CUSTOM_SEVERITY_WARNING   4 // Warning that should be noted
#define CRSF_AP_CUSTOM_SEVERITY_NOTICE    5 // Normal but significant condition
#define CRSF_AP_CUSTOM_SEVERITY_INFO      6 // Informational message
#define CRSF_AP_CUSTOM_SEVERITY_DEBUG     7 // Debugging message

#define CRSF_AP_CUSTOM_TEXT_MAX_LEN 50
struct crsf_ap_custom_status_text {
	uint8_t sub_id;   // Always set to CRSF_AP_CUSTOM_TELEM_STATUS_TEXT (0xF1)
	uint8_t severity; // MAVLink Severity (0=Emergency, 6=Info, etc.)
	char text[CRSF_AP_CUSTOM_TEXT_MAX_LEN]; // Null-terminated string
} __packed;

struct crsf_ap_custom_single_packet {
	uint8_t sub_id;
	CRSF_YAAPU_PACKET packet;
} __packed;

#define CRSF_AP_CUSTOM_MULTI_PACKET_FRAME_MAX_LEN 9
struct crsf_ap_custom_multi_packet {
	uint8_t sub_id;
	uint8_t size;
	CRSF_YAAPU_PACKET packets[CRSF_AP_CUSTOM_MULTI_PACKET_FRAME_MAX_LEN];
} __packed;

#endif /* CRSF_YAAPU_H_ */