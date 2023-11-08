#include <zephyr/logging/log.h>

#include <cerebri/synapse/zbus/channels.h>

LOG_MODULE_REGISTER(synapse_zbus, CONFIG_CEREBRI_SYNAPSE_ZBUS_LOG_LEVEL);

#define SYN_CHAN(CHANNEL, CLASS)                             \
    ZBUS_CHAN_DEFINE(CHANNEL, /* Channel Name */             \
        CLASS, /* Message Type */                            \
        NULL, /* Validator */                                \
        NULL, /* User Data */                                \
        ZBUS_OBSERVERS(), /* Observers (Added Externally) */ \
        ZBUS_MSG_INIT(0) /* Initial Value */                 \
    );

//*******************************************************************
// ZBUS channels
//*******************************************************************
SYN_CHAN(chan_actuators, synapse_msgs_Actuators)
SYN_CHAN(chan_actuators_manual, synapse_msgs_Actuators)
SYN_CHAN(chan_altimeter, synapse_msgs_Altimeter)
SYN_CHAN(chan_battery_state, synapse_msgs_BatteryState)
SYN_CHAN(chan_bezier_trajectory, synapse_msgs_BezierTrajectory)
SYN_CHAN(chan_clock_offset, synapse_msgs_Time)
SYN_CHAN(chan_cmd_vel, synapse_msgs_Twist)
SYN_CHAN(chan_imu, synapse_msgs_Imu)
SYN_CHAN(chan_joy, synapse_msgs_Joy)
SYN_CHAN(chan_led_array, synapse_msgs_LEDArray)
SYN_CHAN(chan_magnetic_field, synapse_msgs_MagneticField)
SYN_CHAN(chan_nav_sat_fix, synapse_msgs_NavSatFix)
SYN_CHAN(chan_estimator_odometry, synapse_msgs_Odometry)
SYN_CHAN(chan_external_odometry, synapse_msgs_Odometry)
SYN_CHAN(chan_fsm, synapse_msgs_Fsm)
SYN_CHAN(chan_safety, synapse_msgs_Safety)
SYN_CHAN(chan_wheel_odometry, synapse_msgs_WheelOdometry)

//*******************************************************************
// helper functions
//*******************************************************************
static const char* unhandled = "UNHANDLED";

void stamp_header(synapse_msgs_Header* hdr, int64_t ticks)
{
    int64_t sec = ticks / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    int32_t nanosec = (ticks - sec * CONFIG_SYS_CLOCK_TICKS_PER_SEC) * 1e9 / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    hdr->has_stamp = true;
    hdr->stamp.sec = sec;
    hdr->stamp.nanosec = nanosec;
}

const char* fsm_mode_str(synapse_msgs_Fsm_Mode mode)
{
    if (mode == synapse_msgs_Fsm_Mode_UNKNOWN_MODE) {
        return "unknown";
    } else if (mode == synapse_msgs_Fsm_Mode_MANUAL) {
        return "manual";
    } else if (mode == synapse_msgs_Fsm_Mode_AUTO) {
        return "auto";
    } else if (mode == synapse_msgs_Fsm_Mode_CMD_VEL) {
        return "cmd_vel";
    }
    return unhandled;
}

const char* fsm_armed_str(synapse_msgs_Fsm_Armed armed)
{
    if (armed == synapse_msgs_Fsm_Armed_UNKNOWN_ARMING) {
        return "unknown";
    } else if (armed == synapse_msgs_Fsm_Armed_ARMED) {
        return "armed";
    } else if (armed == synapse_msgs_Fsm_Armed_DISARMED) {
        return "disarmed";
    }
    return unhandled;
}

const char* safety_str(synapse_msgs_Safety_Status safety)
{
    if (safety == synapse_msgs_Safety_Status_UNKNOWN) {
        return "unknown";
    } else if (safety == synapse_msgs_Safety_Status_SAFE) {
        return "safe";
    } else if (safety == synapse_msgs_Safety_Status_UNSAFE) {
        return "unsafe";
    }
    return unhandled;
}
