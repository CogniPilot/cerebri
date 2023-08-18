#include "synapse/zbus/channels.h"

static const char* unhandled = "UNHANDLED";

const char* fsm_mode_str(synapse_msgs_Fsm_Mode mode)
{
    if (mode == synapse_msgs_Fsm_Mode_UNINITIALIZED) {
        return "unitialized";
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
    if (armed == synapse_msgs_Fsm_Armed_ARMED) {
        return "armed";
    } else if (armed == synapse_msgs_Fsm_Armed_DISARMED) {
        return "disarmed";
    }
    return unhandled;
}

const char* fsm_safety_str(synapse_msgs_Fsm_Safety safety)
{
    if (safety == synapse_msgs_Fsm_Safety_SAFE) {
        return "safe";
    } else if (safety == synapse_msgs_Fsm_Safety_UNSAFE) {
        return "unsafe";
    }
    return unhandled;
}

//*******************************************************************
// (chan_in) channels from ROS computer to cerebri
//*******************************************************************
ZBUS_CHAN_DEFINE(chan_in_actuators, // Name
    synapse_msgs_Actuators, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

ZBUS_CHAN_DEFINE(chan_in_bezier_trajectory, // Name
    synapse_msgs_BezierTrajectory, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

ZBUS_CHAN_DEFINE(chan_in_clock_offset, // Name
    synapse_msgs_Time, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

ZBUS_CHAN_DEFINE(chan_in_cmd_vel, // Name
    synapse_msgs_Twist, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

ZBUS_CHAN_DEFINE(chan_in_joy, // Name
    synapse_msgs_Joy, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

ZBUS_CHAN_DEFINE(chan_in_nav_sat_fix, // Name
    synapse_msgs_NavSatFix, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

ZBUS_CHAN_DEFINE(chan_in_odometry, // Name
    synapse_msgs_Odometry, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

//*******************************************************************
// (chan_out) channels from cerebri to other nodes
// on cerebri or to ROS computer
//*******************************************************************
ZBUS_CHAN_DEFINE(chan_out_actuators, // Name
    synapse_msgs_Actuators, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

ZBUS_CHAN_DEFINE(chan_out_altimeter, // Name
    synapse_msgs_Altimeter, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

ZBUS_CHAN_DEFINE(chan_out_battery_state, // Name
    synapse_msgs_BatteryState, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

ZBUS_CHAN_DEFINE(chan_out_cmd_vel, // Name
    synapse_msgs_Twist, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

ZBUS_CHAN_DEFINE(chan_out_imu, // Name
    synapse_msgs_Imu, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

ZBUS_CHAN_DEFINE(chan_out_magnetic_field, // Name
    synapse_msgs_MagneticField, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

ZBUS_CHAN_DEFINE(chan_out_fsm, // Name
    synapse_msgs_Fsm, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

ZBUS_CHAN_DEFINE(chan_out_nav_sat_fix, // Name
    synapse_msgs_NavSatFix, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

ZBUS_CHAN_DEFINE(chan_out_odometry, // Name
    synapse_msgs_Odometry, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

ZBUS_CHAN_DEFINE(chan_out_wheel_odometry, // Name
    synapse_msgs_WheelOdometry, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);
