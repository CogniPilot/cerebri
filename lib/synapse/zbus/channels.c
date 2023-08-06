#include "synapse/zbus/channels.h"

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
    ZBUS_OBSERVERS(
#if defined(CONFIG_CONTROL_ACKERMANN)
        listener_control_ackermann,
#elif defined(CONFIG_CONTROL_DIFFDRIVE)
        listener_control_diffdrive,
#endif
        ), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

ZBUS_CHAN_DEFINE(chan_in_clock_offset, // Name
    synapse_msgs_Time, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(
#if defined(CONFIG_CONTROL_ACKERMANN)
        listener_control_ackermann,
#elif defined(CONFIG_CONTROL_DIFFDRIVE)
        listener_control_diffdrive,
#endif
        ), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

ZBUS_CHAN_DEFINE(chan_in_cmd_vel, // Name
    synapse_msgs_Twist, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(
#if defined(CONFIG_CONTROL_ACKERMANN)
        listener_control_ackermann,
#elif defined(CONFIG_CONTROL_DIFFDRIVE)
        listener_control_diffdrive,
#endif
        ), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

ZBUS_CHAN_DEFINE(chan_in_joy, // Name
    synapse_msgs_Joy, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(
#if defined(CONFIG_CONTROL_ACKERMANN)
        listener_control_ackermann,
#elif defined(CONFIG_CONTROL_DIFFDRIVE)
        listener_control_diffdrive,
#endif
        ), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

ZBUS_CHAN_DEFINE(chan_in_nav_sat_fix, // Name
    synapse_msgs_NavSatFix, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(
#if defined(CONFIG_SYNAPSE_ETHERNET)
        listener_synapse_ethernet,
#endif
#if defined(CONFIG_SYNAPSE_UART)
        listener_synapse_uart,
#endif
        ), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

ZBUS_CHAN_DEFINE(chan_in_odometry, // Name
    synapse_msgs_Odometry, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(
#if defined(CONFIG_CONTROL_ACKERMANN)
        listener_control_ackermann,
#elif defined(CONFIG_CONTROL_DIFFDRIVE)
        listener_control_diffdrive,
#endif
        ), // observers
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
    ZBUS_OBSERVERS(
#if defined(CONFIG_ACTUATE_PWM)
        listener_actuator_pwm,
#endif
#if defined(CONFIG_ACTUATE_VESC_CAN)
        listener_actuate_vesc_can,
#endif
#if defined(CONFIG_DREAM_SITL)
        listener_dream_sitl,
#endif
#if defined(CONFIG_SYNAPSE_ETHERNET)
        listener_synapse_ethernet,
#endif
#if defined(CONFIG_SYNAPSE_UART)
        listener_synapse_uart,
#endif
#if defined(CONFIG_ESTIMATE_ROVER2D)
        listener_estimate_rover2d,
#endif
        ), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

ZBUS_CHAN_DEFINE(chan_out_altimeter, // Name
    synapse_msgs_Altimeter, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(
#if defined(CONFIG_ESTIMATE_ATTITUDE)
        listener_estimate_attitude,
#endif
        ), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

ZBUS_CHAN_DEFINE(chan_out_battery_state, // Name
    synapse_msgs_BatteryState, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(
#if defined(CONFIG_CONTROL_ACKERMANN)
        listener_control_ackermann,
#elif defined(CONFIG_CONTROL_DIFFDRIVE)
        listener_control_diffdrive,
#endif
        ), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

ZBUS_CHAN_DEFINE(chan_out_imu, // Name
    synapse_msgs_Imu, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(
#if defined(CONFIG_ESTIMATE_ATTITUDE)
        listener_estimate_attitude,
#endif
        ), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

ZBUS_CHAN_DEFINE(chan_out_magnetic_field, // Name
    synapse_msgs_MagneticField, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(
#if defined(CONFIG_ESTIMATE_ATTITUDE)
        listener_estimate_attitude,
#endif
        ), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

ZBUS_CHAN_DEFINE(chan_out_nav_sat_fix, // Name
    synapse_msgs_NavSatFix, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(
#if defined(CONFIG_ESTIMATE_ATTITUDE)
        listener_estimate_attitude,
#endif
        ), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

ZBUS_CHAN_DEFINE(chan_out_odometry, // Name
    synapse_msgs_Odometry, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(
#if defined(CONFIG_SYNAPSE_ETHERNET)
        listener_synapse_ethernet,
#endif
#if defined(CONFIG_SYNAPSE_UART)
        listener_synapse_uart,
#endif
        ), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

ZBUS_CHAN_DEFINE(chan_out_wheel_odometry, // Name
    synapse_msgs_WheelOdometry, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(
#if defined(CONFIG_ESTIMATE_ROVER2D)
        listener_estimate_rover2d,
#endif
        ), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);
