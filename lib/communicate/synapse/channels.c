#include "synapse_zbus/channels.h"

ZBUS_CHAN_DEFINE(chan_in_actuators, // Name
    Actuators, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

ZBUS_CHAN_DEFINE(chan_in_bezier_trajectory, // Name
    BezierTrajectory, // Message type
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
    Twist, // Message type
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
    Joy, // Message type
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

ZBUS_CHAN_DEFINE(chan_in_odometry, // Name
    Odometry, // Message type
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

ZBUS_CHAN_DEFINE(chan_out_actuators, // Name
    Actuators, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(
#if defined(CONFIG_ACTUATE_PWM)
        listener_actuator_pwm,
#endif
#if defined(CONFIG_SIM_SITL)
        listener_sim_sitl,
#endif
#if defined(CONFIG_COMMUNICATE_SYNAPSE_ZBUS_ETHERNET)
        listener_synapse_zbus_ethernet,
#endif
#if defined(CONFIG_COMMUNICATE_SYNAPSE_ZBUS_UART)
        listener_synapse_zbus_uart,
#endif
        ), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

ZBUS_CHAN_DEFINE(chan_out_odometry, // Name
    Odometry, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(
#if defined(CONFIG_COMMUNICATE_SYNAPSE_ZBUS_ETHERNET)
        listener_synapse_zbus_ethernet,
#endif
#if defined(CONFIG_COMMUNICATE_SYNAPSE_ZBUS_UART)
        listener_synapse_zbus_uart,
#endif
        ), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);
