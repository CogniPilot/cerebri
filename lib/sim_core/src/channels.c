#include "channels.h"

// sensor measurements
ZBUS_CHAN_DEFINE(chan_accelerometer, // Name
    struct msg_accelerometer_t, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(listener_estimator), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

ZBUS_CHAN_DEFINE(chan_gyroscope, // Name
    struct msg_gyroscope_t, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(listener_estimator, listener_controller), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

ZBUS_CHAN_DEFINE(chan_magnetometer, // Name
    struct msg_magnetometer_t, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(listener_estimator), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

ZBUS_CHAN_DEFINE(chan_altimeter, // Name
    struct msg_altimeter_t, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(listener_estimator), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

ZBUS_CHAN_DEFINE(chan_navsat, // Name
    struct msg_navsat_t, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(listener_estimator), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

ZBUS_CHAN_DEFINE(chan_external_odometry, // Name
    struct msg_odometry_t, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

// commands
ZBUS_CHAN_DEFINE(chan_rc_input, // Name
    struct msg_rc_input_t, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(listener_controller), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

ZBUS_CHAN_DEFINE(chan_trajectory, // Name
    struct msg_trajectory_t, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

// estimation
ZBUS_CHAN_DEFINE(chan_estimator_odometry, // Name
    struct msg_odometry_t, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);

// control
ZBUS_CHAN_DEFINE(chan_actuators, // Name
    struct msg_actuators_t, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(listener_sim_actuators), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);


