#include "synapse_zbus/channels.h"
#include "synapse_pb/twist.pb.h"

// sensor measurements
ZBUS_CHAN_DEFINE(chan_twist, // Name
    Twist, // Message type
    NULL, // Validator
    NULL, // User Data
    ZBUS_OBSERVERS(), // observers
    ZBUS_MSG_INIT(0) // Initial value {0}
);
