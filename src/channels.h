#ifndef DC9A7AA6_4867_45B4_B139_1F16E724ECC6
#define DC9A7AA6_4867_45B4_B139_1F16E724ECC6

#include <zephyr/zbus/zbus.h>
#include "messages.h"

ZBUS_CHAN_DECLARE(chan_actuators);
ZBUS_CHAN_DECLARE(chan_accelerometer);
ZBUS_CHAN_DECLARE(chan_gyroscope);
ZBUS_CHAN_DECLARE(chan_magnetometer);
ZBUS_CHAN_DECLARE(chan_altimeter);
ZBUS_CHAN_DECLARE(chan_navsat);
ZBUS_CHAN_DECLARE(chan_waypoint);
ZBUS_CHAN_DECLARE(chan_rc_input);

#endif /* DC9A7AA6_4867_45B4_B139_1F16E724ECC6 */