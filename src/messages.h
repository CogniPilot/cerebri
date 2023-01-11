#ifndef D00B381E_08D8_47AC_B779_57B422AF3850
#define D00B381E_08D8_47AC_B779_57B422AF3850

struct msg_actuators_t {
    uint64_t uptime_nsec;
    double ch0;
    double ch1;
    double ch2;
    double ch3;
};

struct msg_accelerometer_t {
    uint64_t uptime_nsec;
    double x;
    double y;
    double z;
};

struct msg_gyroscope_t {
    uint64_t uptime_nsec;
    double x;
    double y;
    double z;
};

struct msg_magnetometer_t {
    uint64_t uptime_nsec;
    double x;
    double y;
    double z;
};

struct msg_altimeter_t {
    uint64_t uptime_nsec;
    double position;
    double reference;
    double velocity;
};

struct msg_navsat_t {
    uint64_t uptime_nsec;
    double latitude_deg;
    double longitude_deg;
    double altitude;
    double velocity_east;
    double velocity_north;
    double velocity_up;
};

struct msg_waypoint_t {
    uint64_t uptime_nsec;
    double position_x;
    double position_y;
    double position_z;
    double orientation_yaw;
    double velocity_x;
    double velocity_y;
    double velocity_z;
};

struct msg_rc_input_t {
    uint64_t uptime_nsec;
    double roll;
    double pitch;
    double yaw;
    double thrust;
    bool armed;
};

#endif /* D00B381E_08D8_47AC_B779_57B422AF3850 */
