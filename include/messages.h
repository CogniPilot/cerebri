#ifndef D00B381E_08D8_47AC_B779_57B422AF3850
#define D00B381E_08D8_47AC_B779_57B422AF3850

enum actuator_type_t{
    ACTUATOR_TYPE_UNDEFINED = 0,
    ACTUATOR_TYPE_ANGULAR_POSITION,
    ACTUATOR_TYPE_ANGULAR_VELOCITY,
    ACTUATOR_TYPE_LINEAR_POSITION,
    ACTUATOR_TYPE_LINEAR_VELOCITY,
    ACTUATOR_TYPE_FORCE,
    ACTUATOR_TYPE_TORQUE,
    ACTUATOR_TYPE_END
};
struct msg_actuators_t {
    uint64_t timestamp;
    double actuator0_value;
    uint8_t actuator0_type;
    double actuator1_value;
    uint8_t actuator1_type;
    double actuator2_value;
    uint8_t actuator2_type;
    double actuator3_value;
    uint8_t actuator3_type;
    double actuator4_value;
    uint8_t actuator4_type;
    double actuator5_value;
    uint8_t actuator5_type;
    double actuator6_value;
    uint8_t actuator6_type;
    double actuator7_value;
    uint8_t actuator7_type;
};

struct msg_accelerometer_t {
    uint64_t timestamp;
    double x;
    double y;
    double z;
};

struct msg_gyroscope_t {
    uint64_t timestamp;
    double x;
    double y;
    double z;
};

struct msg_magnetometer_t {
    uint64_t timestamp;
    double x;
    double y;
    double z;
};

struct msg_altimeter_t {
    uint64_t timestamp;
    double position;
    double reference;
    double velocity;
};

struct msg_navsat_t {
    uint64_t timestamp;
    double latitude_deg;
    double longitude_deg;
    double altitude;
    double velocity_east;
    double velocity_north;
    double velocity_up;
};

struct msg_trajectory_t {
    uint64_t timestamp;
    uint16_t sequence;
    uint64_t time_start;
    uint64_t time_stop;
    double x[8];
    double y[8];
    double z[8];
    double yaw[8];
};

enum control_mode_t {
    MODE_MANUAL=0,
    MODE_AUTO=1,
};

struct msg_rc_input_t {
    uint64_t timestamp;
    double roll;
    double pitch;
    double yaw;
    double thrust;
    enum control_mode_t mode;
    bool armed;
};

struct msg_odometry_t {
    uint64_t timestamp;
    double x;
    double y;
    double z;
    double qx;
    double qy;
    double qz;
    double qw;
    double vx;
    double vy;
    double vz;
    double wx;
    double wy;
    double wz;
    double pose_cov[36];
    double twist_cov[36];
    char child_frame_id[10];
    char parent_frame_id[10];
};



#endif /* D00B381E_08D8_47AC_B779_57B422AF3850 */
