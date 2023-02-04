#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>
#include <cassert>
#include <iostream>
#include <string>
#include <thread>
#include <gz/msgs.hh>
#include <gz/transport.hh>

#include "../sim_core/src/sim_core.h"

static std::weak_ptr<gz::transport::Node::Publisher> pub_esc0_ptr{};
static gz::msgs::Double esc0;
static const std::string esc0_topic = "/model/MR_Buggy3/drive";


static std::weak_ptr<gz::transport::Node::Publisher> pub_servo1_ptr{};
static gz::msgs::Double servo1;
static const std::string servo1_topic = "/model/MR_Buggy3/steer_angle";
static const std::string clock_topic = "/world/default/clock";
static const std::string mag_topic = "/world/default/model/MR_Buggy3/link/base_link/sensor/mag_sensor/mag";
static const std::string navsat_topic = "/world/default/model/MR_Buggy3/link/base_link/sensor/navsat_sensor/navsat";
static const std::string alt_topic = "/world/default/model/MR_Buggy3/link/base_link/sensor/altimeter_sensor/altimeter";
static const std::string imu_topic = "/world/default/model/MR_Buggy3/link/base_link/sensor/imu_sensor/imu";
static const std::string odom_topic = "/model/MR_Buggy3/odometry_with_covariance";

static std::atomic<bool> g_terminatePub(false);
static std::atomic<bool> armed(false);
static const std::string trajectory_topic = "/traj";
static gz::msgs::BezierTrajectory trajectory{};

static gz::msgs::Joy joy{};
static const std::string rc_input_topic = "/joy";
static gz::msgs::Time stamp{};
static gz::msgs::Header header{};

void send_control(sim_time_t time, const msg_actuators_t * msg) {
    
    // set timestamp
    stamp.set_sec(time.sec);
    stamp.set_nsec(time.nsec);
    esc0.set_data(msg->actuator0_value);
    if (!pub_esc0_ptr.lock().get()->Publish(esc0)) {
        std::cerr << "Error publishing topic [" << esc0_topic << "]" << std::endl;
    }
    

    servo1.set_data(msg->actuator1_value);
    if (!pub_servo1_ptr.lock().get()->Publish(servo1)) {
        std::cerr << "Error publishing topic [" << servo1_topic << "]" << std::endl;
    }
    
}

void imu_callback(const gz::msgs::IMU &msg) {
    uint64_t uptime = msg.header().stamp().sec()*1e9 + msg.header().stamp().nsec();
    msg_gyroscope_t msg_gyro{
        .uptime_nsec=uptime,
        .x=msg.angular_velocity().x(),
        .y=msg.angular_velocity().y(),
        .z=msg.angular_velocity().z()
    };
    queue_gyroscope.push(msg_gyro);

    msg_accelerometer_t msg_acc{
        .uptime_nsec=uptime,
        .x=msg.linear_acceleration().x(),
        .y=msg.linear_acceleration().y(),
        .z=msg.linear_acceleration().z()
    };
    queue_accelerometer.push(msg_acc);
}

void mag_callback(const gz::msgs::Magnetometer &msg) {
    uint64_t uptime = msg.header().stamp().sec()*1e9 + msg.header().stamp().nsec();
    msg_magnetometer_t msg_pub {
        .uptime_nsec=uptime,
        .x=msg.field_tesla().x(),
        .y=msg.field_tesla().y(),
        .z=msg.field_tesla().z()
    };
    queue_magnetometer.push(msg_pub);
}

void navsat_callback(const gz::msgs::NavSat &msg) {
    uint64_t uptime = msg.header().stamp().sec()*1e9 + msg.header().stamp().nsec();
    msg_navsat_t msg_pub{
        .uptime_nsec=uptime,
        .latitude_deg=msg.latitude_deg(),
        .longitude_deg=msg.longitude_deg(),
        .altitude=msg.altitude(),
        .velocity_east=msg.velocity_east(),
        .velocity_north=msg.velocity_north(),
        .velocity_up=msg.velocity_up()
    };
    queue_navsat.push(msg_pub);
}

void alt_callback(const gz::msgs::Altimeter &msg) {
    uint64_t uptime = msg.header().stamp().sec()*1e9 + msg.header().stamp().nsec();
    msg_altimeter_t msg_pub {
        .uptime_nsec=uptime,
        .position=msg.vertical_position(),
        .reference=msg.vertical_reference(),
        .velocity=msg.vertical_velocity()
    };
    queue_altimeter.push(msg_pub);
}

void odom_callback(const gz::msgs::OdometryWithCovariance &msg) {
    uint64_t uptime = msg.header().stamp().sec()*1e9 + msg.header().stamp().nsec();
    msg_odometry_t msg_odom{
        .uptime_nsec=uptime,
        .x=msg.pose_with_covariance().pose().position().x(),
        .y=msg.pose_with_covariance().pose().position().y(),
        .z=msg.pose_with_covariance().pose().position().z(),
        .qx=msg.pose_with_covariance().pose().orientation().x(),
        .qy=msg.pose_with_covariance().pose().orientation().y(),
        .qz=msg.pose_with_covariance().pose().orientation().z(),
        .qw=msg.pose_with_covariance().pose().orientation().w(),
        .vx=msg.twist_with_covariance().twist().linear().x(),
        .vy=msg.twist_with_covariance().twist().linear().y(),
        .vz=msg.twist_with_covariance().twist().linear().z(),
        .wx=msg.twist_with_covariance().twist().angular().x(),
        .wy=msg.twist_with_covariance().twist().angular().y(),
        .wz=msg.twist_with_covariance().twist().angular().z()
    };
    queue_external_odometry.push(msg_odom);
}


void trajectory_callback(const gz::msgs::BezierTrajectory &msg) {
    uint64_t uptime = msg.header().stamp().sec()*1e9 + msg.header().stamp().nsec();
    msg_trajectory_t msg_trajectory{
        .uptime_nsec=uptime,
        .sequence=uint16_t(msg.sequence()),
        .time_start=msg.time_start(),
        .time_stop=msg.time_stop(),
        .x={},
        .y={},
        .z={},
        .yaw={}
    };
    for (int m = 0; m < msg.x_size(); m++) {
        msg_trajectory.x[m]=msg.x()[m];
    };
    for (int m = 0; m < msg.y_size(); m++) {
        msg_trajectory.y[m]=msg.y()[m];
    };
    for (int m = 0; m < msg.z_size(); m++) {
        msg_trajectory.z[m]=msg.z()[m];
    };
    for (int m = 0; m < msg.yaw_size(); m++) {
        msg_trajectory.yaw[m]=msg.yaw()[m];
    };
    queue_trajectory.push(msg_trajectory);
}

void rc_input_callback(const gz::msgs::Joy &msg) {
    uint64_t uptime = msg.header().stamp().sec()*1e9 + msg.header().stamp().nsec();
    if (!armed && msg.buttons()[6] == 1) {
        armed = true;
        std::cout << "armed!" << std::endl;
    }
    if (armed && msg.buttons()[7] == 1) {
        armed = false;
        std::cout << "dis-armed!" << std::endl;
    }
    msg_rc_input_t msg_rc_input{
        .uptime_nsec=uptime,
        .yaw=msg.axes()[3],
        .thrust=msg.axes()[2],
        .mode=msg.buttons()[4],
        .armed=armed,
    };
    queue_rc_input.push(msg_rc_input);
}


void clock_callback(const gz::msgs::Clock &msg) {
    sim_time_t msg_pub {
        .sec=msg.sim().sec(),
        .nsec=msg.sim().nsec()
    };
    queue_sim_time.push(msg_pub);
}

void thread_sim_entry_point(void)
{
    // Create a transport node and advertise a topic.
    gz::transport::Node node;
    auto pub_esc0 = std::make_shared<gz::transport::Node::Publisher>(
        node.Advertise<gz::msgs::Double>(esc0_topic));
    if (!pub_esc0)
    {
        std::cerr << "Error advertising topic [" << esc0_topic << "]" << std::endl;
        return;
    }
    pub_esc0_ptr = pub_esc0;
    


    auto pub_servo1 = std::make_shared<gz::transport::Node::Publisher>(
        node.Advertise<gz::msgs::Double>(servo1_topic));
    if (!pub_servo1)
    {
        std::cerr << "Error advertising topic [" << servo1_topic << "]" << std::endl;
        return;
    }
    pub_servo1_ptr = pub_servo1;
    

    // imu sub
    bool sub_imu = node.Subscribe<gz::msgs::IMU>(imu_topic, imu_callback);
    if (!sub_imu)
    {
        std::cerr << "Error subscribing to topic [" << imu_topic << "]" << std::endl;
        return;
    }

    // odom sub
    bool sub_odom = node.Subscribe<gz::msgs::OdometryWithCovariance>(odom_topic, odom_callback);
    if (!sub_odom)
    {
        std::cerr << "Error subscribing to topic [" << odom_topic << "]" << std::endl;
        return;
    }

    // clock sub
    bool sub_clock = node.Subscribe<gz::msgs::Clock>(clock_topic, clock_callback);
    if (!sub_clock)
    {
        std::cerr << "Error subscribing to topic [" << clock_topic << "]" << std::endl;
        return;
    }

    // mag sub
    bool sub_mag = node.Subscribe<gz::msgs::Magnetometer>(mag_topic, mag_callback);
    if (!sub_mag)
    {
        std::cerr << "Error subscribing to topic [" << mag_topic << "]" << std::endl;
        return;
    }

    // navsat sub
    bool sub_navsat = node.Subscribe<gz::msgs::NavSat>(navsat_topic, navsat_callback);
    if (!sub_navsat)
    {
        std::cerr << "Error subscribing to topic [" << navsat_topic << "]" << std::endl;
        return;
    }

    // altimeter sub
    bool sub_alt = node.Subscribe<gz::msgs::Altimeter>(alt_topic, alt_callback);
    if (!sub_alt)
    {
        std::cerr << "Error subscribing to topic [" << alt_topic << "]" << std::endl;
        return;
    }

    // Trajectory sub
    bool sub_trajectory = node.Subscribe<gz::msgs::BezierTrajectory>(trajectory_topic, trajectory_callback);
    if (!sub_trajectory)
    {
        std::cerr << "Error subscribing to topic [" << trajectory_topic << "]" << std::endl;
        return;
    }


    // rc_input sub
    bool sub_rc_input = node.Subscribe<gz::msgs::Joy>(rc_input_topic, rc_input_callback);
    if (!sub_rc_input)
    {
        std::cerr << "Error subscribing to topic [" << rc_input_topic << "]" << std::endl;
        return;
    }

    // sleep main thread, everything done async
    while (!g_terminatePub)
    {
        // host sleep doesn't count for zephyr time
        sleep(1);
    }
}

std::thread th_sim_gz_driver(thread_sim_entry_point);