#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <errno.h>

#include <zephyr/net/net_core.h>
#include <zenoh-pico.h>
#include <stdio.h>
//#include <ignition/transport11/ignition/transport.hh>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(net_native_posix_sample, LOG_LEVEL_DBG);

#define CLIENT_OR_PEER 0  // 0: Client mode; 1: Peer mode
#if CLIENT_OR_PEER == 0
#define MODE "client"
#define PEER ""  // If empty, it will scout
#elif CLIENT_OR_PEER == 1
#define MODE "peer"
#define PEER "udp/224.0.0.225:7447#iface=zeth0"
#else
#error "Unknown Zenoh operation mode. Check CLIENT_OR_PEER value."
#endif

#define KEYEXPR "rt/demo/data"
#define VALUE "hello"

/* --------------- Structs -------------- */
struct Vector3
{
    double x;
    double y;
    double z;
};
struct Twist
{
    struct Vector3 linear;
    struct Vector3 angular;
};

/* -------- Serialize Functions --------- */
char *serialize_float_as_f64_little_endian(double val, char *buf)
{
    long long *c_val = (long long*)&val;
    for (int i = 0; i < sizeof(double); ++i, ++buf) {
       *buf = 0xFF & (*c_val >> (i * 8));
    }

    return buf;
}

char *serialize_vector3(struct Vector3 *v, char *buf)
{
    buf = serialize_float_as_f64_little_endian(v->x, buf);
    buf = serialize_float_as_f64_little_endian(v->y, buf);
    buf = serialize_float_as_f64_little_endian(v->z, buf);

    return buf;
}

void serialize_twist(struct Twist *t, char *buf)
{
    // Serialize Twist header for little endian
    *(buf++) = 0x00;
    *(buf++) = 0x01;
    *(buf++) = 0x00;
    *(buf++) = 0x00;
    buf = serialize_vector3(&t->linear, buf);
    buf = serialize_vector3(&t->angular, buf);
}

void main(void)
{
    printf("hello world!\n");

    // Initialize Zenoh Session and other parameters
    printf("creating zenoh!\n");
    z_owned_config_t config = z_config_default();
    zp_config_insert(z_loan(config), Z_CONFIG_MODE_KEY, z_string_make(MODE));
    if (strcmp(PEER, "") != 0) {
        zp_config_insert(z_loan(config), Z_CONFIG_PEER_KEY, z_string_make(PEER));
    }

    // Open Zenoh session
    printf("Opening Zenoh Session...");
    z_owned_session_t s = z_open(z_move(config));
    if (!z_check(s)) {
        printf("Unable to open session!\n");
        exit(-1);
    }
    printf("OK\n");

    // Start the receive and the session lease loop for zenoh-pico
    printf("Read task starting...\n");
    zp_start_read_task(z_loan(s), NULL);
    printf("done.\n");

    printf("Lease task starting...\n");
    zp_start_lease_task(z_loan(s), NULL);
    printf("done.\n");

    printf("Declaring publisher for '%s'...\n", KEYEXPR);
    z_owned_publisher_t pub = z_declare_publisher(z_loan(s), z_keyexpr(KEYEXPR), NULL);
    if (!z_check(pub)) {
        printf("Unable to declare publisher for key expression!\n");
        exit(-1);
    }
    printf("OK\n");

    double linear_x = 1;
    double linear_y = 2;

    // Create ROS twist message
    struct Twist measure;
    measure.linear.x = linear_x * -1;
    measure.linear.y = 0.0;
    measure.linear.z = 0.0;
    measure.angular.x = 0.0;
    measure.angular.y = 0.0;
    measure.angular.z = linear_y;

    uint8_t twist_serialized_size = 4 + sizeof(double) * 6;
    char buf[twist_serialized_size];
    serialize_twist(&measure, buf);
        
    for (int idx = 0; 1; ++idx) {
        if (idx >= 9999) idx = 0;
        if (z_publisher_put(z_publisher_loan(&pub), (const uint8_t *)buf, twist_serialized_size, NULL) < 0) {
            printk("Error while publishing data");
        }
        k_usleep(1000);
    }

    printf("Closing Zenoh Session...");
    z_undeclare_publisher(z_move(pub));

    // Stop the receive and the session lease loop for zenoh-pico
    zp_stop_read_task(z_loan(s));
    zp_stop_lease_task(z_loan(s));

    z_close(z_move(s));
    printf("OK!\n");
}