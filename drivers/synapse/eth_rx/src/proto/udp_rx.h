#ifndef SYNAPSE_UDP_UDP_RX_H_
#define SYNAPSE_UDP_UDP_RX_H_

#include <zephyr/net/socket.h>

struct udp_rx {
	int sock;
	struct sockaddr_in addr;
	char rx_buf[1024];
};

int udp_rx_init(struct udp_rx *ctx);
int udp_rx_fini(struct udp_rx *ctx);
int udp_rx_receive(struct udp_rx *ctx);

#endif // SYNAPSE_UDP_UDP_RX_H_
// vi: ts=4 sw=4 et
