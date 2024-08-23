#ifndef SYNAPSE_UDP_UDP_TX_H_
#define SYNAPSE_UDP_UDP_TX_H_

#include <zephyr/net/socket.h>

struct udp_tx {
	int sock;
	struct sockaddr_in addr;
};

int udp_tx_init(struct udp_tx *ctx);
int udp_tx_fini(struct udp_tx *ctx);
int udp_tx_send(struct udp_tx *ctx, const uint8_t *buf, size_t len);

#endif // SYNAPSE_UDP_UDP_TX_H_
// vi: ts=4 sw=4 et
