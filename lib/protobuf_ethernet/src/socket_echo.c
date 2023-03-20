 /*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include <zephyr/net/socket.h>
#include <zephyr/kernel.h>

#include <pb_encode.h>
#include <pb_decode.h>

#include "synapse/simple.pb.h"
#include "TinyFrame/TinyFrame.h"
#include "TinyFrame/demo/utils.h"

#define TX_BUF_SIZE 100
#define RX_BUF_SIZE 100

#define TYPE_HELLO 0x21
#define TYPE_SIMPLE 0x22

#define BIND_PORT 4242

static uint8_t tx_buf[TX_BUF_SIZE];
static uint8_t rx_buf[RX_BUF_SIZE];

static TinyFrame *tf0;

static int value = 0;
static int client = 0;

/**
 * This function should be defined in the application code.
 * It implements the lowest layer - sending bytes to UART (or other)
 */
void TF_WriteImpl(TinyFrame *tf, const uint8_t *buf, uint32_t len)
{
  int out_len;
  const char *p;
  p = buf;
  do {
    out_len = send(client, p, len, 0);
    if (out_len < 0) {
      printf("error: send: %d\n", errno);
      return;
    }
    p += out_len;
    len -= out_len;
  } while (len);
}

TF_Result myListener(TinyFrame *tf, TF_Msg *msg)
{
    dumpFrameInfo(msg);
    return TF_STAY;
}

TF_Result simpleListener(TinyFrame *tf, TF_Msg *msg)
{
  /* Allocate space for the decoded message. */
  SimpleMessage message = SimpleMessage_init_zero;

  /* Create a stream that reads from the buffer. */
  pb_istream_t stream = pb_istream_from_buffer(msg->data, msg->len);

  /* Now we are ready to decode the message. */
  int status = pb_decode(&stream, SimpleMessage_fields, &message);

  /* Check for errors... */
  if (status) {
    /* Print the data contained in the message. */
    printk("%lld: %d\n", message.clock, (int)message.lucky_number);
    value = message.lucky_number;
  } else {
    printk("Decoding failed: %s\n", PB_GET_ERROR(&stream));
  }
  return TF_STAY;
}


void main(void)
{
  int serv;
  struct sockaddr_in bind_addr;
  static int counter;

  serv = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

  if (serv < 0) {
    printf("error: socket: %d\n", errno);
    exit(1);
  }

  bind_addr.sin_family = AF_INET;
  bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  bind_addr.sin_port = htons(BIND_PORT);

  if (bind(serv, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
    printf("error: bind: %d\n", errno);
    exit(1);
  }

  if (listen(serv, 5) < 0) {
    printf("error: listen: %d\n", errno);
    exit(1);
  }

  printf("Single-threaded TCP echo server waits for a connection on "
         "port %d...\n", BIND_PORT);

  TF_Msg msg;

  // Set up the TinyFrame library
  tf0 = TF_Init(TF_MASTER); // 1 = master, 0 = slave
  TF_AddGenericListener(tf0, myListener);
  TF_AddTypeListener(tf0, TYPE_SIMPLE, simpleListener);

  while (1) {
    struct sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(client_addr);
    char addr_str[32];
    client = accept(serv, (struct sockaddr *)&client_addr,
            &client_addr_len);

    if (client < 0) {
      printf("error: accept: %d\n", errno);
      continue;
    }

    inet_ntop(client_addr.sin_family, &client_addr.sin_addr,
        addr_str, sizeof(addr_str));
    printf("Connection #%d from %s\n", counter++, addr_str);

    while (1) {
      // send simple message
      {
        SimpleMessage message = SimpleMessage_init_zero;
        message.lucky_number = value;
        message.clock = k_uptime_get();
        pb_ostream_t tx_stream = pb_ostream_from_buffer(tx_buf, SimpleMessage_size);
        pb_encode(&tx_stream, SimpleMessage_fields, &message);
        
        TF_ClearMsg(&msg);
        msg.type = TYPE_SIMPLE;
        msg.len = tx_stream.bytes_written;
        msg.data = (pu8) tx_buf;
        TF_Send(tf0, &msg);
      }

      // receive messages
      {
        int len = recv(client, rx_buf, sizeof(rx_buf), 0);
        for (int i=0;i < len; i++) {
          TF_AcceptChar(tf0, rx_buf[i]);
        }
      }

      // sleep
      k_busy_wait(100000);

      // should move tf tick to a clock thread
      TF_Tick(tf0);
    }
  }
}
