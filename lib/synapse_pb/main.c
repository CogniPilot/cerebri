 /*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include <pb_encode.h>
#include <pb_decode.h>
#include "synapse_pb/twist.pb.h"

#include "tinyframe/TinyFrame.h"
#include "tinyframe/demo/utils.h"

#define MY_STACK_SIZE 500
#define MY_PRIORITY 5

#define TX_BUF_SIZE 100
#define RX_BUF_SIZE 100

#define TYPE_TWIST 0x01

#define BIND_PORT 4242

/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

static uint8_t tx0_buf[TX_BUF_SIZE];

static uint8_t tx1_buf[TX_BUF_SIZE];
static uint8_t rx1_buf[RX_BUF_SIZE];

static int client = 0;

/**
 * This function should be defined in the application code.
 * It implements the lowest layer - sending bytes to UART (or other)
 */
void TF_WriteImpl(TinyFrame *tf, const uint8_t *buf, uint32_t len)
{
  // uart
  if (tf->usertag == 0) {
    for (int i=0; i<len; i++) {
      uart_poll_out(uart_dev, buf[i]);
    }
  }
  // ethernet
  else if (tf->usertag == 1) {
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
    for (int i=0; i<len; i++) {
      uart_poll_out(uart_dev, buf[i]);
    }
  }
}

static TF_Result genericListener(TinyFrame *tf, TF_Msg *msg)
{
    dumpFrameInfo(msg);
    return TF_STAY;
}

static TF_Result twistListener(TinyFrame *tf, TF_Msg *msg)
{
  /* Allocate space for the decoded message. */
  Twist message = Twist_init_zero;

  /* Create a stream that reads from the buffer. */
  pb_istream_t stream = pb_istream_from_buffer(msg->data, msg->len);

  /* Now we are ready to decode the message. */
  int status = pb_decode(&stream, Twist_fields, &message);

  /* Check for errors... */
  if (status) {
    /* Print the data contained in the message. */
    printk("%d %d %d %d %d %d", 
      message.linear.x, message.linear.y, message.linear.z,
      message.angular.x, message.angular.y, message.angular.z);
  } else {
    printk("Decoding failed: %s\n", PB_GET_ERROR(&stream));
  }
  return TF_STAY;
}

static void uart_entry_point(void)
{
    TF_Msg msg;

    // Set up the TinyFrame library
    TinyFrame *tf0;
    tf0 = TF_Init(TF_MASTER); // 1 = master, 0 = slave
    tf0->usertag = 0;
    TF_AddGenericListener(tf0, genericListener);
    TF_AddTypeListener(tf0, TYPE_TWIST, twistListener);

  while (true) {

    // send twist message
    {
      Twist message = Twist_init_zero;
      pb_ostream_t tx_stream = pb_ostream_from_buffer(tx0_buf, Twist_size);
      pb_encode(&tx_stream, Twist_fields, &message);
      
      TF_ClearMsg(&msg);
      msg.type = TYPE_TWIST;
      msg.len = tx_stream.bytes_written;
      msg.data = (pu8) tx0_buf;
      TF_Send(tf0, &msg);
    }

    // receive messages
    {
      uint8_t c;
      int count = 0;
      while (uart_poll_in(uart_dev, &c) == 0) {
        TF_AcceptChar(tf0, c);
        count++;
      }
    }

    // sleep 0.01 seconds
    k_busy_wait(10000);

    // should move TF tick to a clock thread
    TF_Tick(tf0);
  }
}

K_THREAD_DEFINE(protobuf_ethernet_tid, MY_STACK_SIZE, uart_entry_point,
    NULL, NULL, NULL, MY_PRIORITY, 0, 0);


static void ethernet_entry_point(void)
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
  static TinyFrame *tf0;
  tf0 = TF_Init(TF_MASTER); // 1 = master, 0 = slave
  tf0->usertag = 1;
  TF_AddGenericListener(tf0, genericListener);
  TF_AddTypeListener(tf0, TYPE_TWIST, twistListener);

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
      // send twist message
      {
        Twist message = Twist_init_zero;
        message.has_linear = true;
        message.linear.x = 1;
        message.linear.y = 2;
        message.linear.z = 3;
        message.has_angular = true;
        message.angular.x = 4;
        message.angular.y = 5;
        message.angular.z = 6;
        pb_ostream_t tx_stream = pb_ostream_from_buffer(tx1_buf, Twist_size);
        pb_encode(&tx_stream, Twist_fields, &message);
        
        TF_ClearMsg(&msg);
        msg.type = TYPE_TWIST;
        msg.len = tx_stream.bytes_written;
        msg.data = (pu8) tx1_buf;
        TF_Send(tf0, &msg);
      }

      // receive messages
      {
        int len = recv(client, rx1_buf, sizeof(rx1_buf), 0);
        for (int i=0;i < len; i++) {
          TF_AcceptChar(tf0, rx1_buf[i]);
        }
      }

      // sleep
      k_busy_wait(100000);

      // should move tf tick to a clock thread
      TF_Tick(tf0);
    }
  }
}

K_THREAD_DEFINE(protobuf_uart_tid, MY_STACK_SIZE, ethernet_entry_point,
    NULL, NULL, NULL, MY_PRIORITY, 0, 0);

