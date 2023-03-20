/**
 * Copyright (c) 2023, CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#include <string.h>

/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <pb_encode.h>
#include <pb_decode.h>
#include "src/simple.pb.h"

#include <stdio.h>
#include <string.h>
#include "TinyFrame/TinyFrame.h"
#include "TinyFrame/demo/utils.h"

#define TX_BUF_SIZE 100
#define RX_BUF_SIZE 100

#define TYPE_HELLO 0x21
#define TYPE_SIMPLE 0x22

uint8_t tx_buf[TX_BUF_SIZE];
uint8_t rx_buf[RX_BUF_SIZE];

TinyFrame *tf0;

int value = 0;

/**
 * This function should be defined in the application code.
 * It implements the lowest layer - sending bytes to UART (or other)
 */
void TF_WriteImpl(TinyFrame *tf, const uint8_t *buff, uint32_t len)
{
	for (int i=0; i<len; i++) {
		uart_poll_out(uart_dev, buff[i]);
	}
}

/** An example listener function */
TF_Result myListener(TinyFrame *tf, TF_Msg *msg)
{
    dumpFrameInfo(msg);
    return TF_STAY;
}

TF_Result testIdListener(TinyFrame *tf, TF_Msg *msg)
{
    printf("OK - ID Listener triggered for msg!\n");
    dumpFrameInfo(msg);
    return TF_CLOSE;
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
    TF_Msg msg;

    // Set up the TinyFrame library
    tf0 = TF_Init(TF_MASTER); // 1 = master, 0 = slave
    TF_AddGenericListener(tf0, myListener);
	TF_AddTypeListener(tf0, TYPE_SIMPLE, simpleListener);

	while (true) {

		// send Hello message
		{
			//TF_ClearMsg(&msg);
			//msg.type = TYPE_HELLO;
			//msg.data = (pu8) "Hello TinyFrame";
			//msg.len = 16;
			//TF_Send(tf0, &msg);
		}

		// send Simple message
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
