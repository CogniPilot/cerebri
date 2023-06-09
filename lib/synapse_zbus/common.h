/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef SYNAPSE_ZBUS_COMMON_H
#define SYNAPSE_ZBUS_COMMON_H

#include <pb_decode.h>
#include <pb_encode.h>

#include <synapse_tinyframe/SynapseTopics.h>
#include <synapse_tinyframe/TinyFrame.h>
#include <synapse_tinyframe/utils.h>

#include <synapse_zbus/channels.h>

#define TOPIC_LISTENER(CHANNEL, CLASS)                                           \
    static TF_Result CHANNEL##_Listener(TinyFrame* tf, TF_Msg* frame)            \
    {                                                                            \
        CLASS msg = CLASS##_init_zero;                                           \
        pb_istream_t stream = pb_istream_from_buffer(frame->data, frame->len);   \
        int status = pb_decode(&stream, CLASS##_fields, &msg);                   \
        if (status) {                                                            \
            zbus_chan_pub(&chan_##CHANNEL, &msg, K_FOREVER);                     \
        } else {                                                                 \
            printf("%s decoding failed: %s\n", #CHANNEL, PB_GET_ERROR(&stream)); \
        }                                                                        \
        return TF_STAY;                                                          \
    }

#define TOPIC_PUBLISHER(CHANNEL, CLASS, TOPIC)                                   \
    else if (chan == &chan_##CHANNEL)                                            \
    {                                                                            \
        TF_Msg msg;                                                              \
        TF_ClearMsg(&msg);                                                       \
        uint8_t buf[500];                                                        \
        pb_ostream_t stream = pb_ostream_from_buffer((pu8)buf, sizeof(buf));     \
        int status = pb_encode(&stream, CLASS##_fields, chan->message);          \
        if (status) {                                                            \
            msg.type = TOPIC;                                                    \
            msg.data = buf;                                                      \
            msg.len = stream.bytes_written;                                      \
            TF_Send(&g_tf, &msg);                                                \
        } else {                                                                 \
            printf("%s encoding failed: %s\n", #CHANNEL, PB_GET_ERROR(&stream)); \
        }                                                                        \
    }

#endif // SYNAPSE_ZBUS_MACROS_H
/* vi: ts=4 sw=4 et */
