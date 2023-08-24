#include <synapse/zbus/syn_pub_sub.h>

int syn_sub_init(
    struct syn_sub* sub,
    void* msg,
    const struct zbus_channel* chan)
{
    sub->msg = msg;
    sub->chan = chan;
    k_poll_signal_init(&sub->signal);
    k_poll_event_init(&sub->events[0],
        K_POLL_TYPE_SIGNAL,
        K_POLL_MODE_NOTIFY_ONLY,
        &sub->signal);
    return k_mutex_init(&sub->mutex);
}

int syn_sub_poll(struct syn_sub* sub, k_timeout_t timeout)
{
    int ret = k_poll(sub->events, 1, timeout);
    sub->events[0].signal->signaled = 0;
    sub->events[0].state = K_POLL_STATE_NOT_READY;
    return ret;
}

int syn_sub_listen(struct syn_sub* sub, const struct zbus_channel* chan, k_timeout_t timeout)
{
    if (chan == sub->chan) {
        k_mutex_lock(&sub->mutex, timeout);
        memcpy(sub->msg, sub->chan->message, sub->chan->message_size);
        k_mutex_unlock(&sub->mutex);
        k_poll_signal_raise(&sub->signal, 0x1);
    }
    return 0;
}

int syn_sub_claim(struct syn_sub* sub, k_timeout_t timeout)
{
    return k_mutex_lock(&sub->mutex, timeout);
}

int syn_sub_finish(struct syn_sub* sub)
{
    return k_mutex_unlock(&sub->mutex);
}

int syn_pub_init(
    struct syn_pub* pub,
    void* msg,
    const struct zbus_channel* chan)
{
    k_mutex_init(&pub->mutex);
    pub->msg = msg;
    pub->chan = chan;
    return 0;
}

int syn_pub_claim(struct syn_pub* pub, k_timeout_t timeout)
{
    return k_mutex_lock(&pub->mutex, timeout);
}

int syn_pub_finish(struct syn_pub* pub)
{
    return k_mutex_unlock(&pub->mutex);
}

int syn_pub_publish(struct syn_pub* pub, k_timeout_t timeout)
{
    syn_pub_claim(pub, timeout);
    int ret = zbus_chan_pub(pub->chan, pub->msg, timeout);
    syn_pub_finish(pub);
    return ret;
}

int syn_sub_nocopy_init(
    struct syn_sub_nocopy* sub,
    const struct zbus_channel* chan)
{
    sub->chan = chan;
    k_poll_signal_init(&sub->signal);
    k_poll_event_init(&sub->events[0],
        K_POLL_TYPE_SIGNAL,
        K_POLL_MODE_NOTIFY_ONLY,
        &sub->signal);
    return 0;
}

int syn_sub_nocopy_poll(struct syn_sub_nocopy* sub, k_timeout_t timeout)
{
    int ret = k_poll(sub->events, 1, timeout);
    sub->events[0].signal->signaled = 0;
    sub->events[0].state = K_POLL_STATE_NOT_READY;
    return ret;
}

int syn_sub_nocopy_listen(struct syn_sub_nocopy* sub, k_timeout_t timeout)
{
    return k_poll_signal_raise(&sub->signal, 0x1);
}

int syn_sub_nocopy_claim(struct syn_sub_nocopy* sub, k_timeout_t timeout)
{
    return zbus_chan_claim(sub->chan, timeout);
}

int syn_sub_nocopy_finish(struct syn_sub_nocopy* sub)
{
    return zbus_chan_finish(sub->chan);
}

// vi: ts=4 sw=4 et
