#include <zephyr/device.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/zbus/zbus.h>

#include <cerebri/synapse/zbus/syn_pub_sub.h>

LOG_MODULE_DECLARE(synapse_zbus);

int syn_sub_init(
    syn_sub_t* sub,
    void* msg,
    const struct zbus_channel* chan)
{
    __ASSERT(sub != NULL, "sub is null");
    __ASSERT(msg != NULL, "msg is null");
    sub->next = NULL;
    sub->msg = msg;
    sub->chan = chan;
    k_poll_signal_init(&sub->signal);
    k_poll_event_init(&sub->events[0],
        K_POLL_TYPE_SIGNAL,
        K_POLL_MODE_NOTIFY_ONLY,
        &sub->signal);
    return k_mutex_init(&sub->mutex);
}

int syn_sub_poll(syn_sub_t* sub, k_timeout_t timeout)
{
    __ASSERT(sub != NULL, "sub is null");

    int ret = k_poll(sub->events, 1, timeout);
    sub->events[0].signal->signaled = 0;
    sub->events[0].state = K_POLL_STATE_NOT_READY;
    return ret;
}

int syn_sub_listen(syn_sub_t* sub, const struct zbus_channel* chan, k_timeout_t timeout)
{
    __ASSERT(sub != NULL, "sub is null");
    __ASSERT(chan != NULL, "chan is null");

    if (chan == sub->chan) {
        RC(k_mutex_lock(&sub->mutex, timeout), return rc);
        // since we are in a zbus listener, zbus channel is already mutex locked
        memcpy(sub->msg, sub->chan->message, sub->chan->message_size);
        RC(k_mutex_unlock(&sub->mutex), return rc);
        RC(k_poll_signal_raise(&sub->signal, 0x1), return rc);
    }
    return 0;
}

int syn_sub_lock(syn_sub_t* sub, k_timeout_t timeout)
{
    __ASSERT(sub != NULL, "sub is null");

    RC(k_mutex_lock(&sub->mutex, timeout), return rc);
    return 0;
}

int syn_sub_unlock(syn_sub_t* sub)
{
    __ASSERT(sub != NULL, "sub is null");

    RC(k_mutex_unlock(&sub->mutex), return rc);
    return 0;
}

int syn_pub_init(
    syn_pub_t* pub,
    void* msg,
    const struct zbus_channel* chan)
{
    __ASSERT(pub != NULL, "pub is null");
    __ASSERT(msg != NULL, "msg is null");
    __ASSERT(chan != NULL, "chan is null");

    pub->next = NULL;
    RC(k_mutex_init(&pub->mutex), return rc);
    pub->msg = msg;
    pub->chan = chan;
    return 0;
}

int syn_pub_lock(syn_pub_t* pub, k_timeout_t timeout)
{
    __ASSERT(pub != NULL, "pub is null");

    RC(k_mutex_lock(&pub->mutex, timeout), return rc);
    return 0;
}

int syn_pub_unlock(syn_pub_t* pub)
{
    __ASSERT(pub != NULL, "pub is null");

    RC(k_mutex_unlock(&pub->mutex), return rc);
    return 0;
}

int syn_pub_publish(syn_pub_t* pub, k_timeout_t timeout)
{
    __ASSERT(pub != NULL, "pub is null");
    RC(zbus_chan_pub(pub->chan, pub->msg, timeout), return rc);
    return 0;
}

int syn_pub_forward(syn_pub_t* pub, const struct zbus_channel* chan, const struct zbus_channel* chan_src, k_timeout_t timeout)
{
    __ASSERT(pub != NULL, "pub is null");
    __ASSERT(chan != NULL, "chan is null");
    __ASSERT(chan_src != NULL, "chan_src is null");

    int ret = 0;
    if (chan == chan_src) {
        RC(syn_pub_lock(pub, timeout), return rc);
        RC(zbus_chan_pub(pub->chan, chan->message, timeout), return rc);
        RC(syn_pub_unlock(pub), return rc);
    }
    return ret;
}

int syn_node_add_sub(syn_node_t* node,
    syn_sub_t* sub,
    void* msg,
    const struct zbus_channel* chan)
{
    __ASSERT(node != NULL, "node is null");
    __ASSERT(sub != NULL, "sub is null");
    __ASSERT(msg != NULL, "msg is null");

    RC(syn_sub_init(sub, msg, chan), return rc);
    syn_sub_t* tail = node->sub_list_head;
    if (tail == NULL) {
        node->sub_list_head = sub;
    } else {
        while (tail->next != NULL) {
            tail = tail->next;
        }
        tail->next = sub;
    }
    return 0;
}

void syn_node_init(syn_node_t* node, const char* name)
{
    node->name = name;
    node->sub_list_head = NULL;
    node->pub_list_head = NULL;
}

int syn_node_add_pub(syn_node_t* node,
    syn_pub_t* pub,
    void* msg,
    const struct zbus_channel* chan)
{
    __ASSERT(node != NULL, "node is null");
    __ASSERT(pub != NULL, "pub is null");
    __ASSERT(msg != NULL, "msg is null");
    __ASSERT(chan != NULL, "chan is null");

    RC(syn_pub_init(pub, msg, chan), return rc);
    syn_pub_t* tail = node->pub_list_head;
    if (tail == NULL) {
        node->pub_list_head = pub;
    } else {
        while (tail->next != NULL) {
            tail = tail->next;
        }
        tail->next = pub;
    }
    return 0;
}

void syn_node_listen(syn_node_t* node,
    const struct zbus_channel* chan, k_timeout_t timeout)
{
    for (syn_sub_t* sub = node->sub_list_head; sub != NULL; sub = sub->next) {
        int ret = syn_sub_listen(sub, chan, timeout);
        if (ret < 0) {
            LOG_ERR("sub listen timeout for node %s, chan %s",
                node->name,
                zbus_chan_name(sub->chan));
            continue;
        }
    }
}

void syn_node_unlock_all(syn_node_t* node)
{
    // unlock all subs
    for (syn_sub_t* sub = node->sub_list_head; sub != NULL; sub = sub->next) {
        int ret = syn_sub_unlock(sub);
        if (ret < 0) {
            LOG_ERR("sub unlock timeout for node %s, chan %s",
                node->name,
                zbus_chan_name(sub->chan));
            continue;
        }
    }

    // unlock all pubs
    for (syn_pub_t* pub = node->pub_list_head; pub != NULL; pub = pub->next) {
        int ret = syn_pub_unlock(pub);
        if (ret < 0) {
            LOG_ERR("pub unlock timeout for node %s, chan %s",
                node->name,
                zbus_chan_name(pub->chan));
            continue;
        }
    }
}

void syn_node_lock_all(syn_node_t* node, k_timeout_t timeout)
{
    // lock all subs
    for (syn_sub_t* sub = node->sub_list_head; sub != NULL; sub = sub->next) {
        int ret = syn_sub_lock(sub, timeout);
        if (ret < 0) {
            LOG_ERR("sub lock timeout for node %s, chan %s",
                node->name,
                zbus_chan_name(sub->chan));
            continue;
        }
    }

    // lock all pubs
    for (syn_pub_t* pub = node->pub_list_head; pub != NULL; pub = pub->next) {
        int ret = syn_pub_lock(pub, timeout);
        if (ret < 0) {
            LOG_ERR("pub lock timeout for node %s, chan %s",
                node->name,
                zbus_chan_name(pub->chan));
            continue;
        }
    }
}

void syn_node_publish_all(syn_node_t* node, k_timeout_t timeout)
{
    // publish all pubs
    for (syn_pub_t* pub = node->pub_list_head; pub != NULL; pub = pub->next) {
        int ret = syn_pub_publish(pub, timeout);
        if (ret < 0) {
            LOG_ERR("pub timeout for node %s, chan %s",
                node->name,
                zbus_chan_name(pub->chan));
            continue;
        }
    }
}

// vi: ts=4 sw=4 et
