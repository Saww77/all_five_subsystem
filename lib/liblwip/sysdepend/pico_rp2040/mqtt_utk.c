#include "mqtt_utk.h"
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/sys.h"
#include "wifi_secrets.h"
#include "../../../../app_program/subsystem1/telemetry.h"
#include "../../../../app_program/subsystem1/routes.h"
#include <string.h>

#ifndef MQTT_TOPIC_ROOT
#define MQTT_TOPIC_ROOT "robot/" MQTT_CLIENT_ID "/v3"
#endif
#define STATE_TOPIC MQTT_TOPIC_ROOT "/state"
#define AVAIL_TOPIC MQTT_TOPIC_ROOT "/online"
#define COMMAND_TOPIC MQTT_TOPIC_ROOT "/command"
#define ROUTE_SET_TOPIC MQTT_TOPIC_ROOT "/routes/set"
#define ROUTE_STATE_TOPIC MQTT_TOPIC_ROOT "/routes/state"

#define RETRY_MS 3000u
#define HEARTBEAT_MS 5000u
#define ACTIVE_UPDATE_MS 2000u
#define CHANGE_INTERVAL_MS 350u

enum { RECEIVE_NONE, RECEIVE_COMMAND, RECEIVE_ROUTES };

extern void car_stop_command(void);

static mqtt_client_t *client;
static ip_addr_t broker;
static car_status_t last_status;
static u32_t retry;
static u32_t subscribe_retry;
static u32_t last_sent;
static u32_t send_retry;
static routes_t received;
static unsigned rx_size;
static uint8_t rx_kind;
static bool connecting;
static bool connected;
static bool subscribing;
static bool command_subscribed;
static bool routes_subscribed;
static bool announce;
static bool want_status;
static bool want_routes;

static bool deadline(u32_t now, u32_t then)
{
    return (s32_t)(now - then) >= 0;
}

static void on_topic(void *arg, const char *topic, u32_t length)
{
    (void)arg;
    rx_size = 0u;
    rx_kind = RECEIVE_NONE;
    if (length == 1u && strcmp(topic, COMMAND_TOPIC) == 0) {
        rx_kind = RECEIVE_COMMAND;
    } else if (length == ROUTE_BYTES && strcmp(topic, ROUTE_SET_TOPIC) == 0) {
        rx_kind = RECEIVE_ROUTES;
    }
}

static void on_data(void *arg, const u8_t *data, u16_t length, u8_t flags)
{
    unsigned expected;

    (void)arg;
    if (rx_kind == RECEIVE_NONE) { return; }
    expected = rx_kind == RECEIVE_COMMAND ? 1u : ROUTE_BYTES;
    if (length > expected - rx_size) {
        rx_kind = RECEIVE_NONE;
        return;
    }
    if (length != 0u) { memcpy(received.bytes + rx_size, data, length); }
    rx_size += length;
    if ((flags & MQTT_DATA_FLAG_LAST) == 0u) { return; }

    if (rx_size == expected) {
        if (rx_kind == RECEIVE_ROUTES) {
            if (route_config_apply(&received)) { want_routes = true; }
        } else {
            if (received.bytes[0] == MQTT_CAR_STOP) { car_stop_command(); }
            if (received.bytes[0] == MQTT_CAR_STOP || received.bytes[0] == MQTT_CAR_STATUS) {
                want_status = true;
                want_routes |= received.bytes[0] == MQTT_CAR_STATUS;
            }
        }
    }
    rx_kind = RECEIVE_NONE;
}

static void subscribed_routes(void *arg, err_t result)
{
    (void)arg;
    subscribing = false;
    routes_subscribed = connected && result == ERR_OK;
}

static void subscribed_commands(void *arg, err_t result)
{
    (void)arg;
    subscribing = false;
    command_subscribed = connected && result == ERR_OK;
    if (command_subscribed) { subscribe_retry = sys_now(); }
}

static void on_connection(mqtt_client_t *c, void *arg, mqtt_connection_status_t status)
{
    (void)arg;
    connecting = false;
    connected = status == MQTT_CONNECT_ACCEPTED;
    command_subscribed = false;
    routes_subscribed = false;
    subscribing = false;
    rx_kind = RECEIVE_NONE;
    if (connected) {
        mqtt_set_inpub_callback(c, on_topic, on_data, NULL);
        subscribe_retry = sys_now();
        send_retry = subscribe_retry;
        announce = true;
        want_routes = true;
        want_status = true;
    } else {
        retry = sys_now() + RETRY_MS;
    }
}

static void connect_to_broker(u32_t now)
{
    struct mqtt_connect_client_info_t info = {
        .client_id = MQTT_CLIENT_ID,
        .keep_alive = 16u,
        .will_topic = AVAIL_TOPIC,
        .will_msg = "\x00",
        .will_msg_len = 1u,
        .will_qos = 0u,
        .will_retain = 1u
    };
#ifdef MQTT_USERNAME
    info.client_user = MQTT_USERNAME;
    info.client_pass = MQTT_PASSWORD;
#endif
    retry = now + RETRY_MS;
    if (client == NULL) { client = mqtt_client_new(); }
    if (client == NULL || !ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&broker))) { return; }
    IP_SET_TYPE(&broker, IPADDR_TYPE_V4);
    connecting = true;
    if (mqtt_client_connect(client, &broker, MQTT_BROKER_PORT, on_connection,
                            NULL, &info) != ERR_OK) {
        connecting = false;
    }
}

static bool important_change(car_status_t current)
{
    return current.phase != last_status.phase || current.faults != last_status.faults ||
           current.barcode != last_status.barcode || current.plan != last_status.plan ||
           current.max_hump_mm != last_status.max_hump_mm ||
           current.obstacle_width_mm != last_status.obstacle_width_mm;
}

void mqtt_utk_poll(int wifi_up)
{
    u32_t now = sys_now();

    if (!wifi_up) {
        /* Let the broker emit the last will when the network transport dies. */
        if (client != NULL && (connected || connecting)) { mqtt_disconnect(client); }
        connected = false;
        connecting = false;
        command_subscribed = false;
        routes_subscribed = false;
        rx_kind = RECEIVE_NONE;
        return;
    }
    if (!connected) {
        if (!connecting && deadline(now, retry)) { connect_to_broker(now); }
        return;
    }
    if (!subscribing && !(command_subscribed && routes_subscribed) &&
        deadline(now, subscribe_retry)) {
        subscribing = true;
        subscribe_retry = now + RETRY_MS;
        if (!command_subscribed) {
            if (mqtt_subscribe(client, COMMAND_TOPIC, 1, subscribed_commands, NULL) != ERR_OK) {
                subscribing = false;
            }
        } else if (mqtt_subscribe(client, ROUTE_SET_TOPIC, 1, subscribed_routes, NULL) != ERR_OK) {
            subscribing = false;
        }
    }
    if (!deadline(now, send_retry)) { return; }
    send_retry = now + 150u;
    if (announce && command_subscribed) {
        static const uint8_t online = 1u;
        announce = mqtt_publish(client, AVAIL_TOPIC, &online, 1, 0, 1, NULL, NULL) != ERR_OK;
    }
    if (want_routes && command_subscribed && routes_subscribed) {
        routes_t routes;
        route_config_snapshot(&routes);
        want_routes = mqtt_publish(client, ROUTE_STATE_TOPIC, routes.bytes, ROUTE_BYTES,
                                   1, 1, NULL, NULL) != ERR_OK;
    }

    car_status_t current = controller_status();
    unsigned elapsed = now - last_sent;
    bool active = current.phase >= CAR_FOLLOW && current.phase <= CAR_REACQUIRE;
    bool refresh = elapsed >= (active ? ACTIVE_UPDATE_MS : HEARTBEAT_MS);
    if (want_status || refresh || (elapsed >= CHANGE_INTERVAL_MS && important_change(current))) {
        uint8_t bytes[TELEMETRY_BYTES];
        telemetry_encode(current, bytes);
        if (mqtt_publish(client, STATE_TOPIC, bytes, sizeof(bytes), 0, 0, NULL, NULL) == ERR_OK) {
            last_status = current;
            last_sent = now;
            want_status = false;
        }
    }
}
