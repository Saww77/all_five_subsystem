#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "../vehicle_controller/controller.h"

#define TELEMETRY_BYTES 13u
#define TELEMETRY_VERSION 3u
#define MQTT_CAR_STOP 1u
#define MQTT_CAR_STATUS 2u

void telemetry_encode(car_status_t status, uint8_t bytes[TELEMETRY_BYTES]);

#endif
