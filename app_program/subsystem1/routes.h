#ifndef ROUTES_H
#define ROUTES_H

#include <stdbool.h>
#include <stdint.h>

enum { ROUTE_NONE, ROUTE_LEFT, ROUTE_RIGHT, ROUTE_STRAIGHT, ROUTE_UTURN };

#define ROUTE_BYTES 10u
#define ROUTE_VERSION 1u

typedef struct {
    uint8_t bytes[ROUTE_BYTES];
} routes_t;

routes_t routes_default(void);
uint8_t routes_get(const routes_t *routes, char letter);
bool routes_set(routes_t *routes, char letter, unsigned action);
bool routes_valid(const routes_t *routes);

void route_config_init(void);
void route_config_snapshot(routes_t *routes);
bool route_config_apply(const routes_t *routes);
uint8_t route_config_action(char letter);

#endif
