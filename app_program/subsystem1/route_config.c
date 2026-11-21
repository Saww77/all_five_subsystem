#include "routes.h"
#include "../app.h"

static routes_t active_routes;

void route_config_init(void)
{
    active_routes = routes_default();
}

void route_config_snapshot(routes_t *routes)
{
    UW key;

    DI(key);
    *routes = active_routes;
    EI(key);
}

bool route_config_apply(const routes_t *routes)
{
    UW key;

    if (!routes_valid(routes)) { return false; }
    DI(key);
    active_routes = *routes;
    EI(key);
    return true;
}

uint8_t route_config_action(char letter)
{
    routes_t routes;

    route_config_snapshot(&routes);
    return routes_get(&routes, letter);
}
