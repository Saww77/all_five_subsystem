#include "routes.h"
#include "../car_config.h"

static unsigned bit_position(char letter)
{
    return 2u + 3u * (unsigned)(letter - 'A');
}

uint8_t routes_get(const routes_t *routes, char letter)
{
    if (letter < 'A' || letter > 'Z') { return ROUTE_NONE; }

    unsigned bit = bit_position(letter);
    unsigned byte = bit / 8u;
    unsigned shift = bit % 8u;
    unsigned word = routes->bytes[byte];

    if (shift > 5u) { word |= (unsigned)routes->bytes[byte + 1u] << 8; }
    return (uint8_t)((word >> shift) & 7u);
}

bool routes_set(routes_t *routes, char letter, unsigned action)
{
    if (letter < 'A' || letter > 'Z' || action > ROUTE_UTURN) { return false; }

    unsigned bit = bit_position(letter);
    unsigned byte = bit / 8u;
    unsigned shift = bit % 8u;
    unsigned word = routes->bytes[byte];

    if (shift > 5u) { word |= (unsigned)routes->bytes[byte + 1u] << 8; }
    word = (word & ~(7u << shift)) | (action << shift);
    routes->bytes[byte] = (uint8_t)word;
    if (shift > 5u) { routes->bytes[byte + 1u] = (uint8_t)(word >> 8); }
    return true;
}

bool routes_valid(const routes_t *routes)
{
    if ((routes->bytes[0] & 3u) != ROUTE_VERSION) { return false; }

    for (char letter = 'A'; letter <= 'Z'; ++letter) {
        if (routes_get(routes, letter) > ROUTE_UTURN) { return false; }
    }
    return true;
}

routes_t routes_default(void)
{
    routes_t routes = { .bytes = { ROUTE_VERSION } };

#if BARCODE_PROGRESS_MAP
    (void)routes_set(&routes, 'C', ROUTE_LEFT);
    (void)routes_set(&routes, 'F', ROUTE_RIGHT);
    (void)routes_set(&routes, 'A', ROUTE_STRAIGHT);
    (void)routes_set(&routes, 'B', ROUTE_UTURN);
#else
    (void)routes_set(&routes, 'A', ROUTE_LEFT);
    (void)routes_set(&routes, 'B', ROUTE_RIGHT);
    (void)routes_set(&routes, 'C', ROUTE_STRAIGHT);
    (void)routes_set(&routes, 'D', ROUTE_UTURN);
#endif
    return routes;
}
