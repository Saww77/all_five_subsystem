#include "heartbeat.h"
#include "../app.h"
#include "../car_config.h"

static ID timer;

static void heartbeat_task(INT argument, void *context)
{
    UINT events;
    bool on = false;

    (void)argument;
    (void)context;
    gpio_set_val(PIN_HEARTBEAT, 0u);
    gpio_set_pin(PIN_HEARTBEAT, GPIO_MODE_OUT);
    for (;;) {
        if (tk_wai_flg(timer, EVENT_TICK, TWF_ORW | TWF_BITCLR, &events, TMO_FEVR) >= E_OK) {
            on = !on;
            gpio_set_val(PIN_HEARTBEAT, on);
        }
    }
}

bool heartbeat_start(void)
{
    timer = app_events();
    return timer > 0 && app_periodic(timer, 250u) && app_start((FP)heartbeat_task, 17, 512);
}
