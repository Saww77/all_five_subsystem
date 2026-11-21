#include "app.h"
#include "hardware.h"
#include "subsystem1/heartbeat.h"
#include "subsystem1/routes.h"
#include "subsystem3/line.h"
#include "subsystem4/terrain.h"
#include "subsystem5/obstacle.h"
#include "vehicle_controller/controller.h"

#if TK_SUPPORT_SMP
#error Build the car with SMP=0. The shared data and interrupt owner use one CPU.
#endif

ID controller_events;
volatile uint8_t car_faults;

ID app_events(void)
{
    T_CFLG options = { .flgatr = TA_TFIFO, .iflgptn = 0 };
    return tk_cre_flg(&options);
}

static void periodic(void *argument)
{
    (void)tk_set_flg((ID)(uintptr_t)argument, EVENT_TICK);
}

bool app_periodic(ID events, unsigned period_ms)
{
    T_CCYC options = {
        .exinf = (void *)(uintptr_t)events,
        .cycatr = TA_HLNG | TA_STA,
        .cychdr = (FP)periodic,
        .cyctim = period_ms,
        .cycphs = period_ms
    };
    return tk_cre_cyc(&options) > 0;
}

bool app_start(FP entry, PRI priority, unsigned stack)
{
    T_CTSK options = { .task = entry, .itskpri = priority,
                       .stksz = (SZ)stack, .tskatr = TA_HLNG | TA_RNG3 };
    ID task = tk_cre_tsk(&options);
    return task > 0 && tk_sta_tsk(task, 0) >= E_OK;
}

void car_fault(uint8_t faults)
{
    UW key;

    DI(key);
    car_faults |= faults;
    EI(key);
    if ((faults & FATAL_FAULTS) != 0u) {
        hardware_brake(true);
        if (controller_events > 0) { (void)tk_set_flg(controller_events, EVENT_STOP); }
    }
}

void car_stop_command(void)
{
    car_fault(FAULT_ESTOP);
}

EXPORT INT usermain(void)
{
    route_config_init();
    controller_events = app_events();
    if (controller_events <= 0 || !hardware_init() || !line_start() ||
        !terrain_start() || !sonar_start() || !heartbeat_start() || !controller_start()) {
        car_faults |= FAULT_TIMING;
        tm_printf((UB *)"Vehicle initialization failed\n");
    } else {
        hardware_watchdog_start();
    }
    for (;;) {
        (void)tk_slp_tsk(TMO_FEVR);
    }
}
