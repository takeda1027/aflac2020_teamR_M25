//
//  app.h
//  aflac2019
//
//  Created by Wataru Taniguchi on 2019/05/08.
//  Copyright © 2019 Ahiruchan Koubou. All rights reserved.
//

#ifndef app_h
#define app_h

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Common header files
 */
#include "ev3api.h"
#include "target_test.h"

/**
 * Task priorities (smaller number has higher priority)
 */
#define PRIORITY_OBS_TSK    TMIN_APP_TPRI
#define PRIORITY_NAV_TSK    TMIN_APP_TPRI
#define PRIORITY_MAIN_TASK  (TMIN_APP_TPRI + 1)

/**
 * Task periods in micro seconds
 * Note: It used to be in ms with HRP2 kernel)
 */
//#define PERIOD_OBS_TSK  (40 * 1000)
#define PERIOD_OBS_TSK  ( 4 * 1000)
#define PERIOD_NAV_TSK  ( 4 * 1000)
#define PERIOD_TRACE_MSG    1000 * 1000   /* Trace message in every 1000 ms */

/**
 * Default task stack size in bytes
 */
#ifndef STACK_SIZE
#define STACK_SIZE      4096
#endif /* STACK_SIZE */
    
/**
 * Prototypes for configuration
 */
#ifndef TOPPERS_MACRO_ONLY

extern void main_task(intptr_t unused);
extern void observer_task(intptr_t unused);
extern void navigator_task(intptr_t unused);

extern void task_activator(intptr_t tskid);

#endif /* TOPPERS_MACRO_ONLY */

#ifdef __cplusplus
}
#endif
#endif /* app_h */
