//
//  app.cpp
//  aflac2019
//
//  Created by Wataru Taniguchi on 2019/04/28.
//  Copyright © 2019 Ahiruchan Koubou. All rights reserved.
//

#include "app.h"

#include "Observer.hpp"
#include "Navigator.hpp"
#include "StateMachine.hpp"

Clock*          clock;
StateMachine*   stateMachine;
Observer*       observer;
Navigator*      activeNavigator = NULL;
uint8_t         state = ST_start;

// a cyclic handler to activate a task
void task_activator(intptr_t tskid) {
    ER ercd = act_tsk(tskid);
    assert(ercd == E_OK || E_QOVR);
    /*
    if (ercd != E_OK) {
        _debug(syslog(LOG_NOTICE, "%08lu, act_tsk() returned %d", clock->now(), ercd));
    }
    */
}

// Observer's periodic task
void observer_task(intptr_t unused) {
    if (observer != NULL) observer->operate();
}

// Navigator's periodic task
void navigator_task(intptr_t unused) {
    if (activeNavigator != NULL) activeNavigator->operate();
}

void main_task(intptr_t unused) {
    clock    = new Clock;
    stateMachine  = new StateMachine;

    stateMachine->initialize();
    
    // sleep until being waken up
    ER ercd = slp_tsk();
    assert(ercd == E_OK);

    stateMachine->exit();

    delete stateMachine;
    delete clock;
    ext_tsk();
}
