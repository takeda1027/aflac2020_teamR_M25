//
//  StateMachine.cpp
//  aflac2020
//
//  Copyright © 2019 Ahiruchan Koubou. All rights reserved.
//

#include "app.h"
#include "StateMachine.hpp"
#include "Observer.hpp"
#include "LineTracer.hpp"


StateMachine::StateMachine() {
    _debug(syslog(LOG_NOTICE, "%08u, StateMachine default constructor", clock->now()));
}

void StateMachine::initialize() {
    /* 各オブジェクトを生成・初期化する */
    touchSensor = new TouchSensor(PORT_1);
    sonarSensor = new SonarSensor(PORT_2);
    colorSensor = new ColorSensor(PORT_3);
    gyroSensor  = new GyroSensor(PORT_4);
    leftMotor   = new Motor(PORT_C);
    rightMotor  = new Motor(PORT_B);
    tailMotor   = new Motor(PORT_D);
    armMotor   = new Motor(PORT_A);
    steering    = new Steering(*leftMotor, *rightMotor);
    
    /* LCD画面表示 */
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("EV3way-ET aflac2020", 0, CALIB_FONT_HEIGHT*1);
    
    observer = new Observer(leftMotor, rightMotor, armMotor, tailMotor, touchSensor, sonarSensor, gyroSensor, colorSensor);    observer->freeze(); // Do NOT attempt to collect sensor data until unfreeze() is invoked
    observer->activate();
    blindRunner = new BlindRunner(leftMotor, rightMotor, tailMotor);
    lineTracer = new LineTracer(leftMotor, rightMotor, tailMotor);
    lineTracer->activate();
    challengeRunner = new ChallengeRunner(leftMotor, rightMotor, tailMotor);
    challengeRunner->activate();
    
    ev3_led_set_color(LED_ORANGE); /* 初期化完了通知 */

    state = ST_start;
}

void StateMachine::sendTrigger(uint8_t event) {
    syslog(LOG_NOTICE, "%08u, StateMachine::sendTrigger(): event %s received by state %s", clock->now(), eventName[event], stateName[state]);
    switch (state) {
        case ST_start:
            switch (event) {
                case EVT_cmdStart_R:
                case EVT_cmdStart_L:
                case EVT_touch_On:
                    state = ST_tracing;
                    syslog(LOG_NOTICE, "%08u, Departing...", clock->now());
                    
                    /* 走行モーターエンコーダーリセット */
                    leftMotor->reset();
                    rightMotor->reset();
                    
                    observer->reset();
                    
                    /* ジャイロセンサーリセット */
                    gyroSensor->reset();
                    ev3_led_set_color(LED_GREEN); /* スタート通知 */
                    
                    observer->freeze();
                    lineTracer->freeze();
                    lineTracer->haveControl();
                    //clock->sleep() seems to be still taking milisec parm
                    clock->sleep(PERIOD_NAV_TSK*FIR_ORDER/1000); // wait until FIR array is filled
                    lineTracer->unfreeze();
                    observer->unfreeze();
                    syslog(LOG_NOTICE, "%08u, Departed", clock->now());
                    observer->notifyOfDistance(700); // switch to ST_Blind after 700
                   break;
                default:
                    break;
            }
            break;
        case ST_tracing:
            switch (event) {
                case EVT_backButton_On:
                    state = ST_end;
                    wakeupMain();
                    break;
                case EVT_sonar_On:
                case EVT_sonar_Off:
                    break;
                case EVT_dist_reached:
                    state = ST_blind;
                    blindRunner->haveControl();
                    break;
                case EVT_bl2bk:
                case EVT_bk2bl:
                    /*
                    // stop at the start of blue line
                    observer->freeze();
                    lineTracer->freeze();
                    //clock->sleep() seems to be still taking milisec parm
                    clock->sleep(5000); // wait a little
                    lineTracer->unfreeze();
                    observer->unfreeze();
                    */
                    break;
                case EVT_cmdStop:
                    state = ST_stopping;
                    observer->notifyOfDistance(FINAL_APPROACH_LEN);
                    lineTracer->haveControl();
                    break;
                default:
                    break;
            }
            break;
        case ST_stopping:
            switch (event) {
                case EVT_backButton_On:
                case EVT_dist_reached:
                    state = ST_end;
                    wakeupMain();
                    break;
                default:
                    break;
            }
            break;
        case ST_challenge_L:
            switch (event) {
                case EVT_slalom_reached://sano_t　分割
                    printf("ぶつかり\n");
                    armMotor->setPWM(-50);
                    challengeRunner->haveControl();
                    challengeRunner->setPwmLR(20,20,Mode_speed_constant,1);
                    clock->sleep(1000);
                    challengeRunner->setPwmLR(10,10,Mode_speed_constant,1);
                    clock->sleep(1000);
                    challengeRunner->rest();
                break;
                case EVT_slalom_reached_af: //sano_t
                    challengeRunner->setPwmLR(-20,-20,Mode_speed_constant,1);
                    clock->sleep(500);
                    armMotor->setPWM(80);
                    challengeRunner->rest();
                    challengeRunner->setPwmLR(43,40,Mode_speed_decreaseLR,40);
                    break;
                case EVT_right_curve: //sano_t
                    challengeRunner->setPwmLR(15,3,Mode_speed_constant,1);
                    break;
                case EVT_left_curve: //sano_t
                    challengeRunner->setPwmLR(3,15,Mode_speed_constant,1);
                    break;
                case EVT_right_curve_rev: //sano_t
                    challengeRunner->setPwmLR(-15,-5,Mode_speed_constant,1);
                    break;
                case EVT_left_curve_rev: //sano_t
                    challengeRunner->setPwmLR(-5,-15,Mode_speed_constant,1);
                    break;
                case EVT_pause: //sano_t
                    challengeRunner->rest();
                    break;
                case EVT_left_turn: //sano_t
                    challengeRunner->setPwmLR(-10,10,Mode_speed_constant,1);
                    break;
                case EVT_go_straight: //sano_t
                    challengeRunner->setPwmLR(30,30,Mode_speed_constant,1);
                    break;
                case EVT_right_turn: //sano_t
                    challengeRunner->setPwmLR(10,-10,Mode_speed_constant,1);
                    break;
                 case EVT_obstcl_angle:
                    if (challenge_stepNo == 2){
                        challengeRunner->rest();
                        challengeRunner->setPwmLR(15,-15,Mode_speed_incrsRdcrsL,90);
                        challenge_stepNo += 1;
                    }else if(challenge_stepNo == 6){
                        challengeRunner->rest();
                        //challengeRunner->setPwmLR(1,15,Mode_speed_decreaseL,90);
                        //sano_t
                        challengeRunner->setPwmLR(0,15,Mode_speed_decreaseL,90);
                        challenge_stepNo += 1;
                    }else if(challenge_stepNo == 9){
                        challengeRunner->rest();
                        // challengeRunner->setPwmLR(27,-27,Mode_speed_constant,1);
                        // clock->sleep(450);
                        challengeRunner->setPwmLR(15,-15,Mode_speed_incrsRdcrsL,90);
                        challenge_stepNo += 1;
                    }else if(challenge_stepNo == 13){
                        challengeRunner->rest();
                        challengeRunner->setPwmLR(15,-15,Mode_speed_incrsRdcrsL,150);
                        challenge_stepNo += 1;
                    }
                    break;
                case EVT_obstcl_infront:
                    if (challenge_stepNo == 3){
                        challengeRunner->rest();
                        challengeRunner->setPwmLR(32,27,Mode_speed_decreaseL,70);
                        challenge_stepNo += 1;
                    }else if(challenge_stepNo == 10){
                        challengeRunner->rest();
                        //challengeRunner->setPwmLR(30,20,Mode_speed_decreaseL,120);
                        challengeRunner->setPwmLR(32,25,Mode_speed_decreaseL,100);
                        challenge_stepNo += 1;
                    }
                    break;
                case EVT_obstcl_reached:
                    if (challenge_stepNo == 0){
                        challengeRunner->rest();
                        challengeRunner->setPwmLR(-10,12,Mode_speed_incrsLdcrsR,100);
                        challenge_stepNo += 1;
                    }else if(challenge_stepNo == 4){
                        challengeRunner->rest();
                        //sano_t 削除
                        //challengeRunner->setPwmLR(-15,-15,Mode_speed_constant,1);
                        //clock->sleep(300);
                        challengeRunner->setPwmLR(15,-13,Mode_speed_constant,1);
                        challenge_stepNo += 1;
                    }else if(challenge_stepNo == 7){
                        challengeRunner->rest();
                        //challengeRunner->setPwmLR(-15,10,Mode_speed_constant,1);
                        //clock->sleep(500);
                        printf("切り替わり１\n");
                        //sano_t
                        challengeRunner->setPwmLR(10,10,Mode_speed_constant,1);
                        clock->sleep(250);
                        printf("切り替わり２\n");
                        //challengeRunner->setPwmLR(-12,15,Mode_speed_constant,1);
                        challengeRunner->setPwmLR(3,15,Mode_speed_constant,1);
                        clock->sleep(1000);
                        printf("切り替わり３\n");
                        challenge_stepNo += 1;
                    }else if(challenge_stepNo == 11){
                        challengeRunner->rest();
                        challengeRunner->setPwmLR(15,-15,Mode_speed_incrsRdcrsL,110);
                        challenge_stepNo += 1;
                    }
                    break;
                case EVT_obstcl_avoidable:
                    if (challenge_stepNo == 1){
                        challengeRunner->rest();
                        challengeRunner->setPwmLR(25,20,Mode_speed_increaseL,150);
                        challenge_stepNo += 1;
                    }else if(challenge_stepNo == 5){
                        challengeRunner->rest();
                        challengeRunner->setPwmLR(14,16,Mode_speed_increaseR,80);
                        challenge_stepNo += 1;
                    }else if(challenge_stepNo == 8){
                        challengeRunner->rest();
                        challengeRunner->setPwmLR(10,28,Mode_speed_constant,1);
                        challenge_stepNo += 1;
                    }else if(challenge_stepNo == 12){
                        challengeRunner->rest();
                        challengeRunner->setPwmLR(25,25,Mode_speed_increaseR,30);
                        challenge_stepNo += 1;
                    }else if (challenge_stepNo == 14){
                        challengeRunner->rest();
                        challengeRunner->setPwmLR(25,30,Mode_speed_incrsLdcrsR,100);
                        challenge_stepNo += 1;
                    }
                    break;
                default:
                    break;
            }
            break;
        case ST_end:
            break;
        default:
            break;
    }
}

void StateMachine::wakeupMain() {
    syslog(LOG_NOTICE, "%08u, Ending...", clock->now());
    ER ercd = wup_tsk(MAIN_TASK); // wake up the main task
    assert(ercd == E_OK);
}

void StateMachine::exit() {
    if (activeNavigator != NULL) {
        activeNavigator->deactivate();
    }
    leftMotor->reset();
    rightMotor->reset();
    
    delete lineTracer;
    delete blindRunner;
    delete challengeRunner;
    observer->deactivate();
    delete observer;
    
    delete tailMotor;
    delete armMotor;
    delete rightMotor;
    delete leftMotor;
    delete gyroSensor;
    delete colorSensor;
    delete sonarSensor;
    delete touchSensor;
}

StateMachine::~StateMachine() {
    _debug(syslog(LOG_NOTICE, "%08u, StateMachine destructor", clock->now()));
}