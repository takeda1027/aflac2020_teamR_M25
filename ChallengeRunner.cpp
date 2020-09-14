//
//  ChallengeRunner.cpp
//  aflac2020
//
//  Copyright © 2020 Ahiruchan Koubou. All rights reserved.
//

#include "app.h"
#include "ChallengeRunner.hpp"

ChallengeRunner::ChallengeRunner(Motor* lm, Motor* rm, Motor* tm) : LineTracer(lm, rm, tm){
    _debug(syslog(LOG_NOTICE, "%08u, ChallengeRunner constructor", clock->now()));
    leftMotor   = lm;
    rightMotor  = rm;
    pwm_L = 20;
    pwm_R = 20;
    pwmMode = 1;
    count = 0;
    procCount = 1;
    traceCnt = 0;
    frozen = false;
}

void ChallengeRunner::haveControl() {
    activeNavigator = this;
    syslog(LOG_NOTICE, "%08u, ChallengeRunner has control", clock->now());
}

void ChallengeRunner::operate() {

    if (frozen) {
        //printf("Stop");
        pwm_L = 0;
        pwm_R = 0;
    }else{
        if (pwmMode != Mode_speed_constant){
            if (++count && count == procCount){
                switch (pwmMode) {
                    case Mode_speed_increaseL:
                        ++pwm_L;
                        break;
                    case Mode_speed_decreaseL:
                        --pwm_L;
                        break;
                    case Mode_speed_increaseR:
                        ++pwm_R;
                        break;
                    case Mode_speed_decreaseR:
                        --pwm_R;
                        break;
                    case Mode_speed_increaseLR:
                        ++pwm_L;
                        ++pwm_R;
                        break;
                    case Mode_speed_decreaseLR:
                        --pwm_L;
                        --pwm_R;
                        break;
                    case Mode_speed_incrsLdcrsR:
                        ++pwm_L;
                        --pwm_R;
                        break;
                    case Mode_speed_incrsRdcrsL:
                        --pwm_L;
                        ++pwm_R;
                        break;
                    default:
                        break;
                }
                count = 0; //初期化
            }
        }
    }
    
    leftMotor->setPWM(pwm_L);
    rightMotor->setPWM(pwm_R);

    // if (++traceCnt && traceCnt > 50) {
    //     printf(",pwm_L=%d, pwm_R=%d, count=%d, procCount=%d\n", pwm_L,pwm_R,count,procCount);
    //     traceCnt = 0;
    // }
}

//　左右の車輪に駆動にそれぞれ値を指定する sano
void ChallengeRunner::setPwmLR(int p_L,int p_R,int mode,int proc_count) {
    pwm_L = p_L;
    pwm_R = p_R;
    pwmMode = mode;
    procCount = proc_count;
    count = 0;
}

// rest for a while
void ChallengeRunner::rest() {
    freeze();
    clock->sleep(1000); //sano_t
    unfreeze();
}

int8_t ChallengeRunner::getPwmL() {
    return pwm_L;
}

int8_t ChallengeRunner::getPwmR() {
    return pwm_R;
}

ChallengeRunner::~ChallengeRunner() {
    _debug(syslog(LOG_NOTICE, "%08u, ChallengeRunner destructor", clock->now()));
}