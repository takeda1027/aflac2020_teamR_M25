//
//  LineTracer.cpp
//  aflac2020
//
//  Copyright © 2019 Ahiruchan Koubou. All rights reserved.
//

#include "app.h"
#include "LineTracer.hpp"
#include "Observer.hpp"

LineTracer::LineTracer(Motor* lm, Motor* rm, Motor* tm) {
    _debug(syslog(LOG_NOTICE, "%08u, LineTracer constructor", clock->now()));
    leftMotor   = lm;
    rightMotor  = rm;
    trace_pwmLR = 0;
    speed       = SPEED_NORM;
    frozen      = false;
}

void LineTracer::haveControl() {
    activeNavigator = this;
    syslog(LOG_NOTICE, "%08u, LineTracer has control", clock->now());
}

void LineTracer::operate() {
    if (frozen) {
        forward = turn = 0; /* 障害物を検知したら停止 */

    }else if(cntl_p_flg){
        turn = calcPropP(); /* 比例制御*/
        forward = speed;

    }else {
        forward = speed; //前進命令
        /*
        // on-off control
        if (colorSensor->getBrightness() >= (LIGHT_WHITE + LIGHT_BLACK)/2) {
            turn =  20; // 左旋回命令
        } else {
            turn = -20; // 右旋回命令
        }
        */
        /*
        // PID control by brightness
        int16_t sensor = colorSensor->getBrightness();
        int16_t target = (LIGHT_WHITE + LIGHT_BLACK)/2;
        */
        // PID control by Gray Scale with blue cut
        int16_t sensor = g_grayScaleBlueless;
        int16_t target = GS_TARGET;

        turn = _EDGE * ltPid->compute(sensor, target);
        //turn = ltPid->compute(sensor, target);
    }

    /* 左右モータでロボットのステアリング操作を行う */
    pwm_L = forward - turn;
    pwm_R = forward + turn;

    leftMotor->setPWM(pwm_L);
    rightMotor->setPWM(pwm_R);

    // display pwm in every PERIOD_TRACE_MSG ms */
    // if (++trace_pwmLR * PERIOD_NAV_TSK >= PERIOD_TRACE_MSG) {
    //     trace_pwmLR = 0;
    //     _debug(syslog(LOG_NOTICE, "%08u, LineTracer::operate(): pwm_L = %d, pwm_R = %d", clock->now(), pwm_L, pwm_R));
    //     /*
    //     _debug(syslog(LOG_NOTICE, "%08u, LineTracer::operate(): distance = %d, azimuth = %d, x = %d, y = %d", clock->now(), observer->getDistance(), observer->getAzimuth(), observer->getLocX(), observer->getLocY()));
    //     */
    // }
    //printf("cntl_p_flg=%d,forward=%d, turn=%d, pwm_L = %d, pwm_R = %d\n",cntl_p_flg,forward, turn,pwm_L,pwm_R);
}

int8_t LineTracer::getSpeed() {
    return speed;
}

void LineTracer::setSpeed(int8_t s) {
    speed = s;
}

void LineTracer::freeze() {
    frozen = true;
}

void LineTracer::unfreeze() {
    frozen = false;
}

void LineTracer::setCntlP(bool p) {
    cntl_p_flg = p;
}

float LineTracer::calcPropP() {
  const float Kp = 0.83;
  const int target = 18;
  const int bias = 0;
  
  int diff = g_color_brightness - target; 
  //printf("ライントレース2通った g_color_brightness=%d\n",g_color_brightness);
  return (Kp * diff + bias);
}

LineTracer::~LineTracer() {
    _debug(syslog(LOG_NOTICE, "%08u, LineTracer destructor", clock->now()));
}