//
//  ChallengeRunner.cpp
//  aflac2020
//
//  Copyright © 2020 Ahiruchan Koubou. All rights reserved.
//

#include "app.h"
#include "ChallengeRunner.hpp"

ChallengeRunner::ChallengeRunner(Motor* lm, Motor* rm, Motor* tm, Motor* am) : LineTracer(lm, rm, tm){
    _debug(syslog(LOG_NOTICE, "%08u, ChallengeRunner constructor", clock->now()));
    leftMotor   = lm;
    rightMotor  = rm;
    armMotor =am;
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

//　Activate challengeRunner PWM control according to g_challenge_stepNo
void ChallengeRunner::runChallenge() {

    switch (g_challenge_stepNo) {
        //スラローム専用処理
        case 0:
            printf("ぶつかり\n");
            haveControl();
            setPwmLR(20,20,Mode_speed_constant,1);
            clock->sleep(800);
            setPwmLR(10,10,Mode_speed_constant,1);
            clock->sleep(1000);
            rest();
            break;
        case 1:
            setPwmLR(-20,-20,Mode_speed_constant,1);
            clock->sleep(500);
            rest();
            if (_LEFT == 1){
                setPwmLR(43,40,Mode_speed_decreaseLR,20);
            }else{
                setPwmLR(40,43,Mode_speed_decreaseLR,20);
            }
            break;
        case 10:
            rest();
            setPwmLR(3,15,Mode_speed_constant,1);
            break;
        case 11:
            rest();
            setPwmLR(-5,-15,Mode_speed_constant,1);
            break;            
        case 12:
            rest();
            setPwmLR(15,3,Mode_speed_constant,1);
            break;
        case 20:
            rest();
            setPwmLR(-10,10,Mode_speed_constant,1);
            break; 
        case 21:
            rest();
            setPwmLR(30,30,Mode_speed_constant,1);
            break; 
        case 22:
            rest();
            setPwmLR(10,-10,Mode_speed_constant,1);
            break;
        case 30:
            rest();
            setPwmLR(30,30,Mode_speed_constant,1);
            break;
        case 40:
            rest();
            setPwmLR(30,40,Mode_speed_constant,1);
            break;
        case 41:
            rest();
            setPwmLR(15,-13,Mode_speed_constant,1);
            break;
        case 50:
            rest();            
            if (_LEFT == 1){
                setPwmLR(14,16,Mode_speed_increaseR,80);
            }else{
                setPwmLR(16,14,Mode_speed_increaseL,80);
            }
            break;
        case 60:
            rest();  
            if (_LEFT == 1){
                setPwmLR(,15,Mode_speed_decreaseL,90);
                clock->sleep(150);                
                setPwmLR(0,15,Mode_speed_decreaseL,90);
            }else{
                setPwmLR(15,7,Mode_speed_decreaseL,90);
                clock->sleep(150);    
                setPwmLR(15,0,Mode_speed_decreaseR,90);
            }
            break;
       case 70:
            rest();
            setPwmLR(20,20,Mode_speed_constant,1);
            break;
        case 71:
            rest();
            if (_LEFT == 1){
                setPwmLR(-12,18,Mode_speed_constant,1);
                clock->sleep(1000);
            }else{
                setPwmLR(18,-12,Mode_speed_constant,1);
                clock->sleep(1000);
            }
            break;
        case 80:
            if (_LEFT == 1){
                setPwmLR(28,28,Mode_speed_constant,1);
            }else{
                setPwmLR(28,28,Mode_speed_constant,1);
            }
            break;
        case 90:
            pwm_L = _EDGE * 10;
            pwm_R = _EDGE * 1;
            setPwmLR(pwm_L,pwm_R,Mode_speed_constant,1);
            break;
        case 100:
            if (_LEFT == 1){
                setPwmLR(32,0,Mode_speed_decreaseL,100);
            }else{
                setPwmLR(0,32,Mode_speed_decreaseR,100);
            }
            break;
        case 110:
            pwm_L = _EDGE * 15;
            pwm_R = _EDGE * -15;
            setPwmLR(pwm_L,pwm_R,Mode_speed_constant,1);
            break;
        case 120:
            if (_LEFT == 1){
                setPwmLR(25,28,Mode_speed_increaseL,100);
            }else{
                setPwmLR(28,25,Mode_speed_increaseR,100);
            }
            break;
        case 130:
            pwm_L = _EDGE * 15;
            pwm_R = _EDGE * -15;
            setPwmLR(pwm_L,pwm_R,Mode_speed_constant,1);
            break;
        case 140:
            if (_LEFT == 1){
                setPwmLR(27,25,Mode_speed_constant,1);
            }else{
                setPwmLR(25,27,Mode_speed_constant,1);
            }
            break;

        //ボーナスブロック専用処理
        case 151:
            rest();
            if (_LEFT == 1){
                setPwmLR(8,-8,Mode_speed_constant,1);
            }else{
                setPwmLR(-8,8,Mode_speed_constant,1);
            }
            break;
        case 170:
            setPwmLR(50,50,Mode_speed_constant,1);
            break;
        case 190:
            rest();
            if (_LEFT == 1){
                setPwmLR(50,-50,Mode_speed_constant,1);
            }else{
                setPwmLR(-50,50,Mode_speed_constant,1);
            }
            clock->sleep(620);
            rest();            
            break;
        case 200:
            rest();
            if (_LEFT == 1){
                setPwmLR(10,0,Mode_speed_constant,1);
            }else{
                setPwmLR(0,10,Mode_speed_constant,1);
            }
            break;
        case 201:
            setPwmLR(30,30,Mode_speed_constant,1);
            break;
        case 210:
            if (_LEFT == 1){
                setPwmLR(0,40,Mode_speed_constant,1);
            }else{
                setPwmLR(40,0,Mode_speed_constant,1);
            }
            break;
        case 211:
            setPwmLR(30,30,Mode_speed_constant,1);
            break;
        case 230:
            rest();
            if (_LEFT == 1){
                setPwmLR(-15,50,Mode_speed_constant,1);
            }else{
                setPwmLR(50,-15,Mode_speed_constant,1);
            }
            break;
        case 231:
            rest();
            setPwmLR(30,30,Mode_speed_constant,1);
            break;
        case 240:
            setPwmLR(30,30,Mode_speed_constant,1);
            break;
        case 244:
            if (_LEFT == 1){
                setPwmLR(4,30,Mode_speed_constant,1);
            }else{
                setPwmLR(30,4,Mode_speed_constant,1);
            }
            break;
        case 260:
            if (_LEFT == 1){
                setPwmLR(4,30,Mode_speed_constant,1);
            }else{
                setPwmLR(30,4,Mode_speed_constant,1);
            }
            break;
         case 261:
            if (_LEFT == 1){
                setPwmLR(2,20,Mode_speed_constant,1);
            }else{
                setPwmLR(20,2,Mode_speed_constant,1);
            }
            break;
        case 263:
            setPwmLR(30,30,Mode_speed_constant,1);
            break;
        case 280:
            setPwmLR(15,15,Mode_speed_constant,1);
            break;
        case 281:
            if (_LEFT == 1){
                setPwmLR(15,13,Mode_speed_constant,1);
            }else{
                setPwmLR(13,15,Mode_speed_constant,1);
            }
            break;
        case 282:
            rest();
            if (_LEFT == 1){
                setPwmLR(8,-8,Mode_speed_constant,1);
            }else{
                setPwmLR(-8,8,Mode_speed_constant,1);
            }
            break;
        case 284:
            setPwmLR(15,15,Mode_speed_constant,1);
            break;
        case 290:
            setPwmLR(0,0,Mode_speed_constant,1);
            freeze();
            break;
        default:
            break;
    }
}

//　左右の車輪に駆動にそれぞれ値を指定する
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
    clock->sleep(300);
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