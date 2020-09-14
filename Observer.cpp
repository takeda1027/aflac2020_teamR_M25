//
//  Observer.cpp
//  aflac2020
//
//  Copyright © 2019 Ahiruchan Koubou. All rights reserved.
//

#include "app.h"
#include "Observer.hpp"
#include "StateMachine.hpp"

// global variables to pass FIR-filtered color from Observer to Navigator and its sub-classes
rgb_raw_t g_rgb;
hsv_raw_t g_hsv;
int16_t g_grayScale, g_grayScaleBlueless;
// global variables to gyro sensor output from Observer to  Navigator and its sub-classes
int16_t g_angle, g_anglerVelocity;
int8_t challenge_stepNo;

Observer::Observer(Motor* lm, Motor* rm, Motor* am, Motor* tm, TouchSensor* ts, SonarSensor* ss, GyroSensor* gs, ColorSensor* cs) {
    _debug(syslog(LOG_NOTICE, "%08u, Observer constructor", clock->now()));
    leftMotor   = lm;
    rightMotor  = rm;
    armMotor = am;
    tailMotor = tm;
    touchSensor = ts;
    sonarSensor = ss;
    gyroSensor  = gs;
    colorSensor = cs;
    distance = 0.0;
    azimuth = 0.0;
    locX = 0.0;
    locY = 0.0;
    prevAngL = 0;
    prevAngR = 0;
    notifyDistance = 0;
    traceCnt = 0;
    prevGS = INT16_MAX;
    touch_flag = false;
    sonar_flag = false;
    backButton_flag = false;
    lost_flag = false;
    blue_flag = false;
    prevDis = 0;//sano_t
    prevDisX = 0;//sano_t
    prevDisY = 0;//sano_t
    //ot_r = new OutlierTester(OLT_SKIP_PERIOD/PERIOD_OBS_TSK, OLT_INIT_PERIOD/PERIOD_OBS_TSK);
    //ot_g = new OutlierTester(OLT_SKIP_PERIOD/PERIOD_OBS_TSK, OLT_INIT_PERIOD/PERIOD_OBS_TSK);
    //ot_b = new OutlierTester(OLT_SKIP_PERIOD/PERIOD_OBS_TSK, OLT_INIT_PERIOD/PERIOD_OBS_TSK);
    
    // For Challenge Run
    line_over_flg = false;
    blue2_flg = true;
    move_back_flg = false;
    slalom_flg = false;
    curRgbSum = 0;
    prevRgbSum = 0;
    curAngle = 0;
    prevAngle = 0;
    curDegree = 0;
    prevDegree = 0;
    challenge_stepNo = 0;
    gyroSensor->setOffset(0);//sano

    fir_r = new FIR_Transposed<FIR_ORDER>(hn);
    fir_g = new FIR_Transposed<FIR_ORDER>(hn);
    fir_b = new FIR_Transposed<FIR_ORDER>(hn);
    ma = new MovingAverage<int32_t, MA_CAP>();
}

void Observer::activate() {
    // register cyclic handler to EV3RT
    sta_cyc(CYC_OBS_TSK);
    //clock->sleep() seems to be still taking milisec parm
    clock->sleep(PERIOD_OBS_TSK/2/1000); // wait a while
    _debug(syslog(LOG_NOTICE, "%08u, Observer handler set", clock->now()));
}

void Observer::reset() {
    distance = 0.0;
    azimuth = 0.0;
    locX = 0.0;
    locY = 0.0;
    prevAngL = leftMotor->getCount();
    prevAngR = rightMotor->getCount();
}

void Observer::notifyOfDistance(int32_t delta) {
    notifyDistance = delta + distance;
}

int32_t Observer::getDistance() {
    return (int32_t)distance;
}

int16_t Observer::getAzimuth() {
    // degree = 360.0 * radian / M_2PI;
    int16_t degree = (360.0 * azimuth / M_2PI);
    return degree;
}

int16_t Observer::getDegree() {
    // degree = 360.0 * radian / M_2PI;
    int16_t degree = (360.0 * azimuth / M_2PI);
    if (degree > 180){
        degree -= 360;
    }
    return degree;
}

int32_t Observer::getLocX() {
    return (int32_t)locX;
}

int32_t Observer::getLocY() {
    return (int32_t)locY;
}

void Observer::operate() {
    colorSensor->getRawColor(cur_rgb);
    // process RGB by the Low Pass Filter
    cur_rgb.r = fir_r->Execute(cur_rgb.r);
    cur_rgb.g = fir_g->Execute(cur_rgb.g);
    cur_rgb.b = fir_b->Execute(cur_rgb.b);
    curRgbSum = cur_rgb.r + cur_rgb.g + cur_rgb.b;
    rgb_to_hsv(cur_rgb, cur_hsv);
    // save filtered color variables to the global area
    g_rgb = cur_rgb;
    g_hsv = cur_hsv;
    // calculate gray scale and save them to the global area
    g_grayScale = (cur_rgb.r * 77 + cur_rgb.g * 150 + cur_rgb.b * 29) / 256;
    g_grayScaleBlueless = (cur_rgb.r * 77 + cur_rgb.g * 150 + (cur_rgb.b - cur_rgb.g) * 29) / 256; // B - G cuts off blue
    // save gyro sensor output to the global area
    g_angle = gyroSensor->getAngle();
    g_anglerVelocity = gyroSensor->getAnglerVelocity();

    // accumulate distance
    int32_t curAngL = leftMotor->getCount();
    int32_t curAngR = rightMotor->getCount();
    double deltaDistL = M_PI * TIRE_DIAMETER * (curAngL - prevAngL) / 360.0;
    double deltaDistR = M_PI * TIRE_DIAMETER * (curAngR - prevAngR) / 360.0;
    double deltaDist = (deltaDistL + deltaDistR) / 2.0;
    distance += deltaDist;
    prevAngL = curAngL;
    prevAngR = curAngR;
    // calculate azimuth
    double deltaAzi = atan2((deltaDistL - deltaDistR), WHEEL_TREAD);
    azimuth += deltaAzi;
    if (azimuth > M_2PI) {
        azimuth -= M_2PI;
    } else if (azimuth < 0.0) {
        azimuth += M_2PI;
    }
    // estimate location
    locX += (deltaDist * sin(azimuth));
    locY += (deltaDist * cos(azimuth));

    // monitor distance
    if ((notifyDistance != 0.0) && (distance > notifyDistance)) {
        syslog(LOG_NOTICE, "%08u, distance reached", clock->now());
        notifyDistance = 0.0; // event to be sent only once
        stateMachine->sendTrigger(EVT_dist_reached);
    }
    
    // monitor touch sensor
    bool result = check_touch();
    if (result && !touch_flag) {
        syslog(LOG_NOTICE, "%08u, TouchSensor flipped on", clock->now());
        touch_flag = true;
        stateMachine->sendTrigger(EVT_touch_On);
    } else if (!result && touch_flag) {
        syslog(LOG_NOTICE, "%08u, TouchSensor flipped off", clock->now());
        touch_flag = false;
        stateMachine->sendTrigger(EVT_touch_Off);
    }
    
    // monitor sonar sensor
    result = check_sonar();
    if (result && !sonar_flag) {
        syslog(LOG_NOTICE, "%08u, SonarSensor flipped on", clock->now());
        sonar_flag = true;
        stateMachine->sendTrigger(EVT_sonar_On);
    } else if (!result && sonar_flag) {
        syslog(LOG_NOTICE, "%08u, SonarSensor flipped off", clock->now());
        sonar_flag = false;
        stateMachine->sendTrigger(EVT_sonar_Off);
    }
    
    // monitor Back Button
    result = check_backButton();
    if (result && !backButton_flag) {
        syslog(LOG_NOTICE, "%08u, Back button flipped on", clock->now());
        backButton_flag = true;
        stateMachine->sendTrigger(EVT_backButton_On);
    } else if (!result && backButton_flag) {
        syslog(LOG_NOTICE, "%08u, Back button flipped off", clock->now());
        backButton_flag = false;
        stateMachine->sendTrigger(EVT_backButton_Off);
    }

    if (!frozen) { // these checks are meaningless thus bypassed when frozen
        // determine if still tracing the line
        result = check_lost();
        if (result && !lost_flag) {
            syslog(LOG_NOTICE, "%08u, line lost", clock->now());
            lost_flag = true;
            stateMachine->sendTrigger(EVT_line_lost);
        } else if (!result && lost_flag) {
            syslog(LOG_NOTICE, "%08u, line found", clock->now());
            lost_flag = false;
            stateMachine->sendTrigger(EVT_line_found);
        }

        // temporary dirty logic to detect the second black to blue change
        int32_t ma_gs;
        if (prevGS == INT16_MAX) {
            prevTime = clock->now();
            prevGS = g_grayScale;
            ma_gs = ma->add(0);
        } else {
            curTime = clock->now();
            gsDiff = g_grayScale - prevGS;
            timeDiff = curTime - prevTime;
            ma_gs = ma->add(gsDiff * 1000000 / timeDiff);
            prevTime = curTime;
            prevGS = g_grayScale;
        }

        if ( (ma_gs > 150) || (ma_gs < -150) ){
            //syslog(LOG_NOTICE, "gs = %d, MA = %d, gsDiff = %d, timeDiff = %d", g_grayScale, ma_gs, gsDiff, timeDiff);
            if ( !blue_flag && ma_gs > 150 && g_rgb.b - g_rgb.r > 60 && g_rgb.b <= 255 && g_rgb.r <= 255 ) {
                blue_flag = true;
                syslog(LOG_NOTICE, "%08u, line color changed black to blue", clock->now());
                stateMachine->sendTrigger(EVT_bk2bl);
            } else if ( blue_flag && ma_gs < -150 && g_rgb.b - g_rgb.r < 40 ) {
                blue_flag = false;
                syslog(LOG_NOTICE, "%08u, line color changed blue to black", clock->now());
                stateMachine->sendTrigger(EVT_bl2bk);
            }
        }

        // determine if tilt
        //if ( check_tilt() ) {
            //stateMachine->sendTrigger(EVT_cmdStop);
        //}
    }
    
// sano added
    curDegree = getDegree();
    sonarDistance = sonarSensor->getDistance();

    // blue2_flg ON
    if(cur_rgb.b - cur_rgb.r > 60 && sonarDistance < 50  && cur_rgb.b <=255 && cur_rgb.r<=255){
        blue2_flg =true;
    }

    // Preparation for slalom climbing
    if(blue2_flg && !slalom_flg){
        if (sonarDistance >= 1 && sonarDistance <= 10 && !move_back_flg){
            state = ST_challenge_L;
            stateMachine->sendTrigger(EVT_slalom_reached);

            locX = 0; //初期化 sano_t
            locY = 0; //初期化 sano_t 
            distance = 0;//初期化 sano_t
            prevAngL =0; //初期化 sano_t
            prevAngR =0; //初期化 sano_t
            leftMotor->reset(); //初期化 sano_t
            rightMotor->reset(); //初期化 sano_t
            azimuth = 0; //初期化 sano_t

            stateMachine->sendTrigger(EVT_slalom_reached_af);//sano_t
            armMotor->setPWM(30);
            move_back_flg = true;
        }

        // Robot tilts whenn
        curAngle = g_angle;
        if(curAngle < -9){
            prevAngle = curAngle;
        }
        if (prevAngle < -9 && curAngle >= 0){
            printf("スラロームオン\n");
            slalom_flg = true;
            curAngle = 0;//初期化
            prevAngle = 0;//初期化
            prevDis = distance; //sano_t
            prevDisY = locY; // sano_t
            prevDegree=getDegree();//sano_t
            // distance = 0;//初期化
            armMotor->setPWM(-100);
        }
    }

    //スラローム専用処理
    if(slalom_flg){

        // if (challenge_stepNo >= 9 && challenge_stepNo <= 14){
        //      if (++traceCnt && traceCnt > 1) {
        //          printf(",curDegree=%d, sonarDistance=%d, challenge_stepNo=%d,r+g+b=%d\n",curDegree, sonarDistance,challenge_stepNo,curRgbSum);
        //          traceCnt = 0;
        //      }
        // }
    //sano_t
    printf(",stat=%d,kakudo=%d,sner_dis=%d,distance=%lf,X=%lf,Y=%lf,r=%d,g=%d,b=%d\n",challenge_stepNo,getDegree(),sonarSensor->getDistance(),distance,locX,locY,cur_rgb.r, cur_rgb.g ,cur_rgb.b);

        //初期位置の特定
        if(challenge_stepNo == 0 && distance - prevDis > 35){
            prevDisX = locX;
            prevRgbSum = curRgbSum;
            stateMachine->sendTrigger(EVT_pause);
            if(curRgbSum < 100){
                //もともと黒の上にいる場合、ライン下方面に回転
                prevDisX = locX;
                challenge_stepNo = 120;
            }else{
                //左カーブ
                stateMachine->sendTrigger(EVT_left_curve);
                challenge_stepNo = 101;
            }
        }else if(challenge_stepNo == 101){
            //センサーで黒を検知した場合
            if(curRgbSum < 100 ){
                //その場でライン下方面に回転
                prevDisX = locX;
                challenge_stepNo = 120;
            }
            //左カーブで黒を見つけられなかった場合、リバース、元の位置まで戻る
            else if(own_abs(prevDisX-locX) > 200 ){
                if(prevRgbSum - curRgbSum > 20){
                    //黒に近づいていたら、もう少し頑張る
                }else{
                    //黒の片鱗も見えなければ逆走する
                    stateMachine->sendTrigger(EVT_pause);
                    stateMachine->sendTrigger(EVT_left_curve_rev);
                    challenge_stepNo = 102;
                }
            }
        //左カーブから戻る
        }else if(challenge_stepNo == 102){
            if(prevDisX - locX <= 0){
                //右カーブへ移行
                stateMachine->sendTrigger(EVT_pause);
                stateMachine->sendTrigger(EVT_right_curve);
                challenge_stepNo = 103;
            }
        }else if(challenge_stepNo == 103){
            //センサーで黒を検知した場合
            if(curRgbSum < 100 ){
                //その場でライン下方面に回転
                prevDisX = locX;
                challenge_stepNo = 120;
            }        
        }else if(challenge_stepNo == 120){
            //その場で左回転
            stateMachine->sendTrigger(EVT_pause);
            stateMachine->sendTrigger(EVT_left_turn);
            challenge_stepNo = 121;
        }else if(challenge_stepNo == 121){
            //角度が-45度になったら、停止、直進
            if(getDegree()<-48){
                stateMachine->sendTrigger(EVT_pause);
                stateMachine->sendTrigger(EVT_go_straight);
                challenge_stepNo = 122;
            }
        }else if(challenge_stepNo == 122){
            if(own_abs(prevDisX-locX)>110){
                //進行方向に対して横軸に距離進んだら、角度を0度に戻す
                stateMachine->sendTrigger(EVT_pause);
                stateMachine->sendTrigger(EVT_right_turn);
                challenge_stepNo = 123;
            }
        }else if(challenge_stepNo == 123){
            if(getDegree()>prevDegree+1){
                //進行方向に対して横軸に進んだら、角度を戻す
                stateMachine->sendTrigger(EVT_pause);
                stateMachine->sendTrigger(EVT_go_straight);
                challenge_stepNo = 4;
            }
        }


        //スラロームオン後、１つ目の障害物を見つける
        if (challenge_stepNo == 99 && distance - prevDis > 100 ){
            printf(",スラロームオン後、１つ目の障害物を見つける\n");
            stateMachine->sendTrigger(EVT_obstcl_reached);
         // １つ目の障害物に接近する
        }else if(challenge_stepNo == 1 && check_sonar(40,255) ){
            printf(",１つ目の障害物を回避する\n");
            stateMachine->sendTrigger(EVT_obstcl_avoidable);
            prevRgbSum = curRgbSum;
            line_over_flg = false;
        // 黒ラインを超えるまで前進し、超えたら向きを調整する
        }else if (challenge_stepNo == 2 && !line_over_flg){
             if (curRgbSum < 120) {
                 prevRgbSum = curRgbSum;
                 prevDisX=locX;//sano_t
             }
             //if((prevRgbSum < 120 && curRgbSum > 180)){
             if((prevRgbSum < 120 && curRgbSum > 150) || prevDisX-locX > 5 ){  //sano_t
                 printf(",黒ラインを超えるまで前進し、超えたら向きを調整する\n");
                 line_over_flg = true;
                 prevRgbSum = 0;
                 stateMachine->sendTrigger(EVT_obstcl_angle);
             }
             prevDegree = curDegree;
        // ２つ目の障害物に向かって前進する
        //}else if (challenge_stepNo == 3 && check_sonar(25,35) && own_abs(curDegree - prevDegree) > 60){
        }else if (challenge_stepNo == 3  && getDegree() >= 0){ //sano_t
            printf(",２つ目の障害物に向かって前進する\n");
            stateMachine->sendTrigger(EVT_obstcl_infront);
        // ２つ目の障害物に接近したら向きを変える
//        }else if (challenge_stepNo == 4 && check_sonar(0,5)){
        }else if (challenge_stepNo == 4 && (locY - prevDisY > 560 || check_sonar(0,5))){
            printf(",２つ目の障害物に接近したら向きを変える２\n");
            stateMachine->sendTrigger(EVT_obstcl_reached);
            prevRgbSum = curRgbSum;
            prevDegree=curDegree;
            line_over_flg = false;
        // 視界が晴れたら左上に前進する
        //}else if(challenge_stepNo == 5 && check_sonar(40,255)){
        }else if(challenge_stepNo == 5 && (check_sonar(50,255)|| own_abs(curDegree-prevDegree)>55)){ //sano_t
            printf(",視界が晴れたところで前進する");
            stateMachine->sendTrigger(EVT_obstcl_avoidable);
            prevRgbSum = curRgbSum;
            line_over_flg = false;
        // 黒ラインを超えるまで前進し、超えたら向きを調整し３つ目の障害物に接近する
        }else if (challenge_stepNo == 6 && !line_over_flg){
            if (curRgbSum < 100) {
                prevRgbSum = curRgbSum;
                prevDisY=locY;//sano_t
            }
            //if(prevRgbSum < 100 && curRgbSum > 150){
            if(prevRgbSum < 100 && curRgbSum > 120){
                printf(",黒ラインを超えたら向きを調整し障害物に接近する\n");
                stateMachine->sendTrigger(EVT_obstcl_angle);
                prevDegree=curDegree;//sano_t
                line_over_flg = true;
            }
        // ３つ目の障害物に接近したら後退して調整する
//        }else if (challenge_stepNo == 7 && check_sonar(0,5)){
        }else if (challenge_stepNo == 7 && own_abs(prevDegree-curDegree)>55){
                printf(",３つ目の障害物に接近したら後退して調整する\n");
                stateMachine->sendTrigger(EVT_obstcl_reached);
                line_over_flg = true;
                prevDegree = curDegree;
        // 視界が晴れたら左下に前進する
//        }else if (challenge_stepNo == 8 && check_sonar(20,255)){
        }else if (challenge_stepNo == 8){
                printf(",視界が晴れたら左下に前進する\n");
                stateMachine->sendTrigger(EVT_obstcl_avoidable);
                prevRgbSum = curRgbSum;
                line_over_flg = false;
        // 黒ラインを超えるまで前進し、超えたら向きを調整する
        }else if (challenge_stepNo == 9 && !line_over_flg){
            if (curRgbSum < 100) {
                prevRgbSum = curRgbSum;
            }
            if(prevRgbSum < 100 && curRgbSum > 195){
                printf(",黒ラインを超えたら向きを調整する\n");
                stateMachine->sendTrigger(EVT_obstcl_angle);
                line_over_flg = true;
            }
            prevDegree = curDegree;
        // ４つ目の障害物に接近する
        }else if(challenge_stepNo == 10 && check_sonar(0,30) && own_abs(curDegree - prevDegree) > 60){
            printf(",４つ目の障害物に接近する\n");
            stateMachine->sendTrigger(EVT_obstcl_infront);
        // ４つ目の障害物に接近したら向きを変える
        }else if (challenge_stepNo == 11 && check_sonar(0,5)){
            printf(",４つ目の障害物に接近したら向きを変える\n");
            stateMachine->sendTrigger(EVT_obstcl_reached);
            prevDegree = curDegree;
        // 視界が晴れたら左上に前進する
        }else if (challenge_stepNo == 12 && check_sonar(21,255) && own_abs(curDegree - prevDegree) > 70){
            printf(",視界が晴れたら左上に前進する\n");
            stateMachine->sendTrigger(EVT_obstcl_avoidable);
            prevRgbSum = curRgbSum;
            line_over_flg = false;
        // 黒ラインを２つ目まで前進し、２つ目に載ったら向きを調整する
        }else if (challenge_stepNo == 13){
            if (!line_over_flg){
                if (curRgbSum < 100) {
                    prevRgbSum = curRgbSum;
                }
                if(prevRgbSum < 100 && curRgbSum > 150){
                    line_over_flg = true;
                }
            }else if (curRgbSum < 60 && line_over_flg) {
                printf(",黒ラインを２つ目まで前進し、２つ目に載ったら向きを調整する\n");
                stateMachine->sendTrigger(EVT_obstcl_angle);
            }
            prevDegree = curDegree;
        // 直進しスラロームを降りる
        }else if(challenge_stepNo == 14 && own_abs(curDegree - prevDegree) > 70 ){
            printf(",直進しスラロームを降りる\n");
            stateMachine->sendTrigger(EVT_obstcl_avoidable);
        }

        //printf("g_angle=%d\n");
         if(g_angle > 10 && challenge_stepNo > 2){
            printf("スラロームオフ\n");
            slalom_flg = false;
        }
    }
// sano ended


    /*
    int32_t iDeltaD  = (int32_t)(1000.0 * deltaDist );
    int32_t iDeltaDL = (int32_t)(1000.0 * deltaDistL);
    int32_t iDeltaDR = (int32_t)(1000.0 * deltaDistR);
    if (iDeltaDL != 0 && iDeltaDR != 0) {
        _debug(syslog(LOG_NOTICE, "%08u, %06d, %06d, %06d, %06d", clock->now(), getDistance(), iDeltaD, iDeltaDL, iDeltaDR));
    }
    */

    /*
    // display trace message in every PERIOD_TRACE_MSG ms
    if (++traceCnt * PERIOD_OBS_TSK >= PERIOD_TRACE_MSG) {
        traceCnt = 0;
        _debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): distance = %d, azimuth = %d, x = %d, y = %d", clock->now(), getDistance(), getAzimuth(), getLocX(), getLocY()));
        _debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): hsv = (%03u, %03u, %03u)", clock->now(), g_hsv.h, g_hsv.s, g_hsv.v));
        _debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): rgb = (%03u, %03u, %03u)", clock->now(), g_rgb.r, g_rgb.g, g_rgb.b));
        _debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): angle = %d, anglerVelocity = %d", clock->now(), g_angle, g_anglerVelocity));
    }
    */
}

void Observer::deactivate() {
    // deregister cyclic handler from EV3RT
    stp_cyc(CYC_OBS_TSK);
    //clock->sleep() seems to be still taking milisec parm
    clock->sleep(PERIOD_OBS_TSK/2/1000); // wait a while
    _debug(syslog(LOG_NOTICE, "%08u, Observer handler unset", clock->now()));
}

bool Observer::check_touch(void) {
    if (touchSensor->isPressed()) {
        return true;
    } else {
        return false;
    }
}

bool Observer::check_sonar(void) {
    int32_t distance = sonarSensor->getDistance();
    if ((distance <= SONAR_ALERT_DISTANCE) && (distance >= 0)) {
        return true; // obstacle detected - alert
    } else {
        return false; // no problem
    }
}

bool Observer::check_sonar(int16_t sonar_alert_dist_from, int16_t sonar_alert_dist_to) {
    //int32_t distance = sonarSensor->getDistance(); //sano_t 
    int16_t distance = sonarSensor->getDistance(); //sano_t 
    printf("チェックソナーの中=%d\n",distance);
    //printf(",distance2=%d, sonar_alert_dist_from=%d, sonar_alert_dist_to=%d\n",distance, sonar_alert_dist_from, sonar_alert_dist_to );
    if (distance >= sonar_alert_dist_from && distance <= sonar_alert_dist_to) {
        return true; // obstacle detected - alert
    } else {
        return false; // no problem
    }
}

bool Observer::check_backButton(void) {
    if (ev3_button_is_pressed(BACK_BUTTON)) {
        return true;
    } else {
        return false;
    }
}

bool Observer::check_lost(void) {
    if (g_grayScale > GS_LOST) {
        return true;
    } else {
        return false;
    }
    /*
    int8_t otRes_r, otRes_g, otRes_b;
    otRes_r = ot_r->test(cur_rgb.r);
    otRes_g = ot_g->test(cur_rgb.g);
    otRes_b = ot_b->test(cur_rgb.b);
    if ((otRes_r == POS_OUTLIER && otRes_g == POS_OUTLIER) ||
        (otRes_g == POS_OUTLIER && otRes_b == POS_OUTLIER) ||
        (otRes_b == POS_OUTLIER && otRes_r == POS_OUTLIER)) {
        return true;
    } else {
        return false;
    }
    */
}

bool Observer::check_tilt(void) {
    int16_t anglerVelocity = gyroSensor->getAnglerVelocity();
    if (anglerVelocity < ANG_V_TILT && anglerVelocity > (-1) * ANG_V_TILT) {
        return false;
    } else {
        _debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): TILT anglerVelocity = %d", clock->now(), anglerVelocity));
        return true;
    }
}

void Observer::freeze() {
    frozen = true;
}

void Observer::unfreeze() {
    frozen = false;
}

Observer::~Observer() {
    _debug(syslog(LOG_NOTICE, "%08u, Observer destructor", clock->now()));
}