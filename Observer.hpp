//
//  Observer.hpp
//  aflac2020
//
//  Copyright © 2019 Ahiruchan Koubou. All rights reserved.
//

#ifndef Observer_hpp
#define Observer_hpp

#include "aflac_common.hpp"
#include "utility.hpp"

#define OLT_SKIP_PERIOD    1000 * 1000 // period to skip outlier test in miliseconds
#define OLT_INIT_PERIOD    3000 * 1000 // period before starting outlier test in miliseconds

// FIR filter parameters
const int FIR_ORDER = 10;
//const double hn[FIR_ORDER+1] = { 2.993565708123639e-03, 9.143668394023662e-03, -3.564197579813870e-02, -3.996625085414179e-02, 2.852028479250662e-01, 5.600000000000001e-01, 2.852028479250662e-01, -3.996625085414179e-02, -3.564197579813870e-02, 9.143668394023662e-03, 2.993565708123639e-03 };
const double hn[FIR_ORDER+1] = { -1.247414986406201e-18, -1.270350182429102e-02, -2.481243022283666e-02, 6.381419731491805e-02, 2.761351394755998e-01, 4.000000000000000e-01, 2.761351394755998e-01, 6.381419731491805e-02, -2.481243022283666e-02, -1.270350182429102e-02, -1.247414986406201e-18 };

// moving average parameter
const int MA_CAP = 10;

// Calculating average difference of Right/Left motor at initial straight parameters
const int AVERAGE_START =  350;
const int AVERAGE_END   = 1500;

class Observer {
private:
    Motor*          leftMotor;
    Motor*          rightMotor;
    Motor*          armMotor;
    Motor*          tailMotor;
    TouchSensor*    touchSensor;
    SonarSensor*    sonarSensor;
    GyroSensor*     gyroSensor;
    ColorSensor*    colorSensor;
    double distance, azimuth, locX, locY, aveDiffAng, deltaDiff, prevDeltaDiff, prevDis, prevDisX, prevDisY;
    double integD, integDL, integDR;
    int8_t process_count,roots_no;
    int16_t traceCnt, prevGS, curRgbSum, prevRgbSum, curAngle, prevAngle, curDegree180, prevDegree180,curDegree360, prevDegree360,cntDegree,turnDegree;
    int32_t prevAngL, prevAngR, notifyDistance, gsDiff, timeDiff, sonarDistance, diffAng, sumDiffAng, countAng;
    uint64_t curTime, prevTime;
    bool touch_flag, sonar_flag, backButton_flag, lost_flag, frozen, blue_flag, blue2_flg, slalom_flg, line_over_flg, move_back_flg,garage_flg;

    rgb_raw_t cur_rgb;
    hsv_raw_t cur_hsv;
    FIR_Transposed<FIR_ORDER> *fir_r, *fir_g, *fir_b;
    MovingAverage<int32_t, MA_CAP> *ma;
    //OutlierTester*  ot_r;
    //OutlierTester*  ot_g;
    //OutlierTester*  ot_b;

    bool check_touch(void);
    bool check_sonar(void);
    bool check_sonar(int16_t sonar_alert_dist_from, int16_t sonar_alert_dist_to);
    bool check_backButton(void);
    bool check_lost(void);
    bool check_tilt(void);
    int16_t getTurnDgree(int16_t prev_x,int16_t x);
    
protected:
public:
    Observer();
    Observer(Motor* lm, Motor* rm, Motor* am, Motor* tm, TouchSensor* ts, SonarSensor* ss, GyroSensor* gs, ColorSensor* cs);
    void activate();
    void reset();
    void notifyOfDistance(int32_t delta);
    int32_t getDistance();
    int32_t getSonarDistance();
    int16_t getAzimuth();
    int16_t getDegree();
    int32_t getLocX();
    int32_t getLocY();
    void operate(); // method to invoke from the cyclic handler
    void deactivate();
    void freeze();
    void unfreeze();
    ~Observer();
};

extern Observer*    observer;

#endif /* Observer_hpp */