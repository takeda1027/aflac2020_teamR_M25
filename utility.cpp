//
//  utility.cpp
//  aflac2019
//
//  Created by Wataru Taniguchi on 2019/07/05.
//  Copyright © 2019 Ahiruchan Koubou. All rights reserved.
//

#include "app.h"
#include "utility.hpp"

void rgb_to_hsv(rgb_raw_t rgb, hsv_raw_t& hsv) {
    uint16_t max, min;
    double cr, cg, cb, h;  // must be double

    max = rgb.r;
    if(max < rgb.g) max = rgb.g;
    if(max < rgb.b) max = rgb.b;
    
    min = rgb.r;
    if(min > rgb.g) min = rgb.g;
    if(min > rgb.b) min = rgb.b;
    
    hsv.v = 100 * max / (double)255.0;
    
    if (!max) {
        hsv.s = 0;
        hsv.h = 0;
    } else {
        hsv.s = 100 * (max - min) / (double)max;
        cr = (max - rgb.r) / (double)(max - min);
        cg = (max - rgb.g) / (double)(max - min);
        cb = (max - rgb.b) / (double)(max - min);
        
        if (max == rgb.r) {
            h = cb - cg;
        } else if (max == rgb.g) {
            h = 2 + cr - cb;
        } else {
            h = 4 + cg - cr;
        }
        h *= 60;
        if (h < 0) h += 360;
        hsv.h = h;
    }
}

PIDcalculator::PIDcalculator(double p, double i, double d, int16_t t, int16_t min, int16_t max) {
    kp = p;
    ki = i;
    kd = d;
    diff[1] = INT16_MAX; // initialize diff[1]
    deltaT = t;
    minimum = min;
    maximum = max;
    traceCnt = 0;
}

int16_t PIDcalculator::math_limit(int16_t input, int16_t min, int16_t max) {
    if (input < min) {
        return min;
    } else if (input > max) {
        return max;
    }
    return input;
}

int16_t PIDcalculator::compute(int16_t sensor, int16_t target) {
    double p, i, d;
    
    if ( diff[1] == INT16_MAX ) {
	    diff[0] = diff[1] = sensor - target;
    } else {
        diff[0] = diff[1];
        diff[1] = sensor - target;
    }
    integral += (double)(diff[0] + diff[1]) / 2.0 * deltaT / 1000.0;
    
    p = kp * diff[1];
    i = ki * integral;
    d = kd * (diff[1] - diff[0]) * 1000.0 / deltaT;
    /*
    if (++traceCnt * PERIOD_NAV_TSK >= PERIOD_TRACE_MSG) {
        traceCnt = 0;
        char buf[128];
        snprintf(buf, sizeof(buf), "p = %lf, i = %lf, d = %lf", p, i, d);
        _debug(syslog(LOG_NOTICE, "%08u, PIDcalculator::compute(): sensor = %d, target = %d, d0 = %d, d1 = %d +", 0, sensor, target, diff[0], diff[1]));
        _debug(syslog(LOG_NOTICE, "%08u, PIDcalculator::compute(): sensor = %d, target = %d, %s", 0, sensor, target, buf));
    }
    */
    return math_limit(p + i + d, minimum, maximum);
}

OutlierTester::OutlierTester(uint32_t skipCount, uint32_t initCount) {
    cnt = 0L;
    n   = 0L;
    sumSQ = 0.0;
    sum   = 0.0;
    skipCnt = skipCount;
    initCnt = initCount;
    _debug(syslog(LOG_NOTICE, "%08u, OutlierTester::OutlierTester(): skipCnt = %lu, initCnt = %lu", 0, skipCnt, initCnt));
}

int8_t OutlierTester::test(double sample) { // sample is an outlier when true is returned
    if (++cnt <= skipCnt) { // skip initial samples
        return NOT_OUTLIER;
    } else if (cnt <= initCnt) { // do not test until variance gets stable enough
        n++;
        sumSQ += (sample * sample);
        sum   += sample;
        return NOT_OUTLIER;
    }
    double average  = sum / n;
    double variance = (sumSQ / n) - (average * average);
    double diff     = sample - average;
    double diffSQ   = diff * diff;
    //cout << " n=" << n << " a=" << average << " v=" << variance << " ds=" << deltaSQ;
    if ( diffSQ > 2 * 2 * variance ) { // diff > 2 * Sigma then outlier
        // sample is an outlier
        if (diff >= 0) {
            return POS_OUTLIER;
        } else {
            return NEG_OUTLIER;
        }
    } else {
        n++;
        sumSQ += (sample * sample);
        sum   += sample;
        return NOT_OUTLIER; // sample is NOT an outlier
    }
}

int own_abs(int num){
    return (num > 0) ? num : -num;
}