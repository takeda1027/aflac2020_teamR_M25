//
//  utility.hpp
//  aflac2019
//
//  Created by Wataru Taniguchi on 2019/07/05.
//  Copyright © 2019 Ahiruchan Koubou. All rights reserved.
//

#ifndef utility_hpp
#define utility_hpp

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

//#include <cinttypes>
#include "ColorSensor.h"
using namespace ev3api;

#define NOT_OUTLIER          0
#define POS_OUTLIER          1
#define NEG_OUTLIER          2

typedef struct {
    uint16_t h; // Hue
    uint16_t s; // Saturation
    uint16_t v; // Value of brightness
    
} hsv_raw_t;

template<typename T, int CAPACITY> class MovingAverage {
private:
    T elements[CAPACITY];
    int index;
    bool filled;
    T sum;
public:
    MovingAverage();
    void clear();
    T add(T element);
    T get();
};

template<typename T, int CAPACITY>
MovingAverage<T, CAPACITY>::MovingAverage() {
    assert (CAPACITY > 0);
    clear();
}

template<typename T, int CAPACITY>
void MovingAverage<T, CAPACITY>::clear() {
    index = 0;
    sum = 0;
    filled = false;
}

template<typename T, int CAPACITY>
T MovingAverage<T, CAPACITY>::add(T element) {
    if (index == CAPACITY) {
        index = 0;
        filled = true;
    }
    if (filled) {
        sum = sum - elements[index] + element;
        elements[index++] = element;
        return sum / CAPACITY;
    } else {
        sum = sum + element;
        elements[index++] = element;
        return sum / index;
    }
}

template<typename T, int CAPACITY>
T MovingAverage<T, CAPACITY>::get() {
    if (filled) {
        return sum / CAPACITY;
    } else {
        if (index == 0) {
            return 0;
        } else {
            return sum / index;
        }
    }
}

template<int ORDER> class FIR_Direct {
private:
    const double *const hm;
    double un[ORDER+1];
public:
    FIR_Direct(const double hk[]);
    inline double Execute(const double xin);
};

template<int ORDER>
FIR_Direct<ORDER>::FIR_Direct(const double hk[]) : hm(hk) {
    for (int i = 0; i <= ORDER; i++) un[i] = 0.0;
}

template<int ORDER>
inline double FIR_Direct<ORDER>::Execute(const double xin) {
    double acc = 0.0;
    un[0] = xin;
    for (int i = 0; i <= ORDER; i++) acc = acc + hm[i] * un[i];
    for (int i = ORDER; i > 0; i--) un[i] = un[i-1];
    return acc;
}

template<int ORDER> class FIR_Transposed {
private:
    const double *const hm;
    double un[ORDER+1];
public:
    FIR_Transposed(const double hk[]);
    inline double Execute(const double xin);
};

template<int ORDER>
FIR_Transposed<ORDER>::FIR_Transposed(const double hk[]) : hm(hk) {
    for (int i = 0; i <= ORDER; i++) un[i] = 0.0;
}

template<int ORDER>
inline double FIR_Transposed<ORDER>::Execute(const double xin) {
    for (int i = 0; i < ORDER; i++) un[i] = hm[i] * xin + un[i+1];
    un[ORDER] = hm[ORDER] * xin;
    return un[0];
}

void rgb_to_hsv(rgb_raw_t rgb, hsv_raw_t& hsv);

class PIDcalculator {
private:
    double kp, ki, kd;   /* PID constant */
    int16_t diff[2], deltaT, minimum, maximum, traceCnt;
    double integral;
    int16_t math_limit(int16_t input, int16_t min, int16_t max);
public:
    PIDcalculator(double p, double i, double d, int16_t t, int16_t min, int16_t max);
    int16_t compute(int16_t sensor, int16_t target);
    ~PIDcalculator();
};

class OutlierTester {
private:
    double sum, sumSQ;
    uint32_t cnt, n, skipCnt, initCnt;
public:
    OutlierTester(uint32_t skipCount, uint32_t initCount);
    int8_t test(double sample);
    ~OutlierTester();
};

int own_abs(int num);

#endif /* utility_hpp */
