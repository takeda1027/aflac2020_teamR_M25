//
//  LineTracer.hpp
//  aflac2020
//
//  Copyright © 2020 Ahiruchan Koubou. All rights reserved.
//

#ifndef LineTracer_hpp
#define LineTracer_hpp

#include "aflac_common.hpp"
#include "Navigator.hpp"

class LineTracer : public Navigator {
private:
    int32_t motor_ang_l, motor_ang_r;
protected:
    bool    frozen;
public:
    LineTracer();
    LineTracer(Motor* lm, Motor* rm, Motor* tm);
    void haveControl();
    void operate(); // method to invoke from the cyclic handler
    void setSpeed(int8_t s);
    void freeze();
    void unfreeze();
    ~LineTracer();
};

#endif /* LineTracer_hpp */