//
//      
//  aflac2020
//
//  Copyright © 2020 Ahiruchan Koubou. All rights reserved.
//

#ifndef ChallengeRunner_hpp
#define ChallengeRunner_hpp

#include "aflac_common.hpp"
#include "LineTracer.hpp"

class ChallengeRunner : public LineTracer {
private:
    int8_t pwmMode;
    int16_t count, procCount, traceCnt;
    Motor* armMotor;
protected:
public:
    ChallengeRunner();
    ChallengeRunner(Motor* lm, Motor* rm, Motor* tm,Motor* am);
    void haveControl();
    void operate(); // method to invoke from the cyclic handler
    void runChallenge();
    void setPwmLR(int p_L,int p_R,int mode, int proc_count);
    void rest(int16_t rest_time);
    int8_t getPwmL();
    int8_t getPwmR();
    ~ChallengeRunner();
};

#endif /* ChallengeRunner_hpp */