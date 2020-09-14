//
//  BlindRunner.hpp
//  aflac2020
//
//  Copyright © 2020 Ahiruchan Koubou. All rights reserved.
//

#ifndef BlindRunner_hpp
#define BlindRunner_hpp

#include "aflac_common.hpp"
#include "LineTracer.hpp"

#define PROP_NAME_LEN	48	// プロパティー名の最大長
#define NUM_PROPS	13	// プロパティーの個数

class BlindRunner : public LineTracer {
private:
	int16_t s_trace_counter;
    Motor*          leftMotor;
    Motor*          rightMotor;
    Motor*			tailMotor;

	struct property{
		char name[PROP_NAME_LEN];
		int value;
	} ;

	struct property props[NUM_PROPS];

	int readLine( FILE* file, char* dst, size_t len );
	void readPropFile( const char* filename );
	int getProp( const char* propname );
protected:
public:
    BlindRunner();
    BlindRunner(Motor* lm, Motor* rm, Motor* tm);
    void haveControl();
    void operate(); // method to invoke from the cyclic handler
    ~BlindRunner();
};

#endif /* BlindRunner_hpp */