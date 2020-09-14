//
//  BlindRunner.cpp
//  aflac2020
//
//  Copyright © 2020 Ahiruchan Koubou. All rights reserved.
//

#include "app.h"
#include "BlindRunner.hpp"
#include <string.h>
#include <stdlib.h>

BlindRunner::BlindRunner(Motor* lm, Motor* rm, Motor* tm) : LineTracer(lm, rm, tm) {
#if defined(MAKE_SIM)
    _debug(syslog(LOG_NOTICE, "%08lu, BlindRunner attempts to read profile in simulator environment", clock->now()));
	readPropFile("/usr/local/etc/wgetrc");
#else
	readPropFile("/ev3rt/res/BlindRunner_prop.txt");
#endif
	leftMotor = lm;
	rightMotor = rm;
	tailMotor = tm;
	
	s_trace_counter = 0;

    _debug(syslog(LOG_NOTICE, "%08lu, BlindRunner constructor", clock->now()));
}

void BlindRunner::haveControl() {
    activeNavigator = this;

    // private変数の初期化

    // ログ出力
    syslog(LOG_NOTICE, "%08lu, BlindRunner has control", clock->now());
}

void BlindRunner::operate() {
	LineTracer::operate();
	/*
	// 開始ログ出力
	if (++s_trace_counter * PERIOD_NAV_TSK >= PERIOD_TRACE_MSG ) {
		s_trace_counter = 0;
		_debug(syslog(LOG_NOTICE, "%08u, ■■■BlindRunner::operate():開始", clock->now()));
	}

	// 終了ログ出力
	if (++s_trace_counter * PERIOD_NAV_TSK >= PERIOD_TRACE_MSG ) {
		s_trace_counter = 0;
		_debug(syslog(LOG_NOTICE, "%08u, ■■■BlindRunner::operate():終了", clock->now()));
	}
	*/
}

int BlindRunner::readLine( FILE* file, char* dst, size_t len ){
    int c = 0;
    unsigned int i = 0;

    while ((c = fgetc(file)) != EOF) {
      if (c < 0) {
        return c;        // error handling
      }
      if ( i + 1 == len ) {  // reached end of buffer
        dst[i] = '\0';
        break;
      }
      if (c == '\n') {   // reached end of line
        dst[i] = '\0';
        break;
      }
      dst[i] = (char)c;
      i++;
    }
    return i;
}

void BlindRunner::readPropFile( const char* filename ){
	FILE* prop_file = NULL;
	prop_file = fopen( filename, "r" );
	if( prop_file==NULL){
		_debug(syslog(LOG_NOTICE, "%08lu, BlindRunner readProfile() file not found", clock->now()));
		return;
	}

	memset( &props, 0, sizeof(struct property) * NUM_PROPS );

	int retval = 0;
	int i = 0;
	char buf[256];
	memset( &buf, 0, sizeof(buf) );

	while( ( retval = readLine(prop_file, buf, 256)) > 0 ){
		char* comptr = strstr( buf, "," );
		if( i < NUM_PROPS && NULL != comptr ){
			*comptr = '\0';
			strcpy( props[i].name, buf );
			props[i].value = atoi(comptr+1);
			i++;
		}else
			break;
	}
	fclose( prop_file );
}

int BlindRunner::getProp( const char* propname ){
	if( props == NULL ){
		return 0;
	}

	for ( int i = 0; i < NUM_PROPS; i++ ){
		if( strcmp( propname, props[i].name ) == 0 ){
			return props[i].value;
		}
	}
	return 0;
}

BlindRunner::~BlindRunner() {
    _debug(syslog(LOG_NOTICE, "%08lu, BlindRunner destructor", clock->now()));
}