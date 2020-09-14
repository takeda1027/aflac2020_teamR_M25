#include <cstdio>
#include <cstdarg>
#include <cstring>

#include "Logger.hpp"
#include "ev3api.h"

using namespace std;

FILE *Logger::fp_bt;
FILE *Logger::fp_sd;
char *Logger::buffer;
Log *Logger::log;
int Logger::cur_w_line;
int Logger::cur_r_line;

#define MAX_LOG_LINE (10000)
#define LOG_MSG_LEN (100)

void Logger::init() {

	// bluetooth
	if ( ev3_bluetooth_is_connected() ) {
		fp_bt = ev3_serial_open_file(EV3_SERIAL_BT);
		fprintf(fp_bt, "bluetooth is connected!\r\n");
	} else {
		// bluetoothが接続されていない場合
		fp_bt = fopen("/ev3rt/res/bt.log", "w");
		fprintf(fp_bt, "bluetooth isn't connected...\r\n");
	}

	// SDカード
	fp_sd = fopen("/ev3rt/res/sd.log", "w");

	cur_w_line = 0;
	cur_r_line = 0;
	log = new Log[MAX_LOG_LINE];
	buffer = new char[LOG_MSG_LEN];

}

void Logger::exit() {

	for (unsigned int i = 0; i < MAX_LOG_LINE; i++) {
		dump();
	}

	fclose(fp_bt);
	fclose(fp_sd);
}

void Logger::dprint(char *form, ...) {
	va_list arg;
	va_start(arg, form);
	vfprintf(fp_bt, form, arg);
	va_end(arg);

}

void Logger::dprintf(char *form, ...) {
	memset(buffer,0 ,100);
	va_list arg;
	va_start(arg, form);
	vsprintf(buffer, form, arg);
	va_end(arg);

	cur_w_line = cur_w_line % MAX_LOG_LINE;
	if ( log[cur_w_line].writable == true ) {
		log[cur_w_line].msg = buffer;
		log[cur_w_line].writable = false;
//		print_new_line("cur_buf_w_line = %d", cur_w_line);
		cur_w_line++;
	}
}

void Logger::dump() {

	cur_r_line = cur_r_line % MAX_LOG_LINE;
	if  ( log[cur_r_line].writable == false ) {
		fprintf(fp_sd, "%d, %s", cur_r_line, log[cur_r_line].msg.c_str());
		log[cur_r_line].writable = true;
		cur_r_line++;
	}
}

void Logger::flush() {

	fflush(fp_bt);
}
