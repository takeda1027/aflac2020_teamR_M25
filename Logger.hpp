//
//  Logger.hpp
//  aflac2019
//
//  Created by Takahiro Furukawa on 2019/07/29.
//  Copyright © 2019 Ahiruchan Koubou. All rights reserved.
//

#ifndef Logger_hpp
#define Logger_hpp

#include <vector>
#include <string>

using namespace std;

#define DEBUG

#ifdef DEBUG
  #define print_new_line(fmt, ...) { Logger::dprint((char*)"%s %s %d: ", __FILE__, __func__, __LINE__);Logger::dprint((char*)fmt "\r\n",  ##__VA_ARGS__); }
	#define print_new_linef(fmt, ...) { Logger::dprintf((char*)fmt"\r\n",  ##__VA_ARGS__); }
	#define print(fmt, ...) { Logger::dprint((char*)fmt,  ##__VA_ARGS__); }
	#define logger_init Logger::init
	#define logger_exit Logger::exit
	#define logger_dump Logger::dump
	#define logger_flush Logger::flush
#else
	#define print_new_line(...) ((void)0)
	#define print(...) ((void)0)
#endif

class Log {
	public:
		string msg;
		bool writable;
		Log():writable(true) { ; }
};

class Logger {

	public:
		Logger(){;};
		static void init ();
		static void exit ();
		static void dprint(char *form, ...);
		static void dprintf(char *form, ...);
		static void dump();
		static void flush();

	private:
		static FILE* fp_bt;
		static FILE* fp_sd;
		static char* buffer;
		static Log *log;
		static int cur_w_line;
		static int cur_r_line;
};

#endif /* Logger_hpp */
