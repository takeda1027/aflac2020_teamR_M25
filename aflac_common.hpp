//
//  aflac_common.hpp
//  aflac2020
//
//  Copyright © 2019 Ahiruchan Koubou. All rights reserved.
//

#ifndef aflac_common_hpp
#define aflac_common_hpp

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

/**
 * 左コース/右コース向けの設定を定義します
 * デフォルトは左コース(ラインの右エッジをトレース)です
 */
#if defined(MAKE_RIGHT)
    static const int _LEFT = 0;
    static const int _EDGE = -1;
#else
    static const int _LEFT = 1;
    static const int _EDGE = 1;
#endif

//#include <cinttypes>
#include <cmath>
#include "TouchSensor.h"
#include "SonarSensor.h"
#include "ColorSensor.h"
#include "GyroSensor.h"
#include "Motor.h"
#include "Steering.h"
#include "Clock.h"
using namespace ev3api;

/* 下記のマクロは個体/環境に合わせて変更する必要があります */
#define GYRO_OFFSET           0  /* ジャイロセンサオフセット値(角速度0[deg/sec]時) */
#define LIGHT_WHITE          60  /* 白色の光センサ値 */
#define LIGHT_BLACK           3  /* 黒色の光センサ値 */
#define GS_LOST              90  // threshold to determine "line lost"
#define FINAL_APPROACH_LEN  100  // final approch length in milimater
#define ANG_V_TILT           50  // threshold to determine "tilt"
#define SONAR_ALERT_DISTANCE 10  /* 超音波センサによる障害物検知距離[cm] */
#define TAIL_ANGLE_STAND_UP  85  /* 完全停止時の角度[度] */
#define TAIL_ANGLE_DRIVE      3  /* バランス走行時の角度[度] */
#define P_GAIN             2.5F  /* 完全停止用モータ制御比例係数 */
#define PWM_ABS_MAX          60  /* 完全停止用モータ制御PWM絶対最大値 */

#define TIRE_DIAMETER    100.0F  // diameter of tire in milimater
#define WHEEL_TREAD      150.0F  // distance between the right and left wheels
//#define P_CONST            0.4D  // PID constants determined by Ultimate Gain method
//#define I_CONST            0.0D
//#define D_CONST            0.0D


// #define P_CONST           0.46D  // PID constants determined by Ultimate Gain method
// #define I_CONST     0.00000013D sano_t
// #define D_CONST          0.075D //sano_t
#define P_CONST           0.85D
#define I_CONST            0.0000001D
#define D_CONST            0.5D
//#define SPEED_NORM           50
#define GS_TARGET            47


#define SPEED_NORM           50
#define SPEED_SLOW           30
#define SPEED_RECOVER        10
#define SPEED_BLIND          75
#define TURN_MIN            -16  // minimum value PID calculator returns
#define TURN_MAX             16  // maximum value PID calculator returns


//#define GS_TARGET            45 //sano_t

#define M_2PI    (2.0 * M_PI)

//#define DEVICE_NAME     "ET0"  /* Bluetooth名 hrp2/target/ev3.h BLUETOOTH_LOCAL_NAMEで設定 */
//#define PASS_KEY        "1234" /* パスキー    hrp2/target/ev3.h BLUETOOTH_PIN_CODEで設定 */
#define CMD_START_R     'R' // R-mode start command
#define CMD_START_r     'r' // R-mode start command
#define CMD_START_L     'L' // L-mode start command
#define CMD_START_l     'l' // L-mode start command
#define CMD_STOP_S      'S'
#define CMD_STOP_s      's'

// machine state
#define ST_start        0
#define ST_tracing      1
#define ST_blind        2
#define ST_stopping     3
#define ST_end          4
#define ST_slalom 5
#define ST_block  6

#define ST_NAME_LEN     20  // maximum number of characters for a machine state name
const char stateName[][ST_NAME_LEN] = {
    "ST_start",
    "ST_tracing",
    "ST_blind",
    "ST_stopping",
    "ST_end",
    "ST_slalom",
    "ST_block"
};

// event
#define EVT_cmdStart_L      0
#define EVT_cmdStart_R      1
#define EVT_touch_On        2
#define EVT_touch_Off       3
#define EVT_sonar_On        4
#define EVT_sonar_Off       5
#define EVT_backButton_On   6
#define EVT_backButton_Off  7
#define EVT_bk2bl           8
#define EVT_bl2bk           9
#define EVT_cmdStop         10
#define EVT_line_lost       11
#define EVT_line_found      12
#define EVT_dist_reached    13
#define EVT_tilt            14

#define EVT_slalom_reached  15
#define EVT_slalom_challenge    16
#define EVT_block_challenge 17
#define EVT_block_area_in   18
#define EVT_line_on_pid_cntl    19
#define EVT_line_on_p_cntl  20
#define EVT_NAME_LEN        21  // maximum number of characters for an event name
const char eventName[][EVT_NAME_LEN] = {
    "EVT_cmdStart_L",
    "EVT_cmdStart_R",
    "EVT_touch_On",
    "EVT_touch_Off",
    "EVT_sonar_On",
    "EVT_sonar_Off",
    "EVT_backButton_On",
    "EVT_backButton_Off",
    "EVT_bk2bl",
    "EVT_bl2bk",
    "EVT_cmdStop",
    "EVT_line_lost",
    "EVT_line_found",
    "EVT_dist_reached",
    "EVT_tilt",
    "EVT_slalom_reached",
    "EVT_slalom_challenge",
    "EVT_block_challenge",
    "EVT_block_area_in",
    "EVT_line_on_pid_cntl",
    "EVT_line_on_p_cntl"
};

typedef struct {
    uint16_t h; // Hue
    uint16_t s; // Saturation
    uint16_t v; // Value of brightness  
} hsv_raw_t;

// pwmSetMode
#define Mode_speed_constant     1
#define Mode_speed_increaseL    2
#define Mode_speed_decreaseL    3
#define Mode_speed_increaseR    4
#define Mode_speed_decreaseR    5
#define Mode_speed_increaseLR   6
#define Mode_speed_decreaseLR   7
#define Mode_speed_incrsLdcrsR  8
#define Mode_speed_incrsRdcrsL  9

// global variables
extern rgb_raw_t g_rgb;
extern hsv_raw_t g_hsv;
extern int16_t g_grayScale, g_grayScaleBlueless;
extern int16_t g_angle, g_anglerVelocity;
extern int16_t g_challenge_stepNo,g_color_brightness; //sano

extern Clock*       clock;
extern uint8_t      state;

#endif /* aflac_common_hpp */