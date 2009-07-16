#ifndef Header_RILSetup
#define Header_RILSetup

#include "math.h"

// Size
#define MAXROBOT 8

// RIL robot id alias -- ids are same in SwisTracker
#define EPUCK1246 "1"
#define EPUCK1250 "2"
#define EPUCK1253 "3"
#define EPUCK1259 "4"
#define EPUCK1260 "5"
#define EPUCK1265 "6"
#define EPUCK1271 "7"
#define EPUCK1302 "8"

//RIL and AFM params
#define MAXSHOPTASK 3
#define INIT_URGENCY 0.5
#define DELTA_URGENCY 0.01
#define INIT_MATERIAL_COUNT 10
#define XY 2 // for task coordinates
#define DELTA_DISTANCE 0.001

//robot device's instrinsics
#define INIT_SENSITIZATION 1
#define INIT_LEARN_RATE 1
#define INIT_FORGET_RATE 0.16

// for pose nomalization
#define MAX_X 3600
#define MAX_Y 3248
#define MAX_THETA (2* M_PI)

#endif
