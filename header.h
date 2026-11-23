#include <LPC21xx.h>
typedef unsigned int u32;
typedef struct
{
    u32 id;
    u32 rtr;
    u32 dlc;
    u32 byteA;
    u32 byteB;
} CAN2_MSG;
#define TRAIN_CLEAR 0
#define TRAIN_APPROACHING 1
#define TRAIN_ON_CROSSING 2
#define TRAIN_CLEARED 3
#define GATE_OPEN 0
#define GATE_CLOSING 1
#define GATE_CLOSED 2
#define GATE_OPENING 3
#define TRAIN_STATUS_ID 0x100
#define GATE_STATUS_ID 0x200
#define EMERGENCY_ID 0x300

