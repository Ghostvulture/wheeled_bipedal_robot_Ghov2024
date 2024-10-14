#pragma once
#ifndef BANANA_MSGS_H
#define BANANA_MSGS_H
#ifdef __cplusplus
extern "C" {
#endif

struct Msg_Odometer_t {
    float x;
    float v;
    float a_z;
};



#ifdef __cplusplus
}
#endif
#endif