/*!
 * \file      user_struct.h
 *
 * \brief     App data define.
 *
 * \author    ck ( beelinker )
 */
 
#ifndef __USER_STRUCT_H__
#define __USER_STRUCT_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

#define DEV_UNPRODUCTED     0xFF
#define DEV_PRODUCTED       0x55

#define DEV_ACTIVE_DISABLE  0x0
#define DEV_ACTIVE_ENABLE   0x1
#define DEV_CHARGING        0x2

typedef struct
{
    uint8_t                 producted;   
    uint8_t                 active;
    uint8_t                 stor_flg;
}USER_DATA_t;

#ifdef __cplusplus
}
#endif

#endif // __USER_STRUCT_H
