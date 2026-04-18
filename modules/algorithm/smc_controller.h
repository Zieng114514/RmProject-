/**
 ******************************************************************************
 * @file    smc_controller.h
 * @brief   Sliding mode controller declaration
 ******************************************************************************
 */
#ifndef _SMC_CONTROLLER_H
#define _SMC_CONTROLLER_H

#include "main.h"
#include "stdint.h"
#include <math.h>

#ifndef abs
#define abs(x) ((x > 0) ? x : -x)
#endif

typedef struct
{
    float C;
    float K;
    float Ref;
    float ErrorEps;
    float MaxOut;
    float J;
    float Epsilon;
} SMC_Init_Config_s;

typedef struct
{
    /* init config block */
    float C;
    float K;
    float Ref;
    float ErrorEps;
    float MaxOut;
    float J;
    float Epsilon;

    /* runtime state */
    float Angle;
    float AngleVel;
    float Output;

    float Error;
    float ErrorLast;
    float DRef;
    float DDRef;
    float RefLast;
    float Surface;
} SMCInstance;

void SMCInit(SMCInstance *smc, SMC_Init_Config_s *config);
float SMCCalculate(SMCInstance *smc, float angle_now, float angle_vel, float ref);

#endif /* _SMC_CONTROLLER_H */

