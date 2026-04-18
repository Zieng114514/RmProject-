/**
 * @file smc_controller.c
 * @brief Sliding mode controller definition
 */
#include "smc_controller.h"
#include <string.h>

static float smc_sat(float y);
static int8_t smc_signal(float y);

void SMCInit(SMCInstance *smc, SMC_Init_Config_s *config)
{
    if (smc == NULL || config == NULL)
    {
        return;
    }

    memset(smc, 0, sizeof(SMCInstance));

    smc->C = config->C;
    smc->K = config->K;
    smc->Ref = config->Ref;
    smc->ErrorEps = config->ErrorEps;
    smc->MaxOut = config->MaxOut;
    smc->J = config->J;
    smc->Epsilon = config->Epsilon;
}

float SMCCalculate(SMCInstance *smc, float angle_now, float angle_vel, float ref)
{
    if (smc == NULL)
    {
        return 0.0f;
    }

    smc->Angle = angle_now;
    smc->AngleVel = angle_vel;

    smc->Ref = ref;

    smc->Error = smc->Angle - smc->Ref;
    smc->DDRef = (smc->Ref - smc->RefLast) - smc->DRef;
    smc->DRef = (smc->Ref - smc->RefLast);

    if (fabsf(smc->Error) < smc->ErrorEps)
    {
        smc->Output = 0.0f;
        smc->Surface = 0.0f;
        smc->ErrorLast = smc->Error;
        smc->RefLast = smc->Ref;
        return smc->Output;
    }

    smc->Surface = smc->C * smc->Error + (smc->AngleVel - smc->DRef);

    smc->Output = smc->J * (smc->DDRef - smc->C * (smc->AngleVel - smc->DRef) - smc->Epsilon * smc_sat(smc->Surface) - smc->K * smc->Surface);

    if (smc->Output > smc->MaxOut)
    {
        smc->Output = smc->MaxOut;
    }
    if (smc->Output < -smc->MaxOut)
    {
        smc->Output = -smc->MaxOut;
    }

    smc->ErrorLast = smc->Error;
    smc->RefLast = smc->Ref;

    return smc->Output;
}

static float smc_sat(float y)
{
    if (fabsf(y) <= 1.0f)
    {
        return y;
    }
    return (float)smc_signal(y);
}

static int8_t smc_signal(float y)
{
    if (y > 0.0f)
    {
        return 1;
    }
    else if (y == 0.0f)
    {
        return 0;
    }
    return -1;
}

