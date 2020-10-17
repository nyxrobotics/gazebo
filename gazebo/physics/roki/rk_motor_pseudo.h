/*
  rk_motor_pseudo.h
  see also : https://github.com/s-nakaoka/choreonoid/blob/master/src/RokiPlugin/RokiSimulatorItem.cpp
*/
#ifndef __RK_MOTOR_PSEUDO__
#define __RK_MOTOR_PSEUDO__

#include <roki/rk_motor.h>

extern "C" {

typedef struct {
  double t;
} rkMotorPrpPseudo;

rkMotor *rkMotorCreatePseudo(rkMotor *m);
}

#endif
