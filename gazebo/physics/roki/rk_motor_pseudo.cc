/*
  rk_motor_pseudo.c
  see also : https://github.com/s-nakaoka/choreonoid/blob/master/src/RokiPlugin/RokiSimulatorItem.cpp
*/

#include "rk_motor_pseudo.h"
#define _rkc(p) ((rkMotorPrpPseudo *)p)

extern "C" {

static void _rkMotorSetInputPseudo(void *prp, double *val);
static void _rkMotorInertiaPseudo(void *prp, double *val);
static void _rkMotorInputTrqPseudo(void *prp, double *val);
static void _rkMotorRegistancePseudo(void *prp, double *dis, double *vel, double *val);
static void _rkMotorDrivingTrqPseudo(void *prp, double *dis, double *vel, double *acc, double *val);
static void _rkMotorStateCopyPseudo(void *src, void *dst);
static bool _rkMotorQueryFReadPseudo(FILE *fp, char *key, void *prp);
static void _rkMotorFWritePseudo(FILE *fp, void *prp);

void _rkMotorSetInputPseudo(void *prp, double *val){
  _rkc(prp)->t = *val;
}

void _rkMotorInertiaPseudo(void */*prp*/, double *val){
  *val =  0.0;
}

void _rkMotorInputTrqPseudo(void *prp, double *val){
  *val = _rkc(prp)->t;
}

void _rkMotorRegistancePseudo(void */*prp*/, double */*dis*/, double */*vel*/, double *val){
  *val = 0.0;
}

void _rkMotorDrivingTrqPseudo(void *prp, double */*dis*/, double */*vel*/, double */*acc*/, double *val){
  _rkMotorInputTrqPseudo( prp, val);
}

void _rkMotorStateCopyPseudo(void *src, void *dst){
  memcpy(dst, src, sizeof(rkMotorPrpPseudo));
}

bool _rkMotorQueryFReadPseudo(FILE */*fp*/, char */*key*/, void */*prp*/)
{
  return true;
}

void _rkMotorFWritePseudo(FILE */*fp*/, void */*prp*/)
{
}

static rkMotorCom rk_motor_pseudo = {
  1,
  _rkMotorSetInputPseudo,
  _rkMotorInertiaPseudo,
  _rkMotorInputTrqPseudo,
  _rkMotorRegistancePseudo,
  _rkMotorDrivingTrqPseudo,
  _rkMotorStateCopyPseudo,
  _rkMotorQueryFReadPseudo,
  _rkMotorFWritePseudo,
};

void _rkMotorInitPrpPseudo(void *prp)
{
  _rkc(prp)->t = 0;
}

rkMotor *rkMotorCreatePseudo(rkMotor *m)
{
  m->prp = zAlloc(rkMotorPrpPseudo, 1);
  _rkMotorInitPrpPseudo(m->prp);
  m->com = &rk_motor_pseudo;
  return m;
}

}
