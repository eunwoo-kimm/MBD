/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: AutonomousSteeringSystem_private.h
 *
 * Code generated for Simulink model 'AutonomousSteeringSystem'.
 *
 * Model version                  : 1.175
 * Simulink Coder version         : 9.6 (R2021b) 14-May-2021
 * C/C++ source code generated on : Wed Jan 31 16:20:26 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_AutonomousSteeringSystem_private_h_
#define RTW_HEADER_AutonomousSteeringSystem_private_h_
#include "rtwtypes.h"

/* Private macros used by the generated code to access rtModel */
#ifndef rtmIsMajorTimeStep
#define rtmIsMajorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
#define rtmIsMinorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmSetTPtr
#define rtmSetTPtr(rtm, val)           ((rtm)->Timing.t = (val))
#endif

/* Used by FromWorkspace Block: '<S3>/From Workspace2' */
#ifndef rtInterpolate
# define rtInterpolate(v1,v2,f1,f2)    (((v1)==(v2))?((double)(v1)): (((f1)*((double)(v1)))+((f2)*((double)(v2)))))
#endif

#ifndef rtRound
# define rtRound(v)                    ( ((v) >= 0) ? floor((v) + 0.5) : ceil((v) - 0.5) )
#endif

extern real_T rt_hypotd_snf(real_T u0, real_T u1);
extern real_T rt_roundd_snf(real_T u);

/* private model entry point functions */
extern void AutonomousSteeringSystem_derivatives(void);

#endif                      /* RTW_HEADER_AutonomousSteeringSystem_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
