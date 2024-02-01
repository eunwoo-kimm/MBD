/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: AutonomousSteeringSystem.h
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

#ifndef RTW_HEADER_AutonomousSteeringSystem_h_
#define RTW_HEADER_AutonomousSteeringSystem_h_
#include <math.h>
#include <emmintrin.h>
#include <string.h>
#ifndef AutonomousSteeringSystem_COMMON_INCLUDES_
#define AutonomousSteeringSystem_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                           /* AutonomousSteeringSystem_COMMON_INCLUDES_ */

#include "AutonomousSteeringSystem_types.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

/* Block signals (default storage) */
typedef struct
{
  real_T xk1[5];                       /* '<S24>/optimizer' */
  real_T u;                            /* '<S24>/optimizer' */
  boolean_T iAout[52];                 /* '<S24>/optimizer' */
}
B_AutonomousSteeringSystem_T;

/* Block states (default storage) for system '<Root>' */
typedef struct
{
  real_T last_mv_DSTATE;               /* '<S4>/last_mv' */
  real_T last_x_PreviousInput[5];      /* '<S4>/last_x' */
  struct
  {
    void *TimePtr;
    void *DataPtr;
    void *RSimInfoPtr;
  }
  FromWorkspace2_PWORK;                /* '<S3>/From Workspace2' */

  struct
  {
    void *TimePtr;
    void *DataPtr;
    void *RSimInfoPtr;
  }
  FromWorkspace_PWORK;                 /* '<S3>/From Workspace' */

  struct
  {
    int_T PrevIndex;
  }
  FromWorkspace2_IWORK;                /* '<S3>/From Workspace2' */

  struct
  {
    int_T PrevIndex;
  }
  FromWorkspace_IWORK;                 /* '<S3>/From Workspace' */

  boolean_T Memory_PreviousInput[52];  /* '<S4>/Memory' */
}
DW_AutonomousSteeringSystem_T;

/* Continuous states (default storage) */
typedef struct
{
  real_T Plant_CSTATE[4];              /* '<Root>/Plant' */
}
X_AutonomousSteeringSystem_T;

/* State derivatives (default storage) */
typedef struct
{
  real_T Plant_CSTATE[4];              /* '<Root>/Plant' */
}
XDot_AutonomousSteeringSystem_T;

/* State disabled  */
typedef struct
{
  boolean_T Plant_CSTATE[4];           /* '<Root>/Plant' */
}
XDis_AutonomousSteeringSystem_T;

/* Invariant block signals (default storage) */
typedef struct
{
  const real_T MathFunction[2];        /* '<S4>/Math Function' */
  const real_T MathFunction1;          /* '<S4>/Math Function1' */
  const real_T MathFunction2;          /* '<S4>/Math Function2' */
}
ConstB_AutonomousSteeringSyst_T;

#ifndef ODE3_INTG
#define ODE3_INTG

/* ODE3 Integration Data */
typedef struct
{
  real_T *y;                           /* output */
  real_T *f[3];                        /* derivatives */
}
ODE3_IntgData;

#endif

/* Constant parameters (default storage) */
typedef struct
{
  /* Computed Parameter: Plant_A_pr
   * Referenced by: '<Root>/Plant'
   */
  real_T Plant_A_pr[7];

  /* Computed Parameter: Plant_B_pr
   * Referenced by: '<Root>/Plant'
   */
  real_T Plant_B_pr[2];

  /* Computed Parameter: Plant_A_ir
   * Referenced by: '<Root>/Plant'
   */
  uint32_T Plant_A_ir[7];

  /* Pooled Parameter (Expression: [2*Cf/m 0 2*Cf*lf/Iz 0]';
     )
   * Referenced by: '<Root>/Plant'
   */
  uint32_T pooled10[2];

  /* Computed Parameter: Plant_C_ir
   * Referenced by: '<Root>/Plant'
   */
  uint32_T Plant_C_ir[2];
}
ConstP_AutonomousSteeringSyst_T;

/* Real-time Model Data Structure */
struct tag_RTM_AutonomousSteeringSys_T
{
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_AutonomousSteeringSystem_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  boolean_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[4];
  real_T odeF[3][4];
  ODE3_IntgData intgData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct
  {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  }
  Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct
  {
    uint32_T clockTick0;
    time_T stepSize0;
    uint32_T clockTick1;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  }
  Timing;
};

/* Block signals (default storage) */
extern B_AutonomousSteeringSystem_T AutonomousSteeringSystem_B;

/* Continuous states (default storage) */
extern X_AutonomousSteeringSystem_T AutonomousSteeringSystem_X;

/* Block states (default storage) */
extern DW_AutonomousSteeringSystem_T AutonomousSteeringSystem_DW;
extern const ConstB_AutonomousSteeringSyst_T AutonomousSteeringSystem_ConstB;/* constant block i/o */

/* Constant parameters (default storage) */
extern const ConstP_AutonomousSteeringSyst_T AutonomousSteeringSystem_ConstP;

/* Model entry point functions */
extern void AutonomousSteeringSystem_initialize(void);
extern void AutonomousSteeringSystem_step(void);
extern void AutonomousSteeringSystem_terminate(void);

/* Real-time Model object */
extern RT_MODEL_AutonomousSteeringSy_T *const AutonomousSteeringSystem_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<Root>/Lateral Position' : Unused code path elimination
 * Block '<S4>/Floor' : Unused code path elimination
 * Block '<S4>/Floor1' : Unused code path elimination
 * Block '<S5>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S6>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S7>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S8>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S9>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S10>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S11>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S12>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S13>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S14>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S15>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S16>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S17>/Vector Dimension Check' : Unused code path elimination
 * Block '<S18>/Vector Dimension Check' : Unused code path elimination
 * Block '<S19>/Vector Dimension Check' : Unused code path elimination
 * Block '<S20>/Vector Dimension Check' : Unused code path elimination
 * Block '<S21>/Vector Dimension Check' : Unused code path elimination
 * Block '<S22>/Vector Dimension Check' : Unused code path elimination
 * Block '<S4>/constant' : Unused code path elimination
 * Block '<S23>/Vector Dimension Check' : Unused code path elimination
 * Block '<S4>/umin_scale2' : Unused code path elimination
 * Block '<S4>/umin_scale3' : Unused code path elimination
 * Block '<S4>/umin_scale5' : Unused code path elimination
 * Block '<S4>/ym_zero' : Unused code path elimination
 * Block '<S2>/m_zero' : Unused code path elimination
 * Block '<S2>/p_zero' : Unused code path elimination
 * Block '<Root>/Yaw Angle' : Unused code path elimination
 * Block '<Root>/Scope2' : Unused code path elimination
 * Block '<S4>/Reshape' : Reshape block reduction
 * Block '<S4>/Reshape1' : Reshape block reduction
 * Block '<S4>/Reshape2' : Reshape block reduction
 * Block '<S4>/Reshape3' : Reshape block reduction
 * Block '<S4>/Reshape4' : Reshape block reduction
 * Block '<S4>/Reshape5' : Reshape block reduction
 * Block '<S4>/ext.mv_scale' : Eliminated nontunable gain of 1
 * Block '<S4>/ext.mv_scale1' : Eliminated nontunable gain of 1
 * Block '<S4>/umin_scale1' : Eliminated nontunable gain of 1
 * Block '<S4>/umin_scale4' : Eliminated nontunable gain of 1
 * Block '<S4>/ymin_scale1' : Eliminated nontunable gain of 1
 * Block '<S4>/ymin_scale2' : Eliminated nontunable gain of 1
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'AutonomousSteeringSystem'
 * '<S1>'   : 'AutonomousSteeringSystem/Display results'
 * '<S2>'   : 'AutonomousSteeringSystem/MPC Controller'
 * '<S3>'   : 'AutonomousSteeringSystem/Reference'
 * '<S4>'   : 'AutonomousSteeringSystem/MPC Controller/MPC'
 * '<S5>'   : 'AutonomousSteeringSystem/MPC Controller/MPC/MPC Matrix Signal Check'
 * '<S6>'   : 'AutonomousSteeringSystem/MPC Controller/MPC/MPC Matrix Signal Check1'
 * '<S7>'   : 'AutonomousSteeringSystem/MPC Controller/MPC/MPC Matrix Signal Check2'
 * '<S8>'   : 'AutonomousSteeringSystem/MPC Controller/MPC/MPC Preview Signal Check'
 * '<S9>'   : 'AutonomousSteeringSystem/MPC Controller/MPC/MPC Preview Signal Check1'
 * '<S10>'  : 'AutonomousSteeringSystem/MPC Controller/MPC/MPC Preview Signal Check2'
 * '<S11>'  : 'AutonomousSteeringSystem/MPC Controller/MPC/MPC Preview Signal Check3'
 * '<S12>'  : 'AutonomousSteeringSystem/MPC Controller/MPC/MPC Preview Signal Check4'
 * '<S13>'  : 'AutonomousSteeringSystem/MPC Controller/MPC/MPC Preview Signal Check5'
 * '<S14>'  : 'AutonomousSteeringSystem/MPC Controller/MPC/MPC Preview Signal Check6'
 * '<S15>'  : 'AutonomousSteeringSystem/MPC Controller/MPC/MPC Preview Signal Check7'
 * '<S16>'  : 'AutonomousSteeringSystem/MPC Controller/MPC/MPC Preview Signal Check8'
 * '<S17>'  : 'AutonomousSteeringSystem/MPC Controller/MPC/MPC Scalar Signal Check'
 * '<S18>'  : 'AutonomousSteeringSystem/MPC Controller/MPC/MPC Scalar Signal Check1'
 * '<S19>'  : 'AutonomousSteeringSystem/MPC Controller/MPC/MPC Scalar Signal Check2'
 * '<S20>'  : 'AutonomousSteeringSystem/MPC Controller/MPC/MPC Vector Signal Check'
 * '<S21>'  : 'AutonomousSteeringSystem/MPC Controller/MPC/MPC Vector Signal Check1'
 * '<S22>'  : 'AutonomousSteeringSystem/MPC Controller/MPC/MPC Vector Signal Check6'
 * '<S23>'  : 'AutonomousSteeringSystem/MPC Controller/MPC/moorx'
 * '<S24>'  : 'AutonomousSteeringSystem/MPC Controller/MPC/optimizer'
 * '<S25>'  : 'AutonomousSteeringSystem/MPC Controller/MPC/optimizer/optimizer'
 */
#endif                              /* RTW_HEADER_AutonomousSteeringSystem_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
