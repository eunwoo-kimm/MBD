/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: AutonomousSteeringSystem_data.c
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

#include "AutonomousSteeringSystem.h"
#include "AutonomousSteeringSystem_private.h"

/* Invariant block signals (default storage) */
const ConstB_AutonomousSteeringSyst_T AutonomousSteeringSystem_ConstB =
{
  {
    0.0, 0.0
  },                                   /* '<S4>/Math Function' */
  0.0,                                 /* '<S4>/Math Function1' */
  0.0                                  /* '<S4>/Math Function2' */
};

/* Constant parameters (default storage) */
const ConstP_AutonomousSteeringSyst_T AutonomousSteeringSystem_ConstP =
{
  /* Computed Parameter: Plant_A_pr
   * Referenced by: '<Root>/Plant'
   */
  {
    -4.4021164021164028, 1.3913043478260869, 1.0, 15.0, -12.46031746031746, 1.0,
    -5.186782608695653
  },

  /* Computed Parameter: Plant_B_pr
   * Referenced by: '<Root>/Plant'
   */
  {
    24.126984126984127, 15.860869565217392
  },

  /* Computed Parameter: Plant_A_ir
   * Referenced by: '<Root>/Plant'
   */
  {
    0U, 2U, 3U, 3U, 0U, 1U, 2U
  },

  /* Pooled Parameter (Expression: [2*Cf/m 0 2*Cf*lf/Iz 0]';
     )
   * Referenced by: '<Root>/Plant'
   */
  {
    0U, 2U
  },

  /* Computed Parameter: Plant_C_ir
   * Referenced by: '<Root>/Plant'
   */
  {
    1U, 0U
  }
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
