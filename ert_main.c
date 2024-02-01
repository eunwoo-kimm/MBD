/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: ert_main.c
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
#define _CRT_SECURE_NO_WARNINGS

#include <stddef.h>
#include <stdio.h>            /* This example main program uses printf/fflush */
#include "AutonomousSteeringSystem.h"  /* Model's header file */
/*
 * Associating rt_OneStep with a real-time clock or interrupt service routine
 * is what makes the generated code "real-time".  The function rt_OneStep is
 * always associated with the base rate of the model.  Subrates are managed
 * by the base rate from inside the generated code.  Enabling/disabling
 * interrupts and floating point context switches are target specific.  This
 * example code indicates where these should take place relative to executing
 * the generated code step function.  Overrun behavior should be tailored to
 * your application needs.  This example simply sets an error status in the
 * real-time model and returns from rt_OneStep.
 */
void rt_OneStep(void);
void rt_OneStep(void)
{
  static boolean_T OverrunFlag = false;

  /* Disable interrupts here */

  /* Check for overrun */
  if (OverrunFlag)
  {
    rtmSetErrorStatus(AutonomousSteeringSystem_M, "Overrun");
    return;
  }

  OverrunFlag = true;

  /* Save FPU context here (if necessary) */
  /* Re-enable timer or interrupt here */
  /* Set model inputs here */

  /* Step the model for base rate */
  AutonomousSteeringSystem_step();

  /* Get model outputs here */

  /* Indicate task complete */
  OverrunFlag = false;

  /* Disable interrupts here */
  /* Restore FPU context here (if necessary) */
  /* Enable interrupts here */
}

/*
 * The example "main" function illustrates what is required by your
 * application code to initialize, execute, and terminate the generated code.
 * Attaching rt_OneStep to a real-time clock is target specific.  This example
 * illustrates how you do this relative to initializing the model.
 */
int cnt = 0;
int_T main(int_T argc, const char *argv[])
{
    FILE* fa;
    fa = fopen("res1.txt", "w");

  /* Unused arguments */
  (void)(argc);
  (void)(argv);

  /* Initialize model */
  AutonomousSteeringSystem_initialize();

  /* Simulating the model step behavior (in non real-time) to
   *  simulate model behavior at stop time.
   */
  while ((rtmGetErrorStatus(AutonomousSteeringSystem_M) == (NULL)) &&
         !rtmGetStopRequested(AutonomousSteeringSystem_M))
  {
    rt_OneStep();
    fprintf(fa, "%g\n", AutonomousSteeringSystem_B.xk1[1]);
    printf("%g\n", AutonomousSteeringSystem_B.xk1[1]);
    cnt++;
    if (cnt == 200)
        break;

  }

  /* Disable rt_OneStep here */
  /* Terminate model */
  AutonomousSteeringSystem_terminate();
  return 0;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
