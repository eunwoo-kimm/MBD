/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: AutonomousSteeringSystem.c
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

/* Block signals (default storage) */
B_AutonomousSteeringSystem_T AutonomousSteeringSystem_B;

/* Continuous states */
X_AutonomousSteeringSystem_T AutonomousSteeringSystem_X;

/* Block states (default storage) */
DW_AutonomousSteeringSystem_T AutonomousSteeringSystem_DW;

/* Real-time model */
static RT_MODEL_AutonomousSteeringSy_T AutonomousSteeringSystem_M_;
RT_MODEL_AutonomousSteeringSy_T *const AutonomousSteeringSystem_M =
  &AutonomousSteeringSystem_M_;

/* Forward declaration for local functions */
static void AutonomousSteerin_Unconstrained(const real_T b_Hinv[16], const
  real_T f[4], real_T x[4], int16_T n);
static real_T AutonomousSteeringSystem_norm(const real_T x[4]);
static void AutonomousSteeringSystem_abs(const real_T x[4], real_T y[4]);
static real_T AutonomousSteeringSyste_maximum(const real_T x[4]);
static void AutonomousSteeringSystem_abs_c(const real_T x[52], real_T y[52]);
static void AutonomousSteeringSyst_maximum2(const real_T x[52], real_T y, real_T
  ex[52]);
static real_T AutonomousSteeringSystem_xnrm2(int32_T n, const real_T x[16],
  int32_T ix0);
static void AutonomousSteeringSystem_xgemv(int32_T b_m, int32_T n, const real_T
  b_A[16], int32_T ia0, const real_T x[16], int32_T ix0, real_T y[4]);
static void AutonomousSteeringSystem_xgerc(int32_T b_m, int32_T n, real_T alpha1,
  int32_T ix0, const real_T y[4], real_T b_A[16], int32_T ia0);
static void AutonomousSteeringSystem_qr(const real_T b_A[16], real_T Q[16],
  real_T R[16]);
static real_T AutonomousSteeringSy_KWIKfactor(const real_T b_Ac[208], const
  int16_T iC[52], int16_T nA, const real_T b_Linv[16], real_T RLinv[16], real_T
  D[16], real_T b_H[16], int16_T n);
static real_T AutonomousSteeringSystem_mtimes(const real_T b_A[4], const real_T
  B[4]);
static void AutonomousSteeri_DropConstraint(int16_T kDrop, int16_T iA[52],
  int16_T *nA, int16_T iC[52]);
static void AutonomousSteeringSystem_qpkwik(const real_T b_Linv[16], const
  real_T b_Hinv[16], const real_T f[4], const real_T b_Ac[208], const real_T b
  [52], int16_T iA[52], int16_T maxiter, real_T FeasTol, real_T x[4], real_T
  lambda[52], real_T *status);

/*
 * This function updates continuous states using the ODE3 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  /* Solver Matrices */
  static const real_T rt_ODE3_A[3] =
  {
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3] =
  {
    {
      1.0/2.0, 0.0, 0.0
    },

    {
      0.0, 3.0/4.0, 0.0
    },

    {
      2.0/9.0, 1.0/3.0, 4.0/9.0
    }
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE3_IntgData *id = (ODE3_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T hB[3];
  int_T i;
  int_T nXc = 4;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  AutonomousSteeringSystem_derivatives();

  /* f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*)); */
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++)
  {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  AutonomousSteeringSystem_step();
  AutonomousSteeringSystem_derivatives();

  /* f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*)); */
  for (i = 0; i <= 1; i++)
  {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++)
  {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  AutonomousSteeringSystem_step();
  AutonomousSteeringSystem_derivatives();

  /* tnew = t + hA(3);
     ynew = y + f*hB(:,3); */
  for (i = 0; i <= 2; i++)
  {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++)
  {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/* Function for MATLAB Function: '<S24>/optimizer' */
static void AutonomousSteerin_Unconstrained(const real_T b_Hinv[16], const
  real_T f[4], real_T x[4], int16_T n)
{
  int32_T i;
  for (i = 1; i - 1 < n; i++)
  {
    x[(int16_T)i - 1] = ((-b_Hinv[(int16_T)i - 1] * f[0] + -b_Hinv[(int16_T)i +
                          3] * f[1]) + -b_Hinv[(int16_T)i + 7] * f[2]) +
      -b_Hinv[(int16_T)i + 11] * f[3];
  }
}

/* Function for MATLAB Function: '<S24>/optimizer' */
static real_T AutonomousSteeringSystem_norm(const real_T x[4])
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  scale = 3.3121686421112381E-170;
  absxk = fabs(x[0]);
  if (absxk > 3.3121686421112381E-170)
  {
    y = 1.0;
    scale = absxk;
  }
  else
  {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  absxk = fabs(x[1]);
  if (absxk > scale)
  {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  }
  else
  {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabs(x[2]);
  if (absxk > scale)
  {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  }
  else
  {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabs(x[3]);
  if (absxk > scale)
  {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  }
  else
  {
    t = absxk / scale;
    y += t * t;
  }

  return scale * sqrt(y);
}

/* Function for MATLAB Function: '<S24>/optimizer' */
static void AutonomousSteeringSystem_abs(const real_T x[4], real_T y[4])
{
  y[0] = fabs(x[0]);
  y[1] = fabs(x[1]);
  y[2] = fabs(x[2]);
  y[3] = fabs(x[3]);
}

/* Function for MATLAB Function: '<S24>/optimizer' */
static real_T AutonomousSteeringSyste_maximum(const real_T x[4])
{
  real_T ex;
  int32_T idx;
  if (!rtIsNaN(x[0]))
  {
    idx = 1;
  }
  else
  {
    int32_T k;
    boolean_T exitg1;
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 5))
    {
      if (!rtIsNaN(x[k - 1]))
      {
        idx = k;
        exitg1 = true;
      }
      else
      {
        k++;
      }
    }
  }

  if (idx == 0)
  {
    ex = x[0];
  }
  else
  {
    ex = x[idx - 1];
    while (idx + 1 <= 4)
    {
      if (ex < x[idx])
      {
        ex = x[idx];
      }

      idx++;
    }
  }

  return ex;
}

/* Function for MATLAB Function: '<S24>/optimizer' */
static void AutonomousSteeringSystem_abs_c(const real_T x[52], real_T y[52])
{
  int32_T k;
  for (k = 0; k < 52; k++)
  {
    y[k] = fabs(x[k]);
  }
}

/* Function for MATLAB Function: '<S24>/optimizer' */
static void AutonomousSteeringSyst_maximum2(const real_T x[52], real_T y, real_T
  ex[52])
{
  int32_T k;
  for (k = 0; k < 52; k++)
  {
    ex[k] = fmax(x[k], y);
  }
}

/* Function for MATLAB Function: '<S24>/optimizer' */
static real_T AutonomousSteeringSystem_xnrm2(int32_T n, const real_T x[16],
  int32_T ix0)
{
  real_T y;
  int32_T k;
  y = 0.0;
  if (n >= 1)
  {
    if (n == 1)
    {
      y = fabs(x[ix0 - 1]);
    }
    else
    {
      real_T scale;
      int32_T kend;
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++)
      {
        real_T absxk;
        absxk = fabs(x[k - 1]);
        if (absxk > scale)
        {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        }
        else
        {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

real_T rt_hypotd_snf(real_T u0, real_T u1)
{
  real_T a;
  real_T y;
  a = fabs(u0);
  y = fabs(u1);
  if (a < y)
  {
    a /= y;
    y *= sqrt(a * a + 1.0);
  }
  else if (a > y)
  {
    y /= a;
    y = sqrt(y * y + 1.0) * a;
  }
  else if (!rtIsNaN(y))
  {
    y = a * 1.4142135623730951;
  }

  return y;
}

/* Function for MATLAB Function: '<S24>/optimizer' */
static void AutonomousSteeringSystem_xgemv(int32_T b_m, int32_T n, const real_T
  b_A[16], int32_T ia0, const real_T x[16], int32_T ix0, real_T y[4])
{
  int32_T b_iy;
  int32_T ia;
  int32_T iac;
  if ((b_m != 0) && (n != 0))
  {
    int32_T b;
    for (b_iy = 0; b_iy < n; b_iy++)
    {
      y[b_iy] = 0.0;
    }

    b_iy = 0;
    b = ((n - 1) << 2) + ia0;
    for (iac = ia0; iac <= b; iac += 4)
    {
      real_T c;
      int32_T d;
      int32_T ix;
      ix = ix0;
      c = 0.0;
      d = (iac + b_m) - 1;
      for (ia = iac; ia <= d; ia++)
      {
        c += b_A[ia - 1] * x[ix - 1];
        ix++;
      }

      y[b_iy] += c;
      b_iy++;
    }
  }
}

/* Function for MATLAB Function: '<S24>/optimizer' */
static void AutonomousSteeringSystem_xgerc(int32_T b_m, int32_T n, real_T alpha1,
  int32_T ix0, const real_T y[4], real_T b_A[16], int32_T ia0)
{
  int32_T j;
  if (!(alpha1 == 0.0))
  {
    int32_T jA;
    int32_T jy;
    jA = ia0 - 1;
    jy = 0;
    for (j = 0; j < n; j++)
    {
      if (y[jy] != 0.0)
      {
        real_T temp;
        int32_T b;
        int32_T ijA;
        int32_T ix;
        temp = y[jy] * alpha1;
        ix = ix0;
        ijA = jA;
        b = b_m + jA;
        while (ijA + 1 <= b)
        {
          b_A[ijA] += b_A[ix - 1] * temp;
          ix++;
          ijA++;
        }
      }

      jy++;
      jA += 4;
    }
  }
}

/* Function for MATLAB Function: '<S24>/optimizer' */
static void AutonomousSteeringSystem_qr(const real_T b_A[16], real_T Q[16],
  real_T R[16])
{
  real_T c_A[16];
  real_T work[4];
  real_T atmp;
  real_T beta1;
  real_T tau_idx_0;
  real_T tau_idx_1;
  real_T tau_idx_2;
  int32_T b_coltop;
  int32_T c_lastc;
  int32_T coltop;
  int32_T exitg1;
  int32_T knt;
  boolean_T exitg2;
  memcpy(&c_A[0], &b_A[0], sizeof(real_T) << 4U);
  tau_idx_0 = 0.0;
  work[0] = 0.0;
  tau_idx_1 = 0.0;
  work[1] = 0.0;
  tau_idx_2 = 0.0;
  work[2] = 0.0;
  work[3] = 0.0;
  atmp = c_A[0];
  beta1 = AutonomousSteeringSystem_xnrm2(3, c_A, 2);
  if (beta1 != 0.0)
  {
    beta1 = rt_hypotd_snf(c_A[0], beta1);
    if (c_A[0] >= 0.0)
    {
      beta1 = -beta1;
    }

    if (fabs(beta1) < 1.0020841800044864E-292)
    {
      knt = 0;
      do
      {
        knt++;
        for (c_lastc = 1; c_lastc < 4; c_lastc++)
        {
          c_A[c_lastc] *= 9.9792015476736E+291;
        }

        beta1 *= 9.9792015476736E+291;
        atmp *= 9.9792015476736E+291;
      }
      while ((fabs(beta1) < 1.0020841800044864E-292) && (knt < 20));

      beta1 = rt_hypotd_snf(atmp, AutonomousSteeringSystem_xnrm2(3, c_A, 2));
      if (atmp >= 0.0)
      {
        beta1 = -beta1;
      }

      tau_idx_0 = (beta1 - atmp) / beta1;
      atmp = 1.0 / (atmp - beta1);
      for (c_lastc = 1; c_lastc < 4; c_lastc++)
      {
        c_A[c_lastc] *= atmp;
      }

      for (c_lastc = 0; c_lastc < knt; c_lastc++)
      {
        beta1 *= 1.0020841800044864E-292;
      }

      atmp = beta1;
    }
    else
    {
      tau_idx_0 = (beta1 - c_A[0]) / beta1;
      atmp = 1.0 / (c_A[0] - beta1);
      for (knt = 1; knt < 4; knt++)
      {
        c_A[knt] *= atmp;
      }

      atmp = beta1;
    }
  }

  c_A[0] = 1.0;
  if (tau_idx_0 != 0.0)
  {
    knt = 4;
    c_lastc = 3;
    while ((knt > 0) && (c_A[c_lastc] == 0.0))
    {
      knt--;
      c_lastc--;
    }

    c_lastc = 3;
    exitg2 = false;
    while ((!exitg2) && (c_lastc > 0))
    {
      b_coltop = ((c_lastc - 1) << 2) + 4;
      coltop = b_coltop;
      do
      {
        exitg1 = 0;
        if (coltop + 1 <= b_coltop + knt)
        {
          if (c_A[coltop] != 0.0)
          {
            exitg1 = 1;
          }
          else
          {
            coltop++;
          }
        }
        else
        {
          c_lastc--;
          exitg1 = 2;
        }
      }
      while (exitg1 == 0);

      if (exitg1 == 1)
      {
        exitg2 = true;
      }
    }
  }
  else
  {
    knt = 0;
    c_lastc = 0;
  }

  if (knt > 0)
  {
    AutonomousSteeringSystem_xgemv(knt, c_lastc, c_A, 5, c_A, 1, work);
    AutonomousSteeringSystem_xgerc(knt, c_lastc, -tau_idx_0, 1, work, c_A, 5);
  }

  c_A[0] = atmp;
  atmp = c_A[5];
  beta1 = AutonomousSteeringSystem_xnrm2(2, c_A, 7);
  if (beta1 != 0.0)
  {
    beta1 = rt_hypotd_snf(c_A[5], beta1);
    if (c_A[5] >= 0.0)
    {
      beta1 = -beta1;
    }

    if (fabs(beta1) < 1.0020841800044864E-292)
    {
      knt = 0;
      do
      {
        knt++;
        for (c_lastc = 6; c_lastc < 8; c_lastc++)
        {
          c_A[c_lastc] *= 9.9792015476736E+291;
        }

        beta1 *= 9.9792015476736E+291;
        atmp *= 9.9792015476736E+291;
      }
      while ((fabs(beta1) < 1.0020841800044864E-292) && (knt < 20));

      beta1 = rt_hypotd_snf(atmp, AutonomousSteeringSystem_xnrm2(2, c_A, 7));
      if (atmp >= 0.0)
      {
        beta1 = -beta1;
      }

      tau_idx_1 = (beta1 - atmp) / beta1;
      atmp = 1.0 / (atmp - beta1);
      for (c_lastc = 6; c_lastc < 8; c_lastc++)
      {
        c_A[c_lastc] *= atmp;
      }

      for (c_lastc = 0; c_lastc < knt; c_lastc++)
      {
        beta1 *= 1.0020841800044864E-292;
      }

      atmp = beta1;
    }
    else
    {
      tau_idx_1 = (beta1 - c_A[5]) / beta1;
      atmp = 1.0 / (c_A[5] - beta1);
      for (knt = 6; knt < 8; knt++)
      {
        c_A[knt] *= atmp;
      }

      atmp = beta1;
    }
  }

  c_A[5] = 1.0;
  if (tau_idx_1 != 0.0)
  {
    knt = 3;
    c_lastc = 7;
    while ((knt > 0) && (c_A[c_lastc] == 0.0))
    {
      knt--;
      c_lastc--;
    }

    c_lastc = 2;
    exitg2 = false;
    while ((!exitg2) && (c_lastc > 0))
    {
      b_coltop = ((c_lastc - 1) << 2) + 9;
      coltop = b_coltop;
      do
      {
        exitg1 = 0;
        if (coltop + 1 <= b_coltop + knt)
        {
          if (c_A[coltop] != 0.0)
          {
            exitg1 = 1;
          }
          else
          {
            coltop++;
          }
        }
        else
        {
          c_lastc--;
          exitg1 = 2;
        }
      }
      while (exitg1 == 0);

      if (exitg1 == 1)
      {
        exitg2 = true;
      }
    }
  }
  else
  {
    knt = 0;
    c_lastc = 0;
  }

  if (knt > 0)
  {
    AutonomousSteeringSystem_xgemv(knt, c_lastc, c_A, 10, c_A, 6, work);
    AutonomousSteeringSystem_xgerc(knt, c_lastc, -tau_idx_1, 6, work, c_A, 10);
  }

  c_A[5] = atmp;
  atmp = c_A[10];
  beta1 = AutonomousSteeringSystem_xnrm2(1, c_A, 12);
  if (beta1 != 0.0)
  {
    beta1 = rt_hypotd_snf(c_A[10], beta1);
    if (c_A[10] >= 0.0)
    {
      beta1 = -beta1;
    }

    if (fabs(beta1) < 1.0020841800044864E-292)
    {
      knt = 0;
      do
      {
        knt++;
        for (c_lastc = 11; c_lastc < 12; c_lastc++)
        {
          c_A[c_lastc] *= 9.9792015476736E+291;
        }

        beta1 *= 9.9792015476736E+291;
        atmp *= 9.9792015476736E+291;
      }
      while ((fabs(beta1) < 1.0020841800044864E-292) && (knt < 20));

      beta1 = rt_hypotd_snf(atmp, AutonomousSteeringSystem_xnrm2(1, c_A, 12));
      if (atmp >= 0.0)
      {
        beta1 = -beta1;
      }

      tau_idx_2 = (beta1 - atmp) / beta1;
      atmp = 1.0 / (atmp - beta1);
      for (c_lastc = 11; c_lastc < 12; c_lastc++)
      {
        c_A[c_lastc] *= atmp;
      }

      for (c_lastc = 0; c_lastc < knt; c_lastc++)
      {
        beta1 *= 1.0020841800044864E-292;
      }

      atmp = beta1;
    }
    else
    {
      tau_idx_2 = (beta1 - c_A[10]) / beta1;
      atmp = 1.0 / (c_A[10] - beta1);
      for (knt = 11; knt < 12; knt++)
      {
        c_A[knt] *= atmp;
      }

      atmp = beta1;
    }
  }

  c_A[10] = 1.0;
  if (tau_idx_2 != 0.0)
  {
    knt = 2;
    c_lastc = 11;
    while ((knt > 0) && (c_A[c_lastc] == 0.0))
    {
      knt--;
      c_lastc--;
    }

    c_lastc = 1;
    coltop = 14;
    do
    {
      exitg1 = 0;
      if (coltop + 1 <= 14 + knt)
      {
        if (c_A[coltop] != 0.0)
        {
          exitg1 = 1;
        }
        else
        {
          coltop++;
        }
      }
      else
      {
        c_lastc = 0;
        exitg1 = 1;
      }
    }
    while (exitg1 == 0);
  }
  else
  {
    knt = 0;
    c_lastc = 0;
  }

  if (knt > 0)
  {
    AutonomousSteeringSystem_xgemv(knt, c_lastc, c_A, 15, c_A, 11, work);
    AutonomousSteeringSystem_xgerc(knt, c_lastc, -tau_idx_2, 11, work, c_A, 15);
  }

  c_A[10] = atmp;
  R[0] = c_A[0];
  for (knt = 1; knt + 1 < 5; knt++)
  {
    R[knt] = 0.0;
  }

  work[0] = 0.0;
  for (knt = 0; knt < 2; knt++)
  {
    R[knt + 4] = c_A[knt + 4];
  }

  for (knt = 2; knt + 1 < 5; knt++)
  {
    R[knt + 4] = 0.0;
  }

  work[1] = 0.0;
  for (knt = 0; knt < 3; knt++)
  {
    R[knt + 8] = c_A[knt + 8];
  }

  for (knt = 3; knt + 1 < 5; knt++)
  {
    R[knt + 8] = 0.0;
  }

  work[2] = 0.0;
  for (knt = 0; knt < 4; knt++)
  {
    R[knt + 12] = c_A[knt + 12];
  }

  work[3] = 0.0;
  c_A[15] = 1.0;
  for (knt = 0; knt < 3; knt++)
  {
    c_A[14 - knt] = 0.0;
  }

  c_A[10] = 1.0;
  if (tau_idx_2 != 0.0)
  {
    knt = 2;
    c_lastc = 13;
    while ((knt > 0) && (c_A[c_lastc - 2] == 0.0))
    {
      knt--;
      c_lastc--;
    }

    c_lastc = 1;
    b_coltop = 15;
    do
    {
      exitg1 = 0;
      if (b_coltop <= knt + 14)
      {
        if (c_A[b_coltop - 1] != 0.0)
        {
          exitg1 = 1;
        }
        else
        {
          b_coltop++;
        }
      }
      else
      {
        c_lastc = 0;
        exitg1 = 1;
      }
    }
    while (exitg1 == 0);
  }
  else
  {
    knt = 0;
    c_lastc = 0;
  }

  if (knt > 0)
  {
    AutonomousSteeringSystem_xgemv(knt, c_lastc, c_A, 15, c_A, 11, work);
    AutonomousSteeringSystem_xgerc(knt, c_lastc, -tau_idx_2, 11, work, c_A, 15);
  }

  for (knt = 11; knt < 12; knt++)
  {
    c_A[knt] *= -tau_idx_2;
  }

  c_A[10] = 1.0 - tau_idx_2;
  for (knt = 0; knt < 2; knt++)
  {
    c_A[9 - knt] = 0.0;
  }

  c_A[5] = 1.0;
  if (tau_idx_1 != 0.0)
  {
    knt = 3;
    c_lastc = 9;
    while ((knt > 0) && (c_A[c_lastc - 2] == 0.0))
    {
      knt--;
      c_lastc--;
    }

    c_lastc = 2;
    exitg2 = false;
    while ((!exitg2) && (c_lastc > 0))
    {
      coltop = ((c_lastc - 1) << 2) + 10;
      b_coltop = coltop;
      do
      {
        exitg1 = 0;
        if (b_coltop <= (coltop + knt) - 1)
        {
          if (c_A[b_coltop - 1] != 0.0)
          {
            exitg1 = 1;
          }
          else
          {
            b_coltop++;
          }
        }
        else
        {
          c_lastc--;
          exitg1 = 2;
        }
      }
      while (exitg1 == 0);

      if (exitg1 == 1)
      {
        exitg2 = true;
      }
    }
  }
  else
  {
    knt = 0;
    c_lastc = 0;
  }

  if (knt > 0)
  {
    AutonomousSteeringSystem_xgemv(knt, c_lastc, c_A, 10, c_A, 6, work);
    AutonomousSteeringSystem_xgerc(knt, c_lastc, -tau_idx_1, 6, work, c_A, 10);
  }

  for (knt = 6; knt < 8; knt++)
  {
    c_A[knt] *= -tau_idx_1;
  }

  c_A[5] = 1.0 - tau_idx_1;
  c_A[4] = 0.0;
  c_A[0] = 1.0;
  if (tau_idx_0 != 0.0)
  {
    knt = 4;
    c_lastc = 5;
    while ((knt > 0) && (c_A[c_lastc - 2] == 0.0))
    {
      knt--;
      c_lastc--;
    }

    c_lastc = 3;
    exitg2 = false;
    while ((!exitg2) && (c_lastc > 0))
    {
      coltop = ((c_lastc - 1) << 2) + 5;
      b_coltop = coltop;
      do
      {
        exitg1 = 0;
        if (b_coltop <= (coltop + knt) - 1)
        {
          if (c_A[b_coltop - 1] != 0.0)
          {
            exitg1 = 1;
          }
          else
          {
            b_coltop++;
          }
        }
        else
        {
          c_lastc--;
          exitg1 = 2;
        }
      }
      while (exitg1 == 0);

      if (exitg1 == 1)
      {
        exitg2 = true;
      }
    }
  }
  else
  {
    knt = 0;
    c_lastc = 0;
  }

  if (knt > 0)
  {
    AutonomousSteeringSystem_xgemv(knt, c_lastc, c_A, 5, c_A, 1, work);
    AutonomousSteeringSystem_xgerc(knt, c_lastc, -tau_idx_0, 1, work, c_A, 5);
  }

  for (knt = 1; knt < 4; knt++)
  {
    c_A[knt] *= -tau_idx_0;
  }

  c_A[0] = 1.0 - tau_idx_0;
  for (knt = 0; knt < 4; knt++)
  {
    c_lastc = knt << 2;
    Q[c_lastc] = c_A[c_lastc];
    Q[c_lastc + 1] = c_A[c_lastc + 1];
    Q[c_lastc + 2] = c_A[c_lastc + 2];
    Q[c_lastc + 3] = c_A[c_lastc + 3];
  }
}

/* Function for MATLAB Function: '<S24>/optimizer' */
static real_T AutonomousSteeringSy_KWIKfactor(const real_T b_Ac[208], const
  int16_T iC[52], int16_T nA, const real_T b_Linv[16], real_T RLinv[16], real_T
  D[16], real_T b_H[16], int16_T n)
{
  real_T QQ[16];
  real_T RR[16];
  real_T TL[16];
  real_T Status;
  int32_T b_i;
  int32_T f_i;
  int32_T i;
  int32_T j;
  int16_T b_j;
  int16_T c_k;
  Status = 1.0;
  memset(&RLinv[0], 0, sizeof(real_T) << 4U);
  for (i = 1; i - 1 < nA; i++)
  {
    f_i = iC[(int16_T)i - 1];
    for (b_i = 0; b_i <= 2; b_i += 2)
    {
      __m128d tmp;
      j = (((int16_T)i - 1) << 2) + b_i;
      _mm_storeu_pd(&RLinv[j], _mm_set1_pd(0.0));
      tmp = _mm_loadu_pd(&RLinv[j]);
      _mm_storeu_pd(&RLinv[j], _mm_add_pd(tmp, _mm_mul_pd(_mm_set1_pd(b_Ac[f_i -
        1]), _mm_loadu_pd(&b_Linv[b_i]))));
      tmp = _mm_loadu_pd(&RLinv[j]);
      _mm_storeu_pd(&RLinv[j], _mm_add_pd(tmp, _mm_mul_pd(_mm_loadu_pd
        (&b_Linv[b_i + 4]), _mm_set1_pd(b_Ac[f_i + 51]))));
      tmp = _mm_loadu_pd(&RLinv[j]);
      _mm_storeu_pd(&RLinv[j], _mm_add_pd(tmp, _mm_mul_pd(_mm_loadu_pd
        (&b_Linv[b_i + 8]), _mm_set1_pd(b_Ac[f_i + 103]))));
      tmp = _mm_loadu_pd(&RLinv[j]);
      _mm_storeu_pd(&RLinv[j], _mm_add_pd(tmp, _mm_mul_pd(_mm_loadu_pd
        (&b_Linv[b_i + 12]), _mm_set1_pd(b_Ac[f_i + 155]))));
    }
  }

  AutonomousSteeringSystem_qr(RLinv, QQ, RR);
  b_i = 1;
  int32_T exitg1;
  do
  {
    exitg1 = 0;
    if (b_i - 1 <= nA - 1)
    {
      if (fabs(RR[((((int16_T)b_i - 1) << 2) + (int16_T)b_i) - 1]) < 1.0E-12)
      {
        Status = -2.0;
        exitg1 = 1;
      }
      else
      {
        b_i++;
      }
    }
    else
    {
      for (b_i = 1; b_i - 1 < n; b_i++)
      {
        for (j = 1; j - 1 < n; j++)
        {
          i = ((int16_T)b_i - 1) << 2;
          f_i = ((int16_T)j - 1) << 2;
          TL[((int16_T)b_i + f_i) - 1] = ((b_Linv[i + 1] * QQ[f_i + 1] +
            b_Linv[i] * QQ[f_i]) + b_Linv[i + 2] * QQ[f_i + 2]) + b_Linv[i + 3] *
            QQ[f_i + 3];
        }
      }

      memset(&RLinv[0], 0, sizeof(real_T) << 4U);
      for (b_j = nA; b_j > 0; b_j--)
      {
        b_i = (b_j - 1) << 2;
        j = (b_j + b_i) - 1;
        RLinv[j] = 1.0;
        for (c_k = b_j; c_k <= nA; c_k++)
        {
          f_i = (((c_k - 1) << 2) + b_j) - 1;
          RLinv[f_i] /= RR[j];
        }

        if (b_j > 1)
        {
          for (i = 1; i - 1 <= b_j - 2; i++)
          {
            for (c_k = b_j; c_k <= nA; c_k++)
            {
              j = (c_k - 1) << 2;
              f_i = (j + (int16_T)i) - 1;
              RLinv[f_i] -= RR[(b_i + (int16_T)i) - 1] * RLinv[(j + b_j) - 1];
            }
          }
        }
      }

      for (i = 1; i - 1 < n; i++)
      {
        for (b_j = (int16_T)i; b_j <= n; b_j++)
        {
          b_i = (((b_j - 1) << 2) + (int16_T)i) - 1;
          b_H[b_i] = 0.0;
          j = nA + 1;
          if (nA + 1 > 32767)
          {
            j = 32767;
          }

          for (c_k = (int16_T)j; c_k <= n; c_k++)
          {
            j = (c_k - 1) << 2;
            b_H[b_i] -= TL[(j + (int16_T)i) - 1] * TL[(j + b_j) - 1];
          }

          b_H[(b_j + (((int16_T)i - 1) << 2)) - 1] = b_H[b_i];
        }
      }

      for (i = 1; i - 1 < nA; i++)
      {
        for (f_i = 1; f_i - 1 < n; f_i++)
        {
          b_i = ((((int16_T)i - 1) << 2) + (int16_T)f_i) - 1;
          D[b_i] = 0.0;
          for (b_j = (int16_T)i; b_j <= nA; b_j++)
          {
            j = (b_j - 1) << 2;
            D[b_i] += TL[(j + (int16_T)f_i) - 1] * RLinv[(j + (int16_T)i) - 1];
          }
        }
      }

      exitg1 = 1;
    }
  }
  while (exitg1 == 0);

  return Status;
}

/* Function for MATLAB Function: '<S24>/optimizer' */
static real_T AutonomousSteeringSystem_mtimes(const real_T b_A[4], const real_T
  B[4])
{
  return ((b_A[0] * B[0] + b_A[1] * B[1]) + b_A[2] * B[2]) + b_A[3] * B[3];
}

/* Function for MATLAB Function: '<S24>/optimizer' */
static void AutonomousSteeri_DropConstraint(int16_T kDrop, int16_T iA[52],
  int16_T *nA, int16_T iC[52])
{
  int32_T tmp;
  int16_T i;
  iA[iC[kDrop - 1] - 1] = 0;
  if (kDrop < *nA)
  {
    tmp = *nA - 1;
    if (*nA - 1 < -32768)
    {
      tmp = -32768;
    }

    for (i = kDrop; i <= (int16_T)tmp; i++)
    {
      iC[i - 1] = iC[i];
    }
  }

  iC[*nA - 1] = 0;
  tmp = *nA - 1;
  if (*nA - 1 < -32768)
  {
    tmp = -32768;
  }

  *nA = (int16_T)tmp;
}

/* Function for MATLAB Function: '<S24>/optimizer' */
static void AutonomousSteeringSystem_qpkwik(const real_T b_Linv[16], const
  real_T b_Hinv[16], const real_T f[4], const real_T b_Ac[208], const real_T b
  [52], int16_T iA[52], int16_T maxiter, real_T FeasTol, real_T x[4], real_T
  lambda[52], real_T *status)
{
  real_T cTol[52];
  real_T tmp_6[52];
  real_T D[16];
  real_T RLinv[16];
  real_T U[16];
  real_T b_H[16];
  real_T Opt[8];
  real_T Rhs[8];
  real_T b_Ac_0[4];
  real_T r[4];
  real_T z[4];
  real_T Xnorm0;
  real_T rMin;
  int32_T b_k;
  int32_T f_i;
  int32_T i;
  int16_T iC[52];
  int16_T kDrop;
  int16_T kNext;
  int16_T nA;
  boolean_T ColdReset;
  boolean_T DualFeasible;
  boolean_T cTolComputed;
  boolean_T guard1 = false;
  *status = 1.0;
  x[0] = 0.0;
  r[0] = 0.0;
  x[1] = 0.0;
  r[1] = 0.0;
  x[2] = 0.0;
  r[2] = 0.0;
  x[3] = 0.0;
  r[3] = 0.0;
  rMin = 0.0;
  cTolComputed = false;
  for (i = 0; i < 52; i++)
  {
    lambda[i] = 0.0;
    cTol[i] = 1.0;
    iC[i] = 0;
  }

  nA = 0;
  for (i = 0; i < 52; i++)
  {
    if (iA[i] == 1)
    {
      f_i = nA + 1;
      if (nA + 1 > 32767)
      {
        f_i = 32767;
      }

      nA = (int16_T)f_i;
      iC[(int16_T)f_i - 1] = (int16_T)(i + 1);
    }
  }

  guard1 = false;
  if (nA > 0)
  {
    int32_T exitg3;
    uint16_T c_x;
    uint16_T q;
    memset(&Opt[0], 0, sizeof(real_T) << 3U);
    Rhs[0] = f[0];
    Rhs[4] = 0.0;
    Rhs[1] = f[1];
    Rhs[5] = 0.0;
    Rhs[2] = f[2];
    Rhs[6] = 0.0;
    Rhs[3] = f[3];
    Rhs[7] = 0.0;
    DualFeasible = false;
    f_i = 3 * nA;
    if (f_i > 32767)
    {
      f_i = 32767;
    }

    if ((int16_T)f_i >= 50)
    {
      kNext = (int16_T)f_i;
    }
    else
    {
      kNext = 50;
    }

    q = (uint16_T)(kNext / 10U);
    c_x = (uint16_T)((uint32_T)kNext - q * 10);
    if ((c_x > 0) && (c_x >= 5))
    {
      q++;
    }

    ColdReset = false;
    do
    {
      exitg3 = 0;
      if ((!DualFeasible) && (nA > 0) && ((int32_T)*status <= maxiter))
      {
        Xnorm0 = AutonomousSteeringSy_KWIKfactor(b_Ac, iC, nA, b_Linv, RLinv, D,
          b_H, 4);
        if (Xnorm0 < 0.0)
        {
          if (ColdReset)
          {
            *status = -2.0;
            exitg3 = 2;
          }
          else
          {
            nA = 0;
            memset(&iA[0], 0, 52U * sizeof(int16_T));
            memset(&iC[0], 0, 52U * sizeof(int16_T));
            ColdReset = true;
          }
        }
        else
        {
          int32_T U_tmp;
          for (i = 1; i - 1 < nA; i++)
          {
            f_i = (int16_T)i + 4;
            if ((int16_T)i + 4 > 32767)
            {
              f_i = 32767;
            }

            Rhs[f_i - 1] = b[iC[(int16_T)i - 1] - 1];
            for (kNext = (int16_T)i; kNext <= nA; kNext++)
            {
              f_i = ((((int16_T)i - 1) << 2) + kNext) - 1;
              U[f_i] = 0.0;
              for (b_k = 1; b_k - 1 < nA; b_k++)
              {
                U_tmp = ((int16_T)b_k - 1) << 2;
                U[f_i] += RLinv[(U_tmp + kNext) - 1] * RLinv[(U_tmp + (int16_T)i)
                  - 1];
              }

              U[((int16_T)i + ((kNext - 1) << 2)) - 1] = U[f_i];
            }
          }

          for (i = 0; i < 4; i++)
          {
            Opt[i] = ((b_H[i + 4] * Rhs[1] + b_H[i] * Rhs[0]) + b_H[i + 8] *
                      Rhs[2]) + b_H[i + 12] * Rhs[3];
            for (b_k = 1; b_k - 1 < nA; b_k++)
            {
              f_i = (int16_T)b_k + 4;
              if ((int16_T)b_k + 4 > 32767)
              {
                f_i = 32767;
              }

              Opt[i] += D[(((int16_T)b_k - 1) << 2) + i] * Rhs[f_i - 1];
            }
          }

          for (i = 1; i - 1 < nA; i++)
          {
            f_i = (int16_T)i + 4;
            if ((int16_T)i + 4 > 32767)
            {
              f_i = 32767;
            }

            b_k = ((int16_T)i - 1) << 2;
            Opt[f_i - 1] = ((D[b_k + 1] * Rhs[1] + D[b_k] * Rhs[0]) + D[b_k + 2]
                            * Rhs[2]) + D[b_k + 3] * Rhs[3];
            for (b_k = 1; b_k - 1 < nA; b_k++)
            {
              int32_T tmp;
              f_i = (int16_T)i + 4;
              if ((int16_T)i + 4 > 32767)
              {
                f_i = 32767;
              }

              U_tmp = (int16_T)i + 4;
              if ((int16_T)i + 4 > 32767)
              {
                U_tmp = 32767;
              }

              tmp = (int16_T)b_k + 4;
              if ((int16_T)b_k + 4 > 32767)
              {
                tmp = 32767;
              }

              Opt[f_i - 1] = U[((((int16_T)b_k - 1) << 2) + (int16_T)i) - 1] *
                Rhs[tmp - 1] + Opt[U_tmp - 1];
            }
          }

          Xnorm0 = -1.0E-12;
          kDrop = 0;
          for (i = 1; i - 1 < nA; i++)
          {
            f_i = (int16_T)i + 4;
            if ((int16_T)i + 4 > 32767)
            {
              f_i = 32767;
            }

            lambda[iC[(int16_T)i - 1] - 1] = Opt[f_i - 1];
            f_i = (int16_T)i + 4;
            if ((int16_T)i + 4 > 32767)
            {
              f_i = 32767;
            }

            if ((Opt[f_i - 1] < Xnorm0) && ((int16_T)i <= nA))
            {
              kDrop = (int16_T)i;
              f_i = (int16_T)i + 4;
              if ((int16_T)i + 4 > 32767)
              {
                f_i = 32767;
              }

              Xnorm0 = Opt[f_i - 1];
            }
          }

          if (kDrop <= 0)
          {
            DualFeasible = true;
            x[0] = Opt[0];
            x[1] = Opt[1];
            x[2] = Opt[2];
            x[3] = Opt[3];
          }
          else
          {
            (*status)++;
            if ((int32_T)*status > q)
            {
              nA = 0;
              memset(&iA[0], 0, 52U * sizeof(int16_T));
              memset(&iC[0], 0, 52U * sizeof(int16_T));
              ColdReset = true;
            }
            else
            {
              lambda[iC[kDrop - 1] - 1] = 0.0;
              AutonomousSteeri_DropConstraint(kDrop, iA, &nA, iC);
            }
          }
        }
      }
      else
      {
        if (nA <= 0)
        {
          memset(&lambda[0], 0, 52U * sizeof(real_T));
          AutonomousSteerin_Unconstrained(b_Hinv, f, x, 4);
        }

        exitg3 = 1;
      }
    }
    while (exitg3 == 0);

    if (exitg3 == 1)
    {
      guard1 = true;
    }
  }
  else
  {
    AutonomousSteerin_Unconstrained(b_Hinv, f, x, 4);
    guard1 = true;
  }

  if (guard1)
  {
    boolean_T exitg2;
    Xnorm0 = AutonomousSteeringSystem_norm(x);
    exitg2 = false;
    while ((!exitg2) && ((int32_T)*status <= maxiter))
    {
      real_T cMin;
      real_T cVal;
      real_T t2;
      cMin = -FeasTol;
      kNext = 0;
      for (f_i = 0; f_i < 52; f_i++)
      {
        t2 = cTol[f_i];
        if (!cTolComputed)
        {
          b_Ac_0[0] = b_Ac[f_i] * x[0];
          b_Ac_0[1] = b_Ac[f_i + 52] * x[1];
          b_Ac_0[2] = b_Ac[f_i + 104] * x[2];
          b_Ac_0[3] = b_Ac[f_i + 156] * x[3];
          AutonomousSteeringSystem_abs(b_Ac_0, z);
          t2 = fmax(t2, AutonomousSteeringSyste_maximum(z));
        }

        if (iA[f_i] == 0)
        {
          cVal = ((((b_Ac[f_i + 52] * x[1] + b_Ac[f_i] * x[0]) + b_Ac[f_i + 104]
                    * x[2]) + b_Ac[f_i + 156] * x[3]) - b[f_i]) / t2;
          if (cVal < cMin)
          {
            cMin = cVal;
            kNext = (int16_T)(f_i + 1);
          }
        }

        cTol[f_i] = t2;
      }

      cTolComputed = true;
      if (kNext <= 0)
      {
        exitg2 = true;
      }
      else if ((int32_T)*status == maxiter)
      {
        *status = 0.0;
        exitg2 = true;
      }
      else
      {
        int32_T exitg1;
        do
        {
          exitg1 = 0;
          if ((kNext > 0) && ((int32_T)*status <= maxiter))
          {
            boolean_T guard2 = false;
            guard2 = false;
            if (nA == 0)
            {
              for (f_i = 0; f_i <= 2; f_i += 2)
              {
                __m128d tmp_0;
                __m128d tmp_1;
                __m128d tmp_3;
                tmp_3 = _mm_add_pd(_mm_mul_pd(_mm_set1_pd(b_Ac[kNext - 1]),
                  _mm_loadu_pd(&b_Hinv[f_i])), _mm_set1_pd(0.0));
                tmp_0 = _mm_mul_pd(_mm_loadu_pd(&b_Hinv[f_i + 4]), _mm_set1_pd
                                   (b_Ac[kNext + 51]));
                tmp_1 = _mm_loadu_pd(&b_Hinv[f_i + 8]);
                _mm_storeu_pd(&z[f_i], _mm_add_pd(_mm_mul_pd(_mm_loadu_pd
                  (&b_Hinv[f_i + 12]), _mm_set1_pd(b_Ac[kNext + 155])),
                  _mm_add_pd(_mm_mul_pd(tmp_1, _mm_set1_pd(b_Ac[kNext + 103])),
                             _mm_add_pd(tmp_0, tmp_3))));
              }

              guard2 = true;
            }
            else
            {
              cMin = AutonomousSteeringSy_KWIKfactor(b_Ac, iC, nA, b_Linv, RLinv,
                D, b_H, 4);
              if (cMin <= 0.0)
              {
                *status = -2.0;
                exitg1 = 1;
              }
              else
              {
                __m128d tmp_3;
                for (f_i = 0; f_i <= 14; f_i += 2)
                {
                  tmp_3 = _mm_loadu_pd(&b_H[f_i]);
                  _mm_storeu_pd(&U[f_i], _mm_mul_pd(tmp_3, _mm_set1_pd(-1.0)));
                }

                for (f_i = 0; f_i <= 2; f_i += 2)
                {
                  __m128d tmp_0;
                  __m128d tmp_1;
                  __m128d tmp_2;
                  tmp_3 = _mm_loadu_pd(&U[f_i]);
                  tmp_0 = _mm_loadu_pd(&U[f_i + 4]);
                  tmp_1 = _mm_loadu_pd(&U[f_i + 8]);
                  tmp_2 = _mm_loadu_pd(&U[f_i + 12]);
                  _mm_storeu_pd(&z[f_i], _mm_add_pd(_mm_mul_pd(tmp_2,
                    _mm_set1_pd(b_Ac[kNext + 155])), _mm_add_pd(_mm_mul_pd(tmp_1,
                    _mm_set1_pd(b_Ac[kNext + 103])), _mm_add_pd(_mm_mul_pd(tmp_0,
                    _mm_set1_pd(b_Ac[kNext + 51])), _mm_add_pd(_mm_mul_pd
                    (_mm_set1_pd(b_Ac[kNext - 1]), tmp_3), _mm_set1_pd(0.0))))));
                }

                for (f_i = 1; f_i - 1 < nA; f_i++)
                {
                  i = ((int16_T)f_i - 1) << 2;
                  r[(int16_T)f_i - 1] = ((D[i + 1] * b_Ac[kNext + 51] +
                    b_Ac[kNext - 1] * D[i]) + D[i + 2] * b_Ac[kNext + 103]) +
                    D[i + 3] * b_Ac[kNext + 155];
                }

                guard2 = true;
              }
            }

            if (guard2)
            {
              real_T b_Ac_tmp;
              real_T b_Ac_tmp_0;
              real_T zTa;
              boolean_T exitg4;
              kDrop = 0;
              cMin = 0.0;
              DualFeasible = true;
              ColdReset = true;
              if (nA > 0)
              {
                f_i = 0;
                exitg4 = false;
                while ((!exitg4) && (f_i <= nA - 1))
                {
                  if (r[f_i] >= 1.0E-12)
                  {
                    ColdReset = false;
                    exitg4 = true;
                  }
                  else
                  {
                    f_i++;
                  }
                }
              }

              if ((nA != 0) && (!ColdReset))
              {
                for (f_i = 1; f_i - 1 < nA; f_i++)
                {
                  t2 = r[(int16_T)f_i - 1];
                  if (t2 > 1.0E-12)
                  {
                    t2 = lambda[iC[(int16_T)f_i - 1] - 1] / t2;
                    if ((kDrop == 0) || (t2 < rMin))
                    {
                      rMin = t2;
                      kDrop = (int16_T)f_i;
                    }
                  }
                }

                if (kDrop > 0)
                {
                  cMin = rMin;
                  DualFeasible = false;
                }
              }

              t2 = b_Ac[kNext - 1];
              b_Ac_0[0] = t2;
              cVal = b_Ac[kNext + 51];
              b_Ac_0[1] = cVal;
              b_Ac_tmp = b_Ac[kNext + 103];
              b_Ac_0[2] = b_Ac_tmp;
              b_Ac_tmp_0 = b_Ac[kNext + 155];
              b_Ac_0[3] = b_Ac_tmp_0;
              zTa = AutonomousSteeringSystem_mtimes(z, b_Ac_0);
              if (zTa <= 0.0)
              {
                t2 = 0.0;
                ColdReset = true;
              }
              else
              {
                t2 = (b[kNext - 1] - (((t2 * x[0] + cVal * x[1]) + b_Ac_tmp * x
                        [2]) + b_Ac_tmp_0 * x[3])) / zTa;
                ColdReset = false;
              }

              if (DualFeasible && ColdReset)
              {
                *status = -1.0;
                exitg1 = 1;
              }
              else
              {
                if (ColdReset)
                {
                  cVal = cMin;
                }
                else if (DualFeasible)
                {
                  cVal = t2;
                }
                else
                {
                  cVal = fmin(cMin, t2);
                }

                for (f_i = 1; f_i - 1 < nA; f_i++)
                {
                  i = iC[(int16_T)f_i - 1];
                  lambda[i - 1] -= r[(int16_T)f_i - 1] * cVal;
                  if ((i <= 52) && (lambda[i - 1] < 0.0))
                  {
                    lambda[i - 1] = 0.0;
                  }
                }

                lambda[kNext - 1] += cVal;
                if (cVal == cMin)
                {
                  AutonomousSteeri_DropConstraint(kDrop, iA, &nA, iC);
                }

                if (!ColdReset)
                {
                  x[0] += cVal * z[0];
                  x[1] += cVal * z[1];
                  x[2] += cVal * z[2];
                  x[3] += cVal * z[3];
                  if (cVal == t2)
                  {
                    if (nA == 4)
                    {
                      *status = -1.0;
                      exitg1 = 1;
                    }
                    else
                    {
                      f_i = nA + 1;
                      if (nA + 1 > 32767)
                      {
                        f_i = 32767;
                      }

                      nA = (int16_T)f_i;
                      iC[(int16_T)f_i - 1] = kNext;
                      kDrop = (int16_T)f_i;
                      exitg4 = false;
                      while ((!exitg4) && (kDrop > 1))
                      {
                        int16_T tmp_4;
                        int16_T tmp_5;
                        tmp_4 = iC[kDrop - 1];
                        tmp_5 = iC[kDrop - 2];
                        if (tmp_4 > tmp_5)
                        {
                          exitg4 = true;
                        }
                        else
                        {
                          iC[kDrop - 1] = tmp_5;
                          iC[kDrop - 2] = tmp_4;
                          kDrop--;
                        }
                      }

                      iA[kNext - 1] = 1;
                      kNext = 0;
                      (*status)++;
                    }
                  }
                  else
                  {
                    (*status)++;
                  }
                }
                else
                {
                  (*status)++;
                }
              }
            }
          }
          else
          {
            cMin = AutonomousSteeringSystem_norm(x);
            if (fabs(cMin - Xnorm0) > 0.001)
            {
              Xnorm0 = cMin;
              AutonomousSteeringSystem_abs_c(b, tmp_6);
              AutonomousSteeringSyst_maximum2(tmp_6, 1.0, cTol);
              cTolComputed = false;
            }

            exitg1 = 2;
          }
        }
        while (exitg1 == 0);

        if (exitg1 == 1)
        {
          exitg2 = true;
        }
      }
    }
  }
}

real_T rt_roundd_snf(real_T u)
{
  real_T y;
  if (fabs(u) < 4.503599627370496E+15)
  {
    if (u >= 0.5)
    {
      y = floor(u + 0.5);
    }
    else if (u > -0.5)
    {
      y = u * 0.0;
    }
    else
    {
      y = ceil(u - 0.5);
    }
  }
  else
  {
    y = u;
  }

  return y;
}

/* Model step function */
void AutonomousSteeringSystem_step(void)
{
  /* local block i/o variables */
  real_T rtb_FromWorkspace2;
  real_T rtb_FromWorkspace;
  if (rtmIsMajorTimeStep(AutonomousSteeringSystem_M))
  {
    /* set solver stop time */
    rtsiSetSolverStopTime(&AutonomousSteeringSystem_M->solverInfo,
                          ((AutonomousSteeringSystem_M->Timing.clockTick0+1)*
      AutonomousSteeringSystem_M->Timing.stepSize0));
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(AutonomousSteeringSystem_M))
  {
    AutonomousSteeringSystem_M->Timing.t[0] = rtsiGetT
      (&AutonomousSteeringSystem_M->solverInfo);
  }

  {
    real_T a__1[52];
    real_T rseq[20];
    real_T xk[5];
    real_T f[4];
    real_T zopt[4];
    int32_T b_i;
    int16_T iAnew[52];
    static const real_T b_a[10] =
    {
      -0.2439986816695123, 0.14060869226058134, 0.21411690183415727,
      0.506310455215062, -0.0357934370056687, -0.14569818355099967,
      0.068357225516489428, 0.2066640661471332, 0.10481525525491292,
      0.085697711904000662
    };

    real_T b_Mlim[52];
    real_T rtb_xest[5];
    real_T LateralpositionYawangle[2];
    real_T rtb_TmpSignalConversionAtSFunct[2];
    real_T b_Kx;
    int32_T i;
    static const int8_T a[10] =
    {
      0, 0, 0, 1, 0, 0, 1, 0, 0, 1
    };

    static const real_T b_a_0[10] =
    {
      -0.2439986816695123, 0.14060869226058134, 0.21411690183415727,
      0.506310455215062, -0.0357934370056687, -0.14569818355099967,
      0.068357225516489428, 0.2066640661471332, 0.10481525525491292,
      0.085697711904000662
    };

    static const real_T b_Kx_0[15] =
    {
      184.08676582580412, 5226.2460492765185, 427.17993327902514,
      435.93480286522964, 0.94062627644071362, 136.74845550067323,
      3890.3325223428737, 320.80137070326816, 315.35133892065562,
      0.75205013042299185, 97.894146367113066, 2789.90643050548,
      232.10717791822987, 220.00071888654617, 0.58418047485163016
    };

    static const real_T b_Linv[16] =
    {
      0.015252916191996656, -0.24693397368915118, 1.3400370210132624, 0.0, 0.0,
      0.32562318175709576, -3.3962091987977545, 0.0, 0.0, 0.0,
      2.2268835830608316, 0.0, 0.0, 0.0, 0.0, 0.003162277660168379
    };

    static const real_T b_Hinv[16] =
    {
      1.8569082565003732, -4.6314534836913666, 2.9841064427881765, 0.0,
      -4.6314534836913666, 11.640267378496102, -7.5629625094428992, 0.0,
      2.9841064427881765, -7.5629625094428992, 4.9590104925058478, 0.0, 0.0, 0.0,
      0.0, 9.9999999999999974E-6
    };

    static const real_T b_Ac[208] =
    {
      -0.11400709822746803, -0.070741975047093675, -0.45483902928779751,
      -0.24835661419950755, -1.0528441622800591, -0.48687481226947765,
      -1.9518731003560692, -0.75345419653256818, -3.1899925509786087,
      -1.028492378171425, -4.7935381665225441, -1.3023613046684088,
      -6.7777668235418593, -1.5716625995763058, -9.1498755631424853,
      -1.8362351880834913, -11.9121731941277, -2.0971991692233161,
      -15.064517738235741, -2.3558857937584841, 0.11400709822746803,
      0.070741975047093675, 0.45483902928779751, 0.24835661419950755,
      1.0528441622800591, 0.48687481226947765, 1.9518731003560692,
      0.75345419653256818, 3.1899925509786087, 1.028492378171425,
      4.7935381665225441, 1.3023613046684088, 6.7777668235418593,
      1.5716625995763058, 9.1498755631424853, 1.8362351880834913,
      11.9121731941277, 2.0971991692233161, 15.064517738235741,
      2.3558857937584841, -1.0, -1.0, -1.0, 1.0, 1.0, 1.0, -1.0, -0.0, -0.0, 1.0,
      0.0, 0.0, -0.0, -0.0, -0.11400709822746803, -0.070741975047093675,
      -0.45483902928779751, -0.24835661419950755, -1.0528441622800591,
      -0.48687481226947765, -1.9518731003560692, -0.75345419653256818,
      -3.1899925509786087, -1.028492378171425, -4.7935381665225441,
      -1.3023613046684088, -6.7777668235418593, -1.5716625995763058,
      -9.1498755631424853, -1.8362351880834913, -11.9121731941277,
      -2.0971991692233161, 0.0, 0.0, 0.11400709822746803, 0.070741975047093675,
      0.45483902928779751, 0.24835661419950755, 1.0528441622800591,
      0.48687481226947765, 1.9518731003560692, 0.75345419653256818,
      3.1899925509786087, 1.028492378171425, 4.7935381665225441,
      1.3023613046684088, 6.7777668235418593, 1.5716625995763058,
      9.1498755631424853, 1.8362351880834913, 11.9121731941277,
      2.0971991692233161, -0.0, -1.0, -1.0, 0.0, 1.0, 1.0, -0.0, -1.0, -0.0, 0.0,
      1.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.11400709822746803,
      -0.070741975047093675, -0.45483902928779751, -0.24835661419950755,
      -1.0528441622800591, -0.48687481226947765, -1.9518731003560692,
      -0.75345419653256818, -3.1899925509786087, -1.028492378171425,
      -4.7935381665225441, -1.3023613046684088, -6.7777668235418593,
      -1.5716625995763058, -9.1498755631424853, -1.8362351880834913, 0.0, 0.0,
      0.0, 0.0, 0.11400709822746803, 0.070741975047093675, 0.45483902928779751,
      0.24835661419950755, 1.0528441622800591, 0.48687481226947765,
      1.9518731003560692, 0.75345419653256818, 3.1899925509786087,
      1.028492378171425, 4.7935381665225441, 1.3023613046684088,
      6.7777668235418593, 1.5716625995763058, 9.1498755631424853,
      1.8362351880834913, -0.0, -0.0, -1.0, 0.0, 0.0, 1.0, -0.0, -0.0, -1.0, 0.0,
      0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
      1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
      1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    };

    static const real_T b_Ku1[3] =
    {
      4298.2740885724234, 3259.5658683269144, 2384.6417240814585
    };

    static const real_T b_Kr_0[60] =
    {
      -0.91256627377089872, -0.0056625194020038967, -3.6407448709429038,
      -0.01987962798019851, -8.427458368569118, -0.038971742999649076,
      -15.623707556451402, -0.060310006944910781, -25.534196211145197,
      -0.082325352696639029, -38.369727243422844, -0.10424710578400723,
      -54.252423847389949, -0.12580324422070738, -73.239894514853859,
      -0.14698087482351427, -95.350620034109468, -0.16786965557136174,
      -120.583463944574, -0.1885761460177218, -0.0, -0.0, -0.91256627377089872,
      -0.0056625194020038967, -3.6407448709429038, -0.01987962798019851,
      -8.427458368569118, -0.038971742999649076, -15.623707556451402,
      -0.060310006944910781, -25.534196211145197, -0.082325352696639029,
      -38.369727243422844, -0.10424710578400723, -54.252423847389949,
      -0.12580324422070738, -73.239894514853859, -0.14698087482351427,
      -95.350620034109468, -0.16786965557136174, -0.0, -0.0, -0.0, -0.0,
      -0.91256627377089872, -0.0056625194020038967, -3.6407448709429038,
      -0.01987962798019851, -8.427458368569118, -0.038971742999649076,
      -15.623707556451402, -0.060310006944910781, -25.534196211145197,
      -0.082325352696639029, -38.369727243422844, -0.10424710578400723,
      -54.252423847389949, -0.12580324422070738, -73.239894514853859,
      -0.14698087482351427
    };

    static const real_T b_Mlim_0[52] =
    {
      6.0, 0.2, 6.0, 0.2, 6.0, 0.2, 6.0, 0.2, 6.0, 0.2, 6.0, 0.2, 6.0, 0.2, 6.0,
      0.2, 6.0, 0.2, 6.0, 0.2, 2.0, 0.2, 2.0, 0.2, 2.0, 0.2, 2.0, 0.2, 2.0, 0.2,
      2.0, 0.2, 2.0, 0.2, 2.0, 0.2, 2.0, 0.2, 2.0, 0.2, 0.523598775598299,
      0.523598775598299, 0.523598775598299, 0.523598775598299, 0.523598775598299,
      0.523598775598299, 0.261799387799149, 0.261799387799149, 0.261799387799149,
      0.261799387799149, 0.261799387799149, 0.261799387799149
    };

    static const real_T d_a[52] =
    {
      -0.11400709822746803, -0.070741975047093675, -0.45483902928779751,
      -0.24835661419950755, -1.0528441622800591, -0.48687481226947765,
      -1.9518731003560692, -0.75345419653256818, -3.1899925509786087,
      -1.028492378171425, -4.7935381665225441, -1.3023613046684088,
      -6.7777668235418593, -1.5716625995763058, -9.1498755631424853,
      -1.8362351880834913, -11.9121731941277, -2.0971991692233161,
      -15.064517738235741, -2.3558857937584841, 0.11400709822746803,
      0.070741975047093675, 0.45483902928779751, 0.24835661419950755,
      1.0528441622800591, 0.48687481226947765, 1.9518731003560692,
      0.75345419653256818, 3.1899925509786087, 1.028492378171425,
      4.7935381665225441, 1.3023613046684088, 6.7777668235418593,
      1.5716625995763058, 9.1498755631424853, 1.8362351880834913,
      11.9121731941277, 2.0971991692233161, 15.064517738235741,
      2.3558857937584841, -1.0, -1.0, -1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0
    };

    static const real_T c_a[260] =
    {
      -0.081595384470467269, -0.0050186685703353341, -0.13877864635183068,
      -0.014344738164658106, -0.18529748165309923, -0.022991055476267942,
      -0.22908698395518834, -0.029215843983904602, -0.27387095890272906,
      -0.032956659919392062, -0.32082826108487855, -0.034810384796619506,
      -0.36986758889077959, -0.035477453355279479, -0.42043358569917355,
      -0.035522935055108035, -0.47193181402205925, -0.035318784011032167,
      -0.52390142326696321, -0.035069967512791116, 0.081595384470467269,
      0.0050186685703353341, 0.13877864635183068, 0.014344738164658106,
      0.18529748165309923, 0.022991055476267942, 0.22908698395518834,
      0.029215843983904602, 0.27387095890272906, 0.032956659919392062,
      0.32082826108487855, 0.034810384796619506, 0.36986758889077959,
      0.035477453355279479, 0.42043358569917355, 0.035522935055108035,
      0.47193181402205925, 0.035318784011032167, 0.52390142326696321,
      0.035069967512791116, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, -1.5, -1.0, -3.0, -1.0, -4.5, -1.0, -6.0, -1.0, -7.5, -1.0, -9.0,
      -1.0, -10.5, -1.0, -12.0, -1.0, -13.5, -1.0, -15.0, -1.0, 1.5, 1.0, 3.0,
      1.0, 4.5, 1.0, 6.0, 1.0, 7.5, 1.0, 9.0, 1.0, 10.5, 1.0, 12.0, 1.0, 13.5,
      1.0, 15.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      -0.01780173334859092, -0.076034046986390588, -0.0803610893736259,
      -0.11356592359788709, -0.18552610703571754, -0.12695890577302291,
      -0.32182350048566311, -0.12775171340119498, -0.47707446016086008,
      -0.12351649937050892, -0.64187357689931079, -0.11841245704293102,
      -0.81022922294958821, -0.11425102639531211, -0.97895581789913932,
      -0.11149097907783881, -1.1467395599191053, -0.10995792394788988,
      -1.3133124913993222, -0.10927835250265655, 0.01780173334859092,
      0.076034046986390588, 0.0803610893736259, 0.11356592359788709,
      0.18552610703571754, 0.12695890577302291, 0.32182350048566311,
      0.12775171340119498, 0.47707446016086008, 0.12351649937050892,
      0.64187357689931079, 0.11841245704293102, 0.81022922294958821,
      0.11425102639531211, 0.97895581789913932, 0.11149097907783881,
      1.1467395599191053, 0.10995792394788988, 1.3133124913993222,
      0.10927835250265655, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, -1.0, -0.0, -1.0, -0.0, -1.0, -0.0, -1.0, -0.0, -1.0, -0.0, -1.0,
      -0.0, -1.0, -0.0, -1.0, -0.0, -1.0, -0.0, -1.0, -0.0, 1.0, 0.0, 1.0, 0.0,
      1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
      -1.0, -0.0, -1.0, -0.0, -1.0, -0.0, -1.0, -0.0, -1.0, -0.0, -1.0, -0.0,
      -1.0, -0.0, -1.0, -0.0, -1.0, -0.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0,
      0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    };

    static const real_T f_a[5] =
    {
      1.1898718909245078, 0.070741975047093675, 1.3270514387018264,
      0.11400709822746803, 0.0
    };

    static const real_T g_a[10] =
    {
      -0.30452232652087319, 0.15566431832032193, 0.095864384073663142,
      0.70112597935670062, -0.035793437005668761, -0.24090974815303118,
      0.083339519937776677, 0.10004393181656993, 0.19914177282444112,
      0.085697711904000634
    };

    static const real_T e_a[25] =
    {
      0.59029522013726732, 0.0050186685703353341, 0.083693736924718931,
      0.081595384470467269, 0.0, 0.0, 1.0, 0.0, 1.5, 0.0, -0.74954881953164743,
      0.076034046986390588, 0.54309372380774523, 0.01780173334859092, 0.0, 0.0,
      0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0
    };

    /* StateSpace: '<Root>/Plant' */
    LateralpositionYawangle[0] = 0.0;
    LateralpositionYawangle[1] = 0.0;
    LateralpositionYawangle[AutonomousSteeringSystem_ConstP.Plant_C_ir[0U]] +=
      AutonomousSteeringSystem_X.Plant_CSTATE[1U];
    LateralpositionYawangle[AutonomousSteeringSystem_ConstP.Plant_C_ir[1U]] +=
      AutonomousSteeringSystem_X.Plant_CSTATE[3U];
    if (rtmIsMajorTimeStep(AutonomousSteeringSystem_M))
    {
      /* FromWorkspace: '<S3>/From Workspace2' */
      {
        real_T *pDataValues = (real_T *)
          AutonomousSteeringSystem_DW.FromWorkspace2_PWORK.DataPtr;
        real_T *pTimeValues = (real_T *)
          AutonomousSteeringSystem_DW.FromWorkspace2_PWORK.TimePtr;
        int_T currTimeIndex =
          AutonomousSteeringSystem_DW.FromWorkspace2_IWORK.PrevIndex;
        real_T t = ((AutonomousSteeringSystem_M->Timing.clockTick1) * 0.1);

        /* Get index */
        if (t <= pTimeValues[0])
        {
          currTimeIndex = 0;
        }
        else if (t >= pTimeValues[82])
        {
          currTimeIndex = 81;
        }
        else
        {
          if (t < pTimeValues[currTimeIndex])
          {
            while (t < pTimeValues[currTimeIndex])
            {
              currTimeIndex--;
            }
          }
          else
          {
            while (t >= pTimeValues[currTimeIndex + 1])
            {
              currTimeIndex++;
            }
          }
        }

        AutonomousSteeringSystem_DW.FromWorkspace2_IWORK.PrevIndex =
          currTimeIndex;

        /* Post output */
        {
          real_T t1 = pTimeValues[currTimeIndex];
          real_T t2 = pTimeValues[currTimeIndex + 1];
          if (t1 == t2)
          {
            if (t < t1)
            {
              rtb_FromWorkspace2 = pDataValues[currTimeIndex];
            }
            else
            {
              rtb_FromWorkspace2 = pDataValues[currTimeIndex + 1];
            }
          }
          else
          {
            real_T f1 = (t2 - t) / (t2 - t1);
            real_T f2 = 1.0 - f1;
            real_T d1;
            real_T d2;
            int_T TimeIndex = currTimeIndex;
            d1 = pDataValues[TimeIndex];
            d2 = pDataValues[TimeIndex + 1];
            rtb_FromWorkspace2 = (real_T) rtInterpolate(d1, d2, f1, f2);
            pDataValues += 83;
          }
        }
      }

      /* FromWorkspace: '<S3>/From Workspace' */
      {
        real_T *pDataValues = (real_T *)
          AutonomousSteeringSystem_DW.FromWorkspace_PWORK.DataPtr;
        real_T *pTimeValues = (real_T *)
          AutonomousSteeringSystem_DW.FromWorkspace_PWORK.TimePtr;
        int_T currTimeIndex =
          AutonomousSteeringSystem_DW.FromWorkspace_IWORK.PrevIndex;
        real_T t = ((AutonomousSteeringSystem_M->Timing.clockTick1) * 0.1);

        /* Get index */
        if (t <= pTimeValues[0])
        {
          currTimeIndex = 0;
        }
        else if (t >= pTimeValues[82])
        {
          currTimeIndex = 81;
        }
        else
        {
          if (t < pTimeValues[currTimeIndex])
          {
            while (t < pTimeValues[currTimeIndex])
            {
              currTimeIndex--;
            }
          }
          else
          {
            while (t >= pTimeValues[currTimeIndex + 1])
            {
              currTimeIndex++;
            }
          }
        }

        AutonomousSteeringSystem_DW.FromWorkspace_IWORK.PrevIndex =
          currTimeIndex;

        /* Post output */
        {
          real_T t1 = pTimeValues[currTimeIndex];
          real_T t2 = pTimeValues[currTimeIndex + 1];
          if (t1 == t2)
          {
            if (t < t1)
            {
              rtb_FromWorkspace = pDataValues[currTimeIndex];
            }
            else
            {
              rtb_FromWorkspace = pDataValues[currTimeIndex + 1];
            }
          }
          else
          {
            real_T f1 = (t2 - t) / (t2 - t1);
            real_T f2 = 1.0 - f1;
            real_T d1;
            real_T d2;
            int_T TimeIndex = currTimeIndex;
            d1 = pDataValues[TimeIndex];
            d2 = pDataValues[TimeIndex + 1];
            rtb_FromWorkspace = (real_T) rtInterpolate(d1, d2, f1, f2);
            pDataValues += 83;
          }
        }
      }

      /* MATLAB Function: '<S24>/optimizer' incorporates:
       *  SignalConversion generated from: '<S25>/ SFunction '
       */
      for (b_i = 0; b_i < 10; b_i++)
      {
        i = b_i << 1;
        rseq[i] = rtb_FromWorkspace2;
        rseq[i + 1] = rtb_FromWorkspace;
      }

      for (i = 0; i < 5; i++)
      {
        xk[i] = AutonomousSteeringSystem_DW.last_x_PreviousInput[i];
      }

      for (b_i = 0; b_i < 2; b_i++)
      {
        b_Kx = 0.0;
        for (i = 0; i < 5; i++)
        {
          b_Kx += (real_T)a[(i << 1) + b_i] * xk[i];
        }

        rtb_TmpSignalConversionAtSFunct[b_i] = LateralpositionYawangle[b_i] -
          b_Kx;
      }

      for (b_i = 0; b_i <= 2; b_i += 2)
      {
        __m128d tmp;

        /* MATLAB Function: '<S24>/optimizer' */
        tmp = _mm_loadu_pd(&xk[b_i]);
        _mm_storeu_pd(&rtb_xest[b_i], _mm_add_pd(_mm_add_pd(_mm_mul_pd
          (_mm_loadu_pd(&b_a[b_i + 5]), _mm_set1_pd
           (rtb_TmpSignalConversionAtSFunct[1])), _mm_mul_pd(_mm_loadu_pd
          (&b_a[b_i]), _mm_set1_pd(rtb_TmpSignalConversionAtSFunct[0]))), tmp));
      }

      /* MATLAB Function: '<S24>/optimizer' incorporates:
       *  UnitDelay: '<S4>/last_mv'
       */
      for (b_i = 4; b_i < 5; b_i++)
      {
        rtb_xest[b_i] = (b_a_0[b_i + 5] * rtb_TmpSignalConversionAtSFunct[1] +
                         b_a_0[b_i] * rtb_TmpSignalConversionAtSFunct[0]) +
          xk[b_i];
      }

      f[0] = 0.0;
      f[1] = 0.0;
      f[2] = 0.0;
      f[3] = 0.0;
      for (i = 0; i < 3; i++)
      {
        real_T b_Kr;
        b_Kx = 0.0;
        for (b_i = 0; b_i < 5; b_i++)
        {
          b_Kx += b_Kx_0[5 * i + b_i] * rtb_xest[b_i];
        }

        b_Kr = 0.0;
        for (b_i = 0; b_i < 20; b_i++)
        {
          b_Kr += b_Kr_0[20 * i + b_i] * rseq[b_i];
        }

        f[i] = (b_Kx + b_Kr) + b_Ku1[i] *
          AutonomousSteeringSystem_DW.last_mv_DSTATE;
      }

      for (i = 0; i < 52; i++)
      {
        iAnew[i] = AutonomousSteeringSystem_DW.Memory_PreviousInput[i];
        b_Kx = 0.0;
        for (b_i = 0; b_i < 5; b_i++)
        {
          b_Kx += c_a[52 * b_i + i] * rtb_xest[b_i];
        }

        b_Mlim[i] = -((b_Mlim_0[i] + b_Kx) + d_a[i] *
                      AutonomousSteeringSystem_DW.last_mv_DSTATE);
      }

      AutonomousSteeringSystem_qpkwik(b_Linv, b_Hinv, f, b_Ac, b_Mlim, iAnew,
        224, 1.0E-6, zopt, a__1, &b_Kx);
      for (i = 0; i < 52; i++)
      {
        AutonomousSteeringSystem_B.iAout[i] = (iAnew[i] != 0);
      }

      b_Kx = rt_roundd_snf(b_Kx);
      if (b_Kx < 2.147483648E+9)
      {
        if (b_Kx >= -2.147483648E+9)
        {
          b_i = (int32_T)b_Kx;
        }
        else
        {
          b_i = MIN_int32_T;
        }

        if (b_Kx >= -2.147483648E+9)
        {
          i = (int32_T)b_Kx;
        }
        else
        {
          i = MIN_int32_T;
        }
      }
      else
      {
        b_i = MAX_int32_T;
        i = MAX_int32_T;
      }

      if ((b_i < 0) || (i == 0))
      {
        zopt[0] = 0.0;
      }

      AutonomousSteeringSystem_B.u = AutonomousSteeringSystem_DW.last_mv_DSTATE
        + zopt[0];
      for (b_i = 0; b_i < 5; b_i++)
      {
        b_Kx = 0.0;
        for (i = 0; i < 5; i++)
        {
          b_Kx += e_a[5 * i + b_i] * xk[i];
        }

        AutonomousSteeringSystem_B.xk1[b_i] = (g_a[b_i + 5] *
          rtb_TmpSignalConversionAtSFunct[1] + g_a[b_i] *
          rtb_TmpSignalConversionAtSFunct[0]) + (f_a[b_i] *
          AutonomousSteeringSystem_B.u + b_Kx);
      }
    }
  }

  if (rtmIsMajorTimeStep(AutonomousSteeringSystem_M))
  {
    int32_T i;
    if (rtmIsMajorTimeStep(AutonomousSteeringSystem_M))
    {
      /* Update for Memory: '<S4>/Memory' */
      for (i = 0; i < 52; i++)
      {
        AutonomousSteeringSystem_DW.Memory_PreviousInput[i] =
          AutonomousSteeringSystem_B.iAout[i];
      }

      /* End of Update for Memory: '<S4>/Memory' */

      /* Update for UnitDelay: '<S4>/last_mv' */
      AutonomousSteeringSystem_DW.last_mv_DSTATE = AutonomousSteeringSystem_B.u;

      /* Update for Memory: '<S4>/last_x' */
      for (i = 0; i < 5; i++)
      {
        AutonomousSteeringSystem_DW.last_x_PreviousInput[i] =
          AutonomousSteeringSystem_B.xk1[i];
      }

      /* End of Update for Memory: '<S4>/last_x' */
    }
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(AutonomousSteeringSystem_M))
  {
    rt_ertODEUpdateContinuousStates(&AutonomousSteeringSystem_M->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     */
    ++AutonomousSteeringSystem_M->Timing.clockTick0;
    AutonomousSteeringSystem_M->Timing.t[0] = rtsiGetSolverStopTime
      (&AutonomousSteeringSystem_M->solverInfo);

    {
      /* Update absolute timer for sample time: [0.1s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.1, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       */
      AutonomousSteeringSystem_M->Timing.clockTick1++;
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void AutonomousSteeringSystem_derivatives(void)
{
  XDot_AutonomousSteeringSystem_T *_rtXdot;
  int32_T ri;
  uint32_T Plant_CSTATE_tmp;
  _rtXdot = ((XDot_AutonomousSteeringSystem_T *)
             AutonomousSteeringSystem_M->derivs);

  /* Derivatives for StateSpace: '<Root>/Plant' */
  _rtXdot->Plant_CSTATE[0] = 0.0;
  _rtXdot->Plant_CSTATE[1] = 0.0;
  _rtXdot->Plant_CSTATE[2] = 0.0;
  _rtXdot->Plant_CSTATE[3] = 0.0;
  for (ri = 0; (uint32_T)ri < 3U; ri++)
  {
    Plant_CSTATE_tmp = AutonomousSteeringSystem_ConstP.Plant_A_ir[(uint32_T)ri];
    _rtXdot->Plant_CSTATE[Plant_CSTATE_tmp] +=
      AutonomousSteeringSystem_ConstP.Plant_A_pr[(uint32_T)ri] *
      AutonomousSteeringSystem_X.Plant_CSTATE[0U];
  }

  _rtXdot->Plant_CSTATE[AutonomousSteeringSystem_ConstP.Plant_A_ir[3U]] +=
    AutonomousSteeringSystem_X.Plant_CSTATE[1U] *
    AutonomousSteeringSystem_ConstP.Plant_A_pr[3U];
  for (ri = 4; (uint32_T)ri < 7U; ri++)
  {
    Plant_CSTATE_tmp = AutonomousSteeringSystem_ConstP.Plant_A_ir[(uint32_T)ri];
    _rtXdot->Plant_CSTATE[Plant_CSTATE_tmp] +=
      AutonomousSteeringSystem_ConstP.Plant_A_pr[(uint32_T)ri] *
      AutonomousSteeringSystem_X.Plant_CSTATE[2U];
  }

  for (ri = 0; (uint32_T)ri < 2U; ri++)
  {
    Plant_CSTATE_tmp = AutonomousSteeringSystem_ConstP.pooled10[(uint32_T)ri];
    _rtXdot->Plant_CSTATE[Plant_CSTATE_tmp] +=
      AutonomousSteeringSystem_ConstP.Plant_B_pr[(uint32_T)ri] *
      AutonomousSteeringSystem_B.u;
  }

  /* End of Derivatives for StateSpace: '<Root>/Plant' */
}

/* Model initialize function */
void AutonomousSteeringSystem_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&AutonomousSteeringSystem_M->solverInfo,
                          &AutonomousSteeringSystem_M->Timing.simTimeStep);
    rtsiSetTPtr(&AutonomousSteeringSystem_M->solverInfo, &rtmGetTPtr
                (AutonomousSteeringSystem_M));
    rtsiSetStepSizePtr(&AutonomousSteeringSystem_M->solverInfo,
                       &AutonomousSteeringSystem_M->Timing.stepSize0);
    rtsiSetdXPtr(&AutonomousSteeringSystem_M->solverInfo,
                 &AutonomousSteeringSystem_M->derivs);
    rtsiSetContStatesPtr(&AutonomousSteeringSystem_M->solverInfo, (real_T **)
                         &AutonomousSteeringSystem_M->contStates);
    rtsiSetNumContStatesPtr(&AutonomousSteeringSystem_M->solverInfo,
      &AutonomousSteeringSystem_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&AutonomousSteeringSystem_M->solverInfo,
      &AutonomousSteeringSystem_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&AutonomousSteeringSystem_M->solverInfo,
      &AutonomousSteeringSystem_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&AutonomousSteeringSystem_M->solverInfo,
      &AutonomousSteeringSystem_M->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&AutonomousSteeringSystem_M->solverInfo,
                          (&rtmGetErrorStatus(AutonomousSteeringSystem_M)));
    rtsiSetRTModelPtr(&AutonomousSteeringSystem_M->solverInfo,
                      AutonomousSteeringSystem_M);
  }

  rtsiSetSimTimeStep(&AutonomousSteeringSystem_M->solverInfo, MAJOR_TIME_STEP);
  AutonomousSteeringSystem_M->intgData.y = AutonomousSteeringSystem_M->odeY;
  AutonomousSteeringSystem_M->intgData.f[0] = AutonomousSteeringSystem_M->odeF[0];
  AutonomousSteeringSystem_M->intgData.f[1] = AutonomousSteeringSystem_M->odeF[1];
  AutonomousSteeringSystem_M->intgData.f[2] = AutonomousSteeringSystem_M->odeF[2];
  AutonomousSteeringSystem_M->contStates = ((X_AutonomousSteeringSystem_T *)
    &AutonomousSteeringSystem_X);
  rtsiSetSolverData(&AutonomousSteeringSystem_M->solverInfo, (void *)
                    &AutonomousSteeringSystem_M->intgData);
  rtsiSetSolverName(&AutonomousSteeringSystem_M->solverInfo,"ode3");
  rtmSetTPtr(AutonomousSteeringSystem_M,
             &AutonomousSteeringSystem_M->Timing.tArray[0]);
  AutonomousSteeringSystem_M->Timing.stepSize0 = 0.1;

  /* Start for FromWorkspace: '<S3>/From Workspace2' */
  {
    static real_T pTimeValues0[] =
    {
      0.0, 0.099999999999999992, 0.20000000000000004, 0.3000000000000001,
      0.40000000000000019, 0.50000000000000022, 0.60000000000000031,
      0.7000000000000004, 0.80000000000000049, 0.90000000000000058,
      1.0000000000000007, 1.1000000000000008, 1.2000000000000008,
      1.3000000000000009, 1.400000000000001, 1.5000000000000011,
      1.6000000000000012, 1.7000000000000013, 1.8000000000000014,
      1.9000000000000015, 2.0000000000000013, 2.0999999999999992,
      2.1999999999999971, 2.2999999999999949, 2.3999999999999928,
      2.4999999999999907, 2.5999999999999885, 2.6999999999999864,
      2.7999999999999843, 2.8999999999999821, 2.99999999999998,
      3.0999999999999779, 3.1999999999999758, 3.2999999999999736,
      3.3999999999999715, 3.4999999999999694, 3.5999999999999672,
      3.6999999999999651, 3.799999999999963, 3.8999999999999608,
      3.9999999999999587, 4.099999999999957, 4.1999999999999549,
      4.2999999999999527, 4.3999999999999506, 4.4999999999999485,
      4.5999999999999464, 4.6999999999999442, 4.7999999999999421,
      4.89999999999994, 4.9999999999999378, 5.0999999999999357,
      5.1999999999999336, 5.2999999999999314, 5.3999999999999293,
      5.4999999999999272, 5.599999999999925, 5.6999999999999229,
      5.7999999999999208, 5.8999999999999186, 5.9999999999999165,
      6.0999999999999144, 6.1999999999999122, 6.29999999999991,
      6.399999999999908, 6.4999999999999059, 6.5999999999999037,
      6.6999999999999016, 6.7999999999998995, 6.8999999999998973,
      6.9999999999998952, 7.0999999999998931, 7.1999999999998909,
      7.2999999999998888, 7.3999999999998867, 7.4999999999998845,
      7.5999999999998824, 7.69999999999988, 7.7999999999998781,
      7.899999999999876, 7.9999999999998739, 8.0999999999998717,
      8.19999999999987
    } ;

    static real_T pDataValues0[] =
    {
      0.0, -0.00032057688834367633, 0.0001046627707353404, 0.0019633028915715441,
      0.0059429239765455251, 0.012731095719916501, 0.023015362926791384,
      0.037483222580328462, 0.05661776191230368, 0.0807832726616669,
      0.11070949504224792, 0.14713214905782759, 0.19078661801418004,
      0.24240782912164788, 0.3027301079820538, 0.37248700431913734,
      0.45241108632067739, 0.54322828982636229, 0.64482692871996372,
      0.75577957443767907, 0.87455822436566311, 0.99963648279861006,
      1.1294893996997795, 1.2625932112379594, 1.3974250059040985,
      1.5324623399719128, 1.6662786950018476, 1.7984397038989934,
      1.9288182087711181, 2.0572458498503861, 2.1835542137146304,
      2.3075748302864034, 2.429139170234988, 2.5480786428134428,
      2.6642245941628082, 2.7774083061155972, 2.8874369946783749,
      2.9939643927789104, 3.096584841736945, 3.1948924196190918,
      3.2884810058016432, 3.3769442815837016, 3.4598757359962891,
      3.5368686772556726, 3.6075162503092817, 3.671411460922684,
      3.7281578592117861, 3.7777688127128712, 3.8207531578250689,
      3.8576478218748256, 3.88899000870342, 3.9153171356111991,
      3.937166781005621, 3.9550766417191987, 3.9695844989622167,
      3.9812281918741537, 3.9905455976371416, 3.9980746171143191,
      4.0042226492204165, 4.0089633882717308, 4.0124108908590994,
      4.0146952649335228, 4.0159466197411575, 4.0162950653504659,
      4.01587071229594, 4.01480367132374, 4.013224053224536, 4.0112619687388946,
      4.0090475285205036, 4.0067108431425709, 4.0043820231326963,
      4.0021911790215441, 4.0002684213906274, 3.9987605666119856,
      3.9977214126735334, 3.9970952314177151, 3.9968249528974638,
      3.9968535071325086, 3.9971238241271045, 3.997578833881275,
      3.9981614663968053, 3.9988146516792265, 3.9994813197370265
    } ;

    AutonomousSteeringSystem_DW.FromWorkspace2_PWORK.TimePtr = (void *)
      pTimeValues0;
    AutonomousSteeringSystem_DW.FromWorkspace2_PWORK.DataPtr = (void *)
      pDataValues0;
    AutonomousSteeringSystem_DW.FromWorkspace2_IWORK.PrevIndex = 0;
  }

  /* Start for FromWorkspace: '<S3>/From Workspace' */
  {
    static real_T pTimeValues0[] =
    {
      0.0, 0.099999999999999992, 0.20000000000000004, 0.3000000000000001,
      0.40000000000000019, 0.50000000000000022, 0.60000000000000031,
      0.7000000000000004, 0.80000000000000049, 0.90000000000000058,
      1.0000000000000007, 1.1000000000000008, 1.2000000000000008,
      1.3000000000000009, 1.400000000000001, 1.5000000000000011,
      1.6000000000000012, 1.7000000000000013, 1.8000000000000014,
      1.9000000000000015, 2.0000000000000013, 2.0999999999999992,
      2.1999999999999971, 2.2999999999999949, 2.3999999999999928,
      2.4999999999999907, 2.5999999999999885, 2.6999999999999864,
      2.7999999999999843, 2.8999999999999821, 2.99999999999998,
      3.0999999999999779, 3.1999999999999758, 3.2999999999999736,
      3.3999999999999715, 3.4999999999999694, 3.5999999999999672,
      3.6999999999999651, 3.799999999999963, 3.8999999999999608,
      3.9999999999999587, 4.099999999999957, 4.1999999999999549,
      4.2999999999999527, 4.3999999999999506, 4.4999999999999485,
      4.5999999999999464, 4.6999999999999442, 4.7999999999999421,
      4.89999999999994, 4.9999999999999378, 5.0999999999999357,
      5.1999999999999336, 5.2999999999999314, 5.3999999999999293,
      5.4999999999999272, 5.599999999999925, 5.6999999999999229,
      5.7999999999999208, 5.8999999999999186, 5.9999999999999165,
      6.0999999999999144, 6.1999999999999122, 6.29999999999991,
      6.399999999999908, 6.4999999999999059, 6.5999999999999037,
      6.6999999999999016, 6.7999999999998995, 6.8999999999998973,
      6.9999999999998952, 7.0999999999998931, 7.1999999999998909,
      7.2999999999998888, 7.3999999999998867, 7.4999999999998845,
      7.5999999999998824, 7.69999999999988, 7.7999999999998781,
      7.899999999999876, 7.9999999999998739, 8.0999999999998717,
      8.19999999999987
    } ;

    static real_T pDataValues0[] =
    {
      -0.00030952690202518059, -4.151067963970013E-5, 0.00068489518652422069,
      0.0018696906964665828, 0.0035128758501873857, 0.0056144506476866283,
      0.0081744150889643142, 0.011192769174020437, 0.014354132388172543,
      0.017949779650472508, 0.022036345223960224, 0.026613829108635703,
      0.031682231304498931, 0.037241551811549943, 0.043291790629788683,
      0.0498329477592152, 0.05686502319982948, 0.064318929307449133,
      0.071080137287238609, 0.076822326042590566, 0.081545495573505045,
      0.085249645879981975, 0.087934776962021469, 0.089600888819623539,
      0.090247981452788131, 0.089876054861515314, 0.088780684911812929,
      0.087644037886527443, 0.086394861008402449, 0.0850331542774379,
      0.083558917693633841, 0.081972151256990242, 0.080272854967507093,
      0.078461028825184409, 0.076536672830022162, 0.074499786982020422,
      0.072293364619359146, 0.069817747994082038, 0.067071398662752954,
      0.064054316625371963, 0.060766501881939064, 0.05720795443245421,
      0.053378674276917448, 0.049278661415328745, 0.044907915847688121,
      0.040266437573995568, 0.035409499000060636, 0.03081033302075719,
      0.02656978696918616, 0.022687860845347539, 0.019164554649241331,
      0.015999868380867529, 0.013193802040226138, 0.010746355627317163,
      0.0086575291421406, 0.0069273225846964437, 0.0055557359549847016,
      0.0045427692530053687, 0.0036204920164739636, 0.0027149608664335566,
      0.0018961701673482294, 0.0011641199192179809, 0.00051881012204281094,
      -3.9759224177281227E-5, -0.00051158811944229394, -0.00089667656375222785,
      -0.0011950245571070834, -0.0014066320995068605, -0.0015314991909515581,
      -0.001569625831441177, -0.0015210120209757179, -0.00138565775955518,
      -0.0011635630471795643, -0.00084310915478588082, -0.00054877065633727388,
      -0.00029247882197826815, -7.4233651708864126E-5, 0.00010596485447093872,
      0.00024811669656113955, 0.00035222187456173949, 0.00041828038847273774,
      0.00044629223829413425, 0.00043625742402592945
    } ;

    AutonomousSteeringSystem_DW.FromWorkspace_PWORK.TimePtr = (void *)
      pTimeValues0;
    AutonomousSteeringSystem_DW.FromWorkspace_PWORK.DataPtr = (void *)
      pDataValues0;
    AutonomousSteeringSystem_DW.FromWorkspace_IWORK.PrevIndex = 0;
  }

  /* InitializeConditions for StateSpace: '<Root>/Plant' */
  AutonomousSteeringSystem_X.Plant_CSTATE[0] = 0.0;
  AutonomousSteeringSystem_X.Plant_CSTATE[1] = 0.0;
  AutonomousSteeringSystem_X.Plant_CSTATE[2] = 0.0;
  AutonomousSteeringSystem_X.Plant_CSTATE[3] = 0.0;
}

/* Model terminate function */
void AutonomousSteeringSystem_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
