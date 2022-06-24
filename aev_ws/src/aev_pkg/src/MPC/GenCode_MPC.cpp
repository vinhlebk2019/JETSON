//
// File: GenCode_MPC.cpp
//
// Code generated for Simulink model 'GenCode_MPC'.
//
// Model version                  : 1.27
// Simulink Coder version         : 9.4 (R2020b) 29-Jul-2020
// C/C++ source code generated on : Mon Dec  6 18:42:03 2021
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "GenCode_MPC.h"
#include "GenCode_MPC_private.h"

// Named constants for MATLAB Function: '<S35>/VariableHorizonOptimizer'
const real_T GenCode_MPC_Wdu = 0.10456395525912734;
const real_T GenCode_MPC_enable_value = 0.0;
const real_T GenCode_MPC_nu = 1.0;
const real_T GenCode_MPC_ny = 2.0;

//
// This function updates continuous states using the ODE4 fixed-step
// solver algorithm
//
void MPCModelClass::rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE4_IntgData *id = static_cast<ODE4_IntgData *>(rtsiGetSolverData(si));
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T *f3 = id->f[3];
  real_T temp;
  int_T i;
  int_T nXc = 1;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  // Save the state values at time t in y, we'll use x as ynew.
  (void) std::memcpy(y, x,
                     static_cast<uint_T>(nXc)*sizeof(real_T));

  // Assumes that rtsiSetT and ModelOutputs are up-to-date
  // f0 = f(t,y)
  rtsiSetdX(si, f0);
  GenCode_MPC_derivatives();

  // f1 = f(t + (h/2), y + (h/2)*f0)
  temp = 0.5 * h;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f0[i]);
  }

  rtsiSetT(si, t + temp);
  rtsiSetdX(si, f1);
  this->step();
  GenCode_MPC_derivatives();

  // f2 = f(t + (h/2), y + (h/2)*f1)
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f1[i]);
  }

  rtsiSetdX(si, f2);
  this->step();
  GenCode_MPC_derivatives();

  // f3 = f(t + h, y + h*f2)
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (h*f2[i]);
  }

  rtsiSetT(si, tnew);
  rtsiSetdX(si, f3);
  this->step();
  GenCode_MPC_derivatives();

  // tnew = t + h
  // ynew = y + (h/6)*(f0 + 2*f1 + 2*f2 + 2*f3)
  temp = h / 6.0;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + temp*(f0[i] + 2.0*f1[i] + 2.0*f2[i] + f3[i]);
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

real_T rt_powd_snf(real_T u0, real_T u1)
{
  real_T tmp;
  real_T tmp_0;
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else {
    tmp = std::abs(u0);
    tmp_0 = std::abs(u1);
    if (rtIsInf(u1)) {
      if (tmp == 1.0) {
        y = 1.0;
      } else if (tmp > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (tmp_0 == 0.0) {
      y = 1.0;
    } else if (tmp_0 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = std::sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
      y = (rtNaN);
    } else {
      y = std::pow(u0, u1);
    }
  }

  return y;
}

// Function for MATLAB Function: '<S4>/Adaptive Model'
void MPCModelClass::GenCode_MPC_lusolve(const real_T A[16], const real_T B[16],
  real_T X[16])
{
  real_T b_A[16];
  real_T smax;
  real_T y;
  int32_T b_j;
  int32_T c;
  int32_T c_ix;
  int32_T c_k;
  int32_T d;
  int32_T ijA;
  int32_T ix;
  int32_T jA;
  int8_T ipiv[4];
  int8_T ipiv_0;
  std::memcpy(&b_A[0], &A[0], sizeof(real_T) << 4U);
  ipiv[0] = 1;
  ipiv[1] = 2;
  ipiv[2] = 3;
  ipiv[3] = 4;
  for (b_j = 0; b_j < 3; b_j++) {
    c = b_j * 5;
    jA = 0;
    ix = c;
    smax = std::abs(b_A[c]);
    for (c_k = 2; c_k <= 4 - b_j; c_k++) {
      ix++;
      y = std::abs(b_A[ix]);
      if (y > smax) {
        jA = c_k - 1;
        smax = y;
      }
    }

    if (b_A[c + jA] != 0.0) {
      if (jA != 0) {
        jA += b_j;
        ipiv[b_j] = static_cast<int8_T>(jA + 1);
        smax = b_A[b_j];
        b_A[b_j] = b_A[jA];
        b_A[jA] = smax;
        smax = b_A[b_j + 4];
        b_A[b_j + 4] = b_A[jA + 4];
        b_A[jA + 4] = smax;
        smax = b_A[b_j + 8];
        b_A[b_j + 8] = b_A[jA + 8];
        b_A[jA + 8] = smax;
        smax = b_A[b_j + 12];
        b_A[b_j + 12] = b_A[jA + 12];
        b_A[jA + 12] = smax;
      }

      jA = (c - b_j) + 4;
      for (ix = c + 1; ix < jA; ix++) {
        b_A[ix] /= b_A[c];
      }
    }

    jA = c;
    ix = c + 4;
    for (c_k = 0; c_k <= 2 - b_j; c_k++) {
      if (b_A[ix] != 0.0) {
        smax = -b_A[ix];
        c_ix = c + 1;
        d = (jA - b_j) + 8;
        for (ijA = jA + 5; ijA < d; ijA++) {
          b_A[ijA] += b_A[c_ix] * smax;
          c_ix++;
        }
      }

      ix += 4;
      jA += 4;
    }
  }

  std::memcpy(&X[0], &B[0], sizeof(real_T) << 4U);
  for (b_j = 0; b_j < 3; b_j++) {
    ipiv_0 = ipiv[b_j];
    if (b_j + 1 != ipiv_0) {
      smax = X[b_j];
      X[b_j] = X[ipiv_0 - 1];
      X[ipiv_0 - 1] = smax;
      smax = X[b_j + 4];
      X[b_j + 4] = X[ipiv_0 + 3];
      X[ipiv_0 + 3] = smax;
      smax = X[b_j + 8];
      X[b_j + 8] = X[ipiv_0 + 7];
      X[ipiv_0 + 7] = smax;
      smax = X[b_j + 12];
      X[b_j + 12] = X[ipiv_0 + 11];
      X[ipiv_0 + 11] = smax;
    }
  }

  for (c = 0; c < 4; c++) {
    jA = c << 2;
    if (X[jA] != 0.0) {
      for (ix = 2; ix < 5; ix++) {
        b_j = (ix + jA) - 1;
        X[b_j] -= b_A[ix - 1] * X[jA];
      }
    }

    if (X[jA + 1] != 0.0) {
      for (ix = 3; ix < 5; ix++) {
        b_j = (ix + jA) - 1;
        X[b_j] -= X[jA + 1] * b_A[ix + 3];
      }
    }

    smax = X[jA + 2];
    if (smax != 0.0) {
      X[jA + 3] -= smax * b_A[11];
    }
  }

  for (c = 0; c < 4; c++) {
    jA = c << 2;
    smax = X[jA + 3];
    if (smax != 0.0) {
      X[jA + 3] = smax / b_A[15];
      for (ix = 0; ix < 3; ix++) {
        b_j = ix + jA;
        X[b_j] -= X[jA + 3] * b_A[ix + 12];
      }
    }

    smax = X[jA + 2];
    if (smax != 0.0) {
      X[jA + 2] = smax / b_A[10];
      for (ix = 0; ix < 2; ix++) {
        b_j = ix + jA;
        X[b_j] -= X[jA + 2] * b_A[ix + 8];
      }
    }

    smax = X[jA + 1];
    if (smax != 0.0) {
      X[jA + 1] = smax / b_A[5];
      X[jA] -= X[jA + 1] * b_A[4];
    }

    if (X[jA] != 0.0) {
      X[jA] /= b_A[0];
    }
  }
}

// Function for MATLAB Function: '<S4>/Adaptive Model'
void MPCModelClass::GenCode_PadeApproximantOfDegree(const real_T A[16], uint8_T
  m, real_T F[16])
{
  real_T A2[16];
  real_T A3[16];
  real_T A4[16];
  real_T A4_0[16];
  real_T U[16];
  real_T V[16];
  real_T A3_0;
  real_T A4_1;
  real_T d;
  int32_T A2_tmp;
  int32_T A2_tmp_tmp;
  int32_T g_k;
  int32_T i;
  for (g_k = 0; g_k < 4; g_k++) {
    for (i = 0; i < 4; i++) {
      A2_tmp_tmp = g_k << 2;
      A2_tmp = i + A2_tmp_tmp;
      A2[A2_tmp] = 0.0;
      A2[A2_tmp] += A[A2_tmp_tmp] * A[i];
      A2[A2_tmp] += A[A2_tmp_tmp + 1] * A[i + 4];
      A2[A2_tmp] += A[A2_tmp_tmp + 2] * A[i + 8];
      A2[A2_tmp] += A[A2_tmp_tmp + 3] * A[i + 12];
    }
  }

  if (m == 3) {
    std::memcpy(&U[0], &A2[0], sizeof(real_T) << 4U);
    U[0] += 60.0;
    U[5] += 60.0;
    U[10] += 60.0;
    U[15] += 60.0;
    for (g_k = 0; g_k < 4; g_k++) {
      for (i = 0; i < 4; i++) {
        A2_tmp_tmp = g_k << 2;
        A2_tmp = i + A2_tmp_tmp;
        A4_0[A2_tmp] = 0.0;
        A4_0[A2_tmp] += U[A2_tmp_tmp] * A[i];
        A4_0[A2_tmp] += U[A2_tmp_tmp + 1] * A[i + 4];
        A4_0[A2_tmp] += U[A2_tmp_tmp + 2] * A[i + 8];
        A4_0[A2_tmp] += U[A2_tmp_tmp + 3] * A[i + 12];
      }
    }

    for (g_k = 0; g_k < 16; g_k++) {
      U[g_k] = A4_0[g_k];
      V[g_k] = 12.0 * A2[g_k];
    }

    d = 120.0;
  } else {
    for (g_k = 0; g_k < 4; g_k++) {
      for (i = 0; i < 4; i++) {
        A2_tmp_tmp = g_k << 2;
        A2_tmp = i + A2_tmp_tmp;
        A3[A2_tmp] = 0.0;
        A3[A2_tmp] += A2[A2_tmp_tmp] * A2[i];
        A3[A2_tmp] += A2[A2_tmp_tmp + 1] * A2[i + 4];
        A3[A2_tmp] += A2[A2_tmp_tmp + 2] * A2[i + 8];
        A3[A2_tmp] += A2[A2_tmp_tmp + 3] * A2[i + 12];
      }
    }

    if (m == 5) {
      for (g_k = 0; g_k < 16; g_k++) {
        U[g_k] = 420.0 * A2[g_k] + A3[g_k];
      }

      U[0] += 15120.0;
      U[5] += 15120.0;
      U[10] += 15120.0;
      U[15] += 15120.0;
      for (g_k = 0; g_k < 4; g_k++) {
        for (i = 0; i < 4; i++) {
          A2_tmp_tmp = g_k << 2;
          A2_tmp = i + A2_tmp_tmp;
          A4_0[A2_tmp] = 0.0;
          A4_0[A2_tmp] += U[A2_tmp_tmp] * A[i];
          A4_0[A2_tmp] += U[A2_tmp_tmp + 1] * A[i + 4];
          A4_0[A2_tmp] += U[A2_tmp_tmp + 2] * A[i + 8];
          A4_0[A2_tmp] += U[A2_tmp_tmp + 3] * A[i + 12];
        }
      }

      for (g_k = 0; g_k < 16; g_k++) {
        U[g_k] = A4_0[g_k];
        V[g_k] = 30.0 * A3[g_k] + 3360.0 * A2[g_k];
      }

      d = 30240.0;
    } else {
      for (g_k = 0; g_k < 4; g_k++) {
        for (i = 0; i < 4; i++) {
          A2_tmp = g_k << 2;
          A2_tmp_tmp = i + A2_tmp;
          A4[A2_tmp_tmp] = 0.0;
          A4[A2_tmp_tmp] += A2[A2_tmp] * A3[i];
          A4[A2_tmp_tmp] += A2[A2_tmp + 1] * A3[i + 4];
          A4[A2_tmp_tmp] += A2[A2_tmp + 2] * A3[i + 8];
          A4[A2_tmp_tmp] += A2[A2_tmp + 3] * A3[i + 12];
        }
      }

      switch (m) {
       case 7:
        for (g_k = 0; g_k < 16; g_k++) {
          U[g_k] = (1512.0 * A3[g_k] + A4[g_k]) + 277200.0 * A2[g_k];
        }

        U[0] += 8.64864E+6;
        U[5] += 8.64864E+6;
        U[10] += 8.64864E+6;
        U[15] += 8.64864E+6;
        for (g_k = 0; g_k < 4; g_k++) {
          for (i = 0; i < 4; i++) {
            A2_tmp_tmp = g_k << 2;
            A2_tmp = i + A2_tmp_tmp;
            A4_0[A2_tmp] = 0.0;
            A4_0[A2_tmp] += U[A2_tmp_tmp] * A[i];
            A4_0[A2_tmp] += U[A2_tmp_tmp + 1] * A[i + 4];
            A4_0[A2_tmp] += U[A2_tmp_tmp + 2] * A[i + 8];
            A4_0[A2_tmp] += U[A2_tmp_tmp + 3] * A[i + 12];
          }
        }

        for (g_k = 0; g_k < 16; g_k++) {
          U[g_k] = A4_0[g_k];
          V[g_k] = (56.0 * A4[g_k] + 25200.0 * A3[g_k]) + 1.99584E+6 * A2[g_k];
        }

        d = 1.729728E+7;
        break;

       case 9:
        for (g_k = 0; g_k < 4; g_k++) {
          for (i = 0; i < 4; i++) {
            A2_tmp = g_k << 2;
            A2_tmp_tmp = i + A2_tmp;
            V[A2_tmp_tmp] = 0.0;
            V[A2_tmp_tmp] += A2[A2_tmp] * A4[i];
            V[A2_tmp_tmp] += A2[A2_tmp + 1] * A4[i + 4];
            V[A2_tmp_tmp] += A2[A2_tmp + 2] * A4[i + 8];
            V[A2_tmp_tmp] += A2[A2_tmp + 3] * A4[i + 12];
          }
        }

        for (g_k = 0; g_k < 16; g_k++) {
          U[g_k] = ((3960.0 * A4[g_k] + V[g_k]) + 2.16216E+6 * A3[g_k]) +
            3.027024E+8 * A2[g_k];
        }

        U[0] += 8.8216128E+9;
        U[5] += 8.8216128E+9;
        U[10] += 8.8216128E+9;
        U[15] += 8.8216128E+9;
        for (g_k = 0; g_k < 4; g_k++) {
          for (i = 0; i < 4; i++) {
            A2_tmp_tmp = g_k << 2;
            A2_tmp = i + A2_tmp_tmp;
            A4_0[A2_tmp] = 0.0;
            A4_0[A2_tmp] += U[A2_tmp_tmp] * A[i];
            A4_0[A2_tmp] += U[A2_tmp_tmp + 1] * A[i + 4];
            A4_0[A2_tmp] += U[A2_tmp_tmp + 2] * A[i + 8];
            A4_0[A2_tmp] += U[A2_tmp_tmp + 3] * A[i + 12];
          }
        }

        for (g_k = 0; g_k < 16; g_k++) {
          U[g_k] = A4_0[g_k];
          V[g_k] = ((90.0 * V[g_k] + 110880.0 * A4[g_k]) + 3.027024E+7 * A3[g_k])
            + 2.0756736E+9 * A2[g_k];
        }

        d = 1.76432256E+10;
        break;

       default:
        for (g_k = 0; g_k < 16; g_k++) {
          d = A2[g_k];
          A3_0 = A3[g_k];
          A4_1 = A4[g_k];
          U[g_k] = (3.352212864E+10 * A4_1 + 1.05594705216E+13 * A3_0) +
            1.1873537964288E+15 * d;
          V[g_k] = (16380.0 * A3_0 + A4_1) + 4.08408E+7 * d;
        }

        U[0] += 3.238237626624E+16;
        U[5] += 3.238237626624E+16;
        U[10] += 3.238237626624E+16;
        U[15] += 3.238237626624E+16;
        for (g_k = 0; g_k < 4; g_k++) {
          for (i = 0; i < 4; i++) {
            A2_tmp_tmp = g_k << 2;
            A2_tmp = A2_tmp_tmp + i;
            A4_0[A2_tmp] = (((V[A2_tmp_tmp + 1] * A4[i + 4] + V[A2_tmp_tmp] *
                              A4[i]) + V[A2_tmp_tmp + 2] * A4[i + 8]) +
                            V[A2_tmp_tmp + 3] * A4[i + 12]) + U[A2_tmp];
          }
        }

        for (g_k = 0; g_k < 4; g_k++) {
          for (i = 0; i < 4; i++) {
            A2_tmp_tmp = g_k << 2;
            A2_tmp = i + A2_tmp_tmp;
            U[A2_tmp] = 0.0;
            U[A2_tmp] += A4_0[A2_tmp_tmp] * A[i];
            U[A2_tmp] += A4_0[A2_tmp_tmp + 1] * A[i + 4];
            U[A2_tmp] += A4_0[A2_tmp_tmp + 2] * A[i + 8];
            U[A2_tmp] += A4_0[A2_tmp_tmp + 3] * A[i + 12];
          }
        }

        for (g_k = 0; g_k < 16; g_k++) {
          A4_0[g_k] = (182.0 * A4[g_k] + 960960.0 * A3[g_k]) + 1.32324192E+9 *
            A2[g_k];
        }

        for (g_k = 0; g_k < 4; g_k++) {
          for (i = 0; i < 4; i++) {
            A2_tmp_tmp = g_k << 2;
            A2_tmp = A2_tmp_tmp + i;
            V[A2_tmp] = (((((A4_0[A2_tmp_tmp + 1] * A4[i + 4] + A4_0[A2_tmp_tmp]
                             * A4[i]) + A4_0[A2_tmp_tmp + 2] * A4[i + 8]) +
                           A4_0[A2_tmp_tmp + 3] * A4[i + 12]) + A4[A2_tmp] *
                          6.704425728E+11) + A3[A2_tmp] * 1.29060195264E+14) +
              A2[A2_tmp] * 7.7717703038976E+15;
          }
        }

        d = 6.476475253248E+16;
        break;
      }
    }
  }

  V[0] += d;
  V[5] += d;
  V[10] += d;
  V[15] += d;
  for (g_k = 0; g_k < 16; g_k++) {
    d = U[g_k];
    A3_0 = V[g_k] - d;
    d *= 2.0;
    V[g_k] = A3_0;
    U[g_k] = d;
  }

  GenCode_MPC_lusolve(V, U, F);
  F[0]++;
  F[5]++;
  F[10]++;
  F[15]++;
}

// Function for MATLAB Function: '<S4>/Adaptive Model'
void MPCModelClass::GenCode_MPC_expm(real_T A[16], real_T F[16])
{
  static const real_T theta[5] = { 0.01495585217958292, 0.253939833006323,
    0.95041789961629319, 2.097847961257068, 5.3719203511481517 };

  static const uint8_T b[5] = { 3U, 5U, 7U, 9U, 13U };

  real_T F_0[16];
  real_T b_s;
  real_T normA;
  int32_T F_tmp;
  int32_T F_tmp_tmp;
  int32_T b_j;
  int32_T b_s_tmp;
  int32_T e;
  boolean_T exitg1;
  normA = 0.0;
  b_j = 0;
  exitg1 = false;
  while ((!exitg1) && (b_j < 4)) {
    b_s_tmp = b_j << 2;
    b_s = ((std::abs(A[b_s_tmp + 1]) + std::abs(A[b_s_tmp])) + std::abs
           (A[b_s_tmp + 2])) + std::abs(A[b_s_tmp + 3]);
    if (rtIsNaN(b_s)) {
      normA = (rtNaN);
      exitg1 = true;
    } else {
      if (b_s > normA) {
        normA = b_s;
      }

      b_j++;
    }
  }

  if (normA <= 5.3719203511481517) {
    b_j = 0;
    exitg1 = false;
    while ((!exitg1) && (b_j < 5)) {
      if (normA <= theta[b_j]) {
        GenCode_PadeApproximantOfDegree(A, b[b_j], F);
        exitg1 = true;
      } else {
        b_j++;
      }
    }
  } else {
    b_s = normA / 5.3719203511481517;
    if ((!rtIsInf(b_s)) && (!rtIsNaN(b_s))) {
      b_s = frexp(b_s, &e);
    } else {
      e = 0;
    }

    normA = e;
    if (b_s == 0.5) {
      normA = static_cast<real_T>(e) - 1.0;
    }

    b_s = rt_powd_snf(2.0, normA);
    for (b_j = 0; b_j < 16; b_j++) {
      A[b_j] /= b_s;
    }

    GenCode_PadeApproximantOfDegree(A, 13, F);
    for (b_s_tmp = 0; b_s_tmp < static_cast<int32_T>(normA); b_s_tmp++) {
      for (b_j = 0; b_j < 4; b_j++) {
        for (e = 0; e < 4; e++) {
          F_tmp_tmp = e << 2;
          F_tmp = b_j + F_tmp_tmp;
          F_0[F_tmp] = 0.0;
          F_0[F_tmp] += F[F_tmp_tmp] * F[b_j];
          F_0[F_tmp] += F[F_tmp_tmp + 1] * F[b_j + 4];
          F_0[F_tmp] += F[F_tmp_tmp + 2] * F[b_j + 8];
          F_0[F_tmp] += F[F_tmp_tmp + 3] * F[b_j + 12];
        }
      }

      std::memcpy(&F[0], &F_0[0], sizeof(real_T) << 4U);
    }
  }
}

void MPCModelClass::GenCode_MPC_emxInit_real_T(emxArray_real_T_GenCode_MPC_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_real_T_GenCode_MPC_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_real_T_GenCode_MPC_T *)std::malloc(sizeof
    (emxArray_real_T_GenCode_MPC_T));
  emxArray = *pEmxArray;
  emxArray->data = (real_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)std::malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

void MPCModelClass::GenCod_emxEnsureCapacity_real_T
  (emxArray_real_T_GenCode_MPC_T *emxArray, int32_T oldNumel)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = std::calloc(static_cast<uint32_T>(i), sizeof(real_T));
    if (emxArray->data != NULL) {
      std::memcpy(newData, emxArray->data, sizeof(real_T) * oldNumel);
      if (emxArray->canFreeData) {
        std::free(emxArray->data);
      }
    }

    emxArray->data = (real_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

void MPCModelClass::GenCode_MPC_emxFree_real_T(emxArray_real_T_GenCode_MPC_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T_GenCode_MPC_T *)NULL) {
    if (((*pEmxArray)->data != (real_T *)NULL) && (*pEmxArray)->canFreeData) {
      std::free((*pEmxArray)->data);
    }

    std::free((*pEmxArray)->size);
    std::free(*pEmxArray);
    *pEmxArray = (emxArray_real_T_GenCode_MPC_T *)NULL;
  }
}

void MPCModelClass::GenCode_MPC_emxFree_boolean_T
  (emxArray_boolean_T_GenCode_MP_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_boolean_T_GenCode_MP_T *)NULL) {
    if (((*pEmxArray)->data != (boolean_T *)NULL) && (*pEmxArray)->canFreeData)
    {
      std::free((*pEmxArray)->data);
    }

    std::free((*pEmxArray)->size);
    std::free(*pEmxArray);
    *pEmxArray = (emxArray_boolean_T_GenCode_MP_T *)NULL;
  }
}

void MPCModelClass::GenCode_MPC_emxInit_int16_T(emxArray_int16_T_GenCode_MPC_T **
  pEmxArray, int32_T numDimensions)
{
  emxArray_int16_T_GenCode_MPC_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_int16_T_GenCode_MPC_T *)std::malloc(sizeof
    (emxArray_int16_T_GenCode_MPC_T));
  emxArray = *pEmxArray;
  emxArray->data = (int16_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)std::malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

void MPCModelClass::GenCo_emxEnsureCapacity_int16_T
  (emxArray_int16_T_GenCode_MPC_T *emxArray, int32_T oldNumel)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = std::calloc(static_cast<uint32_T>(i), sizeof(int16_T));
    if (emxArray->data != NULL) {
      std::memcpy(newData, emxArray->data, sizeof(int16_T) * oldNumel);
      if (emxArray->canFreeData) {
        std::free(emxArray->data);
      }
    }

    emxArray->data = (int16_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

real_T rt_roundd_snf(real_T u)
{
  real_T y;
  if (std::abs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = std::floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = std::ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
real_T MPCModelClass::GenCode_MPC_norm(const real_T x_data[], const int32_T
  *x_size)
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  int32_T k;
  if (*x_size == 0) {
    y = 0.0;
  } else {
    y = 0.0;
    if (*x_size == 1) {
      y = std::abs(x_data[0]);
    } else {
      scale = 3.3121686421112381E-170;
      for (k = 0; k < *x_size; k++) {
        absxk = std::abs(x_data[k]);
        if (absxk > scale) {
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * std::sqrt(y);
    }
  }

  return y;
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_abs_d(const emxArray_real_T_GenCode_MPC_T *x,
  emxArray_real_T_GenCode_MPC_T *y)
{
  int32_T k;
  k = y->size[0];
  y->size[0] = x->size[0];
  GenCod_emxEnsureCapacity_real_T(y, k);
  for (k = 0; k < x->size[0]; k++) {
    y->data[k] = std::abs(x->data[k]);
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_maximum2(const emxArray_real_T_GenCode_MPC_T *x,
  emxArray_real_T_GenCode_MPC_T *ex)
{
  emxArray_real_T_GenCode_MPC_T *z1;
  real_T u0;
  int32_T k;
  int32_T loop_ub;
  GenCode_MPC_emxInit_real_T(&z1, 1);
  k = ex->size[0];
  ex->size[0] = x->size[0];
  GenCod_emxEnsureCapacity_real_T(ex, k);
  k = z1->size[0];
  z1->size[0] = ex->size[0];
  GenCod_emxEnsureCapacity_real_T(z1, k);
  loop_ub = ex->size[0];
  for (k = 0; k < loop_ub; k++) {
    z1->data[k] = ex->data[k];
  }

  for (k = 0; k < ex->size[0]; k++) {
    u0 = x->data[k];
    if (u0 > 1.0) {
      z1->data[k] = u0;
    } else {
      z1->data[k] = 1.0;
    }
  }

  k = ex->size[0];
  ex->size[0] = z1->size[0];
  GenCod_emxEnsureCapacity_real_T(ex, k);
  loop_ub = z1->size[0];
  for (k = 0; k < loop_ub; k++) {
    ex->data[k] = z1->data[k];
  }

  GenCode_MPC_emxFree_real_T(&z1);
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
real_T MPCModelClass::GenCode_MPC_xnrm2(int32_T n, const real_T x_data[],
  int32_T ix0)
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  int32_T k;
  int32_T kend;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x_data[ix0 - 1]);
    } else {
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        absxk = std::abs(x_data[k - 1]);
        if (absxk > scale) {
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * std::sqrt(y);
    }
  }

  return y;
}

real_T rt_hypotd_snf(real_T u0, real_T u1)
{
  real_T a;
  real_T y;
  a = std::abs(u0);
  y = std::abs(u1);
  if (a < y) {
    a /= y;
    y *= std::sqrt(a * a + 1.0);
  } else if (a > y) {
    y /= a;
    y = std::sqrt(y * y + 1.0) * a;
  } else {
    if (!rtIsNaN(y)) {
      y = a * 1.4142135623730951;
    }
  }

  return y;
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_xgemv(int32_T b_m, int32_T n, const real_T
  b_A_data[], int32_T ia0, int32_T lda, const real_T x_data[], int32_T ix0,
  real_T y_data[])
{
  real_T c;
  int32_T b;
  int32_T b_iy;
  int32_T d;
  int32_T ia;
  int32_T iac;
  int32_T ix;
  if (n != 0) {
    for (b_iy = 0; b_iy < n; b_iy++) {
      y_data[b_iy] = 0.0;
    }

    b_iy = 0;
    b = (n - 1) * lda + ia0;
    iac = ia0;
    while (((lda > 0) && (iac <= b)) || ((lda < 0) && (iac >= b))) {
      ix = ix0;
      c = 0.0;
      d = (iac + b_m) - 1;
      for (ia = iac; ia <= d; ia++) {
        c += b_A_data[ia - 1] * x_data[ix - 1];
        ix++;
      }

      y_data[b_iy] += c;
      b_iy++;
      iac += lda;
    }
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_xgerc(int32_T b_m, int32_T n, real_T alpha1,
  int32_T ix0, const real_T y_data[], real_T b_A_data[], int32_T ia0, int32_T
  lda)
{
  real_T temp;
  int32_T b;
  int32_T ijA;
  int32_T ix;
  int32_T j;
  int32_T jA;
  int32_T jy;
  if (!(alpha1 == 0.0)) {
    jA = ia0 - 1;
    jy = 0;
    for (j = 0; j < n; j++) {
      if (y_data[jy] != 0.0) {
        temp = y_data[jy] * alpha1;
        ix = ix0;
        b = b_m + jA;
        for (ijA = jA; ijA < b; ijA++) {
          b_A_data[ijA] += b_A_data[ix - 1] * temp;
          ix++;
        }
      }

      jy++;
      jA += lda;
    }
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_xzlarf(int32_T b_m, int32_T n, int32_T iv0,
  real_T tau, real_T b_C_data[], int32_T ic0, int32_T ldc, real_T work_data[])
{
  int32_T coltop;
  int32_T exitg1;
  int32_T ia;
  int32_T lastc;
  int32_T lastv;
  boolean_T exitg2;
  if (tau != 0.0) {
    lastv = b_m;
    lastc = iv0 + b_m;
    while ((lastv > 0) && (b_C_data[lastc - 2] == 0.0)) {
      lastv--;
      lastc--;
    }

    lastc = n;
    exitg2 = false;
    while ((!exitg2) && (lastc > 0)) {
      coltop = (lastc - 1) * ldc + ic0;
      ia = coltop;
      do {
        exitg1 = 0;
        if (ia <= (coltop + lastv) - 1) {
          if (b_C_data[ia - 1] != 0.0) {
            exitg1 = 1;
          } else {
            ia++;
          }
        } else {
          lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    lastv = 0;
    lastc = 0;
  }

  if (lastv > 0) {
    GenCode_MPC_xgemv(lastv, lastc, b_C_data, ic0, ldc, b_C_data, iv0, work_data);
    GenCode_MPC_xgerc(lastv, lastc, -tau, iv0, work_data, b_C_data, ic0, ldc);
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_qrf(real_T b_A_data[], const int32_T b_A_size[2],
  int32_T b_m, int32_T n, int32_T nfxd, real_T tau_data[])
{
  real_T work_data[21];
  real_T b_atmp;
  real_T beta1;
  int32_T b_k;
  int32_T c_k;
  int32_T i;
  int32_T ii;
  int32_T knt;
  int32_T lda;
  int32_T mmi_tmp;
  lda = b_A_size[0];
  ii = b_A_size[1];
  for (i = 0; i < ii; i++) {
    work_data[i] = 0.0;
  }

  for (i = 0; i < nfxd; i++) {
    ii = i * lda + i;
    mmi_tmp = b_m - i;
    if (i + 1 < b_m) {
      b_atmp = b_A_data[ii];
      tau_data[i] = 0.0;
      if (mmi_tmp > 0) {
        beta1 = GenCode_MPC_xnrm2(mmi_tmp - 1, b_A_data, ii + 2);
        if (beta1 != 0.0) {
          beta1 = rt_hypotd_snf(b_A_data[ii], beta1);
          if (b_A_data[ii] >= 0.0) {
            beta1 = -beta1;
          }

          if (std::abs(beta1) < 1.0020841800044864E-292) {
            knt = -1;
            b_k = ii + mmi_tmp;
            do {
              knt++;
              for (c_k = ii + 1; c_k < b_k; c_k++) {
                b_A_data[c_k] *= 9.9792015476736E+291;
              }

              beta1 *= 9.9792015476736E+291;
              b_atmp *= 9.9792015476736E+291;
            } while (!(std::abs(beta1) >= 1.0020841800044864E-292));

            beta1 = rt_hypotd_snf(b_atmp, GenCode_MPC_xnrm2(mmi_tmp - 1,
              b_A_data, ii + 2));
            if (b_atmp >= 0.0) {
              beta1 = -beta1;
            }

            tau_data[i] = (beta1 - b_atmp) / beta1;
            b_atmp = 1.0 / (b_atmp - beta1);
            for (c_k = ii + 1; c_k < b_k; c_k++) {
              b_A_data[c_k] *= b_atmp;
            }

            for (b_k = 0; b_k <= knt; b_k++) {
              beta1 *= 1.0020841800044864E-292;
            }

            b_atmp = beta1;
          } else {
            tau_data[i] = (beta1 - b_A_data[ii]) / beta1;
            b_atmp = 1.0 / (b_A_data[ii] - beta1);
            knt = ii + mmi_tmp;
            for (b_k = ii + 1; b_k < knt; b_k++) {
              b_A_data[b_k] *= b_atmp;
            }

            b_atmp = beta1;
          }
        }
      }

      b_A_data[ii] = b_atmp;
    } else {
      tau_data[i] = 0.0;
    }

    if (i + 1 < n) {
      b_atmp = b_A_data[ii];
      b_A_data[ii] = 1.0;
      GenCode_MPC_xzlarf(mmi_tmp, (n - i) - 1, ii + 1, tau_data[i], b_A_data,
                         (ii + lda) + 1, lda, work_data);
      b_A_data[ii] = b_atmp;
    }
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_xzgeqp3(real_T b_A_data[], const int32_T
  b_A_size[2], int32_T b_m, int32_T n, real_T tau_data[], int32_T *tau_size)
{
  int32_T i;
  int32_T minmana;
  int32_T minmn;
  if (b_A_size[0] < b_A_size[1]) {
    minmana = b_A_size[0];
  } else {
    minmana = b_A_size[1];
  }

  if (b_m < n) {
    minmn = b_m;
  } else {
    minmn = n;
  }

  *tau_size = minmana;
  for (i = 0; i < minmana; i++) {
    tau_data[i] = 0.0;
  }

  if ((b_A_size[0] != 0) && (b_A_size[1] != 0) && (minmn >= 1)) {
    GenCode_MPC_qrf(b_A_data, b_A_size, b_m, n, minmn, tau_data);
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_xgeqrf(real_T b_A_data[], int32_T b_A_size[2],
  real_T tau_data[], int32_T *tau_size)
{
  real_T c_A_data[441];
  int32_T c_A_size[2];
  int32_T i;
  int32_T i_0;
  int32_T loop_ub;
  int32_T loop_ub_0;
  c_A_size[0] = b_A_size[0];
  c_A_size[1] = b_A_size[1];
  loop_ub = b_A_size[0] * b_A_size[1] - 1;
  if (0 <= loop_ub) {
    std::memcpy(&c_A_data[0], &b_A_data[0], (loop_ub + 1) * sizeof(real_T));
  }

  GenCode_MPC_xzgeqp3(c_A_data, c_A_size, b_A_size[0], b_A_size[1], tau_data,
                      tau_size);
  b_A_size[0] = c_A_size[0];
  b_A_size[1] = c_A_size[1];
  loop_ub = c_A_size[1];
  for (i_0 = 0; i_0 < loop_ub; i_0++) {
    loop_ub_0 = c_A_size[0];
    for (i = 0; i < loop_ub_0; i++) {
      b_A_data[i + b_A_size[0] * i_0] = c_A_data[c_A_size[0] * i_0 + i];
    }
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_xorgqr(int32_T b_m, int32_T n, int32_T k, real_T
  b_A_data[], const int32_T b_A_size[2], int32_T lda, const real_T tau_data[])
{
  real_T work_data[21];
  int32_T b_k;
  int32_T d;
  int32_T i;
  int32_T iaii;
  int32_T itau;
  if (n >= 1) {
    for (itau = k; itau < n; itau++) {
      i = itau * lda;
      for (iaii = 0; iaii < b_m; iaii++) {
        b_A_data[i + iaii] = 0.0;
      }

      b_A_data[i + itau] = 1.0;
    }

    itau = k - 1;
    iaii = static_cast<int8_T>(b_A_size[1]);
    for (i = 0; i < iaii; i++) {
      work_data[i] = 0.0;
    }

    for (i = k; i >= 1; i--) {
      iaii = (i - 1) * lda + i;
      if (i < n) {
        b_A_data[iaii - 1] = 1.0;
        GenCode_MPC_xzlarf((b_m - i) + 1, n - i, iaii, tau_data[itau], b_A_data,
                           iaii + lda, lda, work_data);
      }

      if (i < b_m) {
        d = (iaii + b_m) - i;
        for (b_k = iaii; b_k < d; b_k++) {
          b_A_data[b_k] *= -tau_data[itau];
        }
      }

      b_A_data[iaii - 1] = 1.0 - tau_data[itau];
      for (d = 0; d <= i - 2; d++) {
        b_A_data[(iaii - d) - 2] = 0.0;
      }

      itau--;
    }
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_qr(const real_T b_A_data[], const int32_T
  b_A_size[2], real_T Q_data[], int32_T Q_size[2], real_T R_data[], int32_T
  R_size[2])
{
  real_T c_A_data[441];
  real_T tau_data[21];
  int32_T c_A_size[2];
  int32_T b_m;
  int32_T g_i;
  int32_T loop_ub;
  int32_T n;
  int8_T b_idx_0;
  b_m = b_A_size[0] - 1;
  n = b_A_size[1];
  b_idx_0 = static_cast<int8_T>(b_A_size[0]);
  Q_size[0] = static_cast<int8_T>(b_A_size[0]);
  Q_size[1] = static_cast<int8_T>(b_A_size[0]);
  R_size[0] = b_A_size[0];
  R_size[1] = b_A_size[1];
  if (b_A_size[0] > b_A_size[1]) {
    for (loop_ub = 0; loop_ub < n; loop_ub++) {
      for (g_i = 0; g_i <= b_m; g_i++) {
        Q_data[g_i + b_idx_0 * loop_ub] = b_A_data[b_A_size[0] * loop_ub + g_i];
      }
    }

    for (loop_ub = b_A_size[1]; loop_ub < b_m + 1; loop_ub++) {
      for (g_i = 0; g_i <= b_m; g_i++) {
        Q_data[g_i + b_idx_0 * loop_ub] = 0.0;
      }
    }

    GenCode_MPC_xgeqrf(Q_data, Q_size, tau_data, &loop_ub);
    for (loop_ub = 0; loop_ub < n; loop_ub++) {
      for (g_i = 0; g_i <= loop_ub; g_i++) {
        R_data[g_i + R_size[0] * loop_ub] = Q_data[Q_size[0] * loop_ub + g_i];
      }

      for (g_i = loop_ub + 1; g_i < b_m + 1; g_i++) {
        R_data[g_i + R_size[0] * loop_ub] = 0.0;
      }
    }

    GenCode_MPC_xorgqr(b_A_size[0], b_A_size[0], b_A_size[1], Q_data, Q_size,
                       b_A_size[0], tau_data);
  } else {
    c_A_size[0] = b_A_size[0];
    c_A_size[1] = b_A_size[1];
    loop_ub = b_A_size[0] * b_A_size[1] - 1;
    if (0 <= loop_ub) {
      std::memcpy(&c_A_data[0], &b_A_data[0], (loop_ub + 1) * sizeof(real_T));
    }

    GenCode_MPC_xgeqrf(c_A_data, c_A_size, tau_data, &loop_ub);
    for (loop_ub = 0; loop_ub <= b_m; loop_ub++) {
      for (g_i = 0; g_i <= loop_ub; g_i++) {
        R_data[g_i + R_size[0] * loop_ub] = c_A_data[c_A_size[0] * loop_ub + g_i];
      }

      for (g_i = loop_ub + 1; g_i < b_m + 1; g_i++) {
        R_data[g_i + R_size[0] * loop_ub] = 0.0;
      }
    }

    for (loop_ub = b_A_size[0]; loop_ub < n; loop_ub++) {
      for (g_i = 0; g_i <= b_m; g_i++) {
        R_data[g_i + R_size[0] * loop_ub] = c_A_data[c_A_size[0] * loop_ub + g_i];
      }
    }

    GenCode_MPC_xorgqr(b_A_size[0], b_A_size[0], b_A_size[0], c_A_data, c_A_size,
                       b_A_size[0], tau_data);
    for (n = 0; n <= b_m; n++) {
      for (loop_ub = 0; loop_ub <= b_m; loop_ub++) {
        Q_data[loop_ub + b_idx_0 * n] = c_A_data[c_A_size[0] * n + loop_ub];
      }
    }
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_mtimes_ojosf(const real_T b_A_data[], const
  int32_T b_A_size[2], const real_T b_B_data[], real_T b_C_data[], int32_T
  *b_C_size)
{
  int32_T aoffset;
  int32_T b_i;
  int32_T b_m;
  int32_T i;
  b_m = b_A_size[0] - 1;
  *b_C_size = b_A_size[0];
  for (i = 0; i <= b_m; i++) {
    b_C_data[i] = 0.0;
  }

  for (i = 0; i < b_A_size[1]; i++) {
    aoffset = i * b_A_size[0];
    for (b_i = 0; b_i <= b_m; b_i++) {
      b_C_data[b_i] += b_A_data[aoffset + b_i] * b_B_data[i];
    }
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
real_T MPCModelClass::GenCode_MPC_KWIKfactor(const emxArray_real_T_GenCode_MPC_T
  *b_Ac, const int16_T iC_data[], int16_T nA, const real_T Linv_data[], const
  int32_T Linv_size[2], real_T RLinv_data[], const int32_T RLinv_size[2], real_T
  b_D_data[], const int32_T b_D_size[2], real_T b_H_data[], const int32_T
  b_H_size[2], int16_T n)
{
  real_T QQ_data[441];
  real_T RR_data[441];
  real_T TL_data[441];
  real_T b_Ac_data[21];
  real_T tmp_data[21];
  real_T Linv;
  real_T Status;
  int32_T QQ_size[2];
  int32_T RR_size[2];
  int32_T RLinv;
  int32_T RLinv_0;
  int32_T TL_size_idx_0;
  int32_T b_i;
  int32_T exitg1;
  int32_T i;
  int32_T loop_ub;
  int16_T b_j;
  int16_T c_k;
  TL_size_idx_0 = Linv_size[0];
  Status = 1.0;
  RLinv = RLinv_size[0];
  RLinv_0 = RLinv_size[1];
  for (b_i = 0; b_i < RLinv_0; b_i++) {
    for (i = 0; i < RLinv; i++) {
      RLinv_data[i + RLinv_size[0] * b_i] = 0.0;
    }
  }

  if (0 <= nA - 1) {
    loop_ub = b_Ac->size[1];
  }

  for (i = 1; i - 1 < nA; i++) {
    RLinv = iC_data[static_cast<int16_T>(i) - 1];
    for (b_i = 0; b_i < loop_ub; b_i++) {
      b_Ac_data[b_i] = b_Ac->data[(b_Ac->size[0] * b_i + RLinv) - 1];
    }

    GenCode_MPC_mtimes_ojosf(Linv_data, Linv_size, b_Ac_data, tmp_data, &RLinv);
    for (b_i = 0; b_i < RLinv; b_i++) {
      RLinv_data[b_i + RLinv_size[0] * (static_cast<int16_T>(i) - 1)] =
        tmp_data[b_i];
    }
  }

  GenCode_MPC_qr(RLinv_data, RLinv_size, QQ_data, QQ_size, RR_data, RR_size);
  b_i = 1;
  do {
    exitg1 = 0;
    if (b_i - 1 <= nA - 1) {
      if (std::abs(RR_data[((static_cast<int16_T>(b_i) - 1) * RR_size[0] +
                            static_cast<int16_T>(b_i)) - 1]) < 1.0E-12) {
        Status = -2.0;
        exitg1 = 1;
      } else {
        b_i++;
      }
    } else {
      for (i = 1; i - 1 < n; i++) {
        for (RLinv = 1; RLinv - 1 < n; RLinv++) {
          loop_ub = Linv_size[0];
          Linv = 0.0;
          for (b_i = 0; b_i < loop_ub; b_i++) {
            Linv += Linv_data[(static_cast<int16_T>(i) - 1) * Linv_size[0] + b_i]
              * QQ_data[(static_cast<int16_T>(RLinv) - 1) * QQ_size[0] + b_i];
          }

          TL_data[(static_cast<int16_T>(i) + TL_size_idx_0 *
                   (static_cast<int16_T>(RLinv) - 1)) - 1] = Linv;
        }
      }

      RLinv = RLinv_size[0];
      RLinv_0 = RLinv_size[1];
      for (b_i = 0; b_i < RLinv_0; b_i++) {
        for (i = 0; i < RLinv; i++) {
          RLinv_data[i + RLinv_size[0] * b_i] = 0.0;
        }
      }

      for (b_j = nA; b_j > 0; b_j = static_cast<int16_T>(b_j - 1)) {
        RLinv_data[(b_j + RLinv_size[0] * (b_j - 1)) - 1] = 1.0;
        for (c_k = b_j; c_k <= nA; c_k = static_cast<int16_T>(c_k + 1)) {
          b_i = ((c_k - 1) * RLinv_size[0] + b_j) - 1;
          RLinv_data[b_i] /= RR_data[((b_j - 1) * RR_size[0] + b_j) - 1];
        }

        if (b_j > 1) {
          for (loop_ub = 1; loop_ub - 1 <= b_j - 2; loop_ub++) {
            for (c_k = b_j; c_k <= nA; c_k = static_cast<int16_T>(c_k + 1)) {
              b_i = (c_k - 1) * RLinv_size[0];
              i = (b_i + static_cast<int16_T>(loop_ub)) - 1;
              RLinv_data[i] -= RR_data[((b_j - 1) * RR_size[0] +
                static_cast<int16_T>(loop_ub)) - 1] * RLinv_data[(b_i + b_j) - 1];
            }
          }
        }
      }

      for (loop_ub = 1; loop_ub - 1 < n; loop_ub++) {
        for (b_j = static_cast<int16_T>(loop_ub); b_j <= n; b_j =
             static_cast<int16_T>(b_j + 1)) {
          b_i = (static_cast<int16_T>(loop_ub) + b_H_size[0] * (b_j - 1)) - 1;
          b_H_data[b_i] = 0.0;
          i = nA + 1;
          if (nA + 1 > 32767) {
            i = 32767;
          }

          for (c_k = static_cast<int16_T>(i); c_k <= n; c_k =
               static_cast<int16_T>(c_k + 1)) {
            i = (c_k - 1) * TL_size_idx_0;
            b_H_data[b_i] -= TL_data[(i + static_cast<int16_T>(loop_ub)) - 1] *
              TL_data[(i + b_j) - 1];
          }

          b_H_data[(b_j + b_H_size[0] * (static_cast<int16_T>(loop_ub) - 1)) - 1]
            = b_H_data[b_i];
        }
      }

      for (i = 1; i - 1 < nA; i++) {
        for (loop_ub = 1; loop_ub - 1 < n; loop_ub++) {
          b_i = (static_cast<int16_T>(loop_ub) + b_D_size[0] *
                 (static_cast<int16_T>(i) - 1)) - 1;
          b_D_data[b_i] = 0.0;
          for (b_j = static_cast<int16_T>(i); b_j <= nA; b_j =
               static_cast<int16_T>(b_j + 1)) {
            b_D_data[b_i] += TL_data[((b_j - 1) * TL_size_idx_0 +
              static_cast<int16_T>(loop_ub)) - 1] * RLinv_data[((b_j - 1) *
              RLinv_size[0] + static_cast<int16_T>(i)) - 1];
          }
        }
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return Status;
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_abs(const real_T x_data[], const int32_T x_size
  [2], real_T y_data[], int32_T y_size[2])
{
  int32_T k;
  y_size[0] = 1;
  y_size[1] = static_cast<int8_T>(x_size[1]);
  for (k = 0; k < x_size[1]; k++) {
    y_data[k] = std::abs(x_data[k]);
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_qpkwik(const real_T Linv_data[], const int32_T
  Linv_size[2], const real_T Hinv_data[], const int32_T Hinv_size[2], const
  real_T f_data[], const emxArray_real_T_GenCode_MPC_T *b_Ac, const
  emxArray_real_T_GenCode_MPC_T *b, emxArray_int16_T_GenCode_MPC_T *iA, int16_T
  b_m, int16_T n, real_T x_data[], int32_T *x_size, real_T lambda_data[],
  int32_T *lambda_size, real_T *status)
{
  emxArray_real_T_GenCode_MPC_T *cTol;
  emxArray_real_T_GenCode_MPC_T *tmp;
  real_T RLinv_data[441];
  real_T b_D_data[441];
  real_T b_H_data[441];
  real_T b_H_data_0[441];
  real_T AcRow_data[21];
  real_T b_Ac_data[21];
  real_T r_data[21];
  real_T z_data[21];
  real_T Xnorm0;
  real_T cMin;
  real_T rMin;
  real_T t2;
  real_T zTa;
  int32_T AcRow_size[2];
  int32_T RLinv_size[2];
  int32_T b_Ac_size[2];
  int32_T b_D_size[2];
  int32_T b_H_size[2];
  int32_T b_H_size_0[2];
  int32_T exitg1;
  int32_T g_i;
  int32_T idx;
  int32_T loop_ub;
  int32_T z_size;
  int16_T kDrop;
  int16_T kNext;
  int16_T nA;
  int16_T tmp_0;
  int16_T tmp_1;
  boolean_T cTolComputed;
  boolean_T exitg2;
  boolean_T exitg3;
  boolean_T guard1 = false;
  boolean_T isT1Inf;
  boolean_T tempOK;
  *status = 1.0;
  *lambda_size = b_m;
  loop_ub = b_m;
  for (idx = 0; idx < loop_ub; idx++) {
    lambda_data[idx] = 0.0;
  }

  if (b_m == 0) {
    *x_size = n;
    loop_ub = n;
    for (idx = 0; idx < loop_ub; idx++) {
      x_data[idx] = 0.0;
    }

    for (g_i = 1; g_i - 1 < n; g_i++) {
      loop_ub = Hinv_size[1];
      Xnorm0 = 0.0;
      for (idx = 0; idx < loop_ub; idx++) {
        Xnorm0 += -Hinv_data[(Hinv_size[0] * idx + static_cast<int16_T>(g_i)) -
          1] * f_data[idx];
      }

      x_data[static_cast<int16_T>(g_i) - 1] = Xnorm0;
    }
  } else {
    loop_ub = n;
    for (idx = 0; idx < loop_ub; idx++) {
      r_data[idx] = 0.0;
    }

    GenCode_MPC_emxInit_real_T(&cTol, 1);
    rMin = 0.0;
    RLinv_size[0] = Linv_size[0];
    RLinv_size[1] = Linv_size[1];
    b_D_size[0] = Linv_size[0];
    b_D_size[1] = Linv_size[1];
    b_H_size[0] = Linv_size[0];
    b_H_size[1] = Linv_size[1];
    idx = cTol->size[0];
    cTol->size[0] = b_m;
    GenCod_emxEnsureCapacity_real_T(cTol, idx);
    loop_ub = b_m;
    for (idx = 0; idx < loop_ub; idx++) {
      cTol->data[idx] = 1.0;
    }

    cTolComputed = false;
    if (0 <= b_m - 1) {
      std::memset(&GenCode_MPC_B.iC_data[0], 0, b_m * sizeof(int16_T));
    }

    nA = 0;
    *x_size = n;
    loop_ub = n;
    for (idx = 0; idx < loop_ub; idx++) {
      x_data[idx] = 0.0;
    }

    for (g_i = 1; g_i - 1 < n; g_i++) {
      loop_ub = Hinv_size[1];
      Xnorm0 = 0.0;
      for (idx = 0; idx < loop_ub; idx++) {
        Xnorm0 += -Hinv_data[(Hinv_size[0] * idx + static_cast<int16_T>(g_i)) -
          1] * f_data[idx];
      }

      x_data[static_cast<int16_T>(g_i) - 1] = Xnorm0;
    }

    Xnorm0 = GenCode_MPC_norm(x_data, x_size);
    GenCode_MPC_emxInit_real_T(&tmp, 1);
    exitg2 = false;
    while ((!exitg2) && (*status <= 120.0)) {
      cMin = -1.0E-6;
      kNext = 0;
      for (g_i = 1; g_i - 1 < b_m; g_i++) {
        if (!cTolComputed) {
          loop_ub = b_Ac->size[1];
          b_Ac_size[0] = 1;
          b_Ac_size[1] = loop_ub;
          for (idx = 0; idx < loop_ub; idx++) {
            b_Ac_data[idx] = b_Ac->data[(b_Ac->size[0] * idx +
              static_cast<int16_T>(g_i)) - 1] * x_data[idx];
          }

          GenCode_MPC_abs(b_Ac_data, b_Ac_size, AcRow_data, AcRow_size);
          if (AcRow_size[1] <= 2) {
            if (AcRow_size[1] == 1) {
              t2 = AcRow_data[0];
            } else if (AcRow_data[0] < AcRow_data[1]) {
              t2 = AcRow_data[1];
            } else if (rtIsNaN(AcRow_data[0])) {
              if (!rtIsNaN(AcRow_data[1])) {
                t2 = AcRow_data[1];
              } else {
                t2 = AcRow_data[0];
              }
            } else {
              t2 = AcRow_data[0];
            }
          } else {
            if (!rtIsNaN(AcRow_data[0])) {
              idx = 1;
            } else {
              idx = 0;
              loop_ub = 2;
              exitg3 = false;
              while ((!exitg3) && (loop_ub <= AcRow_size[1])) {
                if (!rtIsNaN(AcRow_data[loop_ub - 1])) {
                  idx = loop_ub;
                  exitg3 = true;
                } else {
                  loop_ub++;
                }
              }
            }

            if (idx == 0) {
              t2 = AcRow_data[0];
            } else {
              t2 = AcRow_data[idx - 1];
              while (idx + 1 <= AcRow_size[1]) {
                if (t2 < AcRow_data[idx]) {
                  t2 = AcRow_data[idx];
                }

                idx++;
              }
            }
          }

          zTa = cTol->data[static_cast<int16_T>(g_i) - 1];
          if ((zTa > t2) || rtIsNaN(t2)) {
            cTol->data[static_cast<int16_T>(g_i) - 1] = zTa;
          } else {
            cTol->data[static_cast<int16_T>(g_i) - 1] = t2;
          }
        }

        if (iA->data[static_cast<int16_T>(g_i) - 1] == 0) {
          loop_ub = b_Ac->size[1];
          t2 = 0.0;
          for (idx = 0; idx < loop_ub; idx++) {
            t2 += b_Ac->data[(b_Ac->size[0] * idx + static_cast<int16_T>(g_i)) -
              1] * x_data[idx];
          }

          t2 = (t2 - b->data[static_cast<int16_T>(g_i) - 1]) / cTol->data[
            static_cast<int16_T>(g_i) - 1];
          if (t2 < cMin) {
            cMin = t2;
            kNext = static_cast<int16_T>(g_i);
          }
        }
      }

      cTolComputed = true;
      if (kNext <= 0) {
        exitg2 = true;
      } else if (*status == 120.0) {
        *status = 0.0;
        exitg2 = true;
      } else {
        do {
          exitg1 = 0;
          if ((kNext > 0) && (*status <= 120.0)) {
            loop_ub = b_Ac->size[1];
            for (idx = 0; idx < loop_ub; idx++) {
              AcRow_data[idx] = b_Ac->data[(b_Ac->size[0] * idx + kNext) - 1];
            }

            guard1 = false;
            if (nA == 0) {
              loop_ub = b_Ac->size[1];
              for (idx = 0; idx < loop_ub; idx++) {
                b_Ac_data[idx] = b_Ac->data[(b_Ac->size[0] * idx + kNext) - 1];
              }

              GenCode_MPC_mtimes_ojosf(Hinv_data, Hinv_size, b_Ac_data, z_data,
                &z_size);
              guard1 = true;
            } else {
              cMin = GenCode_MPC_KWIKfactor(b_Ac, GenCode_MPC_B.iC_data, nA,
                Linv_data, Linv_size, RLinv_data, RLinv_size, b_D_data, b_D_size,
                b_H_data, b_H_size, n);
              if (cMin <= 0.0) {
                *status = -2.0;
                exitg1 = 1;
              } else {
                b_H_size_0[0] = b_H_size[0];
                b_H_size_0[1] = b_H_size[1];
                g_i = b_H_size[0] * b_H_size[1];
                for (idx = 0; idx < g_i; idx++) {
                  b_H_data_0[idx] = -b_H_data[idx];
                }

                g_i = b_Ac->size[1];
                for (idx = 0; idx < g_i; idx++) {
                  b_Ac_data[idx] = b_Ac->data[(b_Ac->size[0] * idx + kNext) - 1];
                }

                GenCode_MPC_mtimes_ojosf(b_H_data_0, b_H_size_0, b_Ac_data,
                  z_data, &z_size);
                for (g_i = 1; g_i - 1 < nA; g_i++) {
                  cMin = 0.0;
                  for (idx = 0; idx < loop_ub; idx++) {
                    cMin += b_D_data[(static_cast<int16_T>(g_i) - 1) * b_D_size
                      [0] + idx] * AcRow_data[idx];
                  }

                  r_data[static_cast<int16_T>(g_i) - 1] = cMin;
                }

                guard1 = true;
              }
            }

            if (guard1) {
              kDrop = 0;
              cMin = 0.0;
              isT1Inf = true;
              tempOK = true;
              if (nA > 0) {
                idx = 0;
                exitg3 = false;
                while ((!exitg3) && (idx <= nA - 1)) {
                  if (r_data[idx] >= 1.0E-12) {
                    tempOK = false;
                    exitg3 = true;
                  } else {
                    idx++;
                  }
                }
              }

              if ((nA != 0) && (!tempOK)) {
                for (idx = 1; idx - 1 < nA; idx++) {
                  t2 = r_data[static_cast<int16_T>(idx) - 1];
                  if (t2 > 1.0E-12) {
                    t2 = lambda_data[GenCode_MPC_B.iC_data[static_cast<int16_T>
                      (idx) - 1] - 1] / t2;
                    if ((kDrop == 0) || (t2 < rMin)) {
                      rMin = t2;
                      kDrop = static_cast<int16_T>(idx);
                    }
                  }
                }

                if (kDrop > 0) {
                  cMin = rMin;
                  isT1Inf = false;
                }
              }

              zTa = 0.0;
              if (z_size >= 1) {
                for (idx = 0; idx < z_size; idx++) {
                  zTa += z_data[idx] * AcRow_data[idx];
                }
              }

              if (zTa <= 0.0) {
                t2 = 0.0;
                tempOK = true;
              } else {
                loop_ub = b_Ac->size[1];
                t2 = 0.0;
                for (idx = 0; idx < loop_ub; idx++) {
                  t2 += b_Ac->data[(b_Ac->size[0] * idx + kNext) - 1] *
                    x_data[idx];
                }

                t2 = (b->data[kNext - 1] - t2) / zTa;
                tempOK = false;
              }

              if (isT1Inf && tempOK) {
                *status = -1.0;
                exitg1 = 1;
              } else {
                if (tempOK) {
                  zTa = cMin;
                } else if (isT1Inf) {
                  zTa = t2;
                } else if ((cMin < t2) || rtIsNaN(t2)) {
                  zTa = cMin;
                } else {
                  zTa = t2;
                }

                for (idx = 1; idx - 1 < nA; idx++) {
                  loop_ub = GenCode_MPC_B.iC_data[static_cast<int16_T>(idx) - 1];
                  lambda_data[loop_ub - 1] -= r_data[static_cast<int16_T>(idx) -
                    1] * zTa;
                  if ((loop_ub <= b_m) && (lambda_data[loop_ub - 1] < 0.0)) {
                    lambda_data[loop_ub - 1] = 0.0;
                  }
                }

                lambda_data[kNext - 1] += zTa;
                if (zTa == cMin) {
                  iA->data[GenCode_MPC_B.iC_data[kDrop - 1] - 1] = 0;
                  if (kDrop < nA) {
                    while (kDrop <= nA - 1) {
                      GenCode_MPC_B.iC_data[kDrop - 1] =
                        GenCode_MPC_B.iC_data[kDrop];
                      kDrop = static_cast<int16_T>(kDrop + 1);
                    }
                  }

                  GenCode_MPC_B.iC_data[nA - 1] = 0;
                  idx = nA - 1;
                  if (nA - 1 < -32768) {
                    idx = -32768;
                  }

                  nA = static_cast<int16_T>(idx);
                }

                if (!tempOK) {
                  loop_ub = n;
                  for (idx = 0; idx < loop_ub; idx++) {
                    x_data[idx] += zTa * z_data[idx];
                  }

                  if (zTa == t2) {
                    if (nA == n) {
                      *status = -1.0;
                      exitg1 = 1;
                    } else {
                      idx = nA + 1;
                      if (nA + 1 > 32767) {
                        idx = 32767;
                      }

                      nA = static_cast<int16_T>(idx);
                      GenCode_MPC_B.iC_data[static_cast<int16_T>(idx) - 1] =
                        kNext;
                      kDrop = static_cast<int16_T>(idx);
                      exitg3 = false;
                      while ((!exitg3) && (kDrop > 1)) {
                        tmp_0 = GenCode_MPC_B.iC_data[kDrop - 1];
                        tmp_1 = GenCode_MPC_B.iC_data[kDrop - 2];
                        if (tmp_0 > tmp_1) {
                          exitg3 = true;
                        } else {
                          GenCode_MPC_B.iC_data[kDrop - 1] = tmp_1;
                          GenCode_MPC_B.iC_data[kDrop - 2] = tmp_0;
                          kDrop = static_cast<int16_T>(kDrop - 1);
                        }
                      }

                      iA->data[kNext - 1] = 1;
                      kNext = 0;
                      (*status)++;
                    }
                  } else {
                    (*status)++;
                  }
                } else {
                  (*status)++;
                }
              }
            }
          } else {
            cMin = GenCode_MPC_norm(x_data, x_size);
            if (std::abs(cMin - Xnorm0) > 0.001) {
              Xnorm0 = cMin;
              GenCode_MPC_abs_d(b, tmp);
              GenCode_MPC_maximum2(tmp, cTol);
              cTolComputed = false;
            }

            exitg1 = 2;
          }
        } while (exitg1 == 0);

        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
    }

    GenCode_MPC_emxFree_real_T(&tmp);
    GenCode_MPC_emxFree_real_T(&cTol);
  }
}

void MPCModelClass::GenCode_MPC_emxFree_int16_T(emxArray_int16_T_GenCode_MPC_T **
  pEmxArray)
{
  if (*pEmxArray != (emxArray_int16_T_GenCode_MPC_T *)NULL) {
    if (((*pEmxArray)->data != (int16_T *)NULL) && (*pEmxArray)->canFreeData) {
      std::free((*pEmxArray)->data);
    }

    std::free((*pEmxArray)->size);
    std::free(*pEmxArray);
    *pEmxArray = (emxArray_int16_T_GenCode_MPC_T *)NULL;
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_mpc_solveQP(const real_T xQP[5], real_T nCon,
  real_T b_degrees, const real_T Kx_data[], const real_T Kr_data[], const
  int32_T Kr_size[2], const emxArray_real_T_GenCode_MPC_T *rseq, const real_T
  Ku1_data[], real_T old_u, const real_T Kv_data[], const int32_T Kv_size[2],
  const emxArray_real_T_GenCode_MPC_T *vseq, const real_T Kut_data[], const
  int32_T Kut_size[2], const real_T b_utarget_data[], const real_T Linv_data[],
  const int32_T Linv_size[2], const real_T Hinv_data[], const int32_T Hinv_size
  [2], const emxArray_real_T_GenCode_MPC_T *b_Ac, const
  emxArray_real_T_GenCode_MPC_T *Bc, const emxArray_boolean_T_GenCode_MP_T *iA,
  real_T zopt_data[], int32_T *zopt_size, real_T f_data[], int32_T *f_size,
  real_T *status)
{
  emxArray_int16_T_GenCode_MPC_T *iAnew;
  real_T Kr;
  real_T Kut;
  real_T Kv;
  real_T Kx;
  int32_T i;
  int32_T loop_ub;
  int32_T unusedU0_size;
  int16_T tmp;
  int16_T tmp_0;
  *f_size = static_cast<int32_T>(b_degrees);
  loop_ub = static_cast<int32_T>(b_degrees);
  for (unusedU0_size = 0; unusedU0_size < loop_ub; unusedU0_size++) {
    f_data[unusedU0_size] = 0.0;
  }

  for (i = 0; i < static_cast<int32_T>(b_degrees - 1.0); i++) {
    Kx = 0.0;
    for (unusedU0_size = 0; unusedU0_size < 5; unusedU0_size++) {
      Kx += Kx_data[5 * i + unusedU0_size] * xQP[unusedU0_size];
    }

    loop_ub = Kr_size[0];
    Kr = 0.0;
    for (unusedU0_size = 0; unusedU0_size < loop_ub; unusedU0_size++) {
      Kr += Kr_data[Kr_size[0] * i + unusedU0_size] * rseq->data[unusedU0_size];
    }

    loop_ub = Kv_size[0];
    Kv = 0.0;
    for (unusedU0_size = 0; unusedU0_size < loop_ub; unusedU0_size++) {
      Kv += Kv_data[Kv_size[0] * i + unusedU0_size] * vseq->data[unusedU0_size];
    }

    loop_ub = Kut_size[0];
    Kut = 0.0;
    for (unusedU0_size = 0; unusedU0_size < loop_ub; unusedU0_size++) {
      Kut += Kut_data[Kut_size[0] * i + unusedU0_size] *
        b_utarget_data[unusedU0_size];
    }

    f_data[i] = (((Kx + Kr) + Ku1_data[i] * old_u) + Kv) + Kut;
  }

  GenCode_MPC_emxInit_int16_T(&iAnew, 1);
  unusedU0_size = iAnew->size[0];
  iAnew->size[0] = iA->size[0];
  GenCo_emxEnsureCapacity_int16_T(iAnew, unusedU0_size);
  loop_ub = iA->size[0];
  for (unusedU0_size = 0; unusedU0_size < loop_ub; unusedU0_size++) {
    iAnew->data[unusedU0_size] = iA->data[unusedU0_size];
  }

  Kx = rt_roundd_snf(nCon);
  Kr = rt_roundd_snf(b_degrees);
  if (Kx < 32768.0) {
    if (Kx >= -32768.0) {
      tmp = static_cast<int16_T>(Kx);
    } else {
      tmp = MIN_int16_T;
    }
  } else {
    tmp = MAX_int16_T;
  }

  if (Kr < 32768.0) {
    if (Kr >= -32768.0) {
      tmp_0 = static_cast<int16_T>(Kr);
    } else {
      tmp_0 = MIN_int16_T;
    }
  } else {
    tmp_0 = MAX_int16_T;
  }

  GenCode_MPC_qpkwik(Linv_data, Linv_size, Hinv_data, Hinv_size, f_data, b_Ac,
                     Bc, iAnew, tmp, tmp_0, zopt_data, zopt_size,
                     GenCode_MPC_B.unusedU0_data, &unusedU0_size, status);
  GenCode_MPC_emxFree_int16_T(&iAnew);
  if ((*status < 0.0) || (*status == 0.0)) {
    *zopt_size = static_cast<int32_T>(b_degrees);
    loop_ub = static_cast<int32_T>(b_degrees);
    for (unusedU0_size = 0; unusedU0_size < loop_ub; unusedU0_size++) {
      zopt_data[unusedU0_size] = 0.0;
    }
  }
}

void MPCModelClass::Gen_emxEnsureCapacity_boolean_T
  (emxArray_boolean_T_GenCode_MP_T *emxArray, int32_T oldNumel)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = std::calloc(static_cast<uint32_T>(i), sizeof(boolean_T));
    if (emxArray->data != NULL) {
      std::memcpy(newData, emxArray->data, sizeof(boolean_T) * oldNumel);
      if (emxArray->canFreeData) {
        std::free(emxArray->data);
      }
    }

    emxArray->data = (boolean_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_mtimes_o(const real_T b_A_data[], const int32_T
  b_A_size[2], const real_T b_B_data[], const int32_T b_B_size[2], real_T
  b_C_data[], int32_T b_C_size[2])
{
  real_T bkj;
  int32_T b_C_data_tmp;
  int32_T b_i;
  int32_T b_m;
  int32_T boffset;
  int32_T coffset;
  int32_T i;
  int32_T j;
  b_m = b_A_size[1];
  b_C_size[0] = b_A_size[1];
  b_C_size[1] = b_B_size[1];
  for (j = 0; j < b_B_size[1]; j++) {
    coffset = j * b_m;
    boffset = j * b_B_size[0];
    for (i = 0; i < b_m; i++) {
      b_C_data[coffset + i] = 0.0;
    }

    for (i = 0; i < b_A_size[0]; i++) {
      bkj = b_B_data[boffset + i];
      for (b_i = 0; b_i < b_m; b_i++) {
        b_C_data_tmp = coffset + b_i;
        b_C_data[b_C_data_tmp] += b_A_data[b_i * b_A_size[0] + i] * bkj;
      }
    }
  }
}

void MPCModelClass::GenCode_MPC_emxInit_boolean_T
  (emxArray_boolean_T_GenCode_MP_T **pEmxArray, int32_T numDimensions)
{
  emxArray_boolean_T_GenCode_MP_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_boolean_T_GenCode_MP_T *)std::malloc(sizeof
    (emxArray_boolean_T_GenCode_MP_T));
  emxArray = *pEmxArray;
  emxArray->data = (boolean_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)std::malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_mtimes_ojos(const emxArray_real_T_GenCode_MPC_T *
  b_A, const emxArray_real_T_GenCode_MPC_T *b_B, emxArray_real_T_GenCode_MPC_T
  *b_C)
{
  int32_T aoffset;
  int32_T b_i;
  int32_T b_m;
  int32_T i;
  b_m = b_A->size[0] - 1;
  i = b_C->size[0];
  b_C->size[0] = b_A->size[0];
  GenCod_emxEnsureCapacity_real_T(b_C, i);
  for (i = 0; i <= b_m; i++) {
    b_C->data[i] = 0.0;
  }

  for (i = 0; i < b_A->size[1]; i++) {
    aoffset = i * b_A->size[0];
    for (b_i = 0; b_i <= b_m; b_i++) {
      b_C->data[b_i] += b_A->data[aoffset + b_i] * b_B->data[i];
    }
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_mtimes_ojo(const emxArray_real_T_GenCode_MPC_T
  *b_A, const real_T b_B[5], emxArray_real_T_GenCode_MPC_T *b_C)
{
  real_T s;
  int32_T i;
  int32_T k;
  k = b_C->size[0];
  b_C->size[0] = b_A->size[0];
  GenCod_emxEnsureCapacity_real_T(b_C, k);
  for (i = 0; i < b_A->size[0]; i++) {
    s = 0.0;
    for (k = 0; k < 5; k++) {
      s += b_A->data[k * b_A->size[0] + i] * b_B[k];
    }

    b_C->data[i] = s;
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_trisolve(const real_T b_A_data[], const int32_T
  b_A_size[2], real_T b_B_data[], const int32_T b_B_size[2])
{
  real_T tmp_0;
  int32_T i;
  int32_T j;
  int32_T jBcol;
  int32_T k;
  int32_T kAcol;
  int32_T mB;
  int32_T n;
  int32_T nB;
  int32_T tmp;
  int32_T tmp_1;
  mB = b_B_size[0];
  nB = b_B_size[1] - 1;
  if (b_A_size[0] < b_A_size[1]) {
    n = b_A_size[0];
  } else {
    n = b_A_size[1];
  }

  if (n >= b_B_size[0]) {
    n = b_B_size[0];
  }

  if ((b_B_size[1] != 0) && ((b_B_size[0] != 0) && (b_B_size[1] != 0))) {
    for (j = 0; j <= nB; j++) {
      jBcol = mB * j - 1;
      for (k = 1; k - 1 < n; k++) {
        kAcol = (k - 1) * b_A_size[0] - 1;
        tmp = k + jBcol;
        tmp_0 = b_B_data[tmp];
        if (tmp_0 != 0.0) {
          b_B_data[tmp] = tmp_0 / b_A_data[k + kAcol];
          for (i = k + 1; i <= n; i++) {
            tmp_1 = i + jBcol;
            b_B_data[tmp_1] -= b_B_data[tmp] * b_A_data[i + kAcol];
          }
        }
      }
    }
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_linsolve(const real_T b_A_data[], const int32_T
  b_A_size[2], const real_T b_B_data[], const int32_T b_B_size[2], real_T
  b_C_data[], int32_T b_C_size[2])
{
  int32_T i;
  int32_T j;
  int32_T y;
  int8_T b_idx_0;
  if (b_A_size[0] < b_A_size[1]) {
    y = b_A_size[0];
  } else {
    y = b_A_size[1];
  }

  b_idx_0 = static_cast<int8_T>(b_A_size[1]);
  b_C_size[0] = static_cast<int8_T>(b_A_size[1]);
  b_C_size[1] = static_cast<int8_T>(b_B_size[1]);
  for (j = 0; j < b_B_size[1]; j++) {
    for (i = 0; i < y; i++) {
      b_C_data[i + b_idx_0 * j] = b_B_data[b_B_size[0] * j + i];
    }

    for (i = y; i < b_A_size[1]; i++) {
      b_C_data[i + b_idx_0 * j] = 0.0;
    }
  }

  GenCode_MPC_trisolve(b_A_data, b_A_size, b_C_data, b_C_size);
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_eye_g(real_T varargin_1, real_T b_I_data[],
  int32_T b_I_size[2])
{
  int32_T i;
  int32_T loop_ub;
  b_I_size[0] = static_cast<int32_T>(varargin_1);
  b_I_size[1] = static_cast<int32_T>(varargin_1);
  loop_ub = static_cast<int32_T>(varargin_1) * static_cast<int32_T>(varargin_1)
    - 1;
  for (i = 0; i <= loop_ub; i++) {
    b_I_data[i] = 0.0;
  }

  if (static_cast<int32_T>(varargin_1) > 0) {
    for (loop_ub = 0; loop_ub < static_cast<int32_T>(varargin_1); loop_ub++) {
      b_I_data[loop_ub + static_cast<int32_T>(varargin_1) * loop_ub] = 1.0;
    }
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
int32_T MPCModelClass::GenCode_MPC_xpotrf(int32_T n, real_T b_A_data[], int32_T
  lda)
{
  real_T c;
  real_T ssq;
  int32_T b_ix;
  int32_T b_iy;
  int32_T b_k;
  int32_T d;
  int32_T ia;
  int32_T iac;
  int32_T idxAjj;
  int32_T info;
  int32_T iy;
  int32_T j;
  boolean_T exitg1;
  info = 0;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j <= n - 1)) {
    idxAjj = j * lda + j;
    ssq = 0.0;
    if (j >= 1) {
      b_ix = j;
      b_iy = j;
      for (b_k = 0; b_k < j; b_k++) {
        ssq += b_A_data[b_ix] * b_A_data[b_iy];
        b_ix += lda;
        b_iy += lda;
      }
    }

    ssq = b_A_data[idxAjj] - ssq;
    if (ssq > 0.0) {
      ssq = std::sqrt(ssq);
      b_A_data[idxAjj] = ssq;
      if (j + 1 < n) {
        b_ix = (n - j) - 1;
        if ((b_ix != 0) && (j != 0)) {
          b_iy = j;
          b_k = ((j - 1) * lda + j) + 2;
          iac = j + 2;
          while (((lda > 0) && (iac <= b_k)) || ((lda < 0) && (iac >= b_k))) {
            c = -b_A_data[b_iy];
            iy = idxAjj + 1;
            d = (iac + b_ix) - 1;
            for (ia = iac; ia <= d; ia++) {
              b_A_data[iy] += b_A_data[ia - 1] * c;
              iy++;
            }

            b_iy += lda;
            iac += lda;
          }
        }

        ssq = 1.0 / ssq;
        b_ix += idxAjj;
        for (idxAjj++; idxAjj < b_ix + 1; idxAjj++) {
          b_A_data[idxAjj] *= ssq;
        }
      }

      j++;
    } else {
      b_A_data[idxAjj] = ssq;
      info = j + 1;
      exitg1 = true;
    }
  }

  return info;
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_diag(const real_T v_data[], const int32_T
  v_size[2], real_T d_data[], int32_T *d_size)
{
  int32_T dlen;
  int32_T k;
  if ((v_size[0] == 1) && (v_size[1] == 1)) {
    *d_size = 1;
    d_data[0] = v_data[0];
  } else {
    if (0 < v_size[1]) {
      if (v_size[0] < v_size[1]) {
        dlen = v_size[0];
      } else {
        dlen = v_size[1];
      }
    } else {
      dlen = 0;
    }

    *d_size = dlen;
    for (k = 0; k < dlen; k++) {
      d_data[k] = v_data[v_size[0] * k + k];
    }
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
real_T MPCModelClass::GenCode_MPC_minimum(const real_T x_data[], const int32_T
  *x_size)
{
  real_T ex;
  int32_T idx;
  int32_T k;
  boolean_T exitg1;
  if (*x_size <= 2) {
    if (*x_size == 1) {
      ex = x_data[0];
    } else if (x_data[0] > x_data[1]) {
      ex = x_data[1];
    } else if (rtIsNaN(x_data[0])) {
      if (!rtIsNaN(x_data[1])) {
        ex = x_data[1];
      } else {
        ex = x_data[0];
      }
    } else {
      ex = x_data[0];
    }
  } else {
    if (!rtIsNaN(x_data[0])) {
      idx = 1;
    } else {
      idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= *x_size)) {
        if (!rtIsNaN(x_data[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (idx == 0) {
      ex = x_data[0];
    } else {
      ex = x_data[idx - 1];
      while (idx + 1 <= *x_size) {
        if (ex > x_data[idx]) {
          ex = x_data[idx];
        }

        idx++;
      }
    }
  }

  return ex;
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_mpc_checkhessian(real_T b_H_data[], const
  int32_T b_H_size[2], real_T L_data[], int32_T L_size[2], real_T *BadH)
{
  real_T tmp_data[21];
  real_T normH;
  real_T s;
  int32_T Tries;
  int32_T b_degrees;
  int32_T emlN;
  int32_T loop_ub;
  int32_T tmp_size;
  int8_T b_data[441];
  boolean_T exitg1;
  boolean_T exitg2;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  b_degrees = b_H_size[0];
  *BadH = 0.0;
  emlN = b_H_size[1];
  L_size[0] = b_H_size[0];
  L_size[1] = b_H_size[1];
  Tries = b_H_size[0] * b_H_size[1];
  if (0 <= Tries - 1) {
    std::memcpy(&L_data[0], &b_H_data[0], ((Tries - 1) + 1) * sizeof(real_T));
  }

  loop_ub = GenCode_MPC_xpotrf(b_H_size[1], L_data, b_H_size[1]);
  guard1 = false;
  if (loop_ub == 0) {
    GenCode_MPC_diag(L_data, L_size, tmp_data, &tmp_size);
    if (GenCode_MPC_minimum(tmp_data, &tmp_size) > 1.4901161193847656E-7) {
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    if ((b_H_size[0] == 0) || (b_H_size[1] == 0)) {
      normH = 0.0;
    } else if ((b_H_size[0] == 1) || (b_H_size[1] == 1)) {
      normH = 0.0;
      for (loop_ub = 0; loop_ub < Tries; loop_ub++) {
        s = std::abs(b_H_data[loop_ub]);
        if (rtIsNaN(s)) {
          normH = s;
        } else {
          if (s > normH) {
            normH = s;
          }
        }
      }
    } else {
      normH = 0.0;
      Tries = 0;
      exitg2 = false;
      while ((!exitg2) && (Tries <= b_H_size[0] - 1)) {
        s = 0.0;
        for (loop_ub = 0; loop_ub < b_H_size[1]; loop_ub++) {
          s += std::abs(b_H_data[b_H_size[0] * loop_ub + Tries]);
        }

        if (rtIsNaN(s)) {
          normH = (rtNaN);
          exitg2 = true;
        } else {
          if (s > normH) {
            normH = s;
          }

          Tries++;
        }
      }
    }

    if (normH >= 1.0E+10) {
      *BadH = 2.0;
    } else {
      Tries = 0;
      exitg1 = false;
      while ((!exitg1) && (Tries <= 4)) {
        normH = rt_powd_snf(10.0, static_cast<real_T>(Tries)) *
          1.4901161193847656E-7;
        loop_ub = b_degrees * b_degrees - 1;
        if (0 <= loop_ub) {
          std::memset(&b_data[0], 0, (loop_ub + 1) * sizeof(int8_T));
        }

        if (b_degrees > 0) {
          for (loop_ub = 0; loop_ub < b_degrees; loop_ub++) {
            b_data[loop_ub + b_degrees * loop_ub] = 1;
          }
        }

        loop_ub = b_H_size[0] * b_H_size[1] - 1;
        L_size[0] = b_H_size[0];
        L_size[1] = b_H_size[1];
        for (tmp_size = 0; tmp_size <= loop_ub; tmp_size++) {
          b_H_data[tmp_size] += normH * static_cast<real_T>(b_data[tmp_size]);
          L_data[tmp_size] = b_H_data[tmp_size];
        }

        loop_ub = GenCode_MPC_xpotrf(emlN, L_data, emlN);
        guard2 = false;
        if (loop_ub == 0) {
          GenCode_MPC_diag(L_data, L_size, tmp_data, &tmp_size);
          if (GenCode_MPC_minimum(tmp_data, &tmp_size) > 1.4901161193847656E-7)
          {
            *BadH = 1.0;
            exitg1 = true;
          } else {
            guard2 = true;
          }
        } else {
          guard2 = true;
        }

        if (guard2) {
          *BadH = 3.0;
          Tries++;
        }
      }
    }
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_WtMult(const real_T W[2], const real_T M_data[],
  const int32_T M_size[2], real_T WM_data[], int32_T WM_size[2])
{
  real_T W_0;
  int32_T i;
  int32_T i_0;
  int32_T loop_ub;
  int16_T ixw;
  int8_T b_idx_0;
  b_idx_0 = static_cast<int8_T>(M_size[0]);
  WM_size[0] = static_cast<int8_T>(M_size[0]);
  WM_size[1] = static_cast<int8_T>(M_size[1]);
  loop_ub = static_cast<int8_T>(M_size[0]) * static_cast<int8_T>(M_size[1]) - 1;
  for (i_0 = 0; i_0 <= loop_ub; i_0++) {
    WM_data[i_0] = 0.0;
  }

  ixw = 1;
  for (i = 0; i < b_idx_0; i++) {
    W_0 = W[ixw - 1];
    loop_ub = M_size[1];
    for (i_0 = 0; i_0 < loop_ub; i_0++) {
      WM_data[i + b_idx_0 * i_0] = M_data[M_size[0] * i_0 + i] * W_0;
    }

    ixw = static_cast<int16_T>(ixw + 1);
    if (ixw > 2) {
      ixw = 1;
    }
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_WtMult_l(real_T W, const real_T M_data[], const
  int32_T M_size[2], real_T WM_data[], int32_T WM_size[2])
{
  int32_T i;
  int32_T i_0;
  int32_T loop_ub;
  int8_T b_idx_0;
  b_idx_0 = static_cast<int8_T>(M_size[0]);
  WM_size[0] = static_cast<int8_T>(M_size[0]);
  WM_size[1] = static_cast<int8_T>(M_size[1]);
  loop_ub = static_cast<int8_T>(M_size[0]) * static_cast<int8_T>(M_size[1]) - 1;
  for (i_0 = 0; i_0 <= loop_ub; i_0++) {
    WM_data[i_0] = 0.0;
  }

  for (i = 0; i < b_idx_0; i++) {
    loop_ub = M_size[1];
    for (i_0 = 0; i_0 < loop_ub; i_0++) {
      WM_data[i + b_idx_0 * i_0] = M_data[M_size[0] * i_0 + i] * W;
    }
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_WtMult_lm(const real_T M_data[], const int32_T
  M_size[2], real_T WM_data[], int32_T WM_size[2])
{
  int32_T i;
  int32_T i_0;
  int32_T loop_ub;
  int8_T b_idx_0;
  b_idx_0 = static_cast<int8_T>(M_size[0]);
  WM_size[0] = static_cast<int8_T>(M_size[0]);
  WM_size[1] = static_cast<int8_T>(M_size[1]);
  loop_ub = static_cast<int8_T>(M_size[0]) * static_cast<int8_T>(M_size[1]) - 1;
  for (i_0 = 0; i_0 <= loop_ub; i_0++) {
    WM_data[i_0] = 0.0;
  }

  for (i = 0; i < b_idx_0; i++) {
    loop_ub = M_size[1];
    for (i_0 = 0; i_0 < loop_ub; i_0++) {
      WM_data[i + b_idx_0 * i_0] = M_data[M_size[0] * i_0 + i] * GenCode_MPC_Wdu;
    }
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_mtimes_oj(const real_T b_A_data[], const int32_T
  *b_A_size, const real_T b_B_data[], const int32_T b_B_size[2], real_T
  b_C_data[], int32_T b_C_size[2])
{
  int32_T boffset;
  int32_T j;
  int32_T k;
  b_C_size[0] = 1;
  b_C_size[1] = b_B_size[1];
  for (j = 0; j < b_B_size[1]; j++) {
    boffset = j * b_B_size[0];
    b_C_data[j] = 0.0;
    for (k = 0; k < *b_A_size; k++) {
      b_C_data[j] += b_B_data[boffset + k] * b_A_data[k];
    }
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MP_mpc_calculatehessian(const real_T b_Wy[2], real_T
  b_Wu, const real_T SuJm_data[], const int32_T SuJm_size[2], const real_T
  I2Jm_data[], const int32_T I2Jm_size[2], const real_T Jm_data[], const int32_T
  Jm_size[2], const real_T I1_data[], const int32_T *I1_size, const real_T
  Su1_data[], const int32_T *Su1_size, const real_T Sx_data[], const int32_T
  Sx_size[2], const real_T Hv_data[], const int32_T Hv_size[2], real_T b_H_data[],
  int32_T b_H_size[2], real_T Ku1_data[], int32_T Ku1_size[2], real_T Kut_data[],
  int32_T Kut_size[2], real_T Kx_data[], int32_T Kx_size[2], real_T Kv_data[],
  int32_T Kv_size[2], real_T Kr_data[], int32_T Kr_size[2])
{
  real_T tmp_data[400];
  real_T tmp_data_0[20];
  real_T tmp_data_1[20];
  real_T bkj;
  int32_T tmp_size[2];
  int32_T tmp_size_0[2];
  int32_T tmp_size_1[2];
  int32_T Kx_data_tmp;
  int32_T boffset;
  int32_T coffset;
  int32_T i;
  int32_T k;
  int32_T loop_ub;
  GenCode_MPC_WtMult(b_Wy, SuJm_data, SuJm_size, Kr_data, Kr_size);
  GenCode_MPC_WtMult_l(b_Wu, I2Jm_data, I2Jm_size, Kut_data, Kut_size);
  GenCode_MPC_mtimes_o(SuJm_data, SuJm_size, Kr_data, Kr_size,
                       GenCode_MPC_B.tmp_data, tmp_size);
  GenCode_MPC_WtMult_lm(Jm_data, Jm_size, tmp_data, tmp_size_0);
  GenCode_MPC_mtimes_o(Jm_data, Jm_size, tmp_data, tmp_size_0,
                       GenCode_MPC_B.tmp_data_m, tmp_size_1);
  GenCode_MPC_mtimes_o(I2Jm_data, I2Jm_size, Kut_data, Kut_size,
                       GenCode_MPC_B.tmp_data_c, tmp_size_0);
  b_H_size[0] = tmp_size[0];
  b_H_size[1] = tmp_size[1];
  loop_ub = tmp_size[0] * tmp_size[1];
  for (i = 0; i < loop_ub; i++) {
    b_H_data[i] = (GenCode_MPC_B.tmp_data[i] + GenCode_MPC_B.tmp_data_m[i]) +
      GenCode_MPC_B.tmp_data_c[i];
  }

  GenCode_MPC_mtimes_oj(Su1_data, Su1_size, Kr_data, Kr_size, tmp_data_0,
                        tmp_size);
  GenCode_MPC_mtimes_oj(I1_data, I1_size, Kut_data, Kut_size, tmp_data_1,
                        tmp_size_0);
  Ku1_size[0] = 1;
  Ku1_size[1] = tmp_size[1];
  loop_ub = tmp_size[0] * tmp_size[1];
  for (i = 0; i < loop_ub; i++) {
    Ku1_data[i] = tmp_data_0[i] + tmp_data_1[i];
  }

  loop_ub = Kut_size[0] * Kut_size[1] - 1;
  for (i = 0; i <= loop_ub; i++) {
    Kut_data[i] = -Kut_data[i];
  }

  Kx_size[0] = 5;
  Kx_size[1] = Kr_size[1];
  for (loop_ub = 0; loop_ub < Kr_size[1]; loop_ub++) {
    coffset = loop_ub * 5;
    boffset = loop_ub * Kr_size[0];
    for (i = 0; i < 5; i++) {
      Kx_data[coffset + i] = 0.0;
    }

    for (k = 0; k < Sx_size[0]; k++) {
      bkj = Kr_data[boffset + k];
      for (i = 0; i < 5; i++) {
        Kx_data_tmp = coffset + i;
        Kx_data[Kx_data_tmp] += Sx_data[i * Sx_size[0] + k] * bkj;
      }
    }
  }

  GenCode_MPC_mtimes_o(Hv_data, Hv_size, Kr_data, Kr_size,
                       GenCode_MPC_B.tmp_data, tmp_size);
  Kv_size[0] = tmp_size[0];
  Kv_size[1] = tmp_size[1];
  loop_ub = tmp_size[0] * tmp_size[1];
  if (0 <= loop_ub - 1) {
    std::memcpy(&Kv_data[0], &GenCode_MPC_B.tmp_data[0], loop_ub * sizeof(real_T));
  }

  loop_ub = Kr_size[0] * Kr_size[1] - 1;
  for (i = 0; i <= loop_ub; i++) {
    Kr_data[i] = -Kr_data[i];
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_updateWeights(real_T W[2], const real_T
  b_signal[2])
{
  if (b_signal[0] < 0.0) {
    W[0] = 0.0;
  } else {
    W[0] = b_signal[0] * b_signal[0];
  }

  if (b_signal[1] < 0.0) {
    W[1] = 0.0;
  } else {
    W[1] = b_signal[1] * b_signal[1];
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_eye(real_T varargin_1, real_T b_I_data[],
  int32_T b_I_size[2])
{
  int32_T i;
  int32_T loop_ub;
  b_I_size[0] = static_cast<int32_T>(varargin_1);
  b_I_size[1] = static_cast<int32_T>(varargin_1);
  loop_ub = static_cast<int32_T>(varargin_1) * static_cast<int32_T>(varargin_1)
    - 1;
  for (i = 0; i <= loop_ub; i++) {
    b_I_data[i] = 0.0;
  }

  if (static_cast<int32_T>(varargin_1) > 0) {
    for (loop_ub = 0; loop_ub < static_cast<int32_T>(varargin_1); loop_ub++) {
      b_I_data[loop_ub + static_cast<int32_T>(varargin_1) * loop_ub] = 1.0;
    }
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_mtimes(const real_T b_A_data[], const int32_T
  b_A_size[2], const real_T b_B_data[], const int32_T b_B_size[2], real_T
  b_C_data[], int32_T b_C_size[2])
{
  real_T bkj;
  int32_T aoffset;
  int32_T b_C_data_tmp;
  int32_T b_i;
  int32_T b_m;
  int32_T boffset;
  int32_T coffset;
  int32_T i;
  int32_T j;
  b_m = b_A_size[0];
  b_C_size[0] = b_A_size[0];
  b_C_size[1] = b_B_size[1];
  for (j = 0; j < b_B_size[1]; j++) {
    coffset = j * b_m;
    boffset = j * b_B_size[0];
    for (i = 0; i < b_m; i++) {
      b_C_data[coffset + i] = 0.0;
    }

    for (i = 0; i < b_A_size[1]; i++) {
      aoffset = i * b_A_size[0];
      bkj = b_B_data[boffset + i];
      for (b_i = 1; b_i - 1 < b_m; b_i++) {
        b_C_data_tmp = (coffset + b_i) - 1;
        b_C_data[b_C_data_tmp] += b_A_data[(aoffset + b_i) - 1] * bkj;
      }
    }
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_kron_b(const real_T b_A_data[], const int32_T
  b_A_size[2], real_T K_data[], int32_T K_size[2])
{
  int32_T b_j1;
  int32_T i1;
  int32_T kidx;
  K_size[0] = static_cast<int8_T>(b_A_size[0]);
  K_size[1] = static_cast<int8_T>(b_A_size[1]);
  kidx = -1;
  for (b_j1 = 0; b_j1 < b_A_size[1]; b_j1++) {
    for (i1 = 0; i1 < b_A_size[0]; i1++) {
      kidx++;
      K_data[kidx] = b_A_data[b_A_size[0] * b_j1 + i1];
    }
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_tril(real_T x_data[], const int32_T x_size[2])
{
  int32_T b_m;
  int32_T i;
  int32_T iend;
  int32_T j;
  int32_T n;
  b_m = x_size[0];
  n = x_size[1];
  if (1 < x_size[1]) {
    iend = 1;
    for (j = 2; j <= n; j++) {
      for (i = 0; i < iend; i++) {
        x_data[i + x_size[0] * (j - 1)] = 0.0;
      }

      if (iend < b_m) {
        iend++;
      }
    }
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
int32_T MPCModelClass::GenCode_M_combineVectorElements(const boolean_T x_data[],
  const int32_T *x_size)
{
  int32_T k;
  int32_T y;
  y = x_data[0];
  for (k = 2; k <= *x_size; k++) {
    y += x_data[k - 1];
  }

  return y;
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_Mrows_reshape_lgbo5(boolean_T isMrows_data[],
  real_T Mlimfull_data[], real_T Vfull_data[], const boolean_T isMrows0[70],
  const real_T Mlimfull0[70], const real_T Vfull0[70], real_T b_p, real_T ioff)
{
  real_T o_tmp;
  int32_T q;
  int32_T r;
  int32_T tmp;
  if (b_p <= 2.0) {
    if (69.0 > b_p + 68.0) {
      r = 0;
      q = 0;
    } else {
      r = 68;
      q = static_cast<int32_T>(b_p + 68.0);
    }

    tmp = q - r;
    for (q = 0; q < tmp; q++) {
      isMrows_data[static_cast<int32_T>(ioff + static_cast<real_T>(q + 1)) - 1] =
        isMrows0[r + q];
    }

    if (69.0 > b_p + 68.0) {
      r = 0;
      q = 0;
    } else {
      r = 68;
      q = static_cast<int32_T>(b_p + 68.0);
    }

    tmp = q - r;
    for (q = 0; q < tmp; q++) {
      Mlimfull_data[static_cast<int32_T>(ioff + static_cast<real_T>(q + 1)) - 1]
        = Mlimfull0[r + q];
    }

    if (69.0 > b_p + 68.0) {
      r = 0;
      q = 0;
    } else {
      r = 68;
      q = static_cast<int32_T>(b_p + 68.0);
    }

    tmp = q - r;
    for (q = 0; q < tmp; q++) {
      Vfull_data[static_cast<int32_T>(ioff + static_cast<real_T>(q + 1)) - 1] =
        Vfull0[r + q];
    }
  } else {
    isMrows_data[static_cast<int32_T>(ioff + 1.0) - 1] = isMrows0[68];
    Mlimfull_data[static_cast<int32_T>(ioff + 1.0) - 1] = Mlimfull0[68];
    Vfull_data[static_cast<int32_T>(ioff + 1.0) - 1] = Vfull0[68];
    isMrows_data[static_cast<int32_T>(ioff + 2.0) - 1] = isMrows0[69];
    Mlimfull_data[static_cast<int32_T>(ioff + 2.0) - 1] = Mlimfull0[69];
    Vfull_data[static_cast<int32_T>(ioff + 2.0) - 1] = Vfull0[69];
    o_tmp = ioff + b_p;
    if ((ioff + 2.0) + 1.0 > o_tmp) {
      r = -1;
      q = 0;
    } else {
      r = static_cast<int32_T>((ioff + 2.0) + 1.0) - 2;
      q = static_cast<int32_T>(o_tmp);
    }

    tmp = (q - r) - 1;
    for (q = 0; q < tmp; q++) {
      isMrows_data[(r + q) + 1] = isMrows0[69];
    }

    if ((ioff + 2.0) + 1.0 > o_tmp) {
      r = -1;
      q = 0;
    } else {
      r = static_cast<int32_T>((ioff + 2.0) + 1.0) - 2;
      q = static_cast<int32_T>(o_tmp);
    }

    tmp = (q - r) - 1;
    for (q = 0; q < tmp; q++) {
      Mlimfull_data[(r + q) + 1] = Mlimfull0[69];
    }

    if ((ioff + 2.0) + 1.0 > o_tmp) {
      r = -1;
      q = 0;
    } else {
      r = static_cast<int32_T>((ioff + 2.0) + 1.0) - 2;
      q = static_cast<int32_T>(o_tmp);
    }

    tmp = (q - r) - 1;
    for (q = 0; q < tmp; q++) {
      Vfull_data[(r + q) + 1] = 0.0;
    }
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_Mrows_reshape_lgbo(boolean_T isMrows_data[],
  real_T Mlimfull_data[], real_T Vfull_data[], const boolean_T isMrows0[70],
  const real_T Mlimfull0[70], const real_T Vfull0[70], real_T b_p, real_T ioff)
{
  real_T o_tmp;
  int32_T q;
  int32_T r;
  int32_T tmp;
  if (b_p <= 2.0) {
    if (67.0 > b_p + 66.0) {
      r = 0;
      q = 0;
    } else {
      r = 66;
      q = static_cast<int32_T>(b_p + 66.0);
    }

    tmp = q - r;
    for (q = 0; q < tmp; q++) {
      isMrows_data[static_cast<int32_T>(ioff + static_cast<real_T>(q + 1)) - 1] =
        isMrows0[r + q];
    }

    if (67.0 > b_p + 66.0) {
      r = 0;
      q = 0;
    } else {
      r = 66;
      q = static_cast<int32_T>(b_p + 66.0);
    }

    tmp = q - r;
    for (q = 0; q < tmp; q++) {
      Mlimfull_data[static_cast<int32_T>(ioff + static_cast<real_T>(q + 1)) - 1]
        = Mlimfull0[r + q];
    }

    if (67.0 > b_p + 66.0) {
      r = 0;
      q = 0;
    } else {
      r = 66;
      q = static_cast<int32_T>(b_p + 66.0);
    }

    tmp = q - r;
    for (q = 0; q < tmp; q++) {
      Vfull_data[static_cast<int32_T>(ioff + static_cast<real_T>(q + 1)) - 1] =
        Vfull0[r + q];
    }
  } else {
    isMrows_data[static_cast<int32_T>(ioff + 1.0) - 1] = isMrows0[66];
    Mlimfull_data[static_cast<int32_T>(ioff + 1.0) - 1] = Mlimfull0[66];
    Vfull_data[static_cast<int32_T>(ioff + 1.0) - 1] = Vfull0[66];
    isMrows_data[static_cast<int32_T>(ioff + 2.0) - 1] = isMrows0[67];
    Mlimfull_data[static_cast<int32_T>(ioff + 2.0) - 1] = Mlimfull0[67];
    Vfull_data[static_cast<int32_T>(ioff + 2.0) - 1] = Vfull0[67];
    o_tmp = ioff + b_p;
    if ((ioff + 2.0) + 1.0 > o_tmp) {
      r = -1;
      q = 0;
    } else {
      r = static_cast<int32_T>((ioff + 2.0) + 1.0) - 2;
      q = static_cast<int32_T>(o_tmp);
    }

    tmp = (q - r) - 1;
    for (q = 0; q < tmp; q++) {
      isMrows_data[(r + q) + 1] = isMrows0[67];
    }

    if ((ioff + 2.0) + 1.0 > o_tmp) {
      r = -1;
      q = 0;
    } else {
      r = static_cast<int32_T>((ioff + 2.0) + 1.0) - 2;
      q = static_cast<int32_T>(o_tmp);
    }

    tmp = (q - r) - 1;
    for (q = 0; q < tmp; q++) {
      Mlimfull_data[(r + q) + 1] = Mlimfull0[67];
    }

    if ((ioff + 2.0) + 1.0 > o_tmp) {
      r = -1;
      q = 0;
    } else {
      r = static_cast<int32_T>((ioff + 2.0) + 1.0) - 2;
      q = static_cast<int32_T>(o_tmp);
    }

    tmp = (q - r) - 1;
    for (q = 0; q < tmp; q++) {
      Vfull_data[(r + q) + 1] = 0.0;
    }
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_Mrows_reshape_lgb(boolean_T isMrows_data[],
  real_T Mlimfull_data[], real_T Vfull_data[], const boolean_T isMrows0[70],
  const real_T Mlimfull0[70], const real_T Vfull0[70], real_T b_p, real_T ioff)
{
  real_T h_tmp;
  int32_T i;
  int32_T j;
  int32_T tmp;
  if (b_p <= 11.0) {
    tmp = static_cast<int32_T>(b_p + 55.0) - 56;
    for (i = 0; i <= tmp; i++) {
      isMrows_data[static_cast<int32_T>(ioff + static_cast<real_T>(i + 1)) - 1] =
        isMrows0[i + 55];
    }

    tmp = static_cast<int32_T>(b_p + 55.0) - 56;
    for (i = 0; i <= tmp; i++) {
      Mlimfull_data[static_cast<int32_T>(ioff + static_cast<real_T>(i + 1)) - 1]
        = Mlimfull0[i + 55];
    }

    tmp = static_cast<int32_T>(b_p + 55.0) - 56;
    for (i = 0; i <= tmp; i++) {
      Vfull_data[static_cast<int32_T>(ioff + static_cast<real_T>(i + 1)) - 1] =
        Vfull0[i + 55];
    }
  } else {
    for (i = 0; i < 11; i++) {
      tmp = static_cast<int32_T>(ioff + (static_cast<real_T>(i) + 1.0)) - 1;
      isMrows_data[tmp] = isMrows0[i + 55];
      Mlimfull_data[tmp] = Mlimfull0[i + 55];
      Vfull_data[tmp] = Vfull0[i + 55];
    }

    h_tmp = ioff + b_p;
    if ((ioff + 11.0) + 1.0 > h_tmp) {
      j = -1;
      i = 0;
    } else {
      j = static_cast<int32_T>((ioff + 11.0) + 1.0) - 2;
      i = static_cast<int32_T>(h_tmp);
    }

    tmp = (i - j) - 1;
    for (i = 0; i < tmp; i++) {
      isMrows_data[(j + i) + 1] = isMrows0[65];
    }

    if ((ioff + 11.0) + 1.0 > h_tmp) {
      j = -1;
      i = 0;
    } else {
      j = static_cast<int32_T>((ioff + 11.0) + 1.0) - 2;
      i = static_cast<int32_T>(h_tmp);
    }

    tmp = (i - j) - 1;
    for (i = 0; i < tmp; i++) {
      Mlimfull_data[(j + i) + 1] = Mlimfull0[65];
    }

    if ((ioff + 11.0) + 1.0 > h_tmp) {
      j = -1;
      i = 0;
    } else {
      j = static_cast<int32_T>((ioff + 11.0) + 1.0) - 2;
      i = static_cast<int32_T>(h_tmp);
    }

    tmp = (i - j) - 1;
    for (i = 0; i < tmp; i++) {
      Vfull_data[(j + i) + 1] = 0.0;
    }
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_Mrows_reshape_lg(boolean_T isMrows_data[],
  real_T Mlimfull_data[], real_T Vfull_data[], const boolean_T isMrows0[70],
  const real_T Mlimfull0[70], const real_T Vfull0[70], real_T b_p, real_T ioff)
{
  real_T h_tmp;
  int32_T i;
  int32_T j;
  int32_T tmp;
  if (b_p <= 11.0) {
    tmp = static_cast<int32_T>(b_p + 44.0) - 45;
    for (i = 0; i <= tmp; i++) {
      isMrows_data[static_cast<int32_T>(ioff + static_cast<real_T>(i + 1)) - 1] =
        isMrows0[i + 44];
    }

    tmp = static_cast<int32_T>(b_p + 44.0) - 45;
    for (i = 0; i <= tmp; i++) {
      Mlimfull_data[static_cast<int32_T>(ioff + static_cast<real_T>(i + 1)) - 1]
        = Mlimfull0[i + 44];
    }

    tmp = static_cast<int32_T>(b_p + 44.0) - 45;
    for (i = 0; i <= tmp; i++) {
      Vfull_data[static_cast<int32_T>(ioff + static_cast<real_T>(i + 1)) - 1] =
        Vfull0[i + 44];
    }
  } else {
    for (i = 0; i < 11; i++) {
      tmp = static_cast<int32_T>(ioff + (static_cast<real_T>(i) + 1.0)) - 1;
      isMrows_data[tmp] = isMrows0[i + 44];
      Mlimfull_data[tmp] = Mlimfull0[i + 44];
      Vfull_data[tmp] = Vfull0[i + 44];
    }

    h_tmp = ioff + b_p;
    if ((ioff + 11.0) + 1.0 > h_tmp) {
      j = -1;
      i = 0;
    } else {
      j = static_cast<int32_T>((ioff + 11.0) + 1.0) - 2;
      i = static_cast<int32_T>(h_tmp);
    }

    tmp = (i - j) - 1;
    for (i = 0; i < tmp; i++) {
      isMrows_data[(j + i) + 1] = isMrows0[54];
    }

    if ((ioff + 11.0) + 1.0 > h_tmp) {
      j = -1;
      i = 0;
    } else {
      j = static_cast<int32_T>((ioff + 11.0) + 1.0) - 2;
      i = static_cast<int32_T>(h_tmp);
    }

    tmp = (i - j) - 1;
    for (i = 0; i < tmp; i++) {
      Mlimfull_data[(j + i) + 1] = Mlimfull0[54];
    }

    if ((ioff + 11.0) + 1.0 > h_tmp) {
      j = -1;
      i = 0;
    } else {
      j = static_cast<int32_T>((ioff + 11.0) + 1.0) - 2;
      i = static_cast<int32_T>(h_tmp);
    }

    tmp = (i - j) - 1;
    for (i = 0; i < tmp; i++) {
      Vfull_data[(j + i) + 1] = 0.0;
    }
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_repmat_d(const boolean_T a[2], real_T varargin_1,
  boolean_T b_data[], int32_T *b_size)
{
  int32_T ibcol;
  int32_T itilerow;
  *b_size = static_cast<int8_T>(static_cast<int32_T>(varargin_1) << 1);
  for (itilerow = 0; itilerow < static_cast<int32_T>(varargin_1); itilerow++) {
    ibcol = itilerow << 1;
    b_data[ibcol] = a[0];
    b_data[ibcol + 1] = a[1];
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_repmat_dl(const real_T a[2], real_T varargin_1,
  real_T b_data[], int32_T *b_size)
{
  int32_T ibcol;
  int32_T itilerow;
  *b_size = static_cast<int8_T>(static_cast<int32_T>(varargin_1) << 1);
  for (itilerow = 0; itilerow < static_cast<int32_T>(varargin_1); itilerow++) {
    ibcol = itilerow << 1;
    b_data[ibcol] = a[0];
    b_data[ibcol + 1] = a[1];
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_Mrows_reshape_l(boolean_T isMrows_data[], real_T
  Mlimfull_data[], real_T Vfull_data[], const boolean_T isMrows0[70], const
  real_T Mlimfull0[70], const real_T Vfull0[70], real_T b_p, real_T ioff)
{
  real_T tmp_data_0[18];
  real_T h_tmp;
  int32_T i;
  int32_T j;
  int32_T tmp;
  boolean_T tmp_data[18];
  if (b_p <= 11.0) {
    tmp = static_cast<int32_T>(b_p * 2.0 + 22.0) - 23;
    for (i = 0; i <= tmp; i++) {
      isMrows_data[static_cast<int32_T>(ioff + static_cast<real_T>(i + 1)) - 1] =
        isMrows0[i + 22];
    }

    for (i = 0; i <= tmp; i++) {
      Mlimfull_data[static_cast<int32_T>(ioff + static_cast<real_T>(i + 1)) - 1]
        = Mlimfull0[i + 22];
    }

    for (i = 0; i <= tmp; i++) {
      Vfull_data[static_cast<int32_T>(ioff + static_cast<real_T>(i + 1)) - 1] =
        Vfull0[i + 22];
    }
  } else {
    for (i = 0; i < 22; i++) {
      tmp = static_cast<int32_T>(ioff + (static_cast<real_T>(i) + 1.0)) - 1;
      isMrows_data[tmp] = isMrows0[i + 22];
      Mlimfull_data[tmp] = Mlimfull0[i + 22];
      Vfull_data[tmp] = Vfull0[i + 22];
    }

    h_tmp = b_p * 2.0 + ioff;
    if ((ioff + 22.0) + 1.0 > h_tmp) {
      j = -1;
      i = 0;
    } else {
      j = static_cast<int32_T>((ioff + 22.0) + 1.0) - 2;
      i = static_cast<int32_T>(h_tmp);
    }

    GenCode_MPC_repmat_d(&isMrows0[42], b_p - 11.0, tmp_data, &tmp);
    tmp = (i - j) - 1;
    for (i = 0; i < tmp; i++) {
      isMrows_data[(j + i) + 1] = tmp_data[i];
    }

    if ((ioff + 22.0) + 1.0 > h_tmp) {
      j = -1;
      i = 0;
    } else {
      j = static_cast<int32_T>((ioff + 22.0) + 1.0) - 2;
      i = static_cast<int32_T>(h_tmp);
    }

    GenCode_MPC_repmat_dl(&Mlimfull0[42], b_p - 11.0, tmp_data_0, &tmp);
    tmp = (i - j) - 1;
    for (i = 0; i < tmp; i++) {
      Mlimfull_data[(j + i) + 1] = tmp_data_0[i];
    }

    if ((ioff + 22.0) + 1.0 > h_tmp) {
      j = -1;
      i = 0;
    } else {
      j = static_cast<int32_T>((ioff + 22.0) + 1.0) - 2;
      i = static_cast<int32_T>(h_tmp);
    }

    GenCode_MPC_repmat_dl(&Vfull0[42], b_p - 11.0, tmp_data_0, &tmp);
    tmp = (i - j) - 1;
    for (i = 0; i < tmp; i++) {
      Vfull_data[(j + i) + 1] = tmp_data_0[i];
    }
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_Mrows_reshape(boolean_T isMrows_data[], real_T
  Mlimfull_data[], real_T Vfull_data[], const boolean_T isMrows0[70], const
  real_T Mlimfull0[70], const real_T Vfull0[70], real_T b_p)
{
  real_T tmp_data_0[18];
  int32_T d_tmp;
  int32_T tmp_size;
  boolean_T tmp_data[18];
  if (b_p <= 11.0) {
    d_tmp = static_cast<int32_T>(b_p * 2.0);
    if (0 <= d_tmp - 1) {
      std::memcpy(&isMrows_data[0], &isMrows0[0], d_tmp * sizeof(boolean_T));
      std::memcpy(&Mlimfull_data[0], &Mlimfull0[0], d_tmp * sizeof(real_T));
      std::memcpy(&Vfull_data[0], &Vfull0[0], d_tmp * sizeof(real_T));
    }
  } else {
    std::memcpy(&Mlimfull_data[0], &Mlimfull0[0], 22U * sizeof(real_T));
    std::memcpy(&Vfull_data[0], &Vfull0[0], 22U * sizeof(real_T));
    for (d_tmp = 0; d_tmp < 22; d_tmp++) {
      isMrows_data[d_tmp] = isMrows0[d_tmp];
    }

    d_tmp = static_cast<int32_T>(b_p * 2.0) - 22;
    GenCode_MPC_repmat_d(&isMrows0[20], b_p - 11.0, tmp_data, &tmp_size);
    GenCode_MPC_repmat_dl(&Mlimfull0[20], b_p - 11.0, tmp_data_0, &tmp_size);
    if (0 <= d_tmp - 1) {
      std::memcpy(&isMrows_data[22], &tmp_data[0], d_tmp * sizeof(boolean_T));
      std::memcpy(&Mlimfull_data[22], &tmp_data_0[0], d_tmp * sizeof(real_T));
    }

    GenCode_MPC_repmat_dl(&Vfull0[20], b_p - 11.0, tmp_data_0, &tmp_size);
    if (0 <= d_tmp - 1) {
      std::memcpy(&Vfull_data[22], &tmp_data_0[0], d_tmp * sizeof(real_T));
    }
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_mpc_constraintcoef(const real_T b_A[25], const
  real_T Bu[5], const real_T Bv[10], const real_T b_C[10], const real_T Dv[4],
  const real_T Jm_data[], const int32_T Jm_size[2], real_T SuJm_data[], int32_T
  SuJm_size[2], real_T Sx_data[], int32_T Sx_size[2], real_T Su1_data[], int32_T
  *Su1_size, real_T Hv_data[], int32_T Hv_size[2])
{
  real_T varargin_1_data[84];
  real_T Sum_data[40];
  real_T CA[10];
  real_T CA_0[10];
  real_T b_C_0[4];
  real_T Sum[2];
  real_T tmp;
  int32_T Su_size[2];
  int32_T CA_size_idx_1;
  int32_T CA_tmp;
  int32_T Hv_data_tmp;
  int32_T Hv_data_tmp_0;
  int32_T Sum_size_idx_1;
  int32_T b_C_tmp;
  int32_T b_p;
  int32_T i_0;
  int32_T loop_ub;
  int32_T loop_ub_0;
  int32_T loop_ub_1;
  int32_T pny1_tmp;
  int32_T varargin_1_size_idx_1;
  int8_T rows[2];
  int8_T i;
  b_p = Jm_size[0] - 2;
  pny1_tmp = (Jm_size[0] - 1) << 1;
  for (i_0 = 0; i_0 < 2; i_0++) {
    Sum[i_0] = 0.0;
    for (Hv_data_tmp = 0; Hv_data_tmp < 5; Hv_data_tmp++) {
      CA_tmp = i_0 + (Hv_data_tmp << 1);
      CA[CA_tmp] = 0.0;
      for (Hv_data_tmp_0 = 0; Hv_data_tmp_0 < 5; Hv_data_tmp_0++) {
        CA[CA_tmp] += b_C[(Hv_data_tmp_0 << 1) + i_0] * b_A[5 * Hv_data_tmp +
          Hv_data_tmp_0];
      }

      Sum[i_0] += b_C[CA_tmp] * Bu[Hv_data_tmp];
    }

    for (Hv_data_tmp = 0; Hv_data_tmp < 2; Hv_data_tmp++) {
      b_C_tmp = i_0 + (Hv_data_tmp << 1);
      b_C_0[b_C_tmp] = 0.0;
      for (Hv_data_tmp_0 = 0; Hv_data_tmp_0 < 5; Hv_data_tmp_0++) {
        b_C_0[b_C_tmp] += b_C[(Hv_data_tmp_0 << 1) + i_0] * Bv[5 * Hv_data_tmp +
          Hv_data_tmp_0];
      }
    }
  }

  varargin_1_size_idx_1 = pny1_tmp + 4;
  varargin_1_data[0] = b_C_0[0];
  varargin_1_data[4] = Dv[0];
  varargin_1_data[1] = b_C_0[1];
  varargin_1_data[5] = Dv[1];
  varargin_1_data[2] = b_C_0[2];
  varargin_1_data[6] = Dv[2];
  varargin_1_data[3] = b_C_0[3];
  varargin_1_data[7] = Dv[3];
  loop_ub = pny1_tmp << 1;
  for (i_0 = 0; i_0 < loop_ub; i_0++) {
    varargin_1_data[i_0 + 8] = 0.0;
  }

  if (pny1_tmp != 0) {
    b_C_tmp = pny1_tmp;
  } else {
    b_C_tmp = 0;
  }

  Hv_size[0] = b_C_tmp + 2;
  Hv_size[1] = pny1_tmp + 4;
  for (i_0 = 0; i_0 < varargin_1_size_idx_1; i_0++) {
    Hv_data_tmp = i_0 << 1;
    Hv_data_tmp_0 = (b_C_tmp + 2) * i_0;
    Hv_data[Hv_data_tmp_0] = varargin_1_data[Hv_data_tmp];
    Hv_data[Hv_data_tmp_0 + 1] = varargin_1_data[Hv_data_tmp + 1];
  }

  for (i_0 = 0; i_0 < varargin_1_size_idx_1; i_0++) {
    for (Hv_data_tmp = 0; Hv_data_tmp < b_C_tmp; Hv_data_tmp++) {
      Hv_data[(Hv_data_tmp + (b_C_tmp + 2) * i_0) + 2] = 0.0;
    }
  }

  Sx_size[0] = pny1_tmp + 2;
  Sx_size[1] = 5;
  for (i_0 = 0; i_0 < 5; i_0++) {
    Hv_data_tmp = i_0 << 1;
    Hv_data_tmp_0 = (pny1_tmp + 2) * i_0;
    Sx_data[Hv_data_tmp_0] = CA[Hv_data_tmp];
    Sx_data[Hv_data_tmp_0 + 1] = CA[Hv_data_tmp + 1];
  }

  for (i_0 = 0; i_0 < 5; i_0++) {
    for (Hv_data_tmp = 0; Hv_data_tmp < pny1_tmp; Hv_data_tmp++) {
      Sx_data[(Hv_data_tmp + (pny1_tmp + 2) * i_0) + 2] = 0.0;
    }
  }

  *Su1_size = pny1_tmp + 2;
  Su1_data[0] = Sum[0];
  Su1_data[1] = Sum[1];
  for (i_0 = 0; i_0 < pny1_tmp; i_0++) {
    Su1_data[i_0 + 2] = 0.0;
  }

  varargin_1_size_idx_1 = Jm_size[0];
  loop_ub = pny1_tmp - 1;
  varargin_1_data[0] = Sum[0];
  varargin_1_data[1] = Sum[1];
  for (i_0 = 0; i_0 <= loop_ub; i_0++) {
    varargin_1_data[i_0 + 2] = 0.0;
  }

  if ((pny1_tmp != 0) && (Jm_size[0] != 0)) {
    loop_ub = pny1_tmp;
  } else {
    loop_ub = 0;
  }

  Su_size[0] = loop_ub + 2;
  Su_size[1] = Jm_size[0];
  for (i_0 = 0; i_0 < varargin_1_size_idx_1; i_0++) {
    Hv_data_tmp = i_0 << 1;
    Hv_data_tmp_0 = (loop_ub + 2) * i_0;
    GenCode_MPC_B.Su_data[Hv_data_tmp_0] = varargin_1_data[Hv_data_tmp];
    GenCode_MPC_B.Su_data[Hv_data_tmp_0 + 1] = varargin_1_data[Hv_data_tmp + 1];
  }

  for (i_0 = 0; i_0 < varargin_1_size_idx_1; i_0++) {
    for (Hv_data_tmp = 0; Hv_data_tmp < loop_ub; Hv_data_tmp++) {
      GenCode_MPC_B.Su_data[(Hv_data_tmp + (loop_ub + 2) * i_0) + 2] = 0.0;
    }
  }

  if (0 <= Jm_size[0] - 2) {
    if (1 > Jm_size[0] - 1) {
      loop_ub_0 = -1;
    } else {
      loop_ub_0 = Jm_size[0] - 2;
    }

    Sum_size_idx_1 = loop_ub_0 + 2;
    loop_ub_1 = Jm_size[0] << 1;
    CA_size_idx_1 = loop_ub_1 + 2;
  }

  for (varargin_1_size_idx_1 = 0; varargin_1_size_idx_1 <= b_p;
       varargin_1_size_idx_1++) {
    i = static_cast<int8_T>(static_cast<int8_T>(static_cast<int8_T>
      (varargin_1_size_idx_1 + 1) << 1) + 1);
    for (i_0 = 0; i_0 < 2; i_0++) {
      rows[i_0] = static_cast<int8_T>(i_0 + i);
      tmp = 0.0;
      for (Hv_data_tmp = 0; Hv_data_tmp < 5; Hv_data_tmp++) {
        tmp += CA[(Hv_data_tmp << 1) + i_0] * Bu[Hv_data_tmp];
      }

      Sum[i_0] += tmp;
    }

    Su1_data[rows[0] - 1] = Sum[0];
    Sum_data[0] = Sum[0];
    Su1_data[rows[1] - 1] = Sum[1];
    Sum_data[1] = Sum[1];
    for (i_0 = 0; i_0 <= loop_ub_0; i_0++) {
      Hv_data_tmp = (loop_ub + 2) * i_0;
      Hv_data_tmp_0 = (i_0 + 1) << 1;
      Sum_data[Hv_data_tmp_0] = GenCode_MPC_B.Su_data[(Hv_data_tmp + rows[0]) -
        3];
      Sum_data[Hv_data_tmp_0 + 1] = GenCode_MPC_B.Su_data[(Hv_data_tmp + rows[1])
        - 3];
    }

    for (i_0 = 0; i_0 < Sum_size_idx_1; i_0++) {
      Hv_data_tmp = i_0 << 1;
      Hv_data_tmp_0 = (loop_ub + 2) * i_0;
      GenCode_MPC_B.Su_data[(rows[0] + Hv_data_tmp_0) - 1] =
        Sum_data[Hv_data_tmp];
      GenCode_MPC_B.Su_data[(rows[1] + Hv_data_tmp_0) - 1] =
        Sum_data[Hv_data_tmp + 1];
    }

    for (i_0 = 0; i_0 < 2; i_0++) {
      for (Hv_data_tmp = 0; Hv_data_tmp < 2; Hv_data_tmp++) {
        CA_tmp = i_0 + (Hv_data_tmp << 1);
        b_C_0[CA_tmp] = 0.0;
        for (Hv_data_tmp_0 = 0; Hv_data_tmp_0 < 5; Hv_data_tmp_0++) {
          b_C_0[CA_tmp] += CA[(Hv_data_tmp_0 << 1) + i_0] * Bv[5 * Hv_data_tmp +
            Hv_data_tmp_0];
        }
      }
    }

    varargin_1_data[0] = b_C_0[0];
    varargin_1_data[1] = b_C_0[1];
    varargin_1_data[2] = b_C_0[2];
    varargin_1_data[3] = b_C_0[3];
    for (i_0 = 0; i_0 < loop_ub_1; i_0++) {
      Hv_data_tmp = (b_C_tmp + 2) * i_0;
      Hv_data_tmp_0 = (i_0 + 2) << 1;
      varargin_1_data[Hv_data_tmp_0] = Hv_data[(Hv_data_tmp + rows[0]) - 3];
      varargin_1_data[Hv_data_tmp_0 + 1] = Hv_data[(Hv_data_tmp + rows[1]) - 3];
    }

    for (i_0 = 0; i_0 < CA_size_idx_1; i_0++) {
      Hv_data_tmp = i_0 << 1;
      Hv_data_tmp_0 = (b_C_tmp + 2) * i_0;
      Hv_data[(rows[0] + Hv_data_tmp_0) - 1] = varargin_1_data[Hv_data_tmp];
      Hv_data[(rows[1] + Hv_data_tmp_0) - 1] = varargin_1_data[Hv_data_tmp + 1];
    }

    for (i_0 = 0; i_0 < 2; i_0++) {
      for (Hv_data_tmp = 0; Hv_data_tmp < 5; Hv_data_tmp++) {
        CA_tmp = i_0 + (Hv_data_tmp << 1);
        CA_0[CA_tmp] = 0.0;
        for (Hv_data_tmp_0 = 0; Hv_data_tmp_0 < 5; Hv_data_tmp_0++) {
          CA_0[CA_tmp] += CA[(Hv_data_tmp_0 << 1) + i_0] * b_A[5 * Hv_data_tmp +
            Hv_data_tmp_0];
        }
      }
    }

    std::memcpy(&CA[0], &CA_0[0], 10U * sizeof(real_T));
    for (i_0 = 0; i_0 < 5; i_0++) {
      Hv_data_tmp = i_0 << 1;
      Hv_data_tmp_0 = (pny1_tmp + 2) * i_0;
      Sx_data[(rows[0] + Hv_data_tmp_0) - 1] = CA[Hv_data_tmp];
      Sx_data[(rows[1] + Hv_data_tmp_0) - 1] = CA[Hv_data_tmp + 1];
    }
  }

  GenCode_MPC_mtimes(GenCode_MPC_B.Su_data, Su_size, Jm_data, Jm_size, SuJm_data,
                     SuJm_size);
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_repmat(real_T a, real_T varargin_1, real_T
  b_data[], int32_T *b_size)
{
  int32_T i;
  int32_T loop_ub;
  *b_size = static_cast<int8_T>(static_cast<int32_T>(varargin_1));
  loop_ub = static_cast<int8_T>(static_cast<int32_T>(varargin_1));
  for (i = 0; i < loop_ub; i++) {
    b_data[i] = a;
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MPC_kron(const int32_T *b_A_size, real_T K_data[],
  int32_T *K_size)
{
  int32_T i1;
  int32_T kidx;
  *K_size = *b_A_size;
  kidx = -1;
  for (i1 = 0; i1 < *b_A_size; i1++) {
    kidx++;
    K_data[kidx] = 1.0;
  }
}

// Function for MATLAB Function: '<S35>/VariableHorizonOptimizer'
void MPCModelClass::GenCode_MP_mpcblock_optimizerPM(const
  emxArray_real_T_GenCode_MPC_T *rseq, const emxArray_real_T_GenCode_MPC_T *vseq,
  real_T umin, real_T umax, real_T switch_in, const real_T x[5], real_T old_u,
  const real_T Mlim0[4], const real_T utargetseq[11], real_T b_p, real_T moves,
  real_T b_uoff, const real_T ywt[2], real_T uwt, const real_T b_A[25], const
  emxArray_real_T_GenCode_MPC_T *Bu, const emxArray_real_T_GenCode_MPC_T *Bv,
  const real_T b_C[10], const emxArray_real_T_GenCode_MPC_T *Dv, real_T *u,
  real_T useq[21], real_T *status)
{
  emxArray_boolean_T_GenCode_MP_T *tmp_1;
  emxArray_real_T_GenCode_MPC_T *b_Ac;
  emxArray_real_T_GenCode_MPC_T *b_Mlim;
  emxArray_real_T_GenCode_MPC_T *b_Mrows;
  emxArray_real_T_GenCode_MPC_T *b_Mu1;
  emxArray_real_T_GenCode_MPC_T *b_Mv;
  emxArray_real_T_GenCode_MPC_T *b_Mx;
  emxArray_real_T_GenCode_MPC_T *tmp;
  emxArray_real_T_GenCode_MPC_T *tmp_0;
  real_T b_H_data[441];
  real_T b_Linv_data[441];
  real_T tmp_data_0[441];
  real_T I2Jm_data[400];
  real_T I3_data[400];
  real_T Jm_data[400];
  real_T varargin_4_data[400];
  real_T varargin_6_data[400];
  real_T Sx_data[200];
  real_T Mlimfull_data[160];
  real_T Mlimfull_data_0[160];
  real_T Vfull_data[160];
  real_T Kx_data[100];
  real_T Mlimfull0[70];
  real_T Vfull0[70];
  real_T Su1_data[40];
  real_T f_data[21];
  real_T zopt_data[21];
  real_T I1_data[20];
  real_T Ku1_data[20];
  real_T b_utarget_data[20];
  real_T tmp_data[18];
  real_T bb[2];
  real_T j;
  real_T nmoves;
  real_T nr;
  real_T tmp_2;
  int32_T Hv_size[2];
  int32_T I3_size[2];
  int32_T Jm_size[2];
  int32_T Kv_size[2];
  int32_T Kx_size[2];
  int32_T SuJm_size[2];
  int32_T Sx_size[2];
  int32_T b_H_size[2];
  int32_T tmp_size_0[2];
  int32_T tmp_size_1[2];
  int32_T varargin_2_size[2];
  int32_T y_size[2];
  int32_T I1_size;
  int32_T b_i;
  int32_T d;
  int32_T e_i;
  int32_T empty_non_axis_sizes_idx_0;
  int32_T empty_non_axis_sizes_idx_0_0;
  int32_T empty_non_axis_sizes_idx_0_1;
  int32_T f_i;
  int32_T i;
  int32_T isMrows_size;
  int32_T loop_ub;
  int32_T pny;
  int32_T pny_tmp;
  int32_T tmp_size;
  int32_T trueCount;
  uint8_T o_data[160];
  uint8_T q_data[160];
  uint8_T r_data[160];
  uint8_T s_data[160];
  uint8_T t_data[160];
  uint8_T v_data[160];
  boolean_T isMrows_data[160];
  boolean_T isMrows0[70];
  boolean_T exitg1;
  boolean_T umax_incr_flag;
  boolean_T umin_incr_flag;
  for (i = 0; i < 21; i++) {
    useq[i] = 0.0;
  }

  *status = 1.0;
  if (switch_in != 0.0) {
    *u = old_u + b_uoff;
    for (i = 0; i < 21; i++) {
      useq[i] = *u;
    }
  } else {
    if (!(b_p < 20.0)) {
      b_p = 20.0;
    }

    if ((1.0 > b_p) || rtIsNaN(b_p)) {
      b_p = 1.0;
    }

    b_p = std::ceil(b_p);
    pny_tmp = static_cast<int32_T>(b_p);
    pny = pny_tmp << 1;
    if ((b_p < moves) || rtIsNaN(moves)) {
      nmoves = b_p;
    } else {
      nmoves = moves;
    }

    Jm_size[0] = pny_tmp;
    Jm_size[1] = static_cast<int32_T>(nmoves);
    loop_ub = pny_tmp * static_cast<int32_T>(nmoves) - 1;
    for (b_i = 0; b_i <= loop_ub; b_i++) {
      Jm_data[b_i] = 0.0;
    }

    j = 0.0;
    for (i = 0; i < static_cast<int32_T>(nmoves); i++) {
      Jm_data[(static_cast<int32_T>(j + 1.0) + pny_tmp * i) - 1] = 1.0;
      j++;
    }

    tmp_size = pny_tmp;
    GenCode_MPC_kron(&pny_tmp, I1_data, &I1_size);
    for (b_i = 0; b_i < pny_tmp; b_i++) {
      b_utarget_data[b_i] = 0.0;
    }

    if (b_p <= 11.0) {
      if (0 <= pny_tmp - 1) {
        std::memcpy(&b_utarget_data[0], &utargetseq[0], pny_tmp * sizeof(real_T));
      }
    } else {
      std::memcpy(&b_utarget_data[0], &utargetseq[0], 11U * sizeof(real_T));
      GenCode_MPC_repmat(utargetseq[10], b_p - 11.0, tmp_data, &tmp_size);
      if (0 <= (pny_tmp - 11) - 1) {
        std::memcpy(&b_utarget_data[11], &tmp_data[0], (pny_tmp - 11) * sizeof
                    (real_T));
      }
    }

    GenCode_MPC_mpc_constraintcoef(b_A, &Bu->data[0], &Bv->data[0], b_C,
      &Dv->data[0], Jm_data, Jm_size, GenCode_MPC_B.SuJm_data, SuJm_size,
      Sx_data, Sx_size, Su1_data, &tmp_size, GenCode_MPC_B.Hv_data, Hv_size);
    nr = 2.0 * b_p * 3.0 + 2.0 * nmoves;
    for (i = 0; i < 70; i++) {
      isMrows0[i] = false;
      Mlimfull0[i] = 0.0;
      Vfull0[i] = 0.0;
    }

    isMrows0[44] = true;
    isMrows0[45] = true;
    isMrows0[55] = true;
    isMrows0[56] = true;
    Mlimfull0[44] = Mlim0[0];
    Mlimfull0[45] = Mlim0[1];
    Mlimfull0[55] = Mlim0[2];
    Mlimfull0[56] = Mlim0[3];
    Vfull0[44] = 0.0;
    Vfull0[45] = 0.0;
    Vfull0[55] = 0.0;
    Vfull0[56] = 0.0;
    isMrows_size = static_cast<int32_T>(nr);
    if (0 <= static_cast<int32_T>(nr) - 1) {
      std::memset(&isMrows_data[0], 0, static_cast<int32_T>(nr) * sizeof
                  (boolean_T));
    }

    loop_ub = static_cast<int32_T>(nr);
    for (b_i = 0; b_i < loop_ub; b_i++) {
      Mlimfull_data[b_i] = 0.0;
    }

    loop_ub = static_cast<int32_T>(nr);
    for (b_i = 0; b_i < loop_ub; b_i++) {
      Vfull_data[b_i] = 0.0;
    }

    GenCode_MPC_emxInit_real_T(&b_Mrows, 1);
    GenCode_MPC_Mrows_reshape(isMrows_data, Mlimfull_data, Vfull_data, isMrows0,
      Mlimfull0, Vfull0, b_p);
    j = b_p * 2.0;
    GenCode_MPC_Mrows_reshape_l(isMrows_data, Mlimfull_data, Vfull_data,
      isMrows0, Mlimfull0, Vfull0, b_p, j);
    j += b_p * 2.0;
    GenCode_MPC_Mrows_reshape_lg(isMrows_data, Mlimfull_data, Vfull_data,
      isMrows0, Mlimfull0, Vfull0, b_p, j);
    j += b_p;
    GenCode_MPC_Mrows_reshape_lgb(isMrows_data, Mlimfull_data, Vfull_data,
      isMrows0, Mlimfull0, Vfull0, b_p, j);
    j += b_p;
    GenCode_MPC_Mrows_reshape_lgbo(isMrows_data, Mlimfull_data, Vfull_data,
      isMrows0, Mlimfull0, Vfull0, nmoves, j);
    j += nmoves;
    GenCode_MPC_Mrows_reshape_lgbo5(isMrows_data, Mlimfull_data, Vfull_data,
      isMrows0, Mlimfull0, Vfull0, nmoves, j);
    i = GenCode_M_combineVectorElements(isMrows_data, &isMrows_size);
    b_i = b_Mrows->size[0];
    b_Mrows->size[0] = i;
    GenCod_emxEnsureCapacity_real_T(b_Mrows, b_i);
    for (b_i = 0; b_i < i; b_i++) {
      b_Mrows->data[b_i] = 0.0;
    }

    j = 0.0;
    for (b_i = 0; b_i < static_cast<int32_T>(nr); b_i++) {
      if (isMrows_data[b_i]) {
        j++;
        b_Mrows->data[static_cast<int32_T>(j) - 1] = static_cast<real_T>(b_i) +
          1.0;
      }
    }

    GenCode_MPC_emxInit_real_T(&b_Ac, 2);
    b_i = b_Ac->size[0] * b_Ac->size[1];
    b_Ac->size[0] = i;
    b_Ac->size[1] = static_cast<int32_T>(nmoves + 1.0);
    GenCod_emxEnsureCapacity_real_T(b_Ac, b_i);
    loop_ub = static_cast<int32_T>(nmoves + 1.0) * i - 1;
    for (b_i = 0; b_i <= loop_ub; b_i++) {
      b_Ac->data[b_i] = 0.0;
    }

    GenCode_MPC_emxInit_real_T(&b_Mx, 2);
    b_i = b_Mx->size[0] * b_Mx->size[1];
    b_Mx->size[0] = i;
    b_Mx->size[1] = 5;
    GenCod_emxEnsureCapacity_real_T(b_Mx, b_i);
    loop_ub = i * 5 - 1;
    for (b_i = 0; b_i <= loop_ub; b_i++) {
      b_Mx->data[b_i] = 0.0;
    }

    GenCode_MPC_emxInit_real_T(&b_Mu1, 1);
    b_i = b_Mu1->size[0];
    b_Mu1->size[0] = i;
    GenCod_emxEnsureCapacity_real_T(b_Mu1, b_i);
    for (b_i = 0; b_i < i; b_i++) {
      b_Mu1->data[b_i] = 0.0;
    }

    GenCode_MPC_emxInit_real_T(&b_Mv, 2);
    b_i = b_Mv->size[0] * b_Mv->size[1];
    b_Mv->size[0] = i;
    b_Mv->size[1] = Hv_size[1];
    GenCod_emxEnsureCapacity_real_T(b_Mv, b_i);
    loop_ub = i * Hv_size[1] - 1;
    for (b_i = 0; b_i <= loop_ub; b_i++) {
      b_Mv->data[b_i] = 0.0;
    }

    GenCode_MPC_emxInit_real_T(&b_Mlim, 1);
    b_i = b_Mlim->size[0];
    b_Mlim->size[0] = i;
    GenCod_emxEnsureCapacity_real_T(b_Mlim, b_i);
    for (b_i = 0; b_i < i; b_i++) {
      b_Mlim->data[b_i] = 0.0;
    }

    if (1 > i) {
      b_i = -1;
    } else {
      b_i = i - 1;
    }

    d = b_i + 1;
    trueCount = 0;
    for (e_i = 0; e_i < static_cast<int32_T>(nr); e_i++) {
      if (isMrows_data[e_i]) {
        trueCount++;
      }
    }

    e_i = 0;
    for (f_i = 0; f_i < static_cast<int32_T>(nr); f_i++) {
      if (isMrows_data[f_i]) {
        v_data[e_i] = static_cast<uint8_T>(f_i + 1);
        e_i++;
      }
    }

    for (b_i = 0; b_i < trueCount; b_i++) {
      Mlimfull_data_0[b_i] = Mlimfull_data[v_data[b_i] - 1];
    }

    for (b_i = 0; b_i < d; b_i++) {
      b_Mlim->data[b_i] = Mlimfull_data_0[b_i];
    }

    I3_size[0] = pny_tmp;
    I3_size[1] = pny_tmp;
    d = static_cast<int32_T>(static_cast<real32_T>(b_p) * static_cast<real32_T>
      (b_p)) - 1;
    for (b_i = 0; b_i <= d; b_i++) {
      I3_data[b_i] = 1.0;
    }

    GenCode_MPC_tril(I3_data, I3_size);
    GenCode_MPC_kron_b(I3_data, I3_size, I2Jm_data, tmp_size_0);
    GenCode_MPC_mtimes(I2Jm_data, tmp_size_0, Jm_data, Jm_size,
                       GenCode_MPC_B.y_data, y_size);
    I3_size[0] = pny_tmp;
    I3_size[1] = pny_tmp;
    for (b_i = 0; b_i <= d; b_i++) {
      I3_data[b_i] = 1.0;
    }

    GenCode_MPC_tril(I3_data, I3_size);
    GenCode_MPC_kron_b(I3_data, I3_size, I2Jm_data, tmp_size_0);
    GenCode_MPC_mtimes(I2Jm_data, tmp_size_0, Jm_data, Jm_size,
                       GenCode_MPC_B.tmp_data_cx, tmp_size_1);
    loop_ub = tmp_size_1[0] * tmp_size_1[1];
    if (0 <= loop_ub - 1) {
      std::memcpy(&I2Jm_data[0], &GenCode_MPC_B.tmp_data_cx[0], loop_ub * sizeof
                  (real_T));
    }

    GenCode_MPC_eye(nmoves, I3_data, I3_size);
    loop_ub = SuJm_size[0] * SuJm_size[1] - 1;
    for (b_i = 0; b_i <= loop_ub; b_i++) {
      GenCode_MPC_B.tmp_data_cx[b_i] = -GenCode_MPC_B.SuJm_data[b_i];
    }

    loop_ub = y_size[0] * y_size[1] - 1;
    for (b_i = 0; b_i <= loop_ub; b_i++) {
      varargin_4_data[b_i] = -GenCode_MPC_B.y_data[b_i];
    }

    loop_ub = I3_size[0] * I3_size[1] - 1;
    for (b_i = 0; b_i <= loop_ub; b_i++) {
      varargin_6_data[b_i] = -I3_data[b_i];
    }

    if ((SuJm_size[0] != 0) && (SuJm_size[1] != 0)) {
      e_i = SuJm_size[1];
    } else if ((SuJm_size[0] != 0) && (SuJm_size[1] != 0)) {
      e_i = SuJm_size[1];
    } else if ((tmp_size_1[0] != 0) && (tmp_size_1[1] != 0)) {
      e_i = y_size[1];
    } else if ((y_size[0] != 0) && (y_size[1] != 0)) {
      e_i = y_size[1];
    } else if ((I3_size[0] != 0) && (I3_size[1] != 0)) {
      e_i = I3_size[1];
    } else if ((I3_size[0] != 0) && (I3_size[1] != 0)) {
      e_i = I3_size[1];
    } else {
      if (SuJm_size[1] > 0) {
        e_i = SuJm_size[1];
      } else {
        e_i = 0;
      }

      if (SuJm_size[1] > e_i) {
        e_i = SuJm_size[1];
      }

      if (y_size[1] > e_i) {
        e_i = y_size[1];
      }

      if (y_size[1] > e_i) {
        e_i = y_size[1];
      }

      if (I3_size[1] > e_i) {
        e_i = I3_size[1];
      }

      if (I3_size[1] > e_i) {
        e_i = I3_size[1];
      }
    }

    if (e_i == 0) {
      pny_tmp = SuJm_size[0];
      d = SuJm_size[0];
      loop_ub = y_size[0];
      empty_non_axis_sizes_idx_0 = y_size[0];
      empty_non_axis_sizes_idx_0_0 = I3_size[0];
      empty_non_axis_sizes_idx_0_1 = I3_size[0];
    } else {
      if ((SuJm_size[0] != 0) && (SuJm_size[1] != 0)) {
        pny_tmp = SuJm_size[0];
        d = SuJm_size[0];
      } else {
        pny_tmp = 0;
        d = 0;
      }

      if ((y_size[0] != 0) && (y_size[1] != 0)) {
        loop_ub = y_size[0];
        empty_non_axis_sizes_idx_0 = y_size[0];
      } else {
        loop_ub = 0;
        empty_non_axis_sizes_idx_0 = 0;
      }

      if ((I3_size[0] != 0) && (I3_size[1] != 0)) {
        empty_non_axis_sizes_idx_0_0 = I3_size[0];
        empty_non_axis_sizes_idx_0_1 = I3_size[0];
      } else {
        empty_non_axis_sizes_idx_0_0 = 0;
        empty_non_axis_sizes_idx_0_1 = 0;
      }
    }

    f_i = ((((pny_tmp + d) + loop_ub) + empty_non_axis_sizes_idx_0) +
           empty_non_axis_sizes_idx_0_0) + empty_non_axis_sizes_idx_0_1;
    for (b_i = 0; b_i < e_i; b_i++) {
      for (trueCount = 0; trueCount < pny_tmp; trueCount++) {
        GenCode_MPC_B.Mu_data[trueCount + f_i * b_i] =
          GenCode_MPC_B.SuJm_data[pny_tmp * b_i + trueCount];
      }
    }

    for (b_i = 0; b_i < e_i; b_i++) {
      for (trueCount = 0; trueCount < d; trueCount++) {
        GenCode_MPC_B.Mu_data[(trueCount + pny_tmp) + f_i * b_i] =
          GenCode_MPC_B.tmp_data_cx[d * b_i + trueCount];
      }
    }

    for (b_i = 0; b_i < e_i; b_i++) {
      for (trueCount = 0; trueCount < loop_ub; trueCount++) {
        GenCode_MPC_B.Mu_data[((trueCount + pny_tmp) + d) + f_i * b_i] =
          I2Jm_data[loop_ub * b_i + trueCount];
      }
    }

    for (b_i = 0; b_i < e_i; b_i++) {
      for (trueCount = 0; trueCount < empty_non_axis_sizes_idx_0; trueCount++) {
        GenCode_MPC_B.Mu_data[(((trueCount + pny_tmp) + d) + loop_ub) + f_i *
          b_i] = varargin_4_data[empty_non_axis_sizes_idx_0 * b_i + trueCount];
      }
    }

    for (b_i = 0; b_i < e_i; b_i++) {
      for (trueCount = 0; trueCount < empty_non_axis_sizes_idx_0_0; trueCount++)
      {
        GenCode_MPC_B.Mu_data[((((trueCount + pny_tmp) + d) + loop_ub) +
          empty_non_axis_sizes_idx_0) + f_i * b_i] =
          I3_data[empty_non_axis_sizes_idx_0_0 * b_i + trueCount];
      }
    }

    for (b_i = 0; b_i < e_i; b_i++) {
      for (trueCount = 0; trueCount < empty_non_axis_sizes_idx_0_1; trueCount++)
      {
        GenCode_MPC_B.Mu_data[(((((trueCount + pny_tmp) + d) + loop_ub) +
          empty_non_axis_sizes_idx_0) + empty_non_axis_sizes_idx_0_0) + f_i *
          b_i] = varargin_6_data[empty_non_axis_sizes_idx_0_1 * b_i + trueCount];
      }
    }

    nr = (b_p + nmoves) * 2.0;
    loop_ub = Hv_size[0] * Hv_size[1] - 1;
    for (b_i = 0; b_i <= loop_ub; b_i++) {
      GenCode_MPC_B.varargin_1_data[b_i] = -GenCode_MPC_B.Hv_data[b_i];
    }

    if ((Hv_size[0] != 0) && (Hv_size[1] != 0)) {
      pny_tmp = Hv_size[1];
    } else if ((Hv_size[0] != 0) && (Hv_size[1] != 0)) {
      pny_tmp = Hv_size[1];
    } else if ((static_cast<int32_T>(nr) != 0) && (Hv_size[1] != 0)) {
      pny_tmp = Hv_size[1];
    } else {
      pny_tmp = Hv_size[1];
    }

    if ((static_cast<int32_T>(nr) != 0) && (Hv_size[1] != 0)) {
      loop_ub = static_cast<int32_T>(nr);
    } else {
      loop_ub = 0;
    }

    if ((Hv_size[0] != 0) && (Hv_size[1] != 0)) {
      empty_non_axis_sizes_idx_0 = Hv_size[0];
      empty_non_axis_sizes_idx_0_0 = Hv_size[0];
    } else {
      empty_non_axis_sizes_idx_0 = 0;
      empty_non_axis_sizes_idx_0_0 = 0;
    }

    d = (empty_non_axis_sizes_idx_0 + empty_non_axis_sizes_idx_0_0) + loop_ub;
    for (b_i = 0; b_i < pny_tmp; b_i++) {
      for (trueCount = 0; trueCount < empty_non_axis_sizes_idx_0; trueCount++) {
        GenCode_MPC_B.Mv_aux_data[trueCount + d * b_i] =
          GenCode_MPC_B.varargin_1_data[empty_non_axis_sizes_idx_0 * b_i +
          trueCount];
      }
    }

    for (b_i = 0; b_i < pny_tmp; b_i++) {
      for (trueCount = 0; trueCount < empty_non_axis_sizes_idx_0_0; trueCount++)
      {
        GenCode_MPC_B.Mv_aux_data[(trueCount + empty_non_axis_sizes_idx_0) + d *
          b_i] = GenCode_MPC_B.Hv_data[empty_non_axis_sizes_idx_0_0 * b_i +
          trueCount];
      }
    }

    for (b_i = 0; b_i < pny_tmp; b_i++) {
      for (trueCount = 0; trueCount < loop_ub; trueCount++) {
        GenCode_MPC_B.Mv_aux_data[((trueCount + empty_non_axis_sizes_idx_0) +
          empty_non_axis_sizes_idx_0_0) + d * b_i] = 0.0;
      }
    }

    loop_ub = 0;
    for (trueCount = 0; trueCount < isMrows_size; trueCount++) {
      if (isMrows_data[trueCount]) {
        loop_ub++;
      }
    }

    trueCount = 0;
    for (empty_non_axis_sizes_idx_0 = 0; empty_non_axis_sizes_idx_0 <
         isMrows_size; empty_non_axis_sizes_idx_0++) {
      if (isMrows_data[empty_non_axis_sizes_idx_0]) {
        o_data[trueCount] = static_cast<uint8_T>(empty_non_axis_sizes_idx_0 + 1);
        trueCount++;
      }
    }

    for (b_i = 0; b_i < e_i; b_i++) {
      for (trueCount = 0; trueCount < loop_ub; trueCount++) {
        b_Ac->data[trueCount + b_Ac->size[0] * b_i] = -GenCode_MPC_B.Mu_data
          [(f_i * b_i + o_data[trueCount]) - 1];
      }
    }

    trueCount = 0;
    for (e_i = 0; e_i < isMrows_size; e_i++) {
      if (isMrows_data[e_i]) {
        trueCount++;
      }
    }

    e_i = 0;
    for (f_i = 0; f_i < isMrows_size; f_i++) {
      if (isMrows_data[f_i]) {
        q_data[e_i] = static_cast<uint8_T>(f_i + 1);
        e_i++;
      }
    }

    e_i = b_Ac->size[1] - 1;
    for (b_i = 0; b_i < trueCount; b_i++) {
      b_Ac->data[b_i + b_Ac->size[0] * e_i] = Vfull_data[q_data[b_i] - 1];
    }

    e_i = 0;
    for (trueCount = 0; trueCount < isMrows_size; trueCount++) {
      if (isMrows_data[trueCount]) {
        e_i++;
      }
    }

    trueCount = 0;
    for (f_i = 0; f_i < isMrows_size; f_i++) {
      if (isMrows_data[f_i]) {
        r_data[trueCount] = static_cast<uint8_T>(f_i + 1);
        trueCount++;
      }
    }

    f_i = static_cast<int32_T>(nr);
    empty_non_axis_sizes_idx_0 = (Sx_size[0] + Sx_size[0]) + static_cast<int32_T>
      (nr);
    loop_ub = Sx_size[0];
    for (b_i = 0; b_i < 5; b_i++) {
      for (trueCount = 0; trueCount < loop_ub; trueCount++) {
        GenCode_MPC_B.tmp_data_cx[trueCount + empty_non_axis_sizes_idx_0 * b_i] =
          -Sx_data[Sx_size[0] * b_i + trueCount];
      }
    }

    loop_ub = Sx_size[0];
    for (b_i = 0; b_i < 5; b_i++) {
      for (trueCount = 0; trueCount < loop_ub; trueCount++) {
        GenCode_MPC_B.tmp_data_cx[(trueCount + Sx_size[0]) +
          empty_non_axis_sizes_idx_0 * b_i] = Sx_data[Sx_size[0] * b_i +
          trueCount];
      }
    }

    for (b_i = 0; b_i < 5; b_i++) {
      for (trueCount = 0; trueCount < f_i; trueCount++) {
        GenCode_MPC_B.tmp_data_cx[((trueCount + Sx_size[0]) + Sx_size[0]) +
          empty_non_axis_sizes_idx_0 * b_i] = 0.0;
      }
    }

    for (b_i = 0; b_i < 5; b_i++) {
      for (trueCount = 0; trueCount < e_i; trueCount++) {
        b_Mx->data[trueCount + b_Mx->size[0] * b_i] = GenCode_MPC_B.tmp_data_cx
          [(empty_non_axis_sizes_idx_0 * b_i + r_data[trueCount]) - 1];
      }
    }

    trueCount = 0;
    for (e_i = 0; e_i < isMrows_size; e_i++) {
      if (isMrows_data[e_i]) {
        trueCount++;
      }
    }

    e_i = 0;
    for (f_i = 0; f_i < isMrows_size; f_i++) {
      if (isMrows_data[f_i]) {
        s_data[e_i] = static_cast<uint8_T>(f_i + 1);
        e_i++;
      }
    }

    e_i = static_cast<int32_T>(2.0 * nmoves);
    for (b_i = 0; b_i < tmp_size; b_i++) {
      Mlimfull_data[b_i] = -Su1_data[b_i];
    }

    for (b_i = 0; b_i < tmp_size; b_i++) {
      Mlimfull_data[b_i + tmp_size] = Su1_data[b_i];
    }

    for (b_i = 0; b_i < I1_size; b_i++) {
      Mlimfull_data[(b_i + tmp_size) + tmp_size] = -I1_data[b_i];
    }

    for (b_i = 0; b_i < I1_size; b_i++) {
      Mlimfull_data[((b_i + tmp_size) + tmp_size) + I1_size] = I1_data[b_i];
    }

    for (b_i = 0; b_i < e_i; b_i++) {
      Mlimfull_data[(((b_i + tmp_size) + tmp_size) + I1_size) + I1_size] = 0.0;
    }

    for (b_i = 0; b_i < trueCount; b_i++) {
      b_Mu1->data[b_i] = Mlimfull_data[s_data[b_i] - 1];
    }

    e_i = 0;
    for (trueCount = 0; trueCount < isMrows_size; trueCount++) {
      if (isMrows_data[trueCount]) {
        e_i++;
      }
    }

    trueCount = 0;
    for (f_i = 0; f_i < isMrows_size; f_i++) {
      if (isMrows_data[f_i]) {
        t_data[trueCount] = static_cast<uint8_T>(f_i + 1);
        trueCount++;
      }
    }

    for (b_i = 0; b_i < pny_tmp; b_i++) {
      for (trueCount = 0; trueCount < e_i; trueCount++) {
        b_Mv->data[trueCount + b_Mv->size[0] * b_i] = GenCode_MPC_B.Mv_aux_data
          [(d * b_i + t_data[trueCount]) - 1];
      }
    }

    b_H_size[0] = static_cast<int32_T>(nmoves + 1.0);
    b_H_size[1] = static_cast<int32_T>(nmoves + 1.0);
    loop_ub = static_cast<int32_T>(nmoves + 1.0) * static_cast<int32_T>(nmoves +
      1.0) - 1;
    for (b_i = 0; b_i <= loop_ub; b_i++) {
      b_H_data[b_i] = 0.0;
    }

    b_H_data[(static_cast<int32_T>(nmoves + 1.0) + static_cast<int32_T>(nmoves +
               1.0) * (static_cast<int32_T>(nmoves + 1.0) - 1)) - 1] = 100000.0;
    bb[0] = 0.0956352499790037;
    bb[1] = 0.0009563524997900371;
    GenCode_MPC_updateWeights(bb, ywt);
    if (uwt < 0.0) {
      nr = 0.0;
    } else {
      nr = uwt * uwt;
    }

    GenCode_MP_mpc_calculatehessian(bb, nr, GenCode_MPC_B.SuJm_data, SuJm_size,
      GenCode_MPC_B.y_data, y_size, Jm_data, Jm_size, I1_data, &I1_size,
      Su1_data, &tmp_size, Sx_data, Sx_size, GenCode_MPC_B.Hv_data, Hv_size,
      I3_data, I3_size, Ku1_data, tmp_size_0, I2Jm_data, tmp_size_1, Kx_data,
      Kx_size, GenCode_MPC_B.Kv_data, Kv_size, GenCode_MPC_B.tmp_data_cx,
      varargin_2_size);
    loop_ub = I3_size[1];
    for (b_i = 0; b_i < loop_ub; b_i++) {
      pny_tmp = I3_size[0];
      for (trueCount = 0; trueCount < pny_tmp; trueCount++) {
        b_H_data[trueCount + static_cast<int32_T>(nmoves + 1.0) * b_i] =
          I3_data[I3_size[0] * b_i + trueCount];
      }
    }

    GenCode_MPC_mpc_checkhessian(b_H_data, b_H_size, b_Linv_data, Jm_size, &nr);
    if (nr > 1.0) {
      *u = old_u + b_uoff;
      for (i = 0; i < 21; i++) {
        useq[i] = *u;
      }

      *status = -2.0;
    } else {
      GenCode_MPC_emxInit_real_T(&tmp, 1);
      GenCode_MPC_emxInit_real_T(&tmp_0, 1);
      GenCode_MPC_eye_g(nmoves + 1.0, tmp_data_0, tmp_size_0);
      GenCode_MPC_linsolve(b_Linv_data, Jm_size, tmp_data_0, tmp_size_0,
                           b_H_data, b_H_size);
      GenCode_MPC_mtimes_ojo(b_Mx, x, tmp);
      GenCode_MPC_mtimes_ojos(b_Mv, vseq, tmp_0);
      b_i = b_Mu1->size[0];
      b_Mu1->size[0] = b_Mlim->size[0];
      GenCod_emxEnsureCapacity_real_T(b_Mu1, b_i);
      loop_ub = b_Mlim->size[0];
      for (b_i = 0; b_i < loop_ub; b_i++) {
        b_Mu1->data[b_i] = -(((b_Mlim->data[b_i] + tmp->data[b_i]) + b_Mu1->
                              data[b_i] * old_u) + tmp_0->data[b_i]);
      }

      GenCode_MPC_emxFree_real_T(&tmp_0);
      GenCode_MPC_emxFree_real_T(&tmp);
      umax_incr_flag = false;
      nr = 0.0;
      umin_incr_flag = false;
      j = 0.0;
      if ((b_Mrows->size[0] != 0) && (b_Mrows->data[0] > 0.0)) {
        b_i = 0;
        exitg1 = false;
        while ((!exitg1) && (b_i <= b_Mrows->size[0] - 1)) {
          if (b_Mrows->data[b_i] <= pny) {
            b_i++;
          } else {
            tmp_2 = 2.0 * static_cast<real_T>(pny);
            if (b_Mrows->data[b_i] <= tmp_2) {
              b_i++;
            } else if (b_Mrows->data[b_i] <= tmp_2 + b_p) {
              if (!umax_incr_flag) {
                nr = -(1.25 * umax - b_uoff) - (-b_Mlim->data[b_i]);
              }

              umax_incr_flag = true;
              b_Mu1->data[b_i] += nr;
              b_i++;
            } else if (b_Mrows->data[b_i] <= tmp_2 + 2.0 * b_p) {
              if (!umin_incr_flag) {
                j = (1.25 * umin - b_uoff) - (-b_Mlim->data[b_i]);
              }

              umin_incr_flag = true;
              b_Mu1->data[b_i] += j;
              b_i++;
            } else {
              exitg1 = true;
            }
          }
        }
      }

      GenCode_MPC_emxInit_boolean_T(&tmp_1, 1);
      GenCode_MPC_mtimes_o(b_H_data, b_H_size, b_H_data, b_H_size,
                           GenCode_MPC_B.tmp_data_k, tmp_size_0);
      b_i = tmp_1->size[0];
      tmp_1->size[0] = i;
      Gen_emxEnsureCapacity_boolean_T(tmp_1, b_i);
      for (b_i = 0; b_i < i; b_i++) {
        tmp_1->data[b_i] = false;
      }

      GenCode_MPC_mpc_solveQP(x, static_cast<real_T>(i), nmoves + 1.0, Kx_data,
        GenCode_MPC_B.tmp_data_cx, varargin_2_size, rseq, Ku1_data, old_u,
        GenCode_MPC_B.Kv_data, Kv_size, vseq, I2Jm_data, tmp_size_1,
        b_utarget_data, b_H_data, b_H_size, GenCode_MPC_B.tmp_data_k, tmp_size_0,
        b_Ac, b_Mu1, tmp_1, zopt_data, &I1_size, f_data, &isMrows_size, status);
      *u = (old_u + zopt_data[0]) + b_uoff;
      GenCode_MPC_emxFree_boolean_T(&tmp_1);
    }

    GenCode_MPC_emxFree_real_T(&b_Mlim);
    GenCode_MPC_emxFree_real_T(&b_Mv);
    GenCode_MPC_emxFree_real_T(&b_Mu1);
    GenCode_MPC_emxFree_real_T(&b_Mx);
    GenCode_MPC_emxFree_real_T(&b_Ac);
    GenCode_MPC_emxFree_real_T(&b_Mrows);
  }
}

// Model step function
void MPCModelClass::step()
{
  if (rtmIsMajorTimeStep((&GenCode_MPC_M))) {
    // set solver stop time
    rtsiSetSolverStopTime(&(&GenCode_MPC_M)->solverInfo,(((&GenCode_MPC_M)
      ->Timing.clockTick0+1)*(&GenCode_MPC_M)->Timing.stepSize0));
  }                                    // end MajorTimeStep

  // Update absolute time of base rate at minor time step
  if (rtmIsMinorTimeStep((&GenCode_MPC_M))) {
    (&GenCode_MPC_M)->Timing.t[0] = rtsiGetT(&(&GenCode_MPC_M)->solverInfo);
  }

  {
    static const real_T c[30] = { 0.36190476190476195, 0.10573913043478261,
      0.00041918977982970259, 0.00026190470596134378, 0.0, 0.0, 0.0,
      -5.0000000000000008E-7, -0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    static const real_T d_A[25] = { 1.510953024029432E-253,
      1.5958319770361621E-253, 7.9652189134601537E-5, 2.13602141660513E-5, 0.0,
      6.554302258989347E-253, 6.9224952501579629E-253, 8.78811630949753E-5,
      0.00015206438180117475, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.001, 1.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

    static const real_T e[8] = { 0.8, 0.8, 0.8, 0.8, 0.01, 0.01, 0.01, 0.01 };

    static const real_T f[8] = { 0.5, 0.1, 0.5, 0.1, 0.5, 0.1, 0.5, 0.1 };

    static const int8_T d[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1 };

    static const int8_T c_C[10] = { 0, 0, 0, 0, 2, 0, 0, 10, 0, 1 };

    static const int8_T tmp_3[8] = { 0, 0, 0, 0, 1, 0, 0, 1 };

    static const int8_T b_a_0[6] = { 1, 0, 0, 1, 0, -1 };

    emxArray_real_T_GenCode_MPC_T *Bu;
    emxArray_real_T_GenCode_MPC_T *Bv;
    emxArray_real_T_GenCode_MPC_T *Cm;
    emxArray_real_T_GenCode_MPC_T *Dv;
    emxArray_real_T_GenCode_MPC_T *Dvm;
    emxArray_real_T_GenCode_MPC_T *rseq;
    emxArray_real_T_GenCode_MPC_T *vseq;
    real_T CovMat[49];
    real_T c_B_0[35];
    real_T c_B_1[35];
    real_T c_B[30];
    real_T L_tmp[25];
    real_T L_tmp_0[25];
    real_T Product_0[25];
    real_T b_A[25];
    real_T useq1[21];
    real_T A[16];
    real_T Af[16];
    real_T Af_0[16];
    real_T Ai[16];
    real_T b[16];
    real_T b_utarget[11];
    real_T L[10];
    real_T Product[10];
    real_T b_C[10];
    real_T tmp_0[10];
    real_T tmp_1[10];
    real_T a[8];
    real_T b_b[8];
    real_T xk[5];
    real_T xk_0[5];
    real_T Kinv[4];
    real_T b_a[4];
    real_T rtb_Am[4];
    real_T Cm_0[2];
    real_T Dvm_0[2];
    real_T a21;
    real_T a22;
    real_T rtb_Am_tmp;
    real_T tmp_2;
    real_T v_idx_1;
    real_T y_innov_idx_1;
    int32_T Coef;
    int32_T a_tmp;
    int32_T i;
    int32_T r1;
    int8_T b_D[12];
    int8_T tmp[6];
    int8_T UnknownIn[5];
    int8_T c_A[4];
    for (i = 0; i < 10; i++) {
      // Product: '<S1>/Product' incorporates:
      //   Inport: '<Root>/CurvaturePreview'
      //   Inport: '<Root>/LongitudinalVelocity'

      Product[i] = GenCode_MPC_U.LongitudinalVelocity *
        GenCode_MPC_U.CurvaturePreview[i];
    }

    // SignalConversion generated from: '<S38>/Matrix Concatenate' incorporates:
    //   Constant: '<S38>/Vehicle mass constant'
    //   Inport: '<Root>/LongitudinalVelocity'
    //   Product: '<S38>/Divide2'

    rtb_Am[0] = GenCode_MPC_ConstB.Gain2 / 700.0 /
      GenCode_MPC_U.LongitudinalVelocity;

    // Product: '<S38>/Divide4' incorporates:
    //   Inport: '<Root>/LongitudinalVelocity'
    //   Product: '<S38>/Divide3'

    rtb_Am_tmp = GenCode_MPC_ConstB.Gain3 / GenCode_MPC_U.LongitudinalVelocity;

    // SignalConversion generated from: '<S38>/Matrix Concatenate' incorporates:
    //   Constant: '<S38>/Vehicle yaw inertia constant'
    //   Product: '<S38>/Divide4'

    rtb_Am[1] = rtb_Am_tmp / 2875.0;

    // SignalConversion generated from: '<S38>/Matrix Concatenate' incorporates:
    //   Constant: '<S38>/Vehicle mass constant'
    //   Constant: '<S38>/Vehicle yaw inertia constant'
    //   Inport: '<Root>/LongitudinalVelocity'
    //   Product: '<S38>/Divide3'
    //   Product: '<S38>/Divide5'
    //   Sum: '<S38>/Sum2'

    rtb_Am[2] = rtb_Am_tmp / 700.0 - GenCode_MPC_U.LongitudinalVelocity;
    rtb_Am[3] = GenCode_MPC_ConstB.Gain4 / GenCode_MPC_U.LongitudinalVelocity /
      2875.0;

    // MATLAB Function: '<S4>/Adaptive Model' incorporates:
    //   Concatenate: '<S38>/Matrix Concatenate'
    //   Constant: '<S38>/Cm Constant'
    //   Constant: '<S4>/Sample time constant'
    //   Inport: '<Root>/LongitudinalVelocity'

    tmp[0] = 1;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 1;
    tmp[5] = 0;
    for (r1 = 0; r1 < 2; r1++) {
      for (i = 0; i < 2; i++) {
        Coef = i + (r1 << 1);
        b_a[Coef] = 0.0;
        b_a[Coef] += static_cast<real_T>(tmp[3 * r1] * b_a_0[i]);
        b_a[Coef] += static_cast<real_T>(tmp[3 * r1 + 1] * b_a_0[i + 2]);
        b_a[Coef] += static_cast<real_T>(tmp[3 * r1 + 2] * b_a_0[i + 4]);
        Af[r1 + (i << 2)] = rtb_Am[(i << 1) + r1];
        Af[r1 + ((i + 2) << 2)] = 0.0;
      }
    }

    Af[10] = 0.0;
    Af[14] = GenCode_MPC_U.LongitudinalVelocity;
    Af[2] = b_a[0];
    Af[3] = b_a[1];
    Af[11] = 0.0;
    Af[6] = b_a[2];
    Af[7] = b_a[3];
    Af[15] = 0.0;
    for (r1 = 0; r1 < 16; r1++) {
      Af_0[r1] = Af[r1] * 0.1;
    }

    GenCode_MPC_expm(Af_0, A);
    for (r1 = 0; r1 < 16; r1++) {
      Ai[r1] = 0.0;
    }

    Ai[0] = 1.0;
    Ai[5] = 1.0;
    Ai[10] = 1.0;
    Ai[15] = 1.0;
    for (r1 = 0; r1 < 16; r1++) {
      Ai[r1] += A[r1];
    }

    Coef = 2;
    for (i = 0; i < 3; i++) {
      if (Coef == 2) {
        Coef = 4;
      } else {
        Coef = 2;
      }

      for (r1 = 0; r1 < 16; r1++) {
        Af_0[r1] = static_cast<real_T>(i + 1) * Af[r1] * 0.025;
      }

      GenCode_MPC_expm(Af_0, b);
      for (r1 = 0; r1 < 16; r1++) {
        Ai[r1] += static_cast<real_T>(Coef) * b[r1];
      }
    }

    if (rtmIsMajorTimeStep((&GenCode_MPC_M))) {
      GenCode_MPC_emxInit_real_T(&Bu, 3);

      // MATLAB Function: '<S35>/VariableHorizonOptimizer'
      r1 = Bu->size[0] * Bu->size[1] * Bu->size[2];
      Bu->size[0] = 5;
      Bu->size[1] = 1;
      Bu->size[2] = static_cast<int32_T>(GenCode_MPC_ConstB.DataTypeConversion22
        + 1.0);
      GenCod_emxEnsureCapacity_real_T(Bu, r1);
      i = static_cast<int32_T>(GenCode_MPC_ConstB.DataTypeConversion22 + 1.0) *
        5 - 1;
      for (r1 = 0; r1 <= i; r1++) {
        Bu->data[r1] = 0.0;
      }

      GenCode_MPC_emxInit_real_T(&Bv, 3);

      // MATLAB Function: '<S35>/VariableHorizonOptimizer'
      r1 = Bv->size[0] * Bv->size[1] * Bv->size[2];
      Bv->size[0] = 5;
      Bv->size[1] = 2;
      Bv->size[2] = static_cast<int32_T>(GenCode_MPC_ConstB.DataTypeConversion22
        + 1.0);
      GenCod_emxEnsureCapacity_real_T(Bv, r1);
      i = static_cast<int32_T>(GenCode_MPC_ConstB.DataTypeConversion22 + 1.0) *
        10 - 1;
      for (r1 = 0; r1 <= i; r1++) {
        Bv->data[r1] = 0.0;
      }

      GenCode_MPC_emxInit_real_T(&Dv, 3);

      // MATLAB Function: '<S35>/VariableHorizonOptimizer'
      r1 = Dv->size[0] * Dv->size[1] * Dv->size[2];
      Dv->size[0] = 2;
      Dv->size[1] = 2;
      Dv->size[2] = static_cast<int32_T>(GenCode_MPC_ConstB.DataTypeConversion22
        + 1.0);
      GenCod_emxEnsureCapacity_real_T(Dv, r1);
      Coef = (static_cast<int32_T>(GenCode_MPC_ConstB.DataTypeConversion22 + 1.0)
              << 2) - 1;
      for (r1 = 0; r1 <= Coef; r1++) {
        Dv->data[r1] = 0.0;
      }

      GenCode_MPC_emxInit_real_T(&Dvm, 3);

      // MATLAB Function: '<S35>/VariableHorizonOptimizer'
      r1 = Dvm->size[0] * Dvm->size[1] * Dvm->size[2];
      Dvm->size[0] = 2;
      Dvm->size[1] = 2;
      Dvm->size[2] = static_cast<int32_T>
        (GenCode_MPC_ConstB.DataTypeConversion22 + 1.0);
      GenCod_emxEnsureCapacity_real_T(Dvm, r1);
      for (r1 = 0; r1 <= Coef; r1++) {
        Dvm->data[r1] = 0.0;
      }

      GenCode_MPC_emxInit_real_T(&Cm, 3);

      // MATLAB Function: '<S35>/VariableHorizonOptimizer'
      r1 = Cm->size[0] * Cm->size[1] * Cm->size[2];
      Cm->size[0] = 2;
      Cm->size[1] = 5;
      Cm->size[2] = static_cast<int32_T>(GenCode_MPC_ConstB.DataTypeConversion22
        + 1.0);
      GenCod_emxEnsureCapacity_real_T(Cm, r1);
      for (r1 = 0; r1 <= i; r1++) {
        Cm->data[r1] = 0.0;
      }

      std::memcpy(&c_B[0], &c[0], 30U * sizeof(real_T));
      for (r1 = 0; r1 < 12; r1++) {
        b_D[r1] = d[r1];
      }

      for (r1 = 0; r1 < 10; r1++) {
        b_C[r1] = c_C[r1];
      }

      std::memcpy(&b_A[0], &d_A[0], 25U * sizeof(real_T));

      // MATLAB Function: '<S4>/Adaptive Model' incorporates:
      //   SignalConversion generated from: '<S37>/ SFunction '

      b_b[0] = GenCode_MPC_ConstB.b1;
      b_b[1] = GenCode_MPC_ConstB.b2;
      b_b[4] = 0.0;
      b_b[5] = 0.0;
      b_b[2] = 0.0;
      b_b[3] = 0.0;
      b_b[6] = 0.0;
      b_b[7] = -1.0;
      for (r1 = 0; r1 < 2; r1++) {
        for (i = 0; i < 4; i++) {
          Coef = r1 << 2;
          a_tmp = i + Coef;
          a[a_tmp] = 0.0;
          a[a_tmp] += 0.0083333333333333332 * Ai[i] * b_b[Coef];
          a[a_tmp] += Ai[i + 4] * 0.0083333333333333332 * b_b[Coef + 1];
          a[a_tmp] += Ai[i + 8] * 0.0083333333333333332 * 0.0;
          a[a_tmp] += Ai[i + 12] * 0.0083333333333333332 * b_b[Coef + 3];
        }
      }

      // MATLAB Function: '<S35>/VariableHorizonOptimizer' incorporates:
      //   Constant: '<S4>/DX Constant'
      //   MATLAB Function: '<S4>/Adaptive Model'

      for (r1 = 0; r1 < 8; r1++) {
        b_b[r1] = a[r1] * e[r1];
      }

      for (r1 = 0; r1 < 4; r1++) {
        i = r1 << 2;
        b_A[5 * r1] = A[i];
        b_A[5 * r1 + 1] = A[i + 1];
        b_A[5 * r1 + 2] = A[i + 2];
        b_A[5 * r1 + 3] = A[i + 3];
        c_B[r1] = b_b[r1];

        // MATLAB Function: '<S4>/Adaptive Model'
        i = r1 << 1;
        b_C[i] = static_cast<real_T>(tmp_3[i]) / f[i];
        b_C[i + 1] = static_cast<real_T>(tmp_3[i + 1]) / f[i + 1];
        c_B[r1 + 5] = b_b[r1 + 4];
      }

      b_D[2] = 0;
      b_D[3] = 0;
      UnknownIn[0] = 1;
      UnknownIn[1] = 2;
      UnknownIn[2] = 4;
      UnknownIn[3] = 5;
      UnknownIn[4] = 6;
      for (r1 = 0; r1 < 5; r1++) {
        for (i = 0; i < 5; i++) {
          c_B_0[i + 7 * r1] = c_B[(UnknownIn[r1] - 1) * 5 + i];
          c_B_1[i + 5 * r1] = c_B[(UnknownIn[i] - 1) * 5 + r1];
        }

        i = (UnknownIn[r1] - 1) << 1;
        c_B_0[7 * r1 + 5] = b_D[i];
        c_B_0[7 * r1 + 6] = b_D[i + 1];
      }

      for (r1 = 0; r1 < 2; r1++) {
        for (i = 0; i < 5; i++) {
          c_B_1[i + 5 * (r1 + 5)] = b_D[((UnknownIn[i] - 1) << 1) + r1];
        }
      }

      for (r1 = 0; r1 < 7; r1++) {
        for (i = 0; i < 7; i++) {
          a_tmp = i + 7 * r1;
          CovMat[a_tmp] = 0.0;
          for (Coef = 0; Coef < 5; Coef++) {
            CovMat[a_tmp] += c_B_0[7 * Coef + i] * c_B_1[5 * r1 + Coef];
          }
        }
      }

      for (r1 = 0; r1 < 5; r1++) {
        Bu->data[r1] = c_B[r1];
      }

      for (r1 = 0; r1 < 2; r1++) {
        for (i = 0; i < 5; i++) {
          Bv->data[i + 5 * r1] = c_B[(r1 + 1) * 5 + i];
        }
      }

      for (r1 = 0; r1 < 5; r1++) {
        i = r1 << 1;
        Cm->data[i] = b_C[i];
        Cm->data[i + 1] = b_C[i + 1];
      }

      Dv->data[0] = 0.0;
      Dvm->data[0] = 0.0;
      Dv->data[1] = 0.0;
      Dvm->data[1] = 0.0;
      Dv->data[2] = b_D[4];
      Dvm->data[2] = b_D[4];
      Dv->data[3] = b_D[5];
      Dvm->data[3] = b_D[5];
      rtb_Am[0] = 0.5;
      rtb_Am[1] = 0.5;
      rtb_Am[2] = 0.5;
      rtb_Am[3] = 0.5;
      for (i = 0; i < 11; i++) {
        b_utarget[i] = 0.0;
        b_utarget[i] = 0.0;
      }

      Bv->data[5] = 0.0;
      Bv->data[6] = 0.0;
      Bv->data[7] = 0.0;
      Bv->data[8] = 0.0;
      GenCode_MPC_emxInit_real_T(&vseq, 1);

      // MATLAB Function: '<S35>/VariableHorizonOptimizer'
      r1 = vseq->size[0];
      i = static_cast<int32_T>((GenCode_MPC_ConstB.DataTypeConversion22 + 1.0) *
        2.0);
      vseq->size[0] = i;
      GenCod_emxEnsureCapacity_real_T(vseq, r1);
      for (r1 = 0; r1 < i; r1++) {
        vseq->data[r1] = 0.0;
      }

      for (i = 0; i < static_cast<int32_T>
           (GenCode_MPC_ConstB.DataTypeConversion22 + 1.0); i++) {
        vseq->data[(i << 1) + 1] = 1.0;
      }

      GenCode_MPC_emxInit_real_T(&rseq, 1);

      // MATLAB Function: '<S35>/VariableHorizonOptimizer' incorporates:
      //   Memory: '<S7>/LastPcov'

      r1 = rseq->size[0];
      i = static_cast<int32_T>(GenCode_MPC_ConstB.DataTypeConversion22 * 2.0);
      rseq->size[0] = i;
      GenCod_emxEnsureCapacity_real_T(rseq, r1);
      for (r1 = 0; r1 < i; r1++) {
        rseq->data[r1] = 0.0;
      }

      for (i = 0; i < static_cast<int32_T>
           ((GenCode_MPC_ConstB.DataTypeConversion22 - 1.0) + 1.0); i++) {
        r1 = i << 1;
        rseq->data[r1] = 0.0;
        rseq->data[r1 + 1] = 0.0;
      }

      if (10.0 <= GenCode_MPC_ConstB.DataTypeConversion22 + 1.0) {
        for (r1 = 0; r1 < 10; r1++) {
          vseq->data[r1 << 1] = 100.0 * Product[r1];
        }

        for (r1 = 0; r1 < static_cast<int32_T>
             (GenCode_MPC_ConstB.DataTypeConversion22 + -9.0); r1++) {
          vseq->data[(r1 + 10) << 1] = 100.0 * Product[9];
        }
      } else {
        for (r1 = 0; r1 < static_cast<int32_T>
             (GenCode_MPC_ConstB.DataTypeConversion22 + 1.0); r1++) {
          vseq->data[r1 << 1] = 100.0 * Product[r1];
        }
      }

      rtb_Am_tmp = vseq->data[0];
      v_idx_1 = vseq->data[1];
      c_A[0] = 0;
      c_A[1] = 0;
      c_A[2] = 0;
      c_A[3] = 0;
      for (Coef = 0; Coef < 2; Coef++) {
        c_A[Coef + (Coef << 1)] = 1;
        for (r1 = 0; r1 < 5; r1++) {
          a_tmp = (r1 << 1) + Coef;
          L[r1 + 5 * Coef] = Cm->data[a_tmp];
          Product[a_tmp] = 0.0;
          for (i = 0; i < 5; i++) {
            Product[a_tmp] += Cm->data[(i << 1) + Coef] *
              GenCode_MPC_DW.LastPcov_PreviousInput[5 * r1 + i];
          }
        }
      }

      for (r1 = 0; r1 < 2; r1++) {
        for (i = 0; i < 2; i++) {
          a22 = 0.0;
          for (Coef = 0; Coef < 5; Coef++) {
            a22 += Product[(Coef << 1) + r1] * L[5 * i + Coef];
          }

          b_a[r1 + (i << 1)] = CovMat[((i + 5) * 7 + r1) + 5] + a22;
        }
      }

      if (std::abs(b_a[1]) > std::abs(b_a[0])) {
        r1 = 1;
        Coef = 0;
      } else {
        r1 = 0;
        Coef = 1;
      }

      a21 = b_a[Coef] / b_a[r1];
      y_innov_idx_1 = b_a[r1 + 2];
      a22 = b_a[Coef + 2] - y_innov_idx_1 * a21;
      i = r1 << 1;
      Kinv[i] = static_cast<real_T>(c_A[0]) / b_a[r1];
      Coef <<= 1;
      Kinv[Coef] = (static_cast<real_T>(c_A[2]) - Kinv[i] * y_innov_idx_1) / a22;
      Kinv[i] -= Kinv[Coef] * a21;
      Kinv[i + 1] = static_cast<real_T>(c_A[1]) / b_a[r1];
      Kinv[Coef + 1] = (static_cast<real_T>(c_A[3]) - Kinv[i + 1] *
                        y_innov_idx_1) / a22;
      Kinv[i + 1] -= Kinv[Coef + 1] * a21;
      for (r1 = 0; r1 < 5; r1++) {
        for (i = 0; i < 5; i++) {
          a_tmp = r1 + 5 * i;
          L_tmp[a_tmp] = 0.0;
          for (Coef = 0; Coef < 5; Coef++) {
            L_tmp[a_tmp] += b_A[5 * Coef + r1] *
              GenCode_MPC_DW.LastPcov_PreviousInput[5 * i + Coef];
          }
        }

        for (i = 0; i < 2; i++) {
          a22 = 0.0;
          for (Coef = 0; Coef < 5; Coef++) {
            a22 += L_tmp[5 * Coef + r1] * L[5 * i + Coef];
          }

          Product[r1 + 5 * i] = CovMat[(i + 5) * 7 + r1] + a22;
        }
      }

      for (r1 = 0; r1 < 5; r1++) {
        L[r1] = 0.0;
        L[r1] += Product[r1] * Kinv[0];
        a21 = Product[r1 + 5];
        L[r1] += a21 * Kinv[1];
        L[r1 + 5] = 0.0;
        L[r1 + 5] += Product[r1] * Kinv[2];
        L[r1 + 5] += a21 * Kinv[3];
        xk[r1] = Bu->data[r1] * 0.0 + GenCode_MPC_DW.last_x_PreviousInput[r1];
      }

      for (r1 = 0; r1 < 2; r1++) {
        Cm_0[r1] = 0.0;
        for (i = 0; i < 5; i++) {
          a21 = Cm->data[(i << 1) + r1] * xk[i] + Cm_0[r1];
          Cm_0[r1] = a21;
        }

        a21 = Dvm->data[r1] * rtb_Am_tmp;
        a21 += Dvm->data[r1 + 2] * v_idx_1;
        Dvm_0[r1] = a21;
      }

      GenCode_MPC_emxFree_real_T(&Dvm);

      // MATLAB Function: '<S35>/VariableHorizonOptimizer' incorporates:
      //   Inport: '<Root>/LateralDeviation'
      //   Integrator: '<S5>/Integrator2'
      //   Memory: '<S7>/LastPcov'
      //   Memory: '<S7>/Memory'

      a21 = GenCode_MPC_U.LateralDeviation * 2.0 - (Cm_0[0] + Dvm_0[0]);
      y_innov_idx_1 = GenCode_MPC_X.Integrator2_CSTATE * 10.0 - (Cm_0[1] +
        Dvm_0[1]);
      GenCode_MPC_B.iAout[0] = GenCode_MPC_DW.Memory_PreviousInput[0];
      GenCode_MPC_B.iAout[1] = GenCode_MPC_DW.Memory_PreviousInput[1];
      GenCode_MPC_B.iAout[2] = GenCode_MPC_DW.Memory_PreviousInput[2];
      GenCode_MPC_B.iAout[3] = GenCode_MPC_DW.Memory_PreviousInput[3];
      for (r1 = 0; r1 < 5; r1++) {
        for (i = 0; i < 2; i++) {
          Coef = r1 + 5 * i;
          tmp_0[Coef] = 0.0;
          for (a_tmp = 0; a_tmp < 5; a_tmp++) {
            tmp_0[Coef] += GenCode_MPC_DW.LastPcov_PreviousInput[5 * a_tmp + r1]
              * Cm->data[(a_tmp << 1) + i];
          }
        }
      }

      GenCode_MPC_emxFree_real_T(&Cm);

      // MATLAB Function: '<S35>/VariableHorizonOptimizer' incorporates:
      //   Constant: '<S1>/Maximum steering angle constant'
      //   Constant: '<S1>/Minimum steering angle constant'
      //   Math: '<S7>/Math Function'
      //   UnitDelay: '<S7>/last_mv'

      for (r1 = 0; r1 < 5; r1++) {
        tmp_1[r1] = 0.0;
        tmp_1[r1] += tmp_0[r1] * Kinv[0];
        a22 = tmp_0[r1 + 5];
        tmp_1[r1] += a22 * Kinv[1];
        tmp_2 = tmp_1[r1] * a21;
        tmp_1[r1 + 5] = 0.0;
        tmp_1[r1 + 5] += tmp_0[r1] * Kinv[2];
        tmp_1[r1 + 5] += a22 * Kinv[3];
        tmp_2 += tmp_1[r1 + 5] * y_innov_idx_1;
        xk_0[r1] = xk[r1] + tmp_2;
      }

      GenCode_MP_mpcblock_optimizerPM(rseq, vseq, -0.7, 0.7,
        GenCode_MPC_ConstB.DataTypeConversion8, xk_0,
        GenCode_MPC_DW.last_mv_DSTATE, rtb_Am, b_utarget,
        GenCode_MPC_ConstB.DataTypeConversion22,
        GenCode_MPC_ConstB.DataTypeConversion23, 0.0,
        GenCode_MPC_ConstB.MathFunction, GenCode_MPC_ConstB.MathFunction1, b_A,
        Bu, Bv, b_C, Dv, &GenCode_MPC_B.u, useq1, &a22);
      GenCode_MPC_emxFree_real_T(&vseq);
      GenCode_MPC_emxFree_real_T(&rseq);
      GenCode_MPC_emxFree_real_T(&Dv);

      // MATLAB Function: '<S35>/VariableHorizonOptimizer'
      for (r1 = 0; r1 < 5; r1++) {
        for (i = 0; i < 5; i++) {
          a_tmp = r1 + 5 * i;
          L_tmp_0[a_tmp] = 0.0;
          for (Coef = 0; Coef < 5; Coef++) {
            L_tmp_0[a_tmp] += L_tmp[5 * Coef + r1] * b_A[5 * Coef + i];
          }

          Product_0[a_tmp] = 0.0;
          Product_0[a_tmp] += Product[r1] * L[i];
          Product_0[a_tmp] += Product[r1 + 5] * L[i + 5];
        }
      }

      for (r1 = 0; r1 < 5; r1++) {
        a22 = 0.0;
        for (i = 0; i < 5; i++) {
          Coef = 5 * r1 + i;
          L_tmp[Coef] = (L_tmp_0[Coef] - Product_0[Coef]) + CovMat[7 * r1 + i];
          a22 += b_A[5 * i + r1] * xk[i];
        }

        xk_0[r1] = Bu->data[r1] * GenCode_MPC_B.u + a22;
      }

      GenCode_MPC_emxFree_real_T(&Bu);

      // MATLAB Function: '<S35>/VariableHorizonOptimizer'
      for (r1 = 0; r1 < 5; r1++) {
        a22 = Bv->data[r1] * rtb_Am_tmp;
        a22 += Bv->data[r1 + 5] * v_idx_1;
        xk[r1] = a22;
      }

      GenCode_MPC_emxFree_real_T(&Bv);

      // MATLAB Function: '<S35>/VariableHorizonOptimizer'
      for (r1 = 0; r1 < 5; r1++) {
        GenCode_MPC_B.xk1[r1] = (L[r1 + 5] * y_innov_idx_1 + L[r1] * a21) +
          (xk_0[r1] + xk[r1]);
        for (i = 0; i < 5; i++) {
          Coef = 5 * r1 + i;
          GenCode_MPC_B.Pk1[Coef] = (L_tmp[Coef] + L_tmp[5 * i + r1]) * 0.5;
        }
      }

      // Outport: '<Root>/Steering angle' incorporates:
      //   Gain: '<S7>/u_scale'

      GenCode_MPC_Y.Steeringangle = 0.8 * GenCode_MPC_B.u;

      // Assertion: '<S3>/Assertion' incorporates:
      //   Constant: '<S3>/min_val'
      //   Inport: '<Root>/LongitudinalVelocity'
      //   RelationalOperator: '<S3>/min_relop'

      utAssert(0.0 < GenCode_MPC_U.LongitudinalVelocity);
    }

    // Sum: '<S5>/Subtract' incorporates:
    //   Inport: '<Root>/CurrentCurvature'
    //   Inport: '<Root>/LongitudinalVelocity'
    //   Inport: '<Root>/YawRate'
    //   Product: '<S5>/Product'

    GenCode_MPC_B.e2dot = GenCode_MPC_U.YawRate -
      GenCode_MPC_U.LongitudinalVelocity * GenCode_MPC_U.CurrentCurvature;
  }

  if (rtmIsMajorTimeStep((&GenCode_MPC_M))) {
    int32_T i;
    if (rtmIsMajorTimeStep((&GenCode_MPC_M))) {
      // Update for Memory: '<S7>/last_x'
      for (i = 0; i < 5; i++) {
        GenCode_MPC_DW.last_x_PreviousInput[i] = GenCode_MPC_B.xk1[i];
      }

      // End of Update for Memory: '<S7>/last_x'

      // Update for UnitDelay: '<S7>/last_mv'
      GenCode_MPC_DW.last_mv_DSTATE = GenCode_MPC_B.u;

      // Update for Memory: '<S7>/LastPcov'
      std::memcpy(&GenCode_MPC_DW.LastPcov_PreviousInput[0], &GenCode_MPC_B.Pk1
                  [0], 25U * sizeof(real_T));

      // Update for Memory: '<S7>/Memory'
      GenCode_MPC_DW.Memory_PreviousInput[0] = GenCode_MPC_B.iAout[0];
      GenCode_MPC_DW.Memory_PreviousInput[1] = GenCode_MPC_B.iAout[1];
      GenCode_MPC_DW.Memory_PreviousInput[2] = GenCode_MPC_B.iAout[2];
      GenCode_MPC_DW.Memory_PreviousInput[3] = GenCode_MPC_B.iAout[3];
    }
  }                                    // end MajorTimeStep

  if (rtmIsMajorTimeStep((&GenCode_MPC_M))) {
    rt_ertODEUpdateContinuousStates(&(&GenCode_MPC_M)->solverInfo);

    // Update absolute time for base rate
    // The "clockTick0" counts the number of times the code of this task has
    //  been executed. The absolute time is the multiplication of "clockTick0"
    //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
    //  overflow during the application lifespan selected.

    ++(&GenCode_MPC_M)->Timing.clockTick0;
    (&GenCode_MPC_M)->Timing.t[0] = rtsiGetSolverStopTime(&(&GenCode_MPC_M)
      ->solverInfo);

    {
      // Update absolute timer for sample time: [0.1s, 0.0s]
      // The "clockTick1" counts the number of times the code of this task has
      //  been executed. The resolution of this integer timer is 0.1, which is the step size
      //  of the task. Size of "clockTick1" ensures timer will not overflow during the
      //  application lifespan selected.

      (&GenCode_MPC_M)->Timing.clockTick1++;
    }
  }                                    // end MajorTimeStep
}

// Derivatives for root system: '<Root>'
void MPCModelClass::GenCode_MPC_derivatives()
{
  MPCModelClass::XDot_GenCode_MPC_T *_rtXdot;
  _rtXdot = ((XDot_GenCode_MPC_T *) (&GenCode_MPC_M)->derivs);

  // Derivatives for Integrator: '<S5>/Integrator2'
  _rtXdot->Integrator2_CSTATE = GenCode_MPC_B.e2dot;
}

// Model initialize function
void MPCModelClass::initialize()
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
    // Setup solver object
    rtsiSetSimTimeStepPtr(&(&GenCode_MPC_M)->solverInfo, &(&GenCode_MPC_M)
                          ->Timing.simTimeStep);
    rtsiSetTPtr(&(&GenCode_MPC_M)->solverInfo, &rtmGetTPtr((&GenCode_MPC_M)));
    rtsiSetStepSizePtr(&(&GenCode_MPC_M)->solverInfo, &(&GenCode_MPC_M)
                       ->Timing.stepSize0);
    rtsiSetdXPtr(&(&GenCode_MPC_M)->solverInfo, &(&GenCode_MPC_M)->derivs);
    rtsiSetContStatesPtr(&(&GenCode_MPC_M)->solverInfo, (real_T **)
                         &(&GenCode_MPC_M)->contStates);
    rtsiSetNumContStatesPtr(&(&GenCode_MPC_M)->solverInfo, &(&GenCode_MPC_M)
      ->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&(&GenCode_MPC_M)->solverInfo,
      &(&GenCode_MPC_M)->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&(&GenCode_MPC_M)->solverInfo,
      &(&GenCode_MPC_M)->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&(&GenCode_MPC_M)->solverInfo,
      &(&GenCode_MPC_M)->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&(&GenCode_MPC_M)->solverInfo, (&rtmGetErrorStatus
      ((&GenCode_MPC_M))));
    rtsiSetRTModelPtr(&(&GenCode_MPC_M)->solverInfo, (&GenCode_MPC_M));
  }

  rtsiSetSimTimeStep(&(&GenCode_MPC_M)->solverInfo, MAJOR_TIME_STEP);
  (&GenCode_MPC_M)->intgData.y = (&GenCode_MPC_M)->odeY;
  (&GenCode_MPC_M)->intgData.f[0] = (&GenCode_MPC_M)->odeF[0];
  (&GenCode_MPC_M)->intgData.f[1] = (&GenCode_MPC_M)->odeF[1];
  (&GenCode_MPC_M)->intgData.f[2] = (&GenCode_MPC_M)->odeF[2];
  (&GenCode_MPC_M)->intgData.f[3] = (&GenCode_MPC_M)->odeF[3];
  (&GenCode_MPC_M)->contStates = ((X_GenCode_MPC_T *) &GenCode_MPC_X);
  rtsiSetSolverData(&(&GenCode_MPC_M)->solverInfo, static_cast<void *>
                    (&(&GenCode_MPC_M)->intgData));
  rtsiSetSolverName(&(&GenCode_MPC_M)->solverInfo,"ode4");
  rtmSetTPtr((&GenCode_MPC_M), &(&GenCode_MPC_M)->Timing.tArray[0]);
  (&GenCode_MPC_M)->Timing.stepSize0 = 0.1;

  // InitializeConditions for Integrator: '<S5>/Integrator2'
  GenCode_MPC_X.Integrator2_CSTATE = 0.0;

  // InitializeConditions for Memory: '<S7>/LastPcov'
  std::memcpy(&GenCode_MPC_DW.LastPcov_PreviousInput[0],
              &GenCode_MPC_ConstP.LastPcov_InitialCondition[0], 25U * sizeof
              (real_T));
}

// Model terminate function
void MPCModelClass::terminate()
{
  // (no terminate code required)
}

// Constructor
MPCModelClass::MPCModelClass() :
  GenCode_MPC_B(),
  GenCode_MPC_DW(),
  GenCode_MPC_X(),
  GenCode_MPC_U(),
  GenCode_MPC_Y(),
  GenCode_MPC_M()
{
  // Currently there is no constructor body generated.
}

// Destructor
MPCModelClass::~MPCModelClass()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
MPCModelClass::RT_MODEL_GenCode_MPC_T * MPCModelClass::getRTM()
{
  return (&GenCode_MPC_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
