/*
 * Classroom License -- for classroom instructional use only.  Not for
 * government, commercial, academic research, or other organizational use.
 *
 * kalman_filter.cpp
 *
 * Code generation for function 'kalman_filter'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "kalman_filter.h"
#include "mrdivide.h"

/* Function Definitions */
void kalman_filter(const double mu[8], const double cov[64], const double y[4],
                   const double C[32], const double Q_est[64], const double
                   R_est[16], double mu_next[8], double cov_next[64])
{
  int k;
  double cov_p[64];
  int i0;
  double Kt[32];
  double b_C[16];
  int i1;
  double c_C[32];
  double d0;
  signed char I[64];
  double b_y[4];
  double b_I[64];

  /* UNTITLED Summary of this function goes here */
  /*    Detailed explanation goes here */
  for (k = 0; k < 64; k++) {
    cov_p[k] = cov[k] + Q_est[k];
  }

  for (k = 0; k < 8; k++) {
    for (i0 = 0; i0 < 4; i0++) {
      Kt[k + (i0 << 3)] = 0.0;
      for (i1 = 0; i1 < 8; i1++) {
        Kt[k + (i0 << 3)] += cov_p[k + (i1 << 3)] * C[i0 + (i1 << 2)];
      }
    }
  }

  for (k = 0; k < 4; k++) {
    for (i0 = 0; i0 < 8; i0++) {
      c_C[k + (i0 << 2)] = 0.0;
      for (i1 = 0; i1 < 8; i1++) {
        c_C[k + (i0 << 2)] += C[k + (i1 << 2)] * cov_p[i1 + (i0 << 3)];
      }
    }

    for (i0 = 0; i0 < 4; i0++) {
      d0 = 0.0;
      for (i1 = 0; i1 < 8; i1++) {
        d0 += c_C[k + (i1 << 2)] * C[i0 + (i1 << 2)];
      }

      b_C[k + (i0 << 2)] = d0 + R_est[k + (i0 << 2)];
    }
  }

  mrdivide(Kt, b_C);
  for (k = 0; k < 4; k++) {
    d0 = 0.0;
    for (i0 = 0; i0 < 8; i0++) {
      d0 += C[k + (i0 << 2)] * mu[i0];
    }

    b_y[k] = y[k] - d0;
  }

  for (k = 0; k < 8; k++) {
    d0 = 0.0;
    for (i0 = 0; i0 < 4; i0++) {
      d0 += Kt[k + (i0 << 3)] * b_y[i0];
    }

    mu_next[k] = mu[k] + d0;
  }

  memset(&I[0], 0, sizeof(signed char) << 6);
  for (k = 0; k < 8; k++) {
    I[k + (k << 3)] = 1;
  }

  for (k = 0; k < 8; k++) {
    for (i0 = 0; i0 < 8; i0++) {
      d0 = 0.0;
      for (i1 = 0; i1 < 4; i1++) {
        d0 += Kt[k + (i1 << 3)] * C[i1 + (i0 << 2)];
      }

      b_I[k + (i0 << 3)] = (double)I[k + (i0 << 3)] - d0;
    }

    for (i0 = 0; i0 < 8; i0++) {
      cov_next[k + (i0 << 3)] = 0.0;
      for (i1 = 0; i1 < 8; i1++) {
        cov_next[k + (i0 << 3)] += b_I[k + (i1 << 3)] * cov_p[i1 + (i0 << 3)];
      }
    }
  }
}

/* End of code generation (kalman_filter.cpp) */
