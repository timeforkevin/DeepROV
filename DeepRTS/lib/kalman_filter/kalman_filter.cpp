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
                   R_est[16], double dt, double mu_next[8], double cov_next[64])
{
  int j;
  double v[4];
  double A[64];
  static const signed char iv0[64] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  double d[16];
  int i0;
  double mu_p[8];
  double Kt[32];
  double b_A[64];
  double b_C[32];
  int i1;
  double d0;
  double cov_p[64];

  /* UNTITLED Summary of this function goes here */
  /*    Detailed explanation goes here */
  /*  [lb] */
  /*  [lb*in^2] */
  /*  [lb*in^2] */
  /*  [lb*in^2] */
  for (j = 0; j < 64; j++) {
    A[j] = iv0[j];
  }

  v[0] = dt;
  v[1] = dt;
  v[2] = dt;
  v[3] = dt;
  memset(&d[0], 0, sizeof(double) << 4);
  for (j = 0; j < 4; j++) {
    d[j + (j << 2)] = v[j];
  }

  for (j = 0; j < 4; j++) {
    for (i0 = 0; i0 < 4; i0++) {
      A[i0 + ((4 + j) << 3)] = d[i0 + (j << 2)];
    }
  }

  A[36] = 1.0 - 3.90625 * dt;
  A[45] = 1.0 - 0.13391003923564149 * dt;
  A[54] = 1.0 - 0.071370964864073988 * dt;
  A[63] = 1.0 - 0.075714556123414725 * dt;
  for (j = 0; j < 8; j++) {
    mu_p[j] = 0.0;
    for (i0 = 0; i0 < 8; i0++) {
      b_A[j + (i0 << 3)] = 0.0;
      for (i1 = 0; i1 < 8; i1++) {
        b_A[j + (i0 << 3)] += A[j + (i1 << 3)] * cov[i1 + (i0 << 3)];
      }

      mu_p[j] += A[j + (i0 << 3)] * mu[i0];
    }

    for (i0 = 0; i0 < 8; i0++) {
      d0 = 0.0;
      for (i1 = 0; i1 < 8; i1++) {
        d0 += b_A[j + (i1 << 3)] * A[i0 + (i1 << 3)];
      }

      cov_p[j + (i0 << 3)] = d0 + Q_est[j + (i0 << 3)];
    }

    for (i0 = 0; i0 < 4; i0++) {
      Kt[j + (i0 << 3)] = 0.0;
      for (i1 = 0; i1 < 8; i1++) {
        Kt[j + (i0 << 3)] += cov_p[j + (i1 << 3)] * C[i0 + (i1 << 2)];
      }
    }
  }

  for (j = 0; j < 4; j++) {
    for (i0 = 0; i0 < 8; i0++) {
      b_C[j + (i0 << 2)] = 0.0;
      for (i1 = 0; i1 < 8; i1++) {
        b_C[j + (i0 << 2)] += C[j + (i1 << 2)] * cov_p[i1 + (i0 << 3)];
      }
    }

    for (i0 = 0; i0 < 4; i0++) {
      d0 = 0.0;
      for (i1 = 0; i1 < 8; i1++) {
        d0 += b_C[j + (i1 << 2)] * C[i0 + (i1 << 2)];
      }

      d[j + (i0 << 2)] = d0 + R_est[j + (i0 << 2)];
    }
  }

  mrdivide(Kt, d);
  for (j = 0; j < 4; j++) {
    d0 = 0.0;
    for (i0 = 0; i0 < 8; i0++) {
      d0 += C[j + (i0 << 2)] * mu_p[i0];
    }

    v[j] = y[j] - d0;
  }

  for (j = 0; j < 8; j++) {
    d0 = 0.0;
    for (i0 = 0; i0 < 4; i0++) {
      d0 += Kt[j + (i0 << 3)] * v[i0];
    }

    mu_next[j] = mu_p[j] + d0;
  }

  memset(&A[0], 0, sizeof(double) << 6);
  for (j = 0; j < 8; j++) {
    A[j + (j << 3)] = 1.0;
  }

  for (j = 0; j < 8; j++) {
    for (i0 = 0; i0 < 8; i0++) {
      d0 = 0.0;
      for (i1 = 0; i1 < 4; i1++) {
        d0 += Kt[j + (i1 << 3)] * C[i1 + (i0 << 2)];
      }

      b_A[j + (i0 << 3)] = A[j + (i0 << 3)] - d0;
    }

    for (i0 = 0; i0 < 8; i0++) {
      cov_next[j + (i0 << 3)] = 0.0;
      for (i1 = 0; i1 < 8; i1++) {
        cov_next[j + (i0 << 3)] += b_A[j + (i1 << 3)] * cov_p[i1 + (i0 << 3)];
      }
    }
  }
}

/* End of code generation (kalman_filter.cpp) */
