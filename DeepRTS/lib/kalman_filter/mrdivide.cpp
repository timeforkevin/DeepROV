/*
 * Classroom License -- for classroom instructional use only.  Not for
 * government, commercial, academic research, or other organizational use.
 *
 * mrdivide.cpp
 *
 * Code generation for function 'mrdivide'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "kalman_filter.h"
#include "mrdivide.h"

/* Function Definitions */
void mrdivide(double A[32], const double B[16])
{
  double b_A[16];
  int k;
  int j;
  signed char ipiv[4];
  int c;
  int kBcol;
  int jp;
  int ix;
  int jAcol;
  double temp;
  int i;
  double s;
  memcpy(&b_A[0], &B[0], sizeof(double) << 4);
  for (k = 0; k < 4; k++) {
    ipiv[k] = (signed char)(1 + k);
  }

  for (j = 0; j < 3; j++) {
    c = j * 5;
    kBcol = 0;
    ix = c;
    temp = fabs(b_A[c]);
    for (k = 2; k <= 4 - j; k++) {
      ix++;
      s = fabs(b_A[ix]);
      if (s > temp) {
        kBcol = k - 1;
        temp = s;
      }
    }

    if (b_A[c + kBcol] != 0.0) {
      if (kBcol != 0) {
        ipiv[j] = (signed char)((j + kBcol) + 1);
        ix = j;
        kBcol += j;
        for (k = 0; k < 4; k++) {
          temp = b_A[ix];
          b_A[ix] = b_A[kBcol];
          b_A[kBcol] = temp;
          ix += 4;
          kBcol += 4;
        }
      }

      k = (c - j) + 4;
      for (i = c + 1; i + 1 <= k; i++) {
        b_A[i] /= b_A[c];
      }
    }

    jp = c;
    jAcol = c + 4;
    for (kBcol = 1; kBcol <= 3 - j; kBcol++) {
      temp = b_A[jAcol];
      if (b_A[jAcol] != 0.0) {
        ix = c + 1;
        k = (jp - j) + 8;
        for (i = 5 + jp; i + 1 <= k; i++) {
          b_A[i] += b_A[ix] * -temp;
          ix++;
        }
      }

      jAcol += 4;
      jp += 4;
    }
  }

  for (j = 0; j < 4; j++) {
    jp = j << 3;
    jAcol = j << 2;
    for (k = 1; k <= j; k++) {
      kBcol = (k - 1) << 3;
      if (b_A[(k + jAcol) - 1] != 0.0) {
        for (i = 0; i < 8; i++) {
          A[i + jp] -= b_A[(k + jAcol) - 1] * A[i + kBcol];
        }
      }
    }

    temp = 1.0 / b_A[j + jAcol];
    for (i = 0; i < 8; i++) {
      A[i + jp] *= temp;
    }
  }

  for (j = 3; j >= 0; j += -1) {
    jp = j << 3;
    jAcol = (j << 2) - 1;
    for (k = j + 2; k < 5; k++) {
      kBcol = (k - 1) << 3;
      if (b_A[k + jAcol] != 0.0) {
        for (i = 0; i < 8; i++) {
          A[i + jp] -= b_A[k + jAcol] * A[i + kBcol];
        }
      }
    }
  }

  for (kBcol = 2; kBcol >= 0; kBcol += -1) {
    if (ipiv[kBcol] != kBcol + 1) {
      jp = ipiv[kBcol] - 1;
      for (jAcol = 0; jAcol < 8; jAcol++) {
        temp = A[jAcol + (kBcol << 3)];
        A[jAcol + (kBcol << 3)] = A[jAcol + (jp << 3)];
        A[jAcol + (jp << 3)] = temp;
      }
    }
  }
}

/* End of code generation (mrdivide.cpp) */
