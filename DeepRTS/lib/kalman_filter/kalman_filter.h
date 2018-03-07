/*
 * Classroom License -- for classroom instructional use only.  Not for
 * government, commercial, academic research, or other organizational use.
 *
 * kalman_filter.h
 *
 * Code generation for function 'kalman_filter'
 *
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "kalman_filter_types.h"

/* Function Declarations */
extern void kalman_filter(const double mu[8], const double cov[64], const double
  y[4], const double C[32], const double Q_est[64], const double R_est[16],
  double dt, double mu_next[8], double cov_next[64]);

#endif

/* End of code generation (kalman_filter.h) */
