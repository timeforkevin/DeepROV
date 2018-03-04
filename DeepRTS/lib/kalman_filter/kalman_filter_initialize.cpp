/*
 * Classroom License -- for classroom instructional use only.  Not for
 * government, commercial, academic research, or other organizational use.
 *
 * kalman_filter_initialize.cpp
 *
 * Code generation for function 'kalman_filter_initialize'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "kalman_filter.h"
#include "kalman_filter_initialize.h"

// From MATLAB DeepRTS_DLQR.m
double mu[NUM_STATES] =
{
  0,
  0,
  0,
  0
};
double cov[NUM_STATES*NUM_STATES] =
{
  100, 0, 0, 0,
  0, 100, 0, 0,
  0, 0, 100, 0,
  0, 0, 0, 100
};
double y[NUM_STATES] =
{
  0,
  0,
  0,
  0
};
double C[NUM_STATES*NUM_STATES] =
{
  1, 0, 0, 0,
  0, 1, 0, 0,
  0, 0, 1, 0,
  0, 0, 0, 1
};
double Q_est[NUM_STATES*NUM_STATES] =
{
  0.0400, 0, 0, 0,
  0, 0.0003, 0, 0,
  0, 0, 0.0003, 0,
  0, 0, 0, 0.0003
};
double R_est[NUM_STATES*NUM_STATES] =
{
  2.0000, 0, 0, 0,
  0, 0.0001, 0, 0,
  0, 0, 0.0001, 0,
  0, 0, 0, 0.0001
};

/* Function Definitions */
void kalman_filter_initialize()
{
  rt_InitInfAndNaN(8U);
}

/* End of code generation (kalman_filter_initialize.cpp) */
