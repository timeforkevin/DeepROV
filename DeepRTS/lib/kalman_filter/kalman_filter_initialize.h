/*
 * Classroom License -- for classroom instructional use only.  Not for
 * government, commercial, academic research, or other organizational use.
 *
 * kalman_filter_initialize.h
 *
 * Code generation for function 'kalman_filter_initialize'
 *
 */

#ifndef KALMAN_FILTER_INITIALIZE_H
#define KALMAN_FILTER_INITIALIZE_H

/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "kalman_filter_types.h"

#define NUM_STATES 4
extern double mu[NUM_STATES];
extern double cov[NUM_STATES*NUM_STATES];
extern double y[NUM_STATES];
extern double C[NUM_STATES*NUM_STATES];
extern double Q_est[NUM_STATES*NUM_STATES];
extern double R_est[NUM_STATES*NUM_STATES];

/* Function Declarations */
extern void kalman_filter_initialize();

#endif

/* End of code generation (kalman_filter_initialize.h) */
