/*
 * File: Motor_control.h
 *
 * Code generated for Simulink model 'Motor_control'.
 *
 * Model version                  : 1.45
 * Simulink Coder version         : 8.13 (R2017b) 24-Jul-2017
 * C/C++ source code generated on : Wed May 21 23:19:37 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_Motor_control_h_
#define RTW_HEADER_Motor_control_h_
#include <math.h>
#ifndef Motor_control_COMMON_INCLUDES_
# define Motor_control_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* Motor_control_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */

/* Block signals and states (auto storage) for system '<Root>' */
typedef struct {
  real_T DiscreteTransferFcn3_states[2];/* '<Root>/Discrete Transfer Fcn3' */
  real_T DiscreteTransferFcn4_states[2];/* '<Root>/Discrete Transfer Fcn4' */
  real_T Referencemodel_states[2];     /* '<Root>/Reference model' */
  real_T Integrator_DSTATE;            /* '<S1>/Integrator' */
  real_T DiscreteTransferFcn7_states;  /* '<Root>/Discrete Transfer Fcn7' */
  real_T DiscreteTransferFcn6_states;  /* '<Root>/Discrete Transfer Fcn6' */
  real_T DiscreteTransferFcn5_states;  /* '<Root>/Discrete Transfer Fcn5' */
  int32_T counter;                     /* '<Root>/Sine Wave' */
} DW;

/* Block signals and states (auto storage) */
extern DW rtDW;

/*
 * Exported Global Signals
 *
 * Note: Exported global signals are block signals with an exported global
 * storage class designation.  Code generation will declare the memory for
 * these signals and export their symbols.
 *
 */
extern real_T current_speed;           /* '<Root>/current_speed' */
extern real_T set_point;               /* '<Root>/Set_point ' */
extern real_T set_point_mras;          /* '<Root>/In1' */
extern real_T current_speed_PImras;    /* '<Root>/In2' */
extern real_T error;                   /* '<Root>/Sum' */
extern real_T signal_control;          /* '<Root>/Saturation' */
extern real_T set_point_sine;          /* '<Root>/Sine Wave' */
extern real_T signal_controlPImras;    /* '<Root>/Saturation2' */
extern real_T current_speed_model;     /* '<Root>/Reference model' */
extern real_T error_PImras;            /* '<Root>/Sum4' */

/*
 * Exported Global Parameters
 *
 * Note: Exported global parameters are tunable parameters with an exported
 * global storage class designation.  Code generation will declare the memory for
 * these parameters and exports their symbols.
 *
 */
extern real_T gi;                      /* Variable: gi
                                        * Referenced by: '<Root>/gamma3'
                                        */
extern real_T gp;                      /* Variable: gp
                                        * Referenced by: '<Root>/gamma2'
                                        */
extern real_T ki;                      /* Variable: ki
                                        * Referenced by: '<S1>/Integral Gain'
                                        */
extern real_T kp;                      /* Variable: kp
                                        * Referenced by: '<S1>/Proportional Gain'
                                        */

/* Model entry point functions */
extern void Motor_control_initialize(void);
extern void Motor_control_step(void);

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'Motor_control'
 * '<S1>'   : 'Motor_control/PID Controller'
 */
#endif                                 /* RTW_HEADER_Motor_control_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
