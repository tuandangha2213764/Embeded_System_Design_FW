/*
 * File: Motor_control.c
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

#include "Motor_control.h"
#include "math.h"

/* Exported block signals */
real_T current_speed;                  /* '<Root>/current_speed' */
real_T set_point;                      /* '<Root>/Set_point ' */
real_T set_point_mras;                 /* '<Root>/In1' */
real_T current_speed_PImras;           /* '<Root>/In2' */
real_T error;                          /* '<Root>/Sum' */
real_T signal_control;                 /* '<Root>/Saturation' */
real_T set_point_sine;                 /* '<Root>/Sine Wave' */
real_T signal_controlPImras;           /* '<Root>/Saturation2' */
real_T current_speed_model;            /* '<Root>/Reference model' */
real_T error_PImras;                   /* '<Root>/Sum4' */

/* Exported block parameters */
real_T gi = 0.0;                       /* Variable: gi
                                        * Referenced by: '<Root>/gamma3'
                                        */
real_T gp = 0.0;                       /* Variable: gp
                                        * Referenced by: '<Root>/gamma2'
                                        */
real_T ki = 0.0;                       /* Variable: ki
                                        * Referenced by: '<S1>/Integral Gain'
                                        */
real_T kp = 0.0;                       /* Variable: kp
                                        * Referenced by: '<S1>/Proportional Gain'
                                        */

/* Block signals and states (auto storage) */
DW rtDW;

/* Model step function */
void Motor_control_step(void)
{
  real_T rtb_DiscreteTransferFcn3;
  real_T ucy;

  /* Sum: '<Root>/Sum' incorporates:
   *  Inport: '<Root>/Set_point '
   *  Inport: '<Root>/current_speed'
   */
  error = fabsf(set_point) - fabsf(current_speed);

  /* Sum: '<S1>/Sum' incorporates:
   *  DiscreteIntegrator: '<S1>/Integrator'
   *  Gain: '<S1>/Proportional Gain'
   */
  rtb_DiscreteTransferFcn3 = kp * error + rtDW.Integrator_DSTATE;

  /* Saturate: '<Root>/Saturation' */
  if (rtb_DiscreteTransferFcn3 > 100.0) {
    signal_control = 100.0;
  } else if (rtb_DiscreteTransferFcn3 < 0.0) {
    signal_control = 0.0;
  } else {
    signal_control = rtb_DiscreteTransferFcn3;
  }

  /* End of Saturate: '<Root>/Saturation' */

  /* Sin: '<Root>/Sine Wave' */
  set_point_sine = sin((real_T)rtDW.counter * 2.0 * 3.1415926535897931 / 9999.0)
    * 100.0 + 150.0;

  /* Sum: '<Root>/Sum6' incorporates:
   *  Inport: '<Root>/In1'
   *  Inport: '<Root>/In2'
   */
  ucy = set_point_mras - current_speed_PImras;

  /* Sum: '<Root>/Sum5' incorporates:
   *  DiscreteTransferFcn: '<Root>/Discrete Transfer Fcn5'
   *  DiscreteTransferFcn: '<Root>/Discrete Transfer Fcn6'
   *  DiscreteTransferFcn: '<Root>/Discrete Transfer Fcn7'
   *  Gain: '<Root>/gamma2'
   *  Gain: '<Root>/gamma3'
   *  Product: '<Root>/Product'
   *  Product: '<Root>/Product5'
   */
  rtb_DiscreteTransferFcn3 = 0.1 * rtDW.DiscreteTransferFcn7_states * gp * ucy +
    0.1 * rtDW.DiscreteTransferFcn6_states * gi * (0.1 *
    rtDW.DiscreteTransferFcn5_states);

  /* Saturate: '<Root>/Saturation2' */
  if (rtb_DiscreteTransferFcn3 > 100.0) {
    signal_controlPImras = 100.0;
  } else if (rtb_DiscreteTransferFcn3 < 0.0) {
    signal_controlPImras = 0.0;
  } else {
    signal_controlPImras = rtb_DiscreteTransferFcn3;
  }

  /* End of Saturate: '<Root>/Saturation2' */

  /* DiscreteTransferFcn: '<Root>/Reference model' */
  current_speed_model = 0.044618379746072 * rtDW.Referencemodel_states[0] +
    0.0390238756969775 * rtDW.Referencemodel_states[1];

  /* Sum: '<Root>/Sum4' incorporates:
   *  Inport: '<Root>/In2'
   */
  error_PImras = current_speed_PImras - current_speed_model;

  /* Update for DiscreteIntegrator: '<S1>/Integrator' incorporates:
   *  Gain: '<S1>/Integral Gain'
   */
  rtDW.Integrator_DSTATE += ki * error * 0.1;

  /* Update for Sin: '<Root>/Sine Wave' */
  rtDW.counter++;
  if (rtDW.counter == 9999) {
    rtDW.counter = 0;
  }

  /* End of Update for Sin: '<Root>/Sine Wave' */

  /* Update for DiscreteTransferFcn: '<Root>/Discrete Transfer Fcn7' incorporates:
   *  DiscreteTransferFcn: '<Root>/Discrete Transfer Fcn3'
   *  Product: '<Root>/Product4'
   */
  rtDW.DiscreteTransferFcn7_states = (-0.324771506321578 *
    rtDW.DiscreteTransferFcn3_states[0] + 0.324771506321578 *
    rtDW.DiscreteTransferFcn3_states[1]) * error_PImras -
    (-rtDW.DiscreteTransferFcn7_states);

  /* Update for DiscreteTransferFcn: '<Root>/Discrete Transfer Fcn6' incorporates:
   *  DiscreteTransferFcn: '<Root>/Discrete Transfer Fcn4'
   *  Product: '<Root>/Product3'
   */
  rtDW.DiscreteTransferFcn6_states = (-0.0174736729680554 *
    rtDW.DiscreteTransferFcn4_states[0] + -0.0152827253198288 *
    rtDW.DiscreteTransferFcn4_states[1]) * error_PImras -
    (-rtDW.DiscreteTransferFcn6_states);

  /* Update for DiscreteTransferFcn: '<Root>/Discrete Transfer Fcn5' */
  rtDW.DiscreteTransferFcn5_states = ucy - (-rtDW.DiscreteTransferFcn5_states);

  /* Update for DiscreteTransferFcn: '<Root>/Discrete Transfer Fcn3' */
  rtb_DiscreteTransferFcn3 = (ucy - -1.58599173418628 *
    rtDW.DiscreteTransferFcn3_states[0]) - 0.669633989629328 *
    rtDW.DiscreteTransferFcn3_states[1];
  rtDW.DiscreteTransferFcn3_states[1] = rtDW.DiscreteTransferFcn3_states[0];
  rtDW.DiscreteTransferFcn3_states[0] = rtb_DiscreteTransferFcn3;

  /* Update for DiscreteTransferFcn: '<Root>/Discrete Transfer Fcn4' */
  rtb_DiscreteTransferFcn3 = (ucy - -1.58599173418628 *
    rtDW.DiscreteTransferFcn4_states[0]) - 0.669633989629328 *
    rtDW.DiscreteTransferFcn4_states[1];
  rtDW.DiscreteTransferFcn4_states[1] = rtDW.DiscreteTransferFcn4_states[0];
  rtDW.DiscreteTransferFcn4_states[0] = rtb_DiscreteTransferFcn3;

  /* Update for DiscreteTransferFcn: '<Root>/Reference model' incorporates:
   *  Inport: '<Root>/In1'
   */
  rtb_DiscreteTransferFcn3 = (set_point_mras - -1.58599173418628 *
    rtDW.Referencemodel_states[0]) - 0.669633989629328 *
    rtDW.Referencemodel_states[1];
  rtDW.Referencemodel_states[1] = rtDW.Referencemodel_states[0];
  rtDW.Referencemodel_states[0] = rtb_DiscreteTransferFcn3;
}

/* Model initialize function */
void Motor_control_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
