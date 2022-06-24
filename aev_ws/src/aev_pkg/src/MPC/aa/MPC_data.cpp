//
// File: MPC_data.cpp
//
// Code generated for Simulink model 'MPC'.
//
// Model version                  : 1.27
// Simulink Coder version         : 9.4 (R2020b) 29-Jul-2020
// C/C++ source code generated on : Sun Nov 21 18:06:24 2021
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "MPC.h"
#include "MPC_private.h"

// Invariant block signals (default storage)
const MPCModelClass::ConstB_MPC_T MPC_ConstB = {
  3.0
  ,                                    // '<S7>/Data Type Conversion22'
  3.0
  ,                                    // '<S7>/Data Type Conversion23'
  0.0
  ,                                    // '<S7>/Data Type Conversion8'

  {
    0.0,
    0.0
  }
  ,                                    // '<S7>/Math Function'
  0.0
  ,                                    // '<S7>/Math Function1'
  0.0
  ,                                    // '<S7>/Math Function2'
  12.063492063492063
  ,                                    // '<S38>/Divide'
  24.126984126984127
  ,                                    // '<S38>/Gain'
  7.9304347826086961
  ,                                    // '<S38>/Divide1'
  15.860869565217392
  ,                                    // '<S38>/Gain1'
  84480.000000000015
  ,                                    // '<S38>/Divide6'
  27360.0
  ,                                    // '<S38>/Divide7'
  52000.0
  ,                                    // '<S38>/Sum'
  -104000.0
  ,                                    // '<S38>/Gain2'
  52800.0
  ,                                    // '<S38>/Product'
  22800.0
  ,                                    // '<S38>/Product1'
  -30000.0
  ,                                    // '<S38>/Sum1'
  60000.0
  ,                                    // '<S38>/Gain3'
  111840.00000000001
  ,                                    // '<S38>/Sum3'
  -223680.00000000003
  ,                                    // '<S38>/Gain4'
  0U
  // '<S2>/Switch'
};

// Constant parameters (default storage)
const MPCModelClass::ConstP_MPC_T MPC_ConstP = {
  // Expression: lastPcov
  //  Referenced by: '<S7>/LastPcov'

  { 0.54363269209673259, 0.38838008933431462, -0.071531568145133326,
    -0.028280807001852516, 0.026986068896119327, 0.38838008933431462,
    1.31080570982802, 0.32742793223742245, 0.17800810203494544,
    -0.0033958017073179129, -0.071531568145133326, 0.32742793223742245,
    0.38448836559015964, 0.1028820196854511, -0.12252027899228292,
    -0.028280807001852516, 0.17800810203494544, 0.1028820196854511,
    0.043957908611054183, -0.019942338017053168, 0.026986068896119327,
    -0.0033958017073179129, -0.12252027899228292, -0.019942338017053168,
    0.27950228221271567 }
};

//
// File trailer for generated code.
//
// [EOF]
//
