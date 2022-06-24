//
// File: GenCode_MPC_types.h
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
#ifndef RTW_HEADER_GenCode_MPC_types_h_
#define RTW_HEADER_GenCode_MPC_types_h_
#include "rtwtypes.h"

// Model Code Variants
#ifndef DEFINED_TYPEDEF_FOR_struct_qYRJtcce7MM7XuQ3AAWdMD_
#define DEFINED_TYPEDEF_FOR_struct_qYRJtcce7MM7XuQ3AAWdMD_

typedef struct {
  real_T MaxIterations;
  real_T ConstraintTolerance;
  boolean_T UseWarmStart;
} struct_qYRJtcce7MM7XuQ3AAWdMD;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_SmvKLCDySlKdToNTroAGyF_
#define DEFINED_TYPEDEF_FOR_struct_SmvKLCDySlKdToNTroAGyF_

typedef struct {
  real_T MaxIterations;
  real_T ConstraintTolerance;
  real_T OptimalityTolerance;
  real_T ComplementarityTolerance;
  real_T StepTolerance;
} struct_SmvKLCDySlKdToNTroAGyF;

#endif

#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T

struct emxArray_real_T
{
  real_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_real_T

#ifndef typedef_emxArray_real_T_GenCode_MPC_T
#define typedef_emxArray_real_T_GenCode_MPC_T

typedef emxArray_real_T emxArray_real_T_GenCode_MPC_T;

#endif                                 //typedef_emxArray_real_T_GenCode_MPC_T

#ifndef struct_emxArray_boolean_T
#define struct_emxArray_boolean_T

struct emxArray_boolean_T
{
  boolean_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_boolean_T

#ifndef typedef_emxArray_boolean_T_GenCode_MP_T
#define typedef_emxArray_boolean_T_GenCode_MP_T

typedef emxArray_boolean_T emxArray_boolean_T_GenCode_MP_T;

#endif                                 //typedef_emxArray_boolean_T_GenCode_MP_T

#ifndef struct_emxArray_int16_T
#define struct_emxArray_int16_T

struct emxArray_int16_T
{
  int16_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_int16_T

#ifndef typedef_emxArray_int16_T_GenCode_MPC_T
#define typedef_emxArray_int16_T_GenCode_MPC_T

typedef emxArray_int16_T emxArray_int16_T_GenCode_MPC_T;

#endif                                 //typedef_emxArray_int16_T_GenCode_MPC_T
#endif                                 // RTW_HEADER_GenCode_MPC_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
