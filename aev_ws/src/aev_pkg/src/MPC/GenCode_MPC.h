//
// File: GenCode_MPC.h
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
#ifndef RTW_HEADER_GenCode_MPC_h_
#define RTW_HEADER_GenCode_MPC_h_
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <math.h>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "GenCode_MPC_types.h"
#include "rt_nonfinite.h"
#include "rt_assert.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

#ifndef ODE4_INTG
#define ODE4_INTG

// ODE4 Integration Data
typedef struct {
  real_T *y;                           // output
  real_T *f[4];                        // derivatives
} ODE4_IntgData;

#endif

// Class declaration for model GenCode_MPC
class MPCModelClass {
  // public data and function members
 public:
  // Block signals (default storage)
  typedef struct {
    real_T unusedU0_data[32767];
    int16_T iC_data[32767];
    real_T Mv_aux_data[6720];
    real_T Mu_data[3200];
    real_T Hv_data[1680];
    real_T varargin_1_data[1680];
    real_T tmp_data[882];
    real_T tmp_data_m[882];
    real_T tmp_data_c[882];
    real_T tmp_data_k[882];
    real_T Kv_data[840];
    real_T Su_data[800];
    real_T SuJm_data[800];
    real_T y_data[800];
    real_T tmp_data_cx[800];
    real_T e2dot;                      // '<S5>/Subtract'
    real_T xk1[5];                     // '<S35>/VariableHorizonOptimizer'
    real_T u;                          // '<S35>/VariableHorizonOptimizer'
    real_T Pk1[25];                    // '<S35>/VariableHorizonOptimizer'
    boolean_T iAout[4];                // '<S35>/VariableHorizonOptimizer'
  } B_GenCode_MPC_T;

  // Block states (default storage) for system '<Root>'
  typedef struct {
    real_T last_mv_DSTATE;             // '<S7>/last_mv'
    real_T last_x_PreviousInput[5];    // '<S7>/last_x'
    real_T LastPcov_PreviousInput[25]; // '<S7>/LastPcov'
    boolean_T Memory_PreviousInput[4]; // '<S7>/Memory'
  } DW_GenCode_MPC_T;

  // Continuous states (default storage)
  typedef struct {
    real_T Integrator2_CSTATE;         // '<S5>/Integrator2'
  } X_GenCode_MPC_T;

  // State derivatives (default storage)
  typedef struct {
    real_T Integrator2_CSTATE;         // '<S5>/Integrator2'
  } XDot_GenCode_MPC_T;

  // State disabled
  typedef struct {
    boolean_T Integrator2_CSTATE;      // '<S5>/Integrator2'
  } XDis_GenCode_MPC_T;

  // Invariant block signals (default storage)
  typedef const struct tag_ConstB_GenCode_MPC_T {
    real_T DataTypeConversion10;       // '<S7>/Data Type Conversion10'
    real_T DataTypeConversion22;       // '<S7>/Data Type Conversion22'
    real_T DataTypeConversion23;       // '<S7>/Data Type Conversion23'
    real_T DataTypeConversion8;        // '<S7>/Data Type Conversion8'
    real_T DataTypeConversion9[2];     // '<S7>/Data Type Conversion9'
    real_T MathFunction[2];            // '<S7>/Math Function'
    real_T MathFunction1;              // '<S7>/Math Function1'
    real_T MathFunction2;              // '<S7>/Math Function2'
    real_T Divide;                     // '<S38>/Divide'
    real_T b1;                         // '<S38>/Gain'
    real_T Divide1;                    // '<S38>/Divide1'
    real_T b2;                         // '<S38>/Gain1'
    real_T Divide6;                    // '<S38>/Divide6'
    real_T Divide7;                    // '<S38>/Divide7'
    real_T Sum;                        // '<S38>/Sum'
    real_T Gain2;                      // '<S38>/Gain2'
    real_T Product;                    // '<S38>/Product'
    real_T Product1;                   // '<S38>/Product1'
    real_T Sum1;                       // '<S38>/Sum1'
    real_T Gain3;                      // '<S38>/Gain3'
    real_T Sum3;                       // '<S38>/Sum3'
    real_T Gain4;                      // '<S38>/Gain4'
    uint8_T Switch;                    // '<S2>/Switch'
  } ConstB_GenCode_MPC_T;

  // Constant parameters (default storage)
  typedef struct {
    // Expression: lastPcov
    //  Referenced by: '<S7>/LastPcov'

    real_T LastPcov_InitialCondition[25];
  } ConstP_GenCode_MPC_T;

  // External inputs (root inport signals with default storage)
  typedef struct {
    real_T LongitudinalVelocity;       // '<Root>/LongitudinalVelocity'
    real_T CurvaturePreview[10];       // '<Root>/CurvaturePreview'
    real_T CurrentCurvature;           // '<Root>/CurrentCurvature'
    real_T LateralDeviation;           // '<Root>/LateralDeviation'
    real_T YawRate;                    // '<Root>/YawRate'
  } ExtU_GenCode_MPC_T;

  // External outputs (root outports fed by signals with default storage)
  typedef struct {
    real_T Steeringangle;              // '<Root>/Steering angle'
  } ExtY_GenCode_MPC_T;

  // Real-time Model Data Structure
  struct RT_MODEL_GenCode_MPC_T {
    const char_T *errorStatus;
    RTWSolverInfo solverInfo;
    X_GenCode_MPC_T *contStates;
    int_T *periodicContStateIndices;
    real_T *periodicContStateRanges;
    real_T *derivs;
    boolean_T *contStateDisabled;
    boolean_T zCCacheNeedsReset;
    boolean_T derivCacheNeedsReset;
    boolean_T CTOutputIncnstWithState;
    real_T odeY[1];
    real_T odeF[4][1];
    ODE4_IntgData intgData;

    //
    //  Sizes:
    //  The following substructure contains sizes information
    //  for many of the model attributes such as inputs, outputs,
    //  dwork, sample times, etc.

    struct {
      int_T numContStates;
      int_T numPeriodicContStates;
      int_T numSampTimes;
    } Sizes;

    //
    //  Timing:
    //  The following substructure contains information regarding
    //  the timing information for the model.

    struct {
      uint32_T clockTick0;
      time_T stepSize0;
      uint32_T clockTick1;
      SimTimeStep simTimeStep;
      boolean_T stopRequestedFlag;
      time_T *t;
      time_T tArray[2];
    } Timing;
  };

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  void terminate();

  // Constructor
  MPCModelClass();

  // Destructor
  ~MPCModelClass();

  // Root-level structure-based inputs set method

  // Root inports set method
  void setExternalInputs(const ExtU_GenCode_MPC_T* pExtU_GenCode_MPC_T)
  {
    GenCode_MPC_U = *pExtU_GenCode_MPC_T;
  }

  // Root-level structure-based outputs get method

  // Root outports get method
  const MPCModelClass::ExtY_GenCode_MPC_T & getExternalOutputs() const
  {
    return GenCode_MPC_Y;
  }

  // Real-Time Model get method
  MPCModelClass::RT_MODEL_GenCode_MPC_T * getRTM();

  // private data and function members
 private:
  // Block signals
  B_GenCode_MPC_T GenCode_MPC_B;

  // Block states
  DW_GenCode_MPC_T GenCode_MPC_DW;
  X_GenCode_MPC_T GenCode_MPC_X;       // Block continuous states

  // External inputs
  ExtU_GenCode_MPC_T GenCode_MPC_U;

  // External outputs
  ExtY_GenCode_MPC_T GenCode_MPC_Y;

  // Real-Time Model
  RT_MODEL_GenCode_MPC_T GenCode_MPC_M;

  // private member function(s) for subsystem '<Root>'
  void GenCode_MPC_lusolve(const real_T A[16], const real_T B[16], real_T X[16]);
  void GenCode_PadeApproximantOfDegree(const real_T A[16], uint8_T m, real_T F
    [16]);
  void GenCode_MPC_expm(real_T A[16], real_T F[16]);
  void GenCode_MPC_emxInit_real_T(emxArray_real_T_GenCode_MPC_T **pEmxArray,
    int32_T numDimensions);
  void GenCod_emxEnsureCapacity_real_T(emxArray_real_T_GenCode_MPC_T *emxArray,
    int32_T oldNumel);
  void GenCode_MPC_emxFree_real_T(emxArray_real_T_GenCode_MPC_T **pEmxArray);
  void GenCode_MPC_emxFree_boolean_T(emxArray_boolean_T_GenCode_MP_T **pEmxArray);
  void GenCode_MPC_emxInit_int16_T(emxArray_int16_T_GenCode_MPC_T **pEmxArray,
    int32_T numDimensions);
  void GenCo_emxEnsureCapacity_int16_T(emxArray_int16_T_GenCode_MPC_T *emxArray,
    int32_T oldNumel);
  real_T GenCode_MPC_norm(const real_T x_data[], const int32_T *x_size);
  void GenCode_MPC_abs_d(const emxArray_real_T_GenCode_MPC_T *x,
    emxArray_real_T_GenCode_MPC_T *y);
  void GenCode_MPC_maximum2(const emxArray_real_T_GenCode_MPC_T *x,
    emxArray_real_T_GenCode_MPC_T *ex);
  real_T GenCode_MPC_xnrm2(int32_T n, const real_T x_data[], int32_T ix0);
  void GenCode_MPC_xgemv(int32_T b_m, int32_T n, const real_T b_A_data[],
    int32_T ia0, int32_T lda, const real_T x_data[], int32_T ix0, real_T y_data[]);
  void GenCode_MPC_xgerc(int32_T b_m, int32_T n, real_T alpha1, int32_T ix0,
    const real_T y_data[], real_T b_A_data[], int32_T ia0, int32_T lda);
  void GenCode_MPC_xzlarf(int32_T b_m, int32_T n, int32_T iv0, real_T tau,
    real_T b_C_data[], int32_T ic0, int32_T ldc, real_T work_data[]);
  void GenCode_MPC_qrf(real_T b_A_data[], const int32_T b_A_size[2], int32_T b_m,
                       int32_T n, int32_T nfxd, real_T tau_data[]);
  void GenCode_MPC_xzgeqp3(real_T b_A_data[], const int32_T b_A_size[2], int32_T
    b_m, int32_T n, real_T tau_data[], int32_T *tau_size);
  void GenCode_MPC_xgeqrf(real_T b_A_data[], int32_T b_A_size[2], real_T
    tau_data[], int32_T *tau_size);
  void GenCode_MPC_xorgqr(int32_T b_m, int32_T n, int32_T k, real_T b_A_data[],
    const int32_T b_A_size[2], int32_T lda, const real_T tau_data[]);
  void GenCode_MPC_qr(const real_T b_A_data[], const int32_T b_A_size[2], real_T
                      Q_data[], int32_T Q_size[2], real_T R_data[], int32_T
                      R_size[2]);
  void GenCode_MPC_mtimes_ojosf(const real_T b_A_data[], const int32_T b_A_size
    [2], const real_T b_B_data[], real_T b_C_data[], int32_T *b_C_size);
  real_T GenCode_MPC_KWIKfactor(const emxArray_real_T_GenCode_MPC_T *b_Ac, const
    int16_T iC_data[], int16_T nA, const real_T Linv_data[], const int32_T
    Linv_size[2], real_T RLinv_data[], const int32_T RLinv_size[2], real_T
    b_D_data[], const int32_T b_D_size[2], real_T b_H_data[], const int32_T
    b_H_size[2], int16_T n);
  void GenCode_MPC_abs(const real_T x_data[], const int32_T x_size[2], real_T
                       y_data[], int32_T y_size[2]);
  void GenCode_MPC_qpkwik(const real_T Linv_data[], const int32_T Linv_size[2],
    const real_T Hinv_data[], const int32_T Hinv_size[2], const real_T f_data[],
    const emxArray_real_T_GenCode_MPC_T *b_Ac, const
    emxArray_real_T_GenCode_MPC_T *b, emxArray_int16_T_GenCode_MPC_T *iA,
    int16_T b_m, int16_T n, real_T x_data[], int32_T *x_size, real_T
    lambda_data[], int32_T *lambda_size, real_T *status);
  void GenCode_MPC_emxFree_int16_T(emxArray_int16_T_GenCode_MPC_T **pEmxArray);
  void GenCode_MPC_mpc_solveQP(const real_T xQP[5], real_T nCon, real_T
    b_degrees, const real_T Kx_data[], const real_T Kr_data[], const int32_T
    Kr_size[2], const emxArray_real_T_GenCode_MPC_T *rseq, const real_T
    Ku1_data[], real_T old_u, const real_T Kv_data[], const int32_T Kv_size[2],
    const emxArray_real_T_GenCode_MPC_T *vseq, const real_T Kut_data[], const
    int32_T Kut_size[2], const real_T b_utarget_data[], const real_T Linv_data[],
    const int32_T Linv_size[2], const real_T Hinv_data[], const int32_T
    Hinv_size[2], const emxArray_real_T_GenCode_MPC_T *b_Ac, const
    emxArray_real_T_GenCode_MPC_T *Bc, const emxArray_boolean_T_GenCode_MP_T *iA,
    real_T zopt_data[], int32_T *zopt_size, real_T f_data[], int32_T *f_size,
    real_T *status);
  void Gen_emxEnsureCapacity_boolean_T(emxArray_boolean_T_GenCode_MP_T *emxArray,
    int32_T oldNumel);
  void GenCode_MPC_mtimes_o(const real_T b_A_data[], const int32_T b_A_size[2],
    const real_T b_B_data[], const int32_T b_B_size[2], real_T b_C_data[],
    int32_T b_C_size[2]);
  void GenCode_MPC_emxInit_boolean_T(emxArray_boolean_T_GenCode_MP_T **pEmxArray,
    int32_T numDimensions);
  void GenCode_MPC_mtimes_ojos(const emxArray_real_T_GenCode_MPC_T *b_A, const
    emxArray_real_T_GenCode_MPC_T *b_B, emxArray_real_T_GenCode_MPC_T *b_C);
  void GenCode_MPC_mtimes_ojo(const emxArray_real_T_GenCode_MPC_T *b_A, const
    real_T b_B[5], emxArray_real_T_GenCode_MPC_T *b_C);
  void GenCode_MPC_trisolve(const real_T b_A_data[], const int32_T b_A_size[2],
    real_T b_B_data[], const int32_T b_B_size[2]);
  void GenCode_MPC_linsolve(const real_T b_A_data[], const int32_T b_A_size[2],
    const real_T b_B_data[], const int32_T b_B_size[2], real_T b_C_data[],
    int32_T b_C_size[2]);
  void GenCode_MPC_eye_g(real_T varargin_1, real_T b_I_data[], int32_T b_I_size
    [2]);
  int32_T GenCode_MPC_xpotrf(int32_T n, real_T b_A_data[], int32_T lda);
  void GenCode_MPC_diag(const real_T v_data[], const int32_T v_size[2], real_T
                        d_data[], int32_T *d_size);
  real_T GenCode_MPC_minimum(const real_T x_data[], const int32_T *x_size);
  void GenCode_MPC_mpc_checkhessian(real_T b_H_data[], const int32_T b_H_size[2],
    real_T L_data[], int32_T L_size[2], real_T *BadH);
  void GenCode_MPC_WtMult(const real_T W[2], const real_T M_data[], const
    int32_T M_size[2], real_T WM_data[], int32_T WM_size[2]);
  void GenCode_MPC_WtMult_l(real_T W, const real_T M_data[], const int32_T
    M_size[2], real_T WM_data[], int32_T WM_size[2]);
  void GenCode_MPC_WtMult_lm(const real_T M_data[], const int32_T M_size[2],
    real_T WM_data[], int32_T WM_size[2]);
  void GenCode_MPC_mtimes_oj(const real_T b_A_data[], const int32_T *b_A_size,
    const real_T b_B_data[], const int32_T b_B_size[2], real_T b_C_data[],
    int32_T b_C_size[2]);
  void GenCode_MP_mpc_calculatehessian(const real_T b_Wy[2], real_T b_Wu, const
    real_T SuJm_data[], const int32_T SuJm_size[2], const real_T I2Jm_data[],
    const int32_T I2Jm_size[2], const real_T Jm_data[], const int32_T Jm_size[2],
    const real_T I1_data[], const int32_T *I1_size, const real_T Su1_data[],
    const int32_T *Su1_size, const real_T Sx_data[], const int32_T Sx_size[2],
    const real_T Hv_data[], const int32_T Hv_size[2], real_T b_H_data[], int32_T
    b_H_size[2], real_T Ku1_data[], int32_T Ku1_size[2], real_T Kut_data[],
    int32_T Kut_size[2], real_T Kx_data[], int32_T Kx_size[2], real_T Kv_data[],
    int32_T Kv_size[2], real_T Kr_data[], int32_T Kr_size[2]);
  void GenCode_MPC_updateWeights(real_T W[2], const real_T b_signal[2]);
  void GenCode_MPC_eye(real_T varargin_1, real_T b_I_data[], int32_T b_I_size[2]);
  void GenCode_MPC_mtimes(const real_T b_A_data[], const int32_T b_A_size[2],
    const real_T b_B_data[], const int32_T b_B_size[2], real_T b_C_data[],
    int32_T b_C_size[2]);
  void GenCode_MPC_kron_b(const real_T b_A_data[], const int32_T b_A_size[2],
    real_T K_data[], int32_T K_size[2]);
  void GenCode_MPC_tril(real_T x_data[], const int32_T x_size[2]);
  int32_T GenCode_M_combineVectorElements(const boolean_T x_data[], const
    int32_T *x_size);
  void GenCode_MPC_Mrows_reshape_lgbo5(boolean_T isMrows_data[], real_T
    Mlimfull_data[], real_T Vfull_data[], const boolean_T isMrows0[70], const
    real_T Mlimfull0[70], const real_T Vfull0[70], real_T b_p, real_T ioff);
  void GenCode_MPC_Mrows_reshape_lgbo(boolean_T isMrows_data[], real_T
    Mlimfull_data[], real_T Vfull_data[], const boolean_T isMrows0[70], const
    real_T Mlimfull0[70], const real_T Vfull0[70], real_T b_p, real_T ioff);
  void GenCode_MPC_Mrows_reshape_lgb(boolean_T isMrows_data[], real_T
    Mlimfull_data[], real_T Vfull_data[], const boolean_T isMrows0[70], const
    real_T Mlimfull0[70], const real_T Vfull0[70], real_T b_p, real_T ioff);
  void GenCode_MPC_Mrows_reshape_lg(boolean_T isMrows_data[], real_T
    Mlimfull_data[], real_T Vfull_data[], const boolean_T isMrows0[70], const
    real_T Mlimfull0[70], const real_T Vfull0[70], real_T b_p, real_T ioff);
  void GenCode_MPC_repmat_d(const boolean_T a[2], real_T varargin_1, boolean_T
    b_data[], int32_T *b_size);
  void GenCode_MPC_repmat_dl(const real_T a[2], real_T varargin_1, real_T
    b_data[], int32_T *b_size);
  void GenCode_MPC_Mrows_reshape_l(boolean_T isMrows_data[], real_T
    Mlimfull_data[], real_T Vfull_data[], const boolean_T isMrows0[70], const
    real_T Mlimfull0[70], const real_T Vfull0[70], real_T b_p, real_T ioff);
  void GenCode_MPC_Mrows_reshape(boolean_T isMrows_data[], real_T Mlimfull_data[],
    real_T Vfull_data[], const boolean_T isMrows0[70], const real_T Mlimfull0[70],
    const real_T Vfull0[70], real_T b_p);
  void GenCode_MPC_mpc_constraintcoef(const real_T b_A[25], const real_T Bu[5],
    const real_T Bv[10], const real_T b_C[10], const real_T Dv[4], const real_T
    Jm_data[], const int32_T Jm_size[2], real_T SuJm_data[], int32_T SuJm_size[2],
    real_T Sx_data[], int32_T Sx_size[2], real_T Su1_data[], int32_T *Su1_size,
    real_T Hv_data[], int32_T Hv_size[2]);
  void GenCode_MPC_repmat(real_T a, real_T varargin_1, real_T b_data[], int32_T *
    b_size);
  void GenCode_MPC_kron(const int32_T *b_A_size, real_T K_data[], int32_T
                        *K_size);
  void GenCode_MP_mpcblock_optimizerPM(const emxArray_real_T_GenCode_MPC_T *rseq,
    const emxArray_real_T_GenCode_MPC_T *vseq, real_T umin, real_T umax, real_T
    switch_in, const real_T x[5], real_T old_u, const real_T Mlim0[4], const
    real_T utargetseq[11], real_T b_p, real_T moves, real_T b_uoff, const real_T
    ywt[2], real_T uwt, const real_T b_A[25], const
    emxArray_real_T_GenCode_MPC_T *Bu, const emxArray_real_T_GenCode_MPC_T *Bv,
    const real_T b_C[10], const emxArray_real_T_GenCode_MPC_T *Dv, real_T *u,
    real_T useq[21], real_T *status);

  // Continuous states update member function
  void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si );

  // Derivatives member function
  void GenCode_MPC_derivatives();
};

extern const MPCModelClass::ConstB_GenCode_MPC_T GenCode_MPC_ConstB;// constant block i/o 

// Constant parameters (default storage)
extern const MPCModelClass::ConstP_GenCode_MPC_T GenCode_MPC_ConstP;

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S8>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S9>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S10>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S11>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S12>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S13>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S14>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S15>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S16>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S17>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S18>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S19>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S20>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S21>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S22>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S23>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S24>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S25>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S26>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S27>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S28>/Vector Dimension Check' : Unused code path elimination
//  Block '<S29>/Vector Dimension Check' : Unused code path elimination
//  Block '<S30>/Vector Dimension Check' : Unused code path elimination
//  Block '<S31>/Vector Dimension Check' : Unused code path elimination
//  Block '<S32>/Vector Dimension Check' : Unused code path elimination
//  Block '<S33>/Vector Dimension Check' : Unused code path elimination
//  Block '<S34>/Vector Dimension Check' : Unused code path elimination
//  Block '<S7>/useq_scale' : Unused code path elimination
//  Block '<S7>/useq_scale1' : Unused code path elimination
//  Block '<S7>/ym_zero' : Unused code path elimination
//  Block '<S1>/External control signal constant' : Unused code path elimination
//  Block '<S7>/Data Type Conversion1' : Eliminate redundant data type conversion
//  Block '<S7>/Data Type Conversion11' : Eliminate redundant data type conversion
//  Block '<S7>/Data Type Conversion12' : Eliminate redundant data type conversion
//  Block '<S7>/Data Type Conversion13' : Eliminate redundant data type conversion
//  Block '<S7>/Data Type Conversion14' : Eliminate redundant data type conversion
//  Block '<S7>/Data Type Conversion15' : Eliminate redundant data type conversion
//  Block '<S7>/Data Type Conversion16' : Eliminate redundant data type conversion
//  Block '<S7>/Data Type Conversion17' : Eliminate redundant data type conversion
//  Block '<S7>/Data Type Conversion18' : Eliminate redundant data type conversion
//  Block '<S7>/Data Type Conversion19' : Eliminate redundant data type conversion
//  Block '<S7>/Data Type Conversion2' : Eliminate redundant data type conversion
//  Block '<S7>/Data Type Conversion20' : Eliminate redundant data type conversion
//  Block '<S7>/Data Type Conversion21' : Eliminate redundant data type conversion
//  Block '<S7>/Data Type Conversion3' : Eliminate redundant data type conversion
//  Block '<S7>/Data Type Conversion4' : Eliminate redundant data type conversion
//  Block '<S7>/Data Type Conversion5' : Eliminate redundant data type conversion
//  Block '<S7>/Data Type Conversion6' : Eliminate redundant data type conversion
//  Block '<S7>/Data Type Conversion7' : Eliminate redundant data type conversion
//  Block '<S7>/E Conversion' : Eliminate redundant data type conversion
//  Block '<S7>/F Conversion' : Eliminate redundant data type conversion
//  Block '<S7>/G Conversion' : Eliminate redundant data type conversion
//  Block '<S7>/Reshape' : Reshape block reduction
//  Block '<S7>/Reshape1' : Reshape block reduction
//  Block '<S7>/Reshape2' : Reshape block reduction
//  Block '<S7>/Reshape3' : Reshape block reduction
//  Block '<S7>/Reshape4' : Reshape block reduction
//  Block '<S7>/Reshape5' : Reshape block reduction
//  Block '<S7>/S Conversion' : Eliminate redundant data type conversion
//  Block '<S7>/mo or x Conversion' : Eliminate redundant data type conversion


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'GenCode_MPC'
//  '<S1>'   : 'GenCode_MPC/LKA subsystem'
//  '<S2>'   : 'GenCode_MPC/LKA subsystem/Adaptive MPC for LKA'
//  '<S3>'   : 'GenCode_MPC/LKA subsystem/Longitudinal velocity must be positive'
//  '<S4>'   : 'GenCode_MPC/LKA subsystem/Model for Adaptive MPC'
//  '<S5>'   : 'GenCode_MPC/LKA subsystem/Sensor Dynamics'
//  '<S6>'   : 'GenCode_MPC/LKA subsystem/Adaptive MPC for LKA/Adaptive MPC Controller'
//  '<S7>'   : 'GenCode_MPC/LKA subsystem/Adaptive MPC for LKA/Adaptive MPC Controller/MPC'
//  '<S8>'   : 'GenCode_MPC/LKA subsystem/Adaptive MPC for LKA/Adaptive MPC Controller/MPC/MPC Matrix Signal Check'
//  '<S9>'   : 'GenCode_MPC/LKA subsystem/Adaptive MPC for LKA/Adaptive MPC Controller/MPC/MPC Matrix Signal Check A'
//  '<S10>'  : 'GenCode_MPC/LKA subsystem/Adaptive MPC for LKA/Adaptive MPC Controller/MPC/MPC Matrix Signal Check B'
//  '<S11>'  : 'GenCode_MPC/LKA subsystem/Adaptive MPC for LKA/Adaptive MPC Controller/MPC/MPC Matrix Signal Check C'
//  '<S12>'  : 'GenCode_MPC/LKA subsystem/Adaptive MPC for LKA/Adaptive MPC Controller/MPC/MPC Matrix Signal Check D'
//  '<S13>'  : 'GenCode_MPC/LKA subsystem/Adaptive MPC for LKA/Adaptive MPC Controller/MPC/MPC Matrix Signal Check DX'
//  '<S14>'  : 'GenCode_MPC/LKA subsystem/Adaptive MPC for LKA/Adaptive MPC Controller/MPC/MPC Matrix Signal Check U'
//  '<S15>'  : 'GenCode_MPC/LKA subsystem/Adaptive MPC for LKA/Adaptive MPC Controller/MPC/MPC Matrix Signal Check X'
//  '<S16>'  : 'GenCode_MPC/LKA subsystem/Adaptive MPC for LKA/Adaptive MPC Controller/MPC/MPC Matrix Signal Check Y'
//  '<S17>'  : 'GenCode_MPC/LKA subsystem/Adaptive MPC for LKA/Adaptive MPC Controller/MPC/MPC Matrix Signal Check1'
//  '<S18>'  : 'GenCode_MPC/LKA subsystem/Adaptive MPC for LKA/Adaptive MPC Controller/MPC/MPC Matrix Signal Check2'
//  '<S19>'  : 'GenCode_MPC/LKA subsystem/Adaptive MPC for LKA/Adaptive MPC Controller/MPC/MPC Preview Signal Check'
//  '<S20>'  : 'GenCode_MPC/LKA subsystem/Adaptive MPC for LKA/Adaptive MPC Controller/MPC/MPC Preview Signal Check1'
//  '<S21>'  : 'GenCode_MPC/LKA subsystem/Adaptive MPC for LKA/Adaptive MPC Controller/MPC/MPC Preview Signal Check2'
//  '<S22>'  : 'GenCode_MPC/LKA subsystem/Adaptive MPC for LKA/Adaptive MPC Controller/MPC/MPC Preview Signal Check3'
//  '<S23>'  : 'GenCode_MPC/LKA subsystem/Adaptive MPC for LKA/Adaptive MPC Controller/MPC/MPC Preview Signal Check4'
//  '<S24>'  : 'GenCode_MPC/LKA subsystem/Adaptive MPC for LKA/Adaptive MPC Controller/MPC/MPC Preview Signal Check5'
//  '<S25>'  : 'GenCode_MPC/LKA subsystem/Adaptive MPC for LKA/Adaptive MPC Controller/MPC/MPC Preview Signal Check6'
//  '<S26>'  : 'GenCode_MPC/LKA subsystem/Adaptive MPC for LKA/Adaptive MPC Controller/MPC/MPC Preview Signal Check7'
//  '<S27>'  : 'GenCode_MPC/LKA subsystem/Adaptive MPC for LKA/Adaptive MPC Controller/MPC/MPC Preview Signal Check8'
//  '<S28>'  : 'GenCode_MPC/LKA subsystem/Adaptive MPC for LKA/Adaptive MPC Controller/MPC/MPC Scalar Signal Check'
//  '<S29>'  : 'GenCode_MPC/LKA subsystem/Adaptive MPC for LKA/Adaptive MPC Controller/MPC/MPC Scalar Signal Check1'
//  '<S30>'  : 'GenCode_MPC/LKA subsystem/Adaptive MPC for LKA/Adaptive MPC Controller/MPC/MPC Scalar Signal Check2'
//  '<S31>'  : 'GenCode_MPC/LKA subsystem/Adaptive MPC for LKA/Adaptive MPC Controller/MPC/MPC Vector Signal Check'
//  '<S32>'  : 'GenCode_MPC/LKA subsystem/Adaptive MPC for LKA/Adaptive MPC Controller/MPC/MPC Vector Signal Check1'
//  '<S33>'  : 'GenCode_MPC/LKA subsystem/Adaptive MPC for LKA/Adaptive MPC Controller/MPC/MPC Vector Signal Check11'
//  '<S34>'  : 'GenCode_MPC/LKA subsystem/Adaptive MPC for LKA/Adaptive MPC Controller/MPC/MPC Vector Signal Check6'
//  '<S35>'  : 'GenCode_MPC/LKA subsystem/Adaptive MPC for LKA/Adaptive MPC Controller/MPC/optimizer'
//  '<S36>'  : 'GenCode_MPC/LKA subsystem/Adaptive MPC for LKA/Adaptive MPC Controller/MPC/optimizer/VariableHorizonOptimizer'
//  '<S37>'  : 'GenCode_MPC/LKA subsystem/Model for Adaptive MPC/Adaptive Model'
//  '<S38>'  : 'GenCode_MPC/LKA subsystem/Model for Adaptive MPC/Vehicle Dynamics from Parameters'

#endif                                 // RTW_HEADER_GenCode_MPC_h_

//
// File trailer for generated code.
//
// [EOF]
//
