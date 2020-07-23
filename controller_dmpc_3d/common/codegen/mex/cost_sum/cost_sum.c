/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * cost_sum.c
 *
 * Code generation for function 'cost_sum'
 *
 */

/* Include files */
#include "cost_sum.h"
#include "cost_sum_data.h"
#include "cost_sum_emxutil.h"
#include "eml_int_forloop_overflow_check.h"
#include "mwmathutil.h"
#include "rt_nonfinite.h"
#include "separation.h"
#include "sum_sq_distances.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo emlrtRSI = { 16,    /* lineNo */
  "cost_sum",                          /* fcnName */
  "/Users/root1/Desktop/Flocking_maneuvers/flock_maneuver/3D/controller_dmpc_3d/common/cost_sum.m"/* pathName */
};

static emlrtRSInfo b_emlrtRSI = { 34,  /* lineNo */
  "cost_sum",                          /* fcnName */
  "/Users/root1/Desktop/Flocking_maneuvers/flock_maneuver/3D/controller_dmpc_3d/common/cost_sum.m"/* pathName */
};

static emlrtRSInfo c_emlrtRSI = { 9,   /* lineNo */
  "u2acc",                             /* fcnName */
  "/Users/root1/Desktop/Flocking_maneuvers/flock_maneuver/3D/controller_dmpc_3d/common/u2acc.m"/* pathName */
};

static emlrtRSInfo d_emlrtRSI = { 18,  /* lineNo */
  "reshapeSizeChecks",                 /* fcnName */
  "/Applications/MATLAB_R2020a.app/toolbox/eml/eml/+coder/+internal/reshapeSizeChecks.m"/* pathName */
};

static emlrtRSInfo e_emlrtRSI = { 3,   /* lineNo */
  "fitness",                           /* fcnName */
  "/Users/root1/Desktop/Flocking_maneuvers/flock_maneuver/3D/controller_dmpc_3d/fitness.m"/* pathName */
};

static emlrtRSInfo f_emlrtRSI = { 4,   /* lineNo */
  "fitness",                           /* fcnName */
  "/Users/root1/Desktop/Flocking_maneuvers/flock_maneuver/3D/controller_dmpc_3d/fitness.m"/* pathName */
};

static emlrtRSInfo g_emlrtRSI = { 5,   /* lineNo */
  "fitness",                           /* fcnName */
  "/Users/root1/Desktop/Flocking_maneuvers/flock_maneuver/3D/controller_dmpc_3d/fitness.m"/* pathName */
};

static emlrtRSInfo h_emlrtRSI = { 6,   /* lineNo */
  "fitness",                           /* fcnName */
  "/Users/root1/Desktop/Flocking_maneuvers/flock_maneuver/3D/controller_dmpc_3d/fitness.m"/* pathName */
};

static emlrtRSInfo i_emlrtRSI = { 8,   /* lineNo */
  "fitness",                           /* fcnName */
  "/Users/root1/Desktop/Flocking_maneuvers/flock_maneuver/3D/controller_dmpc_3d/fitness.m"/* pathName */
};

static emlrtRSInfo j_emlrtRSI = { 3,   /* lineNo */
  "vm",                                /* fcnName */
  "/Users/root1/Desktop/Flocking_maneuvers/flock_maneuver/3D/controller_dmpc_3d/cost_functions/vm.m"/* pathName */
};

static emlrtRSInfo k_emlrtRSI = { 28,  /* lineNo */
  "repmat",                            /* fcnName */
  "/Applications/MATLAB_R2020a.app/toolbox/eml/lib/matlab/elmat/repmat.m"/* pathName */
};

static emlrtRSInfo l_emlrtRSI = { 64,  /* lineNo */
  "repmat",                            /* fcnName */
  "/Applications/MATLAB_R2020a.app/toolbox/eml/lib/matlab/elmat/repmat.m"/* pathName */
};

static emlrtRSInfo m_emlrtRSI = { 21,  /* lineNo */
  "eml_int_forloop_overflow_check",    /* fcnName */
  "/Applications/MATLAB_R2020a.app/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"/* pathName */
};

static emlrtRTEInfo emlrtRTEI = { 64,  /* lineNo */
  15,                                  /* colNo */
  "assertValidSizeArg",                /* fName */
  "/Applications/MATLAB_R2020a.app/toolbox/eml/eml/+coder/+internal/assertValidSizeArg.m"/* pName */
};

static emlrtRTEInfo b_emlrtRTEI = { 58,/* lineNo */
  23,                                  /* colNo */
  "assertValidSizeArg",                /* fName */
  "/Applications/MATLAB_R2020a.app/toolbox/eml/eml/+coder/+internal/assertValidSizeArg.m"/* pName */
};

static emlrtRTEInfo c_emlrtRTEI = { 13,/* lineNo */
  9,                                   /* colNo */
  "sqrt",                              /* fName */
  "/Applications/MATLAB_R2020a.app/toolbox/eml/lib/matlab/elfun/sqrt.m"/* pName */
};

static emlrtDCInfo emlrtDCI = { 31,    /* lineNo */
  14,                                  /* colNo */
  "repmat",                            /* fName */
  "/Applications/MATLAB_R2020a.app/toolbox/eml/lib/matlab/elmat/repmat.m",/* pName */
  4                                    /* checkKind */
};

static emlrtECInfo emlrtECI = { 2,     /* nDims */
  6,                                   /* lineNo */
  11,                                  /* colNo */
  "vm",                                /* fName */
  "/Users/root1/Desktop/Flocking_maneuvers/flock_maneuver/3D/controller_dmpc_3d/cost_functions/vm.m"/* pName */
};

static emlrtBCInfo emlrtBCI = { 1,     /* iFirst */
  20,                                  /* iLast */
  6,                                   /* lineNo */
  24,                                  /* colNo */
  "params.w_m",                        /* aName */
  "vm",                                /* fName */
  "/Users/root1/Desktop/Flocking_maneuvers/flock_maneuver/3D/controller_dmpc_3d/cost_functions/vm.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo b_emlrtDCI = { 6,   /* lineNo */
  24,                                  /* colNo */
  "vm",                                /* fName */
  "/Users/root1/Desktop/Flocking_maneuvers/flock_maneuver/3D/controller_dmpc_3d/cost_functions/vm.m",/* pName */
  1                                    /* checkKind */
};

static emlrtECInfo b_emlrtECI = { 2,   /* nDims */
  3,                                   /* lineNo */
  23,                                  /* colNo */
  "vm",                                /* fName */
  "/Users/root1/Desktop/Flocking_maneuvers/flock_maneuver/3D/controller_dmpc_3d/cost_functions/vm.m"/* pName */
};

static emlrtRTEInfo d_emlrtRTEI = { 13,/* lineNo */
  13,                                  /* colNo */
  "toLogicalCheck",                    /* fName */
  "/Applications/MATLAB_R2020a.app/toolbox/eml/eml/+coder/+internal/toLogicalCheck.m"/* pName */
};

static emlrtRTEInfo e_emlrtRTEI = { 49,/* lineNo */
  19,                                  /* colNo */
  "assertValidSizeArg",                /* fName */
  "/Applications/MATLAB_R2020a.app/toolbox/eml/eml/+coder/+internal/assertValidSizeArg.m"/* pName */
};

static emlrtRTEInfo f_emlrtRTEI = { 52,/* lineNo */
  13,                                  /* colNo */
  "reshapeSizeChecks",                 /* fName */
  "/Applications/MATLAB_R2020a.app/toolbox/eml/eml/+coder/+internal/reshapeSizeChecks.m"/* pName */
};

static emlrtRTEInfo g_emlrtRTEI = { 57,/* lineNo */
  23,                                  /* colNo */
  "reshapeSizeChecks",                 /* fName */
  "/Applications/MATLAB_R2020a.app/toolbox/eml/eml/+coder/+internal/reshapeSizeChecks.m"/* pName */
};

static emlrtRTEInfo h_emlrtRTEI = { 59,/* lineNo */
  23,                                  /* colNo */
  "reshapeSizeChecks",                 /* fName */
  "/Applications/MATLAB_R2020a.app/toolbox/eml/eml/+coder/+internal/reshapeSizeChecks.m"/* pName */
};

static emlrtBCInfo b_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  32,                                  /* lineNo */
  41,                                  /* colNo */
  "a",                                 /* aName */
  "cost_sum",                          /* fName */
  "/Users/root1/Desktop/Flocking_maneuvers/flock_maneuver/3D/controller_dmpc_3d/common/cost_sum.m",/* pName */
  0                                    /* checkKind */
};

static emlrtECInfo c_emlrtECI = { -1,  /* nDims */
  18,                                  /* lineNo */
  1,                                   /* colNo */
  "cost_sum",                          /* fName */
  "/Users/root1/Desktop/Flocking_maneuvers/flock_maneuver/3D/controller_dmpc_3d/common/cost_sum.m"/* pName */
};

static emlrtRTEInfo i_emlrtRTEI = { 31,/* lineNo */
  17,                                  /* colNo */
  "cost_sum",                          /* fName */
  "/Users/root1/Desktop/Flocking_maneuvers/flock_maneuver/3D/controller_dmpc_3d/common/cost_sum.m"/* pName */
};

static emlrtRTEInfo j_emlrtRTEI = { 30,/* lineNo */
  13,                                  /* colNo */
  "cost_sum",                          /* fName */
  "/Users/root1/Desktop/Flocking_maneuvers/flock_maneuver/3D/controller_dmpc_3d/common/cost_sum.m"/* pName */
};

static emlrtDCInfo c_emlrtDCI = { 17,  /* lineNo */
  26,                                  /* colNo */
  "cost_sum",                          /* fName */
  "/Users/root1/Desktop/Flocking_maneuvers/flock_maneuver/3D/controller_dmpc_3d/common/cost_sum.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo d_emlrtDCI = { 17,  /* lineNo */
  26,                                  /* colNo */
  "cost_sum",                          /* fName */
  "/Users/root1/Desktop/Flocking_maneuvers/flock_maneuver/3D/controller_dmpc_3d/common/cost_sum.m",/* pName */
  4                                    /* checkKind */
};

static emlrtDCInfo e_emlrtDCI = { 17,  /* lineNo */
  1,                                   /* colNo */
  "cost_sum",                          /* fName */
  "/Users/root1/Desktop/Flocking_maneuvers/flock_maneuver/3D/controller_dmpc_3d/common/cost_sum.m",/* pName */
  1                                    /* checkKind */
};

static emlrtRTEInfo l_emlrtRTEI = { 17,/* lineNo */
  1,                                   /* colNo */
  "cost_sum",                          /* fName */
  "/Users/root1/Desktop/Flocking_maneuvers/flock_maneuver/3D/controller_dmpc_3d/common/cost_sum.m"/* pName */
};

static emlrtRTEInfo m_emlrtRTEI = { 3, /* lineNo */
  23,                                  /* colNo */
  "vm",                                /* fName */
  "/Users/root1/Desktop/Flocking_maneuvers/flock_maneuver/3D/controller_dmpc_3d/cost_functions/vm.m"/* pName */
};

static emlrtRTEInfo n_emlrtRTEI = { 1, /* lineNo */
  19,                                  /* colNo */
  "cost_sum",                          /* fName */
  "/Users/root1/Desktop/Flocking_maneuvers/flock_maneuver/3D/controller_dmpc_3d/common/cost_sum.m"/* pName */
};

/* Function Definitions */
real_T cost_sum(const emlrtStack *sp, const real_T u[18], real_T s[132], const
                struct0_T *params)
{
  real_T cost;
  real_T varargin_1[2];
  int32_T k;
  boolean_T guard1 = false;
  int32_T exitg2;
  boolean_T exitg1;
  real_T b_params;
  int32_T num_idx_1_tmp;
  emxArray_real_T *a;
  int32_T i;
  real_T d;
  int32_T loop_ub;
  int32_T iv[3];
  int32_T iv1[2];
  real_T control_step;
  int32_T i1;
  real_T scale;
  real_T z1[18];
  emxArray_real_T *r;
  int32_T h;
  int32_T b_i;
  int32_T ibtile;
  real_T v_idx_2;
  real_T v_idx_0;
  real_T absxk;
  real_T t;
  real_T d1;
  int32_T jtilecol;
  real_T v_idx_1;
  real_T b_a[30];
  real_T b_z1[30];
  boolean_T p;
  real_T rel_speed[10];
  int32_T iv2[2];
  real_T params_data[20];
  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  f_st.prev = &st;
  f_st.tls = st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);

  /*  cost_sum - 3D DMPC */
  /*  Non-liear constraints passed to fmincon as function handle.  */
  /*  like this:  */
  /*      x = fmincon(@myfun,x0,A,b,Aeq,beq,lb,ub,@mycon) */
  /*  Input: */
  /*    - s      % 1x12xn - state of the flock  */
  /*    - u      % 3.h x 1 - The sequence of control action over the horizon  */
  /*    - params % parameters */
  /*  Output: */
  /*    - cost      % combined cost */
  /*  Usama Mehmood - Oct 2019 */
  /*  Add zero acc of neighbors */
  st.site = &emlrtRSI;

  /*  u2acc - 3D DMPC - convert col vector representation of acc to matrix form.  */
  /*  Input: */
  /*     u    - 3.reh x 1 vector */
  /*  Output */
  /*     acc    - 3xnxh Matric. */
  /*  Usama Mehmood - Oct 2019 */
  b_st.site = &c_emlrtRSI;
  varargin_1[0] = 3.0;
  varargin_1[1] = params->h;
  c_st.site = &d_emlrtRSI;
  k = 0;
  guard1 = false;
  do {
    exitg2 = 0;
    if (k < 2) {
      if ((varargin_1[k] != muDoubleScalarFloor(varargin_1[k])) ||
          muDoubleScalarIsInf(varargin_1[k])) {
        guard1 = true;
        exitg2 = 1;
      } else {
        k++;
        guard1 = false;
      }
    } else {
      k = 0;
      exitg2 = 2;
    }
  } while (exitg2 == 0);

  if (exitg2 != 1) {
    exitg1 = false;
    while ((!exitg1) && (k < 2)) {
      if ((varargin_1[k] < -2.147483648E+9) || (varargin_1[k] > 2.147483647E+9))
      {
        guard1 = true;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (guard1) {
    emlrtErrorWithMessageIdR2018a(&c_st, &e_emlrtRTEI,
      "Coder:toolbox:eml_assert_valid_size_arg_invalidSizeVector",
      "Coder:toolbox:eml_assert_valid_size_arg_invalidSizeVector", 4, 12,
      MIN_int32_T, 12, MAX_int32_T);
  }

  if (params->h <= 0.0) {
    b_params = 0.0;
  } else {
    b_params = 3.0 * params->h;
  }

  if (!(b_params <= 2.147483647E+9)) {
    emlrtErrorWithMessageIdR2018a(&c_st, &emlrtRTEI, "Coder:MATLAB:pmaxsize",
      "Coder:MATLAB:pmaxsize", 0);
  }

  num_idx_1_tmp = (int32_T)params->h;
  if (num_idx_1_tmp > 18) {
    emlrtErrorWithMessageIdR2018a(&b_st, &f_emlrtRTEI,
      "Coder:toolbox:reshape_emptyReshapeLimit",
      "Coder:toolbox:reshape_emptyReshapeLimit", 0);
  }

  if (num_idx_1_tmp < 0) {
    emlrtErrorWithMessageIdR2018a(&b_st, &g_emlrtRTEI,
      "MATLAB:checkDimCommon:nonnegativeSize",
      "MATLAB:checkDimCommon:nonnegativeSize", 0);
  }

  if (3 * num_idx_1_tmp != 18) {
    emlrtErrorWithMessageIdR2018a(&b_st, &h_emlrtRTEI,
      "Coder:MATLAB:getReshapeDims_notSameNumel",
      "Coder:MATLAB:getReshapeDims_notSameNumel", 0);
  }

  emxInit_real_T(&b_st, &a, 3, &l_emlrtRTEI, true);
  i = a->size[0] * a->size[1] * a->size[2];
  a->size[0] = 3;
  a->size[1] = 11;
  emxEnsureCapacity_real_T(sp, a, i, &l_emlrtRTEI);
  if (!(params->h >= 0.0)) {
    emlrtNonNegativeCheckR2012b(params->h, &d_emlrtDCI, sp);
  }

  d = (int32_T)muDoubleScalarFloor(params->h);
  if (params->h != d) {
    emlrtIntegerCheckR2012b(params->h, &c_emlrtDCI, sp);
  }

  i = a->size[0] * a->size[1] * a->size[2];
  a->size[2] = num_idx_1_tmp;
  emxEnsureCapacity_real_T(sp, a, i, &l_emlrtRTEI);
  if (params->h != d) {
    emlrtIntegerCheckR2012b(params->h, &e_emlrtDCI, sp);
  }

  loop_ub = 33 * num_idx_1_tmp;
  for (i = 0; i < loop_ub; i++) {
    a->data[i] = 0.0;
  }

  iv[0] = 3;
  iv[1] = 1;
  iv[2] = num_idx_1_tmp;
  iv1[0] = 3;
  iv1[1] = num_idx_1_tmp;
  emlrtSubAssignSizeCheckR2012b(&iv[0], 3, &iv1[0], 2, &c_emlrtECI, sp);
  for (i = 0; i < num_idx_1_tmp; i++) {
    a->data[33 * i] = u[3 * i];
    a->data[33 * i + 1] = u[3 * i + 1];
    a->data[33 * i + 2] = u[3 * i + 2];
  }

  /*  */
  control_step = params->ct / params->dt;
  cost = 0.0;
  if (params->cmpc_prediction_model == 1.0) {
    /* point-model */
    emlrtForLoopVectorCheckR2012b(1.0, 1.0, params->h, mxDOUBLE_CLASS, (int32_T)
      params->h, &j_emlrtRTEI, sp);
    if (0 <= num_idx_1_tmp - 1) {
      i1 = (int32_T)control_step;
    }

    emxInit_real_T(sp, &r, 2, &n_emlrtRTEI, true);
    for (h = 0; h < num_idx_1_tmp; h++) {
      emlrtForLoopVectorCheckR2012b(1.0, 1.0, control_step, mxDOUBLE_CLASS,
        (int32_T)control_step, &i_emlrtRTEI, sp);
      for (k = 0; k < i1; k++) {
        i = h + 1;
        if (i > a->size[2]) {
          emlrtDynamicBoundsCheckR2012b(i, 1, a->size[2], &b_emlrtBCI, sp);
        }

        /*  dynamics_point  */
        /*  Input: */
        /*    - s      % 1x12xn - state of the flock  */
        /*    - a      % 3xn - The sequence of control action over the horizon  */
        /*    - params % parameters */
        /*  Output: */
        /*    - s_new  % next state */
        /*  Usama Mehmood - Oct 2019 */
        d = params->dt;
        b_params = params->vmax;
        for (b_i = 0; b_i < 11; b_i++) {
          /* update vel */
          /*  Usama Mehmood - Oct 2019 */
          scale = 3.3121686421112381E-170;
          i = 12 * b_i + 3;
          ibtile = 3 * b_i + 33 * h;
          v_idx_2 = s[i] + d * a->data[ibtile];
          v_idx_0 = v_idx_2;
          absxk = muDoubleScalarAbs(v_idx_2);
          if (absxk > 3.3121686421112381E-170) {
            d1 = 1.0;
            scale = absxk;
          } else {
            t = absxk / 3.3121686421112381E-170;
            d1 = t * t;
          }

          jtilecol = 12 * b_i + 4;
          v_idx_2 = s[jtilecol] + d * a->data[ibtile + 1];
          v_idx_1 = v_idx_2;
          absxk = muDoubleScalarAbs(v_idx_2);
          if (absxk > scale) {
            t = scale / absxk;
            d1 = d1 * t * t + 1.0;
            scale = absxk;
          } else {
            t = absxk / scale;
            d1 += t * t;
          }

          loop_ub = 12 * b_i + 5;
          v_idx_2 = s[loop_ub] + d * a->data[ibtile + 2];
          absxk = muDoubleScalarAbs(v_idx_2);
          if (absxk > scale) {
            t = scale / absxk;
            d1 = d1 * t * t + 1.0;
            scale = absxk;
          } else {
            t = absxk / scale;
            d1 += t * t;
          }

          d1 = scale * muDoubleScalarSqrt(d1);
          if (d1 > b_params) {
            scale = b_params / d1;
            v_idx_0 *= scale;
            v_idx_1 *= scale;
            v_idx_2 *= scale;
          }

          /* update pos */
          s[i] = v_idx_0;
          s[12 * b_i] += d * v_idx_0;
          s[jtilecol] = v_idx_1;
          i = 12 * b_i + 1;
          s[i] += d * v_idx_1;
          s[loop_ub] = v_idx_2;
          i = 12 * b_i + 2;
          s[i] += d * v_idx_2;
          if (*emlrtBreakCheckR2012bFlagVar != 0) {
            emlrtBreakCheckR2012b(sp);
          }
        }

        if (*emlrtBreakCheckR2012bFlagVar != 0) {
          emlrtBreakCheckR2012b(sp);
        }
      }

      st.site = &b_emlrtRSI;

      /*  Usama Mehmood - Oct 2019 */
      /*  fitness 3D DMPC Usama Mehmood - Oct 2019 */
      b_st.site = &e_emlrtRSI;
      if (muDoubleScalarIsNaN(params->turn)) {
        emlrtErrorWithMessageIdR2018a(&b_st, &d_emlrtRTEI, "MATLAB:nologicalnan",
          "MATLAB:nologicalnan", 0);
      }

      if (params->turn != 0.0) {
        b_st.site = &h_emlrtRSI;
        c_st.site = &j_emlrtRSI;
        d_st.site = &k_emlrtRSI;
        d = muDoubleScalarFloor(params->knn);
        if ((params->knn != d) || muDoubleScalarIsInf(params->knn) ||
            (params->knn < -2.147483648E+9) || (params->knn > 2.147483647E+9)) {
          emlrtErrorWithMessageIdR2018a(&d_st, &b_emlrtRTEI,
            "Coder:MATLAB:NonIntegerInput", "Coder:MATLAB:NonIntegerInput", 4,
            12, MIN_int32_T, 12, MAX_int32_T);
        }

        if (params->knn <= 0.0) {
          b_params = 0.0;
        } else {
          b_params = params->knn;
        }

        if (!(b_params <= 2.147483647E+9)) {
          emlrtErrorWithMessageIdR2018a(&d_st, &emlrtRTEI,
            "Coder:MATLAB:pmaxsize", "Coder:MATLAB:pmaxsize", 0);
        }

        if (!(params->knn >= 0.0)) {
          emlrtNonNegativeCheckR2012b(params->knn, &emlrtDCI, &c_st);
        }

        i = r->size[0] * r->size[1];
        r->size[0] = 3;
        loop_ub = (int32_T)params->knn;
        r->size[1] = loop_ub;
        emxEnsureCapacity_real_T(&c_st, r, i, &m_emlrtRTEI);
        d_st.site = &l_emlrtRSI;
        if ((1 <= loop_ub) && (loop_ub > 2147483646)) {
          e_st.site = &m_emlrtRSI;
          check_forloop_overflow_error(&e_st);
        }

        for (jtilecol = 0; jtilecol < loop_ub; jtilecol++) {
          ibtile = jtilecol * 3;
          r->data[ibtile] = s[3];
          r->data[ibtile + 1] = s[4];
          r->data[ibtile + 2] = s[5];
        }

        iv1[0] = 3;
        iv1[1] = 10;
        emlrtSizeEqCheckNDR2012b(*(int32_T (*)[2])r->size, iv1, &b_emlrtECI,
          &b_st);
        for (i = 0; i < 10; i++) {
          ibtile = 12 * (i + 1);
          b_a[3 * i] = r->data[3 * i] - s[ibtile + 3];
          jtilecol = 3 * i + 1;
          b_a[jtilecol] = r->data[jtilecol] - s[ibtile + 4];
          jtilecol = 3 * i + 2;
          b_a[jtilecol] = r->data[jtilecol] - s[ibtile + 5];
        }

        for (k = 0; k < 30; k++) {
          b_z1[k] = b_a[k] * b_a[k];
        }

        for (b_i = 0; b_i < 10; b_i++) {
          ibtile = b_i * 3;
          rel_speed[b_i] = (b_z1[ibtile] + b_z1[ibtile + 1]) + b_z1[ibtile + 2];
        }

        c_st.site = &j_emlrtRSI;
        p = false;
        for (k = 0; k < 10; k++) {
          if (p || (rel_speed[k] < 0.0)) {
            p = true;
          }
        }

        if (p) {
          emlrtErrorWithMessageIdR2018a(&c_st, &c_emlrtRTEI,
            "Coder:toolbox:ElFunDomainError", "Coder:toolbox:ElFunDomainError",
            3, 4, 4, "sqrt");
        }

        for (k = 0; k < 10; k++) {
          rel_speed[k] = muDoubleScalarSqrt(rel_speed[k]);
        }

        /*  AWNing */
        if (1.0 > params->knn) {
          loop_ub = 0;
        } else {
          if (params->knn != (int32_T)d) {
            emlrtIntegerCheckR2012b(params->knn, &b_emlrtDCI, &b_st);
          }

          if ((loop_ub < 1) || (loop_ub > 20)) {
            emlrtDynamicBoundsCheckR2012b(loop_ub, 1, 20, &emlrtBCI, &b_st);
          }
        }

        iv1[0] = 1;
        iv1[1] = loop_ub;
        iv2[0] = 1;
        iv2[1] = 10;
        if (loop_ub != 10) {
          emlrtSizeEqCheckNDR2012b(&iv1[0], &iv2[0], &emlrtECI, &b_st);
        }

        memcpy(&params_data[0], &params->w_m[0], 10U * sizeof(real_T));
        for (i = 0; i < 10; i++) {
          rel_speed[i] *= params_data[i];
        }

        scale = rel_speed[0];
        for (k = 0; k < 9; k++) {
          scale += rel_speed[k + 1];
        }

        b_st.site = &f_emlrtRSI;
        f_st.site = &g_emlrtRSI;
        scale += params->wc * sum_sq_distances(&b_st, s) + params->ws *
          separation(&f_st, s);
      } else {
        b_st.site = &i_emlrtRSI;
        scale = params->wc * sum_sq_distances(&b_st, s) + params->ws *
          separation(&b_st, s);
      }

      cost += scale;
      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b(sp);
      }
    }

    emxFree_real_T(&r);

    /*  elseif params.cmpc_prediction_model == 2 %quadcopter model */
    /*      u_dash_prev = zeros(4,num_agents); */
    /*      for h = 1:params.h */
    /*          for i = 1:num_agents */
    /*              for k = 1:control_step */
    /*                  u_dash = acc2thrust(a(:,i,h), params, u_dash_prev(2:4,i) ); */
    /*                  u_dash_prev(:,i) = u_dash; */
    /*                  [u_quad, e(i)] = controller_inner_pid(s(1,:,i), u_dash, e(i)); */
    /*                  [s(1,:,i), ~] = dynamics_quadcopter(s(1,:,i), u_quad, params);  */
    /*              end */
    /*          end */
    /*          cost = cost + fitness(s, params); */
    /*      end */
  }

  emxFree_real_T(&a);
  for (k = 0; k < 18; k++) {
    z1[k] = u[k] * u[k];
  }

  scale = z1[0];
  for (k = 0; k < 17; k++) {
    scale += z1[k + 1];
  }

  cost = cost / params->h + scale / params->h;
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
  return cost;
}

/* End of code generation (cost_sum.c) */
