//
// MATLAB Compiler: 4.7 (R2007b)
// Date: Mon Apr 21 14:27:37 2008
// Arguments: "-B" "macro_default" "-W" "cpplib:libFeatureLearningCtf" "-d"
// "/home/mmarko/localsvn/cosy/code/build/subarchitectures/vision/src/matlab/prj
// deploy/build" "-T" "link:lib" "-v"
// "/home/mmarko/localsvn/cosy/code/subarchitectures/vision/src/matlab/extAPfeat
// ures.m"
// "/home/mmarko/localsvn/cosy/code/subarchitectures/vision/src/matlab/CLF/VMsta
// rt.m"
// "/home/mmarko/localsvn/cosy/code/subarchitectures/vision/src/matlab/CLF/LRsta
// rt.m"
// "/home/mmarko/localsvn/cosy/code/subarchitectures/vision/src/matlab/KDBFrec.m
// "
// "/home/mmarko/localsvn/cosy/code/subarchitectures/vision/src/matlab/qnt2ql.m"
// "/home/mmarko/localsvn/cosy/code/subarchitectures/vision/src/matlab/lf2sfa.m"
// "/home/mmarko/localsvn/cosy/code/subarchitectures/vision/src/matlab/CLF/LRloa
// dAVmodels.m"
// "/home/mmarko/localsvn/cosy/code/subarchitectures/vision/src/matlab/CLF/LRvis
// Update.m"
// "/home/mmarko/localsvn/cosy/code/subarchitectures/vision/src/matlab/KDBFupdat
// e.m"
// "/home/mmarko/localsvn/cosy/code/subarchitectures/vision/src/matlab/KDBFunlea
// rn.m"
// "/home/mmarko/localsvn/cosy/code/subarchitectures/vision/src/matlab/cpphelper
// s/cosyRecogniser_recognise.m"
// "/home/mmarko/localsvn/cosy/code/subarchitectures/vision/src/matlab/cpphelper
// s/cosyRecogniser_update.m"
// "/home/mmarko/localsvn/cosy/code/subarchitectures/vision/src/matlab/cpphelper
// s/cosyRecogniser_unlearn.m"
// "/home/mmarko/localsvn/cosy/code/subarchitectures/vision/src/matlab/cpphelper
// s/cosyFeatureExtractor_limitvalue.m" 
//

#ifndef __libFeatureLearningCtf_h
#define __libFeatureLearningCtf_h 1


#ifdef MX_COMPAT_32_OFF
#ifdef MX_COMPAT_32
#undef MX_COMPAT_32
#endif
#else
#ifndef MX_COMPAT_32
#define MX_COMPAT_32
#endif
#endif
#if defined(__cplusplus) && !defined(mclmcr_h) && defined(__linux__)
#  pragma implementation "mclmcr.h"
#endif
#include "mclmcr.h"
#include "mclcppclass.h"
#ifdef __cplusplus
extern "C" {
#endif

#if defined(__SUNPRO_CC)
/* Solaris shared libraries use __global, rather than mapfiles
 * to define the API exported from a shared library. __global is
 * only necessary when building the library -- files including
 * this header file to use the library do not need the __global
 * declaration; hence the EXPORTING_<library> logic.
 */

#ifdef EXPORTING_libFeatureLearningCtf
#define PUBLIC_libFeatureLearningCtf_C_API __global
#else
#define PUBLIC_libFeatureLearningCtf_C_API /* No import statement needed. */
#endif

#define LIB_libFeatureLearningCtf_C_API PUBLIC_libFeatureLearningCtf_C_API

#elif defined(_HPUX_SOURCE)

#ifdef EXPORTING_libFeatureLearningCtf
#define PUBLIC_libFeatureLearningCtf_C_API __declspec(dllexport)
#else
#define PUBLIC_libFeatureLearningCtf_C_API __declspec(dllimport)
#endif

#define LIB_libFeatureLearningCtf_C_API PUBLIC_libFeatureLearningCtf_C_API


#else

#define LIB_libFeatureLearningCtf_C_API

#endif

/* This symbol is defined in shared libraries. Define it here
 * (to nothing) in case this isn't a shared library. 
 */
#ifndef LIB_libFeatureLearningCtf_C_API 
#define LIB_libFeatureLearningCtf_C_API /* No special import/export declaration */
#endif

extern LIB_libFeatureLearningCtf_C_API 
bool MW_CALL_CONV libFeatureLearningCtfInitializeWithHandlers(mclOutputHandlerFcn error_handler,
                                                              mclOutputHandlerFcn print_handler);

extern LIB_libFeatureLearningCtf_C_API 
bool MW_CALL_CONV libFeatureLearningCtfInitialize(void);

extern LIB_libFeatureLearningCtf_C_API 
void MW_CALL_CONV libFeatureLearningCtfTerminate(void);



extern LIB_libFeatureLearningCtf_C_API 
void MW_CALL_CONV libFeatureLearningCtfPrintStackTrace(void);


extern LIB_libFeatureLearningCtf_C_API 
bool MW_CALL_CONV mlxExtAPfeatures(int nlhs, mxArray *plhs[],
                                   int nrhs, mxArray *prhs[]);

extern LIB_libFeatureLearningCtf_C_API 
bool MW_CALL_CONV mlxVMstart(int nlhs, mxArray *plhs[],
                             int nrhs, mxArray *prhs[]);

extern LIB_libFeatureLearningCtf_C_API 
bool MW_CALL_CONV mlxLRstart(int nlhs, mxArray *plhs[],
                             int nrhs, mxArray *prhs[]);

extern LIB_libFeatureLearningCtf_C_API 
bool MW_CALL_CONV mlxKDBFrec(int nlhs, mxArray *plhs[],
                             int nrhs, mxArray *prhs[]);

extern LIB_libFeatureLearningCtf_C_API 
bool MW_CALL_CONV mlxQnt2ql(int nlhs, mxArray *plhs[],
                            int nrhs, mxArray *prhs[]);

extern LIB_libFeatureLearningCtf_C_API 
bool MW_CALL_CONV mlxLf2sfa(int nlhs, mxArray *plhs[],
                            int nrhs, mxArray *prhs[]);

extern LIB_libFeatureLearningCtf_C_API 
bool MW_CALL_CONV mlxLRloadAVmodels(int nlhs, mxArray *plhs[],
                                    int nrhs, mxArray *prhs[]);

extern LIB_libFeatureLearningCtf_C_API 
bool MW_CALL_CONV mlxLRvisUpdate(int nlhs, mxArray *plhs[],
                                 int nrhs, mxArray *prhs[]);

extern LIB_libFeatureLearningCtf_C_API 
bool MW_CALL_CONV mlxKDBFupdate(int nlhs, mxArray *plhs[],
                                int nrhs, mxArray *prhs[]);

extern LIB_libFeatureLearningCtf_C_API 
bool MW_CALL_CONV mlxKDBFunlearn(int nlhs, mxArray *plhs[],
                                 int nrhs, mxArray *prhs[]);

extern LIB_libFeatureLearningCtf_C_API 
bool MW_CALL_CONV mlxCosyRecogniser_recognise(int nlhs, mxArray *plhs[],
                                              int nrhs, mxArray *prhs[]);

extern LIB_libFeatureLearningCtf_C_API 
bool MW_CALL_CONV mlxCosyRecogniser_update(int nlhs, mxArray *plhs[],
                                           int nrhs, mxArray *prhs[]);

extern LIB_libFeatureLearningCtf_C_API 
bool MW_CALL_CONV mlxCosyRecogniser_unlearn(int nlhs, mxArray *plhs[],
                                            int nrhs, mxArray *prhs[]);

extern LIB_libFeatureLearningCtf_C_API 
bool MW_CALL_CONV mlxCosyFeatureExtractor_limitvalue(int nlhs, mxArray *plhs[],
                                                     int nrhs, mxArray *prhs[]);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/* On Windows, use __declspec to control the exported API */
#if defined(_MSC_VER) || defined(__BORLANDC__)

#ifdef EXPORTING_libFeatureLearningCtf
#define PUBLIC_libFeatureLearningCtf_CPP_API __declspec(dllexport)
#else
#define PUBLIC_libFeatureLearningCtf_CPP_API __declspec(dllimport)
#endif

#define LIB_libFeatureLearningCtf_CPP_API PUBLIC_libFeatureLearningCtf_CPP_API

#else

#if !defined(LIB_libFeatureLearningCtf_CPP_API)
#if defined(LIB_libFeatureLearningCtf_C_API)
#define LIB_libFeatureLearningCtf_CPP_API LIB_libFeatureLearningCtf_C_API
#else
#define LIB_libFeatureLearningCtf_CPP_API /* empty! */ 
#endif
#endif

#endif

extern LIB_libFeatureLearningCtf_CPP_API void MW_CALL_CONV extAPfeatures(int nargout
                                                                         , mwArray& F
                                                                         , const mwArray& X
                                                                         , const mwArray& B);

extern LIB_libFeatureLearningCtf_CPP_API void MW_CALL_CONV VMstart(int nargout
                                                                   , mwArray& vmHs);

extern LIB_libFeatureLearningCtf_CPP_API void MW_CALL_CONV LRstart(int nargout
                                                                   , mwArray& lrHs);

extern LIB_libFeatureLearningCtf_CPP_API void MW_CALL_CONV KDBFrec(int nargout
                                                                   , mwArray& rCqnt
                                                                   , const mwArray& F
                                                                   , const mwArray& mC
                                                                   , const mwArray& mFS);

extern LIB_libFeatureLearningCtf_CPP_API void MW_CALL_CONV qnt2ql(int nargout
                                                                  , mwArray& rAVql
                                                                  , const mwArray& rAVqnt
                                                                  , const mwArray& THRs);

extern LIB_libFeatureLearningCtf_CPP_API void MW_CALL_CONV lf2sfa(int nargout
                                                                  , mwArray& av
                                                                  , const mwArray& aav
                                                                  , const mwArray& answ);

extern LIB_libFeatureLearningCtf_CPP_API void MW_CALL_CONV LRloadAVmodels(const mwArray& fname);

extern LIB_libFeatureLearningCtf_CPP_API void MW_CALL_CONV LRvisUpdate();

extern LIB_libFeatureLearningCtf_CPP_API void MW_CALL_CONV KDBFupdate(int nargout
                                                                      , mwArray& mC1
                                                                      , mwArray& mCG1
                                                                      , mwArray& mFS1
                                                                      , const mwArray& F
                                                                      , const mwArray& C
                                                                      , const mwArray& mC
                                                                      , const mwArray& mCG
                                                                      , const mwArray& mFS);

extern LIB_libFeatureLearningCtf_CPP_API void MW_CALL_CONV KDBFunlearn(int nargout
                                                                       , mwArray& mC1
                                                                       , mwArray& mCG1
                                                                       , mwArray& mFS1
                                                                       , const mwArray& F
                                                                       , const mwArray& C
                                                                       , const mwArray& mC
                                                                       , const mwArray& mCG
                                                                       , const mwArray& mFS);

extern LIB_libFeatureLearningCtf_CPP_API void MW_CALL_CONV cosyRecogniser_recognise(int nargout, mwArray& ansYes, mwArray& ansPy, const mwArray& f);

extern LIB_libFeatureLearningCtf_CPP_API void MW_CALL_CONV cosyRecogniser_update(const mwArray& features, const mwArray& avw);

extern LIB_libFeatureLearningCtf_CPP_API void MW_CALL_CONV cosyRecogniser_unlearn(const mwArray& features, const mwArray& avw);

extern LIB_libFeatureLearningCtf_CPP_API void MW_CALL_CONV cosyFeatureExtractor_limitvalue(int nargout, mwArray& newb, const mwArray& b, const mwArray& maxval);

#endif

#endif
