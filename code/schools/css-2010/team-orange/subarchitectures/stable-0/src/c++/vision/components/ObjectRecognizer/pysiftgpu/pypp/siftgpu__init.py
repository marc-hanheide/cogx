import __PySiftGpu
import numpy
import ctypes

# The SiftGPU object must be a singleton.
__SiftGpuSingleton = None
def SiftGPU(np = 1, create_context = True, params=""):
    global __SiftGpuSingleton
    if __SiftGpuSingleton == None:
        __SiftGpuSingleton = __PySiftGpu.SiftGPU(np)
        if create_context:
            not_working = """
            if params != None: params = params.split(); npar = len(params)
            else: npar = 0
            if npar > 0:
                CSTR_ARRAY = ctypes.c_char_p * npar
                argv = CSTR_ARRAY()
                i = 0
                for p in params: argv[i] = p; i += 1
                __SiftGpuSingleton.ParseParam(len(params), argv)
            """
            if params != None: __SiftGpuSingleton.ParseParam(params)
            __SiftGpuSingleton.CreateContextGL()
        else: __SiftGpuSingleton.VerifyContextGL()
    return __SiftGpuSingleton


# The SiftMatchGPU object must be a singleton.
__SiftGpuMatchSingleton = None
def SiftMatchGPU(max_sift=4096, create_context = False):
    global __SiftGpuMatchSingleton
    if __SiftGpuMatchSingleton == None:
        __SiftGpuMatchSingleton = __PySiftGpu.SiftMatchGPU(max_sift)
        if create_context: __SiftGpuMatchSingleton.CreateContextGL()
        else: __SiftGpuMatchSingleton.VerifyContextGL()
    return __SiftGpuMatchSingleton

