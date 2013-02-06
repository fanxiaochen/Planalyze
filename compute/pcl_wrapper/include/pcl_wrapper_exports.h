#pragma once
#ifndef PCL_WRAPPER_EXPORTS_H_
#define PCL_WRAPPER_EXPORTS_H_

// This header is created to include to NVCC compiled sources.
// Header 'pcl_macros' is not suitable since it inludes <Eigen/Core>,
// which can't be eaten by nvcc (it's too weak)

#if defined WIN32 || defined _WIN32 || defined WINCE || defined __MINGW32__
    #ifdef PCL_WRAPPER_API_EXPORTS
        #define PCL_WRAPPER_EXPORTS __declspec(dllexport)
    #else
        #define PCL_WRAPPER_EXPORTS
    #endif
#else
    #define PCL_EXPORTS
#endif

#endif  //#ifndef PCL_WRAPPER_EXPORTS_H_
