set(ThirdParty_DIR ${PROJECT_SOURCE_DIR}/3rd-party)

set(CLAPACK_ROOT $ENV{CLAPACK_ROOT})
set(CLAPACK_INCLUDE_DIR ${CLAPACK_ROOT}/include)
set(CLAPACK_LIBRARY_DIR ${CLAPACK_ROOT}/lib)
set(BLAS_LIBRARY optimized blas.lib debug blasd.lib)
set(LIBF2C_LIBRARY optimized libf2c.lib debug libf2cd.lib)
set(LAPACK_LIBRARY optimized lapack.lib debug lapackd.lib)

include_directories(${CLAPACK_INCLUDE_DIR})
link_directories(${CLAPACK_LIBRARY_DIR})
set(ThirdParty_LIBS ${ThirdParty_LIBS} ${BLAS_LIBRARY} ${LIBF2C_LIBRARY} ${LAPACK_LIBRARY})

#FlyCapture2
if (NOT FLYCAPTURE2_DIR)
  set(FLYCAPTURE2_DIR $ENV{FLYCAPTURE2_DIR})
endif()
set(FlyCapture2_INCLUDE_DIR ${FLYCAPTURE2_DIR}/include)
set(FlyCapture2_LIBRARY_DIRS ${FLYCAPTURE2_DIR}/lib64)
set(FlyCapture2_LIBRARY optimized FlyCapture2_v100.lib debug FlyCapture2d_v100.lib )
include_directories(${FlyCapture2_INCLUDE_DIR})
link_directories(${FlyCapture2_LIBRARY_DIRS})
set(ThirdParty_LIBS ${ThirdParty_LIBS} ${FlyCapture2_LIBRARY})
