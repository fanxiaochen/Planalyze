set(ThirdParty_DIR ${PROJECT_SOURCE_DIR}/3rd-party)

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