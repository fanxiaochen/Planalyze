set(ThirdParty_DIR ${PROJECT_SOURCE_DIR}/3rd-party)

#qextserialport
set(QExtSerialPort_INCLUDE_DIR ${ThirdParty_DIR}/qextserialport-1.2beta2/src)
set(QExtSerialPort_LIBRARY_DIRS ${ThirdParty_DIR}/qextserialport-1.2beta2/lib)
set(QExtSerialPort_LIBRARY optimized qextserialport-1.2.lib debug qextserialport-1.2d.lib )
include_directories(${QExtSerialPort_INCLUDE_DIR})
link_directories(${QExtSerialPort_LIBRARY_DIRS})
set(ThirdParty_LIBS ${ThirdParty_LIBS} ${QExtSerialPort_LIBRARY})

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