project(EvoGeoConvert)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


find_package(3rdParty)


include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)

set(incs )

set(srcs src/main.cpp)

set(impl_incs )

# Put the ui in the windows project file  
IF (${CMAKE_BUILD_TOOL} MATCHES "msdev")
  SET (srcs ${srcs} ${uis})
ENDIF (${CMAKE_BUILD_TOOL} MATCHES "msdev")
IF (${CMAKE_BUILD_TOOL} MATCHES "devenv")
  SET (srcs ${srcs} ${uis})
ENDIF (${CMAKE_BUILD_TOOL} MATCHES "devenv")

add_executable(EvoGeoConvert ${srcs} ${incs} ${impl_incs})

target_link_libraries(EvoGeoConvert  ${ThirdParty_LIBS})