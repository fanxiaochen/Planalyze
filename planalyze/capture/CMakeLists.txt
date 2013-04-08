set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(Boost_USE_STATIC_LIBS  ON)
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF)

find_package(3rdParty)
find_package(Qextserialport REQUIRED)
include_directories(${QEXTSERIALPORT_INCLUDE_DIR})

find_package(OpenSceneGraph REQUIRED osgViewer osgText osgDB osgGA osgQt osgManipulator osgUtil)
if(OPENSCENEGRAPH_FOUND)
    include_directories(${OPENSCENEGRAPH_INCLUDE_DIRS})
endif(OPENSCENEGRAPH_FOUND)

find_package(OpenGL REQUIRED)

find_package(Qt4 REQUIRED QtCore QtGui QtXml QtOpenGL)
include(${QT_USE_FILE})

find_package(Boost REQUIRED thread date_time system chrono)
if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
endif(Boost_FOUND)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)


# Set Qt files and resources here  
set(uis       resources/main_window.ui)
set(moc_incs  include/main_window.h
              include/parameter_dialog.h
              include/pattern_projector.h
              include/plain_text_viewer.h
              include/image_grabber.h
              include/turn_table.h
              )

set(resources	resources/main_window.qrc)

# Qt stuff
QT4_WRAP_UI(ui_srcs ${uis})
QT4_WRAP_CPP(moc_srcs ${moc_incs} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)
QT4_ADD_RESOURCES(resource_srcs ${resources})

set(incs    ${moc_incs}
            include/image_viewer.h
            include/parameter.h
            )

set(srcs  src/main.cpp
          src/main_window.cpp
          src/parameter.cpp
          src/parameter_dialog.cpp
          src/turn_table.cpp
          src/pattern_projector.cpp
          src/image_grabber.cpp
          src/image_viewer.cpp
          src/plain_text_viewer.cpp
          )

set(impl_incs include/impl/parameter.hpp)

# Organize files
SOURCE_GROUP("Resources" FILES ${uis} ${resources})
SOURCE_GROUP("Generated" FILES ${ui_srcs} ${moc_srcs} ${resource_srcs})
SET_SOURCE_FILES_PROPERTIES(${srcs} PROPERTIES OBJECT_DEPENDS "${ui_srcs}")

# Put the ui in the windows project file
IF (${CMAKE_BUILD_TOOL} MATCHES "msdev")
  SET (srcs ${srcs} ${uis})
ENDIF (${CMAKE_BUILD_TOOL} MATCHES "msdev")
IF (${CMAKE_BUILD_TOOL} MATCHES "devenv")
  SET (srcs ${srcs} ${uis})
ENDIF (${CMAKE_BUILD_TOOL} MATCHES "devenv")

set(exe_name capture)
add_executable(${exe_name} ${srcs} ${ui_srcs} ${moc_srcs} ${resource_srcs} ${srcs} ${incs} ${impl_incs})
target_link_libraries(${exe_name} ${ThirdParty_LIBS} ${OPENGL_LIBRARIES} ${QT_LIBRARIES} ${Boost_LIBRARIES} ${QEXTSERIALPORT_LIBRARY})

if(WIN32 AND MSVC)
  set_target_properties(${exe_name} PROPERTIES LINK_FLAGS_RELEASE /OPT:REF)
elseif(CMAKE_SYSTEMname STREQUAL "Darwin")
  if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set_target_properties(${exe_name} PROPERTIES LINK_FLAGS -Wl)
  endif()
elseif(__COMPILER_PATHSCALE)
  set_target_properties(${exe_name} PROPERTIES LINK_FLAGS -mp)
else()
  set_target_properties(${exe_name} PROPERTIES LINK_FLAGS -Wl,--as-needed)
endif()

set_target_properties(${exe_name} PROPERTIES DEBUG_POSTFIX _debug)
set_target_properties(${exe_name} PROPERTIES RELEASE_POSTFIX _release)