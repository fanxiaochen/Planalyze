set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(Boost_USE_STATIC_LIBS  ON)
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF)

find_package(3rdParty)
find_package(Boost REQUIRED system thread)
find_package(CGAL REQUIRED)
find_package(OpenSceneGraph REQUIRED osgViewer osgText osgDB osgGA osgQt osgManipulator osgUtil)
find_package(Qt4 REQUIRED QtCore QtGui QtOpenGL QtXml)
find_package(PCL REQUIRED common io registration kdtree search)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${CGAL_LIBRARIES_DIR})

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rd-party/gco)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rd-party/alg)
include_directories(${PROJECT_SOURCE_DIR}/pcl_wrapper/include)

if(OPENSCENEGRAPH_FOUND)
    include_directories(${OPENSCENEGRAPH_INCLUDE_DIRS})
endif(OPENSCENEGRAPH_FOUND)

if(CGAL_FOUND)
    include_directories(${CGAL_INCLUDE_DIRS})
  set(CMAKE_MODULE_PATH ${CGAL_DIR}/cmake/modules)
  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CGAL_DIR})
  include(CGAL_Macros)
  find_package(GMP REQUIRED)
  if(GMP_FOUND)
    include_directories(${GMP_INCLUDE_DIR})
  endif(GMP_FOUND)
endif(CGAL_FOUND)

if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
endif(Boost_FOUND)

include(${QT_USE_FILE})

# Set Qt files and resources here
set(uis       resources/main_window.ui)
set(moc_incs  include/main_window.h
              include/point_cloud.h
              include/registrator.h
              include/parameter_dialog.h
              include/threaded_painter.h
              include/log_viewer_widget.h
              include/file_system_model.h
              include/osg_viewer_widget.h
              include/task_dispatcher.h
              include/progress_bar.h
              include/sketch_handler.h
              )
set(resources  resources/main_window.qrc)

# Qt stuff
QT4_WRAP_UI(ui_srcs ${uis})
QT4_WRAP_CPP(moc_srcs ${moc_incs} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)
QT4_ADD_RESOURCES(resource_srcs ${resources})

set(incs    ${moc_incs}
            include/organ.h
            include/forward.h
            include/cgal_types.h
            include/color_map.h
            include/main_window.h
            include/parameter.h
            include/parameter_manager.h
            include/axis_indicator.h
            include/light_source.h
            include/toggle_handler.h
            include/resize_handler.h
            include/snapshot_handler.h
            include/file_viewer_widget.h
            include/stateset_manipulator.h
            include/renderable.h
            include/update_callback.h
            include/update_visitor.h
            include/povray_visitor.h
            include/osg_utility.h
            include/cgal_utility.h
            include/adapter_widget.h
            include/math_solvers.h
            include/information.h
            include/mesh_model.h

            )

set(3rd_incs  3rd-party/gco/block.h
              3rd-party/gco/energy.h
              3rd-party/gco/GCoptimization.h
              3rd-party/gco/graph.h
              3rd-party/gco/LinkedBlockList.h
              )

set(srcs    src/main.cpp
            src/organ.cpp
            src/color_map.cpp
            src/main_window.cpp
            src/parameter.cpp
            src/parameter_manager.cpp
            src/parameter_dialog.cpp
            src/point_cloud.cpp
            src/organ_leaf_smooth.cpp
            src/organ_leaf_decompose.cpp
            src/organ_stem_smooth.cpp
            src/organ_stem_decompose.cpp
            src/organ_control_flow.cpp
            src/organ_classify_leaf_stem.cpp
            src/status_io.cpp
            src/adapter_widget.cpp
            src/threaded_painter.cpp
            src/osg_viewer_widget.cpp
            src/log_viewer_widget.cpp
            src/file_viewer_widget.cpp
            src/toggle_handler.cpp
            src/resize_handler.cpp
            src/snapshot_handler.cpp
            src/stateset_manipulator.cpp
            src/renderable.cpp
            src/registrator.cpp
            src/axis_indicator.cpp
            src/light_source.cpp
            src/update_callback.cpp
            src/update_visitor.cpp
            src/povray_visitor.cpp
            src/file_system_model.cpp
            src/osg_utility.cpp
            src/cgal_utility.cpp
            src/math_solvers.cpp
            src/task_dispatcher.cpp
            src/information.cpp
            src/mesh_model.cpp
            src/progress_bar.cpp
            src/sketch_handler.cpp
            )

set(3rd_srcs  3rd-party/gco/GCoptimization.cpp
              3rd-party/gco/graph.cpp
              3rd-party/gco/LinkedBlockList.cpp
              3rd-party/gco/maxflow.cpp
              )

set(impl_incs include/impl/parameter.hpp
              )

# Organize files
SOURCE_GROUP("3rd-party\\include" FILES ${3rd_incs})
SOURCE_GROUP("3rd-party\\src" FILES ${3rd_srcs})
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

set(exe_name analyze)
add_executable(${exe_name} ${ui_srcs} ${moc_srcs} ${resource_srcs} ${srcs} ${3rd_srcs} ${incs} ${3rd_incs} ${impl_incs})
target_link_libraries(${exe_name} pcl_wrapper ${Boost_THREAD_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${OPENSCENEGRAPH_LIBRARIES} ${QT_QTOPENGL_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTXML_LIBRARY} ${QT_QTCORE_LIBRARY} ${CGAL_LIBRARIES} ${GMP_LIBRARIES} ${ThirdParty_LIBS})

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
