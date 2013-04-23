set(Boost_USE_STATIC_LIBS  ON)
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF)

find_package(Boost REQUIRED system)
find_package(PCL REQUIRED common io registration kdtree search)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

set(incs      include/pcl_wrapper_types.h
              include/pcl_wrapper_exports.h
              )

set(srcs      src/pcl_wrapper.cpp
              )

set(impl_incs )


set(lib_name pcl_wrapper)
add_library(${lib_name} SHARED ${srcs} ${incs} ${impl_incs})
target_link_libraries(${lib_name} ${Boost_SYSTEM_LIBRARY}
  debug ${PCL_COMMON_LIBRARY_DEBUG} optimized ${PCL_COMMON_LIBRARY}
  debug ${PCL_IO_LIBRARY_DEBUG} optimized ${PCL_IO_LIBRARY}
  debug ${PCL_REGISTRATION_LIBRARY_DEBUG} optimized ${PCL_REGISTRATION_LIBRARY}
  debug ${PCL_KDTREE_LIBRARY_DEBUG} optimized ${PCL_KDTREE_LIBRARY}
  debug ${PCL_SEARCH_LIBRARY_DEBUG} optimized ${PCL_SEARCH_LIBRARY})

if(WIN32 AND MSVC)
  set_target_properties(${lib_name} PROPERTIES LINK_FLAGS_RELEASE /OPT:REF)
elseif(CMAKE_SYSTEMname STREQUAL "Darwin")
  if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set_target_properties(${lib_name} PROPERTIES LINK_FLAGS -Wl)
  endif()
elseif(__COMPILER_PATHSCALE)
  set_target_properties(${lib_name} PROPERTIES LINK_FLAGS -mp)
else()
  set_target_properties(${lib_name} PROPERTIES LINK_FLAGS -Wl,--as-needed)
endif()

set_target_properties(${lib_name} PROPERTIES DEFINE_SYMBOL "PCL_WRAPPER_API_EXPORTS")

set_target_properties(${lib_name} PROPERTIES DEBUG_POSTFIX _debug)
set_target_properties(${lib_name} PROPERTIES RELEASE_POSTFIX _release)
