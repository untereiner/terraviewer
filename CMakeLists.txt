cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(terraviewer
	LANGUAGES CXX C
	)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/CMakeOptions.txt)
    include(${CMAKE_CURRENT_SOURCE_DIR}/CMakeOptions.txt)
else()
    message(FATAL_ERROR "No local options file specifying CGOGN directory: CMakeOptions.txt (see CMakeOptions.txt.sample)")
endif()

#### Default build type
if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
	message("cgogn in release mode")
	list(APPEND CMAKE_PREFIX_PATH "${CGOGN_RELEASE_BUILD_DIR}/lib/cmake")
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
	message("cgogn in debug mode")
	list(APPEND CMAKE_PREFIX_PATH "${CGOGN_DEBUG_BUILD_DIR}/lib/cmake")
endif()

find_package(Qt5 5.5.0 COMPONENTS Widgets REQUIRED)
find_package(QOGLViewer REQUIRED)
find_package(cgogn_core REQUIRED)
find_package(cgogn_geometry REQUIRED)
find_package(cgogn_io REQUIRED)
find_package(cgogn_rendering REQUIRED)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -std=c++11")

set(HEADER_FILES
	mnt_io.h	
	)

set(SOURCE_FILES
	viewer.cpp
	)

# include_directories(
#         ${CGOGN_ROOT_DIR}/cgogn
#         ${CGOGN_ROOT_DIR}/thirdparty/libQGLViewer/QOGLViewer
#         ${CGOGN_ROOT_DIR}/thirdparty/eigen-3.2.8
#         ${CGOGN_ROOT_DIR}/thirdparty/TinyXml2
#         ${CGOGN_ROOT_DIR}/thirdparty/ply
#         ${CGOGN_ROOT_DIR}/thirdparty/lm6)

# link_directories(${CGOGN_ROOT_DIR}/build/debug/lib/)  #${CMAKE_BUILD_TYPE} in place of debug


add_executable(${PROJECT_NAME} ${HEADER_FILES} ${SOURCE_FILES})# ${SOURCE_MOC} ${SOURCE_UI})
target_link_libraries(${PROJECT_NAME} ${cgogn_core_LIBRARIES} ${cgogn_geometry_LIBRARIES} ${cgogn_io_LIBRARIES} ${cgogn_rendering_LIBRARIES} ${Qt5Widgets_LIBRARIES} ${QOGLViewer_LIBRARIES})
