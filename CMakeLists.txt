cmake_minimum_required(VERSION 2.8)
project(libuvc)

message("CMAKE_SOURCE_DIR : ${CMAKE_SOURCE_DIR}")
message("PROJECT_NAME : ${PROJECT_NAME}")
message("libuvc_SOURCE_DIR : ${libuvc_SOURCE_DIR}")
message("libuvc_BINARY_DIR : ${libuvc_BINARY_DIR}")

# --------------------------------------------------
# Organize output files.  On Windows this also keeps .dll files next
# to the .exe files that need them, making tests easy to run.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${libuvc_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${libuvc_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${libuvc_BINARY_DIR}/lib)
message("CMAKE_LIBRARY_OUTPUT_DIRECTORY=${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
# -----------------------------------------------------------

if (NOT CMAKE_BUILD_TYPE)
  message(STATUS "No build type selected, default to Release")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif ()

if (NOT CMAKE_BUILD_TARGET)
  message(STATUS "No target type selected, default to shared library")
  set(CMAKE_BUILD_TARGET "Shared" CACHE STRING "" FORCE)
endif()

#--------------------------------------------------
#QT5
# Widgets finds its own dependencies (QtGui and QtCore).
find_package(Qt5Widgets)
#  find_package(Qt5Declarative)

# The Qt5Widgets_INCLUDES also includes the include directories for
# dependencies QtCore and QtGui
include_directories(${Qt5Widgets_INCLUDES})

# We need add -DQT_WIDGETS_LIB when using QtWidgets in Qt 5.
add_definitions(${Qt5Widgets_DEFINITIONS})

# Executables fail to build with Qt 5 in the default configuration
# without -fPIE. We add that here.
#  set(CMAKE_CXX_FLAGS "${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
#-----------------------------------------------

set(libuvc_VERSION_MAJOR 0)
set(libuvc_VERSION_MINOR 0)
set(libuvc_VERSION_PATCH 5)
set(libuvc_VERSION ${libuvc_VERSION_MAJOR}.${libuvc_VERSION_MINOR}.${libuvc_VERSION_PATCH})

find_package(PkgConfig)
pkg_check_modules(LIBUSB libusb-1.0)

# Try to find JPEG using a module or pkg-config. If that doesn't work, search for the header.
find_package(jpeg QUIET)
if(NOT JPEG_FOUND)
  find_path(JPEG_INCLUDE_DIR jpeglib.h)
  if(JPEG_INCLUDE_DIR)
    set(JPEG_FOUND ON)
    set(JPEG_LIBRARIES -ljpeg)
  endif()
endif()

SET(CMAKE_C_FLAGS_DEBUG "-g -DUVC_DEBUGGING")

SET(INSTALL_CMAKE_DIR "${CMAKE_INSTALL_PREFIX}/lib/cmake/libuvc" CACHE PATH
	"Installation directory for CMake files")

SET(SOURCES src/ctrl.c src/ctrl-gen.c src/device.c src/diag.c
           src/frame.c src/init.c src/stream.c
           src/misc.c)

include_directories(
  ${libuvc_SOURCE_DIR}/include
  ${libuvc_BINARY_DIR}/include
  ${LIBUSB_INCLUDE_DIRS}
)

if(JPEG_FOUND)
  message(STATUS "Building libuvc with JPEG support.")
  include_directories(${JPEG_INCLUDE_DIR})
  SET(LIBUVC_HAS_JPEG TRUE)
  SET(SOURCES ${SOURCES} src/frame-mjpeg.c)
else()
  message(WARNING "JPEG not found. libuvc will not support JPEG decoding.")
endif()


#searching openMP in toolchain path, instead of host path!
if(0)
find_package(OpenMP)
if(OPENMP_FOUND)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	add_definitions(-DENABLE_OPENMP)
endif()
endif()

#if(${CMAKE_BUILD_TARGET} MATCHES "Shared")
  add_library(uvc SHARED ${SOURCES})
#elseif(${CMAKE_BUILD_TARGET} MATCHES "Static")
  add_library(uvc_static STATIC ${SOURCES})
#else()
#  message( FATAL_ERROR "Invalid build type ${CMAKE_BUILD_TARGET}" )
#endif()

configure_file(include/libuvc/libuvc_config.h.in
  ${PROJECT_BINARY_DIR}/include/libuvc/libuvc_config.h @ONLY)

set_target_properties(uvc PROPERTIES
  PUBLIC_HEADER "include/libuvc/libuvc.h;${libuvc_BINARY_DIR}/include/libuvc/libuvc_config.h" )

if(JPEG_FOUND)
  target_link_libraries (uvc ${JPEG_LIBRARIES})
endif(JPEG_FOUND)

target_link_libraries(uvc ${LIBUSB_LIBRARIES})

message("uvc=${uvc}")

#add_executable(test src/test.c)
#target_link_libraries(test uvc ${LIBUSB_LIBRARIES} opencv_highgui
#  opencv_core)

install(TARGETS uvc
  EXPORT libuvcTargets
  LIBRARY DESTINATION "${CMAKE_INSTALL_PREFIX}/lib"
  ARCHIVE DESTINATION "${CMAKE_INSTALL_PREFIX}/lib"
  PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_PREFIX}/include/libuvc"
)

export(TARGETS uvc
  FILE "${PROJECT_BINARY_DIR}/libuvcTargets.cmake")
export(PACKAGE libuvc)

set(CONF_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/include")
set(CONF_LIBRARY "${CMAKE_INSTALL_PREFIX}/lib/${CMAKE_SHARED_LIBRARY_PREFIX}uvc${CMAKE_SHARED_LIBRARY_SUFFIX}")

configure_file(libuvcConfig.cmake.in ${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/libuvcConfig.cmake)

configure_file(libuvcConfigVersion.cmake.in ${PROJECT_BINARY_DIR}/libuvcConfigVersion.cmake @ONLY)

install(FILES
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/libuvcConfig.cmake"
  "${PROJECT_BINARY_DIR}/libuvcConfigVersion.cmake"
  DESTINATION "${INSTALL_CMAKE_DIR}")

install(EXPORT libuvcTargets
  DESTINATION "${INSTALL_CMAKE_DIR}")

#-------------------
add_subdirectory(test)
