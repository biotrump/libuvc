cmake_minimum_required(VERSION 2.8)
project(test)

message("CMAKE_SOURCE_DIR : ${CMAKE_SOURCE_DIR}")
message("PROJECT_NAME : ${PROJECT_NAME}")
message("test_SOURCE_DIR : ${test_SOURCE_DIR}")
message("test_BINARY_DIR : ${test_BINARY_DIR}")

# --------------------------------------------------
# Organize output files.  On Windows this also keeps .dll files next
# to the .exe files that need them, making tests easy to run.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${test_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${test_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${test_BINARY_DIR}/lib)
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

#SET(CMAKE_C_FLAGS_DEBUG "-g -DUVC_DEBUGGING")

FIND_PACKAGE( OpenCV REQUIRED)

if(JPEG_FOUND)
  message(STATUS "Building test with JPEG support.")
  include_directories(${JPEG_INCLUDE_DIR})
  SET(LIBUVC_HAS_JPEG TRUE)
  SET(SOURCES ${SOURCES} src/frame-mjpeg.c)
else()
  message(WARNING "JPEG not found. test will not support JPEG decoding.")
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

message("uvc=${uvc}")

#--------------------
if (0)
find_library (LIBUVCx
			NAMES uvc_static
			PATHS ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
			PATH_SUFFIXES lib
			NO_DEFAULT_PATH)
message("LIBUVCx=${LIBUVCx}")
endif()

#-------------------
if(1)
add_executable(example
	example.c
)
qt5_use_modules(example Widgets  OpenGL Test Concurrent)
#message("LIBUSB_LIBRARIES=${LIBUSB_LIBRARIES}")
#message("JPEG_LIBRARIES=${JPEG_LIBRARIES}")
target_link_libraries(example
	${LIBUSB_LIBRARIES}
	${JPEG_LIBRARIES}
	uvc
	${OpenCV_LIBS}
#	${uvc}
)
endif()

#
#add_executable(test src/test.c)
#target_link_libraries(test uvc ${LIBUSB_LIBRARIES} opencv_highgui
#  opencv_core)
#
if(1)
add_executable(test
	test.c
)
qt5_use_modules(test Widgets  OpenGL Test Concurrent)
#message("LIBUSB_LIBRARIES=${LIBUSB_LIBRARIES}")
#message("JPEG_LIBRARIES=${JPEG_LIBRARIES}")
target_link_libraries(test
	${LIBUSB_LIBRARIES}
	${JPEG_LIBRARIES}
	uvc
	${OpenCV_LIBS}
#	opencv_highgui
)
endif()
