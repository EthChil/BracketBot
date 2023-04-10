# - Try to find ORB_SLAM2_CUDA
# Set alternative paths to search for using ORB_SLAM2_CUDA_DIR
# Once done this will define
#  ORB_SLAM2_CUDA_FOUND - System has ORB_SLAM2_CUDA
#  ORB_SLAM2_CUDA_INCLUDE_DIRS - The ORB_SLAM2_CUDA include directories
#  ORB_SLAM2_CUDA_LIBRARIES - The libraries needed to use ORB_SLAM2_CUDA
#  ORB_SLAM2_CUDA_DEFINITIONS - Compiler switches required for using ORB_SLAM2_CUDA

# TODO: This need to find dependencies properly, I can't find an example of how to do that
#find_package(OpenCV REQUIRED)
#find_package(Eigen3 REQUIRED)
#find_package(Pangolin REQUIRED)

set(_ORB_SLAM2_SEARCHES /home/bracketbot/Desktop/BracketBot/ORB_SLAM2_CUDA)
if (ORB_SLAM2_DIR)
    set(_ORB_SLAM2_SEARCHES ${ORB_SLAM2_DIR} ${_ORB_SLAM2_SEARCHES})
endif()
find_path(ORB_SLAM2_INCLUDE_DIR ORB_SLAM2_CUDA/System.h
          PATHS ${_ORB_SLAM2_SEARCHES} PATH_SUFFIXES include)

find_library(ORB_SLAM2_LIBRARY NAMES ORB_SLAM2 libORB_SLAM2
             PATHS ${_ORB_SLAM2_SEARCHES} PATH_SUFFIXES lib)

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set ORB_SLAM2_CUDA_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(ORB_SLAM2  DEFAULT_MSG
                                  ORB_SLAM2_LIBRARY ORB_SLAM2_INCLUDE_DIR)

mark_as_advanced(ORB_SLAM2_INCLUDE_DIR ORB_SLAM2_LIBRARY )

set(ORB_SLAM2_LIBRARIES ${ORB_SLAM2_LIBRARY})
set(ORB_SLAM2_INCLUDE_DIRS ${ORB_SLAM2_INCLUDE_DIR})

