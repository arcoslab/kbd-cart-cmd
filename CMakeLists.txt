# YARP needs CMake 2.6 or greater
cmake_minimum_required(VERSION 2.6.4)

PROJECT(kbd-cart-cmd)

# find YARP
find_package(YARP REQUIRED)

# add YARP include directories
include_directories(${YARP_INCLUDE_DIRS})

# set up our program
add_executable(kbd-cart-cmd kbd-cart-cmd.cc)

# link with YARP libraries
target_link_libraries(kbd-cart-cmd ${YARP_LIBRARIES})

INSTALL(PROGRAMS build/kbd-cart-cmd run_right.sh run_left.sh connect.sh DESTINATION bin)

SET(CPACK_GENERATOR "DEB")
SET(CPACK_PACKAGE_VERSION "0.1")
SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "Federico Ruiz Ugalde")
#SET(CPACK_DEBIAN_PACKAGE_VERSION "0.1")
SET(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
SET(CPACK_DEBIAN_PACKAGE_DEPENDS "libc6 (>= 2.3.1-6), libgcc1 (>= 1:3.4.2-12)")
SET(CPACK_DEBIAN_PACKAGE_DESCRIPTION "A program that controls the robot end-effector of vfclik")
INCLUDE(CPack)
