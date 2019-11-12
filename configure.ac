AC_INIT([optar-library], [0.0.1], [peter@remap.ucla.edu], [optar-library], [http://github.com/openptrack/optar-library])
AM_INIT_AUTOMAKE([1.10 -Wall -Werror no-define foreign subdir-objects])

AC_CONFIG_MACRO_DIR([m4])
AC_PREREQ([2.69])

AM_PROG_AR
AM_MAINTAINER_MODE
AM_SILENT_RULES([yes])

LT_INIT()

AC_PREFIX_DEFAULT([/usr/local])

AC_CONFIG_FILES([Makefile])

AC_CONFIG_SRCDIR([source/optar.cpp])
AC_CONFIG_HEADERS([source/config.hpp])

AC_PROG_CXX
AC_PROG_CC
AC_LANG([C++])

AX_CXX_COMPILE_STDCXX_11(, mandatory)
AC_CHECK_PROGS([TAR], [tar])

###############################
# Check for opencv
AC_CHECK_HEADERS([opencv2/opencv.hpp],
    [AC_CHECK_LIB([opencv], [CV_VERSION])],
    [
    PKG_CHECK_MODULES([OPENCV],
        [opencv4],
        ,
        [AC_MSG_FAILURE([can't find OpenCV])]
        )
    ])

###############################
# Check for <NTP client>

###############################
# Check for <ROS client>

###############################
# Check for spdlog

AC_OUTPUT
