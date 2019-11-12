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
AC_CONFIG_HEADERS([source/optar.hpp])

AC_PROG_CXX
AC_PROG_CC
AC_LANG([C++])

AX_CXX_COMPILE_STDCXX_11(, mandatory)
AC_CHECK_PROGS([TAR], [tar])

###############################
# Check for opencv

###############################
# Check for <NTP client>

###############################
# Check for <ROS client>

###############################
# Check for spdlog

AC_OUTPUT
