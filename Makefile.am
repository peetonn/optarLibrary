lib_LTLIBRARIES = liboptar.la

pkginclude_HEADERS = \
	source/optar.hpp \
	source/logging.hpp
liboptar_la_SOURCES = \
	source/optar.cpp \
	source/logging.cpp
liboptar_la_CPPFLAGS = $(OPENCV_CFLAGS)
liboptar_la_LIBADD = $(OPENCV_LIBS)

bin_PROGRAMS = test-optar

test_optar_SOURCES = test/test-optar.cpp
test_optar_CXXFLAGS = -I$(top_srcdir)/source
test_optar_LDADD = liboptar.la
