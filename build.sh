#!/bin/sh
#
# Build script for Unix-like systems, using the CMake subsystem
#
# This script covers some common cases. It does *not* install any
# prerequisites though.
#
# For documentation of AVRDUDE's build system, please refer to the
# Wiki:
#
# https://github.com/avrdudes/avrdude/wiki

# Determine OS type
#
# So far, this script tries to handle three different Unix-like
# systems:
#
# Linux
# FreeBSD
# Darwin (aka. MacOS)
#
# On Linux, if the machine is ARM-based, LINUXSPI and LINUXGPIO are
# enabled.
# On MacOS, an attempt is made to find out whether Mac ports or brew
# are in place, and are assumed to have install the prerequisites.

ostype=$(uname | tr '[A-Z]' '[a-z]')

build_type=RelWithDebInfo
# build_type=Release # no debug info

extra_enable=""
build_flags=""

case "${ostype}" in
    linux)
	# try to find out whether this is an Embedded Linux
	# platform (e.g. Raspberry Pi)
	machine=$(uname -m)
	if expr "${machine}" : '^\(arm\|aarch\)' >/dev/null
	then
	    extra_enable="${extra_enable} -D HAVE_LINUXGPIO=ON -D HAVE_LINUXSPI=ON"
	fi
	;;

    darwin)
	# determine whether we are running using Mac Ports
	# if not, assume Mac Brew
	if [ -f /opt/local/bin/port ]
	then
	    build_flags="${build_flags} -D CMAKE_C_FLAGS=-I/opt/local/include -D CMAKE_EXE_LINKER_FLAGS=-L/opt/local/lib"
	else
            # Apple M1 (may be new version of homebrew also)
            if [ -d /opt/homebrew ]  
            then
                build_flags="${build_flags} -D CMAKE_C_FLAGS=-I/opt/homebrew/include -D CMAKE_EXE_LINKER_FLAGS=-L/opt/homebrew/Cellar -D HAVE_LIBREADLINE:FILEPATH=/opt/homebrew/opt/readline/lib/libreadline.dylib"
            else
                build_flags="${build_flags} -D CMAKE_C_FLAGS=-I/usr/local/include -D CMAKE_EXE_LINKER_FLAGS=-L/usr/local/Cellar -D HAVE_LIBREADLINE:FILEPATH=/usr/local/opt/readline/lib/libreadline.dylib"
            fi
	fi
	;;

    freebsd)
	build_flags="${build_flags} -D CMAKE_C_FLAGS=-I/usr/local/include -D CMAKE_EXE_LINKER_FLAGS=-L/usr/local/lib"
	;;
esac

cmake ${build_flags} ${extra_enable} -D CMAKE_BUILD_TYPE=${build_type} -B build_${ostype} ||\
    { echo "CMake failed."; exit 1; }
cmake --build build_${ostype} ||\
    { echo "Build failed."; exit 1; }

cat <<EOF

Build succeeded.

Run

sudo cmake --build build_${ostype} --target install

to install.

EOF
