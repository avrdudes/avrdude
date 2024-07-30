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

usage()
{
	echo "Build script for avrdude"
	echo
	echo "Syntax: build.sh -h -f <flags> -j <num>"
	echo "Options"
	echo "-h          Display this usage information and exit"
	echo "-f <flags>  Extra build flags to pass to cmake"
	echo "-j <num>    Run num build jobs in parallel"
	echo
}

ostype=$(uname | tr '[A-Z]' '[a-z]')

build_flags=""
cmake_build="cmake --build ."

while getopts :hf:j: OPT; do
  case "$OPT" in
    f)    
	   build_flags="$OPTARG" 
	   ;;
    j)
	   cmake_build="cmake --build . -- -j$OPTARG";
	   ;;
    h | *)
	   usage
	   exit
	   ;;
  esac
done
shift $((OPTIND-1)) # remove parsed options and args from $@ list


build_type=RelWithDebInfo
# build_type=Release # no debug info

# See CMakeLists.txt for all options
#
# Use this to enable (historical) parallel-port based programmers:
#extra_enable="-D HAVE_PARPORT=1"
extra_enable=""

case "${ostype}" in
    linux)
	# try to find out whether this is an Embedded Linux
	# platform (e.g. Raspberry Pi)
	machine=$(uname -m)
	if expr "${machine}" : '^\(arm\|aarch\)' >/dev/null
	then
	    extra_enable="${extra_enable} -D HAVE_LINUXGPIO=1 -D HAVE_LINUXSPI=1"
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
                build_flags="${build_flags} -D CMAKE_C_FLAGS=-I/opt/homebrew/include -D CMAKE_EXE_LINKER_FLAGS=-L/opt/homebrew/Cellar"
            else
                build_flags="${build_flags} -D CMAKE_C_FLAGS=-I/usr/local/include -D CMAKE_EXE_LINKER_FLAGS=-L/usr/local/Cellar"
            fi
	fi
	;;

    netbsd)
	build_flags="${build_flags} -D CMAKE_C_FLAGS=-I/usr/pkg/include -D CMAKE_EXE_LINKER_FLAGS=-R/usr/pkg/lib -D CMAKE_INSTALL_PREFIX:PATH=/usr/pkg"
	;;

    *bsd)
	build_flags="${build_flags} -D CMAKE_C_FLAGS=-I/usr/local/include -D CMAKE_EXE_LINKER_FLAGS=-L/usr/local/lib"
	;;
esac

mkdir -p build_${ostype}
cd build_${ostype}
cmake ${build_flags} ${extra_enable} -D CMAKE_BUILD_TYPE=${build_type} .. ||\
    { echo "CMake failed."; exit 1; }
${cmake_build} ||\
    { echo "Build failed."; exit 1; }

cat <<EOF

Build succeeded.

Run

sudo cmake --build build_${ostype} --target install

to install.

EOF
