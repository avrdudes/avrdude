#! /bin/sh

set -ex

cd "$(dirname "$0")"
top_srcdir="$(pwd)"

ostype="$(uname | tr A-Z a-z)"

if test "x$MAKE" = x; then
  if gmake --version > /dev/null 2>&1; then
    MAKE=gmake
  fi
fi
make="${make-"${MAKE-make} -j$(nproc)"}"

rm -rf autom4te.cache/

prefix="$top_srcdir/build_autotools-prefix"
rm -rf "$prefix"

top_builddir="$top_srcdir/build_autotools"

# Remove remainders of failed distcheck attempts
for dir in "$top_builddir"
do
    if test -d "$dir"; then
	chmod -R +w "$dir"
    fi
done
rm -rf "$top_builddir"

./bootstrap

mkdir "$top_builddir"
cd "$top_builddir"

configure_opts=""
configure_opts="$configure_opts --disable-silent-rules"
configure_opts="$configure_opts --enable-parport"
case "$ostype" in
  *linux)
    configure_opts="$configure_opts --enable-linuxgpio --enable-linuxspi"
    configure_opts="$configure_opts --enable-doc"
    ;;
esac

$top_srcdir/configure --prefix="$prefix" ${configure_opts}

$make all
$make check
$make install
$make installcheck

printf "\n\n" | $top_srcdir/../tools/test-avrdude -e $prefix/bin/avrdude -d0 -p"-cdryrun -pm2560" -p"-cdryrun -pavr64du28"

$make distcheck
