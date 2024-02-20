# -*- makefile-automake -*-

EXTRA_DIST += .editorconfig
EXTRA_DIST += CMakeLists.txt
EXTRA_DIST += CMakeSettings.json
EXTRA_DIST += bootstrap
EXTRA_DIST += autotools-build.sh
EXTRA_DIST += build.sh

# Ship spec file to make "rpmbuild -ta avrdude-*.tar.*" command work
EXTRA_DIST += avrdude.spec
