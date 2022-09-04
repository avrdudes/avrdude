#
# CMakeLists.txt - CMake project for AVRDUDE documentation
# Copyright (C) 2022 Marius Greuel
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
#

set(AVRDUDE_CONF "${PROJECT_BINARY_DIR}/src/avrdude.conf")

set(TEXINFOS "${CMAKE_CURRENT_SOURCE_DIR}/avrdude.texi")
set(GENERATED_TEXINFOS
    programmers.texi
    programmer_types.texi
    parts.texi
    version.texi
    )

string(TIMESTAMP TODAY "%d %B %Y")
set(DOCS_VERSION ${PROJECT_VERSION})
set(DOCS_UPDATED ${TODAY})

find_program(MAKEINFO_EXECUTABLE NAMES makeinfo)
find_program(TEXI2HTML_EXECUTABLE NAMES texi2html)

# =====================================
# Custom rules for auto-generated texi
# =====================================

add_custom_target(avrdude_binaries DEPENDS avrdude conf)

add_custom_command(
    OUTPUT programmers.txt
    DEPENDS avrdude_binaries
    COMMAND $<TARGET_FILE:avrdude> -C ${AVRDUDE_CONF} -c ? 2>&1 | more > programmers.txt
    VERBATIM
    )

add_custom_command(
    OUTPUT programmer_types.txt
    DEPENDS avrdude_binaries
    COMMAND $<TARGET_FILE:avrdude> -C ${AVRDUDE_CONF} -c ?type 2>&1 | more > programmer_types.txt
    VERBATIM
    )

add_custom_command(
    OUTPUT parts.txt
    DEPENDS avrdude_binaries
    COMMAND $<TARGET_FILE:avrdude> -C ${AVRDUDE_CONF} -p ? 2>&1 | more > parts.txt
    VERBATIM
    )

add_custom_command(
    OUTPUT programmers.texi
    DEPENDS programmers.txt
    COMMAND ${CMAKE_COMMAND}
        -D TXT_FILE=programmers.txt
        -D TEXI_FILE=programmers.texi
        -P "${CMAKE_CURRENT_SOURCE_DIR}/programmers.cmake"
    VERBATIM
    )

add_custom_command(
    OUTPUT programmer_types.texi
    DEPENDS programmer_types.txt
    COMMAND ${CMAKE_COMMAND}
        -D TXT_FILE=programmer_types.txt
        -D TEXI_FILE=programmer_types.texi
        -P "${CMAKE_CURRENT_SOURCE_DIR}/programmer_types.cmake"
    VERBATIM
    )

add_custom_command(
    OUTPUT parts.texi
    DEPENDS parts.txt
    COMMAND ${CMAKE_COMMAND}
        -D TXT_FILE=parts.txt
        -D TEXI_FILE=parts.texi
        -D COMMENTS_FILE=${CMAKE_CURRENT_SOURCE_DIR}/parts_comments.txt
        -P "${CMAKE_CURRENT_SOURCE_DIR}/parts.cmake"
    VERBATIM
    )

add_custom_command(
    OUTPUT version.texi
    COMMAND ${CMAKE_COMMAND} -E echo "@set EDITION ${DOCS_VERSION}" > version.texi
    COMMAND ${CMAKE_COMMAND} -E echo "@set VERSION ${DOCS_VERSION}" >> version.texi
    COMMAND ${CMAKE_COMMAND} -E echo "@set UPDATED ${DOCS_UPDATED}" >> version.texi
    VERBATIM
    )

# =====================================
# Custom rules for output files
# =====================================

add_custom_command(
    OUTPUT avrdude.info
    COMMAND ${MAKEINFO_EXECUTABLE} -o avrdude.info ${TEXINFOS}
    DEPENDS ${TEXINFOS} ${GENERATED_TEXINFOS}
    VERBATIM
    )

add_custom_command(
    OUTPUT avrdude.dvi
    COMMAND ${MAKEINFO_EXECUTABLE}
        --dvi
        --Xopt=--quiet
        --Xopt=--build-dir=dvi
        -o avrdude.dvi
        ${TEXINFOS}
    DEPENDS ${TEXINFOS} ${GENERATED_TEXINFOS}
    VERBATIM
    )

add_custom_command(
    OUTPUT avrdude.pdf
    COMMAND ${MAKEINFO_EXECUTABLE}
        --pdf
        --Xopt=--quiet
        --Xopt=--build-dir=pdf
        -o avrdude.pdf
        ${TEXINFOS}
    DEPENDS ${TEXINFOS} ${GENERATED_TEXINFOS}
    VERBATIM
    )

add_custom_command(
    OUTPUT avrdude.ps
    COMMAND ${MAKEINFO_EXECUTABLE}
        --ps
        --Xopt=--quiet
        --Xopt=--build-dir=ps
        -o avrdude.ps
        ${TEXINFOS}
    DEPENDS ${TEXINFOS} ${GENERATED_TEXINFOS}
    VERBATIM
    )

add_custom_command(
    OUTPUT avrdude-html/avrdude.html
    COMMAND ${TEXI2HTML_EXECUTABLE}
        --split=node
        --css-include=avrdude.css
        --output=avrdude-html
        -I ${CMAKE_CURRENT_BINARY_DIR}
        ${TEXINFOS}
    DEPENDS ${TEXINFOS} ${GENERATED_TEXINFOS} avrdude.css
    VERBATIM
    )

# =====================================
# Custom targets for output files
# =====================================

add_custom_target(info ALL DEPENDS avrdude.info)
add_custom_target(dvi ALL DEPENDS avrdude.dvi)
add_custom_target(pdf ALL DEPENDS avrdude.pdf)
add_custom_target(ps ALL DEPENDS avrdude.ps)
add_custom_target(html ALL DEPENDS avrdude-html/avrdude.html)

# =====================================
# Install
# =====================================

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/avrdude.info" DESTINATION ${CMAKE_INSTALL_INFODIR})
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/avrdude.dvi" DESTINATION ${CMAKE_INSTALL_DOCDIR})
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/avrdude.pdf" DESTINATION ${CMAKE_INSTALL_DOCDIR})
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/avrdude.ps" DESTINATION ${CMAKE_INSTALL_DOCDIR})
install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/avrdude-html" DESTINATION ${CMAKE_INSTALL_DOCDIR})
