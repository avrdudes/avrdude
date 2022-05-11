#
# programmers.cmake - create parts.texi from parts.txt
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

file(STRINGS ${COMMENTS_FILE} COMMENTS_CONTENTS)

file(STRINGS ${TXT_FILE} TXT_CONTENTS REGEX "=")

set(TEXI_CONTENTS "")
foreach(TXT_LINE IN LISTS TXT_CONTENTS)
    string(REGEX REPLACE "^[ \t]*([^ \t]+)[ \t]*=[ \t]*(.*)$" "@item @code{\\1} @tab \\2" TEXI_LINE "${TXT_LINE}")

    foreach(COMMENTS_LINE IN LISTS COMMENTS_CONTENTS)
        string(REGEX MATCH "^([^ \t]*)(.*)$" DUMMY "${COMMENTS_LINE}")
        set(PART_REGEX "${CMAKE_MATCH_1}")
        set(COMMENT "${CMAKE_MATCH_2}")
        string(REGEX REPLACE "(${PART_REGEX})" "\\1${COMMENT}" TEXI_LINE "${TEXI_LINE}")
    endforeach()

    set(TEXI_CONTENTS "${TEXI_CONTENTS}${TEXI_LINE}\n")
endforeach()

file(WRITE ${TEXI_FILE} "${TEXI_CONTENTS}")
