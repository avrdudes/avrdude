#
# programmer_types.cmake - create programmer_types.texi from programmer_types.txt
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

file(STRINGS ${TXT_FILE} TXT_CONTENTS REGEX "=")

SET(TEXI_CONTENTS "")
foreach(TXT_LINE IN LISTS TXT_CONTENTS)
    string(REGEX REPLACE "^[ \t]*([^ \t]+)[ \t]*=[ \t]*(.*)$" "@item @code{\\1} @tab \\2" TEXI_LINE "${TXT_LINE}")
    string(REGEX REPLACE "<?(http[s]?://[^ \t,>]+)>?" "@url{\\1}" TEXI_LINE "${TEXI_LINE}")
    set(TEXI_CONTENTS "${TEXI_CONTENTS}${TEXI_LINE}\n")
endforeach()

file(WRITE ${TEXI_FILE} "${TEXI_CONTENTS}")
