#
# configure.cmake - autoconf like multi-line configure
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

# Do a multi-line replace based on @<OPTION>_BEGIN@ and @<OPTION>_END@ tags.
macro(configure_option option)
    if(${${option}})
        string(REGEX REPLACE "(.*)@${option}_BEGIN@(.*)@${option}_END@(.*)" "\\1\\2\\3" CONTENTS "${CONTENTS}")
    else()
        string(REGEX REPLACE "(.*)@${option}_BEGIN@(.*)@${option}_END@(.*)" "\\1\\3" CONTENTS "${CONTENTS}")
    endif()
endmacro()

# Perform autoconf like multi-line configure
file(READ avrdude.conf.in CONTENTS)
configure_option(HAVE_PARPORT)
configure_option(HAVE_LINUXSPI)
configure_option(HAVE_LINUXGPIO)
file(WRITE avrdude.conf.in "${CONTENTS}")

configure_file(avrdude.conf.in avrdude.conf)
