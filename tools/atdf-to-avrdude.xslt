<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
    xmlns:xs="http://www.w3.org/2001/XMLSchema"
    xmlns:f="http://functions"
    exclude-result-prefixes="xs f"
    version="2.0">

    <!--
        avrdude - A Downloader/Uploader for AVR device programmers
        Copyright (C) 2018 Morten Engelhardt Olsen <meolsen@atmel.com>

        This program is free software; you can redistribute it and/or modify
        it under the terms of the GNU General Public License as published by
        the Free Software Foundation; either version 2 of the License, or
        (at your option) any later version.

        This program is distributed in the hope that it will be useful,
        but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
        GNU General Public License for more details.

        You should have received a copy of the GNU General Public License
        along with this program. If not, see <http://www.gnu.org/licenses/>.
    -->

    <xsl:output method="text"/>

    <xsl:template match="/">

        <xsl:variable name="architecture"
            select="/avr-tools-device-file/devices/device/@architecture"/>

        <xsl:choose>
            <xsl:when test="$architecture = 'AVR8X'">
                <xsl:call-template name="create-avr8x-config"/>
            </xsl:when>
            <xsl:otherwise>
                <xsl:message terminate="yes">No support for architecture '<xsl:value-of select="$architecture"/>'</xsl:message>
            </xsl:otherwise>
        </xsl:choose>
    </xsl:template>

    <xsl:template name="create-avr8x-config">

        <xsl:variable name="device-name"
            select="/avr-tools-device-file/devices/device/@name"/>
        <xsl:variable name="device-id">
            <xsl:call-template name="get-device-id">
                <xsl:with-param name="device-name" select="$device-name"/>
            </xsl:call-template>
        </xsl:variable>
        <xsl:variable name="parent-name">
            <xsl:call-template name="get-parent-name">
                <xsl:with-param name="device-name" select="$device-name"/>
            </xsl:call-template>
        </xsl:variable>
        <xsl:variable name="device-signature">
            <xsl:call-template name="get-device-signature">
                <xsl:with-param name="device-name" select="$device-name"/>
            </xsl:call-template>
        </xsl:variable>

        <xsl:call-template name="write-conf-header">
            <xsl:with-param name="device-name" select="$device-name"/>
        </xsl:call-template>

        <xsl:text>part parent    "</xsl:text><xsl:value-of select="normalize-space($parent-name)"/><xsl:text>";&#xa;</xsl:text>
        <xsl:text>    id        = "</xsl:text><xsl:value-of select="normalize-space($device-id)"/><xsl:text>";&#xa;</xsl:text>
        <xsl:text>    desc      = "</xsl:text><xsl:value-of select="normalize-space($device-name)"/><xsl:text>";&#xa;</xsl:text>
        <xsl:text>    signature = </xsl:text><xsl:value-of select="normalize-space($device-signature)"/><xsl:text>;&#xa;</xsl:text>

        <xsl:call-template name="write-memories">
            <xsl:with-param name="device-name" select="$device-name"/>
        </xsl:call-template>

        <xsl:text>;&#xa;</xsl:text>
    </xsl:template>

    <xsl:template name="write-memories">
        <xsl:param name="device-name"/>

        <xsl:call-template name="write-memory">
            <xsl:with-param name="memory-name" select="'PROGMEM'"/>
        </xsl:call-template>
        <xsl:call-template name="write-memory">
            <xsl:with-param name="memory-name" select="'EEPROM'"/>
        </xsl:call-template>
    </xsl:template>

    <xsl:template name="write-memory">
        <xsl:param name="memory-name"/>

        <xsl:variable name="memory-segment" select="//memory-segment[@name = $memory-name]"/>
        <xsl:choose>
            <xsl:when test="not($memory-segment)">
                <xsl:message terminate="yes">No memory-segment named '<xsl:value-of select="$memory-name"/>'</xsl:message>
            </xsl:when>
            <xsl:when test="count($memory-segment) > 1">
                <xsl:message terminate="yes">More than one memory-segments named '<xsl:value-of select="$memory-name"/>'</xsl:message>
            </xsl:when>
            <xsl:otherwise>
                <xsl:call-template name="write-memory-segment">
                    <xsl:with-param name="memory-segment" select="$memory-segment"/>
                </xsl:call-template>
            </xsl:otherwise>
        </xsl:choose>
    </xsl:template>

    <xsl:template name="write-memory-segment">
        <xsl:param name="memory-segment"/>

        <xsl:variable name="size">
            <xsl:value-of select="concat('0x', f:decimalToHex(f:hexToDec($memory-segment/@size)))"/>
        </xsl:variable>

        <xsl:variable name="offset">
            <xsl:choose>
                <xsl:when test="$memory-segment/@type = 'flash' and $memory-segment/ancestor::device/@architecture = 'AVR8X'">
                    <xsl:variable name="progmem-base"
                                  select="f:hexToDec($memory-segment/ancestor::device/property-groups/property-group[@name = 'UPDI_INTERFACE']/property[@name = 'PROGMEM_OFFSET']/@value)"/>
                    <xsl:value-of select="concat('0x', f:decimalToHex(f:hexToDec($memory-segment/@start) + $progmem-base))"/>
                </xsl:when>
                <xsl:otherwise>
                    <xsl:value-of select="concat('0x', f:decimalToHex(f:hexToDec($memory-segment/@start)))"/>
                </xsl:otherwise>
            </xsl:choose>
        </xsl:variable>

        <xsl:text>&#xa;</xsl:text>
        <xsl:text>    memory "</xsl:text><xsl:value-of select="$memory-segment/@type"/><xsl:text>"&#xa;</xsl:text>
        <xsl:text>        size      = </xsl:text><xsl:value-of select="$size"/><xsl:text>;&#xa;</xsl:text>
        <xsl:text>        offset    = </xsl:text><xsl:value-of select="$offset"/><xsl:text>;&#xa;</xsl:text>
        <xsl:text>        page_size = </xsl:text><xsl:value-of select="$memory-segment/@pagesize"/><xsl:text>;&#xa;</xsl:text>
        <xsl:text>        readsize  = 0x100;&#xa;</xsl:text>
        <xsl:text>    ;&#xa;</xsl:text>
    </xsl:template>

    <xsl:template name="write-conf-header">
        <xsl:param name="device-name"/>

        <xsl:text>#------------------------------------------------------------&#xa;</xsl:text>
        <xsl:text># </xsl:text><xsl:value-of select="$device-name"/><xsl:text>&#xa;</xsl:text>
        <xsl:text>#------------------------------------------------------------&#xa;</xsl:text>
        <xsl:text>&#xa;</xsl:text>
    </xsl:template>

    <xsl:template name="get-device-id">
        <xsl:param name="device-name"/>

        <xsl:choose>
            <xsl:when test="contains($device-name, 'ATtiny')">
                <xsl:value-of select="replace($device-name, 'ATtiny', 't')"/>
            </xsl:when>
            <xsl:when test="contains($device-name, 'ATmega')">
                <xsl:value-of select="replace($device-name, 'ATmega', 'm')"/>
            </xsl:when>
            <xsl:otherwise>
                <xsl:message terminate="yes">Unable to deduce device id from '<xsl:value-of select="$device-name"/>'</xsl:message>
            </xsl:otherwise>
        </xsl:choose>
    </xsl:template>

    <xsl:template name="get-parent-name">
        <xsl:param name="device-name"/>

        <xsl:choose>
            <xsl:when test="contains($device-name, 'ATtiny')">.avr8x_tiny</xsl:when>
            <xsl:when test="contains($device-name, 'ATmega')">.avr8x_mega</xsl:when>
            <xsl:otherwise>
                <xsl:message terminate="yes">Unable to deduce parent name from '<xsl:value-of select="$device-name"/>'</xsl:message>
            </xsl:otherwise>
        </xsl:choose>
    </xsl:template>

    <xsl:template name="get-device-signature">
        <xsl:param name="device-name"/>

        <xsl:variable name="signatures"
            select="/avr-tools-device-file/devices/device/property-groups/property-group[@name = 'SIGNATURES']"/>

        <xsl:for-each select="$signatures/property[contains(@name, 'SIGNATURE')]">
            <xsl:value-of select="@value"/><xsl:text> </xsl:text>
        </xsl:for-each>
    </xsl:template>

    <!-- Helper functions (from https://stackoverflow.com/questions/22905134/convert-a-hexadecimal-number-to-an-integer-in-xslt) -->
    <xsl:function name="f:hexToDec">
        <xsl:param name="hex"/>
        <xsl:variable name="dec"
            select="string-length(substring-before('0123456789ABCDEF', substring($hex,1,1)))"/>
        <xsl:choose>
            <xsl:when test="matches($hex, '([0-9]*|[A-F]*)')">
                <xsl:value-of
                    select="if ($hex = '') then 0
                    else $dec * f:power(16, string-length($hex) - 1) + f:hexToDec(substring($hex,2))"/>
            </xsl:when>
            <xsl:otherwise>
                <xsl:message>Provided value is not hexadecimal...</xsl:message>
                <xsl:value-of select="$hex"/>
            </xsl:otherwise>
        </xsl:choose>
    </xsl:function>

    <xsl:function name="f:power">
        <xsl:param name="base"/>
        <xsl:param name="exp"/>
        <xsl:sequence
            select="if ($exp lt 0) then f:power(1.0 div $base, -$exp)
            else if ($exp eq 0)
            then 1e0
            else $base * f:power($base, $exp - 1)"
        />
    </xsl:function>

    <xsl:function name="f:decimalToHex">
        <xsl:param name="dec"/>
        <xsl:if test="$dec > 0">
            <xsl:value-of
                select="f:decimalToHex(floor($dec div 16)),substring('0123456789ABCDEF', (($dec mod 16) + 1), 1)"
                separator=""
            />
        </xsl:if>
    </xsl:function>
</xsl:stylesheet>
