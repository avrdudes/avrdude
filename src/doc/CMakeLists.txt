set(GEN_TEXI_FILES_LIST
	version.texi
	parts.texi
	programmers.texi
	programmer_types.texi
	)

set(DOC_TYPES_LIST
	pdf
	dvi
	ps
	html
	)

if("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
	set(SCRIPT_DEBUG "TRUE")
endif()

# =====================================
# Definitions
# =====================================

set(GEN_TEXI_SCRIPT "gen_texi.sh")
set(GEN_DOC_SCRIPT "gen_doc.sh")
set(GEN_VERSION_SCRIPT "gen_version.sh")
set(AWK_SCRIPT "texify.awk")

set(DOC_NAME ${PROJECT_NAME})

set(MAIN_TEXI "${DOC_NAME}.texi")
set(STYLE_SHEET "${DOC_NAME}.css")
set(COMMENTS "parts_comments.txt")

set(PLATFORM "${CMAKE_SYSTEM_NAME}")
if (${MINGW})
	set(PLATFORM "MINGW")
endif()

set(DOC_FILES_LIST "")
set(TAR_BALL "${PROJECT_NAME}-doc.tar.gz")
set(TAR_BALL_PREFIX_DIR "doc")

if(${CMAKE_SYSTEM_NAME} STREQUAL "Darwin")
	if(${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL "arm64")
		set(BREW_PREFIX "/opt/homebrew/opt")
	else()
		set(BREW_PREFIX "/usr/local/opt")
	endif()
	set(BREW_PATH_TEXINFO "${BREW_PREFIX}/texinfo/bin")
endif()

# =====================================
# Functions
# =====================================
function(find_program_cmake_3_18 VAR HINTS)
	find_program("${VAR}"
		NAMES ${ARGN}
		HINTS "${HINTS}"
		)
	if(${VAR} STREQUAL "RES-NOTFOUND")
		message(FATAL_ERROR "Did not find executable: ${VAR}")
	endif()
endfunction()

# =====================================
# Find programs
# =====================================

# Trying to find a program and proceed even if we didn't find it does not make sense for the
# following ones. Prior to 3.18 we will have to check afterwards with 'if' - That's dumb!
# find_program has a nice REQUIRED option since 3.18.
# We will use a wrapper function to emulate desired behaviour and keep code clean.
# cmake 3.18. Enforce fix of this as soon as we get there
# -------------------------------------
# find_program(VAR
# 	NAMES foo bar
#	HINTS ...
#	REQUIRED
#	)
# -------------------------------------
if(${CMAKE_MINIMUM_REQUIRED_VERSION} VERSION_GREATER 3.18)
	message(FATAL_ERROR "Fix find_program by setting REQUIRED flag")
endif()

find_program_cmake_3_18(AWK_BIN ""
	awk
	)

find_program_cmake_3_18(MAKEINFO_BIN "${BREW_PATH_TEXINFO}"
	makeinfo
	)

find_program_cmake_3_18(TEXI2DVI_BIN "${BREW_PATH_TEXINFO}"
	texi2dvi
	)

find_program_cmake_3_18(TEX_BIN ""
	tex
	)

find_program_cmake_3_18(TAR_BIN ""
	gtar
	tar
	)

# =====================================
# Prepare Build Dir
# =====================================

file(COPY ${MAIN_TEXI} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY ${STYLE_SHEET} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY ${COMMENTS} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY ${AWK_SCRIPT} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# The use of a copy after configure is ugly but there is no other way prior to
# cmake 3.20. Enforce fix of this as soon as we get there
# -------------------------------------
# configure_file(${SCRIPT}.in ${SCRIPT}
#	FILE_PERMISSIONS OWNER_EXECUTE OWNER_READ
#	@ONLY
#	)
# -------------------------------------
if(${CMAKE_MINIMUM_REQUIRED_VERSION} VERSION_GREATER 3.20)
	message(FATAL_ERROR "Fix configure_file by setting permissions directly and drop file copy")
endif()
configure_file(${GEN_TEXI_SCRIPT}.in tmp/${GEN_TEXI_SCRIPT} @ONLY)
file(COPY ${CMAKE_CURRENT_BINARY_DIR}/tmp/${GEN_TEXI_SCRIPT}
		DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
		FILE_PERMISSIONS
		OWNER_EXECUTE OWNER_READ
	)

configure_file(${GEN_VERSION_SCRIPT}.in tmp/${GEN_VERSION_SCRIPT} @ONLY)
file(COPY ${CMAKE_CURRENT_BINARY_DIR}/tmp/${GEN_VERSION_SCRIPT}
		DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
		FILE_PERMISSIONS
		OWNER_EXECUTE OWNER_READ
	)

configure_file(${GEN_DOC_SCRIPT}.in tmp/${GEN_DOC_SCRIPT} @ONLY)
file(COPY ${CMAKE_CURRENT_BINARY_DIR}/tmp/${GEN_DOC_SCRIPT}
		DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
		FILE_PERMISSIONS
		OWNER_EXECUTE OWNER_READ
	)

# =====================================
# Texi Targets
# =====================================

foreach(TEXI ${GEN_TEXI_FILES_LIST})

	add_custom_target("${TEXI}_target" DEPENDS ${TEXI})

	add_custom_command(OUTPUT ${TEXI}
		COMMAND ${CMAKE_CURRENT_BINARY_DIR}/${GEN_TEXI_SCRIPT} ${TEXI} $<TARGET_FILE:avrdude>
		DEPENDS ${GEN_TEXI_SCRIPT}.in
		)

	add_dependencies("${TEXI}_target"
		avrdude
		)

endforeach()

# =====================================
# Doc Targets
# =====================================

list(TRANSFORM GEN_TEXI_FILES_LIST APPEND "_target" OUTPUT_VARIABLE GEN_TEXI_FILES_LIST_TARGETS)
foreach(DOC_TYPE ${DOC_TYPES_LIST})

	set(DOC_FILE "${DOC_TYPE}")

	if("${DOC_TYPE}" STREQUAL "html")
		# required for dependency handling. A directory will always be out of date
		set(OUT_FILE "html/Index.html")

		install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${DOC_FILE}
			DESTINATION ${CMAKE_INSTALL_DOCDIR}
			)
	else()
		string(PREPEND DOC_FILE "${DOC_NAME}.")
		set(OUT_FILE "${DOC_FILE}")

		install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${DOC_FILE}
			DESTINATION ${CMAKE_INSTALL_DOCDIR}
			)
	endif()

	list(APPEND DOC_FILES_LIST ${DOC_FILE})

	add_custom_target("doc_${DOC_TYPE}_target" DEPENDS ${OUT_FILE})

	add_custom_command(OUTPUT ${OUT_FILE}
		COMMAND ${CMAKE_CURRENT_BINARY_DIR}/${GEN_DOC_SCRIPT} ${DOC_TYPE} ${DOC_FILE}
		DEPENDS ${GEN_TEXI_FILES_LIST_TARGETS} ${GEN_TEXI_FILES_LIST} ${GEN_DOC_SCRIPT}.in
		)

	add_dependencies("doc_${DOC_TYPE}_target"
		avrdude
		)

endforeach()

# =====================================
# Tarball
# =====================================

if(SCRIPT_DEBUG)
	set(TAR_EXTRA_ARGS
		--verbose
	)
endif()

add_custom_command(OUTPUT ${TAR_BALL}
	COMMAND ${TAR_BIN}
		--file=${TAR_BALL}
		--create
		--gzip
		--transform="s@.*@${TAR_BALL_PREFIX_DIR}/&@"
		${TAR_EXTRA_ARGS}
		${DOC_FILES_LIST}
	DEPENDS "doc_target"
	)

add_custom_target("tarball_target"
	ALL
	DEPENDS ${TAR_BALL}
	)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${TAR_BALL}
	DESTINATION ${CMAKE_INSTALL_DATADIR}
	)

# =====================================
# Main Target
# =====================================

list(TRANSFORM DOC_TYPES_LIST PREPEND "doc_")
list(TRANSFORM DOC_TYPES_LIST APPEND "_target")
add_custom_target("doc_target"
	ALL
	DEPENDS ${DOC_TYPES_LIST}
	)
