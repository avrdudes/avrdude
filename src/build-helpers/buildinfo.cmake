function(buildinfo_setup)
  message(STATUS "BEGIN buildinfo_setup")
  message(STATUS "END buildinfo_setup")
endfunction()

function(buildinfo_item KEY VAL)
  cmake_parse_arguments(arg "" "" "" ${ARGN})
  message(STATUS "BEGIN buildinfo_item")
  message(STATUS "  K,V: ${KEY}, ${VAL}")
  message(STATUS "END buildinfo_item")
endfunction()

function(buildinfo_output)
  message(STATUS "BEGIN buildinfo_output")
  message(STATUS "END buildinfo_output")
endfunction()

buildinfo_setup()
