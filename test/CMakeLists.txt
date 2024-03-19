

# set compiler flags and linker flags

# copy the program.hex file and the index.html files to the build/test directory

add_custom_target(test
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/interface/avrdude.js ${CMAKE_BINARY_DIR}/test
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/interface/avrdude.wasm ${CMAKE_BINARY_DIR}/test
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/test/index.html ${CMAKE_BINARY_DIR}/test
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/test/nanoevery.html ${CMAKE_BINARY_DIR}/test
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/test/program.hex ${CMAKE_BINARY_DIR}/test
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/src/avrdude.conf ${CMAKE_BINARY_DIR}/test
        DEPENDS avrdude
)