

# set compiler flags and linker flags

# copy the program.hex file and the index.html files to the build/test directory

add_custom_target(test
        # Build files
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/interface/avrdude.js ${CMAKE_BINARY_DIR}/test
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/interface/avrdude.worker.js ${CMAKE_BINARY_DIR}/test
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/interface/avrdude.wasm ${CMAKE_BINARY_DIR}/test
        # HTML files
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/test/index.html ${CMAKE_BINARY_DIR}/test
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/test/nanoevery.html ${CMAKE_BINARY_DIR}/test
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/test/atmega.html ${CMAKE_BINARY_DIR}/test
        # Hex files
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/test/uno.hex ${CMAKE_BINARY_DIR}/test
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/test/every.hex ${CMAKE_BINARY_DIR}/test
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/test/atmega.hex ${CMAKE_BINARY_DIR}/test
        # Configuration files
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/test/serve.json ${CMAKE_BINARY_DIR}/test
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/src/avrdude.conf ${CMAKE_BINARY_DIR}/test
        DEPENDS avrdude
)