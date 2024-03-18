
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-fPIC -s ERROR_ON_UNDEFINED_SYMBOLS=0 -s MAIN_MODULE -s WASM=1 -s FORCE_FILESYSTEM -s ASYNCIFY=1 -s INVOKE_RUN=0 -s WASM_BIGINT=1 -s MODULARIZE=1 -s EXPORT_KEEPALIVE=1 --bind -s \"EXPORTED_FUNCTIONS=['_startAvrdude','_main','_malloc']\" -s EXPORTED_RUNTIME_METHODS='[\"cwrap\", \"FS\", \"allocate\", \"intArrayFromString\"]'")
add_executable(avrdude
        main.cpp
        developer_opts.cpp
        developer_opts.h
        developer_opts_private.h
)
target_link_libraries(avrdude PUBLIC serial libavrdude)
target_include_directories(avrdude PRIVATE ${CMAKE_BINARY_DIR}/src ${CMAKE_SOURCE_DIR}/src)
# set compiler flags and linker flags

# copy the program.hex file and the index.html files to the build/test directory

add_custom_command(TARGET avrdude POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/test/index.html ${CMAKE_BINARY_DIR}/test
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/test/program.hex ${CMAKE_BINARY_DIR}/test
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/src/avrdude.conf ${CMAKE_BINARY_DIR}/test
)