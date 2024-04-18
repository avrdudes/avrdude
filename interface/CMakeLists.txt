set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-fPIC -O3 -s ENVIRONMENT=web -s ERROR_ON_UNDEFINED_SYMBOLS=0 -s WASM=1 -s FORCE_FILESYSTEM -s ASYNCIFY=1 -s INVOKE_RUN=0 -s WASM_BIGINT=1 -s MODULARIZE=1 -s \"EXPORTED_FUNCTIONS=['_startAvrdude','_malloc']\" --bind -s EXPORTED_RUNTIME_METHODS='[\"cwrap\", \"writeStringToMemory\", \"FS\", \"allocate\"]'")

add_executable(avrdude
        ../interface/main.cpp
        ../interface/developer_opts.cpp
        ../interface/developer_opts.h
        ../interface/developer_opts_private.h
)
target_link_libraries(avrdude PUBLIC serial libavrdude)
target_include_directories(avrdude PRIVATE ${CMAKE_BINARY_DIR}/src ${CMAKE_SOURCE_DIR}/src)
