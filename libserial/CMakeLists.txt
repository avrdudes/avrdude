

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -s ASYNCIFY -O3 --bind -s STANDALONE_WASM=1 -s WASM=1")



add_library(serial STATIC LibSerial.cpp)

# copy avrdude-worker.js to build directory
add_custom_command(TARGET serial POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
    ${CMAKE_CURRENT_SOURCE_DIR}/avrdude-worker.js
    ${CMAKE_CURRENT_BINARY_DIR}/avrdude-worker.js)
