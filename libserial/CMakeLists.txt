
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -s \"EXPORTED_FUNCTIONS=['_dataCallback']\" --bind -s MODULARIZE=1 -s STANDALONE_WASM=1 -s WASM=1")
# verbose output
add_library(serial STATIC LibSerial.cpp)
# add options for the compiler/linker

