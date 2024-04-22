set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -s ASYNCIFY -O3 --bind -s STANDALONE_WASM=1 -s WASM=1")

add_custom_target(worker
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
        COMMAND yarn install
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMAND npx rollup -c --format cjs -i ${CMAKE_CURRENT_SOURCE_DIR}/avrdude-worker.js -o ${CMAKE_CURRENT_BINARY_DIR}/avrdude-worker.js
)

add_library(serial STATIC LibSerial.c)

add_dependencies(serial worker)

# copy avrdude-worker.js to build directory

