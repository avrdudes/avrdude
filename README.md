# AVRDUDE

## Documentation

Documentation for current and previous releases is [on Github Pages](https://avrdudes.github.io/avrdude/).

## Using AVRDUDE

You can look at test/index.html for an exmample to embed avrdude in your webpage!

## Building

### Enviroment Setup

Install emsdk using this [guide](https://emscripten.org/docs/getting_started/downloads.html)

Then you can run: 

`cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=$EMSDK/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake`

to configure your CMake project.

### Building

To build everything use:

`cmake --build build --target test`

### Testing

Congrats! You've build your own version, in your build dir under test you can run vite and open that link.

You should be able to upload an example blink program at PWM pin 3.
