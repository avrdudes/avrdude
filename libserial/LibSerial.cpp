#include "LibSerial.h"
#include <sstream>
#include <emscripten.h>
#include <emscripten/val.h>
#include <map>
#include <iostream>
#include <emscripten/bind.h>
#include <iomanip>

using namespace emscripten;

std::vector<int8_t> readBuffer;

extern "C" {
EMSCRIPTEN_KEEPALIVE
void dataCallback(int8_t *array, int length) {
    std::vector<int8_t> data(array, array + length);
    readBuffer.insert(readBuffer.end(), data.begin(), data.end());
}
}

namespace {


//    EM_JS(EM_VAL, start_callback, (), {
//        const port = window.activePort;
//        const abortController = new AbortController();
//
//        const outputStream = new WritableStream({
//                write: async (chunk) => {
//                    // Chunk is a Uint8Array
//                    console.log(chunk);
//                    // Allocate memory and copy the Uint8Array to the Emscripten heap
//                    const ptr = window.funcs._malloc(chunk.length * Uint8Array.BYTES_PER_ELEMENT);
//                    window.funcs.HEAPU8.set(chunk, ptr);
//
//                    // Call the C++ function with the pointer and the length of the array
//                    window.funcs._dataCallback(ptr, chunk.length);
//                },
//        });
//
//        const pipePromise = port.readable.pipeTo(outputStream, { signal: abortController.signal });
//
//
//
//        return abortController;
//    });

    EM_ASYNC_JS(void, read_data, (int timeoutMs), {
        const reader = window.activePort.readable.getReader();
        async function receive() {
            const { value } = await reader.read();
            return value;
        }

        async function timeout(timeoutMs) {
            await new Promise(resolve => setTimeout(resolve, timeoutMs));
            return "timeout";
        }

        var returnBuffer = new Uint8Array();
        while (true) {
            let result = await Promise.race([receive(), timeout(timeoutMs)]);

            if (result instanceof Uint8Array) {
                // check if it is twice the same data so check if the first half is the same as the second half if so remove the second half
                let firstHalf = result.slice(0, result.length / 2);
                let secondHalf = result.slice(result.length / 2, result.length);
                if (firstHalf.every((value, index) => value === secondHalf[index])) {
                    result = firstHalf;
                }

                //console.log("Received: ", result);
                // convert data into an readable string formated like this 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15
                const printResult = Array.from(result).join(",");
                window["avrdudeLog"] = [...window["avrdudeLog"], "Received: " + printResult];
                const ptr = window.funcs._malloc(result.length * Uint8Array.BYTES_PER_ELEMENT);
                window.funcs.HEAPU8.set(result, ptr);

                // Call the C++ function with the pointer and the length of the array
                window.funcs._dataCallback(ptr, result.length);
                break;
            } else {
                window["avrdudeLog"] = [...window["avrdudeLog"], "Timeout"];
                break;
            }
        }
        reader.releaseLock();
        return;
    });

    // clang-format off
    // get serial options as an EM_VAL
    EM_ASYNC_JS(EM_VAL, open_serial_port, (EM_VAL cppSerialOpts), {
        let serialOpts = Emval.toValue(cppSerialOpts);
        serialOpts.bufferSize = 1024*2;
        let port;
        if (!window.activePort) {
            port = await navigator.serial.requestPort();
            await port.open(serialOpts);
        } else {
            port = window.activePort;
            // check if open otherwise open port
            if (!port.readable || !port.writable) {
                await port.open(serialOpts);
            }
        }
        // reset the arduino by setting the DTR signal at baud rate 1200
        await port.close();
        await port.open({baudRate: 1200});
        await port.setSignals({dataTerminalReady: true, requestToSend: true});
        await new Promise(resolve => setTimeout(resolve, 100));
        await port.setSignals({dataTerminalReady: false, requestToSend: false});
        await port.close();
        // open the port with the correct baud rate
        await port.open(serialOpts);
        // set up an activePort variable on the window, so it can be accessed from everywhere
        window.activePort = port;
        window.writeStream = port.writable.getWriter();
        window["avrdudeLog"] = [];
        return port;
    });

    EM_ASYNC_JS(void, set_serial_port_options, (EM_VAL port, EM_VAL serialOpts), {
        if (port.readable && port.writable) {
            await port.close();
        }
        await port.open(serialOpts);
    });

    EM_ASYNC_JS(void, close_serial_port, (EM_VAL port), {
        window.writeStream.releaseLock();
        await port.close();
    });

    EM_ASYNC_JS(bool, is_serial_port_open, (EM_VAL port), {
        return port.readable && port.writable;
    });

    EM_ASYNC_JS(void, set_dts_rts, (bool is_on), {
        await window.activePort.setSignals({dataTerminalReady: is_on, requestToSend: is_on});
    });


    EM_ASYNC_JS(void, write_data, (EM_VAL data), {
        data = new Uint8Array(Emval.toValue(data));
        //console.log("Sending: ", data);
        // convert data into an readable string formated like this 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15
        const printData = Array.from(data).join(",");
        window["avrdudeLog"] = [...window["avrdudeLog"], "Sending: " + printData];
        const port = window.activePort;
        await window.writeStream.ready;
        await window.writeStream.write(data);
        await window.writeStream.ready;
    });

    val generateSerialOptions(const std::map<std::string, int>& serialOptions) {
        val&& serialOpts = val::object();
        for (auto& [key, value] : serialOptions) {
            if (key == "flowControl" || key == "parity" || key == "stopBits") {
                // Handle specific cases for flowControl, parity, and stopBits if needed
                // ...
            } else {
                serialOpts.set(key, value);
            }
        }
        return serialOpts;
    }

    void setSerialOptions(EM_VAL port ,const std::map<std::string, int>& serialOptions)
    {

        set_serial_port_options(port, generateSerialOptions(serialOptions).as_handle());
    }

}

int serialPortOpen(int baudRate) {
    val opts = generateSerialOptions({{"baudRate", baudRate}});
    if (opts.isUndefined() || opts.isNull()) {
        std::cerr << "Invalid options for serialPortOpen" << std::endl;
        return -1;
    }
    open_serial_port(opts.as_handle());
    return 0;
}

void setDtrRts(bool is_on) {
    set_dts_rts(is_on);
}

void serialPortDrain(int timeout) {
    readBuffer.clear();
}

void serialPortWrite(const unsigned char *buf, size_t len) {
    std::vector<unsigned char> data(buf, buf + len);
    write_data(val(typed_memory_view(data.size(), data.data())).as_handle());
    emscripten_sleep(300);
}

int serialPortRecv(unsigned char *buf, size_t len, int timeoutMs) {
    std::vector<unsigned char> data = {};
    data.reserve(len);
    // check if there is leftover data from previous reads
    if (!readBuffer.empty()) {
        if (readBuffer.size() >= len) {
            data = std::vector<unsigned char>(readBuffer.begin(), readBuffer.begin() + len);
            readBuffer.erase(readBuffer.begin(), readBuffer.begin() + len);
            std::copy(data.begin(), data.end(), buf);
            return 0;
        } else {
            data = std::vector<unsigned char>(readBuffer.begin(), readBuffer.end());
            readBuffer.clear();
        }
    }
    read_data(timeoutMs);
    if (!readBuffer.empty()) {
        // check how much data is needed and add that much to the buffer
        if (readBuffer.size() >= len) {
            data = std::vector<unsigned char>(readBuffer.begin(), readBuffer.begin() + len);
            readBuffer.erase(readBuffer.begin(), readBuffer.begin() + len);
        } else {
            data = std::vector<unsigned char>(readBuffer.begin(), readBuffer.end());
            readBuffer.clear();
        }
    }
    if (data.empty()) {
        return -1;
    } else {
        std::copy(data.begin(), data.end(), buf);
    }
    return 0;
}