#include "LibSerial.h"
#include <sstream>
#include <emscripten.h>
#include <emscripten/val.h>
#include <map>
#include <iostream>

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

    EM_ASYNC_JS(void, write_data, (EM_VAL data), {
        const jsData = Emval.toValue(data);
        window.avrDudeWorker.postMessage({ type: 'write', data: jsData });
    });


    EM_ASYNC_JS(void, clear_read_buffer, (int timeoutMs), {
        window.avrdudeLog = [...window.avrdudeLog, "Clearing read buffer"];
        window.avrDudeWorker.postMessage({ type: 'clear-read-buffer', timeout: timeoutMs });
        await new Promise(resolve => {
            window.avrDudeWorker.onmessage = (event) => {
                resolve();
            };
        });
        window.avrdudeLog = [...window.avrdudeLog, "Read buffer cleared"];
    });

    EM_ASYNC_JS(void, read_data, (int timeoutMs, int length), {
        window.avrDudeWorker.postMessage({ type: 'read', timeout: timeoutMs, requiredBytes: length });
        const data = await new Promise((resolve, _) => {
            window.avrDudeWorker.onmessage = (event) => {
                if (event.data.type == "read") {
                    resolve(event.data);
                }
            };
        });
        const result = data.result;

        if (result instanceof Uint8Array && result.length > 0) {
            //console.log("Received: ", result);
            // convert data into an readable string formated like this 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15
            const printResult = Array.from(result).join(",");
            window["avrdudeLog"] = [...window["avrdudeLog"], "Received: " + printResult];
            const ptr = window.funcs._malloc(result.length * Uint8Array.BYTES_PER_ELEMENT);
            window.funcs.HEAPU8.set(result, ptr);

            // Call the C++ function with the pointer and the length of the array
            window.funcs._dataCallback(ptr, result.length);
        } else {
            window["avrdudeLog"] = [...window["avrdudeLog"], "Timeout"];
        }
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
        await port.setSignals({dataTerminalReady: false});
        await new Promise(resolve => setTimeout(resolve, 100));
        await port.setSignals({dataTerminalReady: true});
        await port.close();
        const worker = new Worker('avrdude-worker.js', { type: 'module' });

        // check which port in navigator.serial.getPorts matches the activePort
        let portNumber = 0;
        const ports = await navigator.serial.getPorts();
        for (let i = 0; i < ports.length; i++) {
            const port = ports[i];
            if (port === window.activePort) {
                portNumber = i;
                break;
            }
        }

        worker.postMessage({ type: 'init', options: serialOpts, port: portNumber });
        await new Promise(resolve => {
            worker.onmessage = (event) => {
                resolve();
            };
        });
        // open the port with the correct baud rate
        window.avrDudeWorker = worker;
        window.activePort = port;
        window["avrdudeLog"] = [];
        return port;
    });

    EM_ASYNC_JS(void, close_serial_port, (), {
        window.avrDudeWorker.postMessage({ type: 'close' });
        await new Promise(resolve => {
            window.avrDudeWorker.onmessage = (event) => {
                resolve();
            };
        });
        window.activePort = null;
        window.avrDudeWorker.terminate();
    });

    EM_ASYNC_JS(bool, is_serial_port_open, (EM_VAL port), {
        return port.readable && port.writable;
    });

    EM_ASYNC_JS(void, set_dts_rts, (bool is_on), {
        await window.activePort.setSignals({dataTerminalReady: is_on, requestToSend: is_on});
    });

    val generateSerialOptions(const std::map<std::string, int>& serialOptions) {
        val&& serialOpts = val::object();
        for (auto& [key, value] : serialOptions) {
            if (key == "flowControl" || key == "parity" || key == "stopBits") {
                printf("Not implemented yet\n");
                printf("Key: %s, Value: %d\n", key.c_str(), value);
                // Handle specific cases for flowControl, parity, and stopBits if needed
                // ...
            } else {
                serialOpts.set(key, value);
            }
        }
        return serialOpts;
    }

}

worker_handle worker;

int serialPortOpen(int baudRate) {
    val opts = generateSerialOptions({{"baudRate", baudRate}});
    if (opts.isUndefined() || opts.isNull()) {
        std::cerr << "Invalid options for serialPortOpen" << std::endl;
        return -1;
    }
    open_serial_port(opts.as_handle());

    worker = emscripten_create_worker("avrdude-worker.js");

    return 0;
}

void serialPortClose() {
    close_serial_port();
}

void setDtrRts(bool is_on) {
    set_dts_rts(is_on);
}

void serialPortDrain(int timeout) {
    // print the length of the timeout

    readBuffer.clear();
    clear_read_buffer(timeout);
}

void serialPortWrite(const unsigned char *buf, size_t len) {
    std::vector<unsigned char> data(buf, buf + len);
    emscripten::val js_data = emscripten::val(emscripten::typed_memory_view(data.size(), data.data()));
    js_data = emscripten::val::global("Uint8Array").new_(js_data);

    std::string printData;
    for (size_t i = 0; i < data.size(); ++i) {
        if (i > 0)
            printData += ",";
        printData += std::to_string(data[i]);
    }

    emscripten::val avrdudeLog = emscripten::val::global("window")["avrdudeLog"];
    avrdudeLog.call<void>("push", "Sending: " + printData);

    write_data(js_data.as_handle());
}

int serialPortRecv(unsigned char *buf, size_t len, int timeoutMs) {
    std::vector<unsigned char> data = {};
    data.reserve(len);
    read_data(timeoutMs, len);
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
