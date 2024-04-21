#include "LibSerial.h"
#include <emscripten.h>

unsigned char *readBuffer;
size_t readBufferLen;


void dataCallback(const unsigned char* array, int length) {
//    std::vector<int8_t> data(array, array + length);
//    readBuffer.insert(readBuffer.end(), data.begin(), data.end());
    // add the data to the readBuffer
    for (size_t i = 0; i < length; ++i) {
        readBuffer[readBufferLen++] = array[i];
    }
}




EM_JS(void, write_data, (unsigned char* buf, int len), {
    var jsBuffer = new Uint8Array(Module.HEAPU8.buffer, buf, len);
    window.avrDudeWorker.postMessage({ type: 'write', data: jsBuffer });
    window.avrdudeLog = [...window.avrdudeLog, "Sending: " + Array.from(jsBuffer).join(",")];
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
EM_ASYNC_JS(void, open_serial_port, (int baudRateInt), {
    const serialOpts = {
        baudRate: baudRateInt,
        bufferSize: 1024*2
    };
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
    if (navigator.serial) {
       const ports = await navigator.serial.getPorts();
       for (let i = 0; i < ports.length; i++) {
          const port = ports[i];
          if (port === window.activePort) {
             portNumber = i;
             break;
          }
       }
    }

    worker.postMessage({ type: 'init', options: serialOpts, port: portNumber });
console.log("Port opened");
    await new Promise(resolve => {
        worker.onmessage = (event) => {
            resolve();
        };
    });
    console.log("Port opened");

    // open the port with the correct baud rate
    window.avrDudeWorker = worker;
    window.activePort = port;
    window["avrdudeLog"] = [];
    console.log("Port opened");
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

EM_ASYNC_JS(bool, is_serial_port_open, (), {
    const port = window.activePort;
    return port.readable && port.writable;
});

EM_ASYNC_JS(void, set_dts_rts, (bool is_on), {
    await window.activePort.setSignals({dataTerminalReady: is_on, requestToSend: is_on});
});

#include <stdio.h>

int serialPortOpen(int baudRate) {
    printf("Opening serial port with baud rate: %d\n", baudRate);
    open_serial_port(baudRate);
    printf("Serial port opened\n");
    return 0;
}

void serialPortClose() {
    close_serial_port();
}

void setDtrRts(bool is_on) {
    set_dts_rts(is_on);
}

void serialPortDrain(int timeout) {
    readBufferLen = 0;
    clear_read_buffer(timeout);
}

void serialPortWrite(const unsigned char *buf, size_t len) {
//    std::vector<unsigned char> data(buf, buf + len);
//    emscripten::val js_data = emscripten::val(emscripten::typed_memory_view(data.size(), data.data()));
//    js_data = emscripten::val::global("Uint8Array").new_(js_data);
//
//    std::string printData;
//    for (size_t i = 0; i < data.size(); ++i) {
//        if (i > 0)
//            printData += ",";
//        printData += std::to_string(data[i]);
//    }
//
//    emscripten::val avrdudeLog = emscripten::val::global("window")["avrdudeLog"];
//    avrdudeLog.call<void>("push", "Sending: " + printData);
//
//    write_data(js_data.as_handle());
    write_data((unsigned char*)buf, (int)len);
}

int serialPortRecv(unsigned char *buf, size_t len, int timeoutMs) {
    read_data(timeoutMs, len);
    size_t processed = 0;
    if (readBufferLen != 0) {
        // check how much data is needed and add that much to the buffer
        if (readBufferLen >= len) {
//            data = std::vector<unsigned char>(readBuffer.begin(), readBuffer.begin() + len);
//            readBuffer.erase(readBuffer.begin(), readBuffer.begin() + len);
            for (size_t i = 0; i < len; ++i) {
                buf[i] = readBuffer[i];
                processed++;
            }
            readBufferLen -= len;

        } else {
//            data = std::vector<unsigned char>(readBuffer.begin(), readBuffer.end());
//            readBuffer.clear();
            for (size_t i = 0; i < readBufferLen; ++i) {
                buf[i] = readBuffer[i];
                processed++;
            }
            readBufferLen = 0;
        }
    }
    if (processed == 0) {
        return -1;
    }
    return 0;
}
