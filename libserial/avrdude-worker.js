let port;
let opts;
let writer;
let reader;

let buffer = new Uint8Array([])
let onData
let available = new Promise(resolve => onData = resolve)
let continuousRead = null

const readPromise = (customReader) => new Promise(async () => {
    while (true) {
        const { value, done } = await customReader.read()
        if (done) break

        buffer = new Uint8Array([...buffer, ...value])
        onData()
    }
})



addEventListener('message', async msg => {
    const data = msg.data
    console.log(data)

    switch (data.type) {
        case 'write': {
            console.log('Write: ', data)
            await writer.write(data.data)
            break
        }
        case 'read': {
            const neededBytes = data.requiredBytes
            console.log('Read timeout: ', data.timeout)
            // await available
            // available = new Promise(resolve => onData = resolve)
            let dataBuffer = new Uint8Array([])

            while (dataBuffer.length < neededBytes) {
                console.log('Buffer: ', dataBuffer.length, 'Needed: ', neededBytes, "Buffer: ", buffer.length)
                if (buffer.length > 0) {
                    // only take the needed bytes
                    const bytesToTake = Math.min(neededBytes - dataBuffer.length, buffer.length)
                    dataBuffer = new Uint8Array([...dataBuffer, ...buffer.slice(0, bytesToTake)])
                    buffer = buffer.slice(bytesToTake)
                    continue
                }
                await available
                available = new Promise(resolve => onData = resolve)
            }

            postMessage({ type: 'read', result: dataBuffer })
            break
        }
        case 'clear-read-buffer': {
            const timeoutPromise = new Promise(resolve => setTimeout(resolve, data.timeout))
            // await available
            // available = new Promise(resolve => onData = resolve)
            let result = await Promise.race([timeoutPromise, available])
            available = new Promise(resolve => onData = resolve)
            buffer = new Uint8Array([])

            postMessage({ type: 'clear-read-buffer' })
            break
        }
        case 'init': {
            console.log('Init: ', data)
            port = (await navigator.serial.getPorts())[data.port]
            await port.open(data.options)
            opts = data.options
            writer = port.writable.getWriter()
            reader = port.readable.getReader()
            continuousRead = readPromise(reader)
            postMessage({ type: 'ready' })
            break
        } case 'close': {
            await port.close()
            break
        }
        default: {
            console.error('Unknown message type', data.type)
            break
        }
    }
})