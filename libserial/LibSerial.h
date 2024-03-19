#ifndef AVRDUDE_LIBSERIAL_H
#define AVRDUDE_LIBSERIAL_H

#include <cstddef>
#include <emscripten/val.h>

extern "C" void dataCallback(int8_t *array, int length);
int serialPortOpen(int baudRate);
void setDtrRts(bool is_on);
void serialPortDrain(int timeout);
void serialPortWrite(const unsigned char *buf, size_t len);
int serialPortRecv(unsigned char *buf, size_t len, int timeoutMs);

#endif // AVRDUDE_LIBSERIAL_H