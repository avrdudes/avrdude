#ifndef AVRDUDE_LIBSERIAL_H
#define AVRDUDE_LIBSERIAL_H


#include <stdbool.h>
#include <stddef.h>

int serialPortOpen(int baudRate);
void setDtrRts(bool is_on);
void serialPortDrain(int timeout);
void serialPortWrite(const unsigned char *buf, size_t len);
int serialPortRecv(unsigned char *buf, size_t len, int timeoutMs);
void serialPortClose();

#endif // AVRDUDE_LIBSERIAL_H