#pragma once

#include "pgm.h"
#include "avrpart.h"

//int avrftdi_tpi_write_byte(PROGRAMMER * pgm, unsigned char byte);
//int avrftdi_tpi_read_byte(PROGRAMMER * pgm, unsigned char * byte);
int avrftdi_tpi_program_enable(PROGRAMMER * pgm, AVRPART * p);
int avrftdi_tpi_chip_erase(PROGRAMMER * pgm, AVRPART * p);
int avrftdi_cmd_tpi(PROGRAMMER * pgm, unsigned char cmd[], int cmd_len,
		unsigned char res[], int res_len);
int avrftdi_tpi_initialize(PROGRAMMER * pgm, AVRPART * p);


