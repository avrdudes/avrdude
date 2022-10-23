#pragma once

//int avrftdi_tpi_write_byte(PROGRAMMER *pgm, unsigned char byte);
//int avrftdi_tpi_read_byte(PROGRAMMER *pgm, unsigned char * byte);
int avrftdi_cmd_tpi(const PROGRAMMER *pgm, const unsigned char *cmd, int cmd_len,
		unsigned char *res, int res_len);
int avrftdi_tpi_initialize(const PROGRAMMER *pgm, const AVRPART *p);
void avrftdi_tpi_initpgm(PROGRAMMER *pgm);


