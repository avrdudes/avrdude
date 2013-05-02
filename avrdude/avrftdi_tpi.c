#include "ac_cfg.h"

#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include "pgm.h"
#include "avrpart.h"
#include "pindefs.h"
#include "tpi.h"
#include "usbasp.h"

#include "avrftdi_tpi.h"
#include "avrftdi_private.h"

#ifdef HAVE_LIBUSB_1_0
#ifdef HAVE_LIBFTDI1

#include <libusb-1.0/libusb.h>
#include <libftdi1/ftdi.h>

static void avrftdi_tpi_disable(PROGRAMMER *);

static void
avrftdi_debug_frame(uint16_t frame)
{
	static char bit_name[] = "IDLES01234567PSS";
	//static char bit_name[] = "SSP76543210SELDI";
	char line0[34], line1[34], line2[34];
	int bit, pos;

	for(bit = 0; bit < 16; bit++)
	{
		pos = 16 - bit - 1;
		if(frame & (1 << pos))
		{
			line0[2*pos]  = '_';
			line0[2*pos+1] = ' ';
			
			line2[2*pos]  = ' ';
			line2[2*pos+1] = ' ';
		}
		else
		{
			line0[2*pos]  = ' ';
			line0[2*pos+1] = ' ';
			
			line2[2*pos]  = '-';
			line2[2*pos+1] = ' ';
		}
			
		line1[2*pos]  = bit_name[pos];
		line1[2*pos+1] = ' ';
			
	}

	line0[32] = 0;
	line1[32] = 0;
	line2[32] = 0;

	log_debug("%s\n", line0);
	log_debug("%s\n", line1);
	//log_debug("%s\n", line2);
}

int
avrftdi_tpi_initialize(PROGRAMMER * pgm, AVRPART * p)
{
	int ret;

	avrftdi_t* pdata = to_pdata(pgm);
	unsigned char buf[] = { MPSSE_DO_WRITE | MPSSE_WRITE_NEG | MPSSE_LSB, 0x01, 0x00, 0xff, 0xff };

	log_info("Using TPI interface\n");

	pgm->program_enable = avrftdi_tpi_program_enable;
	pgm->cmd_tpi = avrftdi_cmd_tpi;
	pgm->chip_erase = avrftdi_tpi_chip_erase;
	pgm->disable = avrftdi_tpi_disable;

	pgm->paged_load = NULL;
	pgm->paged_write = NULL;

	log_info("Setting /Reset pin low\n");
	pdata->set_pin(pgm, PIN_AVR_RESET, OFF);
	pdata->set_pin(pgm, PIN_AVR_SCK, OFF);
	pdata->set_pin(pgm, PIN_AVR_MOSI, ON);
	usleep(20 * 1000);

	pdata->set_pin(pgm, PIN_AVR_RESET, ON);
	/* worst case 128ms */
	usleep(2 * 128 * 1000);

	/*setting rst back to 0 */
	pdata->set_pin(pgm, PIN_AVR_RESET, OFF);
	/*wait at least 20ms bevor issuing spi commands to avr */
	usleep(20 * 1000);
	
	log_info("Sending 16 init clock cycles ... ");
	ret = ftdi_write_data(pdata->ftdic, buf, sizeof(buf));
	log_info("Done.\n");

	return ret;
}

#define TPI_BIT_PAR 0x2000

static uint16_t
tpi_byte2frame(uint8_t byte)
{
	uint16_t frame = 0xc00f;
	int parity = __builtin_popcount(byte) & 1;

	frame |= ((byte << 5) & 0x1fe0);

	if(parity)
		frame |= TPI_BIT_PAR;
	
	return frame;
}

static int
tpi_frame2byte(uint16_t frame, uint8_t * byte)
{
	/* drop partity + 2 stop bits */
	*byte = (frame >> 1) & 0xff;

	int parity = __builtin_popcount(*byte) & 1;
	int parity_rcvd = (frame & 0x200) ? 1 : 0;

	log_info("Recevied frame with payload 0x%02x and %s parity\n",
	         *byte, parity ? "odd" : "even");
	
	return parity != parity_rcvd;
}

#define TPI_FRAME_SIZE 2

static int
avrftdi_tpi_break(PROGRAMMER * pgm)
{
	unsigned char buffer[] = { MPSSE_DO_WRITE | MPSSE_WRITE_NEG | MPSSE_LSB, 1, 0, 0, 0 };
	E(ftdi_write_data(to_pdata(pgm)->ftdic, buffer, sizeof(buffer)) != sizeof(buffer), to_pdata(pgm)->ftdic);

	return 0;
}

static int
avrftdi_tpi_write_byte(PROGRAMMER * pgm, unsigned char byte)
{
	uint16_t frame;

	struct ftdi_context* ftdic = to_pdata(pgm)->ftdic;

	unsigned char buffer[] = { MPSSE_DO_WRITE | MPSSE_WRITE_NEG | MPSSE_LSB, 1, 0, 0, 0 };

	frame = tpi_byte2frame(byte);
	
	buffer[3] = frame & 0xff;
	buffer[4] = frame >> 8;
	
	log_debug("TPI frame: 0x%02x 0x%02x, data byte 0x%02x\n",
			buffer[6], buffer[7], byte);
	log_debug("FTDI raw data: 0x%02x 0x%02x 0x%02x  0x%02x 0x%02x\n",
			buffer[0], buffer[1], buffer[2], buffer[3], buffer[4] /*, buffer[5], buffer[6], buffer[7]*/);

	avrftdi_debug_frame(frame);
	
	E(ftdi_write_data(ftdic, buffer, sizeof(buffer)) != sizeof(buffer), ftdic);

	return 0;
}

static int
avrftdi_tpi_read_byte(PROGRAMMER * pgm, unsigned char * byte)
{
	uint16_t frame;
	
	int guard_bits = to_pdata(pgm)->guard_bits;
	int bytes = ((guard_bits + 7) / 8) + 2;
	int i = 0, n = 0;

	/* worst case size:
	 * - 128 guard bits
	 * - 2 default idle bits
	 * - 12 frame bits
	 * = 142 bits
	 */
	unsigned char buffer[18];

	if(bytes > sizeof(buffer))
		log_warn("Requested more bytes (%d) than available buffer space (%d)\n", bytes, sizeof(buffer));

	log_info("Guard bit size (incl. default idle bits) is %d\n", guard_bits);
	log_info("Reading %d bytes for guard bits + frame\n", bytes);

	/* set it high, so the PDI won't detect we're driving the line */
	to_pdata(pgm)->set_pin(pgm, PIN_AVR_MOSI, ON);

	buffer[0] = MPSSE_DO_READ | MPSSE_WRITE_NEG | MPSSE_LSB;
	buffer[1] = (bytes-1) & 0xff;
	buffer[2] = ((bytes-1) >> 8) & 0xff;
	buffer[3] = SEND_IMMEDIATE;

	log_info("Read request: 0x%02x 0x%02x 0x%02x 0x%02x\n",
			buffer[0], buffer[1], buffer[2], buffer[3]);

	ftdi_write_data(to_pdata(pgm)->ftdic, buffer, 4);

	memset(buffer, 0, sizeof(buffer));

	i = 0;
	do {
		n = ftdi_read_data(to_pdata(pgm)->ftdic, &buffer[i], bytes - i);
		E(n < 0, to_pdata(pgm)->ftdic);
		i += n;
	} while(i < bytes);

	/* dismiss at least (guard_bits / 8) bytes */
	i = guard_bits / 8;
	frame = buffer[i] | (buffer[i+1] << 8);

	/* now shift the rest of guard bits out */
	i = guard_bits - (i*8);
	frame >>= i;

	log_info("Received frame 0x%04x (LSB first)\n", frame);

	return tpi_frame2byte(frame, byte);
}

int
avrftdi_tpi_program_enable(PROGRAMMER * pgm, AVRPART * p)
{
	int retry;
	int err;
	int i;
	unsigned char byte = 0;

	log_info("TPI program enable\n");

	//TODO determine guard time:
	//-disable output possible -> guard time
	//-else: minimum guard time
	/* set guard time */
	//avrftdi_tpi_write_byte(pgm, TPI_OP_SSTCS(TPIPCR));
	//avrftdi_tpi_write_byte(pgm, TPIPCR_GT_2b);

	/* send SKEY */
  avrftdi_tpi_write_byte(pgm, TPI_CMD_SKEY);
	for(i = sizeof(tpi_skey) - 1; i >= 0; --i)
  	avrftdi_tpi_write_byte(pgm, tpi_skey[i]);

	/* check if device is ready */
  for(retry = 0; retry < 10; retry++)
  {
		log_info("Reading Identification register\n");
    avrftdi_tpi_write_byte(pgm, TPI_OP_SLDCS(TPIIR));
    err = avrftdi_tpi_read_byte(pgm, &byte);
		if(err || byte != 0x80)
		{
			log_warn("Error. Sending break.\n");
			avrftdi_tpi_break(pgm);
			avrftdi_tpi_break(pgm);
			continue;
		}

    log_info("Reading Status register\n");
		avrftdi_tpi_write_byte(pgm, TPI_OP_SLDCS(TPISR));
    err = avrftdi_tpi_read_byte(pgm, &byte);
		if(err || !(byte & TPISR_NVMEN))
		{
			log_warn("Error. Sending break.\n");
			avrftdi_tpi_break(pgm);
			avrftdi_tpi_break(pgm);
			continue;
		}
		
		return 0;
  }

	log_err("Error connecting to target.\n");
	return -1;
}

static int
avrftdi_tpi_nvm_waitbusy(PROGRAMMER * pgm)
{
	unsigned char byte;
	int err;
	int retry;

	for(retry = 50; retry > 0; retry--)
	{
		avrftdi_tpi_write_byte(pgm, TPI_OP_SIN(NVMCSR));
    err = avrftdi_tpi_read_byte(pgm, &byte);		
		if(err || (byte & NVMCSR_BSY))
			continue;
		return 0;
	}

	return -1;
}

int
avrftdi_cmd_tpi(PROGRAMMER * pgm, unsigned char cmd[], int cmd_len,
		unsigned char res[], int res_len)
{
	int i, err = 0;

	for(i = 0; i < cmd_len; i++)
	{
		err = avrftdi_tpi_write_byte(pgm, cmd[i]);
		if(err)
			return err;
	}

	for(i = 0; i < res_len; i++)
	{
		err = avrftdi_tpi_read_byte(pgm, &res[i]);
		if(err)
			return err;
	}

	return 0;
}

int
avrftdi_tpi_chip_erase(PROGRAMMER * pgm, AVRPART * p)
{
  /* Set PR to flash */
  avrftdi_tpi_write_byte(pgm, TPI_OP_SSTPR(0));
  avrftdi_tpi_write_byte(pgm, 0x01);
  avrftdi_tpi_write_byte(pgm, TPI_OP_SSTPR(1));
  avrftdi_tpi_write_byte(pgm, 0x40);
  /* select ERASE */
  avrftdi_tpi_write_byte(pgm, TPI_OP_SOUT(NVMCMD));
  avrftdi_tpi_write_byte(pgm, NVMCMD_CHIP_ERASE);
  /* dummy write */
  avrftdi_tpi_write_byte(pgm, TPI_OP_SST_INC);
  avrftdi_tpi_write_byte(pgm, 0x00);
  avrftdi_tpi_nvm_waitbusy(pgm);
  
  usleep(p->chip_erase_delay);
  pgm->initialize(pgm, p);

	return 0;
}

static void
avrftdi_tpi_disable(PROGRAMMER * pgm)
{
	log_info("Leaving Programming mode.\n");
	avrftdi_tpi_write_byte(pgm, TPI_OP_SSTCS(TPIPCR));
	avrftdi_tpi_write_byte(pgm, 0);
}

#else /*HAVE_LIBFTDI*/
#endif  /* HAVE_LIBFTDI */

#else /*HAVE_LIBUSB*/

#endif /*HAVE_LIBUSB*/

