/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2022 Stefan Rueger <stefan.rueger@urclocks.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef developer_opts_private_h
#define developer_opts_private_h

#define DEV_SPI_EN_CE_SIG     1
#define DEV_SPI_PROGMEM       2
#define DEV_SPI_PROGMEM_PAGED 4
#define DEV_SPI_LOAD_EXT_ADDR 8
#define DEV_SPI_EEPROM       16
#define DEV_SPI_EEPROM_PAGED 32
#define DEV_SPI_LOCK         64
#define DEV_SPI_CALIBRATION 128
#define DEV_SPI_LFUSE       256
#define DEV_SPI_HFUSE       512
#define DEV_SPI_EFUSE      1024


static int dev_message(int msglvl, const char *fmt, ...);


#ifndef DEV_INFO
#define DEV_INFO    MSG_INFO
#endif

#ifndef DEV_NOTICE
#define DEV_NOTICE  MSG_NOTICE
#endif

#ifndef DEV_NOTICE
#define DEV_NOTICE2  MSG_NOTICE2
#endif

#define dev_info(...)    dev_message(DEV_INFO,    __VA_ARGS__)
#define dev_notice(...)  dev_message(DEV_NOTICE,  __VA_ARGS__)
#define dev_notice2(...) dev_message(DEV_NOTICE2, __VA_ARGS__)

#define _pgmout(fmt, component) \
  dev_part_strct_entry(tsv, ".prog", id, NULL, #component, dev_sprintf(fmt, pgm->component), pgm->comments)

#define _pgmout_fmt(name, fmt, what) \
  dev_part_strct_entry(tsv, ".prog", id, NULL, name, dev_sprintf(fmt, what), pgm->comments)

#define _if_pgmout(cmp, fmt, component) do { \
  if(!base || cmp(base->component, pgm->component)) \
    dev_part_strct_entry(tsv, ".prog", id, NULL, #component, dev_sprintf(fmt, pgm->component), pgm->comments); \
} while(0)

#define _if_pgmout_bool(component) do { \
  if(!base || !!base->component != !!pgm->component) \
    dev_part_strct_entry(tsv, ".prog", id, NULL, #component, dev_sprintf("%s", pgm->component? "true": "false"), pgm->comments); \
} while(0)

// Result must be a malloc'd string
#define _if_pgmout_str(cmp, result, component) do { \
  if(!base || cmp(base->component, pgm->component)) \
    dev_part_strct_entry(tsv, ".prog", id, NULL, #component, result, pgm->comments); \
} while(0)


#define _partout(fmt, component) \
  dev_part_strct_entry(tsv, ".pt", p->desc, NULL, #component, dev_sprintf(fmt, p->component), p->comments)

#define _if_partout(cmp, fmt, component) do { \
  if(!base || cmp(base->component, p->component)) \
    dev_part_strct_entry(tsv, ".pt", p->desc, NULL, #component, dev_sprintf(fmt, p->component), p->comments); \
} while(0)

#define _if_partout_bool(component) do { \
  if(!base || !!base->component != !!p->component) \
    dev_part_strct_entry(tsv, ".pt", p->desc, NULL, #component, dev_sprintf("%s", p->component? "true": "false"), p->comments); \
} while(0)

#define _if_n_partout(cmp, n, fmt, component) do { \
  if(!base || cmp(base->component, p->component, n)) \
    dev_part_strct_entry(tsv, ".pt", p->desc, NULL, #component, dev_sprintf(fmt, p->component), p->comments); \
} while(0)

// Result must be a malloc'd string
#define _partout_str(result, component) \
  dev_part_strct_entry(tsv, ".pt", p->desc, NULL, #component, result, p->comments)

// Result must be a malloc'd string
#define _if_partout_str(cmp, result, component) do { \
  if(!base || cmp(base->component, p->component)) \
    dev_part_strct_entry(tsv, ".pt", p->desc, NULL, #component, result, p->comments); \
} while(0)

// Result must be a malloc'd string
#define _if_n_partout_str(cmp, n, result, component) do { \
  if(!base || cmp(base->component, p->component, n)) \
    dev_part_strct_entry(tsv, ".pt", p->desc, NULL, #component, result, p->comments); \
} while(0)


#define _memout(fmt, component) \
  dev_part_strct_entry(tsv, ".ptmm", p->desc, m->desc, #component, dev_sprintf(fmt, m->component), m->comments)

#define _if_memout(cmp, fmt, component) do { \
  if(!bm || cmp(bm->component, m->component)) \
    dev_part_strct_entry(tsv, ".ptmm", p->desc, m->desc, #component, dev_sprintf(fmt, m->component), m->comments); \
} while(0)

// Result must be a malloc'd string
#define _memout_str(result, component) \
  dev_part_strct_entry(tsv, ".ptmm", p->desc, m->desc, #component, result, m->comments)

// Result must be a malloc'd string
#define _if_n_memout_str(cmp, n, result, component) do { \
  if(!bm || cmp(bm->component, m->component, n)) \
    dev_part_strct_entry(tsv, ".ptmm", p->desc, m->desc, #component, result, m->comments); \
} while(0)

#define _memout_yn(component) \
  dev_part_strct_entry(tsv, ".ptmm", p->desc, m->desc, #component, cfg_strdup("_memout_yn()", m->component? "yes": "no"), m->comments)

#define _if_memout_bool(component) do { \
  if(!bm || !!bm->component != !!m->component) \
    dev_part_strct_entry(tsv, ".ptmm", p->desc, m->desc, #component, dev_sprintf("%s", m->component? "true": "false"), m->comments); \
} while(0)

#define _if_memout_yn(component) do { \
  if(!bm || bm->component != m->component) \
    dev_part_strct_entry(tsv, ".ptmm", p->desc, m->desc, #component, cfg_strdup("_if_memout_yn()", m->component? "yes": "no"), m->comments); \
} while(0)

#define _flagout(mask, name) \
  _partout_str(cfg_strdup("_flagout()", p->flags & (mask)? "yes": "no"), name)

#define _if_flagout(mask, name) do { \
  if(!base || (base->flags & (mask)) != (p->flags & (mask))) \
    _partout_str(cfg_strdup("_if_flagout()", p->flags & (mask)? "yes": "no"), name); \
} while(0)

// Result must be a malloc'd string
#define _cmderr(result, component) \
  dev_part_strct_entry(tsv, ".cmderr", p->desc, m->desc, #component, result, NULL)

#endif
