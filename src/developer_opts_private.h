/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2022, Stefan Rueger <smr@theblueorange.space>
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

#define __partout(fmt, component) \
  dev_part_strct_entry(tsv, ".pt", p->desc, NULL, #component, dev_sprintf(fmt, p->component))

#define __if_partout(cmp, fmt, component) ({ \
  if(!base || cmp(base->component, p->component)) \
    dev_part_strct_entry(tsv, ".pt", p->desc, NULL, #component, dev_sprintf(fmt, p->component)); \
})

#define __if_n_partout(cmp, n, fmt, component) ({ \
  if(!base || cmp(base->component, p->component, n)) \
    dev_part_strct_entry(tsv, ".pt", p->desc, NULL, #component, dev_sprintf(fmt, p->component)); \
})

#define __partout_str(result, component) \
  dev_part_strct_entry(tsv, ".pt", p->desc, NULL, #component, result)

#define __if_partout_str(cmp, result, component) ({ \
  if(!base || cmp(base->component, p->component)) \
    dev_part_strct_entry(tsv, ".pt", p->desc, NULL, #component, result); \
})

#define __if_n_partout_str(cmp, n, result, component) ({ \
  if(!base || cmp(base->component, p->component, n)) \
    dev_part_strct_entry(tsv, ".pt", p->desc, NULL, #component, result); \
})


#define __memout(fmt, component) \
  dev_part_strct_entry(tsv, ".ptmm", p->desc, m->desc, #component, dev_sprintf(fmt, m->component))

#define __if_memout(cmp, fmt, component) ({ \
  if(!bm || cmp(bm->component, m->component)) \
    dev_part_strct_entry(tsv, ".ptmm", p->desc, m->desc, #component, dev_sprintf(fmt, m->component)); \
})

#define __memout_str(result, component) \
  dev_part_strct_entry(tsv, ".ptmm", p->desc, m->desc, #component, result)

#define __if_n_memout_str(cmp, n, result, component) ({ \
  if(!bm || cmp(bm->component, m->component, n)) \
    dev_part_strct_entry(tsv, ".ptmm", p->desc, m->desc, #component, result); \
})

#define __memout_yn(component) \
  dev_part_strct_entry(tsv, ".ptmm", p->desc, m->desc, #component, strdup(m->component? "yes": "no"))

#define __if_memout_yn(component) ({ \
  if(!bm || bm->component != m->component) \
    dev_part_strct_entry(tsv, ".ptmm", p->desc, m->desc, #component, strdup(m->component? "yes": "no")); \
})

#define __flagout(mask, name) \
  __partout_str(strdup(p->flags & (mask)? "yes": "no"), name)

#define __if_flagout(mask, name) ({ \
  if(!base || (base->flags & (mask)) != (p->flags & (mask))) \
    __partout_str(strdup(p->flags & (mask)? "yes": "no"), name); \
})

#define __cmderr(result, component) \
  dev_part_strct_entry(tsv, ".cmderr", p->desc, m->desc, #component, result)

#endif
