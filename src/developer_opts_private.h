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

#ifndef DEV_INFO
#define DEV_INFO    MSG_INFO
#endif

#ifndef DEV_NOTICE
#define DEV_NOTICE  MSG_NOTICE
#endif

#ifndef DEV_NOTICE
#define DEV_NOTICE2  MSG_NOTICE2
#endif

#define dev_info(...)    avrdude_message(DEV_INFO,    __VA_ARGS__)
#define dev_notice(...)  avrdude_message(DEV_NOTICE,  __VA_ARGS__)
#define dev_notice2(...) avrdude_message(DEV_NOTICE2, __VA_ARGS__)

#define dev_partout(fmt, component) (dot? \
   dev_info(".part\t%s\t%s\t" fmt "\n", p->desc, #component, p->component): \
   dev_info("    %-19s = " fmt ";\n", #component, p->component))

#define dev_partout_str(result, component) (dot? \
   dev_info(".part\t%s\t%s\t%s\n", p->desc, #component, result): \
   dev_info("    %-19s = %s;\n", #component, result))

#define dev_memout(fmt, component) (dot? \
   dev_info(".pmem\t%s\t%s\t%s\t" fmt "\n", p->desc, m->desc, #component, m->component): \
   dev_info("        %-15s = " fmt ";\n", #component, m->component))

#define dev_memout_str(result, component) (dot? \
   dev_info(".pmem\t%s\t%s\t%s\t%s\n", p->desc, m->desc, #component, result): \
   dev_info("        %-15s = %s;\n", #component, result))

#define dev_memout_yn(component) (dot? \
   dev_info(".pmem\t%s\t%s\t%s\t%s\n", p->desc, m->desc, #component, m->component? "yes": "no"): \
   dev_info("        %-15s = %s;\n", #component, m->component? "yes": "no"))


#define dev_flagout(name, mask) dev_info("    %-19s = %s;\n", (name), p->flags & (mask)? "yes": "no")
