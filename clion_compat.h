#ifdef __CLION_IDE__

#define NULL nullptr

#undef sleep_bod_disable
#define sleep_bod_disable()

#undef pgm_read_byte
#define pgm_read_byte(addr) (*(const uint8_t *)(addr))

#undef pgm_read_word
#define pgm_read_word(addr) (*(const uint16_t *)(addr))
#endif

