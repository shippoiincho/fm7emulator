// Dummy ROM file fot prebuild
// ROM Data                 at
// fbasic300.rom   31KiB    0x10070000
// subsys_c.rom    10KiB    0x10078000
// boot_bas.rom     1KiB    0x1007c000
// boot_dos.rom     1KiB    0x1007e000

//  BASIC ROM 0x8000 to 0xfc00
//  BOOT_BAS  0xfe00 to 0xfff0
//  BOOT_DOS  0xfe00 to 0xfff0
//  SUB_ROM   0xd800 to 0xffff (at subsystem)

#define ROMBASE 0x10070000u

uint8_t *fm7mainrom=(uint8_t *)(ROMBASE);
uint8_t *fm7subrom =(uint8_t *)(ROMBASE+0x8000);
uint8_t *fm7bootbas=(uint8_t *)(ROMBASE+0xc000);
uint8_t *fm7bootdos=(uint8_t *)(ROMBASE+0xe000);

// uint8_t *fm7kanji  =(uint8_t *)(ROMBASE-0x20000);