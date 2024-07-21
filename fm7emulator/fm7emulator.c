//  Fujitsu Micro-7 emulator
//
//  GP0: HSYNC
//  GP1: VSYNC
//  GP2: Blue
//  GP3: Red
//  GP4: Green
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/sync.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"
#include "hardware/dma.h"
#include "hardware/uart.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/pwm.h"

#include "tusb.h"
#include "bsp/board.h"

#include "vga16_graphics.h"

#include "fm7rom.h"
#include "fm7keymap.h"
#include "mc6809.h"
#include "fm7misc.h"

#include "lfs.h"

#include "fm7test.h"

//#define USE_KANJI

#ifdef USE_KANJI
#include "fm7kanji.h"
#endif

// VGAout configuration

#define DOTCLOCK 25000
#define CLOCKMUL 9
// Pico does not work at CLOCKMUL=7 (175MHz).

#define VGA_PIXELS_X 640
#define VGA_PIXELS_Y 400

#define VGA_CHARS_X 80
#define VGA_CHARS_Y 25

#define VRAM_PAGE_SIZE (VGA_PIXELS_X*VGA_PIXELS_Y/8)

extern unsigned char vga_data_array[];
volatile uint8_t fbcolor,cursor_x,cursor_y,video_mode;

volatile uint32_t video_hsync,video_vsync,scanline;
volatile uint32_t redraw_command=0;
volatile uint32_t scroll_flag=0;

struct repeating_timer timer,timer2;

// MC6809 configuration

mc6809__t      main_cpu,sub_cpu;
uint8_t mainram[0x10000];
uint8_t subram[0xd380];

volatile int sub_cpu_halt=0;
semaphore_t dualport_lock;

// Memory Mapped IO

uint8_t mainioport[0x100];
uint8_t subioport[0x10];

uint16_t vramoffset=0;
uint16_t oldvramoffset=0;
uint8_t color_palette[0x80];

volatile uint8_t keypressed=0;  //last pressed usbkeycode
volatile uint16_t fm7keypressed=0;
//volatile uint16_t fm7lastkeypressed=0;  // for auto repeat
uint32_t key_repeat_flag=1;
uint32_t key_repeat_count=0;
uint32_t key_caps=0;
uint32_t key_kana=0;
uint32_t lastmodifier=0; 

uint32_t fm7cpuclock=1; 
//uint32_t fm7cpuclock=0; 

// irq/firq flags

volatile uint32_t key_break_flag=0;
volatile uint32_t attention_flag=0; 
volatile uint32_t main_timer_irq=0;
volatile uint32_t key_irq=0;
volatile uint32_t mfd_irq=0;

// BEEP & PSG

uint32_t pwm_slice_num;
uint32_t beep_flag=0;
uint32_t beep_short_flag=0;
uint32_t beep_mute_flag=0;
volatile uint32_t sound_tick=0;

uint8_t psg_register_number=0;

uint8_t psg_register[16];
uint32_t psg_osc_interval[4];
uint32_t psg_osc_counter[4];

uint32_t psg_noise_interval;
uint32_t psg_noise_counter;
uint8_t psg_noise_output;
uint32_t psg_noise_seed;
uint32_t psg_envelope_interval;
uint32_t psg_envelope_counter;
//uint32_t psg_master_clock = (3579545/2);
uint32_t psg_master_clock = 1228800;
uint16_t psg_master_volume = 0;

uint8_t psg_tone_on[4], psg_noise_on[4];

const uint16_t psg_volume[] = { 0x00, 0x00, 0x01, 0x01, 0x02, 0x02, 0x03, 0x04,
        0x05, 0x06, 0x07, 0x08, 0x09, 0x0b, 0x0d, 0x10, 0x13, 0x17, 0x1b, 0x20,
        0x26, 0x2d, 0x36, 0x40, 0x4c, 0x5a, 0x6b, 0x80, 0x98, 0xb4, 0xd6, 0xff };

//#define SAMPLING_FREQ 48000
//#define SAMPLING_FREQ 44100
//#define SAMPLING_FREQ 32000
#define SAMPLING_FREQ 22050
#define TIME_UNIT 100000000                           // Oscillator calculation resolution = 10nsec
#define SAMPLING_INTERVAL (TIME_UNIT/SAMPLING_FREQ) 

// UART

#define TAPE_THRESHOLD 200000

uint8_t uart_rx[32];
uint8_t uart_nibble=0;
uint8_t uart_count=0;
volatile uint8_t uart_write_ptr=0;
volatile uint8_t uart_read_ptr=0;
uint32_t uart_cycle;

// UI

uint32_t menumode=0;
uint32_t menuitem=0;

// USB

hid_keyboard_report_t prev_report = { 0, 0, {0} }; // previous report to check key released
extern void hid_app_task(void);
unsigned long sub_hsync_cycle=0;
unsigned long main_hsync_cycle=0;

unsigned int usbcheck_count=0;
unsigned int kbhit=0;            // 4:Key pressed (timer stop)/3&2:Key depressed (timer running)/1:no key irq triggerd
uint8_t hid_dev_addr=255;
uint8_t hid_instance=255;
uint8_t hid_led;

#define HSYNC_INTERVAL 255  // may be 63.5us  
//#define HSYNC_INTERVAL 271  // may be 63.5us  
//#define HSYNC_INTERVAL 127  // may be 63.5us  
#define USB_CHECK_INTERVAL 30 // 31.5us*30=1ms

uint32_t fm7clocks[]= {
    160,127,            // main(1.2MHz)/sub(1MHz)
    255,255             // main&sub (2MHz)
};

uint32_t tapeclocks[] = {
    250,470,520,105,        // 0,1,tail,head
    418,781,892,165,
};


// Define the flash sizes
// This is setup to read a block of the flash from the end 
#define BLOCK_SIZE_BYTES (FLASH_SECTOR_SIZE)
#define HW_FLASH_STORAGE_BYTES  (512 * 1024)
//#define HW_FLASH_STORAGE_BASE   (PICO_FLASH_SIZE_BYTES - HW_FLASH_STORAGE_BYTES) // 655360
// for 1M flash pico
#define HW_FLASH_STORAGE_BASE   (1024*1024 - HW_FLASH_STORAGE_BYTES) // 655360

lfs_t lfs;
lfs_file_t lfs_file;

#define FILE_THREHSOLD 2000000
#define LFS_LS_FILES 9

uint32_t load_enabled=0;
uint32_t save_enabled=0;
uint32_t file_cycle=0;

unsigned char filename[16];

static void draw_framebuffer(uint16_t);
static inline unsigned char tohex(int);
static inline unsigned char fromhex(int);
static inline void video_print(uint8_t *);

// Virtual H-Sync for emulation
bool __not_in_flash_func(hsync_handler)(struct repeating_timer *t) {

    if(scanline%262==0) {
        video_vsync=3;
    }

    scanline++;

//    if((scanline%2)==0) {
        video_hsync=3;
//    }

    if((scanline%32==0)&&(mainioport[2]&4)) {
        main_timer_irq=1;
        main_cpu.irq=true;
    }

    if(scanline%315==0) {   // 315 = 20ms / 63.5us
        sub_cpu.nmi=true;
//        sub_cpu_nmi_count=scanline;
    }


    // if(scanline>262) {
    //     scanline=0;
    // }

//    video_hsync=0;

    return true;

}

// BEEP and PSG emulation
bool __not_in_flash_func(sound_handler)(struct repeating_timer *t) {

    uint32_t pon_count;
    uint16_t master_volume;
    uint8_t tone_output[4], noise_output[3],
            envelope_volume;

//    TIM1->CH4CVR = psg_master_volume;

    pwm_set_chan_level(pwm_slice_num,PWM_CHAN_A,psg_master_volume);

    master_volume = 0;

    // Run Noise generator

        psg_noise_counter += SAMPLING_INTERVAL;
        if (psg_noise_counter > psg_noise_interval) {
            psg_noise_seed = (psg_noise_seed >> 1)
                    | (((psg_noise_seed << 14) ^ (psg_noise_seed << 16))
                            & 0x10000);
            psg_noise_output = psg_noise_seed & 1;
            psg_noise_counter -= psg_noise_interval;
        }
        if (psg_noise_output != 0) {
            noise_output[0] = psg_noise_on[0];
            noise_output[1] = psg_noise_on[1];
            noise_output[2] = psg_noise_on[2];
        } else {
            noise_output[0] = 0;
            noise_output[1] = 0;
            noise_output[2] = 0;
        }

    // Run Envelope

        envelope_volume = 0;

        switch (psg_register[13] & 0xf) {
        case 0:
        case 1:
        case 2:
        case 3:
        case 9:
            if (psg_envelope_counter < psg_envelope_interval * 32) {
                envelope_volume = 31
                        - psg_envelope_counter / psg_envelope_interval;
                psg_envelope_counter += SAMPLING_INTERVAL;
            } else {
                envelope_volume = 0;
            }
            break;
        case 4:
        case 5:
        case 6:
        case 7:
        case 15:
            if (psg_envelope_counter < psg_envelope_interval * 32) {
                envelope_volume = psg_envelope_counter
                        / psg_envelope_interval;
                psg_envelope_counter += SAMPLING_INTERVAL;
            } else {
                envelope_volume = 0;
            }
            break;
        case 8:
            if (psg_envelope_counter < psg_envelope_interval * 32) {
                envelope_volume = 31
                        - psg_envelope_counter / psg_envelope_interval;
                psg_envelope_counter += SAMPLING_INTERVAL;
            } else {
                psg_envelope_counter -= psg_envelope_interval * 32;
                envelope_volume = 31;
            }
            break;
        case 10:
            if (psg_envelope_counter < psg_envelope_interval * 32) {
                envelope_volume = 31
                        - psg_envelope_counter / psg_envelope_interval;
                psg_envelope_counter += SAMPLING_INTERVAL;
            } else if (psg_envelope_counter
                    < psg_envelope_interval * 64) {
                envelope_volume = psg_envelope_counter
                        / psg_envelope_interval - 32;
                psg_envelope_counter += SAMPLING_INTERVAL;
            } else {
                psg_envelope_counter -= psg_envelope_interval * 64;
                envelope_volume = 31;
            }
            break;
        case 11:
            if (psg_envelope_counter < psg_envelope_interval * 32) {
                envelope_volume = 31
                        - psg_envelope_counter / psg_envelope_interval;
                psg_envelope_counter += SAMPLING_INTERVAL;
            } else {
                envelope_volume = 31;
            }
            break;
        case 12:
            if (psg_envelope_counter < psg_envelope_interval * 32) {
                envelope_volume = psg_envelope_counter
                        / psg_envelope_interval;
                psg_envelope_counter += SAMPLING_INTERVAL;
            } else {
                psg_envelope_counter -= psg_envelope_interval * 32;
                envelope_volume = 0;
            }
            break;
        case 13:
            if (psg_envelope_counter < psg_envelope_interval * 32) {
                envelope_volume = psg_envelope_counter
                        / psg_envelope_interval;
                psg_envelope_counter += SAMPLING_INTERVAL;
            } else {
                envelope_volume = 31;
            }
            break;
        case 14:
            if (psg_envelope_counter < psg_envelope_interval * 32) {
                envelope_volume = psg_envelope_counter
                        / psg_envelope_interval;
                psg_envelope_counter += SAMPLING_INTERVAL;
            } else if (psg_envelope_counter
                    < psg_envelope_interval * 64) {
                envelope_volume = 63
                        - psg_envelope_counter / psg_envelope_interval;
                psg_envelope_counter += SAMPLING_INTERVAL;
            } else {
                psg_envelope_counter -= psg_envelope_interval * 64;
                envelope_volume = 0;
            }
            break;
        }


    // Run Oscillator

    for (int i = 0; i < 4 ; i++) {
        pon_count = psg_osc_counter[i] += SAMPLING_INTERVAL;
        if (pon_count < (psg_osc_interval[i] / 2)) {
            tone_output[i] = psg_tone_on[i];
        } else if (pon_count > psg_osc_interval[i]) {
            psg_osc_counter[i] -= psg_osc_interval[i];
            tone_output[i] = psg_tone_on[i];
        } else {
            tone_output[i] = 0;
        }
    }

    // Mixer

    master_volume = 0;

        for (int j = 0; j < 3; j++) {
            if ((tone_output[j] + noise_output[j]) > 0) {
                if ((psg_register[j + 8] & 0x10) == 0) {
                    master_volume += psg_volume[(psg_register[j + 8 ]
                            & 0xf) * 2 + 1];
                } else {
                    master_volume += psg_volume[envelope_volume];
                }
            }
        }

    // add BEEP

    if((mainioport[3]&1)&&(tone_output[3]!=0)) {
        if(mainioport[3]&0x80) {   // beep flag
            master_volume+=psg_volume[31];
        } else if ((mainioport[3]&0x40)||(subioport[3])) {  // short beep
            if((scanline-beep_short_flag)<(262*30)) {
                master_volume+=psg_volume[31];
            } else {
                mainioport[3]&=0xbf;
                subioport[3]=0;
            }
        }
    }

    psg_master_volume = master_volume / (3 + 1);    // Add beep

    if (psg_master_volume > 255)
        psg_master_volume = 255;

    return true;

}

void __not_in_flash_func(uart_handler)(void) {

    uint8_t ch;


    if((main_cpu.cycles-uart_cycle)>TAPE_THRESHOLD) {
        uart_count=0;        
    }

    uart_cycle=main_cpu.cycles;

    if(uart_is_readable(uart0)) {
        ch=uart_getc(uart0);
        if(uart_count==0) {
            uart_nibble=fromhex(ch)<<4;
            uart_count++;
        } else {
            ch=fromhex(ch)+uart_nibble;
            uart_count=0;

            if(uart_read_ptr==uart_write_ptr+1) {  // buffer full
                return;
            }
            if((uart_read_ptr==0)&&(uart_write_ptr==31)) {
                return;
            }

            uart_rx[uart_write_ptr]=ch;
            uart_write_ptr++;
            if(uart_write_ptr>31) {
                uart_write_ptr=0;
            }
        }
    }

}

void tapeout(uint8_t b) {

    static uint32_t lastcycles;
    uint32_t diff;
    static uint8_t bit,tapedata;

    if((b&2)==0) return; // motor off

    diff=main_cpu.cycles-lastcycles;

    if(diff>2*1000) {
        bit=0;
        tapedata=0;
    }

//  printf("%x-%d-%d-%x\n\r",b,diff,bit,tapedata);

    if(b&1) {

//   printf("%x-%d-%d-%x\n\r",b,diff,bit,tapedata);

        if(diff<tapeclocks[3+fm7cpuclock*4]+10)  {  // may be start ? bit
//            bit++;
            bit=0;
            tapedata=0;
        } else if (diff<tapeclocks[0+fm7cpuclock*4]+10) {  // 0
            bit++;
            if(bit<10) {
                tapedata>>=1;
                tapedata&=0x7f;
            }

        } else if (diff<tapeclocks[1+fm7cpuclock*4]+10) {  // 1
            bit++;
            if(bit<10) {
                tapedata>>=1;
                tapedata|=0x80;
            }
        }  else if (diff<tapeclocks[2+fm7cpuclock*4]+10) {  // may be stop bit

                printf("%02x",tapedata);
                bit=0;
            
        }

    }

    lastcycles=main_cpu.cycles;

}

uint8_t tapein(void) {

    static uint32_t nextcycles;
    uint32_t diff;
    static uint8_t step,tapedata;

//    printf("%02x %d-%d ",tapedata,nextcycles-main_cpu.cycles,step);


    if((mainioport[0]&1)==0) {
        step=0;
        return 0x80; // motor off
    }

    if((main_cpu.cycles>nextcycles)&&(main_cpu.cycles-nextcycles)>TAPE_THRESHOLD) {
        step=0;
        nextcycles=main_cpu.cycles+tapeclocks[3+fm7cpuclock*4];

        // wait first data

        while(uart_read_ptr==uart_write_ptr) ;

//        if(uart_read_ptr==uart_write_ptr) {
//            return 0;    // Empty
//        }

        tapedata=uart_rx[uart_read_ptr];

        uart_read_ptr++;
        if(uart_read_ptr>31) {
            uart_read_ptr=0;
        }
    }

    if(nextcycles>main_cpu.cycles) {

        if(step%2) return 0x80;  // odd(=HIGH)

        return 0;   // even(=LOW)

    } else {

        step++;
        if(step<3) {  // Start bits
            nextcycles+=tapeclocks[0+fm7cpuclock*4];
        } else if(step<19) {
            if(tapedata&1) {
                nextcycles+=tapeclocks[1+fm7cpuclock*4];
            } else {
                nextcycles+=tapeclocks[0+fm7cpuclock*4];
            }
            if(step%2) {
                tapedata>>=1;
            }
        } else if((step==19)||(step==21)||(step==22)) {
        
            nextcycles+=tapeclocks[1+fm7cpuclock*4];

        } else if(step==20) { 

            nextcycles+=tapeclocks[2+fm7cpuclock*4];

        } else {  // get next byte
        
            nextcycles+=tapeclocks[3+fm7cpuclock*4];
            step=0;

            while(uart_read_ptr==uart_write_ptr) ;

//        if(uart_read_ptr==uart_write_ptr) {
//            return 0;    // Empty
//        }

            tapedata=uart_rx[uart_read_ptr];

            uart_read_ptr++;
            if(uart_read_ptr>31) {
                uart_read_ptr=0;
            }
        }
        
        if(step%2) return 0x80;  // odd(=HIGH)

        return 0;   // even(=LOW)

    }

    return 0;

}


#if 0
uint8_t acia_getc() {

    uint8_t mode,ch;

    mode=ioport[0xd0];

    if(((mode&0x20)==0)&&(load_enabled!=0)) { // Get data from LFS file

        lfs_file_read(&lfs,&lfs_file,&ch,1);

        load_enabled=2; 
        file_cycle=cpu.cycles;

        return ch;

    } else {    // Get data from UART

        if(uart_read_ptr==uart_write_ptr) {
            return 0;    // Empty
        }

        ch=uart_rx[uart_read_ptr];

        uart_read_ptr++;
        if(uart_read_ptr>31) {
            uart_read_ptr=0;
        }
        return ch;
    }

}

uint8_t acia_rx_available() {

    uint8_t mode,ch;


    mode=ioport[0xd0];

    if(((mode&0x20)==0)&&(load_enabled!=0)) { // Get data from LFS file

        return 1;

    } else {    // Get data from UART

        if(uart_read_ptr==uart_write_ptr) {
            return 0;    // Empty
        }

        return 1;
    }
}

void acia_write(uint8_t b) {

    uint8_t mode;

    mode=ioport[0xd0];

    if(((mode&0x20)==0)&&(save_enabled!=0)) {

        lfs_file_write(&lfs,&lfs_file,&b,1);
        save_enabled=2;
        file_cycle=cpu.cycles;

    } else {

                uart_putc(uart0,tohex(b>>4));
                uart_putc(uart0,tohex(b&0xf));

    }

}
#endif



static inline void video_cls() {
    memset(vga_data_array, 0x0, (640*400/2));
}

static inline void video_scroll() {

    memmove(vga_data_array, vga_data_array + VGA_PIXELS_X/2*16, (640*384/2));
    memset(vga_data_array + (640*384/2), 0, VGA_PIXELS_X/2*16);

}

static inline void video_print(uint8_t *string) {

    int len;
    uint8_t fdata;

    len = strlen(string);

    for (int i = 0; i < len; i++) {

        for(int slice=0;slice<16;slice++) {

            uint8_t ch=string[i];

            fdata=fm7subrom[(ch*8+(slice>>1))];
  
            uint32_t vramindex=cursor_x*4+VGA_PIXELS_X*(cursor_y*16+slice)/2;

            for(int slice_x=0;slice_x<4;slice_x++){

                if(fdata&0x40) {
                    vga_data_array[vramindex+slice_x]=(fbcolor&7)<<4;
                } else {
                    vga_data_array[vramindex+slice_x]=fbcolor&0x70;  
                }

                  if(fdata&0x80) {
                    vga_data_array[vramindex+slice_x]+=fbcolor&7;
                } else {
                    vga_data_array[vramindex+slice_x]+=(fbcolor&0x70)>>4;  
                }              

                fdata<<=2;

            }

        }

        cursor_x++;
        if (cursor_x >= VGA_CHARS_X) {
            cursor_x = 0;
            cursor_y++;
            if (cursor_y >= VGA_CHARS_Y) {
                video_scroll();
                cursor_y = VGA_CHARS_Y - 1;
            }
        }
    }

}

void draw_menu(void) {

    cursor_x=10;
    cursor_y=5;
    fbcolor=7;
    video_print("+-------------------------------------+");
    for(int i=6;i<16;i++) {
        cursor_x=10;
        cursor_y=i;
        video_print("|                                     |");
    }

    cursor_x=10;
    cursor_y=16;
    fbcolor=7;
    video_print("+-------------------------------------+");

}

int draw_files(int num_selected,int page) {

    lfs_dir_t lfs_dirs;
    struct lfs_info lfs_dir_info;
    uint32_t num_entry=0;
    unsigned char str[16];

    int err= lfs_dir_open(&lfs,&lfs_dirs,"/");

    if(err) return -1;

    for(int i=0;i<LFS_LS_FILES;i++) {
        cursor_x=32;
        cursor_y=i+6;
        fbcolor=7;
        video_print("          ");
    }

    while(1) {

        int res= lfs_dir_read(&lfs,&lfs_dirs,&lfs_dir_info);
        if(res<=0) {
            break;
        }

        cursor_x=36;
        cursor_y=15;
        fbcolor=7;
        sprintf(str,"Page %02d",page+1);

        video_print(str);

        switch(lfs_dir_info.type) {

            case LFS_TYPE_DIR:
                break;
            
            case LFS_TYPE_REG:

                if((num_entry>=LFS_LS_FILES*page)&&(num_entry<LFS_LS_FILES*(page+1))) {

                    cursor_x=32;
                    cursor_y=num_entry%LFS_LS_FILES+6;

                    if(num_entry==num_selected) {
                        fbcolor=0x70;
                        memcpy(filename,lfs_dir_info.name,16);
                    } else {
                        fbcolor=7;
                    }

                    video_print(lfs_dir_info.name);

                }

                num_entry++;

                break;

            default:
                break; 

        }

    }

    lfs_dir_close(&lfs,&lfs_dirs);

    return num_entry;

}

int file_selector(void) {

    uint32_t num_selected=0;
    uint32_t num_files=0;
    uint32_t num_pages=0;

    num_files=draw_files(-1,0);

    if(num_files==0) {
         return -1;
    }

    while(1) {

        while(video_vsync==0) ;
        video_vsync=0;

        draw_files(num_selected,num_selected/LFS_LS_FILES);

        tuh_task();

        if(keypressed==0x52) { // up
            keypressed=0;
            if(num_selected>0) {
                num_selected--;
            }
        }

        if(keypressed==0x51) { // down
            keypressed=0;
            if(num_selected<num_files-1) {
                num_selected++;
            }
        }

        if(keypressed==0x28) { // Ret
            keypressed=0;

            return 0;
        }

        if(keypressed==0x29 ) {  // ESC

            return -1;

        }

    }
}

int enter_filename() {

    unsigned char new_filename[16];
    unsigned char str[32];
    uint8_t keycode;
    uint32_t pos=0;

    memset(new_filename,0,16);

    while(1) {

        sprintf(str,"Filename:%s  ",new_filename);
        cursor_x=12;
        cursor_y=15;
        video_print(str);

        while(video_vsync==0) ;
        video_vsync=0;

        tuh_task();

        if(keypressed!=0) {

            if(keypressed==0x28) { // enter
                keypressed=0;
                if(pos!=0) {
                    memcpy(filename,new_filename,16);
                    return 0;
                } else {
                    return -1;
                }
            }

            if(keypressed==0x29) { // escape
                keypressed=0;
                return -1;
            }

            if(keypressed==0x2a) { // backspace
                keypressed=0;

                cursor_x=12;
                cursor_y=15;
                video_print("Filename:          ");

                new_filename[pos]=0;

                if(pos>0) {
                    pos--;
                }
            }

            if(keypressed<0x4f) {
                keycode=usbhidcode[keypressed*2];
                keypressed=0;

                if(pos<7) {

                    if((keycode>0x20)&&(keycode<0x5f)&&(keycode!=0x2f)) {

                        new_filename[pos]=keycode;
                        pos++;

                    }

                }
            }


        }
    }

}


static void draw_framebuffer(uint16_t addr){

    uint32_t slice_x,slice_y;
    uint16_t offset;
    uint32_t vramindex;
    uint8_t bitb,bitr,bitg,color;
    uint32_t bitwb,bitwr,bitwg;

    union bytemember {
        uint32_t w;
        uint8_t b[4];
    };

    union bytemember bitw,bitc;

    if(subioport[0x8]!=0) {
        return;
    }

    uint32_t *vramptr=(uint32_t *)vga_data_array;

    addr&=0x3fff;

    offset=vramoffset+addr;
    offset&=0x3fff;

    slice_x=addr%80;
    slice_y=addr/80;

//    slice_x=offset%80;
//    slice_y=offset/80;

        //  unsigned char str[40];

        //  sprintf(str,"%02d:%03d",slice_x,slice_y);
        //  cursor_x=70;
        //  cursor_y=24;
        //  video_print(str);

//        sem_acquire_blocking(&dualport_lock);

        uint8_t mask=mainioport[0x37];

//        sem_release(&dualport_lock);


    if(slice_y<200) {



        if(mask&0x10) {
            bitb=0;
        } else {
            bitb=subram[offset];   
        }
        if(mask&0x20) {
            bitr=0;
        } else {
            bitr=subram[offset+0x4000];
        }
        if(mask&0x40) {
            bitg=0;
        } else {
            bitg=subram[offset+0x8000];
        }

        vramindex=slice_x + slice_y*640/4;

        bitwb=bitexpand80[bitb*2];
        bitwr=bitexpand80[bitr*2]<<1;
        bitwg=bitexpand80[bitg*2]<<2;

        bitw.w=bitwb+bitwr+bitwg;

        bitc.b[0]=color_palette[bitw.b[0]];
        bitc.b[1]=color_palette[bitw.b[1]];
        bitc.b[2]=color_palette[bitw.b[2]];
        bitc.b[3]=color_palette[bitw.b[3]];

        *(vramptr+vramindex) = bitc.w;
        *(vramptr+vramindex+VGA_PIXELS_X/8) = bitc.w;

    }

}

static inline void redraw(){

    for(int i=0;i<0x3e80;i++) {
        draw_framebuffer(i);
    }

}

static inline void scroll(uint16_t old_offset,uint16_t new_offset) {

    uint32_t delta,size;
    uint32_t movesize;
    uint32_t deltax,deltay;

    oldvramoffset=vramoffset;
    scroll_flag=0;

    if (old_offset-new_offset>80*100) {
        new_offset+=0x4000;
    } else if(new_offset-old_offset>80*100) {
        old_offset+=0x4000;
    }
          
    if(old_offset>new_offset) {  // down scroll

        delta=old_offset-new_offset;

        size=80*200-delta;
    
        if((delta%80)!=0) {    
            redraw();
            return;

        } else {
            delta*=8;
        }

        // move up half
        movesize=640*400/2-delta;
        memmove(vga_data_array+delta,vga_data_array,movesize);

        // redraw new area
        for(int i=0;i<size;i++) {
            draw_framebuffer(i);
        }

    } else {  // up scroll

        delta=new_offset-old_offset;
        size=80*200-delta;

        if((delta%80)!=0) {

            redraw();
            return;

        } else {
            delta*=8;
        }

        // move up half
        movesize=640*400/2-delta;
        memmove(vga_data_array,vga_data_array+delta,movesize);

        // redraw new area
        for(int i=size;i<0x3e80;i++) {
            draw_framebuffer(i);
        }

    }

}

static inline void rebuild_colorpalette(void) {

    for(int i=0;i<8;i++) {
        for(int j=0;j<8;j++) {
            color_palette[i*16+j]=mainioport[0x38+i]*16+mainioport[0x38+j];
        }
    }
}


//----------------------------------------------------------------------------------------------

void psg_reset(int flag) {

    psg_noise_seed = 12345;

    if (flag == 0) {
        for (int i = 0; i < 16; i++) {
            psg_register[i] = 0;
        }
    } else {
        for (int i = 0; i < 15; i++) {
            psg_register[i] = 0;
        }
    }
    psg_register[7] = 0xff;

    psg_noise_interval = UINT32_MAX;
    psg_envelope_interval = UINT32_MAX / 2 - 1;

    for (int i = 0; i < 3; i++) {
        psg_osc_interval[i] = UINT32_MAX;
        psg_tone_on[i] = 0;
        psg_noise_on[i] = 0;
    }

    // for BEEP

    psg_osc_interval[3] = TIME_UNIT/ 2000 ; // beep 2kHz
    psg_tone_on[3]=1;

}



//----------------------------------------------------------------------------------------------------

static inline unsigned char tohex(int b) {

    if(b==0) {
        return '0';
    } 
    if(b<10) {
        return b+'1'-1;
    }
    if(b<16) {
        return b+'a'-10;
    }

    return -1;

}

static inline unsigned char fromhex(int b) {

    if(b=='0') {
        return 0;
    } 
    if((b>='1')&&(b<='9')) {
        return b-'1'+1;
    }
    if((b>='a')&&(b<='f')) {
        return b-'a'+10;
    }

    return -1;

}

// LittleFS

int pico_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
    uint32_t fs_start = XIP_BASE + HW_FLASH_STORAGE_BASE;
    uint32_t addr = fs_start + (block * c->block_size) + off;
    
//    printf("[FS] READ: %p, %d\n", addr, size);
    
    memcpy(buffer, (unsigned char *)addr, size);
    return 0;
}

int pico_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size)
{
    uint32_t fs_start = HW_FLASH_STORAGE_BASE;
    uint32_t addr = fs_start + (block * c->block_size) + off;
    
//    printf("[FS] WRITE: %p, %d\n", addr, size);
        
    uint32_t ints = save_and_disable_interrupts();
    multicore_lockout_start_blocking();     // pause another core
    flash_range_program(addr, (const uint8_t *)buffer, size);
    multicore_lockout_end_blocking();
    restore_interrupts(ints);
    
    return 0;
}

int pico_erase(const struct lfs_config *c, lfs_block_t block)
{           
    uint32_t fs_start = HW_FLASH_STORAGE_BASE;
    uint32_t offset = fs_start + (block * c->block_size);
    
//    printf("[FS] ERASE: %p, %d\n", offset, block);
        
    uint32_t ints = save_and_disable_interrupts();   
    multicore_lockout_start_blocking();     // pause another core
    flash_range_erase(offset, c->block_size);  
    multicore_lockout_end_blocking();
    restore_interrupts(ints);

    return 0;
}

int pico_sync(const struct lfs_config *c)
{
    return 0;
}

// configuration of the filesystem is provided by this struct
const struct lfs_config PICO_FLASH_CFG = {
    // block device operations
    .read  = &pico_read,
    .prog  = &pico_prog,
    .erase = &pico_erase,
    .sync  = &pico_sync,

    // block device configuration
    .read_size = FLASH_PAGE_SIZE, // 256
    .prog_size = FLASH_PAGE_SIZE, // 256
    
    .block_size = BLOCK_SIZE_BYTES, // 4096
    .block_count = HW_FLASH_STORAGE_BYTES / BLOCK_SIZE_BYTES, // 352
    .block_cycles = 16, // ?
    
    .cache_size = FLASH_PAGE_SIZE, // 256
    .lookahead_size = FLASH_PAGE_SIZE,   // 256    
};



//  Keyboard


void process_kbd_leds(void) {

    hid_led=0;

    if(key_caps) hid_led+=KEYBOARD_LED_CAPSLOCK;     // CAPS Lock
    if(key_kana) hid_led+=KEYBOARD_LED_NUMLOCK;           // KANA -> Numlock
    if(subioport[0xd]) hid_led+=KEYBOARD_LED_SCROLLLOCK;        // INS -> ScrollLock

    if((hid_dev_addr!=255)&&(hid_instance!=255)) {
        tuh_hid_set_report(hid_dev_addr, hid_instance, 0, HID_REPORT_TYPE_OUTPUT, &hid_led, sizeof(hid_led));
    }

}


static inline bool find_key_in_report(hid_keyboard_report_t const *report, uint8_t keycode)
{
  for(uint8_t i=0; i<6; i++)
  {
    if (report->keycode[i] == keycode)  return true;
  }

  return false;
}

static inline int16_t getkeycode(uint8_t modifier,uint8_t keycode) {

    uint16_t fm7code;

    if(modifier&0x11) {  // Control

        fm7code=fm7usbcode[keycode*5];

        // Auto repeat control (SHIT+CTRL+0/1)

        if(modifier&22) {
            if(keycode==0x1e) {
                key_repeat_flag=1;
                return -1;
            }
            if(keycode==0x27) {
                key_repeat_flag=0;
            }
        }

        if(fm7code==0) return -1;

        if(fm7code==0x40) return 0;
        if((fm7code>=0x61)&&(fm7code<=0x7a)) return fm7code-0x60;
        if((fm7code>=0x5b)&&(fm7code<=0x5f)) return fm7code-0x40;

        return -1;

    } else if (modifier&0x44) { // ALT=>Graph

        fm7code=fm7usbcode[keycode*5+4];

        if(fm7code!=0) return fm7code;
        return -1;

    } else {

        if(key_kana) {
            if(modifier&0x22) {
                fm7code=fm7usbcode[keycode*5+3];
            } else {
                fm7code=fm7usbcode[keycode*5+2];
            }
            if(fm7code!=0) return fm7code;
            return -1;
        } else {
            if(modifier&0x22) {
                fm7code=fm7usbcode[keycode*5+1];
            } else {
                fm7code=fm7usbcode[keycode*5];
            }

            if(fm7code==0) return -1;

            if(key_caps) {
                if((fm7code>=0x41)&&(fm7code<=0x5a)) {
                    fm7code+=0x20;
                } else  if((fm7code>=0x61)&&(fm7code<=0x7a)) {
                    fm7code-=0x20;
                }
            }

            return fm7code;

        }
    }

    return -1;
}


void process_kbd_report(hid_keyboard_report_t const *report) {

    int usbkey;
    int16_t fm7code;

    if(menumode==0) { // Emulator mode

        // unsigned char str[16];
        // sprintf(str,"%d",sub_cpu.cycles);
        // cursor_x=60;
        // cursor_y=24;
        // video_print(str);

//        printf("%d",sub_cpu.cycles);

        key_repeat_count=0;

        if(report->modifier!=lastmodifier) {  // stop auto repeat when modifier changed

            if(key_repeat_flag) {
                key_repeat_count=0;
            }
        }

        lastmodifier=report->modifier;

        key_break_flag=0;

        for(int i=0;i<6;i++) {

            if ( report->keycode[i] )
                {
                if ( find_key_in_report(&prev_report, report->keycode[i]) )
                {
                // exist in previous report means the current key is holding
                }else
                {
                // check auto repeat on/off

//        unsigned char str[40];

                keypressed=report->keycode[i];

                fm7code=getkeycode(report->modifier,report->keycode[i]);

//        sprintf(str,"%02x:%02x:%02x",report->keycode[i],report->modifier,fm7code);
//        cursor_x=60;
//        cursor_y=22;
//        video_print(str);

                if(fm7code!=-1) {

                    fm7keypressed=fm7code;

                    if(mainioport[0x2]&1) {
                        main_cpu.irq=true;
                        key_irq=1;
                    } else {
                        sub_cpu.firq=true;
                    }

                    if(key_repeat_flag) {
//                        fm7lastkeypressed=fm7keypressed;
                        key_repeat_count=scanline;
                    }

                }

                // PF Keys

                if((keypressed>=0x3a)&&(keypressed<=0x43)) {

                    fm7keypressed=0x8000+(keypressed-0x39);

                    if(mainioport[0x2]&1) {
                        main_cpu.irq=true;
                        key_irq=1;
                    } else {
                        sub_cpu.firq=true;
                    }
                }

                // CapsLock

                if(keypressed==0x39) {
                    if(key_caps==0) {
                        key_caps=1;
                    } else {
                        key_caps=0;
                    }
                    process_kbd_leds();
                }

                // Kana

                if(keypressed==0x88) {
                    if(key_kana==0) {
                        key_kana=1;
                    } else {
                        key_kana=0;
                    }
                    process_kbd_leds();
                }

                // Break

                if(keypressed==0x48) {
                    key_break_flag=1;
                    main_cpu.firq=true;                
                }


            // DEBUG BENCHMARK
            // if(report->keycode[i]==0x44) {
            //     uint32_t scanline_old=scanline;
            //     for(int ii=0;ii<1000;ii++) {
            //         redraw();
            //     }


            //     printf("%d",scanline-scanline_old);
                
            // }   


            // TEST
            if(report->keycode[i]==0x44) {
                memcpy(mainram+0x2000,fm7test00,0x4a00);  // Exec &h6800
            }

            if(report->keycode[i]==0x45) {
                memcpy(mainram+0x1600,fm7test01,0x5100);  // Exec &h3300
            }



            // Enter Menu
            // if(report->keycode[i]==0x45) {
            //     prev_report=*report;
            //     menumode=1;
            // }  



                }
            }   



        }

    prev_report=*report;

    // Auto repeat

} else {  // menu mode

    for(uint8_t i=0; i<6; i++)
    {
        if ( report->keycode[i] )
        {
        if ( find_key_in_report(&prev_report, report->keycode[i]) )
        {
            // exist in previous report means the current key is holding
        }else
        {
            keypressed=report->keycode[i];
        }
        }
    } 
    prev_report = *report;
    }

}

/***********************************************************************/
static mc6809byte__t main_cpu_read(
        mc6809__t     *cpu,
        mc6809addr__t  addr,
        bool           ifetch __attribute__((unused))
)
{

//    unsigned char str[16];
//    uint32_t d1,d2;

    uint8_t b,f;

        // unsigned char str[80];
        // sprintf(str,"R:%04x",addr);
        // cursor_x=10;
        // cursor_y=24;
        // video_print(str);

//    assert(cpu       != NULL);
  
    if(addr<0x8000) {
        // RAM area

        return mainram[addr];

    } else if (addr<0xfc00) { // ROM area

        if(mainioport[0x0f]!=0) {
            return mainram[addr];
        } else {
            return fm7mainrom[addr-0x8000];
        }

    } else if ((addr>=0xfc00)&&(addr<0xfc80)) { // RAM

        return mainram[addr];

    } else if ((addr>=0xfc80)&&(addr<0xfd00)) {  // Dualport RAM

        sem_acquire_blocking(&dualport_lock);

        b=mainram[addr];

        sem_release(&dualport_lock);

        return b;

    } else if ((addr>=0xfe00)&&(addr<0xffe0)) { // Boot ROM

        return fm7bootbas[addr-0xfe00];

    } else if (addr>=0xffe0) {  // Interrupt vector

        return mainram[addr];

    } else  {

            // Memory mapped IO

        //   unsigned char str[80];
//          sprintf(str,"%04x:%02x",addr,mainioport[addr-0xfd00]);
        //             sprintf(str,"%04x:%02x:%04x:%04x",addr,key_break_flag,keypressed,mainioport[5]);
        //   cursor_x=0;
        //   cursor_y=24;
        //   fbcolor=7;
        //   video_print(str);
 //       printf("R:%04x:%02x\n",addr,mainioport[addr-0xfd00]);

        switch(addr-0xfd00) {

            case 0: // Key data & CPU Speed

                sem_acquire_blocking(&dualport_lock);

                b=fm7keypressed>>8;

                sem_release(&dualport_lock);

                if(fm7cpuclock) {
                    b|=1;
                } else {
                    b&=0xfe;
                }

                return b;

            case 1: // Key data

                sem_acquire_blocking(&dualport_lock);

                b=fm7keypressed&0xff;
//                fm7keypressed=0;

                key_irq=0;

                sem_release(&dualport_lock);

                return b;            

            case 2: // Tape in

                b=tapein();

 //       printf("%x-%d\n\r ",b,main_cpu.cycles);

            //    if(b==0) return 0x80;
            //    return 0;

                return b;

            case 3: // IRQ flag

                f=0xff;

                if(key_irq==1) f&=0xfe;
                if(main_timer_irq==1) f&=0xfb;
                if(mfd_irq==1) f&=0xef;

                main_timer_irq=0;
                mfd_irq=0;

                return f;

            case 4: // FIRQ Flag

                f=0xff;

                sem_acquire_blocking(&dualport_lock);

                b=subioport[4];
                subioport[4]=0;   // clear attention

                sem_release(&dualport_lock);

                if(b!=0) f&=0xfe;
//                if(key_break_flag==1) f&=0xfd;
                if(key_break_flag==1) f&=0x7d;

//        printf("R:fd04:%d:%x:%d:%d\n\r",f,main_cpu.pc.w,main_cpu.firq,main_cpu.cc.f);

                return f;

            case 5:

                sem_acquire_blocking(&dualport_lock);
                b=mainioport[5];
                sem_release(&dualport_lock);

                if(b&0x80) return 0xfe;
                return 0x7e;

//                return b;

            case 0x0f:  // Bank select (to ROM)
                mainioport[0xf]=0;
                return 0xff;

            case 7:
            case 0x25:
            case 0x27:
            case 0x29:
            case 0x2b:

                return 0xff;

            default:
                return mainioport[addr-0xfd00];
        }
        
    }

    return 0xff;

}

static mc6809byte__t sub_cpu_read(
        mc6809__t     *cpu,
        mc6809addr__t  addr,
        bool           ifetch __attribute__((unused))
)
{

    unsigned char str[16];

    uint16_t offset;
    uint8_t b;

    uint32_t d1,d2;

//    assert(cpu       != NULL);

    if(addr<0x4000) {

        if(mainioport[0x37]&1) {
            return 0xff;           
        } else {
            offset=(addr+vramoffset)&0x3fff;
            return subram[offset];
//            return subram[addr];
        }

    } else if(addr<0x8000) {

        if(mainioport[0x37]&2) {
            return 0xff;
        } else {
            offset=(addr+vramoffset)&0x3fff;
            return subram[offset+0x4000];
        //    return subram[addr];            
        }

    } else if(addr<0xc000) {

        if(mainioport[0x37]&4) {
            return 0xff;
        } else {
            offset=(addr+vramoffset)&0x3fff;
            return subram[offset+0x8000];
        //    return subram[addr];            
        }

    } else if(addr<0xd380) {
        
        return subram[addr];

    } else if(addr<0xd400) {  // Dual port RAM

        sem_acquire_blocking(&dualport_lock);

        b=mainram[addr-0xd380+0xfc80];

        sem_release(&dualport_lock);

        return b;

    } else if(addr>=0xd800) {

        return fm7subrom[addr-0xd800];

    } else {
        switch((addr-0xd400)%0x10) {

            case 0: // Key data

                sem_acquire_blocking(&dualport_lock);

                b=fm7keypressed>>8;

                sem_release(&dualport_lock);

                return b;

            case 1: // Key data

                sem_acquire_blocking(&dualport_lock);

                b=fm7keypressed&0xff;
//                fm7keypressed=0;

 //           printf("SR:1:%x:%d:%d\n\r",sub_cpu.pc.w,b,key_irq);

                // Clear IRQ Flag ?
                key_irq=0;

                sem_release(&dualport_lock);

                return b;            

            case 2: // Acknowledge Cancel IRQ

                subioport[2]=0;
                return 0xff;

            case 3:

                subioport[3]=1;
                beep_short_flag=scanline;
                return 0xff;

            case 4: // Attention FIRQ

                subioport[4]=1;
                main_cpu.firq=true;

                return 0xff;

            case 0x8: // display ON

                subioport[0x8]=0;

                redraw();

                return 0xff;

            case 0xa: // clear BUSY FLAG

                sem_acquire_blocking(&dualport_lock);
                mainioport[5]&=0x7f;
                sem_release(&dualport_lock);

                return 0xff;

            case 0xd: // set INS LED

                subioport[0xd]=1;
                process_kbd_leds();
                return 0xff;

            default:
                return subioport[(addr-0xd400)%0x10];

        }

    }

  return 0xff;

}




/************************************************************************/

static void main_cpu_write(
        mc6809__t     *cpu,
        mc6809addr__t  addr,
        mc6809byte__t  b
)
{

        // unsigned char str[80];
        // sprintf(str,"W:%04x",addr);
        // cursor_x=20;
        // cursor_y=24;
        // video_print(str);


    if(addr<0x8000) {

        mainram[addr]=b;

        return;

    } else if(addr<0xfc00) {
        if(mainioport[0xf]!=0) {
            mainram[addr]=b;
        }
        return;
    } else if(addr<0xfc80) {
        mainram[addr]=b;
        return;
    } else if(addr<0xfd00) { // dual port ram

//        if(addr==0xfc82) {

        //  unsigned char str[80];
        //  sprintf(str,"%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x%02x",mainram[0xfc82],mainram[0xfc83],mainram[0xfc84],mainram[0xfc85],mainram[0xfc86],mainram[0xfc87],mainram[0xfc88],mainram[0xfc89],mainram[0xfc8a],mainram[0xfc8b],subioport[14],subioport[15]);
        //  cursor_x=0;
        //  cursor_y=22;
        //  fbcolor=7;
        //  video_print(str);
//        }


        sem_acquire_blocking(&dualport_lock);

        mainram[addr]=b;
        
        sem_release(&dualport_lock);

        return;

    } else if(addr>=0xffe0) { // interrupt vector

        mainram[addr]=b;
        return;

    }

    if((addr>=0xfd00)&&(addr<0xfe00)) {

//        printf("W:%04x:%02x\n",addr,b);
        uint diff;

        switch(addr-0xfd00) {

            case 0: // Tape out

                tapeout(b);

                // diff=main_cpu.cycles-uart_cycle;

                // printf("CS:%02x,%d\n\r",b,diff);

                // uart_cycle=main_cpu.cycles;

                return;

            case 3: // beep

                if(b&0x40) beep_short_flag=scanline;
                mainioport[addr-0xfd00]=b;
                return;

            case 5:

                sem_acquire_blocking(&dualport_lock);

                // HALT SUB CPU

                if(b&0x80) {
                    sub_cpu_halt=1;
                    mainioport[5]|=0x80;
                } else {
                    sub_cpu_halt=0;
                }

                // Cancel IRQ to SUB CPU

                if(b&0x40) {
                    sub_cpu.irq=true;
                    subioport[2]=1;
                }

                sem_release(&dualport_lock);

                return;

            case 0xd:  // PSG command

                if(b==3) {
                    psg_register_number=mainioport[0xe];
                } else if(b==2) {

                    uint32_t freq;

                    if(psg_register_number>15) return;

 //               printf("%02x-%02x\n\r",psg_register_number,mainioport[0xe]);

                    psg_register[psg_register_number]=mainioport[0xe];

                    switch(psg_register_number&0xf) {
                        case 0:
                        case 1:
                            if((psg_register[0]==0)&&(psg_register[1]==0)) {
                                psg_osc_interval[0]=UINT32_MAX;
                                break;
                            }
                            freq = psg_master_clock / ( psg_register[0] + ((psg_register[1]&0x0f)<<8) );
                            freq >>= 4;
                            if(freq!=0) {
                                psg_osc_interval[0] = TIME_UNIT / freq;
                                psg_osc_counter[0]=0;
                            } else {
                                psg_osc_interval[0]=UINT32_MAX;
                            }
                            break;
                        case 2:
                        case 3:
                            if((psg_register[2]==0)&&(psg_register[3]==0)) {
                                psg_osc_interval[1]=UINT32_MAX;
                                break;
                            }
                            freq = psg_master_clock / ( psg_register[2] + ((psg_register[3]&0x0f)<<8) );
                            freq >>= 4;
                            if(freq!=0) {
                                psg_osc_interval[1] = TIME_UNIT / freq;
                                psg_osc_counter[1]=0;
                            } else {
                                psg_osc_interval[1]=UINT32_MAX;
                            }
                            break;
                        case 4:
                        case 5:
                            if((psg_register[4]==0)&&(psg_register[5]==0)) {
                                psg_osc_interval[2]=UINT32_MAX;
                                break;
                            }
                            freq = psg_master_clock / ( psg_register[4] + ((psg_register[5]&0x0f)<<8) );
                            freq >>= 4;
                            if(freq!=0) {
                                psg_osc_interval[2] = TIME_UNIT / freq;
                                psg_osc_counter[2]=0;
                            } else {
                                psg_osc_interval[2]=UINT32_MAX;
                            }
                            break;
                        case 6:
                            if((psg_register[6]==0)&&(psg_register[7]==0)) {
                                psg_noise_interval=UINT32_MAX;
                                break;
                            }
                            freq = psg_master_clock / ( psg_register[6] & 0x1f );
                            freq >>= 4;
                            if(freq!=0) {
                            psg_noise_interval = TIME_UNIT / freq;
                            psg_noise_counter = 0;
                            } else {
                                psg_noise_interval=UINT32_MAX;
                            }
                            break;
                        case 7:
                            psg_tone_on[0]=((psg_register[7]&1)==0?1:0);
                            psg_tone_on[1]=((psg_register[7]&2)==0?1:0);
                            psg_tone_on[2]=((psg_register[7]&4)==0?1:0);
                            psg_noise_on[0]=((psg_register[7]&8)==0?1:0);
                            psg_noise_on[1]=((psg_register[7]&16)==0?1:0);
                            psg_noise_on[2]=((psg_register[7]&32)==0?1:0);
                            break;
                        case 0xb:
                        case 0xc:
                            freq = psg_master_clock / ( psg_register[0xb] + (psg_register[0xc]<<8) );
                            if(freq!=0) {
                                psg_envelope_interval= TIME_UNIT / freq;
                                psg_envelope_interval<<=5;
                            } else {
                                psg_envelope_interval=UINT32_MAX/2-1;
                            }
                            break;
                        case 0xd:
                            psg_envelope_counter=0;
                            break;
//                        case 0xf:
//                        psg_reset(1,psg_no);
                    }
                }

                return;

            case 0xf:
                mainioport[0xf]=1;
                return;

            case 0x38:
            case 0x39:
            case 0x3a:
            case 0x3b:
            case 0x3c:
            case 0x3d:
            case 0x3e:
            case 0x3f:
                     
                // change color pallete

                mainioport[addr-0xfd00]=b&7;
                rebuild_colorpalette();
                redraw();
                return;

            default:

                mainioport[addr-0xfd00]=b;
                return;

        }

    }
    
}

static void sub_cpu_write(
        mc6809__t     *cpu,
        mc6809addr__t  addr,
        mc6809byte__t  b
)
{

        uint16_t offset;

         unsigned char str[80];

    if((addr<0x4000)&&((mainioport[0x37]&1)==0)) {

        offset=(addr+vramoffset)&0x3fff;

        subram[offset]=b;
        draw_framebuffer(addr);

        return;

    } else if((addr<0x8000)&&((mainioport[0x37]&2)==0)) {

        offset=(addr+vramoffset)&0x3fff;

        subram[offset+0x4000]=b;

        draw_framebuffer(addr);

        return;

    } else if((addr<0xc000)&&((mainioport[0x37]&4)==0)) {

        offset=(addr+vramoffset)&0x3fff;

        subram[offset+0x8000]=b;

        draw_framebuffer(addr);
        return;

    } else if(addr<0xd380) {

        subram[addr]=b;
        return;

    } else if(addr<0xd400) { // Dualport RAM

        sem_acquire_blocking(&dualport_lock);

        mainram[addr-0xd380+0xfc80]=b;
        
        sem_release(&dualport_lock);


        // if(mainram[0xfc80]!=0) {

        //   unsigned char str[80];
        //   sprintf(str,"%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x%02x",mainram[0xfc80],mainram[0xfc81],mainram[0xfc82],mainram[0xfc83],mainram[0xfc84],mainram[0xfc85],mainram[0xfc86],mainram[0xfc87],mainram[0xfc88],mainram[0xfc89],mainram[0xfc8a],mainram[0xfc8b]);
        //   cursor_x=0;
        //   cursor_y=22;
        //   fbcolor=7;
        //   video_print(str);

        // }




        return;

    } else if((addr>=0xd400)&&(addr<0xd800)) {  

        switch((addr-0xd400)%0x10) {

            case 2: // Acknowledge Cancel IRQ

                subioport[2]=0;
                return;

            case 3:
                subioport[3]=1;
                beep_short_flag=scanline;
                return;

            case 0x4: // Attention FIRQ

                subioport[4]=1;
                main_cpu.firq=true;

                return;

            case 0x8: // display OFF

                subioport[0x8]=1;

                // clear display

                memset(vga_data_array,0,640*400/2);

                return;

            case 0xa:   // set Busy flag

                sem_acquire_blocking(&dualport_lock);
                
                mainioport[0x5]|=0x80;

                sem_release(&dualport_lock);

                return;

            case 0xd: // clear INS LED

                subioport[0xd]=0;
                process_kbd_leds();
                return;

            case 0xe:
            case 0xf:

                // Scroll screen
                subioport[(addr-0xd400)%0x10]=b;

                vramoffset=((subioport[14]*256)+subioport[15])&0x3fe0;

                scroll_flag=1;

                return;

            default:
                subioport[(addr-0xd400)%0x10]=b;
                return;

        }

    }

    
}


/************************************************************************/

static void cpu_fault(
        mc6809__t      *cpu,
        mc6809fault__t  fault
)
{
  assert(cpu != NULL);

  longjmp(cpu->err,fault);
}

void main_core1(void) {

    uint32_t redraw_start,redraw_length;
    uint32_t maincpu_wait;

    multicore_lockout_victim_init();

    // Initialize main CPU

    main_cpu.read=main_cpu_read;
    main_cpu.write=main_cpu_write;
    main_cpu.fault=cpu_fault;

    mainram[0xfffe]=0xfe;
    mainram[0xffff]=0;

    // mainioport[5]=1;

//    memset(mainioport,0xff,0x100);

    mc6809_reset(&main_cpu);

    scanline=0;

    // set virtual Hsync timer

    add_repeating_timer_us(63,hsync_handler,NULL  ,&timer);

    maincpu_wait=fm7clocks[fm7cpuclock*2];

    while(1) {

        if(menumode==0) {

        int rc=mc6809_step(&main_cpu);

            if(key_break_flag==1) main_cpu.firq=true;

    //    if(main_cpu.cycles>1350000) {

    //     // // while((video_vsync&1)==0);
    //     // // video_vsync&=2;

    //     unsigned char str[80];

    //     sprintf(str,"MAIN:%04x %d",main_cpu.pc.w,main_cpu.cycles);
    //     cursor_x=0;
    //     cursor_y=21;
    //     video_print(str);

    //     sprintf(str,"SUB :%04x %d %d",sub_cpu.pc.w,sub_cpu.cycles,sub_cpu_halt);
    //     cursor_x=0;
    //     cursor_y=20;
    //     video_print(str);


    //     }

            if((main_cpu.cycles-main_hsync_cycle)>maincpu_wait) {

                main_hsync_cycle=main_cpu.cycles;

                while((video_hsync&1)==0) ;
                video_hsync&=2;

            }
        }
        
    }
}

int main() {

    uint32_t sub_cpu_nmi_count=0;
    uint32_t menuprint=0;
    uint32_t filelist=0;
    uint32_t subcpu_wait;

 //   uint32_t lastscanline;

    set_sys_clock_khz(DOTCLOCK * CLOCKMUL ,true);

   stdio_init_all();

    uart_init(uart0, 115200);

    initVGA();

    gpio_set_function(12, GPIO_FUNC_UART);
    gpio_set_function(13, GPIO_FUNC_UART);

    // gpio_set_slew_rate(0,GPIO_SLEW_RATE_FAST);
    // gpio_set_slew_rate(1,GPIO_SLEW_RATE_FAST);
    // gpio_set_slew_rate(2,GPIO_SLEW_RATE_FAST);
    // gpio_set_slew_rate(3,GPIO_SLEW_RATE_FAST);
    // gpio_set_slew_rate(4,GPIO_SLEW_RATE_FAST);

    gpio_set_drive_strength(2,GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(3,GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(4,GPIO_DRIVE_STRENGTH_2MA);

    // Beep

//    gpio_init(6);
//    gpio_set_dir(6,GPIO_OUT);
//    gpio_put(6,0);

    gpio_set_function(6,GPIO_FUNC_PWM);
    pwm_slice_num = pwm_gpio_to_slice_num(6);

    pwm_set_wrap(pwm_slice_num, 256);
    pwm_set_chan_level(pwm_slice_num, PWM_CHAN_A, 0);
    pwm_set_enabled(pwm_slice_num, true);

    // set PSG timer

    add_repeating_timer_us(1000000/SAMPLING_FREQ,sound_handler,NULL  ,&timer2);

//    board_init();
    tuh_init(BOARD_TUH_RHPORT);

    sem_init(&dualport_lock,1,1);

    video_cls();

    video_hsync=0;
    video_vsync=0;

    video_mode=0;
    fbcolor=0x7;

//  FM-7 Initialize

    memset(subioport,0,16);
    memset(mainioport,0xff,256);

    mainram[0xfffe]=0xfe;
    mainram[0xffff]=0;

    mainioport[2]=0;
    mainioport[3]=0;
    mainioport[4]=0xff;
    mainioport[5]=0xff;

    mainioport[15]=0;

    mainioport[0x37]=0;
    for(int i=0;i<8;i++) {
        mainioport[0x38+i]=i;
    }
    rebuild_colorpalette();

    psg_reset(0);

    multicore_launch_core1(main_core1);

    multicore_lockout_victim_init();

// uart handler

    irq_set_exclusive_handler(UART0_IRQ,uart_handler);
    irq_set_enabled(UART0_IRQ,true);
    uart_set_irq_enables(uart0,true,false);

// mount littlefs
//    if(lfs_mount(&lfs,&PICO_FLASH_CFG)!=0) {
//        cursor_x=0;
//        cursor_y=0;
//        fbcolor=7;
//        video_print("Initializing LittleFS...");
//        // format
//        lfs_format(&lfs,&PICO_FLASH_CFG);
//        lfs_mount(&lfs,&PICO_FLASH_CFG);
//    }

//  setup emulator (for core 1)
    sub_cpu.read  = sub_cpu_read;
    sub_cpu.write = sub_cpu_write;
    sub_cpu.fault = cpu_fault;

    mc6809_reset(&sub_cpu);

    uint32_t cpuwait=0;

    subcpu_wait=fm7clocks[fm7cpuclock*2+1];

    while(1) {

        if(menumode==0) { // Emulator mode

        if(sub_cpu_halt==0) {

            int rc=mc6809_step(&sub_cpu);

            if(scroll_flag==1) {
                scroll(oldvramoffset,vramoffset);
            }

        }

        // unsigned char str[80];
        // sprintf(str,"SUB :%04x",sub_cpu.pc.w);
        // cursor_x=0;
        // cursor_y=23;
        // video_print(str);

        if((video_vsync&2)!=0) { // Timer
            tuh_task();
            video_vsync&=1;
            if((key_repeat_flag)&&(key_repeat_count!=0)) {
                if((scanline-key_repeat_count)==40*262) {

//                  fm7keypressed=fm7lastkeypressed;

                   if(mainioport[0x2]&1) {
                        main_cpu.irq=true;
                        key_irq=1;
                    } else {
                        sub_cpu.firq=true;
                    }


                } else if (((scanline-key_repeat_count)>40*262)&&((scanline-key_repeat_count)%(4*262)==0)) {

//                    fm7keypressed=fm7lastkeypressed;

                    if(mainioport[0x2]&1) {
                        main_cpu.irq=true;
                        key_irq=1;
                    } else {
                        sub_cpu.firq=true;
                    }


                }
            }
        }

        // Keyboard check

        if((sub_cpu.cycles-sub_hsync_cycle)>subcpu_wait){
 
            // if((scanline-sub_cpu_nmi_count)>315) {   // 315 = 20ms / 63.5us
            //     sub_cpu.nmi=true;
            //     sub_cpu_nmi_count=scanline;
            // }


        //     keycheck_count++;
        //     if(keycheck_count>0) {

        //         if(kbhit!=4) {
        //             keytimer++;
        //             if(keytimer>127) {
        //                 keytimer=0;
        //             }
        //             if(keytimer==127) {
        //                 if(kbhit>0) {
        //                     kbhit--;
        //                     if(kbhit==1) {
        //                         if(ioport[0xe0]&0x40) { 
        //                             cpu.irq=true;
        //                         }
        //                     }    
        //                 }
        //             }
        //             if(keydata[keytimer]==0) { // Key 
        //                 kbhit=4;
        //                 if(ioport[0xe0]&0x40) { 
        //                     cpu.irq=true;
        //                 }
        //             } 
        //         }
        //         keycheck_count=0;
        //     }

            sub_hsync_cycle=sub_cpu.cycles;

            while((video_hsync&2)==0);

            video_hsync|=1;

        //     if((save_enabled==2)&&((cpu.cycles-file_cycle)>FILE_THREHSOLD)) {
        //         lfs_file_close(&lfs,&lfs_file);
        //         save_enabled=0;
        //     }

        //     if((load_enabled==2)&&((cpu.cycles-file_cycle)>FILE_THREHSOLD)) {
        //         lfs_file_close(&lfs,&lfs_file);
        //         load_enabled=0;
        //     }


        } 

        } else { // Menu Mode

            unsigned char str[80];

            fbcolor=7;
            
            if(menuprint==0) {

                draw_menu();
                menuprint=1;
                filelist=0;
            }

            cursor_x=12;
            cursor_y=6;
            video_print("MENU");

            uint32_t used_blocks=lfs_fs_size(&lfs);
            sprintf(str,"Free:%d Blocks",(HW_FLASH_STORAGE_BYTES/BLOCK_SIZE_BYTES)-used_blocks);
            cursor_x=12;
            cursor_y=7;
            video_print(str);

            cursor_x=12;            
            cursor_y=9;
            if(menuitem==0) { fbcolor=0x70; } else { fbcolor=7; } 
            if(save_enabled==0) {
                video_print("SAVE: UART");
            } else {
                sprintf(str,"SAVE: %8s",filename);
                video_print(str);
            }
            cursor_x=12;
            cursor_y=10;

            if(menuitem==1) { fbcolor=0x70; } else { fbcolor=7; } 
            if(load_enabled==0) {
                video_print("LOAD: UART");
            } else {
                sprintf(str,"LOAD: %8s",filename);
                video_print(str);
            }

            cursor_x=12;
            cursor_y=11;

            if(menuitem==2) { fbcolor=0x70; } else { fbcolor=7; } 
            video_print("DELETE File");
            cursor_x=12;
            cursor_y=12;

            if(menuitem==3) { fbcolor=0x70; } else { fbcolor=7; } 
            video_print("Reset");

            cursor_x=12;
            cursor_y=13;

            if(menuitem==4) { fbcolor=0x70; } else { fbcolor=7; } 
            video_print("Power Cycle");

            if(filelist==0) {
                draw_files(-1,0);
                filelist=1;
            }

            while(video_vsync==0);

            video_vsync=0;

                tuh_task();

                if(keypressed==0x52) { // Up
                    keypressed=0;
                    if(menuitem>0) menuitem--;
                }

                if(keypressed==0x51) { // Down
                    keypressed=0;
                    if(menuitem<4) menuitem++; 
                }

                if(keypressed==0x28) {  // Enter
                    keypressed=0;

                    if(menuitem==0) {  // SAVE

                        if((load_enabled==0)&&(save_enabled==0)) {

                            uint32_t res=enter_filename();

                            if(res==0) {
                                lfs_file_open(&lfs,&lfs_file,filename,LFS_O_RDWR|LFS_O_CREAT);
                                save_enabled=1;
                                file_cycle=main_cpu.cycles;
                            }

                        } else if (save_enabled!=0) {
                            lfs_file_close(&lfs,&lfs_file);
                            save_enabled=0;
                        }
                        menuprint=0;
                    }

                    if(menuitem==1) { // LOAD

                        if((load_enabled==0)&&(save_enabled==0)) {

                            uint32_t res=file_selector();

                            if(res==0) {
                                lfs_file_open(&lfs,&lfs_file,filename,LFS_O_RDONLY);
                                load_enabled=1;
                                file_cycle=main_cpu.cycles;
                            }
                        } else if(load_enabled!=0) {
                            lfs_file_close(&lfs,&lfs_file);
                            load_enabled=0;
                        }
                        menuprint=0;
                    }

                    if(menuitem==2) { // Delete

                        if((load_enabled==0)&&(save_enabled==0)) {
                            uint32_t res=enter_filename();

                            if(res==0) {
                                lfs_remove(&lfs,filename);
                            }
                        }

                        menuprint=0;

                    }


                    if(menuitem==3) { // reset
                        menumode=0;
                        menuprint=0;
                        redraw();
                        mc6809_reset(&main_cpu);
                        mc6809_reset(&sub_cpu);
                    }

                    if(menuitem==4) { // power cycle
                        menumode=0;
                        menuprint=0;
                        redraw();

                        // memset(mainram,0,0x10000);
                        // memset(colorram,0,0x4000);
                        // memset(igram,0,0x1800);

                        mc6809_reset(&main_cpu);
                        mc6809_reset(&sub_cpu);

                    }



                }

                if(keypressed==0x45) {
                    keypressed=0;
                    menumode=0;
                    menuprint=0;
                    redraw();
                //  break;     // escape from menu
                }
            
        }

    }

}
