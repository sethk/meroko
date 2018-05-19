/* System Interface Board - Implementation

 This file is subject to the terms and conditions of the GNU General Public
 License.  See the file COPYING in the main directory of this archive for
 more details.

 $Id$
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <strings.h>
#include <signal.h>
#include <termios.h>
#include <time.h>
#include <sched.h>
#include <sys/resource.h>

#ifdef DISPLAY_FB
#include <linux/fb.h>
#include <linux/vt.h>
#include <linux/kd.h>
#endif

#include "sib.h"
#include "mem8.h"
#include "raven_cpu.h"

#ifdef DISPLAY_SDL
#include <SDL/SDL_keysym.h>
#include "sdl.h"
#endif

// Set host framebuffer resolution here
#ifndef FB_WIDTH
#define FB_WIDTH 1280
#endif
#ifndef FB_HEIGHT
#define FB_HEIGHT 1024
#endif

#include "meroko.h"
#include "nubus.h"

uint8 *NBD = (unsigned char *)&NUbus_Data; // Pointer to bytes of NUbus-Data

// REGISTERS
unsigned int keyboard_usart_status=0;
unsigned int keyboard_usart_command=0;
unsigned int keyboard_usart_control=0;
unsigned int keyboard_usart_mode=0;
unsigned char keyboard_usart_tx_data=0;
unsigned char keyboard_usart_rx_fifo[25];
unsigned char keyboard_usart_rx_bot=0;
unsigned char keyboard_usart_rx_top=0;

int extd_key=0;
unsigned long sib_retrace_pi=0;
unsigned long snd_parity_pi=0;

unsigned int sib_config_reg=0;
unsigned int sib_rtc_int_control=0;
unsigned int sib_rtc_int_status=0;
unsigned int sib_rtc_int_sent=0;
unsigned int sib_sound_control=0;
unsigned int sib_int_diag_reg=0;
unsigned int sib_int_diag_data=0;
unsigned int sib_vox_data_reg=0;
unsigned int sib_monitor_ctl_reg=0;
unsigned int sib_speech_data_reg=0;

unsigned long sib_mask_reg=0;
unsigned long sib_opn_reg=0;

unsigned char sib_mouse_motion_reg=0;
unsigned int sib_mouse_x_pos=0;
unsigned int sib_mouse_y_pos=0;

unsigned long sib_itimer_ctr_0=0xFFFF; // Counter
unsigned int  sib_itimer_ctl_0=0; // Control Register
unsigned int  sib_itimer_ena_0=0; // Interrupt Enable
unsigned int  sib_itimer_lod_0=0; // Load flags
unsigned int  sib_itimer_lat_0=0; // Latched Timer, for read
unsigned int  sib_itimer_red_0=0; // Read flags

unsigned long sib_itimer_ctr_1=0;
unsigned long sib_itimer_ped_1=0;
unsigned int  sib_itimer_ctl_1=0;
unsigned int  sib_itimer_ena_1=0;
unsigned int  sib_itimer_lod_1=0;
unsigned int  sib_itimer_lvl_1=0;
unsigned int  sib_itimer_edg_1=0;

unsigned long sib_itimer_ctr_2=0;
unsigned int  sib_itimer_ctl_2=0;
unsigned int  sib_itimer_ena_2=0;
unsigned int  sib_itimer_lod_2=0;

// THESE ARE BCD NUMBERS!
unsigned long sib_rtc_100ns=0; // 2 digits, lower digit unused. This means we are actually counting 1000ns!
unsigned long sib_rtc_ticks=0; // NOT A REAL REGISTER - HERE FOR MY CONVENIENCE
unsigned long sib_rtc_10ms=0;  // 2 digits, 100ms and then 10ms
unsigned long sib_rtc_sec=0;   // 2 digits for all the rest
unsigned long sib_rtc_min=0;
unsigned long sib_rtc_hour=0;
unsigned long sib_rtc_dow=0;
unsigned long sib_rtc_date=0;
unsigned long sib_rtc_month=0;
// THESE ARE BCD NUMBERS!
unsigned long sib_ram_100ns=0;
unsigned long sib_ram_10ms=0;
unsigned long sib_ram_sec=0;
unsigned long sib_ram_min=0;
unsigned long sib_ram_hour=0;
unsigned long sib_ram_dow=0;
unsigned long sib_ram_date=0;
unsigned long sib_ram_month=0;

unsigned int end_of_month[12] = { 0x31,0x28,0x31,0x30,0x31,0x30,0x31,0x31,0x30,0x31,0x30,0x31 };

long previous_nsec;
struct timespec current_time;

unsigned long sib_rtc_int_vector=0;
unsigned long sib_sintv_expired_vector=0;
unsigned long sib_lintv_expired_vector=0;
unsigned long sib_serio_status_vector=0;
unsigned long sib_lpt_ack_vector=0;
unsigned long sib_gcmd_ack_vector=0;
int           sib_gcmd_ack_sent=0;
unsigned long sib_kbd_rtt_vector=0;
int           sib_kbd_rtt_sent=0;
unsigned long sib_psu_overtemp_vector=0;
unsigned long sib_kbd_bootchord_vector=0;
unsigned long sib_mouse_moved_vector=0;
int           sib_mouse_moved_sent=0;
unsigned long sib_mouse_button_vector=0;
int           sib_mouse_button_sent=0;
unsigned long sib_voxdata_present_vector=0;
unsigned long sib_sound_parity_vector=0;
unsigned long sib_fiber_link_vector=0;
unsigned long sib_power_fail1_vector=0;
unsigned long sib_power_fail2_vector=0;

unsigned char sib_video_attr=0;
unsigned char sib_crt_r00=0x2a;
unsigned char sib_crt_r01=0x1f;
unsigned char sib_crt_r02=0x07;
unsigned char sib_crt_r03=0x0a;
unsigned char sib_crt_r04=0x24;
unsigned char sib_crt_r05=0x25;
unsigned char sib_crt_r06=0x80;
unsigned char sib_crt_r07=0x64;
unsigned char sib_crt_r08=0x67;
unsigned char sib_crt_r09=0x4a;
unsigned char sib_crt_r0a=0;
unsigned char sib_crt_r0b=0;
unsigned char sib_crt_r0c=0;
unsigned char sib_crt_r0d=0x40;
unsigned char sib_crt_r0e=0;
unsigned char sib_crt_r0f=0;
unsigned char sib_crt_r10=0x80;
unsigned char sib_crt_r11=0x80;
unsigned char sib_crt_r12=0x80;
unsigned char sib_crt_r13=0;
unsigned char sib_crt_r14=0;
unsigned char sib_crt_r15=0;
unsigned char sib_crt_r16=0;
unsigned char sib_crt_r17=0;
unsigned char sib_crt_r18=0;
unsigned char sib_crt_r19=0;
unsigned char sib_crt_r1a=0;

unsigned char sib_lpt_r00=0;

// Software
unsigned char SIB_ROM[0x2000];
unsigned char NVRAM[2048];
unsigned char VRAM[0x20000]; /* 8K * 16, for 1024x1024 px */

int sib_log_arm = 0;

#ifdef DISPLAY_FB
#define PIX_Z	0x00
#define PIX_NZ	0x0f
#endif

#ifdef DISPLAY_SDL
#define PIX_Z	0x00
#define PIX_NZ	0xff
#endif

#define PROM_FNAME "proms/2236662_SIB"
#define NVRAM_FNAME "NVRAM.BIN"

// FRAMEBUFFER CONTROLS
int fbfd=0,consfd=0;
char *framebuffer = 0;
unsigned long int screensize=0;
long iodata=0;
#ifdef DISPLAY_FB
struct fb_var_screeninfo vinfo;
struct fb_fix_screeninfo finfo;
#endif

#ifdef PS2MOUSE
// PS2 Mouse Support
#define MOUSE_FNAME "/dev/psaux"
int mousefd=0;
unsigned char mousecmd[10];
unsigned char mousedta[100];
int mouseidx=0;
#endif

#define sib_nubus_io_request(access, address, data) \
	nubus_io_request((access), (address), (data), NUBUS_ID_SIB);

// NVRAM SAVE AND LOAD
void save_nvram(){
  FILE *romfile;
  int x=0;
  romfile = fopen(NVRAM_FNAME,"w+");
  if(romfile == NULL){
    return; // No big deal
  }
  while(x < 2048){
    fwrite(&NVRAM[x],1,1,romfile);
    x++;
  }
  fclose(romfile);  
}

void load_nvram(){
  FILE *romfile;
  int x=0;
  romfile = fopen(NVRAM_FNAME,"r");
  if(romfile == NULL){
    return; // No big deal
  }
  while(x < 2048){
    fread(&NVRAM[x],1,1,romfile);
    x++;
  }
  fclose(romfile);  
}

// **** NEW RTC / ITIMER CODE ****

 unsigned char dec_to_bcd(int n) {
   return (((((n)/10)&0xf)<<4)|(((n)%10)&0xf));
 }

 void sib_rtc_initialize(){
   time_t tnow = time(NULL);
   struct tm *now = localtime(&tnow);

   sib_rtc_sec = dec_to_bcd(now->tm_sec);
   sib_rtc_min = dec_to_bcd(now->tm_min);
   sib_rtc_hour = dec_to_bcd(now->tm_hour);
   sib_rtc_dow = dec_to_bcd(now->tm_wday);
   if (now->tm_mon == 1 && now->tm_mday == 29)
     sib_rtc_date = dec_to_bcd(now->tm_mday-1);
   else
     sib_rtc_date = dec_to_bcd(now->tm_mday);
   sib_rtc_month = sib_ram_month = dec_to_bcd(now->tm_mon+1);
   /* Hack */
   sib_ram_10ms = dec_to_bcd((now->tm_year)%100);
   sib_ram_100ns = (((now->tm_year/100))<<5) | (((now->tm_mon == 1 && now->tm_mday == 29)&1)<<4);
 }

void sib_updateslow(){
  // Kill faster timers
  // sib_rtc_100ns=0;
  // sib_rtc_10ms=0;
  // Add a second
  sib_rtc_sec++;
  // RTC 1SEC
  if((sib_rtc_sec&0xF) > 9){
    sib_rtc_sec += 0x10;
    sib_rtc_sec &= 0xF0;
  }else{return;}
  // RTC 10SEC
  if((sib_rtc_sec&0xF0) > 0x50){
    sib_rtc_min++;
    sib_rtc_sec &= 0; // Reset
    // Check for interrupt
    if(sib_rtc_int_control&0x08){
      logmsg("SIB: 1min Interrupt\n");      
      sib_rtc_int_control ^= 0x08; // Disable future interrupt
      sib_rtc_int_status |= 0x08;  // Tell CPU what happened
      // Write 0xFF to this location if interrupts are enabled
      if(sib_config_reg&0x02 && !sib_rtc_int_sent){
        sib_nubus_io_request(NB_BYTE_WRITE, sib_rtc_int_vector, 0xFF);
        sib_rtc_int_sent=1;
      }
    }     
  }else{return;}
  // RTC 1MIN
  if((sib_rtc_min&0xF) > 9){
    sib_rtc_min += 0x10;
    sib_rtc_min &= 0xF0;
  }else{return;}
  // RTC 10MIN
  if((sib_rtc_min&0xF0) > 0x50){
    sib_rtc_hour++;
    sib_rtc_min &= 0x0F;
    // Check for interrupt
    if(sib_rtc_int_control&0x10){
      logmsg("SIB: 1hr Interrupt\n");      
      sib_rtc_int_control ^= 0x10; // Disable future interrupt
      sib_rtc_int_status |= 0x10;  // Tell CPU what happened
      // Write 0xFF to this location if interrupts are enabled
      if(sib_config_reg&0x02 && !sib_rtc_int_sent){
        sib_nubus_io_request(NB_BYTE_WRITE, sib_rtc_int_vector, 0xFF);
        sib_rtc_int_sent=1;
      }
    }     
  }else{return;}
  // RTC 1HOUR
  if((sib_rtc_hour&0xF) > 9){
    sib_rtc_hour += 0x10;
    sib_rtc_hour &= 0xF0;
  }
  // RTC 24HOUR CHECK
  if(sib_rtc_hour > 0x23){
    sib_rtc_dow++;
    sib_rtc_date++;
    sib_rtc_hour=0;
    // Check for interrupt
    if(sib_rtc_int_control&0x20){
      logmsg("SIB: 1day Interrupt\n");      
      sib_rtc_int_control ^= 0x20; // Disable future interrupt
      sib_rtc_int_status |= 0x20;  // Tell CPU what happened
      // Write 0xFF to this location if interrupts are enabled
      if(sib_config_reg&0x02 && !sib_rtc_int_sent){
        sib_nubus_io_request(NB_BYTE_WRITE, sib_rtc_int_vector, 0xFF);
        sib_rtc_int_sent=1;
      }
    }     
  }else{return;}
  // RTC DOW
  if(sib_rtc_dow > 6){
    sib_rtc_dow = 0;
    // Check for interrupt
    if(sib_rtc_int_control&0x40){
      logmsg("SIB: 1wk Interrupt\n");      
      sib_rtc_int_control ^= 0x40; // Disable future interrupt
      sib_rtc_int_status |= 0x40;  // Tell CPU what happened
      // Write 0xFF to this location if interrupts are enabled
      if(sib_config_reg&0x02 && !sib_rtc_int_sent){
        sib_nubus_io_request(NB_BYTE_WRITE, sib_rtc_int_vector, 0xFF);
        sib_rtc_int_sent=1;
      }
    }     
  }
  // RTC 1DAY
  if((sib_rtc_date&0xF) > 9){
    sib_rtc_date += 0x10;
    sib_rtc_date &= 0xF0;
  }
  // RTC END-OF-MONTH
  {
    int month = (sib_rtc_month&0xF)+(10*((sib_rtc_month&0xF0)>>4));
    if(sib_rtc_date > end_of_month[month]){
      sib_rtc_date = 0; // Clear
      sib_rtc_month++;
      // Check for interrupt
      if(sib_rtc_int_control&0x80){
        logmsg("SIB: 1month Interrupt\n");      
        sib_rtc_int_control ^= 0x80; // Disable future interrupt
        sib_rtc_int_status |= 0x80;  // Tell CPU what happened
        // Write 0xFF to this location if interrupts are enabled
        if(sib_config_reg&0x02 && !sib_rtc_int_sent){
          sib_nubus_io_request(NB_BYTE_WRITE, sib_rtc_int_vector, 0xFF);
          sib_rtc_int_sent=1;
        }
      }     
    }else{return;}
  }
  // RTC 1MONTH
  if((sib_rtc_month&0xF) > 9){
    sib_rtc_month += 0x10;
  }
  // RTC 12MONTH
  if(sib_rtc_month > 0x12){
    sib_rtc_month = 0;
  }
}

void usrhandler(int arg){
  signal(SIGUSR1,usrhandler);
}

unsigned char lastchar=0;

#ifdef DISPLAY_SDL

static unsigned short map[512];

void
init_sdl_to_keysym_map(void)
{
  map[0x12] = 1;
  map['1'] = 0x24; // 1
  map['2'] = 0x25; // 2
  map['3'] = 0x26; // 3
  map['4'] = 0x27; // 4
  map['5'] = 0x28; // 5
  map['6'] = 0x29; // 6
  map['7'] = 0x2a; // 7
  map['8'] = 0x2b; // 8
  map['9'] = 0x2c; // 9
  map['0'] = 0x2d; // 0
  map['-'] = 0x2e; // -
//  map['+'] = 0x2f; // + is shifted =
  map['='] = 0x2f;              /* not keypad-equal, real equals */

  map['Q'] = 0x39; // Q
  map['W'] = 0x3a; // W
  map['E'] = 0x3b; // E
  map['R'] = 0x3c; // R
  map['T'] = 0x3d; // T
  map['Y'] = 0x3e; // Y
  map['U'] = 0x3f; // U
  map['I'] = 0x40; // I
  map['O'] = 0x41; // O
  map['P'] = 0x42; // P

  map['A'] = 0x50; // A
  map['S'] = 0x51; // S
  map['D'] = 0x52; // D
  map['F'] = 0x53; // F
  map['G'] = 0x54; // G
  map['H'] = 0x55; // H
  map['J'] = 0x56; // J
  map['K'] = 0x57; // K
  map['L'] = 0x58; // L

  map['Z'] = 0x68; // Z
  map['X'] = 0x69; // X
  map['C'] = 0x6a; // C
  map['V'] = 0x6b; // V
  map['B'] = 0x6c; // B
  map['N'] = 0x6d; // N
  map['M'] = 0x6e; // M

  map['['] = 0x43;              /* special treatment below */
  map[']'] = 0x44;              /* ditto */
  map[';'] = 0x27;
  map['`'] = 060;               /* octal, mind you */
  map['~'] = 061;               /* special treatment (this is too useful to remap) */
  map['\\'] = 0x46;
  map[' '] = 0x7b;

  map[','] = 0x6f;
  map['.'] = 0x70;
  map['/'] = 0x71;
  map[';'] = 0x59;
  map['\''] = 0x5a;

  map[SDLK_RETURN] = 0x5b; // ENTER
  map[SDLK_BACKSPACE] = 0x4f; // BACKSPACE (MAPS TO RUBOUT)
  map[SDLK_TAB] = 0x35; // TAB
  map[SDLK_ESCAPE] = 0x23; // ESC

  map[SDLK_UP] = 0x47; // up arrow
  map[SDLK_DOWN] = 0x75; // down arrow
  map[SDLK_RIGHT] = 0x5f; // right arrow
  map[SDLK_LEFT] = 0x5d; // left arrow
  map[SDLK_HOME] = 0x15; /* HOME - F1 */
  map[SDLK_INSERT] = 0x14; /* INSERT - F2 */

  map[SDLK_PAGEUP] = 0x4c; /* PAGEUP - ABORT */
  map[SDLK_PAGEDOWN] = 0x21; /* PAGEDOWN - RESUME */

  map[SDLK_F1] = 0x08; /* F1 - SYSTEM */
  map[SDLK_F2] = 0x09; /* F2 - NETWORK */
  map[SDLK_F3] = 0x0a; /* F3 - STATUS */
  map[SDLK_F4] = 0x0b; /* F4 - TERMINAL */
  map[SDLK_F5] = 0x01; /* F5 - HELP */
  map[SDLK_F6] = 0x0e;		/* F6 - Clear Input */

  map[SDLK_CAPSLOCK] = 0x03; // CAPSLOCK

  map[SDLK_RSHIFT] = 0x67e7; // RIGHT SHIFT
  map[SDLK_LSHIFT] = 0x72f2; // LEFT SHIFT
#if 1                           /* why are these swapped (up/down)? */
map[SDLK_RSHIFT] = 0xe767; // RIGHT SHIFT
map[SDLK_LSHIFT] = 0xf272; // LEFT SHIFT
#endif
  map[SDLK_RCTRL] = 0x1c; // RIGHT CTRL
  map[SDLK_LCTRL] = 0x1c; // LEFT CTRL
  map[SDLK_RALT] = 0x1b; // RIGHT ALT (META)
  map[SDLK_LALT] = 0x1b; // LEFT ALT (META)

#if 1
  map[SDLK_LMETA] = 0x1a; // LEFT CMD (META)
  map[SDLK_RMETA] = 0x1a; // LEFT CMD (META)
#endif

  map[SDLK_RSUPER] = 0x1a; // RIGHT windows (SUPER)
  map[SDLK_LSUPER] = 0x1a; // LEFT windows (SUPER)

}

static int
put_rx_ring(unsigned char ch)
{
  // If reciever's running, take the key
  if ((keyboard_usart_command&0x04) == 0) {
    return 1;
  }

  keyboard_usart_rx_fifo[keyboard_usart_rx_top++] = ch;
  if (keyboard_usart_rx_top > 25) {
    keyboard_usart_rx_top = 0;
  }
  return 0;
}

void kbd_handle_char(int scancode, int symcode, int down, int shift)
{
  int sdlchar = symcode;
  unsigned char outchar=0;

  if (map[0x12] == 0)
	  init_sdl_to_keysym_map();

  /* for now, fold lower case to upper case */
  /* (because we're ignoring modifiers) */
  if (sdlchar >= 'a' && sdlchar <= 'z')
	  sdlchar -= ' ';

  if (down && map[sdlchar] > 0xff)
    outchar = map[sdlchar] >> 8;
  else
    outchar = map[sdlchar] | (down ? 0x80 : 0);

  if (outchar == 0) {
	  logmsgf("Keystroke lost (%x) - unmapped keycode\n", sdlchar);
          return;
  } else {
    if (0) logmsgf("sdl %x (%x) -> %x\n",sdlchar, down, outchar);
  }

  /* Gross hack to handle keys with other shifting: US KBD specific! */
  /* Need shift-hack for
     [] (shift () unshifted)
     {} (shift `~)
     ~ (unshifted)
   */
  if (!shift && (sdlchar == '[' || sdlchar == ']')) {
    /* unshifted [] become shifted (unshifted) parentheses */
    unsigned char shift = (down ? (map[SDLK_LSHIFT] >> 8) :
                           (map[SDLK_LSHIFT] & 0xff));
#if 1
    logmsgf("Shifting explicitly: %x %x %x\n",
            shift, outchar, shift ^ 0x80);
#endif
    put_rx_ring(shift);
    put_rx_ring(outchar);
    put_rx_ring(shift ^ 0x80);
  } else if (shift && (sdlchar == '[' || sdlchar == ']')) {
    /* shifted [] is {}, which becomes shifted `~ */
    outchar = (sdlchar == '[' ? map['`'] : map['~']) | (down ? 0x80 : 0);
#if 1
    logmsgf("Remapping shift %x to %x\n", sdlchar, outchar);
#endif
    put_rx_ring(outchar);
  } else if (shift && sdlchar == '`') {
    /* shifted ` is (unshifted) ~ */
    unsigned char shift = (down ? (map[SDLK_LSHIFT] & 0xff) :
                           (map[SDLK_LSHIFT] >> 8));
#if 1
    logmsgf("Unshifting explicitly: %x %x %x\n",
            shift, map['~'], shift ^ 0x80);
#endif
    put_rx_ring(shift);
    put_rx_ring(map['~'] | (down ? 0x80 : 0));
    put_rx_ring(shift ^ 0x80);
  } else
  if (put_rx_ring(outchar)) {
    logmsgf("Keystroke lost (%x) - kbd not operating\n", outchar);
  }
}
#endif

/*MADCB11B */

void kbdhandler(int arg){
  unsigned char iochar[3];
  unsigned char outchar=0;
  int done=0;

  // Recapture signal
  signal(SIGIO,kbdhandler);
  // Get keycode
  while(done==0){
    outchar=0;
    if(read(consfd,&iochar[0],1)==1){
      // Remap and handle
      if(extd_key==0){
        switch(iochar[0]){	
        case 0x01: // ESCAPE
          outchar = 0x23; break;
        case 0x81:
          outchar = 0xA3; break;

        case 0x02: // 1
        case 0x82:
        case 0x03: // 2
        case 0x83:
        case 0x04: // 3
        case 0x84:
        case 0x05: // 4
        case 0x85:
        case 0x06: // 5
        case 0x86:
        case 0x07: // 6
        case 0x87:
        case 0x08: // 7
        case 0x88:
        case 0x09: // 8
        case 0x89:
        case 0x0A: // 9
        case 0x8A:
        case 0x0B: // 0
        case 0x8B:
        case 0x0C: // -
        case 0x8C:
        case 0x0D: // +
        case 0x8D:
          outchar = iochar[0]+0x22; break;

        case 0x0E: // BACKSPACE (MAPS TO RUBOUT)
          outchar = 0x4F; break;
        case 0x8E: 
          outchar = 0xCF; break;
        case 0x0F: // TAB
          outchar = 0x35; break;
        case 0x8F: 
          outchar = 0xB5; break;
          
        case 0x10: // Q
        case 0x90:
        case 0x11: // W
        case 0x91:
        case 0x12: // E
        case 0x92:
        case 0x13: // R
        case 0x93:
        case 0x14: // T
        case 0x94:
        case 0x15: // Y
        case 0x95:
        case 0x16: // U
        case 0x96:
        case 0x17: // I
        case 0x97:
        case 0x18: // O
        case 0x98:
        case 0x19: // P
        case 0x99:
          outchar = iochar[0]+0x29; break;

        case 0x1A: // [
          outchar = 0x43; break;
        case 0x9A:
          outchar = 0xC3; break;
        case 0x1B: // ]
          outchar = 0x44; break;
        case 0x9B:
          outchar = 0xC4; break;
        case 0x1C: // ENTER
          outchar = 0x5B; break;
        case 0x9C:
          outchar = 0xDB; break;
          
        case 0x1D: // LEFT CTRL
          outchar = 0x1C; break;
        case 0x9D:
          outchar = 0x9C; break;
                    
        case 0x1E: // A
        case 0x9E:
        case 0x1F: // S
        case 0x9F:
        case 0x20: // D
        case 0xA0:
        case 0x21: // F
        case 0xA1:
        case 0x22: // G
        case 0xA2: 
        case 0x23: // H
        case 0xA3:
        case 0x24: // J
        case 0xA4:
        case 0x25: // K
        case 0xA5:
        case 0x26: // L
        case 0xA6:
          outchar = iochar[0]+0x32; break;

        case 0x27: // ;
          outchar = 0x59; break;
        case 0xA7:
          outchar = 0xD9; break;
        case 0x28: // ' and "
          outchar = 0x5A; break;
        case 0xA8: 
          outchar = 0xDA; break;
        case 0x29: // TILDE 
          outchar = 0x66; break; // MAPS TO SYMBOL
        case 0xA9:
          outchar = 0xE6; break;          
        case 0x2A: // LEFT SHIFT
          outchar = 0x67; break;
        case 0xAA:
          outchar = 0xE7; break;
        case 0x2B: // PIPE / BACKSLASH
          outchar = 0x46; break;
        case 0xAB:
          outchar = 0xC6; break;

        case 0x2C: // Z
        case 0xAC:
        case 0x2D: // X
        case 0xAD:
        case 0x2E: // C
        case 0xAE:
        case 0x2F: // V
        case 0xAF:
        case 0x30: // B
        case 0xB0:
        case 0x31: // N
        case 0xB1:
        case 0x32: // M
        case 0xB2:
          outchar = iochar[0]+0x3C; break;

        case 0x33: // ,
          outchar = 0x6F; break;
        case 0xB3:
          outchar = 0xEF; break;
        case 0x34: // .
          outchar = 0x70; break;
        case 0xB4: 
          outchar = 0xF0; break;
        case 0x35: // slash
          outchar = 0x71; break;
        case 0xB5: 
          outchar = 0xF1; break;

        case 0x36: // RIGHT SHIFT
          outchar = 0x72; break;
        case 0xB6:
          outchar = 0xF2; break;

        case 0x38: // LEFT ALT (META)
          outchar = 0x1B; break;
        case 0xB8: 
          outchar = 0x9B; break;
        case 0x39: // SPACE
          outchar = 0x7B; break;
        case 0xB9:
          outchar = 0xFB; break;
        case 0x3A: // CAPSLOCK
          outchar = 0x03; break;
        case 0xBA: 
          outchar = 0x83; break;

        case 0x3B: // F1
          outchar = 0x08; break; // MAPS TO SYSTEM
        case 0xBB:
          outchar = 0x88; break;
        case 0x3C: // F2
          outchar = 0x09; break; // NETWORK
        case 0xBC:
          outchar = 0x89; break;
        case 0x3D: // F3
          outchar = 0x0A; break; // STATUS
        case 0xBD:
          outchar = 0x8A; break;
        case 0x3E: // F4
          outchar = 0x0B; break; // TERMINAL
        case 0xBE:
          outchar = 0x8B; break;
        case 0x3F: // F5
          outchar = 0x01; break; // HELP
        case 0xBF:
          outchar = 0x81; break;
        case 0x40: // F6
          outchar = 0x21; break; // RESUME
        case 0xC0:
          outchar = 0xA1; break;
        case 0x41: // F7
          outchar = 0x36; break; // BREAK
        case 0xC1:
          outchar = 0xB6; break;
        case 0x42: // F8
          outchar = 0x4C; break; // ABORT
        case 0xC2:
          outchar = 0xCC; break;

        case 0xE0: // E0 means EXTENDED KEYSTROKE COMING
          extd_key = 1; 
          outchar = 0;
          break; // READ NEXT BYTE FOR KEY
          
        default:
          logmsgf("Got unmapped keycode 0x%X\n",iochar[0]);
        }
      }else{
        // EXTENDED KEYS
        switch(iochar[0]){	

        case 0x1D: // RIGHT CONTROL
          outchar = 0x1D; break; // HAH!
        case 0x9D:
          outchar = 0x9D; break;

        case 0x38: // RIGHT ALT
          outchar = 0x1E; break;
        case 0xB8:
          outchar = 0x9E; break;

        case 0x47: // HOME KEY (MAPS F2)
          outchar = 0x15; break;
        case 0xC7:
          outchar = 0x95; break;
        case 0x48: // UP ARROW
          outchar = 0x47; break;
        case 0xC8:
          outchar = 0xC7; break;

        case 0x4B: // LEFT ARROW
          outchar = 0x5D; break;
        case 0xCB:
          outchar = 0xDD; break;

        case 0x4D: // RIGHT ARROW
          outchar = 0x5F; break;
        case 0xCD:
          outchar = 0xDF; break;

        case 0x50: // DOWN ARROW
          outchar = 0x75; break;
        case 0xD0:
          outchar = 0xF5; break;
        case 0x51: // PAGE DOWN KEY
          outchar = 0x30; break; // Maps to { BACKSLASH
        case 0xD1:
          outchar = 0xB0; break;
        case 0x52: // INSERT KEY (MAPS F1)
          outchar = 0x14; break;
        case 0xD2:
          outchar = 0x94; break;
        case 0x53: // DELETE KEY 
          outchar = 0x31; break; // Maps to } ~
        case 0xD3:
          outchar = 0xB1; break;
          
        case 0x5B: // LEFT WINDOWS FLAG (MAPS TO SUPER)
          outchar = 0x1A; break; // Maps to SUPER
        case 0xDB: 
          outchar = 0x9A; break; 
        case 0x5C: // RIGHT WINDOWS FLAG (RIGHT SUPER)
          outchar = 0x1F; break;
        case 0xDC: 
          outchar = 0x9F; break; 
        case 0x5D: // MENU KEY (HYPER)
          outchar = 0x20; break;
        case 0xDD: 
          outchar = 0xA0; break; 

        default:
          logmsgf("Got unmapped extended-keycode 0x%X\n",iochar[0]);
        }
        extd_key = 0;
      }    
      if(outchar != 0 && (keyboard_usart_command&0x04) != 0){
	// If reciever's running, take the key
        // MAKE/BREAK IS REVERSED FROM PC KEYS!
        outchar ^= 0x80;
        if(outchar != lastchar){ // Key repeat is in software
          keyboard_usart_rx_fifo[keyboard_usart_rx_top] = outchar;
          keyboard_usart_rx_top++;
          if(keyboard_usart_rx_top > 25){ keyboard_usart_rx_top = 0; }
        }
        lastchar = outchar;
      }else{
        // Check for extended keystrokes and don't print message
        if(!extd_key){
          logmsg("Keystroke lost - Unknown key or reciever not operating\n");
        }
      }
      iochar[0]=0;
    }else{
      done=1;
    }
  }
#ifdef PS2MOUSE
  // Test mouse
  while(read(mousefd,&mousedta[mouseidx],1) == 1){
    // Top?
    if(mouseidx==0){
      // Look for start of a packet
      if(mousedta[mouseidx]&0x8){// Has check bit
        // Have it
        mouseidx++;
      }
    }else{
      // Read anything
      mouseidx++;
      if(mouseidx==4){
        /* ImPS2 Mouse Packet:
           Byte 0: Status Data
             0x80 - Y Overflow
             0x40 - X Overflow
             0x20 - Y Sign Bit
             0x10 - X Sign Bit
             0x08 - CHECK BIT (Always 1)
             0x04 - Middle Button
             0x02 - Right Button
             0x01 - Left Button
           Byte 1: X-Movement (Left/right, negative means left.)
           Byte 2: Y-Movement (Up/down, negative means down.)
           Byte 3: Z-Movement (The scroll wheel)            
        */
        // Are we in loopback? 
        if((sib_int_diag_reg&0x0E)==0){
          unsigned int keydata;
          // No
          // logmsgf("MOUSE PACKET: Buttons %X XMv %X YMv %X WMv %X\n",mousedta[0]&7,mousedta[1],mousedta[2],mousedta[3]);
          // Did it move?
          if(mousedta[1] != 0 || mousedta[2] != 0){
            // Move X
            sib_mouse_x_pos += (char)mousedta[1];
            // Move Y            
            sib_mouse_y_pos -= (char)mousedta[2];
            
            // Enforce 16-bitness
            if(sib_mouse_y_pos&0x80000000){ // Wrapped negative
              sib_mouse_y_pos = 0; // Don't.
            }else{              
              sib_mouse_y_pos &= 0xFFFF; // Truncate
            }
            if(sib_mouse_x_pos&0x80000000){
              sib_mouse_x_pos = 0;
            }else{
              sib_mouse_x_pos &= 0xFFFF;
            }
            // Update
            // logmsgf("Mouse moved %d x %d, to %ld x %ld\n",(char)mousedta[1],(char)mousedta[2],sib_mouse_x_pos,sib_mouse_y_pos);
            // Interrupt?          
            if ((sib_config_reg&2) && (sib_int_diag_reg&0x40) && sib_mouse_moved_vector != 0 && sib_mouse_moved_sent == 0) {
              // logmsgf("SIB: Sending MOUSE MOVED interrupt\n");
              sib_nubus_io_request(NB_BYTE_WRITE, sib_mouse_moved_vector, 0xFF);
              sib_mouse_moved_sent = 1;
            }
          }
          // Update button presses
          keydata  =   mousedta[0]&0x1;      keydata <<= 1; // Left
          keydata |= ((mousedta[0]&0x4)>>2); keydata <<= 1; // Middle
          keydata |= ((mousedta[0]&0x2)>>1);                // Right
          keydata <<= 4;
          if(keydata != (sib_mouse_motion_reg&0x70)){
            // logmsgf("Mouse buttons changed: %X\n",keydata);
            // Update register
            sib_mouse_motion_reg &= 0x8F;
            sib_mouse_motion_reg |= keydata;
            if ((sib_config_reg&2) && (sib_int_diag_reg&0x20) && sib_mouse_button_vector != 0 && sib_mouse_button_sent == 0) {
              // logmsgf("SIB: Sending MOUSE KEYSWITCH interrupt\n");
              sib_nubus_io_request(NB_BYTE_WRITE, sib_mouse_button_vector, 0xFF);
              sib_mouse_button_sent = 1;
            }
          }
        }
        mouseidx=0;
      }
    }
  }
#endif
}

#ifdef DISPLAY_SDL
void
sib_mouse_event(int x, int y, int dx, int dy, int buttons)
{
  unsigned int keydata;

  sib_mouse_x_pos = x;
  sib_mouse_y_pos = y;

  if ((sib_config_reg&2) &&
      (sib_int_diag_reg&0x40) &&
      sib_mouse_moved_vector != 0 &&
      sib_mouse_moved_sent == 0)
  {
    // logmsgf("SIB: Sending MOUSE MOVED interrupt\n");
    sib_nubus_io_request(NB_BYTE_WRITE, sib_mouse_moved_vector, 0xFF);
    sib_mouse_moved_sent = 1;
  }

  keydata  =   buttons & 1;      keydata <<= 1; // Left
  keydata |= ((buttons & 4)>>2); keydata <<= 1; // Middle
  keydata |= ((buttons & 2)>>1);                // Right
  keydata <<= 4;

  if (keydata != (sib_mouse_motion_reg&0x70)) {
    // logmsgf("Mouse buttons changed: %X\n",keydata);
    // Update register
    sib_mouse_motion_reg &= 0x8F;
    sib_mouse_motion_reg |= keydata;
    if ((sib_config_reg&2) &&
        (sib_int_diag_reg&0x20) &&
        sib_mouse_button_vector != 0 &&
        sib_mouse_button_sent == 0)
    {
      // logmsgf("SIB: Sending MOUSE KEYSWITCH interrupt\n");
      sib_nubus_io_request(NB_BYTE_WRITE, sib_mouse_button_vector, 0xFF);
      sib_mouse_button_sent = 1;
    }
  }
}
#endif


void sib_init(){
  FILE *romfile;
  int x=0;
  romfile = fopen(PROM_FNAME,"r");
  if(romfile == NULL){
    perror("sib-fopen");
    exit(-1);
  }
  while(x < 0x2000){
    fread(&SIB_ROM[x],1,1,romfile);
    x++;
  }
  fclose(romfile);
  /* NVRAM */
  load_nvram();
  /* REGISTER RESETS */
  keyboard_usart_status = 0x0; // transmit and recieve disabled
  keyboard_usart_tx_data = 0;  
  keyboard_usart_rx_fifo[0]=0;
  keyboard_usart_rx_bot=0;
  keyboard_usart_rx_top=0;

  sib_rtc_initialize();

  /* CONSOLE SETUP */
  // Arrange for SIGIO on IO availability
  signal(SIGIO,kbdhandler);
  signal(SIGUSR1,usrhandler);

#ifdef DISPLAY_FB
  // Become a realtime process (Well, as close as timesharing Linux can get...)
  if(setpriority(PRIO_PROCESS,getpid(),-20)<0){
    perror("setpriority");
    exit(-1);
  }

  // Open console and keyboard FD
  consfd = open("/dev/tty0",O_RDWR|O_NONBLOCK);
  if(!consfd){
    perror("console-open");
    exit(-1);
  }

  // Disassociate from process group
  setpgrp();
  
  // Detach CTY
  {
    int fd=0;
    fd=open("/dev/tty",O_RDWR);
    if(ioctl(fd,TIOCNOTTY,0)){
      perror("ioctl(TIOCNOTTY)");
      exit(-1);
    }
    close(fd);
  }
#else
  consfd = open("/dev/tty",O_RDWR|O_NONBLOCK);
  if(!consfd){
    perror("console-open");
    exit(-1);
  }
#endif
  
#ifdef DISPLAY_FB
  {
    struct vt_mode vtmode;
    vtmode.mode = VT_PROCESS;
    vtmode.relsig = SIGUSR1;
    vtmode.acqsig = SIGUSR1;

    if(ioctl(consfd,VT_SETMODE,&vtmode)){
      perror("ioctl(VT_SETMODE)");
      exit(-1);
    }     
  }

  // Make TTY7 the active console
  if(ioctl(consfd,VT_ACTIVATE,7)){
    perror("ioctl(VT_ACTIVATE)");
    exit(-1);
  } 

  // Wait for it...
  if(ioctl(consfd,VT_WAITACTIVE,7)){
    perror("ioctl(VT_WAITACTIVE)");
    exit(-1);
  } 


  if(ioctl(consfd,KDSETMODE,KD_GRAPHICS)){
    perror("ioctl(VT_ACTIVATE)");
    exit(-1);
    } 

  /* Enable this later
  if(ioctl(consfd,VT_LOCKSWITCH,1)){
    perror("ioctl(VT_LOCKSWITCH)");
    exit(-1);
  }
  */

  /* FRAMEBUFFER SETUP */
  fbfd = open("/dev/fb0",O_RDWR);
  if(!fbfd){
    perror("fb-open");
    exit(-1);
  }
  if(ioctl(fbfd,FBIOGET_FSCREENINFO,&finfo)){
    perror("finfo-ioctl");
    exit(-1);
  }
  if(ioctl(fbfd,FBIOGET_VSCREENINFO,&vinfo)){
    perror("vinfo-ioctl");
    exit(-1);
  }
  screensize=vinfo.xres * vinfo.yres * vinfo.bits_per_pixel / 8;

  framebuffer = (char *)mmap(0,screensize,PROT_READ|PROT_WRITE,MAP_SHARED,fbfd,0);
  if((int)framebuffer == -0){
    perror("mmap");
    exit(-1);
  }

  // Get keyboard mode
  if(ioctl(consfd,KDGKBMODE,&iodata)){
    perror("ioctl(KDGKBMODE)");
    exit(-1);
  }

  // Request keyboard-raw mode
  if(ioctl(consfd,KDSKBMODE,K_RAW)){
    perror("ioctl(KDSKBMODE)");
    exit(-1);
  }

  {
    struct termios tio;
    tio.c_iflag = (IGNPAR|IGNBRK)&(~PARMRK)&(~ISTRIP);
    tio.c_oflag = 0;
    tio.c_cflag = CREAD|CS8;
    tio.c_lflag = 0;
    tio.c_cc[VTIME]=0;
    tio.c_cc[VMIN]=1;
    cfsetispeed(&tio,9600);
    cfsetospeed(&tio,9600);
    tcsetattr(consfd,TCSANOW,&tio);    
  }

  // Set signal process owner
  if(fcntl(consfd,F_SETOWN,getpid()) < 0){
    perror("fcntl(F_SETOWN)");
    exit(-1);
  }

  // Get keyboard messages
  if(fcntl(consfd,F_SETFL,O_NONBLOCK|O_ASYNC) < 0){
    perror("fcntl(F_SETFL)");
    exit(-1);
  }
#endif

#ifdef PS2MOUSE
  // Open mouse FD
  mousefd = open(MOUSE_FNAME,O_RDWR|O_NONBLOCK);
  if(!mousefd){
    perror("ps2mouse-open");
    exit(-1);
  }
  // Reset the mouse
  printf("Setting up PS2 Mouse on %s\n",MOUSE_FNAME);
  // Send mouse-command
  mousecmd[0] = 0xFF;
  write(mousefd,&mousecmd[0],1);
  while(read(mousefd,&mousedta[0],1) != 1){
    // Stall
  }
  if(mousedta[0] != 0xFA){
    printf("FATAL: MOUSE DID NOT RESPOND CORRECTLY TO RESET COMMAND\n");
    exit(-1);
  }
  // Send DEFAULTS
  mousecmd[0] = 0xF6;
  write(mousefd,&mousecmd[0],1);
  while(read(mousefd,&mousedta[0],1) != 1){
    // Stall
  }
  if(mousedta[0] != 0xFA){
    printf("FATAL: MOUSE DID NOT RESPOND CORRECTLY TO SET-DEFAULTS COMMAND\n");
    exit(-1);
  }
  // NEGOTIATE Improved-PS2 Mode
  // Send SET-SAMPLE RATE
  mousecmd[0] = 0xF3;
  write(mousefd,&mousecmd[0],1);
  while(read(mousefd,&mousedta[0],1) != 1){
    // Stall
  }
  if(mousedta[0] != 0xFA){
    printf("FATAL: MOUSE DID NOT RESPOND CORRECTLY TO SET-SAMPLE-RATE COMMAND\n");
    exit(-1);
  }
  // Send 200
  mousecmd[0] = 0xC8;
  write(mousefd,&mousecmd[0],1);
  while(read(mousefd,&mousedta[0],1) != 1){
    // Stall
  }
  if(mousedta[0] != 0xFA){
    printf("FATAL: MOUSE DID NOT RESPOND CORRECTLY TO SET-SAMPLE-RATE-200 COMMAND\n");
    exit(-1);
  }
  // Send SET-SAMPLE RATE
  mousecmd[0] = 0xF3;
  write(mousefd,&mousecmd[0],1);
  while(read(mousefd,&mousedta[0],1) != 1){
    // Stall
  }
  if(mousedta[0] != 0xFA){
    printf("FATAL: MOUSE DID NOT RESPOND CORRECTLY TO SET-SAMPLE-RATE COMMAND\n");
    exit(-1);
  }
  // Send 200
  mousecmd[0] = 0x64;
  write(mousefd,&mousecmd[0],1);
  while(read(mousefd,&mousedta[0],1) != 1){
    // Stall
  }
  if(mousedta[0] != 0xFA){
    printf("FATAL: MOUSE DID NOT RESPOND CORRECTLY TO SET-SAMPLE-RATE-100 COMMAND\n");
    exit(-1);
  }
  // Send SET-SAMPLE RATE
  mousecmd[0] = 0xF3;
  write(mousefd,&mousecmd[0],1);
  while(read(mousefd,&mousedta[0],1) != 1){
    // Stall
  }
  if(mousedta[0] != 0xFA){
    printf("FATAL: MOUSE DID NOT RESPOND CORRECTLY TO SET-SAMPLE-RATE COMMAND\n");
    exit(-1);
  }
  // Send 200
  mousecmd[0] = 0x50;
  write(mousefd,&mousecmd[0],1);
  while(read(mousefd,&mousedta[0],1) != 1){
    // Stall
  }
  if(mousedta[0] != 0xFA){
    printf("FATAL: MOUSE DID NOT RESPOND CORRECTLY TO SET-SAMPLE-RATE-80 COMMAND\n");
    exit(-1);
  }
  // Send GET-DEVICE-ID
  mousecmd[0] = 0xF2;
  write(mousefd,&mousecmd[0],1);
  while(read(mousefd,&mousedta[0],1) != 1){
    // Stall
  }
  if(mousedta[0] != 0xFA){
    printf("FATAL: MOUSE DID NOT RESPOND CORRECTLY TO GET-DEVICE-ID COMMAND (Answered %X)\n",mousedta[0]);
    exit(-1);
  }
  while(read(mousefd,&mousedta[0],1) != 1){
    // Stall
  }
  if(mousedta[0] != 0x03){
    printf("FATAL: MOUSE DID NOT ENTER ImPS2 MODE (Answered as type %X)\n",mousedta[0]);
    exit(-1);
  }
  // DONE, IN ImPS2 MODE

  // Send SET-STREAM-MODE
  mousecmd[0] = 0xEA;
  write(mousefd,&mousecmd[0],1);
  while(read(mousefd,&mousedta[0],1) != 1){
    // Stall
  }
  if(mousedta[0] != 0xFA){
    printf("FATAL: MOUSE DID NOT RESPOND CORRECTLY TO SET-STREAM-MODE COMMAND\n");
    exit(-1);
  }
  // Send DATA-REPORTING-ON
  mousecmd[0] = 0xF4;
  write(mousefd,&mousecmd[0],1);
  while(read(mousefd,&mousedta[0],1) != 1){
    // Stall
  }
  if(mousedta[0] != 0xFA){
    printf("FATAL: MOUSE DID NOT RESPOND CORRECTLY TO DATA-REPORTING-ON COMMAND\n");
    exit(-1);
  }

  // Set signal process owner
  if(fcntl(mousefd,F_SETOWN,getpid()) < 0){
    perror("fcntl(F_SETOWN)");
    exit(-1);
  }

  // Get mouse messages
  if(fcntl(mousefd,F_SETFL,O_NONBLOCK|O_ASYNC) < 0){
    perror("fcntl(F_SETFL)");
    exit(-1);
  }

  printf("Mouse initialization completed.\n");
#endif

#ifdef DISPLAY_SDL
  sdl_init(&screensize, &framebuffer, FB_WIDTH, FB_HEIGHT);
#endif
  // Clear screen and VRAM
  bzero(framebuffer,screensize);
  bzero(VRAM,0x20000);

}

#ifdef DISPLAY_FB
void redraw_display(){
  // Translate to 8bpp and post result
  unsigned long FBaddr=0;
  unsigned long long x=1;
  unsigned int row,col;
  unsigned long VRAMaddr=0;
  
  while(VRAMaddr < 0x20000){
    col = VRAMaddr*8;
    row = (col/1024);
    col -= (row*1024); // Remove excess
    x=1;    
    if(col < FB_WIDTH && row < FB_HEIGHT){
      FBaddr = (row*FB_WIDTH)+col;
      // Invert video if required.
      // This is structured like this to cut down on bit tests required in one pass.
      if((sib_video_attr&0x02) == 0x02){
	// Reverse Video
        while(x < 0x100000000LL){
          if((VRAM[VRAMaddr]&x)==x){
            framebuffer[FBaddr]=PIX_Z; // ON
          }else{
            framebuffer[FBaddr]=PIX_NZ; // OFF
          }
          x = x << 1;
          FBaddr++; // Next pixel
        }
      }else{
        // Normal Video
        while(x < 0x100000000LL){
          if((VRAM[VRAMaddr]&x)==x){
            framebuffer[FBaddr]=PIX_NZ; // ON
          }else{
            framebuffer[FBaddr]=PIX_Z; // OFF
          }
          x = x << 1;
          FBaddr++; // Next pixel
        }
      }
      //      FBaddr = row*col;
      FBaddr = (row*FB_WIDTH)+col;
    }
    VRAMaddr++;
  }
#ifdef DISPLAY_FB
  // Redraw the whole screen
  msync(framebuffer,screensize,MS_SYNC);
#endif
#ifdef DISPLAY_SDL
  sdl_sync(framebuffer, screensize);
#endif
}
#endif // DISPLAY_FB

void sib_nubus_io(){
  unsigned long NUbus_Addr = ldb(NUbus_Address,24,0);  

  // VRAM
  if(NUbus_Request == VM_READ && (NUbus_Addr >= 0xE80000 && NUbus_Addr <= 0xE9FFFF)){
    unsigned int VRAMaddr = (NUbus_Addr&0x1FFFF);

    // Initialize

    switch(NUbus_Addr&0x03){
    case 0: // WORD IO
      NBD[3] = VRAM[VRAMaddr+3];
      NBD[2] = VRAM[VRAMaddr+2];
      NBD[1] = VRAM[VRAMaddr+1];
      NBD[0] = VRAM[VRAMaddr+0];
      break;
      
    case 1: // LOW HALFWORD READ
      NBD[1] = VRAM[VRAMaddr];
      NBD[0] = VRAM[VRAMaddr-1];
      break;
      
    case 3: // HIGH HALFWORD READ
      NUbus_Data = 0;
      NBD[3] = VRAM[VRAMaddr];
      NBD[2] = VRAM[VRAMaddr-1];
      break;

    default:
      logmsg("BAD SIB WORD-READ TYPE\n");
      cpu_die_rq=1;
    }
    NUbus_acknowledge=1;
    return;
  }
  if(NUbus_Request == VM_BYTE_READ && (NUbus_Addr >= 0xE80000 && NUbus_Addr <= 0xE9FFFF)){
    unsigned int VRAMaddr = (NUbus_Addr&0x1FFFF);
    NBD[NUbus_Addr&0x03] = VRAM[VRAMaddr];
    
    NUbus_acknowledge=1;
    return;
  }
  if(NUbus_Request == VM_WRITE && (NUbus_Addr >= 0xEC0000 && NUbus_Addr <= 0xEDFFFF) && (NUbus_Addr&0x03)==0){
    // Read-Modify-Write version of VRAM
    unsigned int VRAMaddr = (NUbus_Addr&0x1FFFF);
    unsigned long VRAMdata = NUbus_Data;
    unsigned long VRAMrslt=0;
    unsigned long VSRCdata=0;
    uint8 *VSD = (unsigned char *)&VSRCdata;
    uint8 *VRD = (unsigned char *)&VRAMdata;

    VSD[3] = VRAM[VRAMaddr+3];
    VSD[2] = VRAM[VRAMaddr+2];
    VSD[1] = VRAM[VRAMaddr+1];
    VSD[0] = VRAM[VRAMaddr+0];

    // D means framebuffer-data
    // S means memory-bus-data

    // Modify VRAMdata according to the logical operation register
    switch(sib_opn_reg){
    case 0: // CLEAR
      VRAMrslt = 0; break;
    case 1: // D NOR S
      {
	unsigned long tmp=0;
	unsigned long x=0x80000000;
	while(x >= 1){ 
          if((VSRCdata&x)==0 && (VRAMdata&x)==0){ 
            tmp |= x; 
          } 
          x >>= 1; 
        } 
        VRAMrslt = tmp;
      }
      break;
    case 2: // S AND D-
      VRAMrslt = (VRAMdata&(~VSRCdata)); break;
    case 3: // D-
      VRAMrslt = (~VSRCdata); break;
    case 4:  // S- AND D
      VRAMrslt = ((~VRAMdata)&VSRCdata); break;
    case 5:  // S-
      VRAMrslt = (~VRAMdata); break;
    case 6:  // D XOR S
      VRAMrslt = (VSRCdata^VRAMdata); break;
    case 7:  // D NAND S
      {
	unsigned long tmp=0;
	unsigned long x=0x80000000;
	while(x > 0){ 
          if(!((VSRCdata&x)==x && (VRAMdata&x)==x)){
            tmp |= x; 
          }
          x >>= 1;
        }
        VRAMrslt = tmp;
      }
      //logmsgf("SIB: 0x%lX NAND 0x%lX = 0x%lX\n",VSRCdata,VRAMdata,VRAMrslt);
      break;
    case 8:  // D AND S
      VRAMrslt = (VSRCdata&VRAMdata); break;
    case 9:  // D XNOR S
      {
	unsigned long tmp=0;
	unsigned long x=0x80000000;
	while(x > 0){ 
          if((VSRCdata&x) == (VRAMdata&x)){
            tmp |= x;
          } 
          x >>= 1;
        }
        VRAMrslt = tmp; 
      }
      //logmsgf("SIB: 0x%lX XNOR 0x%lX = 0x%lX\n",VSRCdata,VRAMdata,VRAMrslt);
      break;
    case 0x0A: // NOP(S)
      VRAMrslt = VRAMdata; break;
    case 0x0B: // S OR D-
      VRAMrslt = (VRAMdata|(~VSRCdata)); break;
    case 0x0C: // D
      VRAMrslt = (VSRCdata); break;
    case 0x0D: // D OR -S
      VRAMrslt = (VSRCdata|(~VRAMdata)); break;
    case 0x0E: // D OR S
      VRAMrslt = (VSRCdata|VRAMdata); break;
    case 0x0F: // SET
      VRAMrslt = 0xFFFFFFFF; break;

    default:
      logmsgf("SIB: Unknown logical operation %lX\n",sib_opn_reg);
      cpu_die_rq=1;
    }

    // Ones in the mask means bits in that position (IN MEMORY) carry over to the new data.
    // Zero in the mask means bits in that position are overwritten.

    // First, turn off mask ones in the new data
    VRAMdata = VRAMrslt&(~sib_mask_reg);
    // Now load ones from the source
    VRAMdata |= (VSRCdata&sib_mask_reg);

    //logmsgf("SIB: 0x%lX mask = 0x%lX final-data\n",sib_mask_reg,VRAMdata);

    VRAM[VRAMaddr+0] = VRD[0]; 
    VRAM[VRAMaddr+1] = VRD[1]; 
    VRAM[VRAMaddr+2] = VRD[2]; 
    VRAM[VRAMaddr+3] = VRD[3];

    NUbus_acknowledge=1;
    if(sib_video_attr&0x01){ // Blank Video Output
      return;
    }

#ifdef DISPLAY_SDL
    sdl_video_write(VRAMaddr/(sizeof(unsigned int)/sizeof(unsigned char)), NUbus_Data);
#else
    // Translate to 8bpp and post result
    {
      unsigned long FBaddr=0;
      unsigned long long x=1;
      unsigned int row,col;

      col = VRAMaddr*8;
      row = (col/1024);
      col -= (row*1024); // Remove excess

      //      FBaddr = row*col;
      if(col < FB_WIDTH && row < FB_HEIGHT){ // Don't draw beyond end of host framebuffer
        FBaddr = (row*FB_WIDTH)+col;
        // Invert video if required.
        // This is structured like this to cut down on bit tests required in one pass.
         if((sib_video_attr&0x02) == 0x02){
          // Reverse Video
          while(x < 0x100000000LL){
            if((NUbus_Data&x)==x){
              framebuffer[FBaddr]=PIX_Z; // ON
            }else{
              framebuffer[FBaddr]=PIX_NZ; // OFF
            }
            x = x << 1;
            FBaddr++; // Next pixel
          }
        }else{
          // Normal Video
          while(x < 0x100000000LL){
            if((NUbus_Data&x)==x){
              framebuffer[FBaddr]=PIX_NZ; // ON
            }else{
              framebuffer[FBaddr]=PIX_Z; // OFF
            }
            x = x << 1;
            FBaddr++; // Next pixel
          }
        }
        //      FBaddr = row*col;
        FBaddr = (row*FB_WIDTH)+col;
#ifdef DISPLAY_FB
        msync(framebuffer+FBaddr,256,MS_SYNC);
#endif
#ifdef DISPLAY_SDL
        sdl_sync(framebuffer+FBaddr,256);
#endif
      }
    }
#endif // !DISPLAY_SDL
    // logmsgf("SIB: Word-Write %lX for address %lX (VRAM address %X)\n",NUbus_Data,NUbus_Addr,VRAMaddr);
    //    cpu_die_rq=1;
    return;
  }
  if(NUbus_Request == VM_WRITE && (NUbus_Addr >= 0xE80000 && NUbus_Addr <= 0xE9FFFF)){
    unsigned int VRAMaddr = (NUbus_Addr&0x1FFFF);

    switch(NUbus_Addr&0x03){
    case 0: // WORD IO
      VRAM[VRAMaddr+0] = NBD[0]; 
      VRAM[VRAMaddr+1] = NBD[1]; 
      VRAM[VRAMaddr+2] = NBD[2]; 
      VRAM[VRAMaddr+3] = NBD[3];
      break;

    case 3: // High Halfword Write
      VRAM[VRAMaddr-1] = NBD[2];
      VRAM[VRAMaddr] = NBD[3];
      break;

    case 1: // Low Halfword Write
      VRAM[VRAMaddr-1] = NBD[0];
      VRAM[VRAMaddr] = NBD[1];
      break;

    case 2: // BLOCK TRANSFER (NOT SUPPORTED ON SIB)      
    default:
      logmsgf("BAD SIB WORD-WRITE TYPE %ld\n",NUbus_Addr&0x03);
      cpu_die_rq=1;
    }

    NUbus_acknowledge=1;
    if(sib_video_attr&0x01){ // Blank Video Output
      return;
    }

#ifdef DISPLAY_SDL
    sdl_video_write(VRAMaddr/(sizeof(unsigned int)/sizeof(unsigned char)),
                    NUbus_Data);
#else
    // Translate to 8bpp and post result
    {
      unsigned long FBaddr=0;
      unsigned long long x=1;
      unsigned int row,col;

      col = VRAMaddr*8;
      row = (col/1024);
      col -= (row*1024); // Remove excess
//logmsg("SIB: nubus video write\n");

      //      FBaddr = row*col;
      if(col < FB_WIDTH && row < FB_HEIGHT){
        FBaddr = (row*FB_WIDTH)+col;
        // Invert video if required.
        // This is structured like this to cut down on bit tests required in one pass.
        if((sib_video_attr&0x02) == 0x02){
          // Reverse Video
          while(x < 0x100000000LL){
            if((NUbus_Data&x)==x){
              framebuffer[FBaddr]=PIX_Z; // ON
            }else{
              framebuffer[FBaddr]=PIX_NZ; // OFF
            }
            x = x << 1;
            FBaddr++; // Next pixel
          }
        }else{
          // Normal Video
          while(x < 0x100000000LL){
            if((NUbus_Data&x)==x){
              framebuffer[FBaddr]=PIX_NZ; // ON
            }else{
              framebuffer[FBaddr]=PIX_Z; // OFF
            }
            x = x << 1;
            FBaddr++; // Next pixel
          }
        }
        //      FBaddr = row*col;
        FBaddr = (row*FB_WIDTH)+col;
#ifdef DISPLAY_FB
        msync(framebuffer+FBaddr,256,MS_SYNC);
#endif
#ifdef DISPLAY_SDL
        sdl_sync(framebuffer+FBaddr,256);
#endif
      }
    }
#endif // !DISPLAY_SDL
    // logmsgf("SIB: Word-Write %lX for address %lX (VRAM address %X)\n",NUbus_Data,NUbus_Addr,VRAMaddr);
    //    cpu_die_rq=1;
    return;
  }

  if(NUbus_Request == VM_BYTE_WRITE && (NUbus_Addr >= 0xE80000 && NUbus_Addr <= 0xE9FFFF)){
    // sib_log_arm = 1;
    unsigned int VRAMaddr = (NUbus_Addr&0x1FFFF);    
    unsigned long VRAMdata=0;

    // Derotate
    VRAMdata = NBD[NUbus_Addr&0x3]; // ((NUbus_Data>>(8*(NUbus_Addr&0x3)))&0xFF);
    VRAM[VRAMaddr] = VRAMdata;
//logmsg("SIB: nubus video write\n");
    
#ifdef DISPLAY_SDL
    sdl_video_write_byte(VRAMaddr/(sizeof(unsigned int)/sizeof(unsigned char)),
                         VRAMaddr&3, VRAMdata);
#else
    // Translate to 8bpp and post result
    {
      unsigned long FBaddr=0;
      int x=1;
      unsigned int row,col;

      col = VRAMaddr*8;
      row = (col/1024);
      col -= (row*1024); // Remove excess

#ifdef DISPLAY_SDL
      logmsgf("SIB: write byte at %d: 0x%x\n", VRAMaddr, VRAMdata);
#endif

      if(col > 1024){
	cpu_die_rq=1;
      }
      
      //logmsgf("VRAMaddr %d, row %d col %d\n",VRAMaddr,row,col);

      if(sib_video_attr&0x01){ // Blank Video Output
	return;
      }

      if(col < FB_WIDTH && row < FB_HEIGHT){
        FBaddr = (row*FB_WIDTH)+col;
        
        while(x<0x100){
          if((sib_video_attr&0x02) == 0x02){
            if((VRAMdata&x)==x){
              framebuffer[FBaddr]=PIX_Z; // ON
            }else{
              framebuffer[FBaddr]=PIX_NZ; // OFF
            }
            x = x << 1;
            FBaddr++; // Next pixel
          }else{
            if((VRAMdata&x)==x){
              framebuffer[FBaddr]=PIX_NZ; // ON
            }else{
              framebuffer[FBaddr]=PIX_Z; // OFF
            }
            x = x << 1;
	  FBaddr++; // Next pixel
          }
        }
        //      FBaddr = VRAMaddr*8;
        FBaddr = (row*FB_WIDTH)+col;
#ifdef DISPLAY_FB
        msync(framebuffer+FBaddr,64,MS_SYNC);
#endif
#ifdef DISPLAY_SDL
        sdl_sync(framebuffer+FBaddr,64);
#endif
      }
    }
#endif // !DISPLAY_SDL
    NUbus_acknowledge=1;
    // logmsgf("SIB: Byte-Write %lX for address %lX (VRAM address %X)\n",NUbus_Data,NUbus_Addr,VRAMaddr);
    // cpu_die_rq=1;
    // sib_log_arm = 14;
    return;
  }  

  // Exclude ROM here
  if(NUbus_Request == VM_READ && (NUbus_Addr&0xFF8000)==0xFF8000){
    unsigned int ROMaddr = (NUbus_Addr&0x7FFF)>>2;

    /* Word 0 = byte 0
       Word 1 = byte 4
       Word 2 = byte 8
    */

    // Construct word
    NUbus_Data = SIB_ROM[ROMaddr]; 
    NUbus_Data &= 0xFF;

    if(NUbus_Addr&0x03){
      logmsgf("SIB: Word-Read %lX for odd address %lX (ROM address %X)\n",
              NUbus_Data,NUbus_Addr,ROMaddr);
      cpu_die_rq=1;
    }
    NUbus_acknowledge=1;
    return;
  }   
  
  if(NUbus_Request == VM_BYTE_READ && NUbus_Addr >= 0xFF8000){         
    unsigned int ROMaddr = ((NUbus_Addr >> 2)&0x1FFF);

    if((NUbus_Addr&0x3) != 0){ logmsg("SIB: ODD ROM ADDR\n"); cpu_die_rq=1; }
    if(ROMaddr > 0x1FFF){ logmsg("SIB: TOOBIG ROM ADDR\n"); cpu_die_rq=1; }

    NUbus_Data = SIB_ROM[ROMaddr];
    //    logmsgf("SIB: Byte-Read %lX for address %lX (ROM address %X)\n",NUbus_Data,NUbus_Addr,ROMaddr);
    NUbus_acknowledge=1;
    // cpu_die_rq=1;
    return;
  }  

  // NVRAM
  if((NUbus_Request == VM_BYTE_READ || NUbus_Request == VM_READ) && (NUbus_Addr >= 0xFA0000 && NUbus_Addr <= 0xFA1FFF)){
    // NVRAM
    unsigned int NVRAMaddr = ((NUbus_Addr >> 2)&0x7FF);    
    if((NUbus_Addr&0x3) != 0){ logmsg("SIB: ODD NVRAM ADDR\n"); }
    NUbus_Data = NVRAM[NVRAMaddr]&0xFF;
    NUbus_acknowledge=1;
    return;
  }
  if((NUbus_Request == VM_BYTE_WRITE || NUbus_Request == VM_WRITE) && (NUbus_Addr >= 0xFA0000 && NUbus_Addr <= 0xFA1FFF)){
    // NVRAM
    unsigned int NVRAMaddr = ((NUbus_Addr >> 2)&0x7FF);    
    if((NUbus_Addr&0x3) != 0){ logmsg("SIB: ODD NVRAM ADDR\n"); cpu_die_rq=1; }

    NVRAM[NVRAMaddr] = NUbus_Data&0xFF;
    NUbus_acknowledge=1;
    return;
  }

  // Registers
  if(NUbus_Request == VM_READ){
    switch(NUbus_Addr){
    case 0xE00080: // Video Attributes Register
      NUbus_Data = sib_video_attr;
      NUbus_acknowledge=1;
      return;
    case 0xE00084: // Mask Register
      NUbus_Data = sib_mask_reg;
      NUbus_acknowledge=1;
      return;      
    case 0xE00088: // Operation Register
      NUbus_Data = sib_opn_reg;
      NUbus_acknowledge=1;
      return;      
      //    case 0xE00098: // VIDEO TEST REGISTER
      // THIS TEST IS GOING TO SUCK!

    case 0xF00000: // RTC - Time Interrupt Event Vector Address
      NUbus_Data = sib_rtc_int_vector;
      NUbus_acknowledge=1;
      return;       
    case 0xF00004: // Itimer Short Interval Expired
      NUbus_Data = sib_sintv_expired_vector;
      NUbus_acknowledge=1;
      return;
    case 0xF00008: // Itimer Long Interval Expired
      NUbus_Data = sib_lintv_expired_vector;
      NUbus_acknowledge=1;
      return;
    case 0xF0000C: // Serial IO
      NUbus_Data = sib_serio_status_vector;
      NUbus_acknowledge=1;
      return;
    case 0xF00010: // LPT Acknowledge
      NUbus_Data = sib_lpt_ack_vector;
      NUbus_acknowledge=1;
      return;
    case 0xF00014: // Graphics Controller Command Acknowledgement
      NUbus_Data = sib_gcmd_ack_vector;
      NUbus_acknowledge=1;
      return;
    case 0xF00018: // Keyboard Interrupt
      NUbus_Data = sib_kbd_rtt_vector;
      NUbus_acknowledge=1;
      return;
    case 0xF0001C: // Overtemperature
      NUbus_Data = sib_psu_overtemp_vector;
      NUbus_acknowledge=1;
      return;
    case 0xF00020: // BOOT CHORD pressed
      NUbus_Data = sib_kbd_bootchord_vector;
      NUbus_acknowledge=1;
      return;
    case 0xF00024: // Mouse moved
      NUbus_Data = sib_mouse_moved_vector;
      NUbus_acknowledge=1;
      return;
    case 0xF00028: // Mouse key changed states
      NUbus_Data = sib_mouse_button_vector;
      NUbus_acknowledge=1;
      return;
    case 0xF0002C: // VOX Data Present
      NUbus_Data = sib_voxdata_present_vector;
      NUbus_acknowledge=1;
      return;
    case 0xF00030: // Sound Parity Error
      NUbus_Data = sib_sound_parity_vector;
      NUbus_acknowledge=1;
      return;
    case 0xF00034: // Fiber Link Disconnected
      NUbus_Data = sib_fiber_link_vector;
      NUbus_acknowledge=1;
      return;
    case 0xF00038: // POWER FAILURE
      NUbus_Data = sib_power_fail1_vector;
      NUbus_acknowledge=1;
      return;
    case 0xF0003C: // POWER FAILURE
      NUbus_Data = sib_power_fail2_vector;
      NUbus_acknowledge=1;
      return;
    case 0xF00041: // Configuration Register, Low Halfword Mode
      /* BITS ARE:
	 1 = (W)  RESET 
	 2 = (RW) NUBUS MASTER ENABLE 
	 4 = (R)  SIB Test LED
	 8 = (RW) NUbus Test (UNUSED ON SIB)
	 100 = (RW) Monitor Self-Test LED
	 200 = (RW) Chassis Self-Test LED
	 400 = (R)  PSU Overtemperature Warning
      */
      NUbus_Data = sib_config_reg;
      NUbus_acknowledge = 1;
      return;
    case 0xF20000: // Mouse Y-Position Register
    case 0xF20001: // Mouse Y-Position Register, low-halfword mode
      NUbus_Data = sib_mouse_y_pos&0xFFFF;
      //logmsgf("READ MOUSE Y POS - 0x%lX\n",NUbus_Data);
      NUbus_acknowledge=1;
      sib_mouse_moved_sent = 0;
      return;
    case 0xF20004: // Mouse X-Position Register
    case 0xF20005: // Mouse X-Position Register, low-halfword mode
      NUbus_Data = sib_mouse_x_pos&0xFFFF;
      NUbus_acknowledge=1;
      //logmsgf("READ MOUSE X POS - 0x%lX\n",NUbus_Data);
      sib_mouse_moved_sent = 0;
      return;
    case 0xF20008: // Mouse Keyswitch/Motion Data Register
      /* BITS ARE:
	 0x03 = Mouse X movement quadrature (inverted) (positive means move right/down, negative means move left/up)
	 0x0C = Mouse Y movement quadrature (inverted)
	 0x10 = Right Mouse Key Down
	 0x20 = Middle Mouse Key Down
	 0x40 = Left Mouse Key Down
	 0x80 = Keyboard Serial Data
      */
      /* If diagnostic loopback is set, fake out. */
      if((sib_int_diag_reg&0x0C)!=0){
	if(sib_int_diag_data&0x100){
	  NUbus_Data = 0x00;
	}else{
	  NUbus_Data = 0xFF;
	}
      }else{
	// If we are in data loopback mode...
	if((sib_int_diag_reg&0x02) != 0){
	  // Return diagnostic byte instead
	  NUbus_Data = sib_int_diag_data&0xFF;
	}else{
          // Otherwise, just return the register intact.
	  NUbus_Data = sib_mouse_motion_reg;
	  //logmsgf("READ MOUSE MOTION REG - 0x%lX\n",NUbus_Data);
	}
      }
      NUbus_acknowledge=1;
      sib_mouse_moved_sent = 0;
      sib_mouse_button_sent = 0;
      return;	 

    case 0xF2000C: // Diagnostic and Interrupt Enable plus MONITOR CONTROL REGISTER (word read)
    case 0xF2000D: // Diagnostic and Interrupt Enable plus MONITOR CONTROL REGISTER (low halfword read)
      /* BITS ARE:
	 0x0F = Set Diagnostic Register
	      0x01 = VOICE PARALLEL DATA PATH SELECTED
	             Replace fiber voice data with diag-data bits 0x7F.
		     Bit 0x80 means voice-data-recieved and will cause an interrupt if that's enabled.
		     After the video has been established, setting this will blank the display.
	      0x02 = MOUSE PARALLEL DATA PATH SELECTED
	             Replace fiber mouse/keyboard data with diag-data bits 0x7F.
	      0x04 = FIBER SERIO INTERNAL LOOPBACK SELECTED
	             Replace fiber data with complement of diag-data bit 0x80
	      0x08 = FIBER SERIO EXTERNAL LOOPBACK SELECTED
	             Same as INTERNAL loopback but done at the monitor.
	 0x10 = Voice Interrupt Enable
	 0x20 = Mouse Button Interrupt Enable
	 0x40 = Mouse Motion Interrupt Enable
	 0x80 = Monitor Sound Error Interrupt Enable
      */
      NUbus_Data = (sib_int_diag_reg&0xFF);
      /* BITS ARE:
         0x100: MOSENB Monitor Sound Enabled
         0x200: HIGAIN Microphone Hi-Gain Amplifier Active
         0x400: ALUPBK Microphone Analog Loopback Enabled
         0x800: PARCHK Force bad sound parity
      */
      NUbus_Data |= (sib_monitor_ctl_reg&0xF00);
      NUbus_acknowledge = 1;
      return;

    case 0xF20011: // Diagnostic Data, low-halfword mode
      NUbus_Data = sib_int_diag_data&0xFFFF;
      NUbus_acknowledge=1;
      return;
    case 0xF20015: // Sound Control Register, Low Halfword
      /* BITS ARE:
	 0xFF = Sound Data
	 0x100 = Odd Parity
      */	 
      if(genparity(sib_sound_control&0xFF) != ((sib_sound_control&0x100)>>8)){
        logmsgf("SOUND READ BAD PARITY\n"); cpu_die_rq=1;
      }
      NUbus_Data = sib_sound_control;      
      NUbus_acknowledge = 1;
      return;      
    case 0xF20019: // Speech Synthesis Data Register, Low-Halfword
      NUbus_Data = (sib_speech_data_reg&0xFFFF);
      NUbus_acknowledge=1;
      return;
    case 0xF2001D: // VOX Data Register, Low Halfword
      NUbus_Data = sib_vox_data_reg&0xFFFF;
      NUbus_acknowledge=1;
      return;  
    case 0xF80000:
      NUbus_Data = sib_rtc_100ns;
      NUbus_acknowledge=1;
      return;
    case 0xF80004:
      NUbus_Data = sib_rtc_10ms;
      NUbus_acknowledge=1;
      return;
    case 0xF80008:
      NUbus_Data = sib_rtc_sec;
      NUbus_acknowledge=1;
      return;
    case 0xF8000C:
      NUbus_Data = sib_rtc_min;
      NUbus_acknowledge=1;
      return;
    case 0xF80010:
      NUbus_Data = sib_rtc_hour;
      NUbus_acknowledge=1;
      return;
    case 0xF80014:
      NUbus_Data = sib_rtc_dow;
      NUbus_acknowledge=1;
      return;
    case 0xF80018:
      NUbus_Data = sib_rtc_date;
      NUbus_acknowledge=1;
      return;
    case 0xF8001C:
      NUbus_Data = sib_rtc_month;
      NUbus_acknowledge=1;
      return;
    case 0xF80020:
      NUbus_Data = sib_ram_100ns;
      //logmsgf("SIB: RAM 100NS RD: %X\n",sib_ram_100ns);
      NUbus_acknowledge=1;
      return;
    case 0xF80024: 
      NUbus_Data = sib_ram_10ms;
      //logmsgf("SIB: RAM 10MS RD: %X\n",sib_ram_10ms);
      NUbus_acknowledge=1;
      return;
    case 0xF80028:
      NUbus_Data = sib_ram_sec;
      NUbus_acknowledge=1;
      return;
    case 0xF8002C:
      NUbus_Data = sib_ram_min;
      NUbus_acknowledge=1;
      return;
    case 0xF80030:
      NUbus_Data = sib_ram_hour;
      NUbus_acknowledge=1;
      return;
    case 0xF80034:
      NUbus_Data = sib_ram_dow;
      NUbus_acknowledge=1;
      return;
    case 0xF80038:
      NUbus_Data = sib_ram_date;
      NUbus_acknowledge=1;
      return;
    case 0xF8003C:            /* This one is essential */
      NUbus_Data = sib_ram_month;
      NUbus_acknowledge=1;
      return;
    case 0xF80050: // RTC Update Status Bit
      /* Returns a 1 if the RTC is updating. */
      NUbus_Data=0;
      NUbus_acknowledge=1;
      return;
    }
  }

  if(NUbus_Request == VM_BYTE_READ){
    switch(NUbus_Addr){
    case 0xE00068: // CRT controller R1A
      // Auxiliary register - Retrace Interrupt Control
      /* BITS ARE:
         R 0x80 INTERRUPT GENERATED
         Reading clears the interrupt and allows another to be generated.
      */
      NUbus_Data = sib_crt_r1a&0x80;
      // Still enabled?
      /*
      if(sib_crt_r1a&0x80){ // Interrupt happened?
        if(sib_crt_r1a&0x40){
          sib_retrace_pi = 1; // Start Clock
          // logmsgf("CLOCK RESTARTED\n");
        }else{
          sib_retrace_pi = 0; // Mark not sent
          // logmsgf("CLOCK STOPPED\n");
        }
        // logmsgf("PI CLEARED\n");
        sib_crt_r1a &= 0x7F; // Reset
      }
      */
      NUbus_acknowledge=1;
      sib_gcmd_ack_sent = 0; // RESET
      return;
    case 0xE00080: // Video Attributes Register
      NUbus_Data = sib_video_attr&0xFF;
      NUbus_acknowledge=1;
      return;
    case 0xE00088: // Operation Register
      NUbus_Data = sib_opn_reg&0xFF;
      NUbus_acknowledge=1;
      return;      
    case 0xF00040: // Configuration Register
      /* BITS ARE:
	 1 = (W)  RESET 
	 2 = (RW) NUBUS MASTER ENABLE 
	 4 = (R)  SIB Test LED
	 8 = (RW) NUbus Test (UNUSED ON SIB)
	 100 = (RW) Monitor Self-Test LED
	 200 = (RW) Chassis Self-Test LED
	 400 = (R)  PSU Overtemperature Warning
      */
      NUbus_Data = sib_config_reg;
      NUbus_acknowledge = 1;
      return;
    case 0xF00041: // Configuration register, upper half
      NUbus_Data = (sib_config_reg&0xFF00);
      NUbus_acknowledge = 1;
      return;
    case 0xF10000: // LPT Data Register (R00)
      NUbus_Data = sib_lpt_r00;
      NUbus_acknowledge = 1;
      return;
    case 0xF20008: // Mouse Keyswitch/Motion Data Register
      /* BITS ARE:
	 0x03 = Mouse X movement quadrature (inverted) (positive means move right/down, negative means move left/up)
	 0x0C = Mouse Y movement quadrature (inverted)
	 0x10 = Right Mouse Key Down
	 0x20 = Middle Mouse Key Down
	 0x40 = Left Mouse Key Down
	 0x80 = Keyboard Serial Data
      */
      /* If diagnostic loopback is set, fake out. */
      if((sib_int_diag_reg&0x0C)!=0){
	if(sib_int_diag_data&0x100){
	  NUbus_Data = 0x00;
	}else{
	  NUbus_Data = 0xFF;
	}
      }else{
	// If we are in data loopback mode...
	if((sib_int_diag_reg&0x02) != 0){
	  // Return diagnostic byte instead
	  NUbus_Data = sib_int_diag_data&0xFF;
	}else{
          // Otherwise, just return the register intact.
	  NUbus_Data = sib_mouse_motion_reg;
          // logmsgf("READ MOUSE MOTION REG - 0x%lX\n",NUbus_Data);
	}
      }
      NUbus_acknowledge=1;
      sib_mouse_moved_sent = 0;
      sib_mouse_button_sent = 0;
      return;	 
      
    case 0xF2000C: // Diagnostic and Interrupt Enable
      /* BITS ARE:
	 0x0F = Set Diagnostic Register
	      0x01 = VOICE PARALLEL DATA PATH SELECTED
	             Replace fiber voice data with diag-data bits 0x7F.
		     Bit 0x80 means voice-data-recieved and will cause an interrupt if that's enabled.
		     After the video has been established, setting this will blank the display.
	      0x02 = MOUSE PARALLEL DATA PATH SELECTED
	             Replace fiber mouse/keyboard data with diag-data bits 0x7F.
	      0x04 = FIBER SERIO INTERNAL LOOPBACK SELECTED
	             Replace fiber data with complement of diag-data bit 0x80
	      0x08 = FIBER SERIO EXTERNAL LOOPBACK SELECTED
	             Same as INTERNAL loopback but done at the monitor.
	 0x10 = Voice Interrupt Enable
	 0x20 = Mouse Button Interrupt Enable
	 0x40 = Mouse Motion Interrupt Enable
	 0x80 = Monitor Sound Error Interrupt Enable
      */
      NUbus_Data = (sib_int_diag_reg&0xFF);
      NUbus_acknowledge = 1;
      return;

    case 0xF2000D: // MONITOR CONTROL REGISTER
      /* BITS ARE:
         0x100: MOSENB Monitor Sound Enabled
         0x200: HIGAIN Microphone Hi-Gain Amplifier Active
         0x400: ALUPBK Microphone Analog Loopback Enabled
         0x800: PARCHK Force bad sound parity
      */
      NUbus_Data = sib_monitor_ctl_reg&0xF00;
      NUbus_acknowledge = 1;
      return;

    case 0xF20010: // Diagnostic Data
      NUbus_Data = sib_int_diag_data&0xFF;
      NUbus_acknowledge = 1;
      return;

    case 0xF20014: // Sound Control Register
      /* BITS ARE:
	 0x7F = Sound Data
	 0x80 = Odd Parity
      */	 
      NUbus_Data = sib_sound_control&0xFF;
      if(genparity(sib_sound_control&0xFF) != ((sib_sound_control&0x100)>>8)){
        logmsgf("SOUND READ BAD PARITY\n"); cpu_die_rq=1;
      }
      NUbus_acknowledge = 1;
      return;      

    case 0xF20018: // Speech Synthesis Data Register
      NUbus_Data = (sib_speech_data_reg&0xFF);
      NUbus_acknowledge=1;
      return;

    case 0xF2001C: // VOX Data Register
      NUbus_Data = sib_vox_data_reg&0xFF;
      NUbus_acknowledge=1;
      return;   

    case 0xF80000: // RTC 100ns counter
      NUbus_Data = sib_rtc_100ns&0xFF;
      // logmsgf("SIB: RTC 100NS RD: %X\n",sib_rtc_10ms);
      NUbus_acknowledge=1;
      return;
    case 0xF80004: // RTC 10ms counter
      NUbus_Data = sib_rtc_10ms&0xFF;
      // logmsgf("SIB: RTC 10MS RD: %X\n",sib_rtc_10ms);
      NUbus_acknowledge=1;
      return;
    case 0xF80008: // RTC sec counter
      NUbus_Data = sib_rtc_sec;
//      logmsgf("SIB: sec RD: %X\n",sib_rtc_sec);
      NUbus_acknowledge=1;
      return;
    case 0xF8000C: // RTC 1min counter
      NUbus_Data = sib_rtc_min;
      NUbus_acknowledge=1;
      return;
    case 0xF80010: // RTC hour counter
      NUbus_Data = sib_rtc_hour;
      NUbus_acknowledge=1;
      return;
    case 0xF80014:
      NUbus_Data = sib_rtc_dow;
      NUbus_acknowledge=1;
      return;
    case 0xF80018:
      NUbus_Data = sib_rtc_date;
      NUbus_acknowledge=1;
      return;
    case 0xF8001C:
      NUbus_Data = sib_rtc_month;
      NUbus_acknowledge=1;
      return;
    case 0xF80020:
      NUbus_Data = sib_ram_100ns;
      //logmsgf("SIB: RAM 100NS RD: %X\n",sib_ram_100ns);
      NUbus_acknowledge=1;
      return;
    case 0xF80024: 
      NUbus_Data = sib_ram_10ms;
      //logmsgf("SIB: RAM 10MS RD: %X\n",sib_ram_10ms);
      NUbus_acknowledge=1;
      return;
    case 0xF80028:
      NUbus_Data = sib_ram_sec;
      NUbus_acknowledge=1;
      return;
    case 0xF8002C:
      NUbus_Data = sib_ram_min;
      NUbus_acknowledge=1;
      return;
    case 0xF80030:
      NUbus_Data = sib_ram_hour;
      NUbus_acknowledge=1;
      return;
    case 0xF80034:
      NUbus_Data = sib_ram_dow;
      NUbus_acknowledge=1;
      return;
    case 0xF80038:
      NUbus_Data = sib_ram_date;
      NUbus_acknowledge=1;
      return;
    case 0xF8003C:
      NUbus_Data = sib_ram_month;
      NUbus_acknowledge=1;
      return;

    case 0xF80040: // RTC Interrupt Status
      /* For reads, a 1 means the selected timer set an interrupt condition.
	 Same format as control register below */
      NUbus_Data = sib_rtc_int_status;
      sib_rtc_int_status = 0;
      sib_rtc_int_sent = 0;
      NUbus_acknowledge=1;
      return;

    case 0xF80050: // RTC Update Status Bit
      /* Returns a 1 if the RTC is updating. */
      NUbus_Data=0;
      NUbus_acknowledge=1;
      return;

    case 0xF90000: // READ ITIMER COUNTER 0 
      switch(sib_itimer_ctl_0&0x30){
      case 0x10: // LSB
        NUbus_Data = sib_itimer_ctr_0&0xFF;
	NUbus_acknowledge=1;
	break;
      case 0x20: // MSB
        NUbus_Data = (sib_itimer_ctr_0&0xFF00)>>8;
	NUbus_acknowledge=1;
	break;
      case 0x00: // LATCHED READ (LSB then MSB)
	switch(sib_itimer_red_0){
	case 0: // Read LSB
          NUbus_Data = sib_itimer_lat_0&0xFF;
          sib_itimer_red_0 |= 0x01;
	  NUbus_acknowledge=1;
	  break;
	case 1: // Read MSB
          NUbus_Data = (sib_itimer_lat_0&0xFF00)>>8;
	  sib_itimer_red_0 |= 0x02;
	  NUbus_acknowledge=1;
	  break;
	}
	break;
      case 0x30: // LSB then MSB
	switch(sib_itimer_red_0){
	case 0: // Read LSB
          NUbus_Data = sib_itimer_ctr_0&0xFF;
          sib_itimer_red_0 |= 0x01;
	  NUbus_acknowledge=1;
	  break;
	case 1: // Read MSB
          NUbus_Data = (sib_itimer_ctr_0&0xFF00)>>8;
	  sib_itimer_red_0 |= 0x02;
	  NUbus_acknowledge=1;
	  break;
	}
	break;
      }
      if(sib_itimer_red_0 == 0x3){
	sib_itimer_red_0 = 0; sib_itimer_lat_0 = 0;
      }
      return;

    case 0xFC0000: // Keyboard USART Status
      /* BITS ARE:
	 1  = TRANSMIT READY
	 2  = RECIEVE-READY
	 4  = TRANSMIT-BUFFER-EMPTY
	 8  = RECIEVE-PARITY-ERROR
	 10 = RECIEVE-OVERRUN-ERROR
	 20 = RECIEVE-FRAME-ERROR
	 40 = RECIEVE-BREAK-DETECT
	 80 = DATA-SET-READY (This is always high because it is HARD-WIRED that way!)
               EXCEPT WITH BREAK - See below.
      */
      if(sib_int_diag_reg&0x0C){
	logmsg("USART STATUS READ WITH DEBUG-ANALOG-LOOPBACK ON!\n");
        cpu_die_rq=1;
      }
      // Turn off transmit-buffer-empty off.
      keyboard_usart_status &= 0x7B;
      // If the transmitter is disconnected, turn it on
      if((keyboard_usart_command&0x04)==0){
	keyboard_usart_status |= 0x04; // the Transmit Buffer is Empty
      } 
      // It should probably also be on when there is nothing to transmit.
      // Test rx buffer
      if(keyboard_usart_rx_bot == keyboard_usart_rx_top){
        // If we're at the bottom of the usart buffer, turn off RECIEVER READY.
        keyboard_usart_status &= ~0x02;
      } else {
        // Otherwise, we have a character for pick-up, turn it on.
        keyboard_usart_status |= 0x02;
      }
      // FIDDLE WITH BREAK HERE
      if(keyboard_usart_command&0x08){
        NUbus_Data = keyboard_usart_status|0x80;
      }else{
        NUbus_Data = keyboard_usart_status;
      }
      NUbus_acknowledge = 1;
      return;

    case 0xFC0004: // Keyboard USART Recieve Data
      // If RX ready and no errors
      //      if((keyboard_usart_status&0x02) != 0 && (keyboard_usart_status&0x78)==0){ 
      // && keyboard_usart_rx_bot != keyboard_usart_rx_top){
      // Return data
      if (keyboard_usart_rx_bot != keyboard_usart_rx_top)
        {
	NUbus_Data = keyboard_usart_rx_fifo[keyboard_usart_rx_bot]&0xFF; 
	keyboard_usart_rx_fifo[keyboard_usart_rx_bot]=0; 
	keyboard_usart_rx_bot++;
	if(keyboard_usart_rx_bot > 25){ keyboard_usart_rx_bot = 0; }
        // If the buffer is empty
        if(keyboard_usart_rx_bot == keyboard_usart_rx_top){
          keyboard_usart_rx_bot = 0; keyboard_usart_rx_top = 0;
        }
      }else{
        // Otherwise 0
	NUbus_Data = 0;
      }
      NUbus_acknowledge=1;
      sib_kbd_rtt_sent = 0; // Reset interrupt
      return;	 
    }
  }
  
  if(NUbus_Request == VM_WRITE){
    switch(NUbus_Addr){
    case 0xE00068: // CRT controller R1A
      // Auxiliary register
      sib_crt_r1a = NUbus_Data;
      if(sib_crt_r1a&0x40){
        logmsgf("SIB: Retrace Interrupt Requested\n");
        if(sib_retrace_pi == 0){ sib_retrace_pi = 1; } // Start Clock
      }else{
        logmsgf("SIB: Retrace Interrupt Disabled\n");
        sib_retrace_pi = 0;
      }
        
      NUbus_acknowledge=1;
      return;

    case 0xE00080: // Video Attributes Register
      /* BITS ARE:
	 0x01 = Blank Video (1 = blank)
	 0x02 = Video Polarity (1 = reverse-video)
	 0x04 = Fiber Channel B Disconnected (1 = disconnected)
	 0x08 = Interrupt On Channel B Disconnection
      */
      if(NUbus_Data&0x01 && !(sib_video_attr&0x01)){
	// ** Blank Video *
	logmsg("SIB: Video Blanked\n");
      }
      if(!(NUbus_Data&0x01) && sib_video_attr&0x01){
	// ** Unblank Video **
	logmsg("SIB: Video Unblanked\n");
      }
      if((NUbus_Data&0x02) != (sib_video_attr&0x02)){
        logmsg("SIB: redraw\n");
	logmsgf("Video State Changed to %lX\n",NUbus_Data&0x2);
	// State changed - Update Linux framebuffer
	sib_video_attr = NUbus_Data;
#if DISPLAY_SDL
	sdl_set_bow_mode((sib_video_attr & 2) == 2);
#else
	redraw_display();
#endif
	NUbus_acknowledge=1;
	return;
      }
      sib_video_attr = NUbus_Data;
      NUbus_acknowledge=1;
      return;
    case 0xE00084: // Mask Register
      sib_mask_reg = NUbus_Data;
      NUbus_acknowledge=1;
      return;    
    case 0xE00088: // Operation Register
      sib_opn_reg = NUbus_Data;
      NUbus_acknowledge=1;
      return;        
    case 0xF00000: // RTC - Time Interrupt Event Vector Address
      sib_rtc_int_vector = NUbus_Data;
      //      logmsgf("RTC: Time Interrupt Vector 0x%lX\n",NUbus_Data);
      NUbus_acknowledge=1;
      return;       
    case 0xF00004: // Itimer Short Interval Expired
      sib_sintv_expired_vector = NUbus_Data;
      // logmsgf("RTC: Short Interval Vector 0x%lX\n",NUbus_Data); cpu_die_rq=1;
      NUbus_acknowledge=1;
      return;
    case 0xF00008: //
      sib_lintv_expired_vector = NUbus_Data;
      //      logmsgf("RTC: Long Interval Vector 0x%lX\n",NUbus_Data);
      NUbus_acknowledge=1;
      return;
    case 0xF0000C: //
      sib_serio_status_vector = NUbus_Data;
      NUbus_acknowledge=1;
      return;
    case 0xF00010: //
      sib_lpt_ack_vector = NUbus_Data;
      NUbus_acknowledge=1;
      return;
    case 0xF00014: //
      sib_gcmd_ack_vector = NUbus_Data;
      NUbus_acknowledge=1;
      return;
    case 0xF00018: //
      sib_kbd_rtt_vector = NUbus_Data;
      NUbus_acknowledge=1;
      return;
    case 0xF0001C: //
      sib_psu_overtemp_vector = NUbus_Data;
      NUbus_acknowledge=1;
      return;
    case 0xF00020: //
      sib_kbd_bootchord_vector = NUbus_Data;
      NUbus_acknowledge=1;
      return;
    case 0xF00024: //
      sib_mouse_moved_vector = NUbus_Data;
      NUbus_acknowledge=1;
      return;
    case 0xF00028: //
      sib_mouse_button_vector = NUbus_Data;
      NUbus_acknowledge=1;
      return;
    case 0xF0002C: //
      sib_voxdata_present_vector = NUbus_Data;
      NUbus_acknowledge=1;
      return;
    case 0xF00030: //
      sib_sound_parity_vector = NUbus_Data;
      NUbus_acknowledge=1;
      return;
    case 0xF00034: //
      sib_fiber_link_vector = NUbus_Data;
      NUbus_acknowledge=1;
      return;
    case 0xF00038: //
      sib_power_fail1_vector = NUbus_Data;
      NUbus_acknowledge=1;
      return;
    case 0xF0003C: //
      sib_power_fail2_vector = NUbus_Data;
      NUbus_acknowledge=1;
      return;

    case 0xF00040: // SIB Configuration Register      
    case 0xF00041: // SIB Configuration register, low-halfword mode
      /* BITS ARE:
	 1 = (W)  RESET 
	 2 = (RW) NUBUS MASTER ENABLE 
	 4 = (R)  SIB Test LED
	 8 = (RW) NUbus Test (UNUSED ON SIB)
	 100 = (RW) Monitor Self-Test LED
	 200 = (RW) Chassis Self-Test LED
	 400 = (R)  PSU Overtemperature Warning
      */
      if(NUbus_Data&0x1){
	// Keyboard USART reset
	keyboard_usart_mode=0; // MODESET mode
	keyboard_usart_status =  0x0; 
	keyboard_usart_mode = 0; 
	keyboard_usart_command = 0;
	keyboard_usart_rx_fifo[0] = 0;
	keyboard_usart_rx_top=0;
	keyboard_usart_rx_bot=0;
	
	logmsgf("SIB: MASTER RESET\n");
      }
      if(NUbus_Data&0x2){
	logmsgf("SIB: BUS-MASTER-ENABLE\n");
      }
      // Handle writables
      sib_config_reg &= 0x4F0;               // Turn off low bits
      sib_config_reg |= (NUbus_Data&0xF30A); // Turn on busmaster,nubus-test,LEDs,etc
      NUbus_acknowledge = 1;
      return;

    case 0xF20001: // Mouse Y-Position Register, low-halfword mode.
      sib_mouse_y_pos = NUbus_Data&0xFFFF;
      NUbus_acknowledge=1;
      return;

    case 0xF20005: // Mouse X-Position Register, low-halfword mode.
      sib_mouse_x_pos = NUbus_Data&0xFFFF;
      NUbus_acknowledge=1;
      return;
      
    case 0xF2000C: // word write
    case 0xF2000D: // Diagnostic and Interrupt Enable plus MONITOR CONTROL REGISTER, low halfword write
      /* BITS ARE:
	 0x0F = Set Diagnostic Register
	      0x01 = VOICE PARALLEL DATA PATH SELECTED
	             Replace fiber voice data with diag-data bits 0x7F.
		     Bit 0x80 means voice-data-recieved and will cause an interrupt if that's enabled.
		     After the video has been established, setting this will blank the display.
	      0x02 = MOUSE PARALLEL DATA PATH SELECTED
	             Replace fiber mouse/keyboard data with diag-data bits 0x7F.
	      0x04 = FIBER SERIO INTERNAL LOOPBACK SELECTED
	             Replace fiber data with complement of diag-data bit 0x80
	      0x08 = FIBER SERIO EXTERNAL LOOPBACK SELECTED (ALSO LOOPS KEYBOARD)
	             Same as INTERNAL loopback but done at the monitor.
	 0x10 = Voice Interrupt Enable
	 0x20 = Mouse Button Interrupt Enable
	 0x40 = Mouse Motion Interrupt Enable
	 0x80 = Monitor Sound Error Interrupt Enable
      */
      sib_int_diag_reg &= ~0xFF;               // Clear byte to be written 
      sib_int_diag_reg |= (NUbus_Data&0xFF); // and replace it
      if(sib_int_diag_reg&0xF0){ logmsgf("SIB: Voice/Mouse/Monitor Interrupt Enabled (0x%X)\n",sib_int_diag_reg); }

      /* BITS ARE:
         0x100: MOSENB Monitor Sound Enabled
         0x200: HIGAIN Microphone Hi-Gain Amplifier Active
         0x400: ALUPBK Microphone Analog Loopback Enabled
         0x800: PARCHK Force bad sound parity
      */      sib_monitor_ctl_reg = (NUbus_Data&0xF00);
      if(sib_monitor_ctl_reg&0x100){
        logmsgf("SIB: Monitor Sound Enabled\n");
      }
      if(sib_monitor_ctl_reg&0x200){
        logmsgf("SIB: Mic Hi-Gain Enabled\n");
      }
      if(sib_monitor_ctl_reg&0x400){
        logmsgf("SIB: Mic Analog Loopback Enabled\n");
      }
      if(sib_monitor_ctl_reg&0x800){
        logmsgf("SIB: Force Bad Sound Parity Enabled\n");
      }
      snd_parity_pi = 1; // Inhibit PI until data write
      NUbus_acknowledge=1;
      return;

    case 0xF20010: 
    case 0xF20011: // Diagnostic Data
      sib_int_diag_data = NUbus_Data;
      // logmsgf("SIB: Diagnostic Data: Recieved 0x%lX\n",NUbus_Data);
      // If this is a VOX loopback...
      if((sib_int_diag_reg&0x01) != 0 && sib_int_diag_data&0x100){
	// logmsg("SIB: Loading VOX Data\n");
	// we need to generate a PI.
	if(sib_config_reg&0x02  && (sib_int_diag_reg&0x10) != 0){
	  sib_nubus_io_request(NB_BYTE_WRITE, sib_voxdata_present_vector, 0xFF);
	}
	// and load the register
	sib_vox_data_reg = NUbus_Data&0xFF;
      }
      // Also, if we're in analog-loopback mode
      if(sib_int_diag_reg&0x0C){
	// we're faking out the serial data here. Reciever on?
	if((keyboard_usart_command&0x04)!=0){
	  // Yes, we should throw an error here.
	  logmsg("SIB: Serial data replaced by analog-loopback - Error should throw?\n");
	  // cpu_die_rq=1;
	}
      }
      NUbus_acknowledge=1;
      return;

    case 0xF20014: // Sound Control
    case 0xF20015: // Sound Control Register, Low Halfword
      sib_sound_control = NUbus_Data&0xFEFF; // Can't write parity bit directly
      // Generate parity bit
      if(genparity(sib_sound_control&0xFF) != 0){ sib_sound_control |= 0x100; }else{ sib_sound_control &= 0xFEFF; }
      // If even parity forced
      if(sib_monitor_ctl_reg&0x800){
        // Flip parity bit
        sib_sound_control ^= 0x100;
      }
      snd_parity_pi = 0; // Reset flag      
      // if(sib_sound_control != 0){ logmsgf("SIB: Sound Command 0x%lX Sent\n",NUbus_Data); }
      NUbus_acknowledge=1;
      return;

    case 0xF80008:
      sib_rtc_sec = NUbus_Data&0xFF;
      NUbus_acknowledge=1;
      return;
    case 0xF8000C:
      sib_rtc_min = NUbus_Data&0xFF;
      NUbus_acknowledge=1;
      return;
    case 0xF80010:
      sib_rtc_hour = NUbus_Data&0xFF;
      NUbus_acknowledge=1;
      return;
    case 0xF80014:
      sib_rtc_dow = NUbus_Data&0xFF;
      NUbus_acknowledge=1;
      return;
    case 0xF80018:
      sib_rtc_date = NUbus_Data&0xFF;
      NUbus_acknowledge=1;
      return;
    case 0xF8001C:
      sib_rtc_month = NUbus_Data&0xFF;
      NUbus_acknowledge=1;
      return;
    case 0xF80020:
      sib_ram_100ns = NUbus_Data&0xFF;
      NUbus_acknowledge=1;
      return;
    case 0xF80024: 
      sib_ram_10ms = NUbus_Data&0xFF;
      //logmsgf("SIB: RAM 10MS WT: %X\n",sib_ram_10ms);
      NUbus_acknowledge=1;
      return;
    case 0xF80028:
      sib_ram_sec = NUbus_Data&0xFF;
      NUbus_acknowledge=1;
      return;
    case 0xF8002C:
      sib_ram_min = NUbus_Data&0xFF;
      NUbus_acknowledge=1;
      return;
    case 0xF80030:
      sib_ram_hour = NUbus_Data&0xFF;
      NUbus_acknowledge=1;
      return;
    case 0xF80034:
      sib_ram_dow = NUbus_Data&0xFF;
      NUbus_acknowledge=1;
      return;
    case 0xF80038:
      sib_ram_date = NUbus_Data&0xFF;
      NUbus_acknowledge=1;
      return;
    case 0xF8003C:
      sib_ram_month = NUbus_Data&0xFF;
      NUbus_acknowledge=1;
      return;

    case 0xF90000: // Interval Timer, Counter 0
      //      cpu_die_rq=1;
      switch(sib_itimer_ctl_0&0x30){
      case 0x10: // Load LSB
        sib_itimer_ctr_0 &= 0xFF00;
	sib_itimer_ctr_0 |= NUbus_Data&0xFF;
	sib_itimer_lod_0 |= 0x01;
	NUbus_acknowledge=1;
	break;
      case 0x20: // Load MSB
        sib_itimer_ctr_0 &= 0xFF;
	sib_itimer_ctr_0 |= (NUbus_Data&0xFF)<<8;
	sib_itimer_lod_0 |= 0x02;
	NUbus_acknowledge=1;
	break;
      case 0x30: // Load LSB then MSB
	switch(sib_itimer_lod_0){
	case 0: // Load LSB
	  sib_itimer_ctr_0 &= 0xFF00;
	  sib_itimer_ctr_0 |= NUbus_Data&0xFF;
	  sib_itimer_lod_0 |= 0x01;
	  NUbus_acknowledge=1;
	  break;
	case 1: // Load MSB
	  sib_itimer_ctr_0 &= 0xFF;
	  sib_itimer_ctr_0 |= ((NUbus_Data&0xFF)<<8);
	  sib_itimer_lod_0 |= 0x02;
	  NUbus_acknowledge=1;
	  break;
	}
	break;
      case 0: // INVALID?
        logmsgf("ITIMER 0 WRITE W/ MODE 0\n");
        cpu_die_rq=1;
      }
      logmsgf("SIB: ITIMER 0 WORD-WRITE: 0x%lX, CTL 0x%lX, LOD 0x%lX, REG 0x%lX\n",NUbus_Data,sib_itimer_ctl_0,sib_itimer_lod_0,sib_itimer_ctr_0);
      if(sib_itimer_lod_0 == 0x3){
	sib_itimer_ena_0 = 1;  logmsg("SIB: Interval Timer 0 Started\n");
	sib_itimer_lod_0 = 0;
      }
      return;

    case 0xF90004: // Interval Timer, Counter 1
      if(sib_itimer_ena_1 != 0){   // If running
        sib_itimer_ena_1 = 0; // Disable
        sib_itimer_lod_1 = 0; // Unload
      }
      switch(sib_itimer_ctl_1&0x30){
      case 0x10: // Load LSB
        sib_itimer_ctr_1 &= 0xFF00;
	sib_itimer_ctr_1 |= NUbus_Data&0xFF;
	sib_itimer_lod_1 |= 0x01;
	sib_itimer_ped_1 = sib_itimer_ctr_1;
	NUbus_acknowledge=1;
	break;
      case 0x20: // Load MSB
        sib_itimer_ctr_1 &= 0xFF;
	sib_itimer_ctr_1 |= (NUbus_Data&0xFF)<<8;
	sib_itimer_lod_1 |= 0x02;
	sib_itimer_ped_1 = sib_itimer_ctr_1;
	NUbus_acknowledge=1;
	break;
      case 0x30: // Load LSB then MSB
	switch(sib_itimer_lod_1){
	case 0: // Load LSB
	  sib_itimer_ctr_1 &= 0xFF00;
	  sib_itimer_ctr_1 |= NUbus_Data&0xFF;
	  sib_itimer_lod_1 |= 0x01;
          sib_itimer_ped_1 = sib_itimer_ctr_1;
	  NUbus_acknowledge=1;
	  break;
	case 1: // Load MSB
	  sib_itimer_ctr_1 &= 0xFF;
	  sib_itimer_ctr_1 |= ((NUbus_Data&0xFF)<<8);
	  sib_itimer_lod_1 |= 0x02;
          sib_itimer_ped_1 = sib_itimer_ctr_1;
	  NUbus_acknowledge=1;
	  break;
	}
	break;
      }
      logmsgf("SIB: ITIMER 1 WORD-WRITE: 0x%lX, CTL 0x%lX, LOD 0x%lX, REG 0x%lX\n",NUbus_Data,sib_itimer_ctl_1&0x30,sib_itimer_lod_1,sib_itimer_ctr_1);
      if(sib_itimer_lod_1 == 0x3){
	sib_itimer_ena_1 = 1;
	if(sib_itimer_ctl_1&0xE){
	  logmsg("SIB: Interval Timer 1 Started in Square-Wave-Gen Mode\n");
	}else{
	  logmsg("SIB: Interval Timer 1 Started in Terminal-Count Mode\n");
	}
	sib_itimer_lod_1 = 0;
      }
      return;

    case 0xF90008: // Interval Timer, Counter 2
      if(sib_itimer_ena_2 != 0){   // If running
        sib_itimer_ena_2 = 0; // Disable
        sib_itimer_lod_2 = 0; // Unload
      }
      switch(sib_itimer_ctl_2&0x30){
      case 0x10: // Load LSB
        sib_itimer_ctr_2 &= 0xFF00;
	sib_itimer_ctr_2 |= NUbus_Data&0xFF;
	sib_itimer_lod_2 |= 0x01;
	NUbus_acknowledge=1;
	break;
      case 0x20: // Load MSB
        sib_itimer_ctr_2 &= 0xFF;
	sib_itimer_ctr_2 |= (NUbus_Data&0xFF)<<8;
	sib_itimer_lod_2 |= 0x02;
	NUbus_acknowledge=1;
	break;
      case 0x30: // Load LSB then MSB
	switch(sib_itimer_lod_2){
	case 0: // Load LSB
	  sib_itimer_ctr_2 &= 0xFF00;
	  sib_itimer_ctr_2 |= NUbus_Data&0xFF;
	  sib_itimer_lod_2 |= 0x01;
	  NUbus_acknowledge=1;
	  break;
	case 1: // Load MSB
	  sib_itimer_ctr_2 &= 0xFF;
	  sib_itimer_ctr_2 |= ((NUbus_Data&0xFF)<<8);
	  sib_itimer_lod_2 |= 0x02;
	  NUbus_acknowledge=1;
	  break;
	}
	break;
      }
      if(sib_itimer_lod_2 == 0x3){
	sib_itimer_ena_2 = 1;  logmsg("SIB: Interval Timer 2 Started\n");
	sib_itimer_lod_2 = 0;
      }
      return;

    }
  }

  if(NUbus_Request == VM_BYTE_WRITE){
    switch(NUbus_Addr){

    case 0xE00000: // CRT controller R00
      // Screen Format: Characters-per-horizontal-period
      sib_crt_r00 = NUbus_Data;
      NUbus_acknowledge=1;
      return;

    case 0xE00004: // CRT controller R01
      // Screen Format: Characters-per-data-row
      sib_crt_r01 = NUbus_Data;
      NUbus_acknowledge=1;
      return;

    case 0xE00008: // CRT controller R02
      // Screen Format: Horizontal Delay
      sib_crt_r02 = NUbus_Data;
      NUbus_acknowledge=1;
      return;

    case 0xE0000C: // CRT controller R03
      // Screen Format: Horizontal Sync Width
      sib_crt_r03 = NUbus_Data;
      NUbus_acknowledge=1;
      return;

    case 0xE00010: // CRT controller R04
      // Screen Format: Vertical Sync Width
      sib_crt_r04 = NUbus_Data;
      NUbus_acknowledge=1;
      return;

    case 0xE00014: // CRT controller R05
      // Screen Format: Vertical Delay
      sib_crt_r05 = NUbus_Data;
      NUbus_acknowledge=1;
      return;

    case 0xE00018: // CRT controller R06
      // Screen Format: Skew, pin configuration
      sib_crt_r06 = NUbus_Data;
      NUbus_acknowledge=1;
      return;

    case 0xE0001C: // CRT controller R07
      // Screen Format: Visible data rows per frame
      sib_crt_r07 = NUbus_Data;
      NUbus_acknowledge=1;
      return;

    case 0xE00020: // CRT controller R08
      // Screen Format: Scan Lines (frame, data row)
      sib_crt_r08 = NUbus_Data;
      NUbus_acknowledge=1;
      return;

    case 0xE00024: // CRT controller R09
      // Screen Format: Scan Lines per frame LSB
      sib_crt_r09 = NUbus_Data;
      NUbus_acknowledge=1;
      return;

    case 0xE00028: // CRT controller R0A
      // Auxiliary register
      sib_crt_r0a = NUbus_Data;
      NUbus_acknowledge=1;
      return;

    case 0xE0002C: // CRT controller R0B
      // Auxiliary register
      sib_crt_r0b = NUbus_Data;
      NUbus_acknowledge=1;
      return;

    case 0xE00030: // CRT controller R0C
      // Auxiliary register
      sib_crt_r0c = NUbus_Data;
      NUbus_acknowledge=1;
      return;

    case 0xE00034: // CRT controller R0D
      // Auxiliary register
      sib_crt_r0d = NUbus_Data;
      NUbus_acknowledge=1;
      return;

    case 0xE00038: // CRT controller R0E
      // Auxiliary register
      sib_crt_r0e = NUbus_Data;
      NUbus_acknowledge=1;
      return;

    case 0xE0003C: // CRT controller R0F
      // Auxiliary register
      sib_crt_r0f = NUbus_Data;
      NUbus_acknowledge=1;
      return;

    case 0xE00040: // CRT controller R10
      // Auxiliary register
      sib_crt_r10 = NUbus_Data;
      NUbus_acknowledge=1;
      return;

    case 0xE00044: // CRT controller R11
      // Auxiliary register
      sib_crt_r11 = NUbus_Data;
      NUbus_acknowledge=1;
      return;

    case 0xE00048: // CRT controller R12
      // Auxiliary register
      sib_crt_r12 = NUbus_Data;
      NUbus_acknowledge=1;
      return;

    case 0xE0004C: // CRT controller R13
      // Auxiliary register
      sib_crt_r13 = NUbus_Data;
      NUbus_acknowledge=1;
      return;

    case 0xE00050: // CRT controller R14
      // Auxiliary register
      sib_crt_r14 = NUbus_Data;
      NUbus_acknowledge=1;
      return;

    case 0xE00054: // CRT controller R15
      // Auxiliary register
      sib_crt_r15 = NUbus_Data;
      NUbus_acknowledge=1;
      return;

    case 0xE00058: // CRT controller R16
      // CURSOR CONTROL: RESET COMMAND
      logmsg("SIB: CURSOR RESET\n");
      sib_crt_r16 = NUbus_Data;
      NUbus_acknowledge=1;
      return;

    case 0xE0005C: // CRT controller R17
      // Auxiliary register
      sib_crt_r17 = NUbus_Data;
      NUbus_acknowledge=1;
      return;

    case 0xE00060: // CRT controller R18
      // Auxiliary register
      sib_crt_r18 = NUbus_Data;
      NUbus_acknowledge=1;
      return;

    case 0xE00064: // CRT controller R19
      // Auxiliary register
      sib_crt_r19 = NUbus_Data;
      NUbus_acknowledge=1;
      return;

    case 0xE00068: // CRT controller R1A
      // Auxiliary register - Retrace Interrupt Control
      /* BITS ARE:
         W 0x40 REQUEST RETRACE INTERRUPT
         R 0x80 INTERRUPT GENERATED
         Reading clears the interrupt and allows another to be generated.
      */
      sib_crt_r1a = NUbus_Data;
      if(sib_crt_r1a&0x40){
        logmsgf("SIB: Retrace Interrupt Requested\n");
        if(sib_retrace_pi == 0){ sib_retrace_pi = 1; } // Start Clock
      }else{
        logmsgf("SIB: Retrace Interrupt Disabled\n");
        sib_retrace_pi = 0;
      }

      NUbus_acknowledge=1;
      return;

    case 0xE00080: // Video Attributes Register
      /* BITS ARE:
	 0x01 = Blank Video (1 = blank)
	 0x02 = Video Polarity (1 = reverse-video)
	 0x04 = Fiber Channel B Disconnected (1 = disconnected)
	 0x08 = Interrupt On Channel B Disconnection
      */
      if(NUbus_Data&0x01 && !(sib_video_attr&0x01)){
	// ** Blank Video *
	logmsg("SIB: Video Blanked\n");
      }
      if(!(NUbus_Data&0x01) && sib_video_attr&0x01){
	// ** Unblank Video **
	logmsg("SIB: Video Unblanked\n");
      }
      if((NUbus_Data&0x02) != (sib_video_attr&0x02)){
        logmsg("SIB: redraw\n");
	logmsgf("Video State Changed to %lX\n",NUbus_Data&0x2);
	// State changed - Update Linux framebuffer
	sib_video_attr = NUbus_Data;
#if DISPLAY_SDL
	sdl_set_bow_mode((sib_video_attr & 2) == 2);
#else
	redraw_display();
#endif
	NUbus_acknowledge=1;
	return;
      }
      sib_video_attr = NUbus_Data;
      NUbus_acknowledge=1;
      return;

    case 0xE00088: // Operation Register
      sib_opn_reg = NUbus_Data; 
      NUbus_acknowledge=1;
      return;      

    case 0xF00040: // SIB Configuration Register
      /* BITS ARE:
	 1 = (W)  RESET 
	 2 = (RW) NUBUS MASTER ENABLE 
	 4 = (R)  SIB Test LED
	 8 = (RW) NUbus Test (UNUSED ON SIB)
	 100 = (RW) Monitor Self-Test LED
	 200 = (RW) Chassis Self-Test LED
	 400 = (R)  PSU Overtemperature Warning
      */
      if(NUbus_Data&0x1){
	// Keyboard USART reset
	keyboard_usart_mode=0; // MODESET mode
	keyboard_usart_status =  0x0; 
	keyboard_usart_mode = 0; 
	keyboard_usart_command = 0;
	keyboard_usart_rx_fifo[0] = 0;
	keyboard_usart_rx_top=0;
	keyboard_usart_rx_bot=0;
	
	logmsgf("SIB: MASTER RESET\n");
      }
      if(NUbus_Data&0x2){
	logmsgf("SIB: BUS-MASTER-ENABLE\n");
      }
      // Handle writables
      sib_config_reg &= 0xFF0;             // Turn off low bits
      sib_config_reg |= (NUbus_Data&0x0A); // Turn on busmaster,nubus-test
      NUbus_acknowledge=1;
      return;

    case 0xF00041: // Configuration register, upper half
      sib_config_reg &= 0xFF; // Turn off high bits
      sib_config_reg |= (NUbus_Data&0xFF00);
      NUbus_acknowledge = 1;
      return;

    case 0xF10000: // LPT register 00
      // LPT Data Register
      sib_lpt_r00 = NUbus_Data;
      NUbus_acknowledge=1;
      return;      

    case 0xF2000C: // Diagnostic and Interrupt Enable
      /* BITS ARE:
	 0x0F = Set Diagnostic Register
	      0x01 = VOICE PARALLEL DATA PATH SELECTED
	             Replace fiber voice data with diag-data bits 0x7F.
		     Bit 0x80 means voice-data-recieved and will cause an interrupt if that's enabled.
		     After the video has been established, setting this will blank the display.
	      0x02 = MOUSE PARALLEL DATA PATH SELECTED
	             Replace fiber mouse/keyboard data with diag-data bits 0x7F.
	      0x04 = FIBER SERIO INTERNAL LOOPBACK SELECTED
	             Replace fiber data with complement of diag-data bit 0x80
	      0x08 = FIBER SERIO EXTERNAL LOOPBACK SELECTED (ALSO LOOPS KEYBOARD)
	             Same as INTERNAL loopback but done at the monitor.
	 0x10 = Voice Interrupt Enable
	 0x20 = Mouse Button Interrupt Enable
	 0x40 = Mouse Motion Interrupt Enable
	 0x80 = Monitor Sound Error Interrupt Enable
      */
      sib_int_diag_reg &= ~(0xFF>>(8*(NUbus_Addr&0x3)));           // Clear byte to be written 
      sib_int_diag_reg |= (NUbus_Data>>(8*(NUbus_Addr&0x3))&0xFF); // and replace it
      if(sib_int_diag_reg&0xF0){ logmsgf("SIB: Voice/Mouse/Monitor Interrupt Enabled (0x%X)\n",sib_int_diag_reg); }
      NUbus_acknowledge = 1;
      return;

    case 0xF2000D: // MONITOR CONTROL REGISTER
      /* BITS ARE:
         0x100: MOSENB Monitor Sound Enabled
         0x200: HIGAIN Microphone Hi-Gain Amplifier Active
         0x400: ALUPBK Microphone Analog Loopback Enabled
         0x800: PARCHK Force bad sound parity
      */
      sib_monitor_ctl_reg = (NUbus_Data&0xF00);
      if(sib_monitor_ctl_reg&0x100){
        logmsgf("SIB: Monitor Sound Enabled\n");
      }
      if(sib_monitor_ctl_reg&0x200){
        logmsgf("SIB: Mic Hi-Gain Enabled\n");
      }
      if(sib_monitor_ctl_reg&0x400){
        logmsgf("SIB: Mic Analog Loopback Enabled\n");
      }
      if(sib_monitor_ctl_reg&0x800){
        logmsgf("SIB: Force Bad Sound Parity Enabled\n");
      }
      snd_parity_pi = 1; // Inhibit PI until data write
      NUbus_acknowledge=1;
      return;

    case 0xF20014: // Sound Control
      sib_sound_control = NUbus_Data&0xFF;
      // Generate parity bit
      if(genparity(sib_sound_control&0xFF) != 0){ sib_sound_control |= 0x100; }else{ sib_sound_control &= 0xFEFF; }
      // If bad parity forced
      if(sib_monitor_ctl_reg&0x800){
        // Flip parity bit
        sib_sound_control ^= 0x100;
      }
      snd_parity_pi = 0; // RESET THIS      
      // if(sib_sound_control != 0){ logmsgf("SIB: Sound Command 0x%lX Sent\n",NUbus_Data); }
      NUbus_acknowledge=1;
      return;

    case 0xF20018: // Speech Synthesis Data Register
      sib_speech_data_reg &= 0xFF00;
      sib_speech_data_reg |= (NUbus_Data&0xFF);
      // Generate parity bit
      if(genparity(sib_speech_data_reg&0xFF) != 0){ sib_speech_data_reg |= 0x100; }else{ sib_speech_data_reg &= 0xFEFF; }
      NUbus_acknowledge=1;
      return;

    case 0xF80000: // RTC 100ns count
      sib_rtc_100ns = NUbus_Data;
      NUbus_acknowledge=1;
      return;
    case 0xF80004: 
      sib_rtc_10ms = NUbus_Data;
      logmsgf("SIB: RTC 10MS WT: %X\n",sib_rtc_10ms);
      NUbus_acknowledge=1;
      return;
    case 0xF80008:
      sib_rtc_sec = NUbus_Data;
      NUbus_acknowledge=1;
      return;
    case 0xF8000C:
      sib_rtc_min = NUbus_Data;
      NUbus_acknowledge=1;
      return;
    case 0xF80010:
      sib_rtc_hour = NUbus_Data;
      NUbus_acknowledge=1;
      return;
    case 0xF80014:
      sib_rtc_dow = NUbus_Data;
      NUbus_acknowledge=1;
      return;
    case 0xF80018:
      sib_rtc_date = NUbus_Data;
      NUbus_acknowledge=1;
      return;
    case 0xF8001C:
      sib_rtc_month = NUbus_Data;
      NUbus_acknowledge=1;
      return;
    case 0xF80020:
      sib_ram_100ns = NUbus_Data;
      NUbus_acknowledge=1;
      return;
    case 0xF80024: 
      sib_ram_10ms = NUbus_Data;
      //logmsgf("SIB: RAM 10MS WT: %X\n",sib_ram_10ms);
      NUbus_acknowledge=1;
      return;
    case 0xF80028:
      sib_ram_sec = NUbus_Data;
      NUbus_acknowledge=1;
      return;
    case 0xF8002C:
      sib_ram_min = NUbus_Data;
      NUbus_acknowledge=1;
      return;
    case 0xF80030:
      sib_ram_hour = NUbus_Data;
      NUbus_acknowledge=1;
      return;
    case 0xF80034:
      sib_ram_dow = NUbus_Data;
      NUbus_acknowledge=1;
      return;
    case 0xF80038:
      sib_ram_date = NUbus_Data;
      NUbus_acknowledge=1;
      return;
    case 0xF8003C:
      sib_ram_month = NUbus_Data;
      NUbus_acknowledge=1;
      return;

    case 0xF80040: // RTC Interrupt Status
      /* For reads, a 1 means the selected timer set an interrupt condition.
	 Same format as control register below */
      logmsgf("SIB: WARNING: RTC Interrupt Status WRITE - Wrote 0x%lX, Status is %lX\n",NUbus_Data,sib_rtc_int_status);
      // sib_rtc_int_status = 0;
      NUbus_acknowledge=1;
      // cpu_die_rq=1;
      return;
      
    case 0xF80044: // RTC Interrupt Control
      /* A 1 means set an interrupt whenever the selected timer is updated. 
	 Bits are:
	 0x01 = Compare (Interrupt when RTC timers = RAM timers)
	 0x02 = 10Hz
	 0x04 = 1Hz
	 0x08 = 1 minute
	 0x10 = 1 hour
	 0x20 = 1 day
	 0x40 = 1 week
	 0x80 = 1 month
      */
      sib_rtc_int_control = NUbus_Data;
      if(sib_rtc_int_control != 0){
        logmsgf("SIB: RTC PI requested (0x%lX)\n",NUbus_Data); 
        // HACK HACK -- DELAY SOME IF TOO CLOSE AT COUNTER START
        if(sib_rtc_10ms == sib_ram_10ms){
          logmsgf("SIB: RTC 10MS OVERRIDE\n");
          sib_rtc_10ms = 0;
        }
        
        if(sib_rtc_int_control&0x01){
          logmsgf("SIB: RTC/RAM %X/%X %X/%X %X/%X %X/%X %X/%X %X/%X %X/%X %X/%X\n",
                  sib_rtc_month,sib_ram_month,
                  sib_rtc_date,sib_ram_date,
                  sib_rtc_dow,sib_ram_dow,
                  sib_rtc_hour,sib_ram_hour,
                  sib_rtc_min,sib_ram_min,
                  sib_rtc_sec,sib_ram_sec,
                  sib_rtc_10ms,sib_ram_10ms,
                  sib_rtc_100ns,sib_ram_100ns); 
        }
      }
      NUbus_acknowledge=1;
      return;

    case 0xF80054: // RTC "GO" Command
      logmsgf("SIB: RTC GO PULSE\n");
      // Reset RTC if requested
      sib_rtc_100ns=0;
      sib_rtc_10ms=0;
      sib_rtc_min=0;
      if(sib_rtc_sec > 0x40){
        sib_rtc_min++;
      }
      sib_rtc_sec=0;
      NUbus_acknowledge=1;
      return;      

    case 0xF90000: // Interval Timer, Counter 0
      //      cpu_die_rq=1;
      switch(sib_itimer_ctl_0&0x30){
      case 0x10: // Load LSB
	sib_itimer_ctr_0 &= 0x00FF;
	sib_itimer_ctr_0 |= NUbus_Data&0xFF;
	sib_itimer_lod_0 |= 0x01;
	NUbus_acknowledge=1;
	break;
      case 0x20: // Load MSB
	sib_itimer_ctr_0 &= 0xFF00LL;
	sib_itimer_ctr_0 |= (NUbus_Data<<8);
	sib_itimer_lod_0 |= 0x02;
	NUbus_acknowledge=1;
	break;
      case 0: // APPARENTLY MODE-0 IS OK
        // logmsgf("SIB: ITIMER 0: BYTE-WRITE: MODE ZERO\n");
        // cpu_die_rq=1;
        // Fall thru to
      case 0x30: // Load LSB then MSB
	switch(sib_itimer_lod_0){
	case 0: // Load LSB
	  sib_itimer_ctr_0 &= 0xFF00;
	  sib_itimer_ctr_0 |= NUbus_Data&0xFF;
	  sib_itimer_lod_0 |= 0x01;
	  NUbus_acknowledge=1;
	  break;
	case 1: // Load MSB
	  sib_itimer_ctr_0 &= 0xFF;
	  sib_itimer_ctr_0 |= ((NUbus_Data&0xFF)<<8);
	  sib_itimer_lod_0 |= 0x02;
	  NUbus_acknowledge=1;
	  break;
	}
	break;
      }
      // logmsgf("SIB: ITIMER 0 BYTE-WRITE: 0x%lX, CTL 0x%lX, LOD 0x%lX, REG 0x%lX\n",NUbus_Data,sib_itimer_ctl_0,sib_itimer_lod_0,sib_itimer_ctr_0);
      if(sib_itimer_lod_0 == 0x3){
	sib_itimer_ena_0 = 1;  // logmsg("SIB: Interval Timer 0 Started\n");
	sib_itimer_lod_0 = 0;
      }
      return;

    case 0xF90004: // Interval Timer, Counter 1
      if(sib_itimer_ena_1 != 0){   // If running
        sib_itimer_ena_1 = 0; // Disable
        sib_itimer_lod_1 = 0; // Unload
      }
      switch(sib_itimer_ctl_1&0x30){
      case 0x10: // Load LSB
	sib_itimer_ctr_1 &= 0xFF00;
	sib_itimer_ctr_1 |= NUbus_Data&0xFF;
	sib_itimer_ped_1 = sib_itimer_ctr_1;
	sib_itimer_lod_1 |= 0x01;
	NUbus_acknowledge=1;
	break;
      case 0x20: // Load MSB
	sib_itimer_ctr_1 &= 0xFF;
	sib_itimer_ctr_1 |= (NUbus_Data<<8);
	sib_itimer_lod_1 |= 0x02;
	sib_itimer_ped_1 = sib_itimer_ctr_1;
	NUbus_acknowledge=1;
	break;
      case 0x30: // Load LSB then MSB
	switch(sib_itimer_lod_1){
	case 0: // Load LSB
	  sib_itimer_ctr_1 &= 0xFF00;
	  sib_itimer_ctr_1 |= NUbus_Data&0xFF;
	  sib_itimer_ped_1 = sib_itimer_ctr_1;
	  sib_itimer_lod_1 |= 0x01;
	  NUbus_acknowledge=1;
	  break;
	case 1: // Load MSB
	  sib_itimer_ctr_1 &= 0x00FF;
	  sib_itimer_ctr_1 |= (NUbus_Data&0xFF)<<8;
	  sib_itimer_ped_1 = sib_itimer_ctr_1;
	  sib_itimer_lod_1 |= 0x02;
	  NUbus_acknowledge=1;
	  break;
	}
	break;
      }
      logmsgf("SIB: ITIMER 1 BYTE-WRITE: 0x%lX, CTL 0x%lX, LOD 0x%lX, REG 0x%lX\n",NUbus_Data,sib_itimer_ctl_1&0x30,sib_itimer_lod_1,sib_itimer_ctr_1);
      if(sib_itimer_lod_1 == 0x3){
	sib_itimer_ena_1 = 1;
	if(sib_itimer_ctl_1&0xE){
	  logmsg("SIB: Interval Timer 1 Started in Square-Wave-Gen Mode\n");
	}else{
	  logmsg("SIB: Interval Timer 1 Started in Terminal-Count Mode\n");
	}
	sib_itimer_lod_1 = 0;
      }
      return;

    case 0xF90008: // Interval Timer, Counter 2
      if(sib_itimer_ena_2 != 0){   // If running
        sib_itimer_ena_2 = 0; // Disable
        sib_itimer_lod_2 = 0; // Unload
      }
      switch(sib_itimer_ctl_2&0x30){
      case 0x10: // Load LSB
	sib_itimer_ctr_2 &= 0xFF00;
	sib_itimer_ctr_2 |= NUbus_Data&0xFF;
	sib_itimer_lod_2 |= 0x01;
	NUbus_acknowledge=1;
	break;
      case 0x20: // Load MSB
	sib_itimer_ctr_2 &= 0xFF;
	sib_itimer_ctr_2 |= (NUbus_Data<<8);
	sib_itimer_lod_2 |= 0x02;
	NUbus_acknowledge=1;
	break;
      case 0x30: // Load LSB then MSB
	switch(sib_itimer_lod_2){
	case 0: // Load LSB
	  sib_itimer_ctr_2 &= 0xFF00;
	  sib_itimer_ctr_2 |= NUbus_Data&0xFF;
	  sib_itimer_lod_2 |= 0x01;
	  NUbus_acknowledge=1;
	  break;
	case 1: // Load MSB
	  sib_itimer_ctr_2 &= 0xFF;
	  sib_itimer_ctr_2 |= (NUbus_Data<<8);
	  sib_itimer_lod_2 |= 0x02;
	  NUbus_acknowledge=1;
	  break;
	}
	break;
      }
      if(sib_itimer_lod_2 == 0x3){
	sib_itimer_ena_2 = 1; logmsgf("SIB: Interval Timer 2 Started (IT2: %X IT1: %X)\n",sib_itimer_ctr_2,sib_itimer_ped_1);
	sib_itimer_lod_2 = 0;
      }
      return;

    case 0xF9000C: // Interval timer mode control
      /* BITS ARE:
	 0x01 = BCD-Count-Select (1 = count in BCD, 0 = count in 16-bit binary)
	 0x0E = Mode-Select (0 = interrupt-on-terminal-count, 3 = square-wave-rate-generator)
	 0x30 = Read-Load-Control (0 = Counter-Latching, 1 = Read/Load LSB, 2 = Read/Load MSB, 3 = Read/Load LSB then MSB)
	 0xC0 = Select-Counter (0 thru 2 selects the numbered counter, 3 is illegal)
      */
      //      logmsgf("SIB: ITimer-Control: 0x%lX\n",NUbus_Data);
      /*
      if(NUbus_Data != 0){
        logmsgf("SIB: ITIMER CTL WRITE: 0x%lX\n",NUbus_Data);
        //        cpu_die_rq=1;
      }
      */

      if(NUbus_Data&0x01){ logmsg("SIB: BCD-Mode is not supported!\n"); cpu_die_rq=1;}
      switch(NUbus_Data&0xC0){
      case 0x00: // Load 0
	sib_itimer_ctl_0 = NUbus_Data;
	sib_itimer_ena_0 = 0; sib_itimer_lod_0 = 0;
        // Latch read register if needed and not already latched
        if((sib_itimer_ctl_0&0x30)==0 && sib_itimer_lat_0 == 0){
          sib_itimer_lat_0 = sib_itimer_ctr_0;
        }
	NUbus_acknowledge = 1;
	return;
      case 0x40: // Load 1
        // logmsgf("SIB: ITIMER CTL 1 WRITE: 0x%lX\n",NUbus_Data);
	sib_itimer_ctl_1 = NUbus_Data;
	sib_itimer_ena_1 = 0; sib_itimer_lod_1 = 0;
	sib_itimer_lvl_1 = 0; sib_itimer_edg_1 = 0;
	NUbus_acknowledge = 1;
	return;
      case 0x80: // Load 2
        // logmsgf("SIB: ITIMER CTL 2 WRITE: 0x%lX\n",NUbus_Data);
	sib_itimer_ctl_2 = NUbus_Data;
	sib_itimer_ena_2 = 0; sib_itimer_lod_2 = 0;
	NUbus_acknowledge = 1;
	return;
      }
      cpu_die_rq=1;
      break;

    case 0xFC0000: // Keyboard USART Control
      /* COMMAND BITS ARE:
	 1  = TRANSMIT ENABLE (1 = enabled)
	 2  = DATA-TERMINAL-READY (1 = force low)
	 4  = RECIEVE-ENABLE (1 = enable, enable requires error-status reset)
	 8  = SEND-BREAK-CHARACTER (1 = send break)
	 10 = ERROR-STATUS-RESET (Clears error, overrun, frame-error flags)
	 20 = REQUEST-TO-SEND (1 = force low)
	 40 = INTERNAL-RESET (1 = Return USART to mode-set operation)
	 80 = MUST-BE-ZERO
	 MODE BITS ARE:
	 03 = BAUD-RATE SELECT (00 = SYNCH,01 = No Division, 02 = Clock/26, 03 = Clock/64)
	 0C = CHARACTER LENGTH (00,04,08,0C = 5,6,7,8 bits)
	 10 = PARITY ENABLE    (1 = Enable)
	 20 = PARITY SELECT    (1 = EVEN, 0 = ODD)
	 C0 = STOP BIT SELECT  (00 = ILLEGL, 40 = 1, 80 = 1 1/2, C0 = 2 stop bits)
      */
      if(keyboard_usart_mode==0){ // MODE SET mode
	logmsgf("SIB: USART Mode Set: 0x%lX\n",NUbus_Data);
	keyboard_usart_control = NUbus_Data; // Store this
	keyboard_usart_mode = 1; // Go to COMMAND SET mode	
	keyboard_usart_status |= 0x01; // Transmitter Ready isn't dependent on enable or whatever.
      }else{
	// COMMAND SET mode
	logmsgf("SIB: USART Command Set: 0x%lX\n",NUbus_Data);
	if(NUbus_Data&0x01){ keyboard_usart_status |= 0x01; }else{ keyboard_usart_status &= 0xFFFB; }
	if(NUbus_Data&0x04){    // RX enable
	  if((keyboard_usart_status&0x38)==0){  // requires error reset to turn on.
	  //	  if(NUbus_Data&0x10){
	    keyboard_usart_status |= 0x02;
	  }          
	}else{ keyboard_usart_status &= 0xFFFD; }
	if(NUbus_Data&0x10){ keyboard_usart_status &= 0x07; }
	if(NUbus_Data&0x40){
	  // INTERNAL RESET
	  keyboard_usart_status = 0x0; 
	  keyboard_usart_mode = 0; 
	  keyboard_usart_command = 0;
	  keyboard_usart_rx_fifo[0] = 0;
	  keyboard_usart_rx_top=0;
	  keyboard_usart_rx_bot=0;
	}else{
          // Load keyboard command register.
          // CANNOT TURN OFF TRANSMITTER ONCE ENABLED WITHOUT A RESET!        
	  keyboard_usart_command = (NUbus_Data|(keyboard_usart_command&0x01));
          // logmsgf("SIB: USART Command Loaded: 0x%lX\n",keyboard_usart_command);
	}         
	if(NUbus_Data&0x08){ // BREAK SEND set
	  // Sending a break - Transmitter on?
	  if((keyboard_usart_command&0x01)==1){
	    // In loopback?
	    if(sib_int_diag_reg&0x08){
	      logmsgf("SIB: BREAK looped\n");
	      keyboard_usart_status |= 0x70; // BREAK detected
	    }else{
              logmsgf("SIB: BREAK sent\n");
              //              cpu_die_rq=1;
            }
	  }else{
            logmsgf("BREAK send request with transmitter off\n");
            cpu_die_rq=1;
          }
	}

      }
      NUbus_acknowledge=1;
      return;

    case 0xFC0004: // Keyboard USART Transmit Data
      keyboard_usart_tx_data = NUbus_Data;
      // Push transmitted data if loopback mode
      // Do nothing if transmitter disabled.
      if((keyboard_usart_command&0x01)==1){
	if(sib_int_diag_reg&0x08){
	  keyboard_usart_rx_fifo[keyboard_usart_rx_top] = (NUbus_Data&0xFF);
	  keyboard_usart_rx_top++;
	  if(keyboard_usart_rx_top > 25){ keyboard_usart_rx_top = 0; }
	}
	// HACK-HACK - 0x00 is apparently an initialization code.
	// Respond with 0x70      
	if(NUbus_Data == 0x00){
	  keyboard_usart_rx_fifo[keyboard_usart_rx_top] = 0x70;
	  keyboard_usart_rx_top++;
	  if(keyboard_usart_rx_top > 25){ keyboard_usart_rx_top = 0; }
	}
      }
      //logmsgf("SIB: KBD Transmit Data 0x%X\n",keyboard_usart_tx_data);
      NUbus_acknowledge=1;
      return;      
    }
  }

  logmsgf("SIB: Unknown NUbus IO: Op %ld for address %lX\n",
          NUbus_Request,NUbus_Addr);
  cpu_die_rq=1;
}

unsigned long sib_clockpulses = 0;

inline void sib_clock_pulse(){
  // Add 150ns to faster timers
  sib_clockpulses++;

  // The i-rate is 7500000 per second nominal. 1000000 per second is the interval clock rate.
  if(sib_clockpulses == 6){
    sib_clockpulses = 0;
    // 1MHz
    // Operate short-interval timer
    if(sib_itimer_lod_0 == 0){ // If loaded
      sib_itimer_ctr_0--; // Decrement
      if(sib_itimer_ctr_0 == 0){ // Ran out?
        if(sib_itimer_ena_0 != 0){ // PI enabled?
          logmsg("SIB: Interval timer 0 expired\n");
          // Write 0xFF to this location if interrupts are enabled
          if(sib_config_reg&0x2){
            sib_nubus_io_request(NB_BYTE_WRITE, sib_sintv_expired_vector, 0xFF);
          }
          // Don't repeat PI
          sib_itimer_ena_0 = 0;
        }
        sib_itimer_ctr_0 = 0xFFFF; // Loop
      }
    }

    // Operate medium-interval timer, also 1MHz
    if(sib_itimer_ena_1){
      sib_itimer_ctr_1--; // Decrement
      if(sib_itimer_ctr_1 == 0){
        if(sib_itimer_ctl_1&0xE){
          // logmsgf("SIB: Interval Timer 1 expired in SW mode\n");
          // Square-Wave Mode
          sib_itimer_ctr_1 = sib_itimer_ped_1; // Reset for square-wave mode
          // Transitions happened, too.
          // sib_itimer_edg_1 = 1;
          // sib_itimer_lvl_1 = sib_itimer_edg_1;
          // PULSE TIMER 2
          if(sib_itimer_ena_2){
            sib_itimer_ctr_2--; // Decrement              
            //            logmsgf("SIB: IT2: %X\n",sib_itimer_ctr_2);
            if(sib_itimer_ctr_2 == 0){
              logmsg("SIB: Interval timer 2 expired\n");
              // Write 0xFF to this location if interrupts are enabled
              if(sib_config_reg&0x2){
                sib_nubus_io_request(NB_BYTE_WRITE, sib_lintv_expired_vector, 0xFF);
              }
              sib_itimer_ena_2 = 0;
            }
          }
        }else{
          logmsgf("SIB: Interval Timer 1 expired in TC Mode\n");
          // Terminal Count Mode
          // This one shouldn't be in TC mode and can't interrupt.
          sib_itimer_ena_1 = 0;
        }
      }
    }
  }
  
  // RTC 100NS
  sib_rtc_100ns++; // 100ns
  
  if((sib_rtc_100ns&0xF) > 0x9){ // 900 ns
    sib_rtc_100ns += 0x10;
    sib_rtc_100ns &= 0xF0;
    if((sib_rtc_100ns&0xF0) > 0x90){ // 9000 ns
      //    sib_rtc_100ns &= 0xF;
      sib_rtc_100ns = 0;
      sib_rtc_ticks++; // 1000 nanoseconds have passed.
  
      // INTERNAL TICKS (750 per 10ms nominally, 650 for GDOS purposes)
      if(sib_rtc_ticks > 650){
        sib_rtc_ticks = 0; // Reset
        //    logmsgf("Over by %ld\n",sib_rtc_ticks);
        sib_rtc_10ms++; // Tick 10ms
        // RTC 10MS
        if((sib_rtc_10ms&0xF) > 9){
          sib_rtc_10ms += 0x10; // Carry over
          sib_rtc_10ms &= 0xF0; // Reset to 0
          // Check for interrupt
          if(sib_rtc_int_control&0x02){
            logmsg("SIB: 10hz Interrupt\n");      
            sib_rtc_int_control ^= 0x02; // Disable future interrupt
            sib_rtc_int_status |= 0x02; // Tell CPU what happened
            // Write 0xFF to this location if interrupts are enabled
            if(sib_config_reg&0x02 && !sib_rtc_int_sent){
              sib_nubus_io_request(NB_BYTE_WRITE, sib_rtc_int_vector, 0xFF);
              sib_rtc_int_sent=1;
            }
          }       
          // RTC 100MS
          if((sib_rtc_10ms&0xF0) > 0x90){
            //    sib_rtc_sec++;       // Tick 1sec
            // logmsgf("SIB: TICK!\n");
            // Check for interrupt
            if(sib_rtc_int_control&0x04){
              logmsg("SIB: 1hz Interrupt\n");      
              sib_rtc_int_control ^= 0x04; // Disable future interrupt
              sib_rtc_int_status |= 0x04;  // Tell CPU what happened
              // Write 0xFF to this location if interrupts are enabled
              if(sib_config_reg&0x02 && !sib_rtc_int_sent){
                sib_nubus_io_request(NB_BYTE_WRITE, sib_rtc_int_vector, 0xFF);
                sib_rtc_int_sent=1;
              }
            }     
            sib_rtc_10ms = 0; // Reset to 0
          }
        }
      }
    }
  }

  // Test for RTC match interrupt
  if(sib_rtc_int_control&0x01){ // COMPARE armed
    if(sib_rtc_month == sib_ram_month &&
       sib_rtc_date == sib_ram_date &&
       sib_rtc_dow == sib_ram_dow &&
       sib_rtc_hour == sib_ram_hour &&
       sib_rtc_min == sib_ram_min &&
       sib_rtc_sec == sib_ram_sec &&
       sib_rtc_10ms == sib_ram_10ms &&
       sib_rtc_100ns == sib_ram_100ns){
      
      logmsg("SIB: TIMER MATCH INTERRUPT\n");      
      sib_rtc_int_control ^= 0x01; // Disable future interrupt
      sib_rtc_int_status |= 0x01; // Tell CPU what happened
      sib_rtc_100ns = sib_ram_100ns; // Equalize
      logmsgf("RTC Interrupt Vector = 0x%lX\n",sib_rtc_int_vector);
      
      // Write 0xFF to this location if interrupts are enabled
      if(sib_config_reg&0x02 && !sib_rtc_int_sent){
        sib_nubus_io_request(NB_BYTE_WRITE, sib_rtc_int_vector, 0xFF);
        sib_rtc_int_sent=1;
      }
    }
  }
  
  // Check retrace PI
  if(sib_retrace_pi > 0){
    // Add another
    sib_retrace_pi++;
    // This pulse is nominally every 16.67 milliseconds apart
    if(sib_retrace_pi == 116690){ 
      // Loop
      sib_retrace_pi = 1;
      // Generate PI
      if ((sib_config_reg&2) && sib_gcmd_ack_vector && sib_gcmd_ack_sent == 0) {
        // logmsgf("SIB: VERTICAL RETRACE INTERRUPT GENERATED\n");
        sib_nubus_io_request(NB_BYTE_WRITE, sib_gcmd_ack_vector, 0xFF);
        sib_gcmd_ack_sent = 1;
        sib_crt_r1a |= 0x80;
      }
    }        
  }

  // Check sound parity
  if(sib_monitor_ctl_reg&0x800 && snd_parity_pi == 0){
    // PI if programmed
    if ((sib_config_reg&2) != 0 && (sib_int_diag_reg&0x80) != 0 && sib_sound_parity_vector != 0) {
      snd_parity_pi=1;
      logmsgf("SIB: SOUND BAD PARITY PI - Data %lX, Correct Parity %X, Vector %lX\n",
              sib_sound_control,genparity(sib_sound_control),sib_sound_parity_vector);
      sib_nubus_io_request(NB_BYTE_WRITE, sib_sound_parity_vector, 0xFF);
    }
  }
  
  // Check keyboard RTT and interrupt  
  if (keyboard_usart_rx_bot != keyboard_usart_rx_top){
    // Generate interrupt if requested
    if ((sib_config_reg&2) && sib_kbd_rtt_vector && sib_kbd_rtt_sent == 0) {
      //      logmsgf("SIB: Sending KBD RTT INT for Key %X\n",keyboard_usart_rx_fifo[keyboard_usart_rx_bot]&0xFF);
      sib_nubus_io_request(NB_BYTE_WRITE, sib_kbd_rtt_vector, 0xFF);
      sib_kbd_rtt_sent = 1;
    }
  }
}


/*
 * Local Variables:
 * indent-tabs-mode:nil
 * c-basic-offset:2
 * End:
*/
