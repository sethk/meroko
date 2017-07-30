/* 8MB Memory Board implementation
 
 This file is subject to the terms and conditions of the GNU General Public
 License.  See the file COPYING in the main directory of this archive for
 more details.

 $Id$
*/

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

#include "mem8.h"
#include "meroko.h"
#include "nubus.h"
#include "localbus.h"
#include "nupi.h"
#include "sib.h"
#include "raven_cpu.h"

#define RAM_TOP 0x800000
#define PTY_TOP 0x100000

// PARITY STORAGE: 8 bytes per byte.
// Rotate address right 3 to get offset, 7-0 is rotation count.

unsigned char MEM_ROM[2048];
unsigned char MEM_RAM[RAM_TOP];
unsigned char MEM_PTY[PTY_TOP];

int mem_tracerq=0;

// Registers
unsigned long mem_test_reg = 0;
unsigned char mem_base_reg=0xF4;
unsigned char mem_failure_reg=0;
unsigned long mem_term_reg=0; 
unsigned long mem_xfer_status_reg=0;
unsigned char mem_ptyf_bits=0; // Parity Failed bits
unsigned char mem_ptyu_bits=0; // Parity Used bits

#define PROM_FNAME "proms/2243924-2_8MB"

/* INTERNAL BUS */
unsigned int ITbus_error=0;
unsigned int ITbus_Address=0;
unsigned int ITbus_Request=0;
unsigned int ITbus_acknowledge=0;
uint8 * IBD;

// Registers
unsigned char mem8_config_register = 0x0; 

// Functions
static inline void mem8_itbus_io();
void nubus_io_pulse();
void mem8_nubus_io();

void mem8_init(){
  FILE *romfile;
  int x=0;

  bzero(MEM_ROM,2048);
  bzero(MEM_RAM,RAM_TOP);

  romfile = fopen(PROM_FNAME,"r");
  if(romfile == NULL){
    perror("mem8-fopen");
    exit(-1);
  }
  x=0;
  while(x < 2048){
    fread(&MEM_ROM[0x400^x],1,1,romfile);
    x++;
  }

  fclose(romfile);
}

inline unsigned int genparity(unsigned char data){
  // New Method
  data = data ^ (data >> 4);
  data = data ^ (data >> 2);
  data = data ^ (data >> 1);
  return(data&1);
}

static inline unsigned int storepty(unsigned long address,unsigned char data){
  unsigned long ptyadr;
  unsigned int bitcnt;
  unsigned char stgbit;
  unsigned char ptybit;

  ptyadr = (address>>3);
  bitcnt = (address&0x07);

  stgbit = (0x01 << bitcnt);

  // Generate parity to store
  if((mem_test_reg&0x1000) == 0x1000){
    ptybit = (((mem_test_reg&0xF00)>>(8+(address&0x3)))&0x01);
    // The parity-test register bits are inverted.
    ptybit ^= 0x01; // and flip
    /*    if(mem_tracerq){ 
      logmsgf("PTY-STOR: %X\n",ptybit);
      } */
  }else{
    ptybit = genparity(data);
  }

  if(ptybit==0){
    MEM_PTY[ptyadr] &= (~stgbit);
  }else{
    MEM_PTY[ptyadr] |= stgbit;
  }
  mem_ptyu_bits |= (ptybit << (address&0x03));
  return(ptybit);
}

static inline unsigned char fetchpty(unsigned long address){
  unsigned long ptyadr;
  unsigned int bitcnt;
  unsigned char stgbit;

  ptyadr = (address>>3);
  bitcnt = (address&0x07);
  stgbit = MEM_PTY[ptyadr];
  stgbit >>= bitcnt;
  return(stgbit&1);  
}

static inline unsigned char chkpty(unsigned long address){
  // Parity Test
  int x,y;
  x = fetchpty(address); 
  y = genparity(MEM_RAM[address]);

  // Replace fetched parity if testing?
  if((mem_test_reg&0x1000) == 0x1000){
    x = ((mem_test_reg&0xF00)>>(8+(address&0x3)));
    // The parity-test register bits are inverted.
    x = ((x&0x1)^0x1); // and flip
  }
  
  if(x != y){
    ITbus_error = 1;
    mem_ptyf_bits |= (0x01 << (address&0x03));
    if((mem_test_reg&0x1000) != 0x1000){
      logmsgf("PTY-FAIL OUTSIDE TEST @ 0x%lX, pty %d %d\n",address,x,y);
    }
  }else{
    mem_ptyu_bits |= (1 << (address&0x03));
  }
  return(x);
}

/* NUBUS MASTER */
void nubus_io_pulse(){
  switch(Nubus_Busy){

  case 2: // Start!
    // Having taken a short delay, handle the request.
    {
      int NUbus_Slot = (NUbus_Address&0xFF000000)>>24;
      switch(NUbus_Slot&0xFF){

#ifdef ENABLE_ENET
      case 0xF1: // Ethernet
        enet_nubus_io();
        break;
#endif

      case 0xF2: // NuPI
	nupi_nubus_io();
	break;

      case 0xF4: // Memory
	mem8_nubus_io();
	break;

      case 0xF5: // SIB	
	sib_nubus_io();
	break;

      case 0xF6: // CPU
	raven_nubus_io();
	break;
	
      default:
        if(NUbus_Slot == mem_base_reg){
          mem8_nubus_io();
        }
      }

#ifdef EXTRA_CHECKS
      if((NUbus_Request&0x10)==0 && NUbus_acknowledge > 0){
	if(NUbus_Request == VM_BYTE_READ){
	  // Insanity Test
	  NUbus_Data &= (0xFF<<(8*(NUbus_Address&0x03)));
	}
      }
#endif
    }
    break;
    // NUbus IO devices can pick up their data at clock 1.

  case 1: // Deadline!
    // Anyone take this request?
    // If this is a CPU request, load local bus here.
    if((NUbus_Request&0x10)==0){
      // CPU nubus request. Load termination data.
      mem_xfer_status_reg &= 0xFF00; // Off status info
      if(NUbus_Address&0x01){
        mem_xfer_status_reg |= 0x10;
      }
      if(NUbus_Address&0x02){
        mem_xfer_status_reg |= 0x20;
      }
      switch(NUbus_Request&0x0F){
      case 0: // WORD READ
        mem_xfer_status_reg |= 0x0C; break;
      case 1: // WORD WRITE
        mem_xfer_status_reg |= 0x04; break;
      case 2: // BYTE READ
        mem_xfer_status_reg |= 0x08; break;
      case 3: // BYTE WRITE
        break; // 0
      }
      // TM0 and TM1 are fine
      /* Post-transfer TM0 and TM1 codes:
         0 = Transfer Complete
         1 = Error
         2 = Nubus Timeout
         3 = Try Again Later
      */
      if(NUbus_acknowledge > 0){
#ifdef EXTRA_CHECKS
	if(NUbus_Request == VM_BYTE_READ){
	  // Insanity Test
	  NUbus_Data &= (0xFF<<(8*(NUbus_Address&0x03)));
	}      
#endif
        LCbus_error=NUbus_error;
        LCbus_acknowledge=NUbus_acknowledge;
        LCbus_Data=NUbus_Data;
        mem_xfer_status_reg &= 0x3FFF; // Clear NXM and PTY ERR
      }else{
	// It's ours, but wasn't accepted
	NUbus_error = 1;
        LCbus_error = 1;
        // Fill out fault register
        mem_xfer_status_reg &= 0xC0FF;
        if(NUbus_Address&0x01){
          mem_xfer_status_reg |= 0x100;
        }
        if(NUbus_Address&0x02){
          mem_xfer_status_reg |= 0x200;
        }
        mem_xfer_status_reg |= 0x880; // Nubus Timeout (current and previous operation)
        // Don't know what to do with CONTROL ERROR or >FS ERROR
        mem_xfer_status_reg |= 0x4000; // NXM
        LCbus_acknowledge=NUbus_acknowledge;
        LCbus_Data=NUbus_Data;
      }
    }
    break;
  }
  // Done
}

uint32 nubus_io_request(int access, uint32 address, uint32 data, int owner){

  if(Nubus_Busy > 0){
    // Stall for completion?
    while(Nubus_Busy){
      Nubus_Busy--;
      nubus_io_pulse();
      if(owner != NUBUS_ID_NUPI){ // If it's not NUPI's request
        nupi_clock_pulse(); // Allow NuPI to operate
      }
#ifdef ENABLE_ENET
      if(owner != 0xF1){ // If it's not Ethernet
        enet_clock_pulse();
      }
#endif
      // SIB pulse is not necessary on the nubus and distorts timing.
      // Or does it?
      sib_clock_pulse();
    }
  }

  NUbus_error = 0;
  NUbus_acknowledge = 0;

  // Only 3 is a legal value. All others fail the CPU selftest

  Nubus_Busy = 3; // 2 INSTRUCTIONS PASS BEFORE COMPLETION (counting this one)
  NUbus_Address = address;
  NUbus_Request = access;
  NUbus_Data = data;
  return(0);
}

/* LOCAL BUS EXCHANGE */

void mem8_lcbus_io(){
  int Dst_Card = (LCbus_Address&0xFF000000)>>24;

  // Is it mine?
  if(Dst_Card == 0xF4 || Dst_Card == mem_base_reg){
    // Yes
    // Put request on internal bus
    ITbus_error=0;
    ITbus_acknowledge=0;
    ITbus_Address=LCbus_Address;
    ITbus_Request=LCbus_Request; // Removal of NONCPU bits not required because CPU is only peer.
    IBD = (unsigned char *)&LCbus_Data;
    // Do bus pulse
    mem8_itbus_io();
    // Pull results back to local bus
    LCbus_error=ITbus_error;
    LCbus_acknowledge=ITbus_acknowledge;
    // Return
    return;
  }else{
    // No, repeat to NUbus
    nubus_io_request(LCbus_Request,LCbus_Address,LCbus_Data,0xF6);
  }
}

/* NUBUS SLAVE */
void mem8_nubus_io(){
  // Put request on internal bus
  ITbus_error=0;
  ITbus_acknowledge=0;
  ITbus_Address=NUbus_Address;
  ITbus_Request=(NUbus_Request&0xF); // Remove NON-CPU bits
  IBD = (unsigned char *)&NUbus_Data;
  // Run bus
  mem8_itbus_io();
  // Pull results back to nubus
  NUbus_error=ITbus_error;
  NUbus_acknowledge=ITbus_acknowledge;
  // Return
  return;
}

/* Internal bus IO  */
static inline void mem8_itbus_io(){
  unsigned long ITbus_Addr = (ITbus_Address&0xFFFFFF);

  // Handle RAM here
  if((ITbus_Request == VM_READ || ITbus_Request == VM_BYTE_READ) && ITbus_Addr < RAM_TOP){
    unsigned long RAM_Addr = ITbus_Addr;

    // Initialize
    // RAM_Addr = ITbus_Addr;
    mem_failure_reg = 0; // Reset
    mem_ptyu_bits = 0;
    mem_ptyf_bits = 0;
    
    if(ITbus_Request == VM_READ){ // Read four bytes
      switch(RAM_Addr&0x03){
      case 1: // Read Low Half
        IBD[1] =  MEM_RAM[RAM_Addr];
        chkpty(RAM_Addr);        
        IBD[0] = MEM_RAM[RAM_Addr-1]; 
        chkpty(RAM_Addr-1);
        break;
        
      case 2: // Block Transfer (ILLEGAL)
        logmsg("MEM8: BLOCK READ REQUESTED\n");
        cpu_die_rq=1;
        break;
        
      case 3: // Read High Half
        IBD[3] = MEM_RAM[RAM_Addr];
        chkpty(RAM_Addr);        
        IBD[2] = MEM_RAM[RAM_Addr-1]; 
        chkpty(RAM_Addr-1);
        break;
      
      case 0:
	// Full word read
	IBD[3] = MEM_RAM[RAM_Addr+3];
	chkpty(RAM_Addr+3);
	IBD[2] = MEM_RAM[RAM_Addr+2]; 
	chkpty(RAM_Addr+2);
	IBD[1] = MEM_RAM[RAM_Addr+1]; 
	chkpty(RAM_Addr+1);
	IBD[0] = MEM_RAM[RAM_Addr+0];
	chkpty(RAM_Addr+0);
        break;
      }    
    }else{
      unsigned int ptydata;
      // BYTE READ
      IBD[(RAM_Addr&0x3)] = MEM_RAM[RAM_Addr+0];
      ptydata = chkpty(RAM_Addr+0);
    }

    if(mem_ptyf_bits != 0){
      mem_term_reg |= 0x8000;
      mem_failure_reg = (RAM_Addr&0x03);
      mem_failure_reg <<= 5;
      mem_failure_reg |= 0x10; // Was 0x40
      // mem_failure_reg |= (mem_ptyf_bits^0xF);
      // 0 means failure?
    }else{
      mem_term_reg &= 0x7FFF;
      mem_failure_reg = (RAM_Addr&0x03);
      mem_failure_reg <<= 5;
      mem_failure_reg |= 0x0F; // was ptyu
    }

    ITbus_acknowledge=1;
    return;
  }

  if((ITbus_Request == VM_WRITE || ITbus_Request == VM_BYTE_WRITE) && ITbus_Addr < RAM_TOP){
    unsigned long RAM_Addr = ITbus_Addr;

    mem_failure_reg = 0; // Reset
    mem_ptyu_bits = 0;
    mem_ptyf_bits = 0;

    // Store
    if(ITbus_Request == VM_BYTE_WRITE){ // && (RAM_Addr&0x3) != 0){
      // BYTE WRITE
      unsigned int ptydata;
      unsigned int bcell = ITbus_Addr&0x3;
      
      MEM_RAM[RAM_Addr+0] = IBD[bcell];               // Store data
      ptydata = storepty(RAM_Addr,IBD[bcell]);        // And parity
      if(ptydata != 0){
	mem_ptyu_bits |= 0x0F;
      }else{
	mem_ptyu_bits &= 0xF0;
      }
    }else{
      // WORD WRITE
      switch(RAM_Addr&0x03){
      case 1: // Write low half
        MEM_RAM[RAM_Addr-1] = IBD[0];     
        storepty(RAM_Addr-1,IBD[0]);        
        MEM_RAM[RAM_Addr] = IBD[1]; 
        storepty(RAM_Addr,IBD[1]);
        break;

      case 2: // BLOCK TRANSFER (ILLEGAL)
        logmsg("MEM8: BLOCK TRANSFER REQUESTED\n");
        cpu_die_rq=1;
        break;
        
      case 3: // Write high half
        MEM_RAM[RAM_Addr-1] = IBD[2];     
        storepty(RAM_Addr-1,IBD[2]);
        MEM_RAM[RAM_Addr] = IBD[3]; 
        storepty(RAM_Addr,IBD[3]);
        break;
      
      case 0: // Full Word
        MEM_RAM[RAM_Addr] = IBD[0];     
        storepty(RAM_Addr,IBD[0]);	
	MEM_RAM[RAM_Addr+1] = IBD[1]; 
	storepty(RAM_Addr+1,IBD[1]);       
	MEM_RAM[RAM_Addr+2] = IBD[2]; 
	storepty(RAM_Addr+2,IBD[2]);	
	MEM_RAM[RAM_Addr+3] = IBD[3]; 
	storepty(RAM_Addr+3,IBD[3]);
        break;
      }
    }
      
    mem_term_reg &= 0x7FFF;
    mem_failure_reg = (RAM_Addr&0x03);
    mem_failure_reg <<= 5;
    mem_failure_reg |= mem_ptyu_bits;

    ITbus_acknowledge=1;
    return;
  }

  // Handle ROM here
  if((ITbus_Request == VM_READ || ITbus_Request == VM_BYTE_READ) && ITbus_Addr >= 0xFFE000){
    unsigned int ROMaddr = ((ITbus_Addr >> 2)&0x7FF);

    if(ITbus_Addr&0x3){logmsg("MEM8: ODD ROM ADDR\n"); cpu_die_rq=1; }
   
    IBD[0] = MEM_ROM[ROMaddr];

    ITbus_acknowledge=1;
    return;
  }

  if(ITbus_Request == VM_READ){
    switch(ITbus_Addr){
    case 0xFFC014: // NUBUS TERMINATION & ERROR LATCH REGISTER
      /* BITS   WHAT
          7:0   LAST SLAVE XFER STATUS
                0 = 0
                1 = 0
                2 = MYTM0-
                3 = MYTM1-
                4 = MYAD00
                5 = MYAD01
                6 = TM0-
                7 = TM1-
         13:8   Last NXM XFER status
                8 = AD00
                9 = AD01
                10= TM0- ACCESS
                11= TM1- ACCESS
                12= CONTROL ERROR
                13= >FS
         14     NXM ERROR bit
         15     PTY ERROR bit
      */
      IBD[0] = mem_xfer_status_reg&0xFF;
      IBD[1] = (mem_xfer_status_reg&0xFF00)>>8;
      ITbus_acknowledge=1;
      return;
      break;
    }
  }

  if(ITbus_Request == VM_BYTE_WRITE){
    switch(ITbus_Addr){

    case 0xFFC000: // Configuration Register
      /* BITS ARE:
	 0x1 = BOARD RESET (WRITE ONLY)
	 0x4 = BOARD LED (RW?)
      */
      if((IBD[0]&0x01)==0x01){ 
	// mem8_config_register = 0; 
	// logmsg("MEM8: RESET\n");
	mem_test_reg = 0;
	mem_base_reg = 0xF4;
      }
      if((IBD[0]&0x04)==0x04){ 
	mem8_config_register |= 0x04; 
      }else{ 
	mem8_config_register &= (~0x04); 
      }
      ITbus_acknowledge=1;
      // cpu_die_rq=1;
      return;

    case 0xFFC008: // Base Register
      mem_base_reg = IBD[0];
      ITbus_acknowledge=1;
      return;

    case 0xFFC010: // Failure Location Latch
      mem_test_reg = IBD[1]; // LOAD THIS
      mem_test_reg <<= 8;    // Don't know why
      ITbus_acknowledge=1;
      return;
    
    case 0xFFC011: // Test Register
      /* BITS ARE:
	 100 - Test Parity 0 (0x000000FF)
	 200 - Test Parity 1 (0x0000FF00)
	 400 - Test Parity 2 (0x00FF0000)
	 800 - Test Parity 3 (0xFF000000)
	1000 - Test Enable
      */
      mem_test_reg = IBD[1];
      mem_test_reg <<= 8;      
      ITbus_acknowledge=1;
      return;
    }
  }
  if(ITbus_Request == VM_BYTE_READ){
    switch(ITbus_Addr){
    case 0xFFC000: // Configuration Register
      /* BITS ARE:
	 0x1 = BOARD RESET (WRITE ONLY)
	 0x4 = BOARD LED (RW?)
      */
      IBD[0] = mem8_config_register;
      ITbus_acknowledge=1;
      return;

    case 0xFFC008: // Base Register
      IBD[0] = mem_base_reg;
      ITbus_acknowledge=1;
      return;

    case 0xFFC010: // Failure Location Latch
      IBD[0] = mem_failure_reg;
      ITbus_acknowledge=1;
      return;      

    case 0xFFC011: // Test Register
      IBD[1] = (mem_test_reg&0xFF00)>>8;
      ITbus_acknowledge=1;
      return;
    }
  }

  logmsgf("MEM8: Unknown ITbus IO-(%ld): %lX\n",ITbus_Request,ITbus_Addr);
  cpu_die_rq=1;
}


/*
 * Local Variables:
 * indent-tabs-mode:nil
 * c-basic-offset:2
 * End:
*/
