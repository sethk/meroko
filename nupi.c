/* NUbus Peripheral Interface - Implementation

 This file is subject to the terms and conditions of the GNU General Public
 License.  See the file COPYING in the main directory of this archive for
 more details.

 $Id$
*/

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>

#include "nupi.h"
#include "meroko.h"
#include "nubus.h"
#include "raven_cpu.h"

unsigned char NUPI_ROM[0x4000]; // 16K ROM
unsigned char NUPI_RAM[0x1000]; // 4K MPU RAM

unsigned long NUPI_XFER_BUFFER[0x10000]; // 10KW transfer area
unsigned int nupi_xfer_pointer=0; 
unsigned int nupi_unit_pointer=0;

// The NuPI has a 68000 MPU on board that runs the ROM code.
// The MPU is clocked at 10MHz.

#define nupi_nubus_io_request(access, address, data) \
	nubus_io_request((access), (address), (data), NUBUS_ID_NUPI);

// Registers
unsigned long nupi_test_addr=0;
unsigned long nupi_test_indx=0;
unsigned int  nupi_test_loop=0;
// Events are generated at this address when something really very bad happens.
unsigned long nupi_special_event_addr=0;

unsigned long nupi_cmd_addr = 0;
unsigned long nupi_cmd_prog = 0;
unsigned long nupi_cmd_dadr = 0;
unsigned long nupi_cmd_word = 0;
int nupi_go = 0;

unsigned int nupi_retry_count = 0;
unsigned int nupi_dead_cmd_disable = 0;
unsigned int nupi_periodic_poll_disable = 0;
unsigned int nupi_retry_disable = 0;
unsigned int nupi_ecc_disable = 0;

// Scatter transfers
unsigned int nupi_scatter_block=0;   // Which scatter-block are we on?
unsigned int nupi_scatter_index=0;   // How far in are we?
unsigned int nupi_scatter_address=0; // Where is it going?
unsigned int nupi_scatter_length=0;  // How much is going there?
unsigned int nupi_scatter_phase=0;   // What are we doing to get there?

// SCSI passthrough command parameter block
unsigned int nupi_scsi_nubus_addr=0;
unsigned int nupi_scsi_buffer_len=0;
unsigned int nupi_scsi_status_ptr=0;
unsigned int nupi_scsi_status_ptr_len=0;
unsigned int nupi_scsi_cmd_blk_ptr=0;
unsigned int nupi_scsi_cmd_blk_len=0;
// SCSI passthrough result buffer
unsigned char NUPI_SCSI_CMD_RESULT[1024]; // 256 words
unsigned char nupi_scsi_result_index=0;   // Index into the above
unsigned char nupi_scsi_result_pc=0;      // Sub-PC for result-writing state

// Fields in command block
// Word 0
unsigned char nupi_dst_bit,nupi_formatter_bit,nupi_command;
unsigned char nupi_event,nupi_scatter,nupi_swap_completion;
unsigned char nupi_formatterselect,nupi_devselect;
// Word 1
unsigned long nupi_status;
// Word 2
unsigned long nupi_buffer_pointer;
// Word 3
unsigned long nupi_transfer_count;
// Word 4
unsigned long nupi_device_block_addr;
// Word 5
unsigned long nupi_event_gen_addr;
// Word 6 and 7 are reserved.

// A device
struct nupi_device_entry{
  int fd;                           // Disk file descriptor
  unsigned int last_cmd_option;     // Option field of last command completed
  unsigned int last_cmd;            // Last command completed
  unsigned int cmd_option;          // Option of currently executing command
  unsigned int cmd;                 // Currently executing command
  unsigned int removable;           // Removable device
  unsigned int busy;                // Device is busy
  unsigned int attention;           // Unit requires service
  unsigned int indt_status;         // Unit has indeterminate status
  unsigned int writeprotect;        // Unit is write-protected
  unsigned int overtemp;            // Unit has overtemperature alarm
  unsigned int selftest;            // Unit self-test status
  unsigned int error;               // Unit has error condition
  unsigned int offline;             // Unit is offline
  unsigned int devtype;             // Device type
};

struct nupi_device_entry nupi_device[20];

// NUPI devices:
// 00 = Formatter 0
// 01 = Formatter 1
// 02 = Formatter 2
// 03 = Formatter 3
// 04 = Formatter 4
// 05 = Formatter 6
// 06 = Formatter 7
// 07 = Formatter 0,Device 0
// 08 = Formatter 0,Device 1
// 09 = Formatter 1,Device 0
// 10 = Formatter 1,Device 1
// 11 = Formatter 2,Device 0
// 12 = Formatter 2,Device 1
// 13 = Formatter 3,Device 0
// 14 = Formatter 3,Device 1
// 15 = Formatter 4,Device 0
// 16 = Formatter 4,Device 1
// 17 = Formatter 6,Device 0
// 18 = Formatter 6,Device 1
// 19 = Formatter 7,Device 0
// 20 = Formatter 7,Device 1

#define PROM1_FNAME "proms/2238056-5_NUPI"
#define PROM2_FNAME "proms/2238057-5_NUPI"

void nupi_init(){
  FILE *romfile1,*romfile2;
  int x=0;
  unsigned long tmp;
  romfile1 = fopen(PROM1_FNAME,"r");
  romfile2 = fopen(PROM2_FNAME,"r");
  if(romfile1 == NULL){
    perror("nupi-fopen-1");
    exit(-1);
  }
  if(romfile2 == NULL){
    perror("nupi-fopen-2");
    exit(-1);
  }
  while(x < 0x4000){
    fread(&tmp,1,1,romfile1);
    NUPI_ROM[x+3] = tmp;
    x++;
    fread(&tmp,1,1,romfile2);
    NUPI_ROM[x+1] = tmp; 
    x++;
    fread(&tmp,1,1,romfile1);
    NUPI_ROM[x-1] = tmp; 
    x++;
    fread(&tmp,1,1,romfile2);
    NUPI_ROM[x-3] = tmp; 
    x++;
  }
  fclose(romfile1);
  fclose(romfile2);
  /*
  printf("NUPI ROM BYTES = %X %X %X %X - %X %X %X %X - %X %X %X %X - %X %X %X %X\n",
	 NUPI_ROM[0x3B60],NUPI_ROM[0x3B61],NUPI_ROM[0x3B62],NUPI_ROM[0x3B63],
	 NUPI_ROM[0x3B64],NUPI_ROM[0x3B65],NUPI_ROM[0x3B66],NUPI_ROM[0x3B67],
	 NUPI_ROM[0x3B68],NUPI_ROM[0x3B69],NUPI_ROM[0x3B6A],NUPI_ROM[0x3B6B],
	 NUPI_ROM[0x3B6C],NUPI_ROM[0x3B6D],NUPI_ROM[0x3B6E],NUPI_ROM[0x3B6F]);
  exit(-1);
  */  

  // Initialize devices to disconnected-offline
  {
    int x=0;
    while(x<7){
      nupi_device[x].offline=1;
      nupi_device[x].last_cmd_option=0;
      nupi_device[x].last_cmd=0;
      nupi_device[x].cmd_option=0;
      nupi_device[x].cmd=0;       
      nupi_device[x].removable=0; 
      nupi_device[x].busy=0;      
      nupi_device[x].attention=0; 
      nupi_device[x].indt_status=0;
      nupi_device[x].writeprotect=0;
      nupi_device[x].overtemp=0;    
      nupi_device[x].selftest=0;    
      nupi_device[x].error=0;       
      nupi_device[x].devtype=0;     
      x++;
    }
  }
  // Set formatter 0,1,3 to online
  nupi_device[0].offline = 0;
  nupi_device[1].offline = 0;
  nupi_device[3].offline = 0;
  // Attach drives 0,1,4
  nupi_device[7].fd = open("X1-DISKS/c0-d0.dsk",O_RDWR);
  nupi_device[9].fd = open("X1-DISKS/c0-d1.dsk",O_RDWR); 
  // Was 15 in Nevermore
  nupi_device[13].fd = open("X1-DISKS/c2-d0.dsk",O_RDWR);
  
  if(nupi_device[7].fd < 0){
    perror("Disk-0:open");
    exit(-1);
  }
  if(nupi_device[9].fd < 0){
    perror("Disk-1:open");
    exit(-1);
  }
  if(nupi_device[13].fd < 0){
    perror("Disk-2:open");
    exit(-1);
  }
  nupi_device[7].offline=0;
  nupi_device[8].offline=1;
  nupi_device[9].offline=0;
  nupi_device[10].offline=1;
  nupi_device[13].offline=0;
  nupi_device[14].offline=1; 

  nupi_device[7].devtype = 0x02; // HARD DISK DRIVE
  nupi_device[9].devtype = 0x02; // HARD DISK DRIVE 
  nupi_device[13].devtype = 0x02; // HARD DISK DRIVE
}

int nupi_trace_rq = 0;

void nupi_nubus_io(){
  unsigned long NUbus_Addr = ldb(NUbus_Address,24,0);  

  //  cpu_die_rq=1;

  // Exclude ROM here
  if((NUbus_Request == VM_READ || NUbus_Request == VM_BYTE_READ) && NUbus_Addr >= 0xFFC000){
    unsigned int ROMaddr = (NUbus_Addr&0x3FFF);

    if(NUbus_Request == VM_READ){
      // Read other 3 bytes (This is probably wrong)
      NUbus_Data =  NUPI_ROM[ROMaddr+3]; NUbus_Data <<= 8;
      NUbus_Data |= NUPI_ROM[ROMaddr+2]; NUbus_Data <<= 8;
      NUbus_Data |= NUPI_ROM[ROMaddr+1]; NUbus_Data <<= 8; 
      NUbus_Data |= NUPI_ROM[ROMaddr+0]; // NUbus_Data &= 0xFF;
    }else{
      NUbus_Data = NUPI_ROM[ROMaddr+0];
      NUbus_Data <<= (8*(NUbus_Addr&0x3));
      //      NUbus_Data &= 0xFF;
    }
    NUbus_acknowledge=1;
    // cpu_die_rq=1;

    /*
    if(nupi_trace_rq){
      logmsgf("NuPI: IO-(%ld) data %lX for address %lX\n",
              NUbus_Request,NUbus_Data,NUbus_Addr);
    }
    */
   
    return;
  }

  // Registers
  if(NUbus_Request == VM_BYTE_READ){
    switch(NUbus_Addr){
    case 0xD40002: // FLAG REGISTER
      // BITS ARE:
      // 0x10000 = 0-Self-Test Completed
      // 0x20000 = 0-Self-Test Passed
      // 0x40000 = 0-SCSI Test Passed
      // One indicates failures
      // We will cheat and return all passes.
      NUbus_Data = 0x0;
      NUbus_acknowledge = 1;
      return;

    case 0xE0000B: // Configuration Register
      /* BITS ARE:
	 01 = RESET
	 02 = BUS-MASTER ENABLE
	 04 = FAULT-LED
	 08 = SYSTEM-BUS-TEST
	 30 = MUST-BE-ZERO
	 40 = FAILURE-OVERRIDE
      */

      NUbus_Data = NUPI_RAM[0xB];
      NUbus_Data <<= 24; 
      NUbus_Data = 0;
      NUbus_acknowledge=1;
      return;	
    }
  }

  if(NUbus_Request == VM_WRITE){
    switch(NUbus_Addr){
    case 0xE00004: // Command Address Register
      {
	nupi_cmd_addr = NUbus_Data;
	nupi_go = 1;
	NUbus_acknowledge=1;
      }
      return;
      
    default:
      logmsgf("NuPI: Unknown NUbus word-write - 0x%lX @ %lX\n",
	      NUbus_Data,NUbus_Addr);
      cpu_die_rq=1;      
    }
  }

  if(NUbus_Request == VM_BYTE_WRITE){
    unsigned char NUPI_DATA;

    // Derotate
    NUPI_DATA = ((NUbus_Data >> (8*(NUbus_Addr&0x3)))&0xFF);

    switch(NUbus_Addr){
    case 0xE0000B: // Configuration Register
      // Handle bits that must be handled
      if(NUPI_DATA&0x1){ 
	logmsg("NUPI: RESET\n"); 
	NUPI_RAM[0xB] = 0; // Clobber status
        nupi_go = 0; // HALT
      }
      if(NUPI_DATA&0x8){
	logmsg("NUPI: SYSTEM-BUS-TEST STARTED\n");
	// This causes the NuPI to become a master, copy the configuration ROM to the memory location indicated in
	// E0000F, read it back, and repeat this three times.
	nupi_cmd_prog=36; nupi_go=1; // Start NUPI CPU
      }
      if(NUPI_DATA&0x2){
	logmsg("NUPI: BUS-MASTER ENABLED\n"); 
        if(nupi_go < 0){ nupi_go = 1; } // RESTART NUPI IF STOPPED
      }else{
        logmsg("NUPI: BUS-MASTER DISABLED\n");
        if(nupi_go > 0){ nupi_go = -1; } // STOP NUPI IF RUNNING
      }
      NUPI_DATA &= 0x0A; // Don't write reset or LED bit into RAM
      break;

    case 0xE0000F: // DMA-Test-Register
      // Safe to load without intervention
      // Contains memory test address for bus test.
      // 0xF0 = PA:27-24
      // 0x0F = PA:15-12
      // So for 0xAB, PA = 0x0A00B000
      nupi_test_addr = 0xF0000;                // F0000
      nupi_test_addr |= (NUPI_DATA&0xF0) << 8; // FA000
      nupi_test_addr |= (NUPI_DATA&0xF);       // FA00B
      nupi_test_addr <<= 12;                   // FA00B000
      break;

    default:
      logmsgf("NuPI: Unknown NUbus byte-write - 0x%lX @ %lX\n",
	      NUbus_Data,NUbus_Addr);
      cpu_die_rq=1;
    }
    
    // handle debug window
    if(NUbus_Addr >= 0xE00000 && NUbus_Addr <= 0xE01000){
      // Microprocessor RAM area
      unsigned int NUPI_ADDR = (NUbus_Addr - 0xE00000);
      // Store
      NUPI_RAM[NUPI_ADDR] = NUPI_DATA;
      // Return
      NUbus_acknowledge=1;
      return;
    }
  }

  logmsgf("NuPI: Unknown NUbus IO-(%ld): %lX\n",
	  NUbus_Request,NUbus_Addr);
  cpu_die_rq=1;
}

inline void nupi_clock_pulse(){
  // Check things
  // Test loop finished or not running, check for commands.
  if(nupi_go > 0){
    switch(nupi_cmd_prog){
    case 0: // FETCH
      // if (cpu_die_rq) logmsgf("fetch\n");
      nupi_nubus_io_request(NB_READ,nupi_cmd_addr,0);
      nupi_cmd_prog++;
      break;
      
    case 1: // LOAD COMMAND WORD
      if(Nubus_Busy == 0){
	nupi_cmd_word = NUbus_Data;
	nupi_nubus_io_request(NB_READ,nupi_cmd_addr+0x04,0);
	nupi_cmd_prog++;
      }
      break;
      
    case 2: // LOAD STATUS WORD
      if(Nubus_Busy == 0){
	nupi_status = NUbus_Data;
	nupi_nubus_io_request(NB_READ,nupi_cmd_addr+0x08,0);
	nupi_cmd_prog++;
      }
      break;
      
    case 3: // LOAD BUFFER POINTER / PARAMETER LIST POINTER
      if(Nubus_Busy == 0){
	nupi_buffer_pointer = NUbus_Data;
#ifdef TRACELOG
        logmsgf("DEBUG: NUPI: Buffer Pointer = 0x%lX\n",nupi_buffer_pointer);
#endif
	nupi_nubus_io_request(NB_READ,nupi_cmd_addr+0x0C,0);
	nupi_cmd_prog++;
      }
      break;
      
    case 4: // LOAD TRANSFER COUNT
      if(Nubus_Busy == 0){
	nupi_transfer_count = NUbus_Data;
	nupi_nubus_io_request(NB_READ,nupi_cmd_addr+0x10,0);
	nupi_cmd_prog++;
      }
      break;

    case 5: // LOAD DEVICE BLOCK ADDRESS
      if(Nubus_Busy == 0){
	nupi_device_block_addr = NUbus_Data;
	nupi_nubus_io_request(NB_READ,nupi_cmd_addr+0x14,0);
	nupi_cmd_prog++;
      }
      break;
      
    case 6: // LOAD EVENT GENERATION ADDRESS
      if(Nubus_Busy == 0){
	nupi_event_gen_addr = NUbus_Data;
#ifdef TRACELOG
	logmsgf("DEBUG: NUPI: Event Address = 0x%lX\n",nupi_event_gen_addr);
#endif
	nupi_cmd_prog++;
      }else{
        break;
      }
      // Fall into execute

    case 7: // EXECUTE
      nupi_dst_bit = (nupi_cmd_word&0x80000000)>>31;              // The NUPI is our destination
      nupi_formatter_bit = (nupi_cmd_word&0x40000000)>>29;        // A formatter is our destination
      nupi_command = ((nupi_cmd_word&0x3F000000)>>24);            // The command itself
      nupi_event =   (nupi_cmd_word&0x800000)>>23;
      nupi_scatter = (nupi_cmd_word&0x400000)>>22;
      nupi_swap_completion = (nupi_cmd_word&0x100000)>>20;
      nupi_formatterselect = ((nupi_cmd_word&0x38)>>3);
      nupi_devselect = (nupi_cmd_word&0x1);

#ifdef TRACELOG
      logmsgf("NUPI: Got command word %lX\n",nupi_cmd_word);
#endif

      // Tell CPU we are busy
      nupi_cmd_word = 0x80000000; // BUSY code
      nupi_nubus_io_request(NB_WRITE,nupi_cmd_addr+4,nupi_cmd_word); // Write it back
#ifdef TRACELOG
      logmsgf("DEBUG: NUPI: Initiating nupi execution...\n");
#endif
      
      switch(nupi_command){

      case 0x01: // SETUP COMMAND
	// Setting nupi status
	if(nupi_dst_bit){
	  logmsgf("NUPI: SETUP COMMAND with 0x%lX words of output\n",
		  (nupi_transfer_count>>2));
	  // TRIGGER BUFFER LOAD
	  nupi_cmd_prog=20; // IO BUFFER BUSY-LOOP
	  nupi_xfer_pointer = 0;
          // Initiate request
          if(nupi_scatter){
            logmsg("SCATTER MODE DOESN'T WORK FOR SETUP YET!\n");
            cpu_die_rq=1;
          }
          nupi_nubus_io_request(NB_READ,nupi_buffer_pointer+nupi_xfer_pointer,0);
	  return;
	}
	logmsg("NUPI-Setup: Unknown destination\n");
	cpu_die_rq=1; nupi_cmd_prog=0; nupi_go=0;
	return;
	break;

      case 0x02: // REQUEST STATUS
	// Requesting nupi status?
	if(nupi_dst_bit){
	  int x,y;
	  x=0;
	  logmsgf("NUPI: status requested with 0x%lX words of output\n",
		  (nupi_transfer_count>>2));
	  // Write out NUPI status
	  NUPI_XFER_BUFFER[0] = 0x0; // No error conditions set
	  if(nupi_transfer_count > x){ // More bytes to write?
	    NUPI_XFER_BUFFER[1] = 0x0; // Selftest information
	    x += 4;
	  }
	  y=0;
	  while(nupi_transfer_count > x && y < 20){ // More bytes to write?
	    NUPI_XFER_BUFFER[2+y] = nupi_device[y].devtype;
	    NUPI_XFER_BUFFER[2+y] <<= 1;
	    NUPI_XFER_BUFFER[2+y] |= nupi_device[y].offline; 
	    NUPI_XFER_BUFFER[2+y] <<= 1;
	    NUPI_XFER_BUFFER[2+y] |= nupi_device[y].selftest; 
	    NUPI_XFER_BUFFER[2+y] <<= 1;
	    NUPI_XFER_BUFFER[2+y] |= nupi_device[y].overtemp; 
	    NUPI_XFER_BUFFER[2+y] <<= 1;
	    NUPI_XFER_BUFFER[2+y] |= nupi_device[y].writeprotect; 
	    NUPI_XFER_BUFFER[2+y] <<= 1;
	    NUPI_XFER_BUFFER[2+y] |= nupi_device[y].indt_status; 
	    NUPI_XFER_BUFFER[2+y] <<= 1;
	    NUPI_XFER_BUFFER[2+y] |= nupi_device[y].attention; 
	    NUPI_XFER_BUFFER[2+y] <<= 1;
	    NUPI_XFER_BUFFER[2+y] |= nupi_device[y].busy; 
	    NUPI_XFER_BUFFER[2+y] <<= 1;
	    NUPI_XFER_BUFFER[2+y] |= nupi_device[y].removable; 
	    NUPI_XFER_BUFFER[2+y] <<= 1;
	    NUPI_XFER_BUFFER[2+y] <<= 4; // Reserved bits
	    NUPI_XFER_BUFFER[2+y] <<= 8;	    
	    NUPI_XFER_BUFFER[2+y] |= nupi_device[y].last_cmd;
	    NUPI_XFER_BUFFER[2+y] <<= 8;
	    NUPI_XFER_BUFFER[2+y] |= nupi_device[y].last_cmd_option;
	    x += 4; y++;
	  }
	  // Next is ... wierd stuff.
	  if(nupi_transfer_count > x){ // More bytes to write?
	    logmsgf("NuPI: More status? Geez... It wants %ld more words of data.\n",
		    ((nupi_transfer_count-x)>>2));
	  }
	  // Done
	  nupi_cmd_prog=8; // WRITE RESULT
	  nupi_xfer_pointer = 0;
	  return;
	}
	if(nupi_formatter_bit){	
	  logmsg("NUPI-Request-Status: Unknown destination\n");
	  cpu_die_rq=1; nupi_cmd_prog=0; nupi_go=0;
	  return;
	}
	// Target is a device.
	{
	  int x=0,y=0;
	  logmsgf("NUPI: DRIVE STATUS RQ: Target is ID %d, LUN %d\n",
		  nupi_formatterselect,nupi_devselect);
	  logmsgf("NUPI: status requested with 0x%lX words of output\n",
		  (nupi_transfer_count>>2));
	  x = 4;
	  
	  y = 7 + (nupi_formatterselect<<1);
	  y |= nupi_devselect;
	  logmsgf("DEBUG: NUPI: Internal device # is %d\n",y);
	  NUPI_XFER_BUFFER[0] = nupi_device[y].devtype;
	  NUPI_XFER_BUFFER[0] <<= 1;
	  NUPI_XFER_BUFFER[0] |= nupi_device[y].offline; 
	  NUPI_XFER_BUFFER[0] <<= 1;
	  NUPI_XFER_BUFFER[0] |= nupi_device[y].selftest; 
	  NUPI_XFER_BUFFER[0] <<= 1;
	  NUPI_XFER_BUFFER[0] |= nupi_device[y].overtemp; 
	  NUPI_XFER_BUFFER[0] <<= 1;
	  NUPI_XFER_BUFFER[0] |= nupi_device[y].writeprotect; 
	  NUPI_XFER_BUFFER[0] <<= 1;
	  NUPI_XFER_BUFFER[0] |= nupi_device[y].indt_status; 
	  NUPI_XFER_BUFFER[0] <<= 1;
	  NUPI_XFER_BUFFER[0] |= nupi_device[y].attention; 
	  NUPI_XFER_BUFFER[0] <<= 1;
	  NUPI_XFER_BUFFER[0] |= nupi_device[y].busy; 
	  NUPI_XFER_BUFFER[0] <<= 1;
	  NUPI_XFER_BUFFER[0] |= nupi_device[y].removable; 
	  NUPI_XFER_BUFFER[0] <<= 1;
	  NUPI_XFER_BUFFER[0] <<= 4; // Reserved bits
	  NUPI_XFER_BUFFER[0] <<= 8;	    
	  NUPI_XFER_BUFFER[0] |= nupi_device[y].last_cmd;
	  NUPI_XFER_BUFFER[0] <<= 8;
	  NUPI_XFER_BUFFER[0] |= nupi_device[y].last_cmd_option;
	  if(nupi_transfer_count > x){ // More bytes to write?
	    logmsgf("NuPI: More status? Geez... It wants %ld more words of data.\n",
		    ((nupi_transfer_count-x)>>2));
	  }
	  nupi_cmd_prog=8; // WRITE RESULT
	  nupi_xfer_pointer = 0;
	  return;
	}
	break;

      case 0x10: // RESTORE DEVICE
	{
	  // For disks - Reposition to track 0, clear all faults.
	  // For tapes - rewind to BOT, clear all faults
	  // A bus reset does this to every device.
	  // We don't have to do anything yet, as no errors exist.
	  logmsgf("NUPI: Restore Device requested with 0x%lX words of output, ignored.\n",
		  (nupi_transfer_count>>2));
	  nupi_cmd_prog=8; // DONE	  
	  nupi_xfer_pointer = 0;
	  return;
	}
      	break;

      case 0x12: // READ DATA
	{
	  int y=0;
#ifdef TRACELOG
	  logmsgf("NUPI: DRIVE READ RQ: Target is ID %d, LUN %d\n",
		  nupi_formatterselect,nupi_devselect);
	  logmsgf("NUPI: Data requested with 0x%lX words of output from block 0x%lX\n",
		  (nupi_transfer_count>>2),nupi_device_block_addr);  
#endif
	  y = 7 + (nupi_formatterselect<<1);
	  y |= nupi_devselect;
	  // Reposition the file pointer.
	  lseek(nupi_device[y].fd,(nupi_device_block_addr*0x400),SEEK_SET);
          // Set up pointers
	  nupi_unit_pointer = y;
	  nupi_xfer_pointer = 0;
	  // Dispatch to read-io-busy-loop
	  nupi_cmd_prog=10; // GO 
	  return;
	}
        break;

      case 0x13: // WRITE DATA
	{
	  int y=0;
#ifdef TRACELOG
	  logmsgf("NUPI: DRIVE WRITE RQ: Target is ID %d, LUN %d\n",
		  nupi_formatterselect,nupi_devselect);
#endif
	  logmsgf("NUPI: Data requested with 0x%lX words of output for block 0x%lX\n",
		  (nupi_transfer_count>>2),nupi_device_block_addr);  
	  y = 7 + (nupi_formatterselect<<1);
	  y |= nupi_devselect;
	  // logmsgf("DEBUG: NUPI: Internal device # is %d\n",y);
	  // Reposition the file pointer.
	  lseek(nupi_device[y].fd,(nupi_device_block_addr*0x400),SEEK_SET);
          // Setup unit address and such
	  nupi_unit_pointer = y;
	  nupi_xfer_pointer = 0;
          // Test
          if(nupi_transfer_count > 0x40000){
            logmsgf("NUPI: XFER BUFFER OVERFLOW (%X WORDS)!\n",nupi_transfer_count>>2);
            cpu_die_rq=1;
          }
	  // Read in the data to be written
	  nupi_cmd_prog=20; // IO BUFFER BUSY-LOOP
	  nupi_xfer_pointer = 0;
          // Read initial word
          nupi_nubus_io_request(NB_READ,nupi_buffer_pointer+nupi_xfer_pointer,0);
	  return;
	}
        break;

      case 0x31: // SCSI PASS-THRU READ command
        {
          if(nupi_formatter_bit){
            // SCSI PASS-THRU READ
            logmsgf("NUPI: SCSI PASS-THRU READ - Target is ID %d, LUN %d\n",
		  nupi_formatterselect,nupi_devselect);

            if(nupi_scatter){
              logmsg("SCATTER MODE DOESN'T WORK FOR SCSI-PASSTHRU-READ!\n");
              cpu_die_rq=1;
            }
            nupi_xfer_pointer = 0;
            // Initiate request
            nupi_nubus_io_request(NB_READ,nupi_buffer_pointer+nupi_xfer_pointer,0);
            nupi_cmd_prog=11; // GO
            return;           
          }
          logmsgf("NuPI: Command 0x31 without FORMATTER BIT set is illegal.\n");
          cpu_die_rq=1;
          nupi_cmd_prog=0; nupi_go=0;
          return;
        }
        break;

      case 0x32: // SCSI PASS-THRU WRITE command
        {
          if(nupi_formatter_bit){
            // SCSI PASS-THRU WRITE
            logmsgf("NUPI: SCSI PASS-THRU WRITE - Target is ID %d, LUN %d\n",
		  nupi_formatterselect,nupi_devselect);

            if(nupi_scatter){
              logmsg("SCATTER MODE DOESN'T WORK FOR SCSI-PASSTHRU-WRITE!\n");
              cpu_die_rq=1;
            }
            nupi_xfer_pointer = 0;
            // Initiate request
            nupi_nubus_io_request(NB_READ,nupi_buffer_pointer+nupi_xfer_pointer,0);
            nupi_cmd_prog=11; // GO
            return;           
          }
          logmsgf("NuPI: Command 0x32 without FORMATTER BIT set is illegal.\n");
          cpu_die_rq=1;
          nupi_cmd_prog=0; nupi_go=0;
          return;
        }
        break;

      default:
        logmsgf("NuPI: Unknown Command 0x%X (For manual lookup read top byte)\n",nupi_command);
	cpu_die_rq=1;
	nupi_cmd_prog=0; nupi_go=0;
	return;
      }
      break;

    case 8: // WRITE RESULTS
      if(nupi_xfer_pointer < nupi_transfer_count){
	// Wait for the bus.
	if(Nubus_Busy == 0){
          if(nupi_scatter){
            logmsg("SCATTER MODE DOESN'T WORK FOR WRITE-RESULTS YET!\n");
            cpu_die_rq=1;
          }
	  nupi_nubus_io_request(NB_WRITE,
                                nupi_buffer_pointer+nupi_xfer_pointer,
                                NUPI_XFER_BUFFER[(nupi_xfer_pointer>>2)]);
	  NUPI_XFER_BUFFER[(nupi_xfer_pointer>>2)]=0; // Reset
	  nupi_xfer_pointer += 0x04;
	}	  
	return;
      }
      // Otherwise fall into

    case 9: // TRANSFER COMPLETED
      // Completed!
      // Notify the processor
      nupi_cmd_word = 0x40000000; // COMPLETED code
      nupi_nubus_io_request(NB_WRITE,nupi_cmd_addr+4,nupi_cmd_word); // Write it back
#ifdef TRACELOG
      logmsg("NUPI: Operation Completed!\n");
#endif
      if(nupi_event){
        //	logmsg("NUPI: POSTING EVENT\n");
	nupi_nubus_io_request(NB_WRITE,nupi_event_gen_addr,0xFF);
      }
      nupi_go=0; nupi_cmd_prog=0;
      break;	

    case 10: // READ BUSY-LOOP
      {
        unsigned char diskrs[4];

	if(nupi_xfer_pointer < nupi_transfer_count){
	  // Is memory busy?
	  if(Nubus_Busy == 0){
            if(nupi_scatter){
              // What're we doing?
              switch(nupi_scatter_phase){
              case 4: // RESET AND THEN
                nupi_scatter_index = 0;

              case 0: // READ IN ADDRESS
                //                logmsgf("Reading from 0x%lX\n",nupi_buffer_pointer+nupi_scatter_block);
                nupi_nubus_io_request(NB_READ,nupi_buffer_pointer+nupi_scatter_block,0);
                nupi_scatter_phase = 1;
                return;
                break;

              case 1: // TAKE ADDRESS AND READ IN LENGTH
                nupi_scatter_address = NUbus_Data;
                nupi_scatter_block += 0x04; // Read next word
#ifdef TRACELOG
                logmsgf("DEBUG: NUPI: SCATTER ADDRESS = 0x%lX\n",nupi_scatter_address);
#endif
                nupi_nubus_io_request(NB_READ,nupi_buffer_pointer+nupi_scatter_block,0);
                nupi_scatter_phase = 2;
                return;
                break;

              case 2: // TAKE LENGTH
                nupi_scatter_length = NUbus_Data;
                nupi_scatter_block += 0x04; // Read next word
#ifdef TRACELOG
                logmsgf("DEBUG: NUPI: SCATTER LENGTH = 0x%lX\n",nupi_scatter_length);
#endif
                nupi_scatter_phase = 3;
                return;
                break;

              case 3: // IN PROGRESS (Fall Through)
                break;

              default:
                logmsgf("NUPI: FATAL - UNKNOWN SCATTER PHASE %d IN READ-BUSY-LOOP\n",nupi_scatter_phase);
                nupi_cmd_prog=9; // DONE TRANSFERRING	  
                nupi_xfer_pointer = 0;
                cpu_die_rq=1;
                return;
              }              
            }

	    // Read a word in from the disk.
            // New Method featuring pointer abuse
	    diskrs[0]=read(nupi_device[nupi_unit_pointer].fd,(unsigned char *)&NUPI_XFER_BUFFER,1);
	    diskrs[1]=read(nupi_device[nupi_unit_pointer].fd,(unsigned char *)&NUPI_XFER_BUFFER+1,1);
	    diskrs[2]=read(nupi_device[nupi_unit_pointer].fd,(unsigned char *)&NUPI_XFER_BUFFER+2,1);
	    diskrs[3]=read(nupi_device[nupi_unit_pointer].fd,(unsigned char *)&NUPI_XFER_BUFFER+3,1);           

            // ERROR TEST
            if(diskrs[0] != 1 || diskrs[1] != 1 || diskrs[2] != 1 || diskrs[3] != 1){
              logmsgf("NUPI: DISK READ ERROR: %d %d %d %d\n",diskrs[0],diskrs[1],diskrs[2],diskrs[3]);
              cpu_die_rq=1;
            }

            if(nupi_scatter){
              nupi_nubus_io_request(NB_WRITE,nupi_scatter_address+(nupi_scatter_index),NUPI_XFER_BUFFER[0]);
              nupi_scatter_index += 0x04; // Counts bytes
              // Check completion
              if(nupi_scatter_index >= nupi_scatter_length){
#ifdef TRACELOG
                logmsgf("DEBUG: NUPI: SCATTER BLOCK COMPLETED (0x%X)\n",nupi_scatter_index);
#endif
                nupi_scatter_phase = 4;
              }                
            }else{
              nupi_nubus_io_request(NB_WRITE,nupi_buffer_pointer+nupi_xfer_pointer,NUPI_XFER_BUFFER[0]);
            }
	    NUPI_XFER_BUFFER[0] = 0;
	    nupi_xfer_pointer += 0x04; // Repeat
	    return;
	  }else{
            return;
          }	  
	}else{
#ifdef TRACELOG
	  logmsgf("DEBUG: NUPI: Read %d words of data.\n",(nupi_xfer_pointer>>2));
#endif
          nupi_scatter_index=0;
          nupi_scatter_block=0;
          nupi_scatter_phase=0;
	  nupi_cmd_prog=9; // DONE TRANSFERRING	  
	  nupi_xfer_pointer = 0;
	  return;
	}
      }
      break;

    case 11: // SCSI PASS-THRU READ SETUP - STEP 1
      if(Nubus_Busy == 0){
	nupi_scsi_nubus_addr = NUbus_Data;
	logmsgf("NUPI: SCSI NUPI Address = 0x%lX\n",nupi_scsi_nubus_addr);
        nupi_xfer_pointer += 4;
	nupi_nubus_io_request(NB_READ,nupi_buffer_pointer+nupi_xfer_pointer,0);
	nupi_cmd_prog++;
      }
      break;
    case 12: // SCSI PASS-THRU READ SETUP - STEP 2
      if(Nubus_Busy == 0){
	nupi_scsi_buffer_len = NUbus_Data;
	logmsgf("NUPI: SCSI Buffer Length = 0x%lX\n",nupi_scsi_buffer_len);
        nupi_xfer_pointer += 4;
	nupi_nubus_io_request(NB_READ,nupi_buffer_pointer+nupi_xfer_pointer,0);
	nupi_cmd_prog++;
      }
      break;
    case 13: // SCSI PASS-THRU READ SETUP - STEP 3
      if(Nubus_Busy == 0){
	nupi_scsi_status_ptr = NUbus_Data;
	logmsgf("NUPI: SCSI Status Pointer = 0x%lX\n",nupi_scsi_status_ptr);
        nupi_xfer_pointer += 4;
	nupi_nubus_io_request(NB_READ,nupi_buffer_pointer+nupi_xfer_pointer,0);
	nupi_cmd_prog++;
      }
      break;
    case 14: // SCSI PASS-THRU READ SETUP - STEP 4
      if(Nubus_Busy == 0){
	nupi_scsi_status_ptr_len = NUbus_Data;
	logmsgf("NUPI: SCSI NUPI Status Pointer Length = 0x%lX\n",nupi_scsi_status_ptr_len);
        nupi_xfer_pointer += 4;
	nupi_nubus_io_request(NB_READ,nupi_buffer_pointer+nupi_xfer_pointer,0);
	nupi_cmd_prog++;
      }
      break;
    case 15: // SCSI PASS-THRU READ SETUP - STEP 5
      if(Nubus_Busy == 0){
	nupi_scsi_cmd_blk_ptr = NUbus_Data;
	logmsgf("NUPI: SCSI Command Block Address = 0x%lX\n",nupi_scsi_cmd_blk_ptr);
        nupi_xfer_pointer += 4;
	nupi_nubus_io_request(NB_READ,nupi_buffer_pointer+nupi_xfer_pointer,0);
	nupi_cmd_prog++;
      }
      break;
    case 16: // SCSI PASS-THRU READ SETUP - STEP 6
      if(Nubus_Busy == 0){
	nupi_scsi_cmd_blk_len = NUbus_Data;
	logmsgf("NUPI: SCSI NUPI Command Block Length = 0x%lX\n",nupi_scsi_cmd_blk_len);
	nupi_nubus_io_request(NB_READ,nupi_scsi_cmd_blk_ptr,0);
        nupi_xfer_pointer = 0;
	nupi_cmd_prog++; 
      }
      break;
    case 17: // SCSI PASS_THRU READ COMMAND BLOCK
      if(Nubus_Busy == 0){
        NUPI_XFER_BUFFER[nupi_xfer_pointer>>2] = NUbus_Data;    // Save Word
        if(nupi_xfer_pointer < nupi_scsi_cmd_blk_len){
          nupi_xfer_pointer += 4;                                 // Point to next
          nupi_nubus_io_request(NB_READ,nupi_scsi_cmd_blk_ptr,0); // Get it
        }else{
          // Have all or more than we need
          nupi_cmd_prog++; // Execute next          
        }
      }
      break;
    case 18: // SCSI PASS_THRU READ EXECUTION
      {
        // SCSI Command is Word 0. Switch!
        // LUN is apparently always 0.
        switch(NUPI_XFER_BUFFER[0]&0xFF){
          
        case 0x12: // IDENTIFY
          {
            int device_lun = (NUPI_XFER_BUFFER[0]&0xE000)>>14;
            logmsgf("NUPI: SCSI IDENTIFY command - LUN %d - NOT RETURNING ANY DATA\n",device_lun);
            // Store OK status
            nupi_nubus_io_request(NB_WRITE,nupi_scsi_status_ptr,0);
          }
          break;

        case 0x1D: // SEND DIAGNOSTIC
          {
            int device_lun = (NUPI_XFER_BUFFER[0]&0xE000)>>14;
            logmsgf("NUPI: SCSI SEND DIAGNOSTIC command - LUN %d - NOT RETURNING ANY DATA\n",device_lun);
            // Store OK status
            nupi_nubus_io_request(NB_WRITE,nupi_scsi_status_ptr,0);
          }
          break;

        case 0x25: // READ CAPACITY
          {
            int device_lun = (NUPI_XFER_BUFFER[0]&0xE000)>>14;
            int pmi_bit    = (NUPI_XFER_BUFFER[2]&0x01);
            unsigned long lba = (NUPI_XFER_BUFFER[0]&0xFFFF0000); // MSB
            lba |=              (NUPI_XFER_BUFFER[1]&0x0000FFFF); // LSB
            logmsgf("NUPI: SCSI READ CAPACITY command - LUN %d, pmi %d, LBA %d\n",device_lun,pmi_bit,lba);
            // The disks are 112,446,208 bytes long.
            // The sectors are 256 bytes. Adaptec 5500 SCSI to ST506 bridge, with a Maxtor 1140 drive.
            // This means 439,243 sectors (logical blocks)
            if(pmi_bit == 0){ 
              logmsgf("NUPI - ERROR - PMI IS ZERO\n");
              cpu_die_rq=1;
              return;
            }else{
              memset(NUPI_SCSI_CMD_RESULT,0,1024); // CLEAR XFER RAM
              // Return size of selected sector              
              NUPI_SCSI_CMD_RESULT[0] = (lba&0xFF000000)>>24;
              NUPI_SCSI_CMD_RESULT[1] = (lba&0x00FF0000)>>16;
              NUPI_SCSI_CMD_RESULT[2] = (lba&0x0000FF00)>>8;
              NUPI_SCSI_CMD_RESULT[3] = (lba&0x000000FF);
              NUPI_SCSI_CMD_RESULT[4] = 0x00; // Sector size
              NUPI_SCSI_CMD_RESULT[5] = 0x00;
              NUPI_SCSI_CMD_RESULT[6] = 0x01; // 256 (0x100)
              NUPI_SCSI_CMD_RESULT[7] = 0x00;
              // Store 0 status (OK)
              nupi_nubus_io_request(NB_WRITE,nupi_scsi_status_ptr,0);
              nupi_scsi_result_index = 0;
              nupi_cmd_prog=19; // Store 
              return;
            }
          }
          break;
            
        default:
          logmsgf("NUPI: UNKNOWN SCSI COMMAND ON PASSTHRU READ - CMD WORD 0 = 0x%lX\n",NUPI_XFER_BUFFER[0]);
          return;
        }
        // If we're here, command completed OK and not returning data
        logmsgf("NUPI: SCSI PASSTHRU READ COMMAND COMPLETED\n");
        nupi_cmd_prog=9; // DONE
      }
      break;

    case 19: // WRITE RESULT OF SCSI COMMAND
      {
        if(nupi_scsi_result_index < nupi_scsi_buffer_len){
          if(Nubus_Busy == 0){
            // Issue Write
            nupi_nubus_io_request(NB_BYTE_WRITE,nupi_scsi_nubus_addr+nupi_scsi_result_index,
                                  (NUPI_SCSI_CMD_RESULT[nupi_scsi_result_index]<<((nupi_scsi_result_index&0x3)*8)));
            nupi_scsi_result_index++;
          }
        }else{
          logmsgf("NUPI: SCSI RESULT DATA STORED\n");
          nupi_cmd_prog=9;
        }
      }
      break;

    case 20: // READ INTO BUFFER BUSY-LOOP
      {
	if(nupi_xfer_pointer < nupi_transfer_count){
	  // Is memory busy?
	  if(Nubus_Busy == 0){
	    // Get the byte

            /*
            if(nupi_scatter){
	      logmsg("NUPI: SCATTER MODE DOESN'T WORK FOR READ-INTO-BUFFER-BUSY-LOOP YET!\n");
	      cpu_die_rq=1;
            }
            */

            if(nupi_scatter){
              // What're we doing?
              switch(nupi_scatter_phase){
              case 4: // RESET AND THEN
                nupi_scatter_index = 0;

              case 0: // READ IN ADDRESS
#ifdef TRACELOG
                logmsgf("Reading from 0x%lX + 0x%lX\n",nupi_buffer_pointer,nupi_scatter_block);
#endif
                nupi_nubus_io_request(NB_READ,nupi_buffer_pointer+nupi_scatter_block,0);
                nupi_scatter_phase = 1;
                return;
                break;

              case 1: // TAKE ADDRESS AND READ IN LENGTH
                nupi_scatter_address = NUbus_Data;
                nupi_scatter_block += 0x04; // Read next word
#ifdef TRACELOG                
                logmsgf("NUPI: SCATTER ADDRESS = 0x%lX\n",nupi_scatter_address);
#endif
                nupi_nubus_io_request(NB_READ,nupi_buffer_pointer+nupi_scatter_block,0);
                nupi_scatter_phase = 2;
                return;
                break;

              case 2: // TAKE LENGTH
                nupi_scatter_length = NUbus_Data;
                nupi_scatter_block += 0x04; // Read next word
#ifdef TRACELOG
                logmsgf("NUPI: SCATTER LENGTH = 0x%lX\n",nupi_scatter_length);
#endif
                nupi_scatter_phase = 3;
                return;
                break;

              case 3: // IN PROGRESS (Fall Through)
                break;

              default:
                logmsgf("NUPI: FATAL - UNKNOWN SCATTER PHASE %d IN READ-INTO-BUFFER-BUSY-LOOP\n",nupi_scatter_phase);
                nupi_cmd_prog=9; // DONE TRANSFERRING	  
                nupi_xfer_pointer = 0;
                cpu_die_rq=1;
                return;
              }              

              NUPI_XFER_BUFFER[(nupi_xfer_pointer>>2)] = NUbus_Data;
              nupi_xfer_pointer += 4;
              // Get next
              if(nupi_xfer_pointer < nupi_transfer_count){
                nupi_nubus_io_request(NB_READ,nupi_scatter_address+(nupi_scatter_index),0);
              }
              // Check completion
              if(nupi_scatter_index >= nupi_scatter_length){
#ifdef TRACELOG
                logmsgf("NUPI: SCATTER BLOCK COMPLETED (0x%X)\n",nupi_scatter_index);
#endif
                nupi_scatter_phase = 4;
                cpu_die_rq=1;
              }                
            }else{
              if(NUbus_Address != nupi_buffer_pointer+nupi_xfer_pointer){
                logmsgf("NuPI: READ INTO BUFFER - NOT OUR BUS! (0x%lX wanted, got 0x%lX @ 0x%lX)\n",nupi_buffer_pointer+nupi_xfer_pointer,NUbus_Request,NUbus_Address);
                cpu_die_rq=1;
              }
              NUPI_XFER_BUFFER[(nupi_xfer_pointer>>2)] = NUbus_Data;
              nupi_xfer_pointer += 4;
              // Get next
              if(nupi_xfer_pointer < nupi_transfer_count){
                nupi_nubus_io_request(NB_READ,nupi_buffer_pointer+nupi_xfer_pointer,0);
              }
            }
          }
	}else{
	  // FINISHED READING TO BUFFER
#ifdef TRACELOG
	  logmsgf("NUPI: Memory Buffering Completed, Executing Command\n");
	  logmsgf("NUPI: command %x, xfer count %d bytes\n", nupi_command, nupi_transfer_count);
#endif
	  nupi_cmd_prog=40+nupi_command; // DONE TRANSFERRING	  
	  nupi_xfer_pointer = 0;
	}
      }
      break;

    case 21: // SCSI PASS-THRU WRITE SETUP - STEP 1
      if(Nubus_Busy == 0){
	nupi_scsi_nubus_addr = NUbus_Data;
	logmsgf("NUPI: SCSI NUPI Address = 0x%lX\n",nupi_scsi_nubus_addr);
        nupi_xfer_pointer += 4;
	nupi_nubus_io_request(NB_READ,nupi_buffer_pointer+nupi_xfer_pointer,0);
	nupi_cmd_prog++;
      }
      break;
    case 22: // SCSI PASS-THRU WRITE SETUP - STEP 2
      if(Nubus_Busy == 0){
	nupi_scsi_buffer_len = NUbus_Data;
	logmsgf("NUPI: SCSI Buffer Length = 0x%lX\n",nupi_scsi_buffer_len);
        nupi_xfer_pointer += 4;
	nupi_nubus_io_request(NB_READ,nupi_buffer_pointer+nupi_xfer_pointer,0);
	nupi_cmd_prog++;
      }
      break;
    case 23: // SCSI PASS-THRU WRITE SETUP - STEP 3
      if(Nubus_Busy == 0){
	nupi_scsi_status_ptr = NUbus_Data;
	logmsgf("NUPI: SCSI Status Pointer = 0x%lX\n",nupi_scsi_status_ptr);
        nupi_xfer_pointer += 4;
	nupi_nubus_io_request(NB_READ,nupi_buffer_pointer+nupi_xfer_pointer,0);
	nupi_cmd_prog++;
      }
      break;
    case 24: // SCSI PASS-THRU WRITE SETUP - STEP 4
      if(Nubus_Busy == 0){
	nupi_scsi_status_ptr_len = NUbus_Data;
	logmsgf("NUPI: SCSI NUPI Status Pointer Length = 0x%lX\n",nupi_scsi_status_ptr_len);
        nupi_xfer_pointer += 4;
	nupi_nubus_io_request(NB_READ,nupi_buffer_pointer+nupi_xfer_pointer,0);
	nupi_cmd_prog++;
      }
      break;
    case 25: // SCSI PASS-THRU WRITE SETUP - STEP 5
      if(Nubus_Busy == 0){
	nupi_scsi_cmd_blk_ptr = NUbus_Data;
	logmsgf("NUPI: SCSI Command Block Address = 0x%lX\n",nupi_scsi_cmd_blk_ptr);
        nupi_xfer_pointer += 4;
	nupi_nubus_io_request(NB_READ,nupi_buffer_pointer+nupi_xfer_pointer,0);
	nupi_cmd_prog++;
      }
      break;
    case 26: // SCSI PASS-THRU WRITE SETUP - STEP 6
      if(Nubus_Busy == 0){
	nupi_scsi_cmd_blk_len = NUbus_Data;
	logmsgf("NUPI: SCSI NUPI Command Block Length = 0x%lX\n",nupi_scsi_cmd_blk_len);
	nupi_nubus_io_request(NB_READ,nupi_scsi_cmd_blk_ptr,0);
        nupi_xfer_pointer = 0;
	nupi_cmd_prog++; 
      }
      break;
    case 27: // SCSI PASS_THRU WRITE COMMAND BLOCK
      if(Nubus_Busy == 0){
        NUPI_XFER_BUFFER[nupi_xfer_pointer>>2] = NUbus_Data;    // Save Word
        if(nupi_xfer_pointer < nupi_scsi_cmd_blk_len){
          nupi_xfer_pointer += 4;                                 // Point to next
          nupi_nubus_io_request(NB_READ,nupi_scsi_cmd_blk_ptr,0); // Get it
        }else{
          // Have all or more than we need
          nupi_cmd_prog++; // Execute next          
        }
      }
      break;
    case 28: // SCSI PASS_THRU WRITE EXECUTION
      {
        // SCSI Command is Word 0. Switch!
        // LUN is apparently always 0.
        switch(NUPI_XFER_BUFFER[0]&0xFF){

        case 0x1D: // SEND DIAGNOSTIC
          {
            int device_lun = (NUPI_XFER_BUFFER[0]&0xE000)>>14;
            logmsgf("NUPI: SCSI SEND DIAGNOSTIC command - LUN %d - NOT RETURNING ANY DATA\n",device_lun);
            // Store OK status
            nupi_nubus_io_request(NB_WRITE,nupi_scsi_status_ptr,0);
          }
          break;
          
        default:
          logmsgf("NUPI: UNKNOWN SCSI COMMAND ON PASSTHRU WRITE - CMD WORD 0 = 0x%lX\n",NUPI_XFER_BUFFER[0]);
          return;
        }
        // If we're here, command completed OK and not returning data
        logmsgf("NUPI: SCSI PASSTHRU WRITE COMMAND COMPLETED\n");
        nupi_cmd_prog=9; // DONE
      }
      break;
    case 29: // WRITE RESULT OF SCSI COMMAND
      {
        if(nupi_scsi_result_index < nupi_scsi_buffer_len){
          if(Nubus_Busy == 0){
            // Issue Write
            nupi_nubus_io_request(NB_BYTE_WRITE,nupi_scsi_nubus_addr+nupi_scsi_result_index,
                                  (NUPI_SCSI_CMD_RESULT[nupi_scsi_result_index]<<((nupi_scsi_result_index&0x3)*8)));
            nupi_scsi_result_index++;
          }
        }else{
          logmsgf("NUPI: SCSI RESULT DATA STORED\n");
          nupi_cmd_prog=9;
        }
      }
      break;

    case 35: // NUPI TEST - STOP (Shouldn't ever get here!)
      nupi_go=0; nupi_cmd_prog=0;
      break;
    case 36: // NUPI TEST - START WRITE PHASE
      if(nupi_test_indx < 0x40){
	if(Nubus_Busy == 0){
	  nupi_nubus_io_request(NB_WRITE,nupi_test_addr+(4*nupi_test_indx),NUPI_ROM[0xFC0+nupi_test_indx]);
	  nupi_test_indx++;
	}
      }else{
	nupi_cmd_prog++;
	nupi_test_indx=0;
      }
      return;
      break;
    case 37: // NUPI TEST - START READ PHASE
      if(nupi_test_indx < 0x40){
	// Don't actually check the result, for now.
	if(Nubus_Busy == 0){
	  nupi_nubus_io_request(NB_READ,nupi_test_addr+(4*nupi_test_indx),0);
	  nupi_test_indx++;
	}
      }else{ 
	nupi_cmd_prog++;
	nupi_test_indx=0;
      }
      return;
      break;
    case 38: // NUPI TEST - LOOP CONTROL
      if(nupi_test_loop < 3){
	nupi_test_indx=0;
	nupi_cmd_prog=36; // LOOP
	nupi_test_loop++;
      }else{
	nupi_cmd_prog=0; nupi_go=0;
	NUPI_RAM[0xB] &= 0xF7; // Turn off test bit
	logmsg("NUPI: System Bus Test Completed\n");
      }      
      return;
      break;
      
      /* COMMANDS 40 AND ABOVE ARE RESERVED TO COMMANDS WHICH TAKE DATA FROM THE BUFFER READER AT 20! */
      /* AFTER READING IN BUFFER DATA IT JUMPS TO 40 + NUPI COMMAND VALUE */

    case 41: // CONTINUATION OF SETUP COMMAND
      {
	// Setting nupi status
	if(nupi_dst_bit){
	  int x=0;
	  if(x<nupi_transfer_count){
	    nupi_special_event_addr = NUPI_XFER_BUFFER[0];
	    logmsgf("NUPI: Special Event Address reset to 0x%lX\n",nupi_special_event_addr);
	    x += 4;
	  }else{
	    // ERROR!
	    logmsg("NUPI: SETUP command has no data?\n");
	    cpu_die_rq=1; nupi_cmd_prog=0; nupi_go=0;
	  }
	  if(x<nupi_transfer_count){
	    nupi_retry_count = NUPI_XFER_BUFFER[1]&0xFF;
	    nupi_dead_cmd_disable = (NUPI_XFER_BUFFER[1]>>28)&0x01;
	    nupi_periodic_poll_disable = (NUPI_XFER_BUFFER[1]>>29)&0x01;
	    nupi_retry_disable = (NUPI_XFER_BUFFER[1]>>30)&0x01;
	    nupi_ecc_disable = (NUPI_XFER_BUFFER[1]>>31)&0x01;
	    logmsgf("NUPI: Setup Word 2 = 0x%lX\n",NUPI_XFER_BUFFER[1]);
	  }
	  nupi_cmd_prog=9; // DONE
	  return;
	}
	logmsg("NUPI-Setup-2: Unknown destination\n");
	cpu_die_rq=1; nupi_cmd_prog=0; nupi_go=0;
	return;
      }
      break;

    case 59: // CONTINUATION OF WRITE DATA COMMAND
      {
	unsigned char diskio[4];

        // nupi_xfer_pointer has been reset for us. nupi_transfer_count is the count IN BYTES to be written.
        if(nupi_xfer_pointer < nupi_transfer_count){
          // Word at NUPI_XFER_BUFFER[(nupi_xfer_pointer>>2)] goes to disk in four bytes
          // Read into diskio bytes
          diskio[0] = (NUPI_XFER_BUFFER[(nupi_xfer_pointer>>2)]&0x000000FF);
          diskio[1] = (NUPI_XFER_BUFFER[(nupi_xfer_pointer>>2)]&0x0000FF00)>>8;
          diskio[2] = (NUPI_XFER_BUFFER[(nupi_xfer_pointer>>2)]&0x00FF0000)>>16;
          diskio[3] = (NUPI_XFER_BUFFER[(nupi_xfer_pointer>>2)]&0xFF000000)>>24;          
          // Write out to disk
          write(nupi_device[nupi_unit_pointer].fd,&diskio[0],1);
          write(nupi_device[nupi_unit_pointer].fd,&diskio[1],1);
          write(nupi_device[nupi_unit_pointer].fd,&diskio[2],1);
          write(nupi_device[nupi_unit_pointer].fd,&diskio[3],1);
          // Next!
          nupi_xfer_pointer += 4;
          return;
        }else{
          // Done
#ifdef TRACELOG
	  logmsgf("NUPI: Wrote %d words of data.\n",(nupi_xfer_pointer>>2));
#endif
          nupi_scatter_index=0;
          nupi_scatter_block=0;
          nupi_scatter_phase=0;
	  nupi_cmd_prog=9; // DONE TRANSFERRING	  
	  nupi_xfer_pointer = 0;
	  return;
        }
      }
      break;

    default:
      logmsgf("NUPI: BAD INTERNAL PC %ld\n",nupi_cmd_prog);
      nupi_go=0; cpu_die_rq=1;
    } 
  }
}


/*
 * Local Variables:
 * indent-tabs-mode:nil
 * c-basic-offset:2
 * End:
*/
