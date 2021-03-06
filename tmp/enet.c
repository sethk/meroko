/* Ethernet Interface - Implementation

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
#include <endian.h>

#ifndef USE_PTHREADS
# define USE_PTHREADS 1
#endif

#if USE_PTHREADS
#include <pthread.h>
#endif

#include "meroko.h"
#include "nubus.h"

#ifndef ENET_TAPIF
# define ENET_TAPIF 1
# ifndef ENET_TAP_PTHREADS
#  define ENET_TAP_PTHREADS USE_PTHREADS
# endif
#endif

#if ENET_TAPIF
# include "enet-tap.h"
#endif

unsigned char ENET_ROM[0x1000]; // 4K ROM
unsigned char ENET_RAM[0x8000]; // 32K BUFFER RAM

#define PROM_FNAME "proms/2236430_ENET"

// External stuff
extern int cpu_die_rq;
extern unsigned int ldb(unsigned long long value, int size, int position);

void ru_received_packet(u_char *pkt_rx_buf, int pkt_rx_size);
void describe_enet_pkt(u_char *pkt, int len);
#if USE_PTHREADS
pthread_mutex_t scb_status_lock;
pthread_mutex_t rfa_lock;
#define PLOCK(x) if (pthread_mutex_lock(x) != 0) logmsg("ENET: FAILED TO LOCK\n")
#define PUNLOCK(x) if (pthread_mutex_unlock(x) != 0) logmsg("ENET: FAILED TO UNLOCK\n")
#else
#define PLOCK(x) {}
#define PUNLOCK(x) {}
#endif

// Explorer Registers
unsigned long enet_conf_reg = 0x002C0000; // ALSO FLAG REGISTER (This value means 32K RAM + CRS + RTS) 
unsigned long enet_evnt_addr = 0;
unsigned int delay_counter = 0;
unsigned int channel_attn=0; // CHANNEL ATTENTION pulse

// Intel 82586 
// My MAC address
unsigned char enet_mac_addr[6];
// The size of the packet IO buffers
#define ENET_BFR_SIZE 2048
// Transmit and recieve packet buffers and their size / go triggers.
unsigned char pkt_rx_buf[ENET_BFR_SIZE];
unsigned int pkt_rx_size=0;
unsigned char pkt_tx_buf[ENET_BFR_SIZE];
unsigned int pkt_tx_size=0;
// Information from the CONFIGURE action
unsigned char enet_ext_lb=0;       // External Loopback selected
unsigned char enet_int_lb=0;       // External Loopback selected
unsigned char enet_fifo_limit=8;   // FIFO threshold
unsigned char enet_extn_sync=0;    // SRDY/ARDY: use external sync
unsigned char enet_preamble_len=2; // Preamble Length
unsigned char enet_a_t_loc=0;      // Address and Type Location
unsigned char enet_addr_len=6;     // Ethernet Address Length
unsigned char enet_sav_bf=0;       // SAV-BF: Save Bad Frames flag
unsigned char enet_intrfrm_len=96; // Interframe Length
unsigned char enet_bof_met=0;      // Use experimental backoff method
unsigned char enet_acr=0;          // Use accelerated contention resolution
unsigned char enet_linear_prio=0;  // Linear Priority
unsigned char enet_retry_num=15;   // Retry Number
unsigned int  enet_slot_time=512;  // S10-S0
unsigned char enet_cdt_src=0;      // Internal/External Carrier Detect
unsigned char enet_cdt_filter=0;   // Carrier Detect filter (delay)
unsigned char enet_cs_src=0;       // Internal/External Carrier Sense
unsigned char enet_cs_filter=0;    // Carrier Sense filter (delay)
unsigned char enet_padding=0;      // Padding
unsigned char enet_bt_stf=0;       // Bit Stuffing / End of Carrier
unsigned char enet_crc_16=0;       // CRC16/CRC32 select
unsigned char enet_no_crc=0;       // NO CRC select
unsigned char enet_tx_ncs=0;       // Transmit without Carrier Sense
unsigned char enet_man_nrz=0;      // Manchester/NRZ Data Format (from serial interface)
unsigned char enet_bc_dis=0;       // Disable incoming broadcast packets
unsigned char enet_pr_m=0;         // Promiscuous Mode
unsigned char enet_min_frm_len=64; // Minimum Frame Length

// Address of Intermediate SCP
unsigned long iscp_addr=0;
// Base address and offset to SCB
unsigned long scb_base_addr=0;
unsigned long scb_offset_addr=0;
// Final address to SCB
volatile unsigned long scb_addr=0;
// Address of CBL
unsigned long cbl_addr=0;
// Address of RFA
volatile unsigned long rfa_addr=0;
// SCB Command Word
unsigned long scb_command=0;

// From command word
int ru_cmd,cu_cmd; // RU and CU commands	
int ru_run,cu_run; // RU and CU run flags
int cu_cdn;        // CU Command Done flag
volatile int ru_rx_init=0;  // RX initialized

#if ENET_TAPIF
int netsock = 0;
unsigned long crc32(unsigned char const *p, size_t len);
#endif

// Initialization
void enet_init()
{
  FILE *romfile;

#if USE_PTHREADS
  if (pthread_mutex_init(&scb_status_lock, NULL) != 0)
    perror("ENET: pthread_mutex_init(scb_status_lock)");
  if (pthread_mutex_init(&rfa_lock, NULL) != 0)
    perror("ENET: pthread_mutex_init(rfa_lock)");
#endif

#if ENET_TAPIF
  netsock = get_packet_socket();
#endif
  // Load ROM image
  romfile = fopen(PROM_FNAME,"r");
  if(romfile == NULL){
    perror("enet-fopen");
    exit(-1);
  }
  fread(&ENET_ROM, sizeof(ENET_ROM), 1, romfile);
  fclose(romfile);
  logmsgf("ENET: Read Ethernet PROM: address %02x:%02x:%02x:%02x:%02x:%02x\n",
	  ENET_ROM[0xFB8],ENET_ROM[0xFB8+1],ENET_ROM[0xFB8+2],
	  ENET_ROM[0xFB8+3],ENET_ROM[0xFB8+4],ENET_ROM[0xFB8+5]);
#if ENET_TAPIF
  /* patch in real MAC address */
  if (memcmp(my_ether_addr, "\0\0\0\0\0\0", 6) != 0) {
    // #### 16-bit CRC, but which polynomial?
    //unsigned long crc = crc32(my_ether_addr, 6);
    memcpy(&ENET_ROM[0xFB8], my_ether_addr, 6);
    logmsgf("ENET: Patched ethernet address: %02x:%02x:%02x:%02x:%02x:%02x\n",
	    ENET_ROM[0xFB8],ENET_ROM[0xFB8+1],ENET_ROM[0xFB8+2],
	    ENET_ROM[0xFB8+3],ENET_ROM[0xFB8+4],ENET_ROM[0xFB8+5]);
  }
  /* but should fix checksum too in order to pass self-test */
# if ENET_TAP_PTHREADS
  if (!create_receiver_thread(netsock, pkt_rx_buf, sizeof(pkt_rx_buf)))
    logmsg("ENET: **** Failed to create receiver thread!\n");
# endif
#endif
}

// Reset ethernet configuration
void enet_conf_reset()
{
  enet_ext_lb=0;       // External Loopback selected
  enet_int_lb=0;       // External Loopback selected
  enet_fifo_limit=8;   // FIFO threshold
  enet_extn_sync=0;    // SRDY/ARDY: use external sync
  enet_preamble_len=2; // Preamble Length
  enet_a_t_loc=0;      // Address and Type Location
  enet_addr_len=6;     // Ethernet Address Length
  enet_sav_bf=0;       // SAV-BF: Save Bad Frames flag
  enet_intrfrm_len=96; // Interframe Length
  enet_bof_met=0;      // Use experimental backoff method
  enet_acr=0;          // Use accelerated contention resolution
  enet_linear_prio=0;  // Linear Priority
  enet_retry_num=15;   // Retry Number
  enet_slot_time=512;  // S10-S0
  enet_cdt_src=0;      // Internal/External Carrier Detect
  enet_cdt_filter=0;   // Carrier Detect filter (delay)
  enet_cs_src=0;       // Internal/External Carrier Sense
  enet_cs_filter=0;    // Carrier Sense filter (delay)
  enet_padding=0;      // Padding
  enet_bt_stf=0;       // Bit Stuffing / End of Carrier
  enet_crc_16=0;       // CRC16/CRC32 select
  enet_no_crc=0;       // NO CRC select
  enet_tx_ncs=0;       // Transmit without Carrier Sense
  enet_man_nrz=0;      // Manchester/NRZ Data Format (from serial interface)
  enet_bc_dis=0;       // Disable incoming broadcast packets
  enet_pr_m=0;         // Promiscuous Mode
  enet_min_frm_len=64; // Minimum Frame Length
}

// Nubus IO handler
void enet_nubus_io()
{
  unsigned long NUbus_Addr = ldb(NUbus_Address,24,0);  

  // Buffer RAM
  if(NUbus_Request == VM_READ && NUbus_Addr < 0x008000){
    unsigned int RAMaddr = (NUbus_Addr&0x7FFF);

    switch(NUbus_Addr&0x03){
    case 0: // WORD IO
      NUbus_Data  = ENET_RAM[RAMaddr+3]; NUbus_Data <<= 8;
      NUbus_Data |= ENET_RAM[RAMaddr+2]; NUbus_Data <<= 8;
      NUbus_Data |= ENET_RAM[RAMaddr+1]; NUbus_Data <<= 8;
      NUbus_Data |= ENET_RAM[RAMaddr+0];
      break;

    case 1: // LOW HALFWORD READ
      NUbus_Data =  ENET_RAM[RAMaddr]; NUbus_Data <<= 8;
      NUbus_Data |= ENET_RAM[RAMaddr-1];
      break;

    case 3: // HIGH HALFWORD READ
      NUbus_Data =  ENET_RAM[RAMaddr]; NUbus_Data <<= 8;
      NUbus_Data |= ENET_RAM[RAMaddr-1];
      NUbus_Data <<= 16;
      break;

    default:
      logmsg("BAD ENET WORD-READ TYPE\n");
      cpu_die_rq=1;
    }
#if 0
    if ((scb_addr != 0) && (RAMaddr >= scb_addr) && (RAMaddr < (scb_addr + 4)))  
      logmsgf("ENET: Reading SCB status: word addr 0x%x, data 0x%x, SCB @ 0x%x\n",
	      RAMaddr, NUbus_Data, scb_addr);
#endif

    NUbus_acknowledge=1;
    return;
  }

  if(NUbus_Request == VM_BYTE_READ && NUbus_Addr < 0x008000){
    unsigned int RAMaddr = (NUbus_Addr&0x7FFF);
    NUbus_Data = ENET_RAM[RAMaddr]; NUbus_Data &= 0xFF;
    NUbus_Data <<= (8*(NUbus_Addr&0x03));
#if 0
    if ((scb_addr != 0) && (RAMaddr <= scb_addr+1))
      logmsgf("ENET: Reading SCB status: byte addr 0x%x, data 0x%x, SCB @ 0x%x\n",
	      RAMaddr, NUbus_Data, scb_addr);
    if ((scb_addr != 0) && (RAMaddr > scb_addr+1) && (RAMaddr <= scb_addr+3))
      logmsgf("ENET: Reading SCB command: byte addr 0x%x, data 0x%x, SCB @ 0x%x\n",
	      RAMaddr, NUbus_Data, scb_addr);
#endif
    NUbus_acknowledge=1;
    return;
  }

  // Exclude ROM here
  if(NUbus_Request == VM_READ && 
     (NUbus_Addr >= 0xFFE000 || (NUbus_Addr >= 0xE000 && NUbus_Addr < 0x10000))){
    unsigned int ROMaddr = ((NUbus_Addr&0x3FFF)>>2);

    /* Word 0 = byte 0
       Word 1 = byte 4
       Word 2 = byte 8
    */
    // Construct word
    NUbus_Data = ENET_ROM[ROMaddr]; NUbus_Data &= 0xFF;

    if(NUbus_Addr&0x03){
      logmsgf("ENET: Word-Read %lX for odd address %lX (ROM address %X)\n",
              NUbus_Data,NUbus_Addr,ROMaddr);
      cpu_die_rq=1;
    }

    //    logmsgf("ROM addr 0x%lX (0x%lX) read for data 0x%lX\n",ROMaddr,NUbus_Address,ENET_ROM[ROMaddr]);

    NUbus_acknowledge=1;
    return;
  }   
  
  if(NUbus_Request == VM_BYTE_READ && 
     (NUbus_Addr >= 0xFFE000 || (NUbus_Addr >= 0xE000 && NUbus_Addr < 0x10000))){         
    unsigned int ROMaddr = ((NUbus_Addr&0x3FFF)>>2);

    if((NUbus_Addr&0x3) != 0){ logmsg("ENET: ODD ROM ADDR\n"); }
    if(ROMaddr > 0xFFF){ logmsg("ENET: TOOBIG ROM ADDR\n"); }

    NUbus_Data = ENET_ROM[ROMaddr];
    //    logmsgf("ENET: Byte-Read %lX for address %lX (ROM address %X)\n",NUbus_Data,NUbus_Addr,ROMaddr);
    // cpu_die_rq=1;

    //    logmsgf("ROM addr 0x%lX (0x%lX) read for data 0x%lX\n",ROMaddr,NUbus_Address,ENET_ROM[ROMaddr]);

    NUbus_acknowledge=1;
    return;
  }  

  //  logmsgf("ENET: IO-OP %d: 0x%lX @ 0x%lX\n",NUbus_Request,NUbus_Data,NUbus_Addr);

  if(NUbus_Request == VM_BYTE_WRITE && NUbus_Addr < 0x008000){
    unsigned int RAMaddr = (NUbus_Addr&0x7FFF);
    if ((scb_addr != 0) && (RAMaddr >= scb_addr) && (RAMaddr < (scb_addr + 2)))  {
      logmsgf("ENET: Writing NuBus byte in SCB status (locking)\n");
      PLOCK(&scb_status_lock);
      ENET_RAM[RAMaddr] = ((NUbus_Data>>(8*(NUbus_Addr&0x03)))&0xFF);
      PUNLOCK(&scb_status_lock);
    } else
      ENET_RAM[RAMaddr] = ((NUbus_Data>>(8*(NUbus_Addr&0x03)))&0xFF);
    NUbus_acknowledge=1;
    return;
  }

  if(NUbus_Request == VM_WRITE && NUbus_Addr < 0x008000){
    unsigned int RAMaddr = (NUbus_Addr&0x7FFF);
    unsigned long RAMdata = NUbus_Data;

    //    logmsgf("ENET: RAM WORD WRITE: 0x%lX @ 0x%lX\n",NUbus_Data,RAMaddr);

    if ((scb_addr != 0) && (RAMaddr >= scb_addr) && (RAMaddr < (scb_addr + 2)))
      logmsgf("ENET: Writing NuBus word in SCB status: 0x%x @ 0x%x (SCB @ 0x%x)\n",
	      RAMdata, RAMaddr, scb_addr);

    switch(NUbus_Addr&0x03){
    case 0: // WORD IO
      ENET_RAM[RAMaddr+0] = RAMdata&0xFF; 
      RAMdata = RAMdata >> 8;
      ENET_RAM[RAMaddr+1] = RAMdata&0xFF; 
      RAMdata = RAMdata >> 8;
      ENET_RAM[RAMaddr+2] = RAMdata&0xFF; 
      RAMdata = RAMdata >> 8;
      ENET_RAM[RAMaddr+3] = RAMdata&0xFF;
      break;

    case 3: // High Halfword Write
      RAMdata >>= 16;
    case 1: // Low Halfword Write
      ENET_RAM[RAMaddr-1] = RAMdata&0xFF;
      RAMdata >>= 8;
      ENET_RAM[RAMaddr] = RAMdata&0xFF;
      break;

    case 2: // BLOCK TRANSFER
    default:
      logmsgf("BAD ENET WORD-WRITE TYPE %ld\n",NUbus_Addr&0x03);
      cpu_die_rq=1;
    }
    NUbus_acknowledge=1;
    return;
  }

  // Registers
  switch(NUbus_Addr){
  case 0x8000: // CHANNEL ATTENTION (GO COMMAND)
  case 0x8001:
    switch(NUbus_Request){
    case VM_WRITE:
    case VM_BYTE_WRITE:
      logmsgf("ENET: CHANNEL ATTENTION RECIEVED\n");
      channel_attn=1;
      NUbus_acknowledge=1;
      //      cpu_die_rq=1;
      return;
    }
    break;

  case 0xA000: // EVENT ADDRESS REGISTER
    switch(NUbus_Request){
    case VM_WRITE:
      enet_evnt_addr = NUbus_Data;
      logmsgf("ENET: Event Address = 0x%lX\n",enet_evnt_addr);
      NUbus_acknowledge=1;
      return;      
      break;
    }
    break;

  case 0xFFC000: // MIRROR OF
  case 0x00C000: // CONFIGURATION REGISTER (AND FLAG REGISTER)
    switch(NUbus_Request){
    case VM_BYTE_READ:
      NUbus_Data = enet_conf_reg&0xFF;      
      NUbus_acknowledge=1;
      return;
      break;
    case VM_READ:
      NUbus_Data = enet_conf_reg;      
      NUbus_acknowledge=1;
      return;
      break;
    case VM_BYTE_WRITE:
      /* WRITE FORMAT:
	 0x001 = RESET COMMAND
	 0x002 = BUS MASTER ENABLE
	 0x004 = FAULT LED
	 0x100 = SERIAL LOOPBACK COMMAND
      */
      if(NUbus_Data&0x01){
	logmsgf("ENET: RESET\n");
	// RESET
	// enet_evnt_addr=0; Don't do this?
	// Stop CU and RU
	cu_run=0;
	ru_run=0;
	enet_conf_reset();
      }
      if(NUbus_Data&0x02){
	logmsgf("ENET: EVENT GENERATION ENABLED\n");
      }
      if(NUbus_Data&0x100){
	logmsgf("ENET: SERIAL LOOPBACK COMMANDED\n");
	cpu_die_rq=1;
      }
      enet_conf_reg &= 0xFFFFFF00;
      enet_conf_reg |= (NUbus_Data&0xFE);
      NUbus_acknowledge=1;
      return;
      break;

    }
    break;

  case 0xC001: // UPPER BYTE / LOWER HALF, CONFIGURATION REGISTER (AND FLAG REGISTER)
    switch(NUbus_Request){
    case VM_BYTE_WRITE:
      if(NUbus_Data&0x1){
	logmsgf("ENET: SERIAL LOOPBACK COMMANDED\n");
	cpu_die_rq=1;
      }
      enet_conf_reg &= 0xFFFF00FF;
      enet_conf_reg |= (NUbus_Data&0xFF)<<8;
      NUbus_acknowledge=1;
      return;
      break;
    case VM_READ:
      NUbus_Data = enet_conf_reg;      
      NUbus_acknowledge=1;
      return;
      break;
    case VM_WRITE:
      /* WRITE FORMAT:
	 0x001 = RESET COMMAND
	 0x002 = BUS MASTER ENABLE
	 0x003 = FAULT LED
	 0x100 = SERIAL LOOPBACK COMMAND
      */
      if(NUbus_Data&0x01){
	logmsgf("ENET: RESET\n");
	// RESET
	// Stop CU and RU
	cu_run=0;
	ru_run=0;
	ru_rx_init = 0;
	enet_conf_reset();
      }
      if(NUbus_Data&0x02){
	logmsgf("ENET: EVENT GENERATION ENABLED\n");
      }
      if(NUbus_Data&0x100){
	logmsgf("ENET: SERIAL LOOPBACK COMMANDED\n");
      }
      enet_conf_reg &= 0xFFFF0000;
      enet_conf_reg |= (NUbus_Data&0xFFFE);
      NUbus_acknowledge=1;
      return;
      break;
    }
    break;

  case 0xC003: // HIGH HALFWORD READ, CONFIGURATION REGISTER (REALLY FLAG REGISTER)
    switch(NUbus_Request){     
    case VM_READ:
      /* BITS FOR FLAG REGISTER (READ ONLY, WRITES IGNORED):
	 0x00010000 = BUS ERROR ON EVENT POST
	 0x00020000 = HOLD LED ON (MEMORY HANDSHAKE BUSY)
	 0x00040000 = RTS LED ON (READY TO SEND)
	 0x00080000 = CRS LED ON (CARRIER SENSED)
	 0x00100000 = CDT LED ON (COLLISION DETECTED)
	 0x00200000 = MEM SIZE (1 = 32KB, 0 = 8KB, 32KB is normal);
      */
      NUbus_Data = enet_conf_reg&0xFFFF0000;      
      NUbus_acknowledge=1;
      return;
      break;
    }
    break;
  }


  logmsgf("ENET: Unknown NUbus IO: Op %ld for address %lX\n",
          NUbus_Request,NUbus_Addr);
  cpu_die_rq=1;
}

#if 0
void
show_dump_enet_status(char *hdr)
{
  logmsgf("ENET: %s:\n", hdr);
  logmsgf(" FIFO: %d\tByte count: %d\n",
	  enet_fifo_limit, 12);
  logmsgf(" EXT LB: %d, INT LB: %d, Preamb len: %d, A,T loc: %d\n",
	  enet_ext_lb, enet_int_lb, enet_preamble_len, enet_a_t_loc);
  logmsgf(" Addr len: %d, SavBF: %d, Srdy/Ardy: %d\n",
	  enet_addr_len, enet_sav_bf, enet_extn_sync);
  logmsgf(" Interfr spacing: %d, BOF/MET: %d, ACR: %d, Lin.prio: %d\n",
	  enet_intrfrm_len, enet_bof_met, enet_acr, enet_linear_prio);
  logmsgf(" Retry nr: %d, Slot time: %d\n",
	  enet_retry_num, enet_slot_time);
  logmsgf(" CDT src: %d, CDT fltr: %d, CS src: %d, CS fltr: %d\n",
	  enet_cdt_src, enet_cdt_filter, enet_cs_src, enet_cs_filter);
  logmsgf(" BitStf: %d, CRC16: %d, NoCRC: %d, TX/NCS: %d, MAN/NRZ: %d, BC/DIS %d, PR/M: %d\n",
	  enet_bt_stf, enet_crc_16, enet_no_crc, enet_tx_ncs, enet_man_nrz,
	  enet_bc_dis, enet_pr_m);
}
#endif

// CLOCK PULSE
/* inline */
void enet_clock_pulse()
{
  if (channel_attn) {
    // CHANNEL ATTENTION - The system configuration data has changed
    // First, read the System Configuration Pointer from bytes 0x7FF4-0x7FFF of buffer RAM 
    // The Intermediate SCP Address is at SCP+6,7
    iscp_addr = (ENET_RAM[0x7FFD] << 8) | ENET_RAM[0x7FFC];

    logmsgf("ENET: ISCP Address 0x%lX\n",iscp_addr);
    // Now read the System Configuration Block address from ISCB
    scb_offset_addr = (ENET_RAM[iscp_addr+3] << 8) | ENET_RAM[iscp_addr+2];
    scb_base_addr = (ENET_RAM[iscp_addr+5] << 8) | ENET_RAM[iscp_addr+4];

    logmsgf("ENET: SCB Base Address 0x%lX, Offset 0x%lX\n",scb_base_addr,scb_offset_addr);
    // Now read SCB
    PLOCK(&scb_status_lock);
    scb_addr = scb_base_addr + scb_offset_addr;
    PUNLOCK(&scb_status_lock);

    logmsgf("ENET: SCB status: 0x%X\n",
	    (ENET_RAM[scb_addr+1] << 8) | ENET_RAM[scb_addr+0]);

    scb_command = (ENET_RAM[scb_addr+3] << 8) | ENET_RAM[scb_addr+2];

    logmsgf("ENET: SCB Command = 0x%lX\n",scb_command);

    // Parse out command
    ru_cmd = (scb_command&0x70)>>4;
    cu_cmd = (scb_command&0x700)>>8;
    
    if(scb_command&0x80){
      logmsgf("ENET: SCB RESET COMMAND\n");       
      enet_conf_reset();
      cpu_die_rq=1;		/* #### Why? */
    }
    if (scb_command & 0xf000) { /* Any bits? */
      if (scb_command & 0x8000)
	logmsg("ENET: Ack CU command completed\n");
      if (scb_command & 0x4000)
	logmsg("ENET: Ack RU frame received\n");
      if (scb_command & 0x2000)
	logmsg("ENET: Ack Command Unit not active\n");
      if (scb_command & 0x1000)
	logmsg("ENET: Ack Receive Unit not active\n");
      PLOCK(&scb_status_lock);
      ENET_RAM[scb_addr+1] &= ~0xf0; /* clear all bits */
      PUNLOCK(&scb_status_lock);
    }
    /* clear command word */
    ENET_RAM[scb_addr+2] = ENET_RAM[scb_addr+3] = 00;

    logmsgf("ENET: RU CMD = %X, CU CMD = %X\n",ru_cmd,cu_cmd);

    // Read address of CU command block
    cbl_addr = (ENET_RAM[scb_addr+5] << 8) | ENET_RAM[scb_addr+4];
    logmsgf("ENET: CBL @ 0x%lX\n",cbl_addr);
    cu_cdn = 0; // Not Done
    // Read address of RFA
    PLOCK(&rfa_lock);
    rfa_addr = (ENET_RAM[scb_addr+7] << 8) | ENET_RAM[scb_addr+6];
    PUNLOCK(&rfa_lock);
    //rfa_addr = (ENET_RAM[scb_addr+7] << 8) | ENET_RAM[scb_addr+6];
    logmsgf("ENET: RFA @ 0x%lX\n", rfa_addr);
    // Allow CU and RU to run
    ru_run=1; cu_run=1;
    // and continue on
    // Clear iSCP BUSY flag
    ENET_RAM[iscp_addr]=0; ENET_RAM[iscp_addr+1]=0;
    channel_attn=0; 
    delay_counter = 0;
    ru_rx_init = 1;
  }  

  if(cu_run != 0){
    // CU EXECUTION
    //    ENET_RAM[scb_addr+1] &= 0xD0; // Turn off CNA
    // ENET_RAM[scb_addr+1] |= 0x02; // CU Active
    // Go!
    switch(cu_cmd){
    case 0: // NOP
      logmsgf("ENET: CU NOP\n");
      cu_run=0;
      break;
    
    case 1: // Run Command List
      PLOCK(&scb_status_lock);
      ENET_RAM[scb_addr+1] &= 0xD0; // Turn off CNA and CUS
      ENET_RAM[scb_addr+1] |= 0x02; // CU Active
      PUNLOCK(&scb_status_lock);
      logmsgf("ENET: CU Start command list\n");
      // CU command processing loop
      //      logmsgf("ENET: SCB CBLADR = 0x%lX\n",cbl_addr);	      
      ENET_RAM[cbl_addr+1] &= 0x3F; // Turn off flags
      ENET_RAM[cbl_addr+1] |= 0x40; // Turn on BUSY
      // Run!
      switch((ENET_RAM[cbl_addr+2]&0x7)){

      case 0: // NOP
	//logmsgf("ENET: CU Command NOP\n");
	ENET_RAM[cbl_addr+1] &= 0xC0; // Off status 
	ENET_RAM[cbl_addr+1] |= 0x20; // On COMPLETED OK
	cu_cdn=1;
	break;

      case 1: // IA-SETUP
	// This sets the MAC address to use.
	// These are in TRANSMISSION ORDER.
	logmsgf("ENET: IA-Setup: MAC Address %X:%X:%X:%X:%X:%X\n",
		ENET_RAM[cbl_addr+6],ENET_RAM[cbl_addr+7],
		ENET_RAM[cbl_addr+8],ENET_RAM[cbl_addr+9],
		ENET_RAM[cbl_addr+10],ENET_RAM[cbl_addr+11]);
	// Store supplied MAC address
	enet_mac_addr[0]=ENET_RAM[cbl_addr+6];  enet_mac_addr[1]=ENET_RAM[cbl_addr+7];
	enet_mac_addr[2]=ENET_RAM[cbl_addr+8];  enet_mac_addr[3]=ENET_RAM[cbl_addr+9];
	enet_mac_addr[4]=ENET_RAM[cbl_addr+10]; enet_mac_addr[5]=ENET_RAM[cbl_addr+11];
	// Return OK
	ENET_RAM[cbl_addr+1] &= 0xC0; // Off status 
	ENET_RAM[cbl_addr+1] |= 0x20; // On COMPLETED OK
	cu_cdn=1;
	break;

      case 2: // CONFIGURE
	// Provides a big list of parameters we should know
	ENET_RAM[cbl_addr+1] &= 0xC0; // Off status 
	ENET_RAM[cbl_addr+1] |= 0x20; // On COMPLETED OK
	logmsgf("ENET: CU Reconfiguration request @ 0x%lX\n",cbl_addr);
	// Reconfigure
	if((ENET_RAM[cbl_addr+6]&0xF) != 12){
	  logmsgf("ENET: CONFIGURATION REQUEST SIZE %d UNSUPPORTED\n",ENET_RAM[cbl_addr+6]&0xF);
	  cpu_die_rq=1;
	}
	enet_fifo_limit=     (ENET_RAM[cbl_addr+7] &0xFF);         // FIFO threshold
	enet_extn_sync=      (ENET_RAM[cbl_addr+8] >> 5) & 0x1;    // SRDY/ARDY: use external sync
	enet_sav_bf=	     (ENET_RAM[cbl_addr+8] >> 6) & 0x1;    // SAV-BF: Save Bad Frames flag
	enet_addr_len=       (ENET_RAM[cbl_addr+8] >> 7) & 0x1;    // Ethernet Address Length
	enet_addr_len |=     (ENET_RAM[cbl_addr+9] & 0x3)<<1;       // Other bits
	//enet_addr_len += 3;                                           // Maybe do this?
	enet_a_t_loc=        (ENET_RAM[cbl_addr+9] >> 1) & 0x3;    // Address and Type Location
	enet_preamble_len=   (ENET_RAM[cbl_addr+9] >> 4) & 0x3;    // Preamble Length
	enet_int_lb=         (ENET_RAM[cbl_addr+9] >> 6) & 0x1;    // External Loopback selected
	enet_ext_lb=         (ENET_RAM[cbl_addr+9] >> 7) & 0x1;    // External Loopback selected
	enet_linear_prio=    (ENET_RAM[cbl_addr+10]&0x7);          // Linear Priority
	enet_acr=            (ENET_RAM[cbl_addr+10]>> 4) & 0x7;    // Use accelerated contention resolution
	enet_bof_met=        (ENET_RAM[cbl_addr+10]>> 7) & 0x1;    // Use experimental backoff method
	enet_intrfrm_len=    ENET_RAM[cbl_addr+11];                // Interframe Length
	enet_slot_time=      ENET_RAM[cbl_addr+12];                // S10-S0     
	enet_slot_time |=    (ENET_RAM[cbl_addr+13]&0x7)<<8;          // Other bits
	enet_retry_num=      (ENET_RAM[cbl_addr+13]>> 4) & 0xF;    // Retry Number
	enet_pr_m=           (ENET_RAM[cbl_addr+14] & 0x1);        // Promiscuous Mode
	enet_bc_dis=         (ENET_RAM[cbl_addr+14]>> 1) & 0x1;    // Disable incoming broadcast packets
	enet_man_nrz=        (ENET_RAM[cbl_addr+14]>> 2) & 0x1;    // Manchester/NRZ Data Format (from serial interface)
	enet_tx_ncs=         (ENET_RAM[cbl_addr+14]>> 3) & 0x1;    // Transmit without Carrier Sense
	enet_no_crc=         (ENET_RAM[cbl_addr+14]>> 4) & 0x1;    // NO CRC select
	enet_crc_16=         (ENET_RAM[cbl_addr+14]>> 5) & 0x1;    // CRC16/CRC32 select
	enet_bt_stf=         (ENET_RAM[cbl_addr+14]>> 6) & 0x1;    // Bit Stuffing / End of Carrier
	enet_padding=        (ENET_RAM[cbl_addr+14]>> 7) & 0x1;    // Padding
	enet_cs_filter=      (ENET_RAM[cbl_addr+15] & 0x7);        // Carrier Sense filter (delay)
	enet_cs_src=         (ENET_RAM[cbl_addr+15]>> 3) & 0x1;    // Internal/External Carrier Sense
	enet_cdt_filter=     (ENET_RAM[cbl_addr+15]>> 4) & 0x7;    // Carrier Detect filter (delay)
	enet_cdt_src=        (ENET_RAM[cbl_addr+15]>> 7) & 0x1;    // Internal/External Carrier Detect
	enet_min_frm_len=    ENET_RAM[cbl_addr+16];                // Minimum Frame Length
		
	if(enet_ext_lb != 0){
	  logmsgf("ENET: External loopback requested\n");
	  //	  cpu_die_rq=1;
	}
	if(enet_int_lb != 0){
	  logmsgf("ENET: Internal loopback requested\n");
	  //	  cpu_die_rq=1;
	}
#if 0
	show_dump_enet_status("configuration setup");
#endif
	cu_cdn=1;
	break;

      case 3:
	logmsg("ENET: Multicast Setup command Not Implemented Yet!\n");
	ENET_RAM[cbl_addr+1] &= 0xC0; // Off status 
	ENET_RAM[cbl_addr+1] |= 0x20; // Set OK anyway
	cu_cdn = 1;
	break;

      case 4: // TRANSMIT
	{
	  // Miscellaneous flags and addresses and stuff
	  unsigned long tbd_addr=0,data_addr=0,tx_done=0,bf_size=0;
	  // Transmit a packet on the network.
	  // Get address to TBD
	  tbd_addr = ENET_RAM[cbl_addr+7];
	  tbd_addr <<= 8;
	  tbd_addr |= ENET_RAM[cbl_addr+6];
	  logmsgf("ENET: TX: TBD @ 0x%lX\n",tbd_addr);
	  logmsgf("ENET: TX: Destination MAC Address %X:%X:%X:%X:%X:%X, Size/Type %02X,%02X\n",
		  ENET_RAM[cbl_addr+8],ENET_RAM[cbl_addr+9],
		  ENET_RAM[cbl_addr+10],ENET_RAM[cbl_addr+11],
		  ENET_RAM[cbl_addr+12],ENET_RAM[cbl_addr+13],
		  ENET_RAM[cbl_addr+14],ENET_RAM[cbl_addr+15]);
	  // Construct header
	  // Destination Address
	  memcpy(pkt_tx_buf, &ENET_RAM[cbl_addr+8], 6);
	  // Source Address
	  memcpy(pkt_tx_buf+6, enet_mac_addr, 6);
	  // Type Code
	  pkt_tx_buf[12] = ENET_RAM[cbl_addr+14];
	  pkt_tx_buf[13] = ENET_RAM[cbl_addr+15];
	  pkt_tx_size=14; // Add Header Size
	  // Read TBD
	  while(!tx_done) {
	    // Get buffer size
	    bf_size = (ENET_RAM[tbd_addr]&0x3F);
	    logmsgf("ENET: %d bytes in this TBD, plus %d waiting for transmit.\n",bf_size,pkt_tx_size);
	    // Get data address
	    data_addr = ENET_RAM[tbd_addr+5];
	    data_addr <<= 8;
	    data_addr |= ENET_RAM[tbd_addr+4];
	    logmsgf("ENET: Data is at 0x%lX\n",data_addr);
	    // Copy data to buffer
	    memcpy(pkt_tx_buf+pkt_tx_size,ENET_RAM+data_addr,bf_size);
	    // Add to TX total
	    pkt_tx_size += bf_size;
	    // Continue
	    if((ENET_RAM[tbd_addr+1]&0x80)!=0){
	      logmsgf("ENET: This is the last buffer in the chain.\n");
	      tx_done=1;
	    }else{
	      tbd_addr = ENET_RAM[tbd_addr+3];
	      tbd_addr <<= 8;
	      tbd_addr |= ENET_RAM[tbd_addr+2];
	      logmsgf("ENET: Next TBD is at 0x%X\n",tbd_addr);		      
	    }
	  }
	  // Enforce minimum packet size
	  if(pkt_tx_size < enet_min_frm_len){
	    logmsgf("ENET: Packet padded to %d bytes\n",enet_min_frm_len);
	    pkt_tx_size = enet_min_frm_len;
	  }
	  //describe_enet_pkt(pkt_tx_buf, pkt_tx_size);
	  // Buffer ready, issue transmit
	  if(enet_int_lb != 0 || enet_ext_lb != 0 || (enet_conf_reg&0x100) != 0){
	    // Loopback?
	    logmsgf("ENET: Loopback TX\n");
#if 1
	    ru_received_packet(pkt_tx_buf, pkt_tx_size);
#else
	    if(pkt_rx_size == 0){
	      memcpy(pkt_rx_buf,pkt_tx_buf,pkt_tx_size);
	      pkt_rx_size = pkt_tx_size;
	    }else{
	      logmsg("ENET: RX COLLISION WHILE REPEATING PACKET FOR LOOPBACK\n");
	      //cpu_die_rq=1;
	    }
#endif
	  }else{
	    // Real transmit
#if ENET_TAPIF
	    logmsg("ENET: Real transmit\n");
	    if (netsock)
	      if_send_packet(netsock, pkt_tx_buf, pkt_tx_size);
#else
	    logmsg("ENET: Real transmit (Not Implemented Yet)\n");
#endif
	  }
	  pkt_tx_size=0; // Reset TX flag
	}
	ENET_RAM[cbl_addr+1] &= 0xC0; // Off status 
	ENET_RAM[cbl_addr+1] |= 0x20; // On COMPLETED OK, S9 NOT! set (Xmit failed, lost CTS)
	cu_cdn=1;
	break;

      case 5:
	logmsg("ENET: TDR command\n");
	ENET_RAM[cbl_addr+1] &= 0xC0; // Off status 
	ENET_RAM[cbl_addr+1] |= 0x20; // Set OK flag
	ENET_RAM[cbl_addr+7] = 0x80; /* Link OK */
	ENET_RAM[cbl_addr+6] = 0;
	cu_cdn = 1;
	break;

      case 6:
	logmsg("ENET: Dump Status Registers command\n");
	{
	  /* #### This doesn't seem to be correct - FIXME! */
	  unsigned int bufaddr = ENET_RAM[cbl_addr+6];
	  bufaddr <<= 8;
	  bufaddr |= ENET_RAM[cbl_addr+7]; /* Top of 170-byte buffer */
	  /* Dump contents of LCC registers */
	  ENET_RAM[bufaddr+0] = 12; /* byte count */
	  ENET_RAM[bufaddr+1]  = enet_fifo_limit;
	  ENET_RAM[bufaddr+2] = ((enet_addr_len&1) << 7) | (enet_sav_bf << 6) |
	    (enet_extn_sync << 5);
	  ENET_RAM[bufaddr+3] = (enet_ext_lb << 7) | (enet_int_lb << 6) |
	    (enet_preamble_len << 4) | (enet_a_t_loc << 2) | ((enet_addr_len & 6)>>1);
	  ENET_RAM[bufaddr+4] = (enet_bof_met << 7) | (enet_acr << 4) | enet_linear_prio;
	  ENET_RAM[bufaddr+5] = enet_intrfrm_len;
	  ENET_RAM[bufaddr+6] = enet_slot_time & 0xff;
	  ENET_RAM[bufaddr+7] = (enet_retry_num << 4) | (enet_slot_time >> 8);
	  ENET_RAM[bufaddr+8] = (enet_padding << 7) | (enet_bt_stf << 6) |
	    (enet_crc_16 << 5) | (enet_no_crc << 4) | (enet_tx_ncs << 3) |
	    (enet_man_nrz << 2) | (enet_bc_dis << 1) | enet_pr_m;
	  ENET_RAM[bufaddr+9] = (enet_cdt_src << 7) | (enet_cdt_filter << 4) |
	    (enet_cs_src << 3) | enet_cs_filter;
	  ENET_RAM[bufaddr+10] = enet_min_frm_len;
#if 0
	  show_dump_enet_status("dump status");
#endif
	}
	ENET_RAM[cbl_addr+1] &= 0xC0; // Off status 
	ENET_RAM[cbl_addr+1] |= 0x20; // On COMPLETED OK
	cu_cdn = 1;
	break;

      case 7: // DIAGNOSE
	// Causes an internal self-test to run.
	// We'll force this to return OK after a delay.
	if(delay_counter == 0){
	  logmsgf("ENET: DIAGNOSE: CBL @ 0x%lX\n",cbl_addr);
	}
	if(delay_counter < 3){
	  delay_counter++;
	}else{
	  ENET_RAM[cbl_addr+1] = 0x20; // On TEST COMPLETED OK
	  logmsgf("ENET: DIAGNOSE COMPLETED\n");
	  delay_counter = 0;
	  cu_cdn=1;
	}
	break;

      default:
	logmsgf("ENET: CU Command Unknown! 0x%X\n",(ENET_RAM[cbl_addr+2]&0x07));
	cpu_die_rq=1; cu_run = 0;
      }

      // Mark command completed
      if(cu_cdn != 0){
	ENET_RAM[cbl_addr+1] &= 0x3F; // Turn off BUSY
	ENET_RAM[cbl_addr+1] |= 0x80; // Turn on COMPLETED 
	// Are we done?
	// logmsgf("ENDFLAG 0x%X\n",ENET_RAM[cbl_addr+3]);
	if((ENET_RAM[cbl_addr+3]&0x20) != 0){
	  // INTERRUPT ON COMPLETION OF THIS COMMAND
	  logmsgf("ENET: Interrupt-on-Completion requested\n");
	  PLOCK(&scb_status_lock);
	  ENET_RAM[scb_addr+1] |= 0x80; // Event Pending
	  PUNLOCK(&scb_status_lock);
	  // If events enabled
	  if(enet_conf_reg&0x02){
	    // Generate event
	    nubus_io_request(NB_WRITE,enet_evnt_addr,0xFF,0xF1); 
	    logmsgf("ENET: Event posted to 0x%lX\n",enet_evnt_addr);
	  }else{
	    logmsgf("ENET: Event requested w/ events disabled\n");
	    // nubus_io_request(NB_WRITE,enet_evnt_addr,0xFF,0xF1); // Event anyway
	    //	    cpu_die_rq=1;
	  }
	}
	if((ENET_RAM[cbl_addr+3]&0x40) != 0){
	  // SUSPEND ON COMPLETION OF THIS COMMAND
	  unsigned long tmpaddr = cbl_addr;
	  // Load address of next command block for execution
	  cbl_addr = ENET_RAM[tmpaddr+5];
	  cbl_addr <<= 8;
	  cbl_addr |= ENET_RAM[tmpaddr+4];
	  // Just stop here.
	  PLOCK(&scb_status_lock);
	  ENET_RAM[scb_addr+1] &= 0xF0;
	  ENET_RAM[scb_addr+1] |= 0x01; // CU Suspended
	  PUNLOCK(&scb_status_lock);
	  cu_run=0;
	  break; // Leave CU run loop
	}
	if((ENET_RAM[cbl_addr+3]&0x80) != 0){
	  // END of command list
	  PLOCK(&scb_status_lock);
	  ENET_RAM[scb_addr+1] &= 0xF0;
	  PUNLOCK(&scb_status_lock);
	  // ENET_RAM[scb_addr+1] = 0x00; // CU Not Active
	  cu_run=0;
	  cu_cdn=1; // Stay Set
	}else{
	  // Get next command address
	  unsigned long tmpaddr = cbl_addr;
	  cbl_addr = ENET_RAM[tmpaddr+5];
	  cbl_addr <<= 8;
	  cbl_addr |= ENET_RAM[tmpaddr+4];
	  // cpu_die_rq=1; cu_done=1;
	  // logmsgf("NXTCMD 0x%lX\n",cbl_addr);
	  // Run next command
	  cu_cdn=0;
	}      
      }
      break;

    case 2: // RESUME COMMAND LIST
      PLOCK(&scb_status_lock);
      ENET_RAM[scb_addr+1] &= 0xD0; // Turn off CNA
      ENET_RAM[scb_addr+1] |= 0x02; // CU Active
      PUNLOCK(&scb_status_lock);
      cu_cmd = 1; // START LIST AGAIN
      break;

    case 3: // SUSPEND COMMAND LIST
      // If I just stop here...
      PLOCK(&scb_status_lock);
      ENET_RAM[scb_addr+1] &= 0xD0;
      ENET_RAM[scb_addr+1] |= 0x21; // CU Suspended and Not Active
      PUNLOCK(&scb_status_lock);
      cu_run=0;
      break;

    case 4: // ABORT COMMAND
      PLOCK(&scb_status_lock);
      ENET_RAM[scb_addr+1] &= 0xF0; // CU Idle
      ENET_RAM[scb_addr+1] |= 0x20; // CU Not Active
      PUNLOCK(&scb_status_lock);
      cu_run=0;
      break;

    default:
      logmsgf("ENET: UNKNOWN CU Command 0x%x\n", cu_cmd);
      cpu_die_rq=1;
    }
    /* clear command: needed to "ack" it's done */
    logmsgf("ENET: SCB command byte 1 now 0x%x\n",
	    ENET_RAM[scb_addr+3]);
    ENET_RAM[scb_addr+3] = 0;
  }

  if(ru_run != 0){
    // RU EXECUTION
    switch (ru_cmd) {		/* (ENET_RAM[scb_addr+2]>>4)&0x3 */
    case 0: // NOP
      //logmsgf("ENET: RU NOP\n");
      ru_run = 0;
      break;

    case 1: // OPERATE RECIEVER
      logmsgf("ENET: RU Start\n");
      // Mark RU operating (allows receiver thread to work)
      PLOCK(&scb_status_lock);
      ENET_RAM[scb_addr+0] = 0x40; /* Ready/enabled */
      ENET_RAM[scb_addr+1] &= 0xEF; // Turn off RNR
      PUNLOCK(&scb_status_lock);
      ru_cmd = ru_run = 0;
      break;

    case 2: // Resume
      PLOCK(&scb_status_lock);
      if (ENET_RAM[scb_addr+0] != 0x10)
	logmsgf("ENET: **** RU Resume when not suspended: in state %d\n",
		ENET_RAM[scb_addr+0]>>4);
      else
	logmsg("ENET: RU Resume\n");
      ENET_RAM[scb_addr+0] = 0x00; /* Idle */
      ENET_RAM[scb_addr+1] &= 0xEF; // Turn off RNR
      PUNLOCK(&scb_status_lock);
      ru_cmd = ru_run = 0;
      break;

    case 3: // SUSPEND
      logmsgf("ENET: RU Suspend\n");
      PLOCK(&scb_status_lock);
      ENET_RAM[scb_addr+0] = 0x10;  // RU Suspended
      // ENET_RAM[scb_addr+1] |= 0x10; // RU Not Ready
      PUNLOCK(&scb_status_lock);
      ru_cmd = ru_run=0; ru_rx_init=0;
      break;

    case 4: // ABORT
      logmsgf("ENET: RU Abort\n");
      PLOCK(&scb_status_lock);
      ENET_RAM[scb_addr+0] = 0x00;  // RU Idle
      ENET_RAM[scb_addr+1] |= 0x10; // RU Not Ready
      PUNLOCK(&scb_status_lock);
      ru_cmd = ru_run=0; ru_rx_init=0;
      break;
      
    default:
      logmsgf("ENET: UNKNOWN RU Command\n");
      ru_run=0; cpu_die_rq=1;
    }
  }
}

void
describe_enet_pkt(u_char *pkt_buf, int pkt_size)
{
  int i, type = (pkt_buf[12] << 8) | pkt_buf[13];
  logmsgf("ENET: Here's the packet (length %d):\n", pkt_size);
  logmsg(" Dest:");
  for (i = 0; i < 6; i++)
    logmsgf(" %02x", pkt_buf[i]);
  logmsg("\n Src: ");
  for (i = 6; i < 12; i++)
    logmsgf(" %02x", pkt_buf[i]);
  if (type < 0x0600)
    logmsgf("\n Length: %d\n", type);
  else
    logmsgf("\n Type: %04x (%s)\n",
	    type, (type == 0x0800 ? "IP" :
		   (type == 0x0804 ? "Chaos" :
		    (type == 0x806 ? "ARP" :
		     ((type >= 0x6000 && type <= 0x6009) ? "DECnet" : "unknown")))));
  logmsg(" Data part:\n ");
  for (i = 0; i < pkt_size; i++) {
    logmsgf(" %02x", pkt_buf[i+14]);
    if ((i+1) % 16 == 0)
      logmsg("\n ");
  }
  logmsg("\n");
}

void show_rbd_structure(u_short rbd_addr)
{
  if (rbd_addr == 0xffff)
    logmsgf("   Null RBD\n");
  else {
    u_short status = (ENET_RAM[rbd_addr+1] << 8) | ENET_RAM[rbd_addr];
    u_short rbd_next = (ENET_RAM[rbd_addr+3] << 8) | ENET_RAM[rbd_addr+2];
    u_short data = (ENET_RAM[rbd_addr+5] << 8) | ENET_RAM[rbd_addr+4];
    u_short size = (ENET_RAM[rbd_addr+9] << 8) | ENET_RAM[rbd_addr+8];
    logmsgf("   RBD @ 0x%x\n    Status: 0x%x\n    Next RBD @ 0x%x\n",
	    rbd_addr, status, rbd_next);
    logmsgf("    Data address: 0x%x\n    EOL: %d, size: %d\n",
	    data, size >> 15, size & 0x3ff);
    if ((rbd_next != 0xffff) && ((size >> 15) != 1))
      show_rbd_structure(rbd_next);
  }
}
void show_rfd_structure(u_long rfd_addr)
{
  if (rfd_addr == 0xffff)
    logmsgf(" Null RFD\n");
  else {
    u_short bits = (ENET_RAM[rfd_addr+1] << 8) | ENET_RAM[rfd_addr];
    u_short morebits = (ENET_RAM[rfd_addr+3] << 8) | ENET_RAM[rfd_addr+2];
    u_short rfd_next = (ENET_RAM[rfd_addr+5] << 8) | ENET_RAM[rfd_addr+4];
    u_short rfd_rbd = (ENET_RAM[rfd_addr+7] << 8) | ENET_RAM[rfd_addr+6];
    logmsgf(" RFD @ 0x%x:\n  Status 0x%x, 0x%x", rfd_addr,
	    bits, morebits);
    logmsgf("  Next RFD @ 0x%x\n  RBD: 0x%x\n",
	    rfd_next, rfd_rbd);
    if (rfd_rbd != 0xffff)
      show_rbd_structure(rfd_rbd);
    if ((rfd_next != 0xffff) && ((morebits >> 15) != 1))
      show_rfd_structure(rfd_next);
  }  
}

/* "callback" from receiver thread */
/* A packet is received:  (see 82586 datasheet pp 10-11)
   put dest/source/type into the current RFD,
   fill in RBDs with the data,
   finally pick the next RFD and put the next-free-RBD pointer into it,
   set the FR flag on the previous one,
   and possibly interrupt the cpu
   (#### can the thread do that or should it tell the main proc to do it?)
*/
void
ru_received_packet(u_char *pkt_rx_buf, int pkt_rx_size)
{
  int pkt_index, pkt_remain, data_done = 0;
  unsigned long rfd_next, rfd_addr;
  unsigned long data_addr, rbd_addr = 0;

  logmsgf("ENET: PACKET RECEIVED: %d BYTES\n",pkt_rx_size);
  PLOCK(&rfa_lock);
  rfd_addr = rfa_addr; //(ENET_RAM[scb_addr+7] << 8) | ENET_RAM[scb_addr+6];
  PUNLOCK(&rfa_lock);
 
  if ((!ru_rx_init) || (ENET_RAM[scb_addr] != 0x40))
    logmsgf("ENET: Receiver not ready, dropping packet (scb 0x%x, state %d)\n",
	    scb_addr, ENET_RAM[scb_addr]>>4);
  else {
    // Find free RFD
    //show_rfd_structure(rfd_addr);
    while ((rfd_addr != 0xffff) &&
	   (ENET_RAM[rfd_addr+1] & 0xC0) && /* complete or busy */
	   ((ENET_RAM[rfd_addr+3] & 0x80) != 0)) /* not end-of-list */
      {
	logmsgf("ENET: RFD CDR past 0x%lx (bits 0x%lx)\n",
		rfd_addr, ENET_RAM[rfd_addr+1]);
	/* cdr down the list */
	rfd_addr = (ENET_RAM[rfd_addr+5] << 8) | ENET_RAM[rfd_addr+4];
      }
    if ((rfd_addr != 0xffff) && ((ENET_RAM[rfd_addr+1]&0xC0) == 0)) {
      // Not marked completed, must be ours for the taking
      logmsgf("ENET: Free RFD found at %X\n",rfd_addr);
      // Fill out RFD
      // Mark BUSY
      ENET_RAM[rfd_addr+1] |= 0x40; // LCC takes this RFD
      // I am purely guessing here, as the pack of vicious lies and
      // vague accusations that TI calls documentation
      // for the 82586 is blatantly wrong.
      // BV: but now we have a *little* more info from the 82586 datasheet!
      //     plus reading the TI source...

      /* Copy in the dest, source, and type fields */
      memcpy(&ENET_RAM[rfd_addr+8],pkt_rx_buf,14);

      // Set packet buffer index
      pkt_index=14; // Start data at end of header
      // and remaining data to write
      pkt_remain=pkt_rx_size - 14;

      // Get inital RBD address
      rbd_addr = ENET_RAM[rfd_addr+7];
      rbd_addr <<= 8;
      rbd_addr |= ENET_RAM[rfd_addr+6];
      logmsgf("ENET: RBD Address %X\n",rbd_addr);

      if (rbd_addr == 0xffff) {	/* Null pointer */
	logmsgf("ENET: Null RBD address %X\n", rbd_addr);
	data_done = -1;
      }
      while(!data_done) {
	if((rbd_addr != 0xffff) && ((ENET_RAM[rbd_addr+1]&0xC0) == 0)) { // Not full or finished
	  unsigned int rbd_limit;
	  // Get limit
	  rbd_limit = ENET_RAM[rbd_addr+9]&0x3F;
	  rbd_limit <<= 8;
	  rbd_limit |= ENET_RAM[rbd_addr+8];
	  logmsgf("ENET: RBD limit is %d bytes\n",rbd_limit);

	  // and data address (24-bit absolute, but only 16 bit wired)
	  data_addr = ENET_RAM[rbd_addr+5];
	  data_addr <<= 8;
	  data_addr |=ENET_RAM[rbd_addr+4];
	  //data_addr |= (ENET_RAM[rbd_addr+6] << 16);
	  logmsgf("ENET: RBD Data Address = %X\n",data_addr);

	  // Copy some data
	  if(pkt_remain > rbd_limit){
	    // Fill data buffer
	    memcpy(ENET_RAM+data_addr, pkt_rx_buf+pkt_index, rbd_limit);
	    // and fill out RBD itself
	    ENET_RAM[rbd_addr]   = rbd_limit&0xFF;
	    ENET_RAM[rbd_addr+1] = ((rbd_limit&0x3F00)>>8)|0x40; // ACTUAL COUNT VALID

	    pkt_index += rbd_limit; // Add to length and continue
	  } else {
	    // Partially fill data buffer		    
	    memcpy(ENET_RAM+data_addr,pkt_rx_buf+pkt_index,pkt_remain);
	    // and fill out RBD itself
	    ENET_RAM[rbd_addr]   = pkt_remain&0xFF;
	    ENET_RAM[rbd_addr+1] = ((pkt_remain&0x3F00)>>8)|0xC0; // ACTUAL COUNT VALID, END OF FRAME
	    // Completed!
	    data_done=1;
	  }
	  /* Get next RBD */
	  if (ENET_RAM[rbd_addr+9] & 0x80) /* End of list */
	    rbd_addr = 0xffff;
	  else
	    rbd_addr = (ENET_RAM[rbd_addr+3] << 8) | ENET_RAM[rbd_addr+2];

	} else {
	  logmsgf("ENET: Next RBD not free? Status 0x%x\n",
		  ENET_RAM[rbd_addr+1]);
	  //cpu_die_rq=1;
	  data_done = -1;
	}
      }
      // Packet finished!
      logmsgf("ENET: RX PACKET STORAGE COMPLETED\n");

      /* Put in the RBD of the next RFD */
      rfd_next = (ENET_RAM[rfd_addr+5] << 8) | ENET_RAM[rfd_addr+4];
#if 1
      /* Probably not? */
      if ((rfd_next != 0xffff) && (rbd_addr != 0xffff)) {
	unsigned int rfd_next_rbd = (ENET_RAM[rfd_next+7] << 8) | ENET_RAM[rfd_next+6];
	logmsgf("ENET: Next RFD's RBD at 0x%x, setting 0x%x\n",
		rfd_next_rbd, rbd_addr);
	ENET_RAM[rfd_next+7] = (rbd_addr >> 8) & 0xff;
	ENET_RAM[rfd_next+6] = rbd_addr & 0xff;
      } else
	logmsgf("ENET: Next RFD at 0x%x, next RBD at 0x%x\n",
		rfd_next, rbd_addr);
#endif

      // Mark RFD
      ENET_RAM[rfd_addr+1] &= 0x0F; // Remove all flags
      ENET_RAM[rfd_addr+0] = 0; // ALL flags!
      if (data_done < 0)
	ENET_RAM[rfd_addr+1] |= 0x02; // S9: Ran out of resources during reception
      else
	ENET_RAM[rfd_addr+1] |= 0xA0; // COMPLETE, RX OK

      if(ENET_RAM[rfd_addr+3]&0x40){
	logmsgf("ENET: RU Suspend-on-Completion enabled\n");
	PLOCK(&scb_status_lock);
	if (ENET_RAM[scb_addr+0] == 0x40)
	  ENET_RAM[scb_addr+1] |= 0x10; /* receiver not ready anymore */
	ENET_RAM[scb_addr+0] = 0x10; /* Suspended */
	PUNLOCK(&scb_status_lock);
      } else if (rfd_next == 0xffff) {
	logmsgf("ENET: RU end of RFDs, now idle\n");
	PLOCK(&scb_status_lock);
	if (ENET_RAM[scb_addr+0] == 0x40)
	  ENET_RAM[scb_addr+1] |= 0x10; /* receiver not ready anymore */
	ENET_RAM[scb_addr+0] = 0x00; /* receiver idle: no resources */
	PUNLOCK(&scb_status_lock);
      } else if (data_done < 0) {
	logmsgf("ENET: RU ran out of RFDs, now NO-RESOURCES\n");
	PLOCK(&scb_status_lock);
	if (ENET_RAM[scb_addr+0] == 0x40)
	  ENET_RAM[scb_addr+1] |= 0x10; /* receiver not ready anymore */
	ENET_RAM[scb_addr+0] = 0x20; /* no resources */
	PUNLOCK(&scb_status_lock);
      }	else {
	PLOCK(&scb_status_lock);
	ENET_RAM[scb_addr+0] = 0x40; /* receiver ready */
	PUNLOCK(&scb_status_lock);
#if 1
	PLOCK(&rfa_lock);
	/* Need locking to dare to do this */
	rfa_addr = rfd_next;
	/* This is really the trick, it seems */
	ENET_RAM[scb_addr+7] = (rfd_next >> 8) & 0xff;
	ENET_RAM[scb_addr+6] = rfd_next & 0xff;
	PUNLOCK(&rfa_lock);
#endif
      }
      PLOCK(&scb_status_lock);
      ENET_RAM[scb_addr+1] |= 0x40; // Frame Recieved
      PUNLOCK(&scb_status_lock);

/* This hardly works from a thread, but Lisp doesn't enable interrupts anyway? */
      // If events enabled
      if(enet_conf_reg&0x02){
	// Generate event
#if 0
	nubus_io_request(NB_WRITE,enet_evnt_addr,0xFF,0xF1); 
	logmsgf("ENET: EVENT POSTED TO 0x%lX\n",enet_evnt_addr);
#else
	logmsgf("ENET: **** Event requested but not implemented!\n");
	/* #### Main thread should to do it for us */
#endif
      } else {
	logmsgf("ENET: Event requested w/ events disabled\n");
      }
      // All done, release buffer
    } else {
      logmsgf("ENET: RFA has no free RFD (0x%x)\n",
	      ENET_RAM[rfd_addr+1]);	      
      PLOCK(&scb_status_lock);
      ENET_RAM[scb_addr] = 0x20; /* No resources */
      PUNLOCK(&scb_status_lock);
    }
  }
}
