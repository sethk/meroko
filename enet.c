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

#include "meroko.h"
#include "nubus.h"

unsigned char ENET_ROM[0x1000]; // 4K ROM
unsigned char ENET_RAM[0x8000]; // 32K BUFFER RAM

#define PROM_FNAME "proms/2236430_ENET"

// External stuff
extern int cpu_die_rq;
extern unsigned int ldb(unsigned long long value, int size, int position);

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
unsigned long scb_addr=0;
// Address of CBL
unsigned long cbl_addr=0;
// Address of RFA
unsigned long rfa_addr=0;
// SCB Command Word
unsigned long scb_command=0;

// From command word
int ru_cmd,cu_cmd; // RU and CU commands	
int ru_run,cu_run; // RU and CU run flags
int cu_cdn;        // CU Command Done flag
int ru_rx_init=0;  // RX initialized

// Initialization
void enet_init(){
  FILE *romfile;
  int x=0;

  // Load ROM image
  romfile = fopen(PROM_FNAME,"r");
  if(romfile == NULL){
    perror("enet-fopen");
    exit(-1);
  }
  while(x < 0x1000){
    fread(&ENET_ROM[x],1,1,romfile);
    x++;
  }
  fclose(romfile);
}

// Reset ethernet configuration
void enet_conf_reset(){
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
void enet_nubus_io(){
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
    NUbus_acknowledge=1;
    return;
  }

  if(NUbus_Request == VM_BYTE_READ && NUbus_Addr < 0x008000){
    unsigned int RAMaddr = (NUbus_Addr&0x7FFF);
    NUbus_Data = ENET_RAM[RAMaddr]; NUbus_Data &= 0xFF;
    NUbus_Data <<= (8*(NUbus_Addr&0x03));
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
    ENET_RAM[RAMaddr] = ((NUbus_Data>>(8*(NUbus_Addr&0x03)))&0xFF);
    NUbus_acknowledge=1;
    return;
  }

  if(NUbus_Request == VM_WRITE && NUbus_Addr < 0x008000){
    unsigned int RAMaddr = (NUbus_Addr&0x7FFF);
    unsigned long RAMdata = NUbus_Data;

    //    logmsgf("ENET: RAM WORD WRITE: 0x%lX @ 0x%lX\n",NUbus_Data,RAMaddr);

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
	 0x003 = FAULT LED
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

// CLOCK PULSE
inline void enet_clock_pulse(){
  if(channel_attn){
    // CHANNEL ATTENTION - The system configuration data has changed
    // First, read the System Configuration Pointer from bytes 0x7FF4-0x7FFF of buffer RAM 
    // The Intermediate SCP Address is at SCP+6,7
    iscp_addr = ENET_RAM[0x7FFD];
    iscp_addr = iscp_addr << 8;
    iscp_addr |= ENET_RAM[0x7FFC];
    logmsgf("ENET: ISCP Address 0x%lX\n",iscp_addr);
    // Now read the System Configuration Block address from ISCB
    scb_offset_addr = ENET_RAM[iscp_addr+3];
    scb_offset_addr <<= 8;
    scb_offset_addr |= ENET_RAM[iscp_addr+2];
    
    scb_base_addr = ENET_RAM[iscp_addr+5];
    scb_base_addr <<= 8;
    scb_base_addr |= ENET_RAM[iscp_addr+4];
    logmsgf("ENET: SCB Base Address 0x%lX, Offset 0x%lX\n",scb_base_addr,scb_offset_addr);
    // Now read SCB
    scb_addr = scb_base_addr + scb_offset_addr;

    scb_command = ENET_RAM[scb_addr+3];
    scb_command <<= 8;
    scb_command |= ENET_RAM[scb_addr+2];
    
    logmsgf("ENET: SCB COMMND = 0x%lX\n",scb_command);
    
    // Parse out command
    ru_cmd = (scb_command&0x70)>>4;
    cu_cmd = (scb_command&0x700)>>8;
    if(scb_command&0x80){
      logmsgf("ENET: SCB RESET COMMAND\n");       
      enet_conf_reset();
      cpu_die_rq=1;
    }
    logmsgf("ENET: RU CMD = %X, CU CMD = %X\n",ru_cmd,cu_cmd);

    // Read address of CU command block
    cbl_addr = ENET_RAM[scb_addr+5];
    cbl_addr <<= 8;
    cbl_addr |= ENET_RAM[scb_addr+4];
    logmsgf("ENET: CBL @ 0x%lX\n",cbl_addr);
    cu_cdn = 0; // Not Done
    // Read address of RFA
    rfa_addr = ENET_RAM[scb_addr+7];
    rfa_addr <<= 8;
    rfa_addr |= ENET_RAM[scb_addr+6];
    // Allow CU and RU to run
    ru_run=1; cu_run=1;
    // and continue on
    // Clear iSCP BUSY flag
    ENET_RAM[iscp_addr]=0; ENET_RAM[iscp_addr+1]=0;
    channel_attn=0; 
    delay_counter = 0;
  }  

  //    ENET_RAM[scb_addr+2] = 0x00; // Don't Clear

  if(cu_run != 0){
    // CU EXECUTION
    //    ENET_RAM[scb_addr+1] &= 0xD0; // Turn off CNA
    // ENET_RAM[scb_addr+1] |= 0x02; // CU Active
    // Go!
    switch(cu_cmd){
    case 0: // NOP
      // logmsgf("ENET: CU NOP\n");
      // ENET_RAM[cbl_addr+1] = 0x80; // Status: CU Command Completed (With Interrupt)
      // ENET_RAM[scb_addr+3] = 0x80; // Control: Acknowledge CU Command Completed 
      // ENET_RAM[scb_addr+1] = 0x00; // CU Not Active
      // cu_run=0;
      break;
    
    case 1: // Run Command List
      ENET_RAM[scb_addr+1] &= 0xD0; // Turn off CNA and CUS
      ENET_RAM[scb_addr+1] |= 0x02; // CU Active
      // logmsgf("ENET: CU START COMMAND LIST\n");
      // CU command processing loop
      //      logmsgf("ENET: SCB CBLADR = 0x%lX\n",cbl_addr);	      
      ENET_RAM[cbl_addr+1] &= 0x3F; // Turn off flags
      ENET_RAM[cbl_addr+1] |= 0x40; // Turn on BUSY
      // Run!
      switch((ENET_RAM[cbl_addr+2]&0x7)){

      case 0: // NOP
	// logmsgf("ENET: CU Command NOP\n");
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
	logmsgf("ENET: CU RECONFIGURATION REQUEST @ 0x%lX\n",cbl_addr);
	// Reconfigure
	if((ENET_RAM[cbl_addr+6]&0xF) != 12){
	  logmsgf("ENET: CONFIGURATION REQUEST SIZE %d UNSUPPORTED\n",ENET_RAM[cbl_addr+6]&0xF);
	  cpu_die_rq=1;
	}
	enet_fifo_limit=     (ENET_RAM[cbl_addr+7] &0xFF);         // FIFO threshold
	enet_extn_sync=      (ENET_RAM[cbl_addr+8] >> 5) & 0x1;    // SRDY/ARDY: use external sync
	enet_sav_bf=	     (ENET_RAM[cbl_addr+8] >> 6) & 0x1;    // SAV-BF: Save Bad Frames flag
	enet_addr_len=       (ENET_RAM[cbl_addr+8] >> 7) & 0x1;    // Ethernet Address Length
	enet_addr_len |=     (ENET_RAM[cbl_addr+9] << 1) & 0x2;       // Other bit
	enet_addr_len += 3;                                           // Maybe do this?
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
	  logmsgf("ENET: EXTERNAL LOOPBACK requested\n");
	  //	  cpu_die_rq=1;
	}
	if(enet_int_lb != 0){
	  logmsgf("ENET: INTERNAL LOOPBACK requested\n");
	  //	  cpu_die_rq=1;
	}
	cu_cdn=1;
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
	  logmsgf("ENET: TX: Destination MAC Address %X:%X:%X:%X:%X:%X, Size/Type %X,%X\n",
		  ENET_RAM[cbl_addr+8],ENET_RAM[cbl_addr+9],
		  ENET_RAM[cbl_addr+10],ENET_RAM[cbl_addr+11],
		  ENET_RAM[cbl_addr+12],ENET_RAM[cbl_addr+13],
		  ENET_RAM[cbl_addr+14],ENET_RAM[cbl_addr+15]);
	  // Construct header
	  // Destination Address
	  pkt_tx_buf[0] = ENET_RAM[cbl_addr+8];
	  pkt_tx_buf[1] = ENET_RAM[cbl_addr+9];
	  pkt_tx_buf[2] = ENET_RAM[cbl_addr+10];
	  pkt_tx_buf[3] = ENET_RAM[cbl_addr+11];
	  pkt_tx_buf[4] = ENET_RAM[cbl_addr+12];
	  pkt_tx_buf[5] = ENET_RAM[cbl_addr+13];
	  // Source Address
	  pkt_tx_buf[6] = enet_mac_addr[0];
	  pkt_tx_buf[7] = enet_mac_addr[1];
	  pkt_tx_buf[8] = enet_mac_addr[2];
	  pkt_tx_buf[9] = enet_mac_addr[3];
	  pkt_tx_buf[10] = enet_mac_addr[4];
	  pkt_tx_buf[11] = enet_mac_addr[5];
	  // Padding
	  pkt_tx_buf[12] = 0;
	  pkt_tx_buf[13] = 0;
	  // Type Code
	  pkt_tx_buf[14] = ENET_RAM[cbl_addr+14];
	  pkt_tx_buf[15] = ENET_RAM[cbl_addr+15];
	  pkt_tx_size=16; // Add Header Size
	  // Read TBD
	  while(!tx_done){
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
	    logmsgf("ENET: PACKET PADDED TO %d BYTES\n",enet_min_frm_len);
	    pkt_tx_size = enet_min_frm_len;
	  }
	  // Buffer ready, issue transmit
	  if(enet_int_lb != 0 || enet_ext_lb != 0 || (enet_conf_reg&0x100) != 0){
	    // Loopback?
	    logmsgf("ENET: LOOPBACK TX\n");
	    if(pkt_rx_size == 0){
	      memcpy(pkt_rx_buf,pkt_tx_buf,pkt_tx_size);
	      pkt_rx_size = pkt_tx_size;
	    }else{
	      logmsg("ENET: RX COLLISION WHILE REPEATING PACKET FOR LOOPBACK\n");
	      cpu_die_rq=1;
	    }
	  }else{
	    // Real transmit not implemented.
	  }
	  pkt_tx_size=0; // Reset TX flag
	}
	ENET_RAM[cbl_addr+1] &= 0xC0; // Off status 
	ENET_RAM[cbl_addr+1] |= 0x22; // On COMPLETED OK, S9 set
	cu_cdn=1;
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
	  logmsgf("ENET: INTERRUPT-ON-COMPLETION REQUESTED\n");
	  ENET_RAM[scb_addr+1] |= 0x80; // Event Pending
	  // If events enabled
	  if(enet_conf_reg&0x02){
	    // Generate event
	    nubus_io_request(NB_WRITE,enet_evnt_addr,0xFF,0xF1); 
	    logmsgf("ENET: EVENT POSTED TO 0x%lX\n",enet_evnt_addr);
	  }else{
	    logmsgf("ENET: EVENT REQUESTED W/ EVENTS DISABLED\n");
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
	  ENET_RAM[scb_addr+1] &= 0xF0;
	  ENET_RAM[scb_addr+1] |= 0x01; // CU Suspended
	  cu_run=0;
	  break; // Leave CU run loop
	}
	if((ENET_RAM[cbl_addr+3]&0x80) != 0){
	  // END of command list
	  ENET_RAM[scb_addr+1] &= 0xF0;
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
      ENET_RAM[scb_addr+1] &= 0xD0; // Turn off CNA
      ENET_RAM[scb_addr+1] |= 0x02; // CU Active
      cu_cmd = 1; // START LIST AGAIN
      break;

    case 3: // SUSPEND COMMAND LIST
      // If I just stop here...
      ENET_RAM[scb_addr+1] &= 0xD0;
      ENET_RAM[scb_addr+1] |= 0x21; // CU Suspended and Not Active
      cu_run=0;
      break;

    case 4: // ABORT COMMAND
      ENET_RAM[scb_addr+1] &= 0xF0; // CU Idle
      ENET_RAM[scb_addr+1] |= 0x20; // CU Not Active
      cu_run=0;
      break;

    default:
      logmsgf("ENET: UNKNOWN CU Command\n");
      cpu_die_rq=1;
    }
  }
  
  if(ru_run != 0){
    // RU EXECUTION
    switch(ru_cmd){
    case 0: // NOP
      // logmsgf("ENET: RU NOP\n");
      break;

    case 1: // OPERATE RECIEVER
      {
	int rx_done=0,data_done=0,data_addr=0,pkt_index=0;
	int pkt_remain=0;
	unsigned long rfd_addr = rfa_addr;
	unsigned long rbd_addr = 0;

	// Mark RU operating
	if(!ru_rx_init){
	  ENET_RAM[scb_addr]  |= 0x04;  // RU Ready
	  ENET_RAM[scb_addr+1] &= 0xEF; // Turn off RNR
	  ru_rx_init=1;
	}
	// Do we have a packet?
	if(pkt_rx_size != 0){
	  // Yes
	  logmsgf("ENET: PACKET RECIEVED: %d BYTES\n",pkt_rx_size);
	  // cpu_die_rq=1;
	  while(!rx_done){
	    // Find free RFD
	    if((ENET_RAM[rfd_addr+1]&0x80) == 0){
	      // Not marked completed, must be ours for the taking
	      logmsgf("ENET: Free RFD found at %X\n",rfd_addr);
	      // Fill out RFD
	      // Mark BUSY
	      ENET_RAM[rfd_addr+1] |= 0x40; // LCC takes this RFD
	      // I am purely guessing here, as the pack of vicious lies and vague accusations that TI calls documentation
	      // for the 82586 is blatantly wrong.
	      // Destination Address
	      ENET_RAM[rfd_addr+8] = pkt_rx_buf[0];
	      ENET_RAM[rfd_addr+9] = pkt_rx_buf[1];
	      ENET_RAM[rfd_addr+10] = pkt_rx_buf[2];
	      ENET_RAM[rfd_addr+11] = pkt_rx_buf[3];
	      ENET_RAM[rfd_addr+12] = pkt_rx_buf[4];
	      ENET_RAM[rfd_addr+13] = pkt_rx_buf[5];
	      // Source Address
	      ENET_RAM[rfd_addr+14] = pkt_rx_buf[6];
	      ENET_RAM[rfd_addr+15] = pkt_rx_buf[7];
	      ENET_RAM[rfd_addr+16] = pkt_rx_buf[8];
	      ENET_RAM[rfd_addr+17] = pkt_rx_buf[9];
	      ENET_RAM[rfd_addr+18] = pkt_rx_buf[10];
	      ENET_RAM[rfd_addr+19] = pkt_rx_buf[11];
	      // Type Field
	      ENET_RAM[rfd_addr+20] = pkt_rx_buf[14];
	      ENET_RAM[rfd_addr+21] = pkt_rx_buf[15];
	      // Set packet buffer index
	      pkt_index=16; // Start data at end of header
	      // and remaining data to write
	      pkt_remain=pkt_rx_size - 16;
	      // Get inital RBD address
	      rbd_addr = ENET_RAM[rfd_addr+7];
	      rbd_addr <<= 8;
	      rbd_addr |= ENET_RAM[rfd_addr+6];
	      logmsgf("ENET: RBD Address %X\n",rbd_addr);
	      while(!data_done){
		if((ENET_RAM[rbd_addr+1]&0x40) == 0){ // Not full or finished
		  unsigned int rbd_limit;
		  // Get limit
		  rbd_limit = ENET_RAM[rbd_addr+9]&0x3F;
		  rbd_limit <<= 8;
		  rbd_limit |= ENET_RAM[rbd_addr+8];		    
		  logmsgf("ENET: RBD limit is %d bytes\n",rbd_limit);
		  // and data size
		  data_addr = ENET_RAM[rbd_addr+5];
		  data_addr <<= 8;
		  data_addr |=ENET_RAM[rbd_addr+4];
		  logmsgf("ENET: RBD Data Address = %X\n",data_addr);
		  // Copy some data
		  if(pkt_remain > rbd_limit){
		    // Fill data buffer
		    memcpy(ENET_RAM+data_addr,pkt_rx_buf+pkt_index,rbd_limit);
		    // and fill out RBD itself
		    ENET_RAM[rbd_addr]   = rbd_limit&0xFF;
		    ENET_RAM[rbd_addr+1] = ((rbd_limit&0x3F00)>>8)|0x40; // ACTUAL COUNT VALID
		    pkt_index += rbd_limit; // Add to length and continue
		  }else{
		    // Partially fill data buffer		    
		    memcpy(ENET_RAM+data_addr,pkt_rx_buf+pkt_index,pkt_remain);
		    // and fill out RBD itself
		    ENET_RAM[rbd_addr]   = rbd_limit&0xFF;
		    ENET_RAM[rbd_addr+1] = ((rbd_limit&0x3F00)>>8)|0xC0; // ACTUAL COUNT VALID, END OF FRAME
		    // Completed!
		    data_done=1;
		  }
		}
		if(!data_done){
		  logmsgf("ENET: NEED ANOTHER RBD\n");
		  cpu_die_rq=1; data_done=1;
		}
	      }
	      // Packet finished!
	      logmsgf("ENET: RX PACKET STORAGE COMPLETED\n");
	      // Mark RFD
	      ENET_RAM[rfd_addr+1] &= 0x0F; // Remove all flags
	      ENET_RAM[rfd_addr+1] |= 0xA0; // COMPLETE, RX OK
	      if(ENET_RAM[rfd_addr+3]&0x40){
		logmsgf("ENET: RX SUSPEND-ON-COMPLETION ENABLED\n");
		ru_cmd = 3;
	      }
	      ENET_RAM[scb_addr+1] |= 0x40; // Frame Recieved
	      // If events enabled
	      if(enet_conf_reg&0x02){
		// Generate event
		nubus_io_request(NB_WRITE,enet_evnt_addr,0xFF,0xF1); 
		logmsgf("ENET: EVENT POSTED TO 0x%lX\n",enet_evnt_addr);
	      }else{
		logmsgf("ENET: EVENT REQUESTED W/ EVENTS DISABLED\n");
		//		cpu_die_rq=1;
		// nubus_io_request(NB_WRITE,enet_evnt_addr,0xFF,0xF1); // Event anyway
	      }
	      pkt_rx_size=0; // All done, release buffer
	      // Done!
	    }else{
	      // Pick next RFD
	      if(ENET_RAM[rfd_addr+3]&0x80){ // End of list
		logmsgf("ENET: RFA has no free RFDs!\n");
	      }else{
		logmsgf("ENET: RX CONTINUE RQD\n");
	      }
	      // logmsgf("ENET: DATA: 0x%X 0x%X 0x%X 0x%X\n",ENET_RAM[rfa_addr+0],ENET_RAM[rfa_addr+1],ENET_RAM[rfa_addr+2],ENET_RAM[rfa_addr+3]);
	      // logmsgf("ENET: DATA: 0x%X 0x%X 0x%X 0x%X\n",ENET_RAM[rfa_addr+4],ENET_RAM[rfa_addr+5],ENET_RAM[rfa_addr+6],ENET_RAM[rfa_addr+7]);
	      cpu_die_rq=1;
	    }
	    rx_done=1; 
	  }
	}	   
      }
      break;

    case 3: // SUSPEND
      // logmsgf("ENET: RU SUSPEND\n");
      ENET_RAM[scb_addr+0] = 0x10;  // RU Suspended
      // ENET_RAM[scb_addr+1] |= 0x10; // RU Not Ready
      ru_run=0; ru_rx_init=0;
      break;

    case 4: // ABORT
      ENET_RAM[scb_addr+0] = 0x00;  // RU Idle
      ENET_RAM[scb_addr+1] |= 0x10; // RU Not Ready
      ru_run=0; ru_rx_init=0;
      break;
      
    default:
      logmsgf("ENET: UNKNOWN RU Command\n");
      ru_run=0; cpu_die_rq=1;
    }
  }
}
