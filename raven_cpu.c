/* Raven CPU - Implementation

 This file is subject to the terms and conditions of the GNU General Public
 License.  See the file COPYING in the main directory of this archive for
 more details.

*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <ncurses.h>
#include <string.h>

#include "raven_cpu.h"
#include "meroko.h"
#include "nubus.h"
#include "localbus.h"
#include "nubus.h"
#include "nupi.h"
#include "mem8.h"
#include "sib.h"
#include "enet.h"
#include "raven_rom.h"
#include "sdl.h"


// NCurses
WINDOW *cpu_w;

int breakpoint_armed=0;
unsigned int tracepoint_data=0;
int delay_tune=0; // Was 20
int delay_indx=0;
int do_disp_trace=0;
int do_tmem_trace=0;
int trap_test=0,trap_type=0;

int stop_at_nubus=0;

/* MCR bits */
#define MCR_Slot_ID                    0xF0000000 
#define MCR_Self_Test_Flag             0x08000000
#define MCR_Chaining_Enable            0x04000000
#define MCR_Misc_Op_Group_1            0x02000000
#define MCR_Misc_Op_Group_0            0x01000000
#define MCR_Loop_On_Test               0x00800000
#define MCR_Need_Fetch                 0x00400000
#define MCR_NU_Bus_Reset               0x00200000
// POWERFAIL and WARM-BOOT ENABLE bit
#define MCR_Local_Reset                0x00100000
#define MCR_Int_Level_3                0x00080000
#define MCR_Int_Level_2                0x00040000
#define MCR_Int_Level_1                0x00020000
#define MCR_Int_Level_0                0x00010000
// All interrupts
#define MCR_Interrupts                 0x000F0000
#define MCR_Int_Enable                 0x00008000
#define MCR_Sequence_Break             0x00004000
// ABORT-ON-BUS-ERROR ENABLE
#define MCR_Parity_Trap_Enable         0x00002000
// HALT-ON-PARITY-ERROR ENABLE
#define MCR_Parity_Halt_Enable         0x00001000
#define MCR_PROM_Disable               0x00000800
// This is also called BUS LOCK
#define MCR_Interlocked_Memory_Control 0x00000400
#define MCR_Forced_Access_Request      0x00000200
#define MCR_Memory_Cycle_Enable        0x00000100
#define MCR_Subsys_Flag                0x00000080
#define MCR_Test_Fail_Flag             0x00000040
// LEDs
#define MCR_LED_5_                     0x00000020
#define MCR_LED_4_                     0x00000010
#define MCR_LED_3_                     0x00000008
#define MCR_LED_2_                     0x00000004
#define MCR_LED_1_                     0x00000002
#define MCR_LED_0_                     0x00000001
// Mine
#define MCR_LED_CODE                   0x0000003F

// String enumeration

// Instruction fields
const char *opcode[4] = { "ALU ","BYTE","JUMP","DISP" };  

const char *alu_op_str[040] = {
  "SETZ",       // 0
  "AND",        // 1
  "ANDCA",      // 2
  "SETM",       // 3
  "ANDCM",      // 4
  "SETA",       // 5
  "XOR",        // 6
  "IOR",        // 7
  "ANDCB",      // 10
  "EQV",        // 11   
  "SETCA",      // 12
  "ORCA",       // 13
  "SETCM",      // 14
  "ORCM",       // 15
  "ORCB",       // 16
  "SETO",       // 17
  "MUL",        // 20
  "MUL-Last",   // 21
  "DIV",        // 22
  "DIV-First",  // 23
  "DIV-Corr",   // 24
  "ALU-25",     // 25
  "ALU-26",     // 26
  "ALU-27",     // 27
  "ALU-30",     // 30
  "ADD",        // 31
  "ALU-32",     // 32
  "ALU-33",     // 33
  "ALU-M",      // 34
  "ALU-35",     // 35
  "SUB",        // 36
  "M+M"         // 37
};

const char *obus_control_str[8] = {
  "OBus-A-Bus",
  "OBus-R-Bus",
  "OBus-A-Bus2",
  "OBus-Normal",
  "OBus-LeftShift-1",
  "OBus-RightShift-1",
  "OBus-Pointer-Extend",
  "OBus-Mirror" };

const char *q_control_str[4] = {
  "NOP",
  "Shift-Left",
  "Shift-Right",
  "Load" };

const char *byte_op_str[4] = {
  "NOP",
  "Load-Byte",
  "Selective-Deposit",
  "Deposit-Byte" };

const char *jump_op_str[10] = {
  "Jump-XCT-Next",
  "JUMP",
  "Call-XCT-Next",
  "CALL",
  "Return-XCT-Next",
  "RETURN",
  "NOP",
  "SKIP" };

const char *disp_src_str[4] = {
  "M-Source",
  "M-Tag",
  "MIR",
  "MIR2" };

const char *disp_map_ena_str[4] = {
  "Normal",
  "GC-Volatile",
  "Old-Space",
  "GCV-Or-Oldspace" };
  
const char *disp_op_str[4] = {
  "DISPATCH",
  "I-READ",
  "I-WRITE",
  "DISP-ERR" };

const char *mbs_str[060] = {
  "VMA",            // 100
  "Q-R",
  "Macro-Argument",
  "Micro-Stack-Ptr",
  "MCR",
  "Location-Counter",
  "Map-lv2-addr",
  "Read-I-Arg",
  "Map-level-1",     // 110
  "Map-Lv2-Cntl",
  "Macro-IR",
  "Macro-Branch-Field",
  "114",
  "115",
  "116",
  "117",
  "uStack-Data",     // 120
  "uStack-Pop",
  "MD",
  "123",
  "124",
  "125",
  "126",
  "127",
  "130",
  "131",
  "132",
  "133",
  "134",
  "135",
  "136",
  "137",
  "C-PDL-Pointer",    // 140
  "C-PDL-Index",
  "142",
  "143",
  "C-PDL-Pointer-Pop",
  "C-PDL-Index-Dec",
  "146",
  "147",
  "PDL-Pointer",
  "PDL-Index",
  "PDL-Pointer-Pop",
  "153",
  "154",
  "PDL-Index-Dec"     // 155 
};
  
const char *mbd_str[0100] = {
  "NOP",
  "Location-Counter",
  "MCR",
  "uStack-Pointer",
  "uStack-Data",
  "uStack-Data-Push",
  "IMod-Low",
  "IMod-Hi",
  "Macro-IR", // 010
  "11",
  "12",
  "13",
  "14",
  "15",
  "16",
  "TEST-SYNC",
  "VMA",        // 020
  "VMA-Write-Map-LV1",
  "VMA-Write-Map-LV2-C",
  "VMA-Write-Map-LV2-A",
  "VMA-Start-Read",
  "VMA-Start-Write",
  "VMA-Start-Unmapped-Read",
  "VMA-Start-Unmapped-Write",
  "MD",         // 030
  "MD-Write-Map-LV1",
  "MD-Write-Map-LV2-C",
  "MD-Write-Map-LV2-A",
  "MD-Start-Read",
  "MD-Start-Write",
  "MD-Start-Unmapped-Read",
  "MD-Start-Unmapped-Write",
  "C-PDL-Pointer", // 040
  "C-PDL-Index",
  "42",
  "43",
  "C-PDL-Pointer-Push",
  "C-PDL-Pointer-Inc",
  "46",
  "47",
  "PDL-Pointer", // 050
  "PDL-Index",
  "52",
  "53",
  "54",
  "55",
  "56",
  "57",
  "60",
  "61",
  "62",
  "63",
  "64",
  "65",
  "VMA-Start-Unmapped-Byte-Read",
  "VMA-Start-Unmapped-Byte-Write",
  "70",
  "71",
  "72",
  "73",
  "74",
  "75",
  "MD-Start-Unmapped-Byte-Read",
  "MD-Start-Unmapped-Byte-Write" };  

const char *abj_str[010] = {
  "A-NOP",
  "A-SKIP",
  "A-CALL-ILLOP",
  "A-CALL-TRAP",
  "A-CALL-BUSERR",
  "A-UNUSED",
  "A-RETURN",
  "A-RETURN-XCT-NEXT" };

const char *cond_str[021] = {
  "BIT-SET",
  "LESS",
  "LESS-OR-EQ",
  "NOT-EQ",
  "PAGE-FAULT",
  "PAGE-FAULT-OR-INT",
  "PAGE-FAULT-OR-INT-OR-SQB",
  "ALWAYS",
  "TAG-NOT-EQ",   // 010
  "NOT-MEM-BUSY",
  "Q-ZERO",
  "NUBUS-ERROR",
  "NOT-FIXNUM-OVERFLOW",
  "BOXED-SIGN-BIT",
  "NO-INTERRUPT",
  "17",
  "TAG-CLASSIFY" };

// MACROINSTRUCTION FIELDS
const char *macro_reg_str[8] = {
  "FEF",
  "FEF+64",
  "FEF+128",
  "FEF+192",
  "CONSTANTS",
  "LOCALS",
  "ARGS",
  "PDL/IVAR" };

const char *macro_dst_str[4] = {
  "IGNORE",
  "PDL",
  "RETURN",
  "LAST" };

/* ***** Onboard stuffs ***** */

/* Buses */
uint32 Abus; // A-bus
uint32 Mbus; // M-bus
uint32 Rbus; // R-bus
uint64 Ibus; // I-bus
uint32 Obus; // O-bus

/* Local Bus Interface */
int Memory_Busy;       // Local Bus Busy

uint32 PI_Sources;
int Highest_PI=0;
int Active_PI=0;
int Current_PI=0;

int Page_Fault;
int cached_gcv=0;

/* Halt control */
int cpu_die = 0;
int cpu_die_rq = 0;
int cpu_stop_en = 0;
/* Debug */
int tracerq = 0;

/* Memories */
uint8 Irom[7][2048];      // Control Store PROMs
uint64 PCS[2048];	// PROM Control Store
uint64 WCS[16384];	// Writable Control Store
uint32 Amemory[1024];	// A-memory
uint32 Mmemory[64];	// M-memory
uint32 PDLmemory[1024];	// PDL-memory
uint32 Dmemory[4096];	// Dispatch Memory
uint32 Tmemory[16];	// Tag-Classifier RAM 

const uint32 shift_left_mask[040] = {
  0x00000001,0x00000003,0x00000007,0x0000000F,
  0x0000001F,0x0000003F,0x0000007F,0x000000FF,
  0x000001FF,0x000003FF,0x000007FF,0x00000FFF,
  0x00001FFF,0x00003FFF,0x00007FFF,0x0000FFFF,
  0x0001FFFF,0x0003FFFF,0x0007FFFF,0x000FFFFF,
  0x001FFFFF,0x003FFFFF,0x007FFFFF,0x00FFFFFF,
  0x01FFFFFF,0x03FFFFFF,0x07FFFFFF,0x0FFFFFFF,
  0x1FFFFFFF,0x3FFFFFFF,0x7FFFFFFF,0xFFFFFFFF 
};

const uint32 shift_right_mask[040] = {
  0xFFFFFFFF,0xFFFFFFFE,0xFFFFFFFC,0xFFFFFFF8,
  0xFFFFFFF0,0xFFFFFFE0,0xFFFFFFC0,0xFFFFFF80,
  0xFFFFFF00,0xFFFFFE00,0xFFFFFC00,0xFFFFF800,
  0xFFFFF000,0xFFFFE000,0xFFFFC000,0xFFFF8000,
  0xFFFF0000,0xFFFE0000,0xFFFC0000,0xFFF80000,
  0xFFF00000,0xFFE00000,0xFFC00000,0xFF800000,
  0xFF000000,0xFE000000,0xFC000000,0xF8000000,
  0xF0000000,0xE0000000,0xC0000000,0x80000000
};
    
/* MF-bus sources */
uint32 disp_constant_reg;  // Dispatch Constant
uint32 pdl_index_reg;      // PDL Index Register
uint32 pdl_ptr_reg;        // PDL Pointer Register
uint32 uPCS_ptr_reg=0;     // Microcode counter stack pointer 

uint32 loc_ctr_reg=0;      // (Micro)Location Counter Register -- Address of NEXT instruction
uint32 loc_ctr_cnt=0;      // (Micro)Location Counter Register -- Address of CURRENT instruction
int32  loc_ctr_nxt=-1;     // (Micro)Location Counter Register -- Address of AFTER-NEXT instruction
uint32 Qshift_reg;         // Q Shift Register
uint32 vm_lv1_map[4096];   // VM Map, level 1
uint32 vm_lv2_ctl[4096];   // VM Map, level 2, Control Storage
uint32 vm_lv2_adr[4096];   // VM Map, level 2, Address Storage
uint32 cached_lv1;
int32  pj14_fetch_addr=-1; // POPJ-14 Fetch Address
uint32 pj14_fetch_vma=0;   // POPJ-14 Delayed VMA
int32  pj14_fetch_go=0;    // And start pulse
/* L registers */
int last_loc_ctr;                 // Lpc

/* Others */
uint32 BARREL_SHIFTER;            // Barrel Shifter
uint32 Obus_mux;                  // O-bus multiplexer
uint32 Obus_mask;                 // O-bus mask
uint32 uPCS_stack[64];            // Microcode counter stack memory

uint32 dispatch_ram[4096];        // Dispatch RAM

uint64 Iregister;                 // 56b Instruction register

uint32 LCregister;                // (Macro)Location Counter Register
int need_fetch;
uint32 MIbuffer;                  // Macroinstruction Buffer

uint32 MDregister;                // Memory Data register for local bus
uint32 VMAregister;               // Virtual Memory Address
uint32 Qregister;                 // Q register
uint32 MCregister;                // Machine Control Register

int64 ALU_Result;                 // Result of ALU operation
uint64 Obus_Input;                // Obus Multiplexer Input

bool ALU_Carry_Out;               // Carry-Out
bool ALU_Fixnum_Oflow;            // FIXNUM Overflow
bool imod_en;                     // Enable IMOD
uint32 imod_lo,imod_hi;           // IMOD storage
uint8 raven_config_reg=0;         // NUbus Configuration Register

// Instruction Data that gets shared around
int opword;
int MInst_Parity;
int MInst_Abbrv_Jump;
int MInst_M_Source_Addr;
int MInst_A_Source_Addr;
int MInst_Halt;
int MInst_M_Source_Addr;
int MInst_A_Source_Addr;
int MInst_Dest_Functnl;
int MInst_Cond_ClRAM_Sel;
int MInst_Cond_Invrt_Sns;
int MInst_Cond_Select;
int MInst_ALU_Opcode;
int MInst_ALU_Carry_In;
int MInst_Output_Bus_Ctl;
int MInst_Write_ClRAM;
int MInst_Mask_Rotate;
int MInst_Source_Rotate;
int MInst_Rotation_Directn;
int MInst_Rotation_Count;
unsigned int MInst_Rotation_Len;

// Software flags (Nonexistent on real hardware)
bool power_on_reset=1;            // Power-On Reset (LOAD PROM CONTENTS)
bool test_true;                   // Condition Test True

// Performance monitoring
unsigned long long ccount;
unsigned long long cycle_count;

// Utility functions
void raven_halt(){
  cpu_die_rq=1;
  cpu_stop_en=1; // Enable stop condition
}

void raven_step(){
  cpu_die_rq=1;
  cpu_die=0;
}

void raven_cont(){
  cpu_die_rq=0;
  cpu_die=0;
  tracerq=0;
}

inline uint32 ldb(uint64 value, int size, int position){
  uint64 mask,result;
  // Create mask
  mask = 0xFFFFFFFFFFFFFFFFLL; // Get all ones
  mask = mask >> (64 - size);  // Left-shift out the ones we don't need
  // logmsgf("Generated mask 0x%LX for length %d",mask,size);
  // Shift value into position
  result = (value >> position);
  return(result&mask);
}

// ROTATION (THIS IS VERY INEFFICIENT FOR THIS! C NEEDS ROTATION OPERATORS! SOMEONE FIX THAT!)
static inline uint32 left_rotate(uint32 data, int rot){
#ifdef OLD_ROTATION
  uint32 x=0xFFFFFFFF;
  uint32 y;
  uint32 result;
  // Grab a mask for the shifted-off part.
  x = x << (32 - rot);  // Left-shift out the ones we don't need
  // Save those bits and fix position
  y = (data&x)>>(32-rot);
  // Shift data
  result = data << rot;
  // Add saved bits
  result |= y;
#else
  // THIS IS SUCH A GREAT CHEAT!
  uint32 result=0;
  result  = (data >> (32-rot));
  result |= (data << rot);
#endif
  // Done
  return(result);
}

static inline uint32 right_rotate(uint32 data, int rot){
#ifdef OLD_ROTATION
  uint32 x=0xFFFFFFFF;
  uint32 y;
  uint32 result;
  // Grab a mask for the shifted-off part.
  x = x >> (32 - rot);  // Left-shift out the ones we don't need
  // Save those bits and fix position
  y = (data&x)<<(32-rot);
  // Shift data
  result = data >> rot;
  // Add saved bits
  result |= y;
#else
  // AGAIN!
  uint32 result=0;
  result  = (data << (32-rot));
  result |= (data >> rot);
#endif
  // Done
  return(result);
}

#ifdef I_PARITY_CHECK
static inline int gen_i_parity(uint64 data){
  uint64 x=0x1;
  uint32 y=0x0;

  while(x < 0x100000000000000LL){
    if(x == 0x004000000000000LL){
      x <<= 1; // Skip parity bit
    } 

    if(data&x){
      y++;
    } 
    x = x << 1; 
  }  
  y &= 1;
  /*
  logmsgf("Parity for 0x%LX is %d\n",data,y);
  */
  return(y^1); // Return ODD parity
}
#endif

void disassemble(int loc, uint64 inst, char *disasm)
{
  char dtemp[64];   // Temporary
  int opcode = ldb(inst,2,54); 

  // Disassemble the instruction in IR.
  
  // First, get opcode and switch
  switch(opcode){
  case 0: // ALU
    {
      // Destination first.
      if(ldb(inst,1,31) != 0){
	// A-Memory-Select
	sprintf(dtemp,"(A-%d)",ldb(inst,10,19));
      }else{
	// M-Memory-Select
	if(ldb(inst,6,25) != 0){
	  // Include functional destination
	  sprintf(dtemp,"(M-%d MF-%s)",ldb(inst,6,19),mbd_str[ldb(inst,6,25)]);
	}else{
	  sprintf(dtemp,"(M-%d)",ldb(inst,6,19));
	}
      }
      sprintf(disasm,"%s ", dtemp); // Create string
      // Now do function type
      if(ldb(inst,1,2) != 0){
	strcat(disasm,"CARRY-IN ");
      }
      if(ldb(inst,1,8) != 0){
	strcat(disasm,"TAGGED ");
      }
      sprintf(dtemp,"%s ",alu_op_str[ldb(inst,5,3)]);
      strcat(disasm,dtemp);
      // And source
      if(ldb(inst,7,42) > 077){
	sprintf(dtemp,"<A-%d,M-%s> ",ldb(inst,10,32),mbs_str[ldb(inst,7,42)-0100]);
      }else{
	sprintf(dtemp,"<A-%d,M-%d> ",ldb(inst,10,32),ldb(inst,7,42));
      }
      strcat(disasm,dtemp);
      // O-control
      sprintf(dtemp,"%s ",obus_control_str[ldb(inst,3,16)]);
      strcat(disasm,dtemp);
      // Conditional?
      if(ldb(inst,1,9) != 0){
	// T-Write
	sprintf(dtemp,"TM-Wt-%d ",ldb(inst,4,10));
	strcat(disasm,dtemp);
      }
      if(ldb(inst,1,14) != 0){
	// T-Read
	sprintf(dtemp,"TM-Rd-%d ",ldb(inst,4,10));
	strcat(disasm,dtemp);
      }else{
	sprintf(dtemp,"IF %s%s ",
		ldb(inst,1,15) ? "NOT " : "",
		cond_str[ldb(inst,4,10)]);
	strcat(disasm,dtemp);
      }
      // ABJ
      if(ldb(inst,3,51) != 0){
	sprintf(dtemp,"%s ",abj_str[ldb(inst,3,51)]);
	strcat(disasm,dtemp);	
      }
      // Q-Control
      if(ldb(inst,2,0) != 0){
	sprintf(dtemp,"Q-%s ",q_control_str[ldb(inst,2,0)]);
	strcat(disasm,dtemp);
      }
    }
    break;
    
  case 1: // BYTE
    {
      // Destination first.
      if(ldb(inst,1,31) != 0){
	// A-Memory-Select
	sprintf(dtemp,"(A-%d)",ldb(inst,10,19));
      }else{
	// M-Memory-Select
	if(ldb(inst,6,25) != 0){
	  // Include functional destination
	  sprintf(dtemp,"(M-%d MF-%s)",ldb(inst,6,19),mbd_str[ldb(inst,6,25)]);
	}else{
	  sprintf(dtemp,"(M-%d)",ldb(inst,6,19));
	}
      }
      sprintf(disasm,"%s ", dtemp); // Create string

      // Operation
      if(ldb(inst,1,16) != 0){
	strcat(disasm,"RIGHT ");
      }
      sprintf(dtemp,"%s (L=%d C=%d) ",byte_op_str[ldb(inst,2,17)],ldb(inst,5,5),ldb(inst,5,0));
      strcat(disasm,dtemp);
      // And source
      if(ldb(inst,7,42) > 077){
	sprintf(dtemp,"<A-%d,M-%s> ",ldb(inst,10,32),mbs_str[ldb(inst,7,42)-0100]);
      }else{
	sprintf(dtemp,"<A-%d,M-%d> ",ldb(inst,10,32),ldb(inst,7,42));
      }
      strcat(disasm,dtemp);
      // Conditional?
      if(ldb(inst,1,14) != 0){
	// T-Read
	sprintf(dtemp,"TM-Rd-%d ",ldb(inst,4,10));
	strcat(disasm,dtemp);
      }else{
	sprintf(dtemp,"IF %s%s ",
		ldb(inst,1,15) ? "NOT " : "",
		cond_str[ldb(inst,4,10)]);
	strcat(disasm,dtemp);
      }
      // ABJ
      if(ldb(inst,3,51) != 0){
	sprintf(dtemp,"%s ",abj_str[ldb(inst,3,51)]);
	strcat(disasm,dtemp);	
      }
    }
    break;

  case 2: // JUMP
    {
      // Operation and new-micro-PC
      sprintf(disasm,"%s %d ",
              jump_op_str[ldb(inst,3,5)], ldb(inst,14,18));

      // And source
      if(ldb(inst,7,42) > 077){
	sprintf(dtemp,"<A-%d,M-%s> ",ldb(inst,10,32),mbs_str[ldb(inst,7,42)-0100]);
      }else{
	sprintf(dtemp,"<A-%d,M-%d> ",ldb(inst,10,32),ldb(inst,7,42));
      }
      strcat(disasm,dtemp);
      // Conditional?
      if(ldb(inst,1,9) != 0){
	// WCS-Write
	strcat(disasm,"WCS-Write ");
      }
      if(ldb(inst,1,8) != 0){
	// WCS-Write
	strcat(disasm,"WCS-Read ");
      }

      if(ldb(inst,1,14) != 0){
	// T-Read
	sprintf(dtemp,"TM-Rd-%d ",ldb(inst,4,10));
	strcat(disasm,dtemp);
      }else{
	sprintf(dtemp,"IF %s%s ",
		ldb(inst,1,15) ? "NOT " : "",
		cond_str[ldb(inst,4,10)]);
	strcat(disasm,dtemp);
      }
      if(ldb(inst,4,10) == 0){
	sprintf(dtemp,"(Bit %d ",ldb(inst,5,0));
	if(ldb(inst,1,16) != 0){
	  strcat(dtemp,"RIGHT ");
	}
	strcat(dtemp,") ");
	strcat(disasm,dtemp);
      }
      // ABJ
      if(ldb(inst,3,51) != 0){
	sprintf(dtemp,"%s ",abj_str[ldb(inst,3,51)]);
	strcat(disasm,dtemp);	
      }
      strcat(disasm,"");
    }
    break;

  case 3: // DISP
    // Operation and new-micro-PC
    sprintf(disasm,"DISPATCH ");

    // And source
    if(ldb(inst,7,42) > 077){
      sprintf(dtemp,"<A-%d,M-%s> ",ldb(inst,10,32),mbs_str[ldb(inst,7,42)-0100]);
    }else{
      sprintf(dtemp,"<A-%d,M-%d> ",ldb(inst,10,32),ldb(inst,7,42));
    }
    strcat(disasm,dtemp);

    sprintf(dtemp, "addr %d ", (int)((Iregister>>20)&0xFFF));
    strcat(disasm,dtemp);

    if ((int)(Iregister&0x20000))
      strcat(disasm,"N ");

    if ((int)(Iregister&0x8000))
      strcat(disasm,"ISTREAM ");

    sprintf(dtemp, "%s ", disp_map_ena_str[ (int)(Iregister>>10)&3 ]);
    strcat(disasm,dtemp);

    sprintf(dtemp, "%s ", disp_src_str[ (int)(Iregister>>12)&3 ]);
    strcat(disasm,dtemp);

    sprintf(dtemp, "%s ", disp_op_str[ (int)(Iregister>>8)&3 ]);
    strcat(disasm,dtemp);

    // Operation
    if(ldb(inst,1,16) != 0){
      strcat(disasm,"RIGHT ");
    }
    sprintf(dtemp,"(L=%d C=%d) ",ldb(inst,3,5),ldb(inst,5,0));
    strcat(disasm,dtemp);

    break;
  }
}

void disassemble_MIR(){
  unsigned int MacroInstruction;
  unsigned int tmp=0;
  
  if((LCregister&0x01) == 0x01){
    MacroInstruction = MIbuffer & 0xffff;
  }else{
    MacroInstruction = ((MIbuffer&0xFFFF0000)>>16);
  }

  // Get "opcode"
  tmp = ((MacroInstruction&0x3E)>>9);
  switch(tmp){
    // Class 1
  case 0:
  case 1:
  case 2:
  case 3:
  case 4:
  case 5:
  case 6:
  case 7:
  case 8:
    {
      unsigned int dst,opc,reg,ofs;
      /*
        Bits  What
        15:14 Destination Field
        13:09 Opcode
        08:06 Register Field
        05:00 Offset Field
      */
      
      dst=ldb(MacroInstruction,2,14);
      opc=ldb(MacroInstruction,5,9);
      reg=ldb(MacroInstruction,3,6);
      ofs=ldb(MacroInstruction,6,0);
      
      logmsgf("MIR: MAIN-OP %X, D-%s, R-%s, OFS %X\n",opc,macro_dst_str[dst],macro_reg_str[reg],ofs);            
    }
    break;

    // Class 2
  case 9:
  case 10:
  case 11:
  case 14:
  case 25:
  case 26:
  case 27:
  case 30:
    logmsgf("MIR: Class 2\n");
    break;
    
    // Class 3
  case 12:
  case 28:
    logmsgf("MIR: Class 3\n");
    break;
    
    // Class 4
  case 13:
  case 29:
    logmsgf("MIR: Class 4\n");
    break;
    
    // Class 5
  case 16:
    logmsgf("MIR: Class 5\n");
    break;

    // Others
  default:
    logmsgf("MIR: Unknown/Unused %d\n",tmp);
  }  
}

void disassemble_IR(){
  char disasm[128]; // Disassembled instruction
  disassemble(loc_ctr_cnt, Iregister, disasm);
  logmsgf("[%d] %s\n",loc_ctr_cnt, disasm);
}

void debug_disassemble_IR(){
  char disasm[128]; // Disassembled instruction
  disassemble(loc_ctr_cnt, Iregister, disasm);
  logmsgf(":: [%.6d A=%.8X M=%.8X O=%.8X] %s\n",loc_ctr_cnt,Abus,Mbus,Obus,disasm);
}

#ifdef DET_TRACELOG
static inline void dei_disasm(){
  // Trace DEI for the log. Tracing DEI to the console usually crashes the console due to spam.
  if(loc_ctr_cnt == 1504){ // At the top of the DEI jump table
    switch(Mmemory[41]){ // OPCODE-LO
      
    case 0x00:  // CONTROL-OPS
      switch(Mmemory[42]){
      case 0: // EXIT
        logmsgf("DEBUG: DEI: L-%X : EXIT\n",Mmemory[61]);
        break;
      case 2: // RETURN
        logmsgf("DEBUG: DEI: L-%X : RETURN\n",Mmemory[61]);
        break;
      case 3: // WAIT
        logmsgf("DEBUG: DEI: L-%X : WAIT D-%lX\n",Mmemory[61],Amemory[Mmemory[56]]); 
        break;
      default:
        logmsgf("DEI: UNKNOWN DEI-CONTROL-OP, OPCODE L-%lX H-%lX\n",Mmemory[41],Mmemory[42]);
        cpu_die_rq=1;
      }
      break;
      
    case 0x01: // JUMP-OPS
      switch(Mmemory[42]){
      case 0: // GOTO
        logmsgf("DEBUG: DEI: L-%X : GOTO\n",Mmemory[61]);
        break;
      case 1: // JIFZ
        logmsgf("DEBUG: DEI: L-%X : JIFZ D-%lX\n",Mmemory[61],Amemory[Mmemory[56]]); 
        break;
      case 2: // JNBE
        logmsgf("DEBUG: DEI: L-%X : JNBE\n",Mmemory[61]);
        break;
      case 3: // ILOOP
        logmsgf("DEBUG: DEI: L-%X : ILOOP\n",Mmemory[61]);
        break;
        
      default:
        logmsgf("DEI: UNKNOWN DEI-JUMP-OP, OPCODE L-%lX H-%lX\n",Mmemory[41],Mmemory[42]);
          cpu_die_rq=1;
      }
      break;
      
    case 0x02: // ARITHMETIC OPS
      switch(Mmemory[42]){
      case 0: // PLUS
        logmsgf("DEBUG: DEI: L-%X : PLUS D1-%lX D2-%lX\n",Mmemory[61],Amemory[Mmemory[56]],Amemory[Mmemory[56]-1]);  
        break;
      case 1: // MINUS
        logmsgf("DEBUG: DEI: L-%X : MINUS D1-%lX D2-%lX\n",Mmemory[61],Amemory[Mmemory[56]],Amemory[Mmemory[56]-1]);  
        break;
      case 2: // TIMES
        logmsgf("DEBUG: DEI: L-%X : TIMES D1-%lX D2-%lX\n",Mmemory[61],Amemory[Mmemory[56]],Amemory[Mmemory[56]-1]);  
        break;
      case 3: // DIV
        logmsgf("DEBUG: DEI: L-%X : DIV D1-%lX D2-%lX\n",Mmemory[61],Amemory[Mmemory[56]],Amemory[Mmemory[56]-1]);  
        break;
      default:
        logmsgf("DEI: UNKNOWN DEI-ARITHMETIC-OP, OPCODE L-%lX H-%lX\n",Mmemory[41],Mmemory[42]);
        cpu_die_rq=1;
      }
      break;
      
    case 0x03: // BOOLEAN OPS
      switch(Mmemory[42]){
      case 0: // NOT
        logmsgf("DEBUG: DEI: L-%X : NOT D-%lX\n",Mmemory[61],Amemory[Mmemory[56]]); 
        break;
      case 1: // AND
        logmsgf("DEBUG: DEI: L-%X : AND D1-%lX D2-%lX\n",Mmemory[61],Amemory[Mmemory[56]],Amemory[Mmemory[56]-1]); 
        break;          
      case 2: // OR
        logmsgf("DEBUG: DEI: L-%X : OR D1-%lX D2-%lX\n",Mmemory[61],Amemory[Mmemory[56]],Amemory[Mmemory[56]-1]); 
        break;          
      default:
        logmsgf("DEI: UNKNOWN DEI-BOOLEAN-OP, OPCODE L-%lX H-%lX\n",Mmemory[41],Mmemory[42]);
        cpu_die_rq=1;
      }
      break;

    case 0x04: // SHIFT OPS
      switch(Mmemory[42]){
      case 0: // ROTATE
        logmsgf("DEBUG: DEI: L-%X : ROTATE COUNT-%lX DATA-%lX\n",Mmemory[61],Amemory[Mmemory[56]],Amemory[Mmemory[56]-1]); 
        break;          
      case 1: // SHIFT
        logmsgf("DEBUG: DEI: L-%X : SHIFT COUNT-%lX DATA-%lX\n",Mmemory[61],Amemory[Mmemory[56]],Amemory[Mmemory[56]-1]); 
        break;          
      default:
        logmsgf("DEI: UNKNOWN DEI-SHIFT-OP, OPCODE L-%lX H-%lX\n",Mmemory[41],Mmemory[42]);
        cpu_die_rq=1;
      }
      break;
      
    case 0x05: // COMPARISON OPS
      switch(Mmemory[42]){
      case 0: // EQUAL
        logmsgf("DEBUG: DEI: L-%X : EQUAL D1-%lX D2-%lX\n",Mmemory[61],Amemory[Mmemory[56]],Amemory[Mmemory[56]-1]);  
        break;
      case 1: // LESS
        logmsgf("DEBUG: DEI: L-%X : LESS D1-%lX D2-%lX\n",Mmemory[61],Amemory[Mmemory[56]],Amemory[Mmemory[56]-1]);  
        break;
      default:
        logmsgf("DEI: UNKNOWN DEI-COMPARISON-OP, OPCODE L-%lX H-%lX\n",Mmemory[41],Mmemory[42]);
        cpu_die_rq=1;
      }
      break;
      
    case 0x06: // NOVER OPS
      switch(Mmemory[42]){
      case 0: // NOVER
        logmsgf("DEBUG: DEI: L-%X : NOVER INDEX-%lX\n",Mmemory[61],Amemory[Mmemory[56]]); 
        break;
      case 1: // SWAP
        logmsgf("DEBUG: DEI: L-%X : SWAP\n",Mmemory[61]);          
        break;
      default:
        logmsgf("DEI: UNKNOWN DEI-NOVER-OP, OPCODE L-%lX H-%lX\n",Mmemory[41],Mmemory[42]);
        cpu_die_rq=1;
      }
      break;

    case 0x07: // PICK OPS
      switch(Mmemory[42]){
      case 0: // PICK
        logmsgf("DEBUG: DEI: L-%X : PICK INDEX-%lX\n",Mmemory[61],Amemory[Mmemory[56]]); 
        break;          
      case 1: // DUP
        logmsgf("DEBUG: DEI: L-%X : DUP\n",Mmemory[61]);          
        break;
      case 2: // DROP
        logmsgf("DEBUG: DEI: L-%X : DROP\n",Mmemory[61]);          
        break;
      default:
        logmsgf("DEI: UNKNOWN DEI-PICK-OP, OPCODE L-%lX H-%lX\n",Mmemory[41],Mmemory[42]);
        cpu_die_rq=1;
      }
      break;

    case 0x08: // VAL
      // VAL - Store data bytes (as a word) on the stack.
      // OP-HI has the number of bytes to store.
      logmsgf("DEBUG: DEI: L-%X : VAL (Size %d)\n",Mmemory[61],Mmemory[42]);
      break;

    case 0x09: // FROMR
      // Pop the top item from R-STACK, push to D-STACK
      logmsgf("DEBUG: DEI: L-%X : FROMR\n",Mmemory[61]);          
      break;

    case 0x0A: // TOR
      logmsgf("DEBUG: DEI: L-%X : TOR D-%lX\n",Mmemory[61],Amemory[Mmemory[56]]); 
      break;

    case 0x0B: // RGET
      switch(Mmemory[42]){
      case 1:
        logmsgf("DEBUG: DEI: L-%X : RGET-INDEX\n",Mmemory[61]);
        break;
      case 2:
        logmsgf("DEBUG: DEI: L-%X : RGET-LIMIT\n",Mmemory[61]);
        break;
      default:
        logmsgf("DEI: UNKNOWN DEI-RGET-OP, OPCODE L-%lX H-%lX\n",Mmemory[41],Mmemory[42]);
        cpu_die_rq=1;
      }
      break;

    case 0x0C: // OUTPUT OPS
      switch(Mmemory[42]){        
      case 0: // EMIT
        logmsgf("DEBUG: DEI: L-%X : EMIT CHAR-%lX (%c)\n",Mmemory[61],Amemory[Mmemory[56]],Amemory[Mmemory[56]]); 
        break;
      case 1: // CLRSCR
        logmsgf("DEBUG: DEI: L-%X : CLRSCR\n",Mmemory[61]);
        break;
      default:
        logmsgf("DEI: UNKNOWN DEI-EMIT-OP, OPCODE L-%lX H-%lX\n",Mmemory[41],Mmemory[42]);
        cpu_die_rq=1;
      }
      break;
        
    case 0x10: // DPUT
      logmsgf("DEBUG: DEI: L-%X : DPUT ADDR-%lX DATA-%lX\n",Mmemory[61],Amemory[Mmemory[56]],Amemory[Mmemory[56]-1]); 
      break;          

    case 0x11: // NPUT
      logmsgf("DEBUG: DEI: L-%X : NPUT ADDR-%lX DATA-%lX\n",Mmemory[61],Amemory[Mmemory[56]],Amemory[Mmemory[56]-1]); 
      break;          

    case 0x12: // BPUT
      logmsgf("DEBUG: DEI: L-%X : BPUT ADDR-%lX DATA-%lX\n",Mmemory[61],Amemory[Mmemory[56]],Amemory[Mmemory[56]-1]); 
      break;          
        
    case 0x14: // DGET
      logmsgf("DEBUG: DEI: L-%X : DGET ADDR-%lX\n",Mmemory[61],Amemory[Mmemory[56]]); 
      break;               

    case 0x15: // NGET
      logmsgf("DEBUG: DEI: L-%X : NGET ADDR-%lX SIZE=%lX\n",Mmemory[61],Amemory[Mmemory[56]],Mmemory[42]); 
      break;

    case 0x16: // BGET
      logmsgf("DEBUG: DEI: L-%X : BGET ADDR-%lX SIZE=%lX\n",Mmemory[61],Amemory[Mmemory[56]],Mmemory[42]); 
      break;       

    case 0x17: // IGET
      logmsgf("DEBUG: DEI: L-%X : IGET ADDR-%lX\n",Mmemory[61],Amemory[Mmemory[56]]); 
      break;               

    case 0x18: // SETMEM
      logmsgf("DEBUG: DEI: L-%X : SETMEM START-%lX DATA-%lX COUNT-%lX\n",
              Mmemory[61],Amemory[Mmemory[56]],Amemory[Mmemory[56]-1],Amemory[Mmemory[56]-2]);         
      break;

    case 0x19: // CHECKMEM
      logmsgf("DEBUG: DEI: L-%X : CHECKMEM START-%lX DATA-%lX COUNT-%lX\n",
              Mmemory[61],Amemory[Mmemory[56]],Amemory[Mmemory[56]-1],Amemory[Mmemory[56]-2]);         
      break;

    case 0x1A: // WALKMEM
      logmsgf("DEBUG: DEI: L-%X : WALKMEM START-%lX DATA-%lX COUNT-%lX CHECK-%lX\n",
              Mmemory[61],Amemory[Mmemory[56]],Amemory[Mmemory[56]-1],Amemory[Mmemory[56]-2],Amemory[Mmemory[56]-3]);         
      break;

    case 0x1E: // EREX
      logmsgf("DEBUG: DEI: L-%X : EREX\n",Mmemory[61]);          
      break;
                
    default:
      logmsgf("DEI: UNKNOWN DEI-LO, OPCODE L-%lX H-%lX\n",Mmemory[41],Mmemory[42]);
      cpu_die_rq=1;
    }
  }
}
#endif

void raven_trap(){
  // Cause a trap to zero
  trap_test = 2;
  trap_type = 1;
  return;
}

void raven_dump(){
  // Dump CRAM to disk
  FILE *output;
  unsigned long addr=0;
  char buffer[1024];
  logmsgf("Dumping WCS to WCS.DUMP ...\n");
  output = fopen("WCS.DUMP","w+");
  if(!output){
    logmsgf("Can't open WCS.DUMP\n");
    return;
  }
  while(addr<16384){
    disassemble(addr,WCS[addr],buffer);
    fprintf(output,"[%.5ld] (%.16LX) %s\n",addr,WCS[addr],buffer);
    addr++;
  }
  fclose(output);
  addr=0;
  logmsgf("Dumping A-Memory to AMEM.DUMP...\n");
  output = fopen("AMEM.DUMP","w+");
  if(!output){
    logmsgf("Can't open AMEM.DUMP\n");
    return;
  }
  while(addr<1024){
    fprintf(output,"[A-%.5ld] 0x%.8X\n",addr,Amemory[addr]);
    addr++;
  }
  fclose(output);
  addr=0;
  logmsgf("Dumping M-Memory to MMEM.DUMP...\n");
  output = fopen("MMEM.DUMP","w+");
  if(!output){
    logmsgf("Can't open MMEM.DUMP\n");
    return;
  }
  while(addr<64){
    fprintf(output,"[M-%.5ld] 0x%.8X\n",addr,Mmemory[addr]);
    addr++;
  }
  fclose(output);

  addr=0;
  logmsgf("Dumping Physical-Memory to PMEM.DUMP...\n");
  output = fopen("PMEM.DUMP","w+");
  if(!output){
    logmsgf("Can't open PMEM.DUMP\n");
    return;
  }
  while(addr<0x800000){
    fprintf(output,"[RAM-%.6lX] %.2X%.2X%.2X%.2X\n",addr,MEM_RAM[addr+3],MEM_RAM[addr+2],MEM_RAM[addr+1],MEM_RAM[addr+0]);
    addr += 0x04;
  }
  fclose(output);

  addr=0;
  logmsgf("Dumping CPU registers and status information to CPU.DUMP\n");
  output = fopen("CPU.DUMP","w+");
  if(!output){
    logmsgf("Can't open CPU.DUMP\n");
    return;
  }
  //  fprintf(output,"[CPU] LC = 0x%lX IR = 0x%LX\n",loc_ctr_reg,Iregister);

  fprintf(output,"[CPU] PDL Index = 0x%.8X, PDL Pointer = 0x%.8X\n",pdl_index_reg,pdl_ptr_reg);
  addr=0;
  while(addr < 1024){
    fprintf(output,"[PDL-RAM %.5ld] 0x%.8X\n",addr,PDLmemory[addr]);
    addr++;
  }
  
  fprintf(output,"[CPU] uPCS = %d\n",uPCS_ptr_reg);
  addr=0;
  while(addr < 64){
    fprintf(output,"[uPCS-%.3ld] 0x%.8X\n",addr,uPCS_stack[addr]);
    addr++;
  }

  fprintf(output,"[CPU] DISPATCH RAM FOLLOWS\n");
  addr=0;
  while(addr < 4096){
    fprintf(output,"[D-RAM %.5ld] 0x%.8X\n",addr,dispatch_ram[addr]);
    addr++;
  }

  fprintf(output,"[CPU] TAG RAM FOLLOWS\n");
  addr=0;
  while(addr < 16){
    fprintf(output,"[T-RAM %.3ld] 0x%.8X\n",addr,Tmemory[addr]);
    addr++;
  }

  fprintf(output,"[CPU] VM MAP DATA FOLLOWS\n");
  addr=0;
  while(addr < 4096){
    fprintf(output,"[VM-MAP %.5ld] LV1 MAP 0x%.8X LV2 CTL = 0x%.8X LV2 ADR = 0x%.8X\n",
            addr,vm_lv1_map[addr],vm_lv2_ctl[addr],vm_lv2_adr[addr]);
    addr++;
  }

  addr=0;
  logmsgf("Dumping screenshot to SCREENSHOT.BMP...\n");
  output = fopen("SCREENSHOT.BMP","w+");
  if(!output){
    logmsgf("Can't open SCREENSHOT.BMP\n");
    return;
  }
  {
    // BMP header
    const char bmpheader[14] = {
        0x42,0x4D,  // Constant 19778, 'BM'
        0x3E,0,2,0, // FILE SIZE - 62+131072 (131134)
        0,0,        // Must be zero
        0,0,        // Must be zero
        62,0,0,0  // Offset to start of data (Constant 0x436)
        };
    const char bmpinfohdr[40] = {
      0x28,0,0,0, // Constant 40, size of info header
      0,4,0,0,    // Width (1024 px)
      0,4,0,0,    // Height (1024 px)
      1,0,        // Number of bitplanes (mono)
      1,0,        // Bits per pixel (mono)
      0,0,0,0,    // Compression (none)
      0,0,0,0,    // Size of image data (0 for not compressed)
      0,0,0,0,    // pels per meter etc
      0,0,0,0,
      0,0,0,0,    // Colors used (mono)
      0,0,0,0     // Important colors (mono)
    };
    // Now at byte 54
    const char rgbinfo[8] = {
      0,0,0,0,       // Black
      255,255,255,0  // White
    };
    // Now at byte 62

    int x=1024,y=128;
    // Write out header and such
    fwrite(bmpheader,14,1,output);
    fwrite(bmpinfohdr,40,1,output);
    fwrite(rgbinfo,8,1,output);
    
    // Write pixels    
    while(x > 0){
      x--;
      y=0; // 128 bytes in a row
      while(y < 128){
        unsigned char dto;
        dto=VRAM[(x*128)+y];             
        // Reverse bits
        dto = ((dto >>  1) & 0x55) | ((dto <<  1) & 0xaa);
        dto = ((dto >>  2) & 0x33) | ((dto <<  2) & 0xcc);
        dto = ((dto >>  4) & 0x0f) | ((dto <<  4) & 0xf0);
        fwrite(&dto,1,1,output);
        y++;
      }
    }
    
  }
  fclose(output);
    
  logmsgf("Dump completed.\n");
}

// ALU operation M-A (M-A-1) 
// Used by BYTE,JUMP
static inline void alu_sub_stub(){
  ALU_Result = Mbus - Abus - (MInst_ALU_Carry_In ? 0 : 1);
  // FIXNUM Overflow Check  
  if((((Mbus^Abus)&(Mbus^ALU_Result))&0x01000000)==0x01000000){
    ALU_Fixnum_Oflow=1;
  }
}

static inline void alu_cleanup_result(){
  // Reduce carry-out to a flag without use of boolean type
  if((ALU_Result&0xFFFFFFFF00000000LL) != 0){ ALU_Carry_Out = 1; }	
  // Clean Output (ALU is 32 bits wide)
  ALU_Result &= 0xFFFFFFFF;
  
  // Make the result
  Obus = ALU_Result;
}

static inline void fix_alu_carry_out()
{
  int cout = ((ALU_Result < Mbus ? 1 : 0) +
              ((Mbus>>31)&1) + ((Abus>>31)&1)) & 1;
  ALU_Result &= 0xffffffff;
  if (cout) 
    ALU_Result |= 0x100000000LL; // Arrange for carry-out
}

static inline void operate_shifter(){
  uint32 x=0;
  uint32 Mask = 0;
  int left_mask_index; 
  int right_mask_index;
  uint32 R = Mbus;
 
  // Shifter run
  // Rotate R
  if(MInst_Source_Rotate != 0){
    if(MInst_Rotation_Directn == 0){
      R = left_rotate(R,MInst_Rotation_Count);
    }else{
      R = right_rotate(R,MInst_Rotation_Count);
    }
  }
  // Create mask
  // Get Right mask
  if(MInst_Mask_Rotate != 0){
    if(MInst_Rotation_Directn == 0){
      right_mask_index = MInst_Rotation_Count;
    }else{
      right_mask_index = (040-MInst_Rotation_Count);
    }
  }else{
    right_mask_index = 0;
  }
  right_mask_index &= 037;
  // Get Left mask
  left_mask_index = (right_mask_index + (MInst_Rotation_Len-1)) % 32;  
  // Final mask
  Mask = shift_left_mask[left_mask_index]&shift_right_mask[right_mask_index];

  // Merge A with R, using bits from R if the mask bit is 1
  Obus = 0;
  // Bottom cycle
  if((Mask&0x01) == 0x01){
    // Bit set, pull this one from R
    Obus |= (R&0x01);
  }else{
    // Bit clear, pull this one from A
    Obus |= (Abus&0x01);
  }	    
  // Continue
  x=0x01;
  while(x != 0x80000000){
    x = x << 1;
    if((Mask&x) == x){
      // Bit set, pull this one from R
      Obus |= (R&x);
    }else{
      // Bit clear, pull this one from A
      Obus |= (Abus&x);
    }	    
  }
  // SAVE FOR LATER ABUSE
  Rbus = R;
#if 0
  if (bustrace)
    logmsgf("operate_shifter: done, "
            "R %lx, Abus %lx, Mask %lx, Obus %lx, Rbus %lx\n",
            R, Abus, Mask, Obus, Rbus);
#endif
}

// *** INTERRUPT CONTROL ***
static inline void pi_system_check(){
  int IMask=0;
  int PI_Level=15;
  int PI_RQ = 0;
  
  Highest_PI = 0;
  Active_PI = 0;

  // No interrupts active.
  MCregister &= 0xFFF0FFFF;
  
  // Scan PI status
  IMask = 0x8000; // 1 << PI_Level;
  while(PI_Level > 1){    
    if((PI_Sources&IMask) != 0){
      //      logmsgf("PISys: Level %d active.\n",PI_Level);
      Highest_PI = PI_Level;
      PI_RQ = 1;
    }
    PI_Level--;
    IMask >>= 1;
  }
  if(PI_RQ != 0){
    if((MCregister&MCR_Int_Enable) != 0){
      //    logmsgf("Signalling active PI.\n");  
      Active_PI = 1; // Signal that PI needs handled
    }
    // Update MCR
    MCregister |= ((Highest_PI&0xF) << 16);
  }
}

// *** RAVEN'S NUBUS SLAVE ***
// *** RAVEN'S NUBUS INTERFACE IS SLAVE-ONLY! ***
void raven_nubus_io(){
  uint32 NUbus_Addr = NUbus_Address&0xFFFFFF; // ldb(NUbus_Address,24,0);  

  // C00000 == Flag Register
  // D00000 == Configuration Register
  // E00000 == Post PI-0 // POWER FAILURE 
  // E00004 == Post PI-1
  // E00008 == Post PI-2
  // E0000C == Post PI-3
  // E00010 == Post PI-4
  // E00014 == Post PI-5
  // and so on until...
  // E0003C == Post PI-15 (lowest priority)
  // FFFC00
  // thru   == Configuration PROM
  // FFFFFF 

  if((NUbus_Addr&0xFFFF00) == 0xE00000){
    // PI
    if(NUbus_Request&0x1){ // Write
      int PI_Nr = (NUbus_Addr&0x3C)>>2; // ldb(NUbus_Addr,4,2);
      uint32 PI_Mask = (0x1 << PI_Nr);

      if(NUbus_Data == 0){
        PI_Sources &= (~PI_Mask);
      }else{
        PI_Sources |= PI_Mask;
      }
      NUbus_acknowledge=1;
    }else{
      logmsgf("RAVEN: Reading interrupt area?\n");
      cpu_die_rq=1;
    }

    pi_system_check();      
    return;
  }

  // Configuration Register
  if(NUbus_Addr == 0xD00000){
    if(NUbus_Request == VM_BYTE_WRITE){
      logmsgf("RAVEN: Configuration Register Write, Data = 0x%lX\n",NUbus_Data);
      // Bit 01 = RESET
      // Bit 02 = Light Error LED
      // Bit 04 = ??
      if(NUbus_Data&0x01){
	// RESET
	logmsg("RESET NOT IMPLEMENTED\n");
	cpu_die_rq=1;
      }
      raven_config_reg = NUbus_Data&0xFF;
      NUbus_acknowledge=1;
      return;
    }
    if(NUbus_Request == VM_BYTE_READ){
      NUbus_Data = raven_config_reg;
      NUbus_acknowledge=1;
      return;
    }
  }

  // Flag Register
  if(NUbus_Addr == 0xC00000){
    // Flag register bits:
    // 0x4 = SUBSYSTEM TEST FAILED (Always 1 for CPU)
    // 0x2 = BOARD TEST FAILED     (1 = Passed Test, 0 = Test Failed or Still Running)
    // 0x1 = SELF-TEST RUNNING     (0 = Test Completed)
    if(NUbus_Request == VM_READ || NUbus_Request == VM_BYTE_READ){
      NUbus_Data = 0;
      if((MCregister&MCR_Subsys_Flag)==0){
	NUbus_Data |= 0x04;
      }
      if((MCregister&MCR_Test_Fail_Flag)==0){
	NUbus_Data |= 0x02;
      }
      if((MCregister&MCR_Self_Test_Flag)==0){
	NUbus_Data |= 0x01;
      }
    }
    NUbus_acknowledge=1;
    return;
  }

  if((NUbus_Request == VM_READ || NUbus_Request == VM_BYTE_READ) && NUbus_Addr >= 0xFFFC00){
    // For Configuration ROM addresses
    // Use ROM instead!
    NUbus_Data = raven_rom[((NUbus_Addr&0x3FF)>>2)];
    NUbus_acknowledge=1;
    return;
  }

  logmsgf("RAVEN: Unknown NUbus IO-(%ld): %lX\n",NUbus_Request,NUbus_Addr);
}

// **** LOCAL BUS INTERFACE ****

/* Localbus IO pulse */
static inline void lcbus_io_pulse(){
  if(Memory_Busy > 0 && (LCbus_Request&0x01)==0 && LCbus_acknowledge > 0){
    // Since this is a read request, reload MD here.
    // No NUbus access is on the local bus so that no longer needs checked.
    MDregister = LCbus_Data;      
  }
  // Then...
#ifdef TRACELOG
  if(do_inst_trace){
    if(Memory_Busy == 1){
      if(LCbus_acknowledge > 0){
        logmsgf("DEBUG: LCbus Read Completed, returned 0x%lX\n",LCbus_Data);
      }else{
        logmsgf("DEBUG: LCbus Timeout for %lX\n",LCbus_Address);
      }
    }
  }
#endif  
  // Anyone take this request?
  if(Memory_Busy == 1 && LCbus_acknowledge == 0){
    // No.
    LCbus_error = 1;
    // Arm NXM trap
    trap_test = 4;
    trap_type = 0;
  }
}

// Localbus IO stall
static inline void lcbus_io_stall(){
  // If MEMORY-BUSY, work it..
  while(Memory_Busy > 0){
    if(Nubus_Busy){
      Nubus_Busy--;
      nubus_io_pulse();
      nupi_clock_pulse();
#ifdef ENABLE_ENET
      enet_clock_pulse();    
#endif
    }
    // Preserve timing
    sib_clock_pulse();
    Memory_Busy--;
    lcbus_io_pulse();
    // Are we supposed to trap?
    if(trap_test > 0){
#ifdef TRACELOG
      logmsgf("DEBUG: MEMORY WAIT W/ TRAP PENDING - FORCING IMMEDIATE TRAP\n");
#endif
      trap_test = 2; // TRAP IMMEDIATE
    }
  }  
}

// Make an IO request via the Local Bus
static inline void lcbus_io_request(int access, uint32 address, uint32 data, int owner){

  // Wait for the bus
  lcbus_io_stall();

  Page_Fault = 0; // Reset flags
  LCbus_error = 0;
  LCbus_acknowledge = 0;

  if(!(MCregister&MCR_Memory_Cycle_Enable)){
    Memory_Busy = 0;
    return;
  }

#ifdef TRACELOG
  if(do_inst_trace){
    logmsgf("DEBUG: LCBUS-IO: OP %d PA %lX, Data %lX\n",
            access,address,data);
  }
#endif

  // Only 3 is a legal value. All others fail the CPU selftest, except 4 which breaks the
  // slot-empty testing.

  Memory_Busy = 3; // 2 INSTRUCTIONS PASS BEFORE COMPLETION (counting this one)
  LCbus_Address = address;
  LCbus_Request = access;
  LCbus_Data = data;
  
  // Immediately trigger first pulse, straight to mem8
  mem8_lcbus_io();

  // If the mem8 responded, drop the data in MD
  if((LCbus_Request&0x01) == 0 && LCbus_acknowledge > 0){
    MDregister = LCbus_Data;      
  }
  
  return;
}

// *** MMU ***
uint32 VM_resolve_address(int access)
{
  int vpage_block, vpage_offset, page_offset;
  uint32 lvl_1_map_data;
  int lv2_index;
  uint32 lv2_control;
  uint32 result;
  int unmapped, awrite, byteio;
  int interlocked, force_request;
  int m1_valid, m2_access, m2_forceable, m2_writable;

  vpage_block =  (VMAregister & 0x01FFE000) >> 13; // ldb(VMAregister,12,13);
  vpage_offset = (VMAregister & 0x00001F00) >> 8;  // ldb(VMAregister,5,8);
  page_offset =   VMAregister & 0x000000FF;        // ldb(VMAregister,8,0);

  lvl_1_map_data = vm_lv1_map[vpage_block];
  lv2_index = ((lvl_1_map_data & 0x7F) << 5) | vpage_offset;
  lv2_control = vm_lv2_ctl[lv2_index];

  // Update cached-gcv
  cached_gcv = (lv2_control & 0x1800) >> 11;

  unmapped = access & 0x100;
  awrite = access & 1;
  byteio = access & 2;

  m1_valid = lvl_1_map_data & 0x800;
  m2_forceable = lv2_control & 0x400;
  m2_access = lv2_control & 0x200;
  m2_writable = lv2_control & 0x100;
  force_request = MCregister & MCR_Forced_Access_Request;
  interlocked = MCregister & MCR_Interlocked_Memory_Control;

  Page_Fault = 0;
  result = VMAregister;

  if (unmapped && !awrite) {
    /* unmapped read */

    /* update lv1 map bits */
    lvl_1_map_data &= 0x3fff;
    lvl_1_map_data |= 0x8000 | (force_request ? 0 : 0x4000);

    vm_lv1_map[vpage_block] = lvl_1_map_data;
cached_lv1 = lvl_1_map_data;

    /* update lv2 control bits */
    lv2_control &= 0x1fff;

    if (byteio)
// 0x4000 : 0x6000
      lv2_control |= (interlocked ? 2 : 3) << 13;
    else
// 0xc000 : 0xe000
      lv2_control |= (interlocked ? 6 : 7) << 13;

    vm_lv2_ctl[lv2_index] = lv2_control;

    if (VMAregister < 0xf0000000) logmsgf("unmapped-read @%x %x\n", VMAregister, lv2_control);
    return result;
  }

  if (unmapped && awrite) {
    /* unmapped write */

    /* update lv1 map bits */
    lvl_1_map_data &= 0x3fff;
    lvl_1_map_data |= 0x8000 | (force_request ? 0 : 0x4000);

    vm_lv1_map[vpage_block] = lvl_1_map_data;
cached_lv1 = lvl_1_map_data;

    /* update lv2 control bits */
    lv2_control &= 0x1fff;

    if (byteio)
// 0x8000 : 0xa000
      lv2_control |= (interlocked ? 0 : 1) << 13;
    else
// 0x0000 : 0x2000
      lv2_control |= (interlocked ? 4 : 5) << 13;

    vm_lv2_ctl[lv2_index] = lv2_control;

if (VMAregister < 0xf0000000) logmsgf("unmapped-write @%x %x\n", VMAregister, lv2_control);
    return result;
  }

  /* map virtual to physical */

  result =
    (((vm_lv2_adr[lv2_index] & 0x3FFFFF) << 8) |
     (page_offset & 0xFF)) << 2;

  if (!awrite) {
    /* mapped read */
    if (!m1_valid)
      Page_Fault = 1;

    if (!m2_access)
      Page_Fault = 1;

    /* update lv1 map bits */
    lvl_1_map_data &= 0x0fff;
    lvl_1_map_data |= 0x4000;

    if (Page_Fault)
      lvl_1_map_data |= 0x1000;

    vm_lv1_map[vpage_block] = lvl_1_map_data;
cached_lv1 = lvl_1_map_data;

#ifdef TRACELOG
 if(do_inst_trace){
   logmsgf("DEBUG: MMU: MAPPED-ACCESS, VMA 0x%lX, PF = %d, L1-IDX = 0x%lX, L2-IDX = 0x%lX\n",
           VMAregister,Page_Fault,vpage_block,lv2_index);
 }
#endif

    if (Page_Fault)
      return 0;

    return result;
  }

  /* mapped write */
  if (!m1_valid)
    Page_Fault = 1;

  if (!m2_access)
    Page_Fault = 1;

  if (m2_writable || (m2_forceable && force_request))
    ;
  else
    Page_Fault = 1;

  /* update lv1 map bits */
  lvl_1_map_data &= 0x1fff;

  if (m2_forceable && force_request)
    ;
  else
    lvl_1_map_data |= 0x4000;

  if (Page_Fault)
    lvl_1_map_data |= 0x2000;

  vm_lv1_map[vpage_block] = lvl_1_map_data;
cached_lv1 = lvl_1_map_data;

  if (Page_Fault)
    return 0;

  return result;
}

static inline void VM_unmapped_io(int access){
  // Unmapped accesses update the VM system.
  VM_resolve_address(access|0x100); // Access in unmapped mode
  // Inhibit page-faults in unmapped mode
  Page_Fault = 0;
  // Continue

  lcbus_io_request(access,VMAregister,MDregister,NUBUS_ID_CPU);
}

// Update status display
void raven_dispupdate(){
  char LEDstring[10];
  unsigned int MacroInstruction;
  int i, l;
  
  if((LCregister&0x01) == 0x01){
    MacroInstruction = MIbuffer & 0xffff;
  }else{
    MacroInstruction = ((MIbuffer&0xFFFF0000)>>16);
  }

  // Draw clock information
  // Obsoleted

  if (cpu_w == 0) {
	  printf("UPC %.5d | IR %.16LX ", loc_ctr_reg, Iregister);
	  printf("INST:%s AJMP:%.2o MSRC:%.3o ASRC:%.4o ",
		 opcode[opword], MInst_Abbrv_Jump, MInst_M_Source_Addr,
		 MInst_A_Source_Addr);
	  printf("OBUS:%.8X uPC-SP:%.2d ", Obus, uPCS_ptr_reg);
	  printf("QR:0x%.8X MCR:0x%.8X PDL-PTR:0x%.8X", Qregister, MCregister,pdl_ptr_reg);
	  printf("MD:0x%.8X VMA:0x%.8X", MDregister, VMAregister);

	  printf("\n");
	  return;
  }

  // Draw rate
  mvwprintw(cpu_w,1,2,"UCYCLE RATE = %Ld IPS (%d)\n",ccount,delay_tune);
  // Adjust delay
  if(ccount < 7500000){
   if(delay_tune > 0){
      delay_tune -= 1;
    }
  }else{
    delay_tune += 1;
  }
  ccount=0;

  // Draw LC and IR
  mvwprintw(cpu_w,2,2,"UPC %.5d | IR %.16LX",loc_ctr_reg,Iregister);
  // Parse instruction in IR if we can
  MInst_Parity     = (Iregister&0x4000000000000LL) ? 1 : 0;  // ((Iregister >> 50)&0x1);  //     = ldb(Iregister,1,50);
  MInst_Halt       = (int)(Iregister&0x2000000000000LL) ? 1 : 0;  // ((Iregister >> 49)&0x01); //     = ldb(Iregister,1,49);

  // GLOBAL FIELDS AND REGISTERS
  mvwprintw(cpu_w,3,2,"INST:%s AJMP:%.2o MSRC:%.3o ASRC:%.4o",
            opcode[opword],MInst_Abbrv_Jump,MInst_M_Source_Addr,
	    MInst_A_Source_Addr);
  mvwprintw(cpu_w,3,40,"P:%s H:%s uPC-SP:%.2d",
	    MInst_Parity ? "Y" : "N",
            MInst_Halt ? "Y" : "N",
            uPCS_ptr_reg);
  mvwprintw(cpu_w,4,2,"MBUS:0x%.8X ABUS:0x%.8X OBUS:%.8X",Mbus,Abus,Obus);

  mvwprintw(cpu_w,5,2,"QR:0x%.8X MCR:0x%.8X LC:0x%.8X MIR:0x%.4lX",
            Qregister,MCregister,LCregister,MacroInstruction);
  mvwprintw(cpu_w,6,2,"MD:0x%.8X VMA:0x%.8X PDL-P: 0x%.8X PDL-I: 0x%.8X",
            MDregister,VMAregister,pdl_ptr_reg,pdl_index_reg);

  l = 8;
  for (i = 0; i < 32; i+= 4) {
    mvwprintw(cpu_w,l++,2,"M-%02d:0x%.8X M-%02d:0x%.8X M-%02d:0x%.8X M-%02d:0x%.8X ",
              i+0, Mmemory[i+0], i+1, Mmemory[i+1], i+2, Mmemory[i+2], i+3, Mmemory[i+3]);
  }

  for (i = 55; i < 58; i+= 4) {
    mvwprintw(cpu_w,l++,2,"M-%02d:0x%.8X M-%02d:0x%.8X M-%02d:0x%.8X M-%02d:0x%.8X ",
              i+0, Mmemory[i+0], i+1, Mmemory[i+1], i+2, Mmemory[i+2], i+3, Mmemory[i+3]);
  }

  l++;
  for (i = 220; i < 248; i+= 4) {
    mvwprintw(cpu_w,l++,2,"M-%03d:0x%.8X M-%03d:0x%.8X M-%03d:0x%.8X M-%03d:0x%.8X ",
              i+0, Mmemory[i+0], i+1, Mmemory[i+1], i+2, Mmemory[i+2], i+3, Mmemory[i+3]);
  }

  /*
  else{
    mvwprintw(cpu_w,3,2,"INST:XXXX AJMP:XX MSRC:XXX ASRC:XXXX");
    mvwprintw(cpu_w,4,2,"P:X H:X OBUS:XXXXXXXX");
  }
  */

  sprintf(LEDstring,"%c%c%c%c%c%c",
	  MCregister&MCR_LED_5_ ? 'O' : '*',
	  MCregister&MCR_LED_4_ ? 'O' : '*',
	  MCregister&MCR_LED_3_ ? 'O' : '*',
	  MCregister&MCR_LED_2_ ? 'O' : '*',
	  MCregister&MCR_LED_1_ ? 'O' : '*',
	  MCregister&MCR_LED_0_ ? 'O' : '*');

  // Draw box and title
  box(cpu_w,0,0);
  wmove(cpu_w,0,(COLS/4)-6);          // Move cursor
  wprintw(cpu_w,"CPU [%s]",LEDstring);
  wrefresh(cpu_w);
}

// Reset
void raven_initialize(){

  if(power_on_reset){
    int x;
    FILE *in;

    power_on_reset=0; // Clear flag

    if(!(in=fopen("proms/2236480-03","r"))){ perror("in0"); exit(1); }
    x=2047;
    while(!feof(in)){
      fread(&Irom[0][x],1,1,in); /* IR48-55 */
      x--;
    }
    fclose(in);
    if(!(in=fopen("proms/2236481-03","r"))){ perror("in1"); exit(1); }
    x=2047;
    while(!feof(in)){
      fread(&Irom[1][x],1,1,in); /* IR40-47 */
      x--;
    }
    fclose(in);
    if(!(in=fopen("proms/2236482-03","r"))){ perror("in2"); exit(1); }
    x=2047;
    while(!feof(in)){
      fread(&Irom[2][x],1,1,in); /* IR32-39 */
      x--;
    }
    fclose(in);
    if(!(in=fopen("proms/2236483-03","r"))){ perror("in3"); exit(1); }
    x=2047;
    while(!feof(in)){
      fread(&Irom[3][x],1,1,in); /* IR24-31 */
      x--;
    }
    fclose(in);
    if(!(in=fopen("proms/2236484-03","r"))){ perror("in4"); exit(1); }
    x=2047;
    while(!feof(in)){
      fread(&Irom[4][x],1,1,in); /* IR16-23 */
      x--;
    }
    fclose(in);
    if(!(in=fopen("proms/2236485-03","r"))){ perror("in5"); exit(1); }
    x=2047;
    while(!feof(in)){
      fread(&Irom[5][x],1,1,in); /* IR08-15 */
      x--;
    }
    fclose(in);
    if(!(in=fopen("proms/2236486-03","r"))){ perror("in6"); exit(1); }
    x=2047;
    while(!feof(in)){
      fread(&Irom[6][x],1,1,in); /* IR00-07 */
      x--;
    }
    fclose(in);
    
    x=0;
    while(x<2048){
      // PROM overlay is as follows:
      // Start with rom 6, read backwards
      PCS[x]  = Irom[6][x]; PCS[x] = PCS[x] << 8;
      PCS[x] |= Irom[5][x]; PCS[x] = PCS[x] << 8;
      PCS[x] |= Irom[4][x]; PCS[x] = PCS[x] << 8;
      PCS[x] |= Irom[3][x]; PCS[x] = PCS[x] << 8;
      PCS[x] |= Irom[2][x]; PCS[x] = PCS[x] << 8;
      PCS[x] |= Irom[1][x]; PCS[x] = PCS[x] << 8;
      PCS[x] |= Irom[0][x];
      x++;
    }
    // Initialize other cards
    nupi_init();
    mem8_init();
    sib_init();
    enet_init();
  }

  // Force start PC
  loc_ctr_reg=0;
  loc_ctr_nxt=-1;
  pj14_fetch_addr=-1;
  pj14_fetch_go=0;
  loc_ctr_cnt=0;
  // Force low
  MCregister = 0x90800000; // CPU in slot 9 (Which is NUbus 6), loop-self-test
  Memory_Busy = 0;
  Nubus_Busy = 0;
  Page_Fault = 0;
  NUbus_error = 0;
  VMAregister = 0x0A0; // Is this correct?
  imod_lo = 0;
  imod_hi = 0;
  uPCS_ptr_reg = 0;
}

void raven_disp_init(){
  if (no_curses == 0) {
    // Create CPU window
    cpu_w = newwin((LINES/2),(COLS/* /2 */),0,0);
  }
  // Update it.
  raven_dispupdate();
}

// *** INSTRUCTION PARTS ***

static inline void handle_m_fcn_src(){
  switch(MInst_M_Source_Addr){
  case 0100: // MBS-VMA
    // lcbus_io_stall();
    Mbus = VMAregister;
    break;
    
  case 0101: // MBS-Q-R
    Mbus = Qregister;
    break;
    
  case 0102: // MBS-Macro-Instruction-Argument-Field
    if((LCregister&0x01) == 0){
      Mbus = ldb(MIbuffer,6,16);
    }else{
      Mbus = ldb(MIbuffer,6,0);
    }
    break;
    
  case 0103: // MBS-Micro-Stack-Pointer
    uPCS_ptr_reg &= 0x3F;
    Mbus = uPCS_ptr_reg;
    break;
    
  case 0104: // MBS-MCR
    Mbus =
      (MCregister & ~0x00400000) |
      (need_fetch ? 0x00400000 : 0);
    break;
    
  case 0105: // MBS-Location-Counter
    LCregister &= 0x03FFFFFF;
    Mbus = LCregister;
    break;

  case 0106: // MBS-Memory-Map-Level-2-Address
    {
      int map1_addr = 0;
      uint32 map1_data = 0;
      
      int map2_addr = 0;
      uint32 map2_indx = 0; // INDEX bits from level 1
      uint32 map2_mdrg = 0; // Bits from MD
      
      // Get Level-1 index into 2
      map1_addr = ldb(MDregister,12,13);
      map1_data = vm_lv1_map[map1_addr];
      // Get level-2 address
      map2_indx = (map1_data&0x7F);            // Get index bits
      map2_mdrg = ((MDregister&0x1F00)>>8);    // And MD register
      map2_addr = (map2_indx << 5);            // Load index bits
      map2_addr |= map2_mdrg;                  // and vpage-offset
      // Check
      if(map2_addr > 0x1000){ logmsg("L2MEMOVR\n"); cpu_die_rq=1; }
      // Read
      Mbus = vm_lv2_adr[map2_addr];
      
      // if(loc_ctr_reg > 334){ logmsgf("L2AM: map1 = %lX map2_a = %lX M = %lX\n",map1_data,map2_addr,Mbus); }
    }
    break;

  case 0107: // MBS-Read-I-Arg (Dispatch constant?)
    disp_constant_reg &= 0x3FF;
    Mbus = disp_constant_reg;
    break;
	    
  case 0110: // MBS-Memory-Map-Level-1
//    Mbus = vm_lv1_map[ldb(MDregister,12,13)];
    Mbus = cached_lv1 & 0xffff;
    if(cpu_stop_en){
      logmsg("MBS-Memory-Map-Level-1 halt\n");
      cpu_die_rq=1;
      cpu_stop_en=2;
    }
    break;
    
  case 0111: // MBS-Memory-Map-Level-2-Control
    {
      int map1_addr = 0;
      uint32 map1_data = 0;
      
      int map2_addr = 0;
      uint32 map2_indx = 0; // INDEX bits from level 1
      uint32 map2_mdrg = 0; // Bits from MD
      
      // Get Level-1 index into 2
      map1_addr = ((MDregister&0x01FFE000)>>13); // ldb(MDregister,12,13);
      map1_data = vm_lv1_map[map1_addr];
      // Get level-2 address
      map2_indx = (map1_data&0x7F);            // Get index bits
      map2_mdrg = ((MDregister&0x1F00)>>8);    // And MD register
      map2_addr = (map2_indx << 5);            // Load index bits
      map2_addr |= map2_mdrg;                  // and vpage-offset
      // Check
      if(map2_addr > 0x1000){ logmsg("L2MEMOVR\n"); cpu_die_rq=1; }
      // Read
      Mbus = vm_lv2_ctl[map2_addr]&0xffff;
    }
    break;

  case 0112: // MBS-Macro-Instruction-Buffer;
    if((LCregister&0x01) == 0x01){
      Mbus = MIbuffer & 0xffff;
    }else{
      Mbus = ((MIbuffer&0xFFFF0000)>>16);
    }
    break;
    
  case 0113: // MBS-Macro-Instruction-Branch-Field
    if((LCregister&0x01) == 0){
      Mbus = ldb(MIbuffer,9,16);
    }else{
      Mbus = ldb(MIbuffer,9,0);
    }
    break;
    
  case 0120: // MBS-Micro-Stack-Data
    // Returns the microstack data at the pointer.
    // Push is pre-increment
    Mbus = (uPCS_stack[uPCS_ptr_reg]&0xFFFFF);
    break;
    
  case 0121: // MBS-Micro-Stack-Data-Pop
    Mbus = (uPCS_stack[uPCS_ptr_reg]&0xFFFFF);
    uPCS_ptr_reg--;  uPCS_ptr_reg &= 0x3F;
    break;
    
  case 0122: // MBS-MD
    // Starve out if memory busy
    // lcbus_io_stall();
    Mbus = MDregister;
    break;
    
  case 0140: // MBS-C-PDL-Buffer-Pointer
    Mbus = PDLmemory[pdl_ptr_reg];
    // logmsgf("C-PDL-Buffer-Pointer = 0x%lX\n",pdl_ptr_reg);
    break;

  case 0141: // MBS-C-PDL-Buffer-Index
    Mbus = PDLmemory[pdl_index_reg];
    break;
    
  case 0144: // MBS-C-PDL-Buffer-Pointer-Pop
    Mbus = PDLmemory[pdl_ptr_reg];
    pdl_ptr_reg--; pdl_ptr_reg &= 0x3FF;
    break;
    
  case 0145: // MBS-C-PDL-Buffer-Index-Decrement
    Mbus = PDLmemory[pdl_index_reg];
    pdl_index_reg--; pdl_index_reg &= 0x3FF;
    break;

  case 0150: // MBS-PDL-Buffer-Pointer
    Mbus = pdl_ptr_reg;
    break;
    
  case 0151: // MBS-PDL-Buffer-Index
    Mbus = pdl_index_reg;
    break;

  case 0155: // MBS-PDL-Buffer-Index-Decrement
    Mbus = pdl_index_reg;
    pdl_index_reg--; pdl_index_reg &= 0x3FF;
    break;
    
  default:
    logmsgf("UNKNOWN-M-FCN-SRC(0%o)\n",MInst_M_Source_Addr);
    cpu_die_rq=1;
  }
}

static inline void handle_m_fcn_dst(){
  // Handle MF bus
  switch(MInst_Dest_Functnl){
  case 0: // NOP
    break;
    
  case 01: // MBD-Location-Counter
    need_fetch = 1;
    LCregister = (Obus&0x03FFFFFF);
    break;
    
  case 02: // MBD-MCR
    // logmsgf("MCR = 0x%lX\n",Obus);
    {
      // Test
      if((MCregister&MCR_PROM_Disable) == 0 && (Obus&MCR_PROM_Disable)!= 0){
	logmsg("CPU: PROM DISABLED\n");
        // cpu_die_rq=1;
      }
      if((MCregister&MCR_Chaining_Enable) == 0 && (Obus&MCR_Chaining_Enable) != 0){
	logmsg("MACROINSTRUCTION CHAINING ENABLED\n");
      }
      // AND off the bits we can set
      MCregister &= 0xF08F0000; // Not allowed to write slotid,loop-on-test,interrupts
      // OR in the bits we want set
      MCregister |= (Obus&0x0F70FFFF);
      // Test bits we haven't supported yet
      if(MCregister&MCR_NU_Bus_Reset){
	logmsg("CPU: NUBUS RESET\n");
        cpu_die_rq=1;
      }
    }
    pi_system_check();      
    break;
    
  case 03: // MBD-Micro-Stack-Pointer
    uPCS_ptr_reg = Obus&0x3F;
    break;
    
  case 04: // MBD-Micro-Stack-Data
    // Returns the microstack data at the pointer.
    // Push is pre-increment
    uPCS_stack[uPCS_ptr_reg] = Obus;
    break;
    
  case 05: // MBD-Micro-Stack-Data-Push
    uPCS_ptr_reg++;  uPCS_ptr_reg &= 0x3F;
    uPCS_stack[uPCS_ptr_reg] = Obus;
    break;
    
  case 06: // MBD-OA-Reg-Low
    // (IMOD-LOW)
    // Data stored here will be deposited over the low half
    // of the next instruction.
    imod_lo = Obus;
    imod_en = 1;
    break;
    
  case 07: // MBD-OA-Reg-High
    // (IMOD-HI)
    // Data stored here will be deposited over the high half
    // of the next instruction.
    imod_hi = Obus&0xFFFFFF;
    imod_en = 1;
    break;

  case 010: // MBD-Macro-Instruction-Buffer
    MIbuffer = Obus;
    break;

  case 017: // MBD-Test-Synch
    //    logmsgf("[CPU] TEST-SYNCH @ %ld\n",loc_ctr_reg);
    MDregister = 0;
    LCbus_error = 0;
    NUbus_error = 0;
    pi_system_check();      
    break;
    
  case 020: // MBD-VMA
    lcbus_io_stall(); // Lisp crashes if this is not here
    VMAregister = Obus;
    break;
    
  case 021: // MBD-VMA-Write-Map-Level-1
    VMAregister = Obus;
    cached_lv1 = Obus;
    vm_lv1_map[ldb(MDregister,12,13)] = VMAregister;
#ifdef TRACELOG
    if(do_inst_trace){
      logmsgf("DEBUG: MMU: LV1-MAP-WRITE, L1-IDX = 0x%lX, DATA 0x%lX\n",
              ldb(MDregister,12,13),VMAregister);
    }
#endif
    break;
    
  case 022: // MBD-Write-Map-Level-2-Control
    {
      int map1_addr = 0;
      uint32 map1_data = 0;
      
      int map2_addr = 0;
      uint32 map2_indx = 0; // INDEX bits from level 1
      uint32 map2_mdrg = 0; // Bits from MD
      
      // Load VMA
      VMAregister = Obus;
      // Get Level-1 index into 2
      map1_addr = ((MDregister&0x01FFE000)>>13); // ldb(MDregister,12,13);
      map1_data = vm_lv1_map[map1_addr];
      // Get level-2 address
      map2_indx = (map1_data&0x7F);            // Get index bits
      map2_mdrg = ((MDregister&0x1F00)>>8);    // And MD register
      map2_addr = (map2_indx << 5);            // Load index bits
      map2_addr |= map2_mdrg;                  // and vpage-offset
      // Check
      if(map2_addr > 0x1000){ logmsg("L2MEMOVR\n"); cpu_die_rq=1; }
      // Write
      vm_lv2_ctl[map2_addr] = VMAregister&0xFFFF;
#ifdef TRACELOG
      if(do_inst_trace){
        logmsgf("DEBUG: MMU: LV2-CTL-WRITE, L2-IDX = 0x%lX, DATA (16b) = 0x%lX\n",
                map2_addr,VMAregister);
      }
#endif
    }
    break;
    
  case 023: // MBD-Write-Map-Level-2-Address
    {
      int map1_addr = 0;
      uint32 map1_data = 0;
      
      int map2_addr = 0;
      uint32 map2_indx = 0; // INDEX bits from level 1
      uint32 map2_mdrg = 0; // Bits from MD
      
      // Load VMA
      VMAregister = Obus;
      // Get Level-1 index into 2
      map1_addr = ldb(MDregister,12,13);
      map1_data = vm_lv1_map[map1_addr];
      // Get level-2 address
      map2_indx = (map1_data&0x7F);            // Get index bits
      map2_mdrg = ((MDregister&0x1F00)>>8);    // And MD register
      map2_addr = (map2_indx << 5);            // Load index bits
      map2_addr |= map2_mdrg;                  // and vpage-offset
      // Checking
      if(map2_addr > 0x1000){ logmsg("L2MEMOVR\n"); cpu_die_rq=1; }
      // Write
      vm_lv2_adr[map2_addr] = VMAregister&0x3FFFFF;
#ifdef TRACELOG
      if(do_inst_trace){
        logmsgf("DEBUG: MMU: LV2-ADR-WRITE, L2-IDX = 0x%lX, DATA (32b) = 0x%lX\n",
                map2_addr,VMAregister);
      }
#endif
      //      if(tracerq){ logmsgf("L2AM:(W) map1 = %lX map2_a = %lX M = %lX\n",map1_data,map2_addr,VMAregister); }
    }
    break;
    
  case 024: // MBD-VMA-Start-Read
    {
      uint32 physaddr=0;
      
      Page_Fault = 0;
      VMAregister = Obus; // Load VMA
      
      physaddr = VM_resolve_address(VM_READ);
      if(Page_Fault == 0){
	lcbus_io_request(VM_READ,physaddr,0,NUBUS_ID_CPU);
      }
    }
    break;
    
  case 025: // MBD-VMA-Start-Write
    {
      uint32 physaddr=0;
      
      Page_Fault = 0;
      VMAregister = Obus; // Load VMA
      
      physaddr = VM_resolve_address(VM_WRITE);
      if(Page_Fault == 0){
	lcbus_io_request(VM_WRITE,physaddr,MDregister,NUBUS_ID_CPU);
      }
    }
    break;

  case 026: // MBD-VMA-Start-Read-Unmapped
    {
      VMAregister = Obus; // Load VMA
      VM_unmapped_io(VM_READ);
    }
    break;
    
  case 027: // MBD-VMA-Start-Write-Unmapped
    {
      VMAregister = Obus; // Load VMA
      VM_unmapped_io(VM_WRITE);
    }
    break;
    
  case 030: // MBD-MD
    // Don't overwrite if memory is not ready
    // lcbus_io_stall();
    MDregister = Obus;
cached_lv1 = vm_lv1_map[ldb(Obus,12,13)];
    break;

  case 031: // MBD-MD-Write-Map-Level-1
    if(Memory_Busy > 0 && (LCbus_Request&0x1)==0){ logmsg("CPU: MD CLOBBER PROTECTION STOP 1\n"); cpu_die_rq=1; }
    MDregister = Obus;
    vm_lv1_map[ldb(MDregister,12,13)] = VMAregister;
    cached_lv1 = VMAregister;
#ifdef TRACELOG
    if(do_inst_trace){
      logmsgf("DEBUG: MMU: LV1-MAP-WRITE, L1-IDX = 0x%lX, DATA 0x%lX\n",
              ldb(MDregister,12,13),VMAregister);
    }
#endif
    break;
    
  case 032: // MBD-MD-Write-Map-Level-2-Control
    {
      int map1_addr = 0;
      uint32 map1_data = 0;
      
      int map2_addr = 0;
      uint32 map2_indx = 0; // INDEX bits from level 1
      uint32 map2_mdrg = 0; // Bits from MD
      
      // Load MD
      if(Memory_Busy > 0 && (LCbus_Request&0x1)==0){ logmsg("CPU: MD CLOBBER PROTECTION STOP 2\n"); cpu_die_rq=1; }
      MDregister = Obus;
      // Get Level-1 index into 2
      map1_addr = ldb(MDregister,12,13);
      map1_data = vm_lv1_map[map1_addr];
cached_lv1 = map1_data;
      // Get level-2 address
      map2_indx = (map1_data&0x7F);            // Get index bits
      map2_mdrg = ((MDregister&0x1F00)>>8);    // And MD register
      map2_addr = (map2_indx << 5);            // Load index bits
      map2_addr |= map2_mdrg;                  // and vpage-offset
      // Check
      if(map2_addr > 0x1000){ logmsg("L2MEMOVR\n"); cpu_die_rq=1; }
      // Write
      vm_lv2_ctl[map2_addr] = VMAregister&0xffff;
#ifdef TRACELOG
    if(do_inst_trace){
      logmsgf("DEBUG: MMU: LV2-CTL-WRITE, L2-IDX = 0x%lX, DATA (16b) = 0x%lX\n",
              map2_addr,VMAregister);
    }
#endif
    }
    break;
    
  case 033: // MBD-MD-Write-Map-Level-2-Address
    {
      int map1_addr = 0;
      uint32 map1_data = 0;
      
      int map2_addr = 0;
      uint32 map2_indx = 0; // INDEX bits from level 1
      uint32 map2_mdrg = 0; // Bits from MD
      
      // Load MD
      if(Memory_Busy > 0 && (LCbus_Request&0x1)==0){ logmsg("CPU: MD CLOBBER PROTECTION STOP 3\n"); cpu_die_rq=1; }
      MDregister = Obus;
      // Get Level-1 index into 2
      map1_addr = ldb(MDregister,12,13);
      map1_data = vm_lv1_map[map1_addr];
      cached_lv1 = map1_data;
      // Get level-2 address
      map2_indx = (map1_data&0x7F);            // Get index bits
      map2_mdrg = ((MDregister&0x1F00)>>8);    // And MD register
      map2_addr = (map2_indx << 5);            // Load index bits
      map2_addr |= map2_mdrg;                  // and vpage-offset
      // Checking
      if(map2_addr > 0x1000){ logmsg("L2MEMOVR\n"); cpu_die_rq=1; }
      // Write
      vm_lv2_adr[map2_addr] = VMAregister;
#ifdef TRACELOG
    if(do_inst_trace){
      logmsgf("DEBUG: MMU: LV2-ADR-WRITE, L2-IDX = 0x%lX, DATA (32b) = 0x%lX\n",
              map2_addr,VMAregister);
    }
#endif
    //      if(tracerq){ logmsgf("L2AM:(W) map1 = %lX map2_a = %lX M = %lX\n",map1_data,map2_addr,VMAregister); }
    }
    break;
    
  case 034: // MBD-MD-Start-Read
    {
      uint32 physaddr=0;
      
      Page_Fault = 0;
      if(Memory_Busy > 0 && (LCbus_Request&0x1)==0){ logmsg("CPU: MD CLOBBER PROTECTION STOP 4\n"); cpu_die_rq=1; }
      MDregister = Obus; // Load MD
      
      physaddr = VM_resolve_address(VM_READ);
      if(Page_Fault == 0){
	lcbus_io_request(VM_READ,physaddr,0,NUBUS_ID_CPU);
      }
    }
    break;
    
  case 035: // MBD-MD-Start-Write
    {
      uint32 physaddr=0;

      Page_Fault = 0;

      // LISP likes to cause collision for MD here. We ignore this in this one case...
      if(Memory_Busy > 1 && (LCbus_Request&0x1)==0){ logmsgf("CPU: MD CLOBBER PROTECTION STOP 5 (%d)\n",Memory_Busy); cpu_die_rq=1; }

      MDregister = Obus; // Load MD      
      physaddr = VM_resolve_address(VM_WRITE);
      if(Page_Fault == 0){
	lcbus_io_request(VM_WRITE,physaddr,MDregister,NUBUS_ID_CPU);
      }
    }
    break;
    
  case 036: // MBD-MD-Start-Read-Unmapped
    {
      if(Memory_Busy > 0 && (LCbus_Request&0x1)==0){ logmsg("CPU: MD CLOBBER PROTECTION STOP 6\n"); cpu_die_rq=1; }
      MDregister = Obus; // Load MD
cached_lv1 = vm_lv1_map[ldb(Obus,12,13)];
      VM_unmapped_io(VM_READ);
    }
    break;
    
  case 037: // MBD-MD-Start-Write-Unmapped
    {
      if(Memory_Busy > 0 && (LCbus_Request&0x1)==0){ logmsg("CPU: MD CLOBBER PROTECTION STOP 7\n"); cpu_die_rq=1; }
      MDregister = Obus; // Load VMA
cached_lv1 = vm_lv1_map[ldb(Obus,12,13)];
      VM_unmapped_io(VM_WRITE);
    }
    break;

  case 040: // MBD-C-PDL-Buffer-Pointer
	    // Write to PDL buffer at pointer
    PDLmemory[pdl_ptr_reg] = Obus;    
    // logmsgf("C-PDL-Buffer-Pointer = 0x%lX\n",pdl_ptr_reg);
    break;
    
  case 041: // MBD-C-PDL-Buffer-Index
	    // Write to PDL buffer at index
    PDLmemory[pdl_index_reg] = Obus;
    break;
    
  case 044: // MBD-C-PDL-Buffer-Pointer-Push
    pdl_ptr_reg++; pdl_ptr_reg &= 0x3FF;
    PDLmemory[pdl_ptr_reg] = Obus;
    break;
    
  case 045: // MBD-C-PDL-Buffer-Index-Increment
	    // Write to PDL buffer at index++
    pdl_index_reg++; pdl_index_reg &= 0x3FF;
    PDLmemory[pdl_index_reg] = Obus;
    break;
    
  case 050: // MBD-PDL-Buffer-Pointer
    pdl_ptr_reg = Obus&0x3FF;
    break;
    
  case 051: // MBD-PDL-Buffer-Index
    pdl_index_reg = Obus&0x3FF;
    break;
    
  case 066: // MBD-VMA-Start-Read-Unmapped-NU (NU = BYTE)
    {
      VMAregister = Obus; // Load VMA
      VM_unmapped_io(VM_BYTE_READ);
    }
    break;
    
  case 067: // MBD-VMA-Start-Write-Unmapped-NU (NU = BYTE)
    {
      VMAregister = Obus; // Load VMA
      VM_unmapped_io(VM_BYTE_WRITE);
    }
    break;
    
  case 076: // MBD-MD-Start-Read-Unmapped-NU (NU = BYTE)
    {
      if(Memory_Busy > 0 && (LCbus_Request&0x1)==0){ logmsg("CPU: MD CLOBBER PROTECTION STOP 8\n"); cpu_die_rq=1; }
      MDregister = Obus; // Load MD
      VM_unmapped_io(VM_BYTE_READ);
    }
    break;

  case 077: // MBD-MD-Start-Write-Unmapped-NU (NU = BYTE)
    {
      if(Memory_Busy > 0 && (LCbus_Request&0x1)==0){ logmsg("CPU: MD CLOBBER PROTECTION STOP 9\n"); cpu_die_rq=1; }
      MDregister = Obus; // Load VMA
      VM_unmapped_io(VM_BYTE_WRITE);
    }
    break;    
    
  default:
    logmsgf("UNKNOWN-FCN-DST(0%o)\n",MInst_Dest_Functnl);
    cpu_die_rq=1;
    break;
  }
}

static inline void handle_condition_select(){
  // Initialize
  test_true = 0;

  if(MInst_Cond_ClRAM_Sel == 0){
    switch(MInst_Cond_Select){
    case 00: // Condition-Bit-Set
      {
	int MInst_Rotation_Directn = (Iregister&0x10000);
	int MInst_Bit_Set_Pos = (Iregister&0x1F); 
	
	// What bit where?
	if(MInst_Rotation_Directn == 0){
	  test_true = left_rotate(Mbus,MInst_Bit_Set_Pos)&0x01;
	}else{
	// RIGHT ROTATION, rotate the mask from the opposite side
	  test_true = right_rotate(Mbus,MInst_Bit_Set_Pos)&0x01;
	}
      }
      break;
	
    case 01: // Condition-Less
      //	  logmsgf("CONDITION-LESS %lX %lX\n",0x80000000^Mbus,0x80000000^Abus);
      if((0x80000000^Mbus) < (0x80000000^Abus)){
	test_true = 1;
      }
      break;

    case 02: // Condition-Less-Or-Equal
      if((ALU_Result&0x80000000) != 0){
	test_true=1;
      }
      break;
      
    case 03: // Condition-Not-Equal
      if(ALU_Result != 0xFFFFFFFF){
        test_true = 1;
      }      
      break;
      
    case 04: // Condition-Page-Fault
      if(Page_Fault != 0){
	test_true = 1;
      }
      break;
      
    case 05: // Condition-Page-Fault-Or-Interrupt
      if(Page_Fault != 0 || Active_PI != 0){
	test_true = 1;
      }
      break;
	    
    case 06: // Condition-Page-Fault-Or-Interrupt-Or-Sequence-Break
      if(Page_Fault != 0 || Active_PI != 0 || (MCregister&MCR_Sequence_Break) != 0){
	test_true = 1;
      }
      break;
      
    case 07: // Condition-True
      test_true=1;
      break;
      
    case 010: // Condition-Tag-Not-Equal
      if(ldb(Mbus,5,25) != ldb(Abus,5,25)){
	test_true=1;
      }
      break;
      
    case 011: // Condition-Not-Memory-Busy
      if(Memory_Busy == 0){
	test_true = 1;
      }
      break;
      
    case 012: // Condition-Q0
      test_true = (Qregister&0x01);
      break;
      
    case 013: // Condition-NU-Bus-Error
      // Really checks for Local Bus errors, only sees errors that are Raven's fault.
      if(LCbus_error != 0){
	test_true=1;
      }
      break;
      
    case 014: // Condition-Not-Fixnum-Overflow
      if(ALU_Fixnum_Oflow == 0){
	test_true=1;
      }
      break;
      
    case 015: // Condition-Boxed-Sign-Bit
      if((ALU_Result&0x01000000) == 0x01000000){
	test_true=1;
      }
      break;
      
    case 016: // Condition-No-Interrupt
      if(Active_PI==0){
	test_true=1;
      }
      break;

    case 017: // RESERVED-UNKNOWN-OPERATION
      // EXPT uses this one during the T-memory test.
      // I don't know what it does, and so I will fake it out.
      logmsg("CONDITION-17-USED\n");
      test_true=1;
      break;

    default:
      logmsgf("UNKNOWN COND SELECT (0%o) - OP %d @ uPC %ld TMW %d TMS %d\n",MInst_Cond_Select,opword,loc_ctr_reg,
	      MInst_Write_ClRAM,MInst_Cond_ClRAM_Sel);
      cpu_die_rq=1;
    }    
  }else{
    // T-MEM READ: MInst_Cond_Select has address
    uint32 data;
    int Tpos,result;
    data = Tmemory[MInst_Cond_Select];
    Tpos = ldb(Mbus,5,25);     // Tag position
    result = ldb(data,1,Tpos); // Isolate tag bit
    #ifdef TRACELOG
    if(do_tmem_trace){ logmsgf("T-MEM READ: Loc %X, Data %X, want bit %d, got %d\n",MInst_Cond_Select,data,Tpos,result); }
    #endif
    if(result){ test_true = 1; }
  }

  if(MInst_Cond_Invrt_Sns != 0){
    test_true ^= 1; // Flip bit
  }
}

static inline void handle_popj_14_nxt(){
  // handle popj-14-after-next
  // This is wrong somehow...
  // Check PC-After-Next
  if (loc_ctr_nxt != -1 && (loc_ctr_nxt&0x4000)) {          
    int chain_enable = MCregister & 0x04000000;
    uint32 saved_vma,physaddr;

    if (need_fetch) {
      saved_vma = VMAregister;
      VMAregister = (LCregister >> 1) & 0x1ffffff;
      pj14_fetch_vma = VMAregister;
      physaddr = VM_resolve_address(VM_READ);
      VMAregister = saved_vma;
      
      if(Page_Fault == 0){        
        pj14_fetch_addr = physaddr; // Arrange for fetch and VMA overwrite
#ifdef TRACELOG
        if(do_inst_trace){
          logmsgf("DEBUG: PJ14-After-Next Fetch Needed\n");
        }
#endif
      }else{
        // Overwrite VMA immediately
        VMAregister = pj14_fetch_vma;        
      }
    }

    LCregister++;
    
    if (chain_enable == 0)
      loc_ctr_nxt |= 2;
    
    if (need_fetch == 0 && chain_enable){
      // I think MCR_Local_Reset may affect this.
      // It seems to be the only difference between the EXPT case and LISP case that makes any sense.
      if(MCregister&MCR_Local_Reset){
        loc_ctr_nxt |= 2; // LISP
      }else{
        loc_ctr_nxt |= 3; // EXPT
      }
    }

    if (need_fetch == 0 || chain_enable)
      need_fetch = (LCregister & 1) == 0;    

    // Turn off PJ14
    loc_ctr_nxt &= 0xFBFFF;    
  }
  if(loc_ctr_nxt != -1 && loc_ctr_nxt&0xFFFF8000){
    logmsgf("UNKNOWN HI BITS IN PC-NXT (0x%lX)\n",loc_ctr_nxt);
    cpu_die_rq=1;
  }
}

static inline void handle_popj_14(){
  // Check PC
  if (loc_ctr_reg & 0x4000) {          
    int chain_enable = MCregister & 0x04000000;
    uint32 physaddr;
    
    if (need_fetch) {
      VMAregister = (LCregister >> 1) & 0x1ffffff;
      physaddr = VM_resolve_address(VM_READ);
      if(Page_Fault == 0){        
#ifdef TRACELOG
        if(do_inst_trace){
          logmsgf("DEBUG: PJ14 Fetch Needed\n");
        }
#endif
        pj14_fetch_addr = physaddr; // Arrange for fetch
      }
    }

    LCregister++;
    if (chain_enable == 0)
      loc_ctr_reg |= 2;
    
    if (need_fetch == 0 && chain_enable){
      // See comment for this above
      if(MCregister&MCR_Local_Reset){
        loc_ctr_reg |= 2; // LISP
      }else{
        loc_ctr_reg |= 3; // EXPT
      }
    }

    if (need_fetch == 0 || chain_enable)
      need_fetch = (LCregister & 1) == 0;    
  }
  // Anyway...

  // Turn off PJ14 and UNKNOWN-18
  loc_ctr_reg &= 0x7BFFF;

  // FIXME: INVESTIGATE THIS LATER
  if(loc_ctr_reg&0xFFFF8000){
    logmsgf("UNKNOWN HI BITS IN PC (0x%lX)\n",loc_ctr_reg);
    cpu_die_rq=1;
  }
}

static inline void handle_abj(){
  switch(MInst_Abbrv_Jump){
  case 0: // A-Jump-Field-Nop
    break;
    
  case 1: // A-Jump-Field-Skip
    // Skip next instruction
    loc_ctr_reg++;
    break;
    
  case 2: // A-Jump-Call-Illop
    // PUSH ADDRESS
    uPCS_ptr_reg++;  uPCS_ptr_reg &= 0x3F;
    uPCS_stack[uPCS_ptr_reg] = loc_ctr_reg;
    // Jump
    loc_ctr_reg = 010;
    break;
    
  case 3: // A-Jump-Call-Trap
    // PUSH ADDRESS
    uPCS_ptr_reg++;  uPCS_ptr_reg &= 0x3F;
    uPCS_stack[uPCS_ptr_reg] = loc_ctr_reg;
    // Jump
    loc_ctr_reg = 012;
    break;
    
  case 4: // A-Jump-Call-Buserr
    // PUSH ADDRESS
    uPCS_ptr_reg++;  uPCS_ptr_reg &= 0x3F;
    uPCS_stack[uPCS_ptr_reg] = loc_ctr_reg;
    // Jump
    loc_ctr_reg = 014;
    break;
    
  case 5: // A-Jump-Call-Unused
    // PUSH ADDRESS
    uPCS_ptr_reg++;  uPCS_ptr_reg &= 0x3F;
    uPCS_stack[uPCS_ptr_reg] = loc_ctr_reg;
    // Jump
    loc_ctr_reg = 016;
    break;
    
  case 6: // A-Jump-Popj
    // POP ADDRESS
    loc_ctr_reg = (uPCS_stack[uPCS_ptr_reg]&0xFFFFF);
    uPCS_ptr_reg--;  uPCS_ptr_reg &= 0x3F;
    handle_popj_14();
    break;
    
  case 7: // A-Jump-Popj-After-Next
    loc_ctr_nxt = (uPCS_stack[uPCS_ptr_reg]&0xFFFFF);
    uPCS_ptr_reg--;  uPCS_ptr_reg &= 0x3F;
    handle_popj_14_nxt();
    break;
    
  default:
    logmsgf("AJMP UNKNOWN(0%o)\n",MInst_Abbrv_Jump);
    cpu_die_rq=1;
  }
}

static inline void handle_output_select(){
  switch(MInst_Output_Bus_Ctl){
	    
  case 00: // Output-Bus-A-Bus
  case 02: // Output-Bus-A-Bus2
    Obus = Abus;
    break;
	    
  case 01: // Output-Bus-R-Bus
    {
      MInst_Rotation_Count = (Iregister&0x1F);             // ldb(Iregister,5,0);
      MInst_Rotation_Directn = (int)(Iregister&0x10000);  // ((Iregister>>16)&0x01); // ldb(Iregister,1,16);
      MInst_Rotation_Len = 32;
      MInst_Mask_Rotate = 0;
      MInst_Source_Rotate = 1;
      operate_shifter();
      Obus = Rbus;
    }
    break;
	  
  case 03: // Output-Bus-Normal
    Obus = Obus_Input;
    break;
	    
  case 04: // Output-Bus-LeftShift-1
    // Left shift gets SIGN from Qregister...
    Obus = (Obus_Input << 1);
    if(Qregister&0x80000000){ Obus |= 1; }
    break;
	    
  case 05: // Output-Bus-RightShift-1
    Obus = Obus_Input >> 1;
    // Different method
    if(ALU_Carry_Out != 0){
      Obus |= 0x80000000;
    }else{
      Obus &= ~0x80000000;
    }
    break;
	    
  case 06: // Output-Bus-Sign-Extend
    if((Obus_Input&0x01000000)==0){
      Obus = Obus_Input&0x00FFFFFF;
    }else{
      Obus = Obus_Input|0xFF000000;
    }
    break;
	    
  case 07: // Output-Bus-Mirror
    /* Reverse the bits in the ALU output
       Finally, a chance to use this famous example of confusing code!
       (Is it bad that I immediately recognised what it was doing when first reading it?) */
    Obus = Obus_Input;
    Obus = ((Obus >>  1) & 0x55555555) | ((Obus <<  1) & 0xaaaaaaaa);
    Obus = ((Obus >>  2) & 0x33333333) | ((Obus <<  2) & 0xcccccccc);
    Obus = ((Obus >>  4) & 0x0f0f0f0f) | ((Obus <<  4) & 0xf0f0f0f0);
    Obus = ((Obus >>  8) & 0x00ff00ff) | ((Obus <<  8) & 0xff00ff00);
    Obus = ((Obus >> 16) & 0x0000ffff) | ((Obus << 16) & 0xffff0000);
    /* I also just realized that Nevermore does the same thing in LISP.
       LISP somehow makes it look much more confusing.
       Maybe it's just me. */
    break;
	    
  default:
    logmsgf("UNKNOWN-OBUS-CTL(%d)\n",MInst_Output_Bus_Ctl);
    cpu_die_rq=1;
    break;
  }
}

// Clock Pulse
void raven_clockpulse(){
  cycle_count++;
  // Perform control cycle
  while(!cpu_die){    
    ccount++;
    delay_indx=0;
    while(delay_indx < delay_tune){
      delay_indx++;
    }

    // Run one clock

    // Do POPJ-14 fetch initiation here
    if(pj14_fetch_addr != -1){      
      if(pj14_fetch_go == 1){
#ifdef TRACELOG
        if(do_inst_trace){
          logmsgf("DEBUG: PJ14 INITIATING MEMORY READ\n");
        }
#endif
        // Overwrite VMA here
        VMAregister = pj14_fetch_vma;
        lcbus_io_request(VM_READ,pj14_fetch_addr,0,NUBUS_ID_CPU);

        pj14_fetch_addr = -1;
        pj14_fetch_go = 0;
      }else{
        pj14_fetch_go = 1;
      }
    }

    // Update PC.
    // New version with correct pipelining.
    loc_ctr_cnt = loc_ctr_reg;     // Prepare to fetch next

    // If AfterNEXT
    if(loc_ctr_nxt != -1){
      loc_ctr_reg = loc_ctr_nxt; // Load next instruction
      loc_ctr_nxt = -1;          // Disarm
    }else{
      loc_ctr_reg = loc_ctr_cnt + 1; // Otherwise get next instruction
    }
    last_loc_ctr = loc_ctr_reg;    // Save This

    // TRAP PC GENERATOR
    if(trap_test > 0){
      trap_test--;    
      if((MCregister&MCR_Parity_Trap_Enable) && LCbus_error != 0 && trap_test == 1 && trap_type == 0){
        // IT'S A TRAP!
        logmsgf("GENERATING BUS-ERROR TRAP! (LCbus_Addr = 0x%lX)\n",LCbus_Address);
        // PUSH ADDRESS
        uPCS_ptr_reg++;  uPCS_ptr_reg &= 0x3F;
        uPCS_stack[uPCS_ptr_reg] = loc_ctr_cnt; // Push THIS address since we will overwrite it
        // Now push pc-after-next
        uPCS_ptr_reg++;  uPCS_ptr_reg &= 0x3F;
        uPCS_stack[uPCS_ptr_reg] = loc_ctr_reg;
        
        loc_ctr_cnt = 16; // Change current address to trap vector (16)
        loc_ctr_reg = 17;
        trap_test = 0;
        trap_type = 0;
      }    
    }

    // 819 to trap GDOS startup
    // 894 to trap macro instructions
    // 1232 to trap microloading
    // 15024 to trap ucode paging

    //    if((MCregister&MCR_PROM_Disable) != 0 && loc_ctr_cnt == 28){    
    // 2903 to trap bug
    // 13024 to trap new bug (w/ LC 0x33)

    // Failing macro instruction is at LC 1857AA
    // 18534D is start of initialization
    // 183CE5 is start of DEATH

    /*
    if(loc_ctr_cnt == 894 && LCregister == 0x18534D){
      logmsgf("[CPU] UCode breakpoint halt\n");
      tracerq=1;
      cpu_die_rq=1;
    }         
    */
    
    // Trace macro events
#ifdef TRACELOG2
    switch(loc_ctr_cnt){
    case 894:
    case 5450:
    case 5809:
    case 6047:
    case 6068:
    case 6093:
    case 6113:
      logmsgf("DEBUG: L-%.5d ** MACRO-INSTRUCTION ** - LC = 0x%lX, A-58 = 0x%lX, A-70 = 0x%lX\n",loc_ctr_cnt,LCregister,Amemory[58],Amemory[70]);      
    }
#endif

#ifdef DEI_TRACELOG
    dei_disasm();
#endif
    
#ifdef TRACELOG
    if(loc_ctr_cnt > 16384){
      logmsgf("[CPU] LC %d (0x%lX) IS OUT OF BOUNDS FOR WCS!\n",loc_ctr_cnt,loc_ctr_cnt);
      cpu_die=1;
      break;
    }        
#endif

    // FETCH    
    if(loc_ctr_cnt > 2048 || MCregister&MCR_PROM_Disable){
      Iregister = WCS[loc_ctr_cnt];
    }else{
      Iregister = PCS[loc_ctr_cnt];
    }

    // RAM CYCLES
    // Cause a NUbus cycle if there is a bus cycle to do
    if(Nubus_Busy > 0){
      Nubus_Busy--;
      nubus_io_pulse();
    }

    // Operate peripherals
    nupi_clock_pulse();
#ifdef ENABLE_ENET
    enet_clock_pulse();    
#endif
    sib_clock_pulse();

    // Do a memory cycle (This turns out to be very time-expensive)
    if(Memory_Busy > 0){
      Memory_Busy--;
      lcbus_io_pulse();
    }

    // IMOD LOGIC > IR
    if(imod_en != 0){
      Iregister |= ((uint64)imod_hi << 32) | imod_lo;
      imod_en = 0;
      imod_hi = 0;
      imod_lo = 0;
    }else{
#ifdef I_PARITY_CHECK
      // IMod inhibits parity checking?      
      if((MCregister&(MCR_Parity_Halt_Enable|MCR_Parity_Trap_Enable)) != 0){
	// PARITY TEST
	MInst_Parity = ((Iregister >> 50)&0x1);  //     = ldb(Iregister,1,50);
	if(gen_i_parity(Iregister) != MInst_Parity){
	  logmsgf("PARITY ERROR %s: %d\n",
                  MCregister&MCR_Parity_Halt_Enable ? "HALT" : "TRAP",
                  gen_i_parity(Iregister)); 
	}
      }
#endif
    }
    
    // Collect source addresses
    MInst_M_Source_Addr  = ((Iregister>>42)&0x7F);  // ldb(Iregister,7,42);
    MInst_A_Source_Addr  = ((Iregister>>32)&0x3FF); // ldb(Iregister,10,32);

    // Prepare source reads
    // Trigger read operations
    Abus = Amemory[MInst_A_Source_Addr];
    // Handle M Bus Input
    if(MInst_M_Source_Addr > 077){
      handle_m_fcn_src();
    }else{
      Mbus = Mmemory[MInst_M_Source_Addr];
    }
    
    // GLOBAL FIELD DECODE
    opword           = ((Iregister >> 54)&0x3);  //     = ldb(Iregister,2,54); 
    MInst_Abbrv_Jump = ((Iregister >> 51)&0x07); //     = ldb(Iregister,3,51);
    
    // MInst_Halt
    if((Iregister&0x2000000000000LL)){
      logmsg("**MInst-HALT**\n");
      cpu_die_rq=1;
      // Save NVRAM
      save_nvram();
    }

    switch(opword){
    case 0: // ALU-OP
      {
	// Fields
	uint32 MInst_Dest_A_Select  = (int)(Iregister&0x80000000); // ((Iregister>>31)&0x01);   // ldb(Iregister,1,31);
	int MInst_Dest_AMem_Addr = ((Iregister>>19)&0x3FF);      // ldb(Iregister,10,19);
	int MInst_Dest_MMem_Addr = ((Iregister>>19)&0x3F);       // ldb(Iregister,6,19);

	MInst_Write_ClRAM    = (int)(Iregister&0x200);      // ((Iregister>>9)&0x01);   // ldb(Iregister,1,9); 
	int MInst_Q_Control      = (Iregister&0x03);             // ldb(Iregister,2,0);

	MInst_Output_Bus_Ctl = ((Iregister>>16)&0x07);       // ldb(Iregister,3,16);
	MInst_Dest_Functnl   = ((Iregister>>25)&0x3F);       // ldb(Iregister,6,25);
	MInst_Cond_Invrt_Sns = (int)(Iregister&0x8000);     // ((Iregister>>15)&0x01);   // ldb(Iregister,1,15);
	MInst_Cond_ClRAM_Sel = (int)(Iregister&0x4000);     // ((Iregister>>14)&0x01);  // ldb(Iregister,1,14); 
	MInst_Cond_Select    = ((Iregister>>10)&0x0F);       // ldb(Iregister,4,10);

	MInst_ALU_Opcode     = ((Iregister>>3)&0x1F);        // ldb(Iregister,5,3);
	MInst_ALU_Carry_In   = (int)(Iregister&0x4);        // ((Iregister>>2)&0x01);   // ldb(Iregister,1,2);
      
	// Reset flags
	ALU_Fixnum_Oflow=0;
	ALU_Carry_Out=0;
        
	switch(MInst_ALU_Opcode){
	case 0: // ALU-Opcode-SETZ
	  ALU_Result = 0;
	  break;	      

	case 1: // ALU-Opcode-AND
	  ALU_Result = Mbus&Abus;
	  // If bit sign is set, carry-out.
	  if(ALU_Result&0x80000000){
	    ALU_Result |= 0x100000000LL;
	  }
	  break;
	
	case 2: // ALU-Opcode-ANDCA
	  ALU_Result = Mbus&(~Abus);
	  break;

	case 3: // ALU-Opcode-SETM
	  ALU_Result = Mbus;
	  break;

	case 4: // ALU-Opcode-ANDCM
	  ALU_Result = (~Mbus)&Abus;
	  break;

	case 5: // ALU-Opcode-SETA
	  ALU_Result = Abus;
	  break;

	case 6: // ALU-Opcode-XOR
	  ALU_Result = Mbus^Abus;
	  break;

	case 7: // ALU-Opcode-IOR
	  ALU_Result = Mbus|Abus;
	  break;

	case 010: // ALU-Opcode-ANDCB
	  ALU_Result = (~Mbus)&(~Abus);
	  break;

	case 011: // ALU-Opcode-EQV
	  Abus = Mbus; // What for?
	  ALU_Result = Mbus;
	  break;

	case 012: // ALU-Opcode-SETCA
	  ALU_Result = ~Abus;
	  break;

	case 013: // ALU-Opcode-ORCA
	  ALU_Result = Mbus|(~Abus);
	  break;

	case 014: // ALU-Opcode-SETCM
	  ALU_Result = ~Mbus;
	  break;
	  
	case 015: // ALU-Opcode-ORCM
	  ALU_Result = (~Mbus)|Abus;
	  break;

	case 017: // ALU-Opcode-SETO
	  ALU_Result = 0xFFFFFFFF;
	  break;

	case 020: // ALU-Opcode-MUL
	  if((Qregister&0x01)==0x01){
	    ALU_Result = Mbus + Abus;
	  }else{
	    ALU_Result = Mbus;
	  }
	  break;

	case 022: // ALU-Opcode-DIV
	  if((Qregister&0x01)==0){
	    ALU_Result = Mbus + Abus;
	  }else{
	    ALU_Result = Mbus - Abus;
	  }
	  break;

	case 023: // ALU-Opcode-DIV-First
	  ALU_Result = Mbus - Abus;
	  break;

	case 024: // ALU-Opcode-DIV-Corr
	  if((Qregister&0x01)==0x00){
	    ALU_Result = Mbus + Abus;
	  }else{
	    ALU_Result = Mbus;
	  }
	  break;

	case 031: // ALU-Opcode-ADD
          ALU_Result = Mbus + Abus + (MInst_ALU_Carry_In ? 1 : 0);
	  if((((Mbus^ALU_Result)&(Abus^ALU_Result))&0x01000000)==0x01000000){
	    ALU_Fixnum_Oflow=1;
	  }
          fix_alu_carry_out();
	  break;
	
	case 034: // ALU-Opcode-M
	  // Output = M or M+1
	  ALU_Result = Mbus;
	  if(MInst_ALU_Carry_In != 0){
	    ALU_Result++;
	  }
          fix_alu_carry_out();
	  break;

	case 036: // ALU-Opcode-SUB
	  ALU_Result = Mbus - Abus - (MInst_ALU_Carry_In ? 0 : 1);
	  // FIXNUM Overflow Check
	  if((((Mbus^Abus)&(Mbus^ALU_Result))&0x01000000)==0x01000000){
	    ALU_Fixnum_Oflow=1;
	  }
	  break;

	case 037: // ALU-Opcode-M+M
	  // Output = M+M or M+M+1
	  ALU_Result = Mbus+Mbus;
	  if(MInst_ALU_Carry_In != 0){
	    ALU_Result++;
	  }
	  /*
	  // HACKHACK is this correct?
	  if(ALU_Result < Mbus){
	    ALU_Result |= 0x100000000LL; // Arrange for carry-out
	  }
	  */

          if (Mbus & 0x80000000)
	    ALU_Result |= 0x100000000LL; // Arrange for carry-out

	  break;

	default:
          logmsgf("UNKNOWN-ALU-OPCODE(0%o)\n",MInst_ALU_Opcode);          
	  cpu_die_rq=1;
	  break;
	}

	// Reduce carry-out to a flag
	ALU_Carry_Out = (ALU_Result&0xFFFFFFFF00000000LL) ? 1 : 0;
	// Clean Output (ALU is 32 bits wide)
	ALU_Result &= 0xFFFFFFFF;

        if (MInst_ALU_Opcode < 030) {
          ALU_Carry_Out = (ALU_Result & 0x80000000) ? 1 : 0;
        }

	// Make the result
	Obus = ALU_Result;

	if(MInst_Write_ClRAM != 0){
	  uint32 data = 0;
	  uint32 Tmask=0,Tdata=0;
	  int Trotn=0;

	  data = ALU_Result;

	  // Get position of tag bit to overwrite
	  Trotn = (data>>25)&0x1F; // ldb(data,5,25);
	  // And make a mask
	  Tmask = 1 << Trotn;
	  
	  // Test
	  if(Trotn > 31){ logmsgf("Trotn %d > 31!\n",Trotn); cpu_die_rq=1; }

	  Tdata = Tmemory[MInst_Cond_Select];
	  // Just...
	  if(data & 0x40000000){
	    // Set it...
	    Tdata |= Tmask;
	    if(do_tmem_trace){ logmsgf("T-MEM WRITE: Bit %d set for %X = %LX\n",Trotn,MInst_Cond_Select,Tdata); }
	  }else{
	    // Or forget it!
	    Tdata &= (~Tmask);
	    if(do_tmem_trace){ logmsgf("T-MEM WRITE: Bit %d reset for %X = %LX\n",Trotn,MInst_Cond_Select,Tdata); }
	  }
	  // But wait, there's more!	
	  Tmemory[MInst_Cond_Select] = Tdata;
	  // Now how much would you pay?
	}
	// Run conditionalizer
	handle_condition_select(); 
	
	if(test_true == 1){
	  // Don't handle ABJ if no test truth
	  handle_abj();
	}

	// Determine output location
	Obus_Input = ALU_Result;

	// If in tagged mode...
	if((Iregister&0x100) == 0x100){
          // High bits come from A input. Bits 24-0 pass through the ALU.
	  Obus_Input = (ALU_Result&0x01FFFFFF);
	  Obus_Input |= (Abus&0xFE000000);

	  switch(MInst_Output_Bus_Ctl){
	  case 1: // Rbus
	    {
	      MInst_Rotation_Count = (Iregister&0x1F);             // ldb(Iregister,5,0);
	      MInst_Rotation_Directn = (int)(Iregister&0x10000);  // ((Iregister>>16)&0x01); // ldb(Iregister,1,16);
	      MInst_Rotation_Len = 32;
	      MInst_Mask_Rotate = 0;
	      MInst_Source_Rotate = 1;
	      operate_shifter();
	      Obus  = Obus_Input&0xFE000000; // Preserve A bits
	      Obus |= (Rbus&0x1FFFFFF);      // Include R bus
	    }
	    break;
	    
	  case 3: // ALU
	    Obus = Obus_Input;
	    break;

	  case 5: // Output-Bus-RightShift-1
	    Obus = Obus_Input >> 1;                   // Rightshift...
	    Obus &= 0x1FFFFFF;                        // Then eliminate the high bits
	    // Different method for shorter math
	    {
	      // Funny carry arithmetic...
	      uint32 atmp=0;
	      atmp =  ((Abus&0xFF000000)>>24);        // Insert A-bus
	      atmp += ((Obus_Input&0xFF000000)>>24);  // and the high output bits?
	      if(atmp&0xFFFFFF00){atmp |= 0x80; }     // Summation of overflow
	      Obus |= (atmp<<24);                     // Tack it on the O bus
	    }
	    if(ALU_Carry_Out){                        // If ALU carried-out
	      Obus ^= 0x80000000;                     // invert 32-bit sign.
	    }
	    break;
	    
	  case 7: // MIRROR
	    {
	      // Reverse
	      Obus_Input = (Obus_Input>>8);
	      Obus = ALU_Result;
	      Obus = ((Obus >>  1) & 0x55555555) | ((Obus <<  1) & 0xaaaaaaaa);
	      Obus = ((Obus >>  2) & 0x33333333) | ((Obus <<  2) & 0xcccccccc);
	      Obus = ((Obus >>  4) & 0x0f0f0f0f) | ((Obus <<  4) & 0xf0f0f0f0);
	      Obus = ((Obus >>  8) & 0x00ff00ff) | ((Obus <<  8) & 0xff00ff00);
	      Obus = ((Obus >> 16) & 0x0000ffff) | ((Obus << 16) & 0xffff0000);
	      // And sign-extend
	      if((ALU_Result&0x01000000)==0){
		Obus = Obus&0x01FFFFFF;
	      }else{
		Obus = Obus|0xFE000000;
	      }
	    }
	    break;
	    
	  default:
	    logmsgf("CPU: Unknown Tagged-Mode OBus Control 0x%X\n",MInst_Output_Bus_Ctl);
	    cpu_die_rq=1;
	  }
	}else{
          handle_output_select();
        }

	// Run Q
        switch(MInst_Q_Control){
        case 0: // Q-Control-NOP
          break;
          
        case 01: // Q-Control-Shift-Left
          // Shift
          Qregister = Qregister << 1;
          // Carry-In inverse of ALU sign bit
          if((ALU_Result&0x80000000) == 0){
            Qregister |= 1;
          }
          break;
          
          // For later reference, a right-shift is not inverted from the ALU output.
          
        case 02: // Q-Control-Shift-Right
          Qregister = Qregister >> 1;
          // Carry-In ALU result
          if((ALU_Result&01) == 0x01){
            Qregister |= 0x80000000;
          }
          break;
          
        case 03: // Q-Control-Load
          Qregister=ALU_Result;
          break;
          
        default:
          logmsgf("UNKNOWN-Q-CONTROL(0%o)\n",MInst_Q_Control);
          cpu_die_rq=1;
        }	

	// Store result
	if(MInst_Dest_A_Select != 0){
	  // Store in A-Memory
          Amemory[MInst_Dest_AMem_Addr] = Obus;
	}else{
	  // Store in M-Memory
          Mmemory[MInst_Dest_MMem_Addr] = Obus;
          Amemory[MInst_Dest_MMem_Addr] = Obus;
          // Also do this
	  handle_m_fcn_dst();
	}
	break;
      }

    case 1: // BYTE-OP
      {
	uint32 MInst_Dest_A_Select  = (int)(Iregister&0x80000000); // ((Iregister>>31)&0x01);   // ldb(Iregister,1,31);
	int MInst_Dest_MMem_Addr = ((Iregister>>19)&0x03F);      // ldb(Iregister,6,19);
	int MInst_Dest_AMem_Addr = ((Iregister>>19)&0x3FF);      // ldb(Iregister,10,19);
	// BYTE-specific
	MInst_Mask_Rotate      = (int)(Iregister&0x40000);  // ldb(Iregister,1,18);
	MInst_Source_Rotate    = (int)(Iregister&0x20000);  // ldb(Iregister,1,17);
	MInst_Rotation_Directn = (int)(Iregister&0x10000);  // ((Iregister>>16)&0x01); // ldb(Iregister,1,16);
	// int MInst_Byte_Op           = ((Iregister>>17)&0x03);    // ldb(Iregister,2,17);
	// Shared
	// Byte-specific
	MInst_Rotation_Len   = ((Iregister>>5)&0x1F);        // ldb(Iregister,5,5);
	MInst_Rotation_Count = (Iregister&0x1F);             // ldb(Iregister,5,0);

	MInst_Dest_Functnl   = ((Iregister>>25)&0x03F);      // ldb(Iregister,6,25);
	MInst_Cond_Invrt_Sns = (int)(Iregister&0x8000);     // ((Iregister>>15)&0x01);    // ldb(Iregister,1,15);
	MInst_Cond_ClRAM_Sel = (int)(Iregister&0x4000);     // ((Iregister>>14)&0x01);    // ldb(Iregister,1,14); 
	MInst_Cond_Select    = ((Iregister>>10)&0x0F);       // ldb(Iregister,4,10);

	// Whack some fields
	MInst_ALU_Opcode = 0;
	MInst_ALU_Carry_In = 0;

	alu_sub_stub(); // Do M-A-1
	alu_cleanup_result();

	// Run conditionalizer
	handle_condition_select();
      
	if(test_true == 1){
	  // Only handle ABJ if test true
	  handle_abj();
	}
      
	// ** DO WORK HERE **
	operate_shifter();

	// Select O-bus
      
	// Store result
	if(MInst_Dest_A_Select != 0){
	  // Store in A-Memory
          Amemory[MInst_Dest_AMem_Addr] = Obus;
	}else{
	  // Store in M-Memory
          Mmemory[MInst_Dest_MMem_Addr] = Obus;
          Amemory[MInst_Dest_MMem_Addr] = Obus;	  
	  // Handle MF bus
	  handle_m_fcn_dst();
	}
      }
      break;

    case 2: // JUMP-OP
      {
	// Fields
	int MInst_New_Micro_PC   = ((Iregister>>18)&0x3FFF);  // ldb(Iregister,14,18);
	int MInst_M_Source_Sel   = (int)(Iregister&0x20000); // ((Iregister>>17)&0x01);   // ldb(Iregister,1,17);
	/* More unused fields were here... */
	
	int MInst_Write_to_WCS   = (int)(Iregister&0x200);   // ((Iregister>>9)&0x01);    // ldb(Iregister,1,9);
	int MInst_Read_from_WCS  = (int)(Iregister&0x100);   // ((Iregister>>8)&0x01);    // ldb(Iregister,1,8);
	int MInst_Jump_Op        = ((Iregister>>5)&0x07);     // ldb(Iregister,3,5);
	// int MInst_Return_Bit     = ((Iregister>>7)&0x01);    // ldb(Iregister,1,7);
	// int MInst_Call_Bit       = ((Iregister>>6)&0x01);    // ldb(Iregister,1,6);
	// int MInst_Inhibit_Bit    = ((Iregister>>5)&0x01);    // ldb(Iregister,1,5);
	/* More unused fields were here */

	MInst_Cond_Invrt_Sns = (int)(Iregister&0x8000);  // ((Iregister>>15)&0x01);   // ldb(Iregister,1,15);
	MInst_Cond_ClRAM_Sel = (int)(Iregister&0x4000);     // ((Iregister>>14)&0x01);  // ldb(Iregister,1,14); 
	MInst_Cond_Select    = ((Iregister>>10)&0x0F);    // ldb(Iregister,4,10);

	// Whack some fields
	MInst_ALU_Opcode = 0;
	MInst_ALU_Carry_In = 0;

	if(MInst_Read_from_WCS != 0){
	  uint32 MInst_Dest_A_Select  = (int)(Iregister&0x80000000); 
	  int MInst_Dest_MMem_Addr = ((Iregister>>19)&0x03F); // WCS M bits (31:0) go here
	  int MInst_Dest_AMem_Addr = ((Iregister>>19)&0x3FF); // WCS A bits (56:32) go here

	  // See below for bad explanation of how this works

	  if((MCregister&0x800) == 0 && last_loc_ctr < 2048){
	    // Read PCS
	    if(MInst_Dest_A_Select == 0){
	      Mmemory[MInst_Dest_MMem_Addr] = PCS[last_loc_ctr]&0xFFFFFFFF; // ldb(PCS[last_loc_ctr],32,0);
	    }else{
	      Amemory[MInst_Dest_AMem_Addr] = (PCS[last_loc_ctr]>>32)&0xFFFFFF; // ldb(PCS[last_loc_ctr],24,32);
	    }
            #ifdef TRACELOG
	    if(tracerq){logmsgf("R-CS %ld = %LX\n",last_loc_ctr,PCS[last_loc_ctr]); }
            #endif
	  }else{
	    // Read WCS
	    if(MInst_Dest_A_Select == 0){
	      Mmemory[MInst_Dest_MMem_Addr] = WCS[last_loc_ctr]&0xFFFFFFFF; // ldb(WCS[last_loc_ctr],32,0);	      
	    }else{
	      Amemory[MInst_Dest_AMem_Addr] = (WCS[last_loc_ctr]>>32)&0xFFFFFF; // ldb(WCS[last_loc_ctr],24,32);
	    }
#ifdef I_PARITY_CHECK
	    if((MCregister&(MCR_Parity_Halt_Enable|MCR_Parity_Trap_Enable)) != 0){
	      // PARITY TEST
	      MInst_Parity = ((WCS[last_loc_ctr] >> 50)&0x1);  //     = ldb(Iregister,1,50);
              // logmsgf("read w/HE wcs@%lo=%llo, par bit %d\n", last_loc_ctr, WCS[last_loc_ctr], MInst_Parity);
	      if(gen_i_parity(WCS[last_loc_ctr]) != MInst_Parity){
                		logmsgf("WCS-READ PARITY ERROR STOP: %d\n",
                                        gen_i_parity(Iregister));
//		cpu_die_rq=1;
	      }
	    }
#endif
#ifdef TRACELOG
	    if(tracerq){logmsgf("R-CS %ld = %LX\n",last_loc_ctr,WCS[last_loc_ctr]); }
#endif
	  }

	  // Continue as nothing happened - No jump.
	  //	  cpu_die_rq=1;
	}

        // THIS MUST BE HERE IN CASE WCS DATA COMES FROM M-FCN
        /*
	if(MInst_M_Source_Addr > 077){          
	  handle_m_fcn_src();
	}
        */

	if(MInst_Write_to_WCS != 0){
	  // last-micro-PC is the destination to write.
	  // Data comes from A and M.
	  // Write WCS
          //          if(last_loc_ctr == 5693){ logmsgf("LOADING 5693\n"); cpu_die_rq=1; }
	  WCS[last_loc_ctr] = ((uint64)Abus << 32) | Mbus;

#ifdef I_PARITY_CHECK
	  {
	    int x = gen_i_parity(WCS[last_loc_ctr]);
	    WCS[last_loc_ctr] |= (uint64)x << 60;
	  }

	  if((MCregister&(MCR_Parity_Halt_Enable|MCR_Parity_Trap_Enable)) != 0){
	    // PARITY TEST
	    MInst_Parity = ((WCS[last_loc_ctr] >> 50)&0x1);  //     = ldb(Iregister,1,50);
            // logmsgf("write w/HE wcs@%lo=%llo, par bit %d\n", last_loc_ctr, WCS[last_loc_ctr], MInst_Parity);
	    if(gen_i_parity(WCS[last_loc_ctr]) != MInst_Parity){
	      logmsgf("WCS-WRITE PARITY ERROR STOP: %d\n",gen_i_parity(Iregister));
//	      cpu_die_rq=1;
	    }
	  }
#endif
	}

	/*
	if(MInst_Inhibit_Bit != 0){
	printw("INH ");
	// This is a pipeline thing - Kill the next instruction fetch
	// if it's currently occuring. It's not.
	}
	*/
	
	if(MInst_M_Source_Sel != 0){
	  logmsg("RAVEN: JUMP-ASEL (THIS IS BAD?)\n");
	  cpu_die_rq=1;
	}

	// Whack some fields
	MInst_ALU_Opcode = 0;
	MInst_ALU_Carry_In = 0;
	alu_sub_stub(); // Do M-A-1
	alu_cleanup_result();

	handle_condition_select();

        // ABJ on JUMP is an ELSE condition!
	if(test_true == 0){
          handle_abj();
	  break;
	}

        // Wait, what?
        //        if(MInst_Abbrv_Jump != 0){ logmsgf("JUMP W/ ABJ!\n"); cpu_die_rq=1; }

	/* CODES ARE:
	   R P I  (RETURN, PUSH, INHIBIT)
	   
	   0 0 0 = Branch-Xct-Next
	   0 0 1 = Branch
	   0 1 0 = Call-Xct-Next
	   0 1 1 = Call
	   1 0 0 = Return-Xct-Next
	   1 0 1 = Return
	   1 1 0 = NOP (JUMP2-XCT-NEXT)
	   1 1 1 = SKIP (JUMP2)

	   We ALWAYS load j-dst!

	*/

	switch(MInst_Jump_Op){
	case 6: // RAVFMT says Jump-Nop, THIS IS WRONG! It's really JUMP2-XCT-NEXT
	  // PUSH ADDRESS
	  uPCS_ptr_reg++;  uPCS_ptr_reg &= 0x3F;
	  uPCS_stack[uPCS_ptr_reg] = loc_ctr_reg;
	  // POP ADDRESS
	  loc_ctr_reg = uPCS_stack[uPCS_ptr_reg]&0xFFFFF;
	  uPCS_ptr_reg--;  uPCS_ptr_reg &= 0x3F;
	  // FALL THRU
	case 0: // Jump-Branch-Xct-Next
	  // Jump, but DO NOT inhibit the next instruction!
	  loc_ctr_nxt = MInst_New_Micro_PC;
	  break;

	case 7: // RAVFMT says Jump-Skip with is WRONG. It's really JUMP2
	  // PUSH ADDRESS
	  uPCS_ptr_reg++;  uPCS_ptr_reg &= 0x3F;
	  uPCS_stack[uPCS_ptr_reg] = loc_ctr_reg;
	  // POP ADDRESS
	  loc_ctr_reg = uPCS_stack[uPCS_ptr_reg]&0xFFFFF;
	  uPCS_ptr_reg--;  uPCS_ptr_reg &= 0x3F;
	  // FALL THRU
	case 1: // Jump-Branch
	  loc_ctr_reg = MInst_New_Micro_PC;
	  break;

	case 2: // Jump-Call-Xct-Next
	  // Call, but DO NOT inhibit the next instruction!
          // HACK HERE - IF TEST TRUE (we are here) AND ABJ-POPJ-NEXT, SIMPLY JUMP-AFTER-NEXT.
          if(MInst_Abbrv_Jump != 07){
            uPCS_ptr_reg++;  uPCS_ptr_reg &= 0x3F;
            uPCS_stack[uPCS_ptr_reg] = loc_ctr_reg+1; // Pushes the address of the next instruction
          }
          loc_ctr_nxt = MInst_New_Micro_PC;
	  break;	  

	case 3: // Jump-Call
	  // PUSH ADDRESS
	  uPCS_ptr_reg++;  uPCS_ptr_reg &= 0x3F;
	  uPCS_stack[uPCS_ptr_reg] = loc_ctr_reg;
	  // Jump
	  loc_ctr_reg = MInst_New_Micro_PC;
	  break;

	case 4: // Jump-Return-XCT-Next
	  // POP ADDRESS
	  loc_ctr_nxt = uPCS_stack[uPCS_ptr_reg]&0xFFFFF;
	  uPCS_ptr_reg--;  uPCS_ptr_reg &= 0x3F;
          // handle popj-14
          handle_popj_14_nxt();
	  break;

	case 5: // Jump-Return
	  // POP ADDRESS
	  loc_ctr_reg = uPCS_stack[uPCS_ptr_reg]&0xFFFFF;
	  uPCS_ptr_reg--;  uPCS_ptr_reg &= 0x3F;
          // IF INHIBIT IS SET, THE ADDRESS HAS ONE ADDED?
          // loc_ctr_reg++;
          // handle popj-14
          handle_popj_14();
	  break;

	default:
	  logmsgf("UNKNOWN-JUMP-OP(%d)\n",MInst_Jump_Op);
	  cpu_die_rq=1;
	}
      }
      break;

    case 3: // DISP-OP
      {
	// Fields
	uint32 MInst_Disp_Constant    = MInst_A_Source_Addr;
	int MInst_Disp_Address     = (Iregister>>20)&0xFFF;
	/* Unused fields were here... */
	// Alter-Retn-Addr means STACK-OWN-ADDRESS
	// If N is set, instead of putting the next instruction's address on the stack during a uPush, put ours.
	int MInst_Alter_Retn_Addr  = (int)(Iregister&0x20000); 

	int MInst_Enable_IStream   = (int)(Iregister&0x8000);  // ((Iregister>>15)&0x01);   // ldb(Iregister,1,15);
	int MInst_MIR              = (int)(Iregister&0x2000);
	int MInst_Disp_Source      = (Iregister>>12)&0x03;
	int MInst_Map_Oldspace     = (int)(Iregister&0x800);
	int MInst_Map_GC_Volatile  = (int)(Iregister&0x400);
	/* These are in DISP-OPCODE
	int MInst_Write_Disp_Mem   = (int)(Iregister&0x200);
	int MInst_Read_Disp_Mem    = (int)(Iregister&0x100);
	*/
	// MInst-Map-Enable is MAP-OLDSPACE and GC-VOLATILE together
	int MInst_Disp_Opcode      = (Iregister>>8)&0x03;
        uint32 Mask=0;
	int disp_address=0;
	uint32 disp_word=0;

	uint32 dispatch_source=0;
	uint32 mir_mask;
	int gc_volatilty_flag;
	int oldspace_flag;

        int live_abj=0;
        int MInst_New_Micro_PC=0;
        int MInst_Jump_Op=0;
	    
	// R bus controls
	MInst_Rotation_Directn = (int)(Iregister&0x10000); // ((Iregister>>16)&0x01); // ldb(Iregister,1,16);
	MInst_Rotation_Len   = ((Iregister>>5)&0x7);         // ldb(Iregister,3,5);
	MInst_Rotation_Count = (Iregister&0x1F);             // ldb(Iregister,5,0);
	// Force operation code
	MInst_Mask_Rotate = 0;
	MInst_Source_Rotate = 1; 

	mir_mask = MInst_MIR ? 0xc7f : 0xfff;

	switch(MInst_Disp_Source) {

	case 0: // Dispatch-Source-M-Source (Rbus)

	  Mask = (1 << MInst_Rotation_Len) - 1;
	  if (MInst_Map_Oldspace || MInst_Map_GC_Volatile)
		  Mask = Mask & 0xfffffffe;

	  if(MInst_Rotation_Directn == 0){
		  dispatch_source = left_rotate(Mbus, MInst_Rotation_Count) & Mask;
	  }else{
		  dispatch_source = right_rotate(Mbus, MInst_Rotation_Count) & Mask;
	  }
//operate_shifter();
//if ((MCregister & MCR_PROM_Disable) && MInst_Disp_Opcode == 0)
//logmsgf("Rbus %o, Mbus %o, Mask %o, dispatch_source %o\n", Rbus, Mbus, Mask, dispatch_source);
	  break;

	case 1: // Dispatch-Source-M-Tag
	  dispatch_source = ((Mbus >> 25) & 0x1f) << 1;
	  break;

	default: // MIR, MIR2
//logmsgf("MIR, MIR2!\n");
	  MInst_M_Source_Addr = 0112;
	  handle_m_fcn_src();
	  if ((MCregister & MCR_Misc_Op_Group_0) != 0 &&
	      (((1 ^ ((MCregister >> 25) & 1)) & ((Mbus >> 13) & 1)) == 0) &&
	      (((Mbus >> 9) & 0xf) == 0xd))
	  {
		  dispatch_source = 0x800 |
			  ((1 ^ ((Mbus >> 13) & 1)) << 9) | 
			  (Mbus & 0x1ff);
	  } else {
		  dispatch_source = 0xc00 | ((Mbus >> 6) & 0x3ff);
	  }
	  break;
	}

	gc_volatilty_flag = 0;
	if (MInst_Map_GC_Volatile) {
		uint32 map_1 = vm_lv1_map[(MDregister >> 13) & 0xfff];
		int map_1_volatility = ((map_1 >> 7) & 7);
		gc_volatilty_flag =
			(cached_gcv + 4 > (map_1_volatility ^ 7)) ? 0 : 1;
	}

	oldspace_flag = 0;
	if (MInst_Map_Oldspace) {
		uint32 map_1 = vm_lv1_map[(MDregister >> 13) & 0xfff];
		oldspace_flag = (map_1 & (1 << 10)) ? 1 : 0;
	}
	
	disp_address = (mir_mask & MInst_Disp_Address) |
		dispatch_source | gc_volatilty_flag | oldspace_flag;

	// Load dispatch-constant register
	disp_constant_reg = MInst_Disp_Constant;

	switch (MInst_Disp_Opcode) {
	  
	case 00: // DISPATCH
	  {
	    // disp_address points to an index into D-memory
	    disp_word = Dmemory[disp_address];

            if (do_inst_trace) {
              logmsgf("DEBUG: DISPATCH: CONSTANT %o, ADDRESS %o, SOURCE %lo, MEM[%lo] -> %lo\n",
                      MInst_Disp_Constant, MInst_Disp_Address,
                      dispatch_source, disp_address, disp_word);

              logmsgf("DEBUG: Abus %lo, Mask %o, Obus %lo, Rbus %lo, Mbus %lo\n",
                     Abus, Mask, Obus, Rbus, Mbus);
            }

	    /* DISPATCH-ACTION FORMAT:
	       0x03FFF = NEW-MICRO-PC
14	       0x04000 = N
15	       0x08000 = P
16	       0x10000 = R
	    */

	    MInst_New_Micro_PC = (disp_word&0x3FFF);
	    MInst_Jump_Op = ((disp_word&0x1C000)>>14);

            if (do_inst_trace) {
              logmsgf("DEBUG: DISPATCH: DW-OP %o DW-ADDR %lo (%ld) DW %lo (DMEM-ADDR %lo) ROT-Dn %o RTC %d RTL %d MBus %lo\n",
                      MInst_Jump_Op, MInst_New_Micro_PC,
                      MInst_New_Micro_PC, disp_word,
                      disp_address, MInst_Rotation_Directn,
                      MInst_Rotation_Count, MInst_Rotation_Len, Mbus);
            }

	    // instructon stream fetch
	    if (MInst_Enable_IStream) {

		    if (need_fetch) {
			    uint32 physaddr;
			    VMAregister = (LCregister >> 1) & 0x1ffffff;
			    physaddr = VM_resolve_address(VM_READ);
			    if(Page_Fault == 0){
                              lcbus_io_request(VM_READ,physaddr,0,NUBUS_ID_CPU);
			    }
		    }

		    LCregister++;
		    need_fetch = (LCregister & 1) == 0;
	    }

	    /*
	     * RPN
             * 000 %Jump-Branch-Xct-Next
             * 001 %Jump-Branch
             * 010 %Jump-Call-Xct-Next
             * 011 %Jump-Call
             * 100 %Jump-Return-Xct-Next
             * 101 %Jump-Return
             * 110 %Jump-Nop
             * 111 %Jump-Skip
	     */

	    switch(MInst_Jump_Op){
	      
	    case 0: // Jump-Branch-Xct-Next
	      // Jump, but DO NOT inhibit the next instruction!
	      loc_ctr_nxt = MInst_New_Micro_PC;
	      break;

	    case 1: // Jump-Branch
	      loc_ctr_reg = MInst_New_Micro_PC;
	      break;
	      
	    case 2: // Jump-Call-Xct-Next (P-Type)
	      uPCS_ptr_reg++;  uPCS_ptr_reg &= 0x3F;
              uPCS_stack[uPCS_ptr_reg] = loc_ctr_reg+1;
	      // Jump after next
	      loc_ctr_nxt = MInst_New_Micro_PC;
	      break;	  
	      
	    case 3: // Jump-Call
	      // PUSH ADDRESS (next), JUMP
	      uPCS_ptr_reg++;  uPCS_ptr_reg &= 0x3F;
#if 1
              if(MInst_Alter_Retn_Addr != 0){
		uPCS_stack[uPCS_ptr_reg] = loc_ctr_cnt;
	      }else{
		uPCS_stack[uPCS_ptr_reg] = loc_ctr_reg;
	      }
#else
	      uPCS_stack[uPCS_ptr_reg] = loc_ctr_reg;
#endif
	      // Jump
	      loc_ctr_reg = MInst_New_Micro_PC;
	      break;
	      
	    case 4: // Jump-Return-XCT-Next
	      loc_ctr_nxt = uPCS_stack[uPCS_ptr_reg]&0xFFFFF;
	      uPCS_ptr_reg--;  uPCS_ptr_reg &= 0x3F;
              handle_popj_14_nxt();
	      break;

	    case 5: // Jump-Return
	      // POP ADDRESS, JUMP
	      loc_ctr_reg = uPCS_stack[uPCS_ptr_reg]&0xFFFFF;
	      uPCS_ptr_reg--;  uPCS_ptr_reg &= 0x3F;
              handle_popj_14();
	      break;	      

	    case 6: // Jump-Nop
	      // PUSH ADDRESS
	      uPCS_ptr_reg++; uPCS_ptr_reg &= 0x3F;
	      uPCS_stack[uPCS_ptr_reg] = loc_ctr_reg;	      
	      // POP ADDRESS
	      loc_ctr_reg = uPCS_stack[uPCS_ptr_reg]&0xFFFFF;
	      uPCS_ptr_reg--;  uPCS_ptr_reg &= 0x3F;
              // LIVE ABJ
              live_abj = 1;
	      break;

	    case 7: // RAVFMT says Jump-Skip.
	      // Quoth the CPU manual: With R and P both 1,
              // the dispatch is ignored and the status of the next
              // instruction depends on N.
	      // In this case, Nhibit is set, so we skip.
	      // PUSH ADDRESS
	      uPCS_ptr_reg++; uPCS_ptr_reg &= 0x3F;
              uPCS_stack[uPCS_ptr_reg] = loc_ctr_reg+1;

	      // POP ADDRESS
	      loc_ctr_reg = uPCS_stack[uPCS_ptr_reg]&0xFFFFF;
	      uPCS_ptr_reg--;  uPCS_ptr_reg &= 0x3F;
	      // This is basically a skip. We don't fall through, like the JUMP version.
	      break;
	      
	    default:
	      logmsgf("UNKNOWN-DISPATCH-WORD-JUMP-OP(%d)\n",MInst_Jump_Op);
	      cpu_die_rq=1;
	    }
	  }
	  break;
	  
	case 01: // READ (AND FALL THROUGH)
	  Qregister = Dmemory[MInst_Disp_Address];
	  break;

	case 02: // WRITE (AND FALL THROUGH)
	  Dmemory[MInst_Disp_Address] = (Abus&0x1FFFF);
	  break;

	default:
	  logmsgf("UNKNOWN-DISP-OPCODE 0%o\n",MInst_Disp_Opcode);
	  cpu_die_rq=1;
	}

        if(live_abj != 0 && MInst_Abbrv_Jump != 0){
          if(MInst_Abbrv_Jump != 07 && MInst_Jump_Op != 06){
            logmsgf("RAVEN: LIVE ABJ W/ ABJ 0%o (Jump-Op %o)\n",MInst_Abbrv_Jump,MInst_Jump_Op);
            cpu_die_rq=1;
          }
          // EXPT breaks when this is enabled.
          handle_abj();
        }
      }
      break;
    }
    // Done parsing

#if 0
    if(pcss_jdst == 18 && pc_source_select == 1){
      logmsgf("[CPU] JUMP TO SELFTEST-ERROR FROM %ld\n",loc_ctr_reg);
      cpu_die_rq=1;
    }
#endif

#if 0
    if(pcss_jdst == 1215 && pc_source_select == 1){
      logmsgf("[CPU] JUMP TO MICROLOAD-ERROR FROM %ld\n",loc_ctr_reg);
      cpu_die_rq=1;
    }
#endif
    
    // Check for MD clobberage
    /*
    if((MDregister&0x3FFFFFFF)==0x2385018){
      logmsgf("MD Register Match Stop\n");
      cpu_die_rq=1;
    }
    */

    // Die if requested
    if(cpu_die_rq){
      cpu_die=1;
      logmsg("[CPU] MASTER CLOCK STOPPED\n");
      raven_dispupdate();
      disassemble_IR();
      disassemble_MIR();
    }

#ifdef TRACELOG
    if(do_inst_trace){
      debug_disassemble_IR();
      // usleep(1); // SSH will crash if spammed
    }
#endif

#ifdef DEI_TRACELOG
    if(loc_ctr_cnt == 1867){ // End of DEI-PUSH-DS
      logmsgf("DEBUG: DEI: D-STACK PUSH DATA %lX\n",Amemory[Mmemory[56]]); 
    }
#endif

#ifdef DISPLAY_SDL
    if ((cycle_count++ & 0x3fff) == 0) {
      display_poll();
    }
#endif

  }
  while(cpu_die != 0){
    sleep(1);
  }
}

void
disassemble_prom(void)
{
  int loc;
  uint64 inst;
  char line[256];

  for (loc = 0; loc < 2048; loc++) {
    inst = PCS[loc];
    disassemble(loc, inst, line);
    printf("%s\n", line);
  }
}

void
disassemble_range(int from, int to)
{
  int loc;
  uint64 inst;
  char line[256];

  for (loc = from; loc < to+1; loc++) {
    if((MCregister & MCR_PROM_Disable) || loc > 2048)
      inst = WCS[loc];
    else
      inst = PCS[loc];
    disassemble(loc, inst, line);
    printf("[%4d] %21llo %s\n", loc, inst, line);
  }
}



/*
 * Local Variables:
 * indent-tabs-mode:nil
 * c-basic-offset:2
 * End:
*/
