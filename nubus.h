/* NuBUS header

 This file is subject to the terms and conditions of the GNU General Public
 License.  See the file COPYING in the main directory of this archive for
 more details.

*/



/* Operations */
// CPU MASTER WORD IO
#define VM_READ  0x00
#define VM_WRITE 0x01
// CPU MASTER BYTE IO
#define VM_BYTE_READ  0x02
#define VM_BYTE_WRITE 0x03

// BUS MASTER WORD IO
#define NB_READ  0x10
#define NB_WRITE 0x11
// BUS MASTER BYTE IO
#define NB_BYTE_READ  0x12
#define NB_BYTE_WRITE 0x13

/* NuBus card id's */
#define NUBUS_ID_NUPI	0xF2 // NuPI
#define NUBUS_ID_MEM8	0xF4 // Memory
#define NUBUS_ID_SIB	0xF5 // SIB	
#define NUBUS_ID_CPU	0xF6 // CPU

/* NUbus Interface */
extern int NUbus_error;
extern int Nubus_Busy;
extern int NUbus_acknowledge;
extern unsigned long NUbus_Address;
extern unsigned long NUbus_Request;
extern unsigned long NUbus_Data;

extern uint32 nubus_io_request(int access, uint32 address, uint32 data, int owner);
