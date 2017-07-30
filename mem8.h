/* 8MB Memory Board - Header

 This file is subject to the terms and conditions of the GNU General Public
 License.  See the file COPYING in the main directory of this archive for
 more details.

*/


void mem8_init();
void mem8_nubus_io();
void mem8_lcbus_io();
extern unsigned char MEM_RAM[];
extern inline unsigned int genparity(unsigned char data);

/* NUbus (slave-only) Interface */
extern int Nubus_Busy; // NUbus Busy (On MEM8)
extern void nubus_io_pulse();

