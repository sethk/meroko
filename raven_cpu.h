/* Raven (Explorer 1) CPU - Header

 This file is subject to the terms and conditions of the GNU General Public
 License.  See the file COPYING in the main directory of this archive for
 more details.

*/


void raven_initialize();
void raven_clockpulse();
extern void raven_nubus_io();
extern void raven_disp_init();
void raven_dispupdate();
void raven_halt();
void raven_trap();
void raven_step();
void raven_cont();
void raven_dump();

extern int cpu_die_rq;
extern unsigned int ldb(unsigned long long value, int size, int position);

