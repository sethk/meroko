/* System Interface Board - Header

 This file is subject to the terms and conditions of the GNU General Public
 License.  See the file COPYING in the main directory of this archive for
 more details.

*/

void sib_init();
void sib_nubus_io();
extern void sib_updateslow();
extern inline void sib_clock_pulse();
extern unsigned char VRAM[0x20000];
extern void save_nvram();
extern unsigned char sib_mouse_motion_reg;
extern unsigned int sib_mouse_x_pos;
extern unsigned int sib_mouse_y_pos;
extern void kbd_handle_char(int sc, int sym, int updown, int shifted);

