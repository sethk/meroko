/* Local Bus header

 This file is subject to the terms and conditions of the GNU General Public
 License.  See the file COPYING in the main directory of this archive for
 more details.

*/

#include "types.h"

/* local bus Interface */
extern int LCbus_error;
extern int LCbus_acknowledge;
extern unsigned long LCbus_Address;
extern unsigned long LCbus_Request;
extern unsigned long LCbus_Data;

extern uint32 nubus_io_request(int access, uint32 address, uint32 data, int owner);
