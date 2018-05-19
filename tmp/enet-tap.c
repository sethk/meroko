#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <errno.h>

#include <sys/select.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <netpacket/packet.h>
#include <net/ethernet.h>     /* the L2 protocols */
#include <net/if.h>
#include <net/if_arp.h>
#include <netinet/in.h>
#include <linux/if_tun.h>

#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include "meroko.h"

#ifndef ENET_TAP_DEBUG
# define ENET_TAP_DEBUG 1
#endif

#ifndef ETHERTYPE_CHAOS
# define ETHERTYPE_CHAOS 0x0804
#endif

extern void ru_received_packet(u_char *pkt_buf, int pkt_size);

u_char my_ether_addr[6];	/* My Ethernet address */

static char ifname[IFNAMSIZ];	/* device */

/* Get a PACKET/DGRAM socket for the specified ethernet type, on the specified interface */
int
get_packet_socket(void)
{
  int fd, tap, ifix;
  struct ifreq ifr;

  memset(&my_ether_addr, 0, 6);
  if ((tap = open("/dev/net/tun", O_RDWR)) < 0) {
    perror("ENET: open /dev/net/tun");
    return -1;
  }

  memset(&ifr, 0, sizeof(ifr));
  ifr.ifr_flags = IFF_TAP | IFF_NO_PI;
  if (ioctl(tap, TUNSETIFF, &ifr) < 0) {
    perror("ENET: Couldn't set flags on tun device");
    return -1;
  }
  strcpy(ifname, ifr.ifr_name);
  /* Let the current user control the device */
  if (ioctl(tap, TUNSETOWNER, getuid()) < 0)
    perror("ioctl(TUNSETOWNER)");

  /* Need a socket, just any socket (but INET, I guess) */
  if ((fd = socket(PF_INET, SOCK_DGRAM, 0)) < 0) {
    perror("ENET: socket(PF_PACKET, SOCK_DGRAM)");
    return -1;
  }
  
  /* Get index */
  memset(&ifr, 0, sizeof(ifr));
  strncpy(ifr.ifr_name, ifname, strlen(ifname));
  if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
    perror("ENET: ioctl(SIOCGIFINDEX)");
    close(fd);			/* cleanup: close socket */
    return -1;
  }
  ifix = ifr.ifr_ifindex;

#if 0
  printf("ENET: ifindex %d\n", ifix);
#endif

  /* Get MAC address */
  memset(&ifr, 0, sizeof(ifr));
  strncpy(ifr.ifr_name, ifname, strlen(ifname));
  if (ioctl(fd, SIOCGIFHWADDR, &ifr) < 0 ) {
    perror("ENET: ioctl(SIOCGIFHWADDR)");
    close(fd);			/* cleanup: close socket */
    return -1;
  }

  if (ifr.ifr_hwaddr.sa_family != ARPHRD_ETHER) {
    fprintf(stderr,"wrong ARPHDR %d ", ifr.ifr_hwaddr.sa_family);
    perror("ENET: ioctl");
    close(fd);			/* cleanup: close socket */
    return -1;
  }

  memcpy(&my_ether_addr, ifr.ifr_hwaddr.sa_data, ETH_ALEN);
#if ENET_TAP_DEBUG
  printf("ENET: HW addr %02x:%02x:%02x:%02x:%02x:%02x\n",
	 my_ether_addr[0],my_ether_addr[1],my_ether_addr[2],
	 my_ether_addr[3],my_ether_addr[4],my_ether_addr[5]);
#endif

  close(fd);			/* cleanup: close socket */
  return tap;
}

/* Send a packet of the specified type to the specified address  */
void
if_send_packet (int if_fd, u_char *packet, int packetlen)
{
  int cc;

#if ENET_TAP_DEBUG
  logmsgf("ENET: Sending %d bytes\n", packetlen);
#endif
  cc = write(if_fd, packet, packetlen);

  if (cc == packetlen)
    return;
  else if (cc >= 0)
    logmsgf("ENET: if_send_packet sent only %d bytes\n", cc);
  else
    {
      perror("ENET: if_send_packet");
    }
}

int if_receive_poll(int if_fd)
{
  struct pollfd pfd[1];
  int v;

  pfd[0].fd = if_fd;
  pfd[0].events = POLLIN;
  pfd[0].revents = 0;
  if ((v = poll(pfd, 1, 0)) < 0) {
    perror("ENET: poll error");
    return 0;
  }
  else if (pfd[0].revents & (POLLERR | POLLHUP | POLLNVAL)) {
    logmsgf("ENET: poll error bits 0x%x\n",
	    pfd[0].revents & (POLLERR | POLLHUP | POLLNVAL));
    return 0;
  }
  else
    return v;
}

/* Read a packet from the socket. */
int
if_get_packet (int if_fd, u_char *buf, int buflen)
{
  int rlen;

  rlen = read(if_fd, buf, buflen);
  if (rlen < 0)
    {
      if (errno != EAGAIN)
	perror ("ENET: if_get_packet: Read error");
      return 0;
    }
  if (rlen == 0)
    return 0;
#if ENET_TAP_DEBUG
  logmsgf("ENET: Received %d bytes\n", rlen);
#endif
  return rlen;
}

#if 1 // ENET_TAP_PTHREADS
#include <pthread.h>

static pthread_mutex_t receiver_mutex;
static pthread_t receiver_thread;

static
struct rec_thr_args {
  int rec_fd;
  u_char *rec_buf;
  int rec_buf_len;
} rec_thr_state;

/* Consider having more than one buffer? */
static void *
receiver_thread_process(void *argp)
{
  struct rec_thr_args *args = argp;
  int fd = args->rec_fd;
  u_char *buf = args->rec_buf;
  int buflen = args->rec_buf_len;
  int rlen = 0;
  while ((rlen = if_get_packet(fd, buf, buflen)) >= 0) {
    if (rlen > 0)
      ru_received_packet(args->rec_buf, rlen);
  }
  perror("ENET: if_get_packet (receiver thread)");
  pthread_exit(NULL);
}

int
create_receiver_thread(int fd, u_char *buf, int buflen)
{
  if (pthread_mutex_init(&receiver_mutex, NULL) != 0) {
    perror("ENET: pthread_mutex_init");
    return 0;
  }
  /* Set up args for thread */
  rec_thr_state.rec_fd = fd;
  rec_thr_state.rec_buf = buf;
  rec_thr_state.rec_buf_len = buflen;
  if (pthread_create(&receiver_thread, NULL,
		     &receiver_thread_process, &rec_thr_state) != 0) {
    perror("ENET: pthread_create");
    return 0;
  }
  return 1;
}
#endif
