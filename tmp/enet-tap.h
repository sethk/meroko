extern u_char my_ether_addr[6];
extern int get_packet_socket(void);
extern int if_set_ether(int if_fd, u_char *addr);
extern void if_send_packet (int if_fd, u_char *packet, int packetlen);
extern int if_receive_poll(int if_fd);
extern int if_get_packet (int if_fd, u_char *buf, int buflen);

#if ENET_TAP_PTHREADS
extern int create_receiver_thread(int fd, u_char *buf, int buflen);
#endif
