#ifndef ETH_UDP_H
#define ETH_UDP_H

#include <stdio.h>
//#include <linux/socket.h>
#include <linux/types.h>
#include <linux/if_ether.h>
#include <linux/if_packet.h>

#include <sys/socket.h>
#include <unistd.h>	//чтобы не возникало предупреждения о том, что ф-ция close не объявлена
#include <netinet/in.h>	//чтобы не возникало предупреждения о том, что ф-ция htons не объявлена
#include <string.h>
#include <net/if.h>	//для ф-ции if_nametoindex(char*);

#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <inttypes.h>

extern int UDP_init(int* socket_descriptor, sockaddr_in* si, sockaddr_in* si_other, const char* __restrict IP, uint16_t port, char mode, int broadcast, int priority);
extern void UDP_deinit(int socket_descriptor);
extern int UDP_send_datagram(u_int32_t ip, uint16_t port, uint8_t* buf, uint32_t buf_len, int priority);
extern int UDP_send(int socket_descriptor, sockaddr_in* si_other, uint8_t* buf, uint32_t buf_len);
extern int UDP_recv(int socket_descriptor, sockaddr_in* si_other, uint8_t* buf, uint32_t buf_len);
extern void disp_recv_addr_info(sockaddr_in si_other);

#endif
