/** @file UdpServer.c
 *
 *  Version 0.3  22.1.2018
 *
 *  @author U. Lehnert, Helmholtz-Zentrum Dresden-Rossendorf
 *
 *  This is an UDP server running on the CAENels
 *  [FAST-PS](http://www.caenels.com/products/fast-ps/) power supplies
 *  for remote control of the devices.
 *  
 *  @section Functionality
 *  - listen for UDP Packets at port 16665
 *  - set voltage and current setpoints if requested
 *  - send reply with status, setpoints and readback values
 *
 *  At port 16665 setpoint data are received. Every packet received at this port
 *  is answered with a response packet showing the actual operation values.
 *  The setpoint package is ignored if set=0 is received but the
 *  response still is triggered.
 *  
 *  The received packet are expected to have the following content:
 *  - uint32_t magic;              // signature word expected 0x4C556543
 *  - uint32_t set;                // if set=0 the setpoints are not modified
 *  - int64_t current_setpoint;    // in uA
 *  - int64_t voltage_setpoint;    // in uV
 *  
 *  All valid packet are anwered with the following response:
 *  - uint32_t status;              // device status word
 *  - int64_t current_setpoint;     // current setpoint in uA
 *  - int64_t voltage_setpoint;     // voltage setpoint in uV
 *  - int64_t current_value;        // current readback value in uA
 *  - int64_t voltage_value;        // voltage readback value in uV
 * 
 *  @section Build
 *  The server is built with a cross-compiler running on a Linux system
 *  for the ARM target CPU of the power supplies.
 *  
 *  A makefile is not yet provided, just a few lines are required to build the server.
 *  - source ../tools/environment
 *  - $CXX -o udpserver UdpServer.c
 *
 *  @section Installation
 *  For istallation just the binary file needs to be copied onto the device:
 *  - udpserver binary installed in /tmp/ for testing
 *
 *  The server can then be run by executing /tmp/udpserver.
 * 
 *  @section Testing
 *  A Python script demonstrating the access is provided.
 *
 *  @section TODO
 *
 */

#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>		     // for flags
#include <stdlib.h>		     // for exit()
#include <signal.h>		     // for signal()
#include <errno.h>		     // for error messages
#include <string.h>
#include <math.h>
#include <time.h>

#include <sys/socket.h>      // for UDP communication
#include <netinet/udp.h>	 // declarations for udp header
#include <netinet/ip.h>		 // declarations for ip header
#include <arpa/inet.h>

// this variable is a flag for the running server
// when set to false the server stops
bool running = true;

// every loop includes a delay of 100000 ns to yield to other processes
#define DELAY 100000;

/***********************************/
/* interrupt and error handling    */
/***********************************/

// print error message and abort the running program
void Die(const char *mess)
{
    printf(mess);
    exit(1);
}

// handle SIGINT und SIGTERM
static void stopHandler(int signal)
{
    printf("UDP Server received ctrl-c\n");
    running = 0;
}

/***********************************/
/* TCP/IP communication            */
/***********************************/

int sock;
struct sockaddr_in tcpserver;

#define BUFSIZE 80
char command[BUFSIZE];			// command string buffer
char response[BUFSIZE];			// receive buffer

// send the string in command to the device
// receive the answer in response
// return the number of read characters
unsigned int TcpSendReceive() {
    unsigned int buflen = strlen(command);
    if (send(sock, command, buflen, 0) != buflen)
        Die("Mismatch in number of sent bytes\n");
    // receive the answer from the server
    unsigned int reclen;
    reclen = recv(sock, response, BUFSIZE-1, 0);
    response[reclen] = '\0';			// assure null terminated string
    if (reclen<3)
        printf("warning : less than 3 character received as device response to %s\n",command);
    return reclen;
}

/************************************/
/* UDP communication                */
/************************************/

#define UDPPORT 16665

// size of the send/receive buffers
#define UDPBUFLEN 256

// the data structure received by the PS
#define CONTROLSIZE 24
#define CONTROLMAGIC 0x4C556543
struct __attribute__((packed, aligned(4))) control_data {
   uint32_t magic;              // signature word
   uint32_t set;                // if set=0 the setpoints are not modified
   int64_t current_setpoint;    // in uA
   int64_t voltage_setpoint;    // in uV
};

// the data structure sent from the PS
#define RESPONSESIZE 36
struct __attribute__((packed, aligned(4))) response_data {
   uint32_t status;
   int64_t current_setpoint;
   int64_t voltage_setpoint;
   int64_t current_value;
   int64_t voltage_value;
};

// header structure needed for checksum calculation
struct pseudo_header
{
    u_int32_t source_address;
    u_int32_t dest_address;
    u_int8_t placeholder;
    u_int8_t protocol;
    u_int16_t udp_length;
};
 
// data structures for the UDP communication

// generic checksum calculation function
unsigned short csum(unsigned short *ptr, int nbytes)
{
    register long sum;
    unsigned short oddbyte;
    register short answer;
    sum=0;
    while(nbytes>1) {
        sum+=*ptr++;
        nbytes-=2;
    }
    if(nbytes==1) {
        oddbyte=0;
        *((unsigned char*)&oddbyte)=*(unsigned char*)ptr;
        sum+=oddbyte;
    }
    sum = (sum>>16)+(sum & 0xffff);
    sum = sum + (sum>>16);
    answer=(short)~sum;
    return(answer);
}

/***********************************/
/* main program                    */
/***********************************/

int main(int argc, char *argv[])
{

    // server will be running until we receive a SIGINT or SIGTERM
    signal(SIGINT,  stopHandler);
    signal(SIGTERM, stopHandler);

    //***********************************
    // connect to the internal TCP/IP server
    //***********************************

    if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
        Die("ERROR : Failed to create socket\n");
    // Construct the server sockaddr_in structure
    memset(&tcpserver, 0, sizeof(tcpserver));			   // clear struct
    tcpserver.sin_family = AF_INET;				           // Internet/IP
    tcpserver.sin_addr.s_addr = inet_addr("127.0.0.1");	   // IP address
    tcpserver.sin_port = htons(10001);				       // server port
    // Establish connection
    if (connect(sock, (struct sockaddr *) &tcpserver, sizeof(tcpserver)) < 0)
        Die("ERROR : Failed to connect to TCP/IP server\n");
    printf("UDP-Server : Connected to internal TCP/IP server.\n");

    //***********************************
    // open the UDP port
    //***********************************

    uint64_t udp_counter;           // count received/transmitted packets
    int udp_socket;                 // socket file descriptor

    // data structures for receiving UDP packets
    char udp_buffer[UDPBUFLEN];
    struct control_data *in_data;
    static struct sockaddr_in udp_server;
    static struct sockaddr_in udp_client;
    socklen_t slen = sizeof(udp_client);

    // data structures for sending response packets
    struct pseudo_header psh;	        // header for checksum calculation
    char pseudogram[UDPBUFLEN];	        // datagram for checksum calculation
    char send_buffer[UDPBUFLEN];	    // UDP send packet buffer
    // the IP header is at the beginning of the buffer
    struct iphdr *iph = (struct iphdr *) send_buffer;
    // the UDP header follows after the IP header
    struct udphdr *udph = (struct udphdr *) (send_buffer + sizeof(struct iphdr));
    // pointer to the payload within the data buffer
    struct response_data *databuffer = (struct response_data *)(send_buffer + sizeof(struct iphdr) + sizeof(struct udphdr));

    // create a UDP socket
    udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udp_socket == -1)
    {
        printf("failed to create socket\n");
        running = false;
    }
    // zero out the structure
    memset((char *) &udp_server, 0, sizeof(udp_server));
    udp_server.sin_family = AF_INET;
    udp_server.sin_port = htons(UDPPORT);
    udp_server.sin_addr.s_addr = htonl(INADDR_ANY);
    // bind socket to port
    if ( bind(udp_socket,(struct sockaddr*)&udp_server, sizeof(udp_server)) == -1 )
    {
        Die("failed to bind socket\n");
    }
    // set to non-blocking
    int flags = fcntl(udp_socket, F_GETFL);
    fcntl(udp_socket, F_SETFL, flags | O_NONBLOCK);

    printf("UDP-Server : UDP socket open.\n");

    struct timespec req, rem;
    req.tv_sec = 0;
    req.tv_nsec = DELAY;
    // the thread continues as long as the OPC server is running
    while (running)
    {

        // sleep for a while (be cooperative - yield to other processes)
        nanosleep(&req, &rem);

        // try to receive some data
        int recv_len = recvfrom(udp_socket, udp_buffer, UDPBUFLEN, 0, (struct sockaddr *) &udp_client, &slen);
        // if nothing is received
        if (recv_len == -1) continue;
        udp_counter++;
        if (recv_len!=CONTROLSIZE)
        {
            printf("Received unknown packet from %s:%d\n", inet_ntoa(udp_client.sin_addr), ntohs(udp_client.sin_port));
            continue;
        }
        in_data = (struct control_data *) udp_buffer;
        if (in_data->magic != CONTROLMAGIC)
        {
            printf("Received wrong magic %d from %s:%d, should be %d \n",
                in_data->magic, inet_ntoa(udp_client.sin_addr), ntohs(udp_client.sin_port),CONTROLMAGIC);
            continue;
        }
        if (in_data->set!=0)
        {
            int resplen;
            // send request to server
            // printf(" U = %g\n", 1.0e-6*in_data->voltage_setpoint);
            // printf(" I = %g\n", 1.0e-6*in_data->current_setpoint);
            sprintf(command,"MWV:%9.6f\r\n",1.0e-6*in_data->voltage_setpoint);
            resplen = TcpSendReceive();
            if(strncmp(response,"#AK",3)!=0)
                printf("error setting the voltage : %s\n",response);
            sprintf(command,"MWI:%9.6f\r\n",1.0e-6*in_data->current_setpoint);
            resplen = TcpSendReceive();
            if(strncmp(response,"#AK",3)!=0)
                printf("error setting the current : %s\n",response);
        }

        // clear the packet buffer
        memset(send_buffer, 0, UDPBUFLEN);
        // collect the response data
        strcpy(command,"MST\r\n");
        TcpSendReceive();
        sscanf(response+5,"%x",&(databuffer->status));
        strcpy(command,"MWI:?\r\n");
        TcpSendReceive();
        double isp;
        sscanf(response+5,"%lf",&isp);
        databuffer->current_setpoint=(int)round(1e6*isp);
        strcpy(command,"MWV:?\r\n");
        TcpSendReceive();
        double usp;
        sscanf(response+5,"%lf",&usp);
        databuffer->voltage_setpoint=(int)round(1e6*usp);
        strcpy(command,"MRI\r\n");
        TcpSendReceive();
        double irb;
        sscanf(response+5,"%lf",&irb);
        databuffer->current_value=(int)round(1e6*irb);
        strcpy(command,"MRV\r\n");
        TcpSendReceive();
        double urb;
        sscanf(response+5,"%lf",&urb);
        databuffer->voltage_value=(int)round(1e6*urb);
        // printf("sending UDP response\n");
        // fill in the IP Header
        iph->ihl = 5;
        iph->version = 4;
        iph->tos = 0;
        iph->tot_len = sizeof (struct iphdr) + sizeof (struct udphdr) + RESPONSESIZE;
        iph->id = udp_counter;               // Id of this packet
        iph->frag_off = 0;
        iph->ttl = 255;
        iph->protocol = IPPROTO_UDP;
        iph->check = 0;			             // set to 0 before calculating checksum
        iph->saddr = udp_server.sin_addr.s_addr;      // spoof the source IP address
        iph->daddr = udp_client.sin_addr.s_addr;      // receiver IP address
        // IP checksum
        iph->check = csum ((unsigned short *) send_buffer, iph->tot_len);
        // UDP header
        udph->source = udp_server.sin_port;
        udph->dest = udp_client.sin_port;
        udph->len = htons(8 + RESPONSESIZE);    // tcp header size
        udph->check = 0;                     // leave checksum 0 now, filled later from pseudo header
        // now compute the UDP checksum using the pseudo header
        psh.source_address = udp_server.sin_addr.s_addr;
        psh.dest_address = udp_client.sin_addr.s_addr;
        psh.placeholder = 0;
        psh.protocol = IPPROTO_UDP;
        psh.udp_length = htons(sizeof(struct udphdr) + RESPONSESIZE );
        memcpy(pseudogram , (char*) &psh , sizeof (struct pseudo_header));
        memcpy(pseudogram + sizeof(struct pseudo_header) , udph , sizeof(struct udphdr) + RESPONSESIZE);
        int psize = sizeof(struct pseudo_header) + sizeof(struct udphdr) + RESPONSESIZE;
        udph->check = csum( (unsigned short*) pseudogram , psize);
        //Send the packet
        sendto (udp_socket, send_buffer, iph->tot_len ,  0, (struct sockaddr *) &udp_client, sizeof (udp_client));

    }

    // the server has stopped running
    printf("UDP server stopped running.\n");

    // close TCP/IP connection
    close(sock);
    // close UDP connection
    close(udp_socket);

    printf("UDP Server : graceful exit\n");
    return 0;

}
