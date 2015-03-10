#include <arpa/inet.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/time.h>
#include <getopt.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <strings.h>
#include <string.h>
#include "rs232.h"
#include "routines.h"
#include "NazaDecoderLib.h"
#include <stdio.h>

#ifndef ATTITUDE_SENSING_DISABLED
	uint32_t currTime, attiTime;
#endif

#define CAM_CMD "/usr/local/bin/camera_streamer.sh"
char cam_cmd[128];
int verbose;

#define BUF_SIZE 1024 //receiving buffer
unsigned char buf[BUF_SIZE];
int portno = 1035;

#define MAX_CLIENTS 5
#define CLIENT_TIMEOUT 10
struct sockaddr_in raddress[MAX_CLIENTS];
int clients[MAX_CLIENTS];
int sock;
struct sockaddr_in tmpaddress;
socklen_t addrlen = sizeof(tmpaddress);

char cmd[256];
int background = 0;
int stop = 0;

int cam_active = 0;
unsigned int prev_fpvType;

void print_usage() {
	printf("-d run in background\n");
	printf("-c [path] path to camera_streamer.sh (defaults to %s)\n",CAM_CMD);
	printf("-p [port] port to listen on (defaults to %i)\n",portno);
	printf("-v [level] debug level\n");
}

void catch_signal(int sig)
{
	if (verbose) printf("Signal: %i\n",sig);
	stop = 1;
}

int check_client(struct sockaddr_in *c) {
	int i;
	for (i=0;i<MAX_CLIENTS;i++)
		if (clients[i]) //client active
			if ((c->sin_port == raddress[i].sin_port) &&
					(c->sin_addr.s_addr == raddress[i].sin_addr.s_addr)) {
				clients[i] = CLIENT_TIMEOUT; 
				return i;
			}

	return -1;
}

int add_client(struct sockaddr_in *c) {
	if (verbose) printf("New client... ");
	int i;

	for (i=0;i<MAX_CLIENTS && clients[i]!=0;i++); //find a free spot

	if (i==MAX_CLIENTS) {
		if (verbose) printf("All connections taken!\n");
		return -1;
	} else {
		clients[i] = CLIENT_TIMEOUT;
		memcpy(&raddress[i],c,addrlen);
	}

	if (verbose) printf("OK: %i\n",i);
	return i;
}

void startCam(unsigned char ip[4],int port, uint8_t type) {
	if (cam_active) {
		if (verbose) printf("Camera is already streaming!\n");
		return;
	}
	int ret;
	memset(cmd, '\0', 256);
	sprintf(cmd, "%s start %i.%i.%i.%i %i %i",cam_cmd, ip[0],ip[1],ip[2],ip[3],port,type);
	if (verbose) printf("Executing: %s\n",cmd);
	ret=system(cmd);

	if (ret==0) cam_active = 1;

	if (verbose) printf("Starting camera_streamer returned: %i\n",ret);
}

void stopCam() {
	if (!cam_active) {
		return;
	}
	int ret;
	memset(cmd, '\0', 256);
	sprintf(cmd, "%s stop",cam_cmd);
	if (verbose) printf("Executing: %s\n",cmd);
	ret = system(cmd);
	if (ret==0) cam_active = 0;
	if (verbose) printf("Stoping camera_streamer returned: %i\n",ret);
}

void processMsg(int client,unsigned char *buf, int len) {	
	unsigned char ip[4];
	int port;
	int tmp;
	unsigned int type;
	unsigned int fpvtype;

	type = buf[0];
	if (verbose) printf("Received type: %i\n",type);

	switch (type) {
		case 0: 
			memcpy(ip,buf+1,4);
			memcpy(&tmp,buf+5,4);
			port = ntohl(tmp);
			fpvtype = buf[9];
			if (prev_fpvType != fpvtype) {
				stopCam();
				prev_fpvType = fpvtype;
				startCam(ip,port,fpvtype);
			}
			if (verbose) printf("Received fpvtype: %i\n",fpvtype);				
			if (!cam_active) startCam(ip,port,fpvtype);
			break;
		case 1: stopCam(); 
			clients[client] = 0;
			break;
		default: printf("Unknown msg type received: %i\n",type);
	}

}

int main(int argc, char **argv)
{
	int max_fd;
	int i,ret;
	struct sockaddr_in address;
	struct timeval timeout;
	fd_set readfds;

	struct timespec time_now,time_prev;
	struct timespec *dt;
	long dt_ms = 0;
	verbose = 1;
	
	struct Telemetry
	{
		float volt;		// voltage, analog read (TODO)
		float amp;		// amperage, analog read (TODO)
		double lon;     // longitude in degree decimal
	    double lat;     // latitude in degree decimal
	    double gpsAlt;  // altitude in m (from GPS)
	    double spd;     // speed in m/s
	    double gpsVsi;  // vertical speed indicator (from GPS) in m/s (a.k.a. climb speed)
	} telem;

	int option;

	strcpy(cam_cmd,CAM_CMD);

	while ((option = getopt(argc, argv,"dc:p:v:")) != -1) {
		switch (option)  {
			case 'd': background = 1; verbose=0; break;
			case 'c': strcpy(cam_cmd,optarg); break;
			case 'p': portno = atoi(optarg);  break;
			case 'v': verbose = atoi(optarg); break;
			default:
				  print_usage();
				  return -1;
		}
	}

	signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);

	sock = 0;
	for (i=0;i<MAX_CLIENTS;i++)
		clients[i] = 0;

	sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (sock < 0) {
		perror("opening socket");
		exit(1);
	}

	/* Open Com Port  */
	int ci, cn,
			cport_nr=22,        /* /dev/ttyAMA0 */
			bdrate=115200;       /* 115200 baud */

		unsigned char cbuf[512];
		char cmode[]={'8','N','1',0};

	if(RS232_OpenComport(cport_nr, bdrate, cmode))
	{
		printf("Can not open comport\n");
		exit(1);
	}

	/* Create name. */
	bzero((char *) &address, sizeof(address));
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = INADDR_ANY;
	address.sin_port = htons(portno);

	if (bind(sock, (struct sockaddr *) &address, sizeof(struct sockaddr_in))) {
		perror("binding socket");
		exit(1);
	}
	if (verbose) printf("Socket created on port %i\n", portno);

	clock_gettime(CLOCK_REALTIME, &time_now);
	clock_gettime(CLOCK_REALTIME, &time_prev);

	if (background) {
		if (daemon(0,1) < 0) { 
			perror("daemon");
			return -1;
		}
		if (verbose) printf("Running in the background\n");
	}


	if (verbose) printf("Starting main loop\n");
	while (!stop) {
		FD_ZERO(&readfds);
		max_fd = 0;
		FD_SET(sock, &readfds);
		max_fd = sock;

		timeout.tv_sec = 0;
		timeout.tv_usec = 100*1000L; //every sec 
		int sel = select( max_fd + 1 , &readfds , NULL , NULL , &timeout);
		if ((sel<0) && (errno!=EINTR)) {
			perror("select");
			stop=1;
		}

		//check for orphan connections
		clock_gettime(CLOCK_REALTIME, &time_now);
		dt = TimeSpecDiff(&time_now,&time_prev);
		dt_ms = dt->tv_sec*1000 + dt->tv_nsec/1000000;
		if (dt_ms>1000) {
			time_prev = time_now;
			for (i=0;i<MAX_CLIENTS;i++) 
				if (clients[i]>0) {
					clients[i]--;
					if (clients[i]<=0) {
						if (verbose) printf("Client %i timeout.\n",i);
						stopCam();
					}
				}
		}


		if (!stop && FD_ISSET(sock, &readfds)) {
			ret = recvfrom(sock, buf, BUF_SIZE, 0, (struct sockaddr *)&tmpaddress, &addrlen);
			if (ret!=10) {
				printf("recvfrom error %i\n",ret);
				continue;
			}
			i = check_client(&tmpaddress);
			if (i<0) { //new client
				i=add_client(&tmpaddress);
			}
			if (i>=0) {
				if (verbose) printf("Client %i received %d bytes\n", i,ret);
				processMsg(i,buf,ret);
				sendto(sock, &telem, sizeof(telem), 0, (struct sockaddr *)&tmpaddress, addrlen);
			}

		}

		if (!stop) {
			cn = RS232_PollComport(cport_nr, cbuf, 512);

		    if(cn > 0)
		    {
		      	cbuf[cn] = 0;   /* always put a "null" at the end of a string! */
		    	unsigned char decodedMessage;

			    for(ci=0; ci < cn; ci++)
			    {
			        decodedMessage = NazaDecoder.decode(cbuf[ci]);
			    }

			    //printf("received %i bytes: %s\n", cn, (char *)cbuf);
			    switch (decodedMessage)
			    {
			      case NAZA_MESSAGE_GPS:
			        printf("Lat: %lf\n", NazaDecoder.getLat());
			        printf("Lon: %lf\n", NazaDecoder.getLon());
			        printf("Alt: %lf\n", NazaDecoder.getGpsAlt());
			        printf("Spd: %lf\n", NazaDecoder.getSpeed());
			        //printf("Fix: %s\n", NazaDecoder.getFixType());
			        printf("Sat: %c\n", NazaDecoder.getNumSat());
			        telem.volt 		= 3.5;
					telem.amp 		= 16.99;
					telem.lon 		= NazaDecoder.getLon();
				    telem.lat 		= NazaDecoder.getLat();
				    telem.gpsAlt 	= NazaDecoder.getGpsAlt();
				    telem.spd 		= NazaDecoder.getSpeed();
				    telem.gpsVsi 	= NazaDecoder.getGpsVsi();

			        break;
			      case NAZA_MESSAGE_COMPASS:
			        printf("Heading: %lf\n", NazaDecoder.getHeadingNc());
			        break;
			    }
		    }
		} else {
			printf("Can not open comport\n");
		}
	}

	if (verbose) {
		printf("closing\n");
		fflush(NULL);
	}

	stopCam();

	sleep(1);

	close(sock);
}