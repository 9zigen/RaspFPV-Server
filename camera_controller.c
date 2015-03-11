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
#include <assert.h>
#include <string.h>
#include "rs232.h"
#include "routines.h"
#include "NazaDecoderLib.h"
#include <stdio.h>

#define pi 3.14159265358979323846;

int ret;
int err = 0;
int stop = 0;

int sock;
struct sockaddr_in address;
int portno = 1035;
char data[256];
int background = 0;
char pidfile[256] = "\0";
int verbose = 0;

int yprt[4] = {0, 0, 0, 0};

long dt_ms = 0;
static struct timespec ts, t1, t2, t3, *dt, lastPacketTime;

int recording = 0;

int logmode = 0;

#define CAMERA_CMD "/usr/local/bin/camera.sh"

#define FIXED_HEIGHT 240

struct Telemetry
	{
		float volt;		// voltage, analog read (TODO)
		float amp;		// amperage, analog read (TODO)
		double lon;     // longitude in degree decimal
	    double lat;     // latitude in degree decimal
	    double gpsAlt;  // altitude in m (from GPS)
	    double spd;     // speed in m/s
	    double gpsVsi;  // vertical speed indicator (from GPS) in m/s (a.k.a. climb speed)
	    double heading;	// heading
        bool gotHome;   // Have Home point
        double hDop;    // horizontal dilution of precision
        double vDop;    // vertical dilution of precision
        char fixType;
        double altHome;
        double homeDistance;
        double homeLat;
        double homeLon;
        int homeDirection;
	} telem;

int ci, cn,
	cport_nr=22,        /* /dev/ttyAMA0 */
	bdrate=115200;       /* 115200 baud */

unsigned char cbuf[512];
char cmode[]={'8','N','1',0};

double deg2rad(double deg) {
  return (deg * pi / 180);
}

double rad2deg(double rad) {
  return (rad * 180 / pi);
}

char** str_split(char* a_str, const char a_delim) {
    char** result = 0;
    size_t count = 0;
    char* tmp = a_str;
    char* last_comma = 0;
    char delim[2];
    delim[0] = a_delim;
    delim[1] = 0;

    /* Count how many elements will be extracted. */
    while (*tmp) {
        if (a_delim == *tmp) {
            count++;
            last_comma = tmp;
        }
        tmp++;
    }

    /* Add space for trailing token. */
    count += last_comma < (a_str + strlen(a_str) - 1);

    /* Add space for terminating null string so caller
       knows where the list of returned strings ends. */
    count++;

    result = (char**) malloc(sizeof (char*) * count);

    if (result) {
        size_t idx = 0;
        char* token = strtok(a_str, delim);

        while (token) {
            assert(idx < count);
            *(result + idx++) = strdup(token);
            token = strtok(0, delim);
        }
        assert(idx == count - 1);
        *(result + idx) = 0;
    }

    return result;
}

const char * handle_packet(char * data, sockaddr_in remoteAddr) {

    char** tokens = str_split(data, ' ');
    if (!tokens)
        return "KO 0";

    char * resp = (char*) malloc(sizeof (char) * 255);
    memset(resp, '\0', 255);

    clock_gettime(CLOCK_REALTIME, &lastPacketTime);

    if (strcmp(tokens[0], "hello") == 0) {

    } else if (strcmp(tokens[0], "stream") == 0) {
        char *ip = inet_ntoa(remoteAddr.sin_addr);

        // handle aspect ratio
        float aspectratio = atof(tokens[4]) / atof(tokens[5]);
        int height = FIXED_HEIGHT;
        int width = (aspectratio * height);

        char cmd[256];
        memset(cmd, '\0', 256);
        sprintf(cmd, "%s %s %s %s %s %i %i &", CAMERA_CMD, tokens[0], tokens[2], ip, tokens[3], width, height);
        if (verbose) printf("Executing: %s\n", cmd);
        ret = system(cmd);

        if (ret != 0) {
            snprintf(resp, 255, "KO %s %s", tokens[1], tokens[2]);
            return resp;
        } else {
            snprintf(resp, 255, "OK %s %s", tokens[1], tokens[2]);
            return resp;
        }
    } else if (strcmp(tokens[0], "heartbeat") == 0) {

    } else if (strcmp(tokens[0], "takepicture") == 0) {
        char timeString[128];
        timeval curTime;
        gettimeofday(&curTime, NULL);
        strftime(timeString, 80, "%Y%m%d_%H%M%S", localtime(&curTime.tv_sec));

        //take picture
        char cmd[128];
        memset(cmd, '\0', 128);
        sprintf(cmd, "%s %s %s &", CAMERA_CMD, tokens[0], timeString);
        ret = system(cmd);
    } else if (strcmp(tokens[0], "vidsnap") == 0) {
        if (strcmp(tokens[2], "record") == 0) {
            char timeString[128];
            timeval curTime;
            gettimeofday(&curTime, NULL);
            strftime(timeString, 80, "%Y%m%d_%H%M%S", localtime(&curTime.tv_sec));

            char cmd[128];
            memset(cmd, '\0', 128);
            sprintf(cmd, "%s video %s %s &", CAMERA_CMD, tokens[2], timeString);
            ret = system(cmd);
            recording = 1;
        } else if (strcmp(tokens[2], "stop") == 0) {
            char cmd[128];
            memset(cmd, '\0', 128);
            sprintf(cmd, "%s video %s &", CAMERA_CMD, tokens[2]);
            ret = system(cmd);
            recording = 0;
        } else if (strcmp(tokens[2], "pause") == 0) {
            char cmd[128];
            memset(cmd, '\0', 128);
            sprintf(cmd, "%s video %s &", CAMERA_CMD, tokens[2]);
            ret = system(cmd);
            recording = 2;
        }
    } else if (strcmp(tokens[0], "querystatus") == 0) {
        // yaw pitch roll altitudetarget altitude recording
        //snprintf(resp, 255, "status %.6f %.6f %.6f %.6f %i %i %i %i %i %i", telem.lon, telem.lat, telem.gpsAlt, telem.spd, telem.gpsVsi, avr_s[LOG_ALTITUDE], recording, avr_s[LOG_MOTOR_FL], avr_s[LOG_MOTOR_BL], avr_s[LOG_MOTOR_BR], avr_s[LOG_MOTOR_FR]);
        return resp;
    }
    snprintf(resp, 255, "OK %s", tokens[1]);

    return resp;
}

void recvNaza() {
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
                if (verbose) {
                    printf("Lat: %lf\n", NazaDecoder.getLat());
                    printf("Lon: %lf\n", NazaDecoder.getLon());
                    printf("Alt: %lf\n", NazaDecoder.getGpsAlt());
                    printf("Spd: %lf\n", NazaDecoder.getSpeed());
                    printf("Vsi: %lf\n", NazaDecoder.getGpsVsi());
                    printf("Cog: %lf\n", NazaDecoder.getCog());
                    printf("Fix: %d\n", (char) NazaDecoder.getFixType());
                    printf("Sat: %d\n", NazaDecoder.getNumSat());
                    printf("Hdop: %lf\n", NazaDecoder.getHdop());
                    printf("Vdop: %lf\n", NazaDecoder.getVdop());

                    printf("Year: %d\n", NazaDecoder.getYear());
                    printf("Month: %d\n", NazaDecoder.getMonth());
                    printf("Day: %d\n", NazaDecoder.getDay());
                    printf("Hour: %d\n", NazaDecoder.getHour());
                    printf("Minute: %d\n", NazaDecoder.getMinute());
                    printf("Second: %d\n", NazaDecoder.getSecond());
                }
                
                telem.lon       = NazaDecoder.getLon();
                telem.lat       = NazaDecoder.getLat();
                telem.gpsAlt    = NazaDecoder.getGpsAlt();
                telem.spd       = NazaDecoder.getSpeed();
                telem.gpsVsi    = NazaDecoder.getGpsVsi();
                telem.fixType   = (char) NazaDecoder.getFixType();
                telem.hDop      = NazaDecoder.getHdop();
                telem.vDop      = NazaDecoder.getVdop();
                break;
            case NAZA_MESSAGE_COMPASS:
                if (verbose) { printf("Heading: %lf\n", NazaDecoder.getHeadingNc()); }
                telem.heading = NazaDecoder.getHeadingNc();

                break;
        }
    }
}

void setHomeVars()
{
    long bearing;
    
    if(telem.gotHome == 0 && telem.fixType > 2){
        telem.homeLat = telem.lat;
        telem.homeLon = telem.lon;
        telem.gotHome = 1;
        
        if (telem.vDop < 4) {
            telem.altHome = telem.gpsAlt;
        }
    }
    else if(telem.gotHome == 1){
        // DST to HOME
        double theta, dist, head, bearing;

        theta = telem.homeLon - telem.lon;
        dist = sin(deg2rad(telem.homeLat)) * sin(deg2rad(telem.lat)) + cos(deg2rad(telem.homeLat)) * cos(deg2rad(telem.lat)) * cos(deg2rad(theta));
        dist = acos(dist);
        dist = rad2deg(dist);
        dist = dist * 60 * 1.1515 * 1.609344; // Kilometers
        telem.homeDistance = dist;
        if (verbose) { printf("Home Distance: %lf\n", telem.homeDistance); }

        //DIR to Home
        head = atan2((sin(deg2rad(theta)) * cos(deg2rad(telem.lat))), ((cos(deg2rad(telem.homeLat)) * sin(deg2rad(telem.lat))) - (sin(deg2rad(telem.homeLat)) * cos(deg2rad(telem.lat)) * cos(deg2rad(theta)))));
        bearing = (head * 180)/pi;
        if(bearing <= 0)
        {
            bearing = bearing + 360;
        }
        telem.homeDirection = bearing;
        if (verbose) { printf("Absolute Home Direction: %i\n", telem.homeDirection); }
        if (verbose) { printf("Relative Home Direction: %lf\n", telem.homeDirection - telem.heading); }
    }
}

void recvMsgs() {
    static int sel = 0;

    static fd_set fds;
    static struct timeval timeout;

    do {
        FD_ZERO(&fds);
        FD_SET(sock, &fds);
        timeout.tv_sec = 0;
        timeout.tv_usec = 0; // If the timeout argument points to an object of type struct timeval whose members are 0, select() does not block
        int max_fd = sock;
        sel = select(max_fd + 1, &fds, NULL, NULL, &timeout);
        if ((sel < 0) && (errno != EINTR)) {
            perror("select");
            err = 1;
        } else if (sel && !err && !stop && FD_ISSET(sock, &fds)) {
            sockaddr_in remoteAddr;
            int received_bytes = -1;
            //sockaddr_in from;
            socklen_t fromLength = sizeof (remoteAddr);

            received_bytes = recvfrom(sock, data, sizeof (data), 0, (sockaddr*) & remoteAddr, &fromLength);
            if (received_bytes == -1) {
                perror("received bytes = -1 \n");
            } else {
                data[received_bytes] = 0;

                if (verbose >= 3) {
                    printf("Received: %s\n", data);
                }

                const char* resp = handle_packet(data, remoteAddr);

                if (strlen(resp) > 0) {
                    if (verbose >= 2) {
                        printf("Send response: %s\n", resp);
                    }

                    if (sendto(sock, resp, strlen(resp), 0, (sockaddr*) & remoteAddr, (socklen_t) sizeof (remoteAddr)) == -1) {
                        perror(strerror(errno));
                    }
                }
            }
        }
    } while (!err && !stop && sel); //no error happened; select picked up socket state change; read got some data back
}

void catch_signal(int sig) {
    printf("signal: %i\n", sig);
    stop = 1;
}

unsigned long k = 0;

void loop() {
    clock_gettime(CLOCK_REALTIME, &t3);
    lastPacketTime = ts = t1 = t2 = t3;
    if (verbose) printf("Starting main loop...\n");

    while (!err && !stop) {

        // if no packet for 5sec maybe lost connection
        clock_gettime(CLOCK_REALTIME, &t2);
        dt = TimeSpecDiff(&t2, &lastPacketTime);
        dt_ms = dt->tv_sec * 1000 + dt->tv_nsec / 1000000;
        if (dt_ms > 2500) {
            printf("lost connection\n");
            lastPacketTime = t2;
        }

        recvNaza();
        recvMsgs();
        usleep(100000);
    }
}

void print_usage() {
    printf("-d - run as daemon\n");
    printf("-i [file] - PID file\n");
    printf("-v [level] - verbose mode\n");
    printf("-l [port] - port to listen on (defaults to 1032)\n");
}

int main(int argc, char **argv) {

    signal(SIGTERM, catch_signal);
    signal(SIGINT, catch_signal);

    int option;
    verbose = 0;
    while ((option = getopt(argc, argv, "di:v:u:l:")) != -1) {
        switch (option) {
            case 'd': background = 1;
                break;
            case 'i': strcpy(pidfile, optarg);
                break;
            case 'v': verbose = atoi(optarg);
                break;
            case 'l': portno = atoi(optarg);
                break;
            default:
                print_usage();
                return -1;
        }
    }

    // create address for incoming udp connection
    bzero((char *) &address, sizeof (address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(portno);

    /* Create socket to listen */
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        perror("opening server socket");
        exit(EXIT_FAILURE);
    }

    //Binding to desired port number
    if (bind(sock, (struct sockaddr *) &address, sizeof (struct sockaddr_in))) {
        close(sock);
        perror("binding stream socket");
        exit(EXIT_FAILURE);
    }

    //setting Socket to non blocking mode
    int nonBlocking = 1;
    if (fcntl(sock, F_SETFL, O_NONBLOCK, nonBlocking) == -1) {
        close(sock);
        perror("failed to set non-blocking socket");
        exit(EXIT_FAILURE);
    }

    // open com port
    if(RS232_OpenComport(cport_nr, bdrate, cmode))
    {
        printf("Can not open comport\n");
        exit(EXIT_FAILURE);
    }

    // run as daemon
    if (background) {
        if (daemon(0, 0) < 0) {
            close(sock);
            perror("unable to start as daemon");
            exit(EXIT_FAILURE);
        }
        if (strcmp(pidfile, "") != 0) {
            int fd = open(pidfile, O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
            if (ftruncate(fd, 0) == -1)
                printf("Could not truncate PID file '%s'", pidfile);

            char buf[256];
            snprintf(buf, 256, "%ld\n", (long) getpid());
            if ((size_t)write(fd, buf, strlen(buf)) != strlen(buf))
                printf("Writing to PID file '%s'", pidfile);
            close(fd);
        }
    }

    while (!stop) {
        err = 0;
        if (verbose) printf("Opening socket...\n");

        // discard all received packet until avrspi is not connected (don't flood!)
        int udpBufSize = 0;
        socklen_t optlen = sizeof(udpBufSize);
        setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &udpBufSize, optlen);

        loop();

        if (err)
            sleep(2);
    }
    close(sock);
    if (verbose) printf("Closing.\n");
    exit(EXIT_SUCCESS);
}
