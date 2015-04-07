#include <gst/gst.h>
#include <glib.h>
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
#include "spi.h"
#include <stdio.h>

int ret;
int err = 0;
int stop = 0;

int sock;
struct sockaddr_in address;
int portno = 1035;
char data_packet[256];
int background = 0;
char pidfile[256] = "\0";
int verbose = 0;

int yprt[4] = {0, 0, 0, 0};

long dt_ms = 0;
static struct timespec ts, t1, t2, t3, t4, t5, adc, *dt, lastPacketTime;

int recording = 0;

int logmode = 0;

#define CAMERA_CMD "/usr/local/bin/camera.sh"

#define FIXED_HEIGHT 240

GST_DEBUG_CATEGORY (gst_slxgpu_debug);
#define GST_CAT_DEFAULT gst_slxgpu_debug

/* Structure to contain all our information, so we can pass it to callbacks */
typedef struct _CustomData {
  GstElement *pipeline;
  GstElement *source;
  GstElement *tee;
  GstElement *stream_queue;
  GstElement *parse;
  GstElement *pay;
  GstElement *stream_sink;
  GstElement *file_queue;
  GstElement *file_parse;
  GstElement *file_sink;
  GstElement *filter;
} CustomData;

CustomData data;

static const int ADC_MAX = 1023; // 10bit ADC
static const double DEFAULT_SENSOR_MAX_VOLTS = 51.8;
static const double DEFAULT_SENSOR_MAX_AMPS = 89.4;
static const void * NO_SPI = (void*)1;

#define ANALOG_SAMPLES 50

struct AnalogTelemetry {
    SPIInterface *spi;
    int spi_bus;
    int spi_device;

    int voltage_channel;
    int current_channel;

    double max_volts;
    double max_amps;
};

struct Telemetry
    {
        double volt;    // voltage, analog read SPI (mcp3002)
        double amp;     // amperage, analog read SPI (mcp3002)
        double lon;     // longitude in degree decimal
        double lat;     // latitude in degree decimal
        double gpsAlt;  // altitude in m (from GPS)
        double spd;     // speed in m/s
        double gpsVsi;  // vertical speed indicator (from GPS) in m/s (a.k.a. climb speed)
        double heading; // heading
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

int startStream(char addr[15], int port) {
    
    g_object_set(data.stream_sink, "port", port, "host", addr, NULL);
    return 1;
}
int startRecord() {
    GstPad *sinkpad, *srcpad;

    char timeString[128];
    timeval curTime;
    gettimeofday(&curTime, NULL);
    strftime(timeString, 80, "%Y%m%d_%H%M%S", localtime(&curTime.tv_sec));
    
    char location[256];
    memset(location, '\0', 256);
    sprintf(location, "/rpicopter/cam/video-%s.h264", timeString);
    //sprintf(location, "/tmp/video-%s.h264", timeString);
    
    if (verbose) printf("Executing: start record %s", location);

    g_object_set(data.file_sink, "location", location, NULL);
    //g_object_set(data.file_sink, "location", "/tmp/eww.h264", NULL);
    GstElement *branch = gst_bin_new ("recording_bin"); 

    gst_bin_add_many(GST_BIN(branch), data.file_queue, data.file_parse, data.file_sink, NULL); 
    gst_element_link_many(data.file_queue, data.file_parse, data.file_sink, NULL); 

    sinkpad = gst_element_get_static_pad(data.file_queue, "sink"); 
    gst_element_add_pad(branch, gst_ghost_pad_new("sink", sinkpad)); 
    gst_object_unref(GST_OBJECT(sinkpad)); 

    // set the new bin to PAUSE to preroll? 
    gst_element_set_state(branch, GST_STATE_PAUSED); 

    // get regular pipeline pre-existing tee element 
    GstElement *tee = gst_bin_get_by_name (GST_BIN(data.pipeline), "tee"); 

    sinkpad         = gst_element_get_static_pad(branch, "sink"); 
    srcpad          = gst_element_get_request_pad(tee, "src_%u"); 

    //gst_pad_set_blocked(srcpad, TRUE); 

    // the main pipeline becomes paused (because the new one is paused) 
    gst_bin_add(GST_BIN(data.pipeline), branch); 
    gst_pad_link(srcpad, sinkpad);
    g_print ("Received new pad '%s'", GST_PAD_NAME (srcpad));

    gst_object_unref(GST_OBJECT(srcpad)); 
    gst_object_unref(GST_OBJECT(sinkpad));
    gst_object_unref(GST_OBJECT(branch)); 
    gst_object_unref(GST_OBJECT(tee)); 

    printf("set to PLAYING");
    ret = gst_element_set_state (data.pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        g_printerr ("Unable to set the pipeline and branch to the playing state.\n");
    }
    //gst_element_set_state(GST_ELEMENT(data.pipeline), GST_STATE_PLAYING); 
    return 1;
}

int stopRecording() {
    GstElement *branch  = gst_bin_get_by_name(GST_BIN(data.pipeline), "recording_bin"); 
    GstElement *tee     = gst_bin_get_by_name(GST_BIN(data.pipeline), "tee"); 
    GstPad *srcpad      = gst_element_get_static_pad(tee, "src_1"); 
    GstPad *sinkpad     = gst_element_get_static_pad(branch, "sink"); 

    gst_element_set_state (data.pipeline, GST_STATE_PAUSED);
    gst_element_set_state (branch, GST_STATE_NULL);

    gst_pad_unlink(srcpad, sinkpad); 
    
    gst_element_remove_pad(tee, srcpad);
    
    gst_bin_remove(GST_BIN(data.pipeline), branch); 
    
    gst_element_set_state (data.pipeline, GST_STATE_PLAYING); 

    gst_object_unref(GST_OBJECT(srcpad)); 
    gst_object_unref(GST_OBJECT(sinkpad));
    gst_object_unref(GST_OBJECT(branch)); 
    gst_object_unref(GST_OBJECT(tee));

    return 1;
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
            if(idx < count) {
                *(result + idx++) = strdup(token);
                token = strtok(0, delim);
            }

        }
        if(idx == count - 1) {
            *(result + idx) = 0;
        }

    }

    return result;
}

const char * handle_packet(char * data_packet, sockaddr_in remoteAddr) {

    char** tokens = str_split(data_packet, ' ');
    if (!tokens)
        return "KO 0";

    char * resp = (char*) malloc(sizeof (char) * 255);
    memset(resp, '\0', 255);

    clock_gettime(CLOCK_REALTIME, &lastPacketTime);

    if (strcmp(tokens[0], "hello") == 0) {
        snprintf(resp, 255, "OK %s", tokens[1]);
        return resp;
    } else if (strcmp(tokens[0], "stream") == 0) {
        if (strcmp(tokens[2], "start") == 0) {
            char *ip = inet_ntoa(remoteAddr.sin_addr);

            char addr[15];
            memset(addr, '\0', 15);
            sprintf(addr, "%s", ip);
            if (verbose) printf("Executing: start stream %s %i", ip, atoi(tokens[3]));
            ret = startStream(addr, atoi(tokens[3]));
        }
    } else if (strcmp(tokens[0], "vidsnap") == 0) {
        if (strcmp(tokens[2], "record") == 0) {
            if (recording == 0) ret = startRecord();
            recording = 1;
        } else if (strcmp(tokens[2], "stop") == 0) {
            if (recording == 1) ret = stopRecording();
            recording = 0;
        } 
    } else if (strcmp(tokens[0], "querygps") == 0) {
        snprintf(resp, 255, "status gps %s %.6f %.6f %.6f %.6f %.6f", tokens[1], telem.lon, telem.lat, telem.gpsAlt, telem.spd, telem.gpsVsi);
        return resp;
    } else if (strcmp(tokens[0], "queryadc") == 0) {
        snprintf(resp, 255, "status analog %s %.6f %.6f", tokens[1], telem.volt, telem.amp);
        return resp;
    }

    if (ret != 1) {
        snprintf(resp, 255, "KO %s %s", tokens[1], tokens[2]);
        return resp;
    } else {
        snprintf(resp, 255, "OK %s %s", tokens[1], tokens[2]);
        return resp;
    }   
}

struct AnalogTelemetry * at = (AnalogTelemetry*)calloc(1, sizeof(AnalogTelemetry));

AnalogTelemetry * newSPI() {
    at->spi_bus = 0;
    at->spi_device = 0;
    at->voltage_channel = 0;
    at->current_channel = 1;
    at->max_amps = DEFAULT_SENSOR_MAX_AMPS;
    at->max_volts = DEFAULT_SENSOR_MAX_VOLTS;
    return at;
}

int setSPI(AnalogTelemetry * at, int bus, int device) {
    if ( at->spi && at->spi != NO_SPI ) {
        spi_dispose(at->spi);
    }
    at->spi_bus = bus;
    at->spi_device = device;
    at->spi = spi_new(bus, device);
    if ( !at->spi ) {
        at->spi = (SPIInterface*)NO_SPI;
        fprintf(stderr, "SPI-based telemetry transmission will be disabled\n");
        return 0;
    }
    return 1;
}

static double SpiReadChannel(AnalogTelemetry * at, int channel) {
    if ( at->spi == NO_SPI ) return 0.0;

    if ( !at->spi ) {
        at->spi = spi_new(at->spi_bus, at->spi_device);
        if ( !at->spi ) {
            fprintf(stderr, "SPI-based telemetry transmission will be disabled\n");
            at->spi = (SPIInterface*)NO_SPI;
            return 0.0;
        }
    }

    uint8_t outbuf[3];
    uint8_t inbuf[3] = {1, (2+channel) << 6, 0};
    spi_transaction(at->spi, inbuf, outbuf, sizeof(outbuf));
    int output = ((outbuf[1] & 3) << 8) + outbuf[2];
    return (double)output;
}

void recvSPI() {

    clock_gettime(CLOCK_REALTIME, &t5);
    dt = TimeSpecDiff(&t5, &adc);
    dt_ms = dt->tv_sec * 1000 + dt->tv_nsec / 1000000;

    int sample_v;
    long sample_sum_v = 0;
    for (sample_v = 0; sample_v < ANALOG_SAMPLES; sample_v++) {
        sample_sum_v += SpiReadChannel(at, at->voltage_channel);
    }
    double voltage = (sample_sum_v / ANALOG_SAMPLES) * at->max_volts / (double)ADC_MAX;

    int sample_i;
    long sample_sum_i = 0;
    for (sample_i = 0; sample_i < ANALOG_SAMPLES; sample_i++) {
        sample_sum_i += SpiReadChannel(at, at->current_channel);
    }
    double current = (sample_sum_i / ANALOG_SAMPLES) * at->max_amps / (double)ADC_MAX;

    telem.volt = voltage;
    telem.amp = current;

    if (verbose >= 2) {
        if (dt_ms > 3000) {
            adc = t5;
            printf("Voltage: %lf\n", voltage);
            printf("Current: %lf\n", current);
        }
    }
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
        bearing = (head * 180)/M_PI;
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

            received_bytes = recvfrom(sock, data_packet, sizeof (data_packet), 0, (sockaddr*) & remoteAddr, &fromLength);
            if (received_bytes == -1) {
                perror("received bytes = -1 \n");
            } else {
                data_packet[received_bytes] = 0;

                if (verbose >= 3) {
                    printf("Received: %s\n", data_packet);
                }

                const char* resp = handle_packet(data_packet, remoteAddr);

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

void loops() {
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

        recvSPI();
        recvNaza();
        setHomeVars();
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

/* This function will be called by the pad-added signal */
static void pad_added_handler (GstElement *src, GstPad *new_pad, CustomData *data) {
  GstPad *sink_pad = gst_element_get_static_pad (data->file_queue, "sink");
  GstPadLinkReturn ret;
  GstCaps *new_pad_caps = NULL;
  GstStructure *new_pad_struct = NULL;
  const gchar *new_pad_type = NULL;
   
  g_print ("Received new pad '%s' from '%s':\n", GST_PAD_NAME (new_pad), GST_ELEMENT_NAME (src));
   
  /* If our converter is already linked, we have nothing to do here */
  if (gst_pad_is_linked (sink_pad)) {
    g_print ("  We are already linked. Ignoring.\n");
    goto exit;
  }
   
  /* Check the new pad's type */
  new_pad_caps = gst_pad_get_allowed_caps (new_pad);
  new_pad_struct = gst_caps_get_structure (new_pad_caps, 0);
  new_pad_type = gst_structure_get_name (new_pad_struct);
  if (!g_str_has_prefix (new_pad_type, "audio/x-raw")) {
    g_print ("  It has type '%s' which is not raw audio. Ignoring.\n", new_pad_type);
    goto exit;
  }
   
  /* Attempt the link */
  ret = gst_pad_link (new_pad, sink_pad);
  if (GST_PAD_LINK_FAILED (ret)) {
    g_print ("  Type is '%s' but link failed.\n", new_pad_type);
  } else {
    g_print ("  Link succeeded (type '%s').\n", new_pad_type);
  }
   
    exit:
  /* Unreference the new pad's caps, if we got them */
  if (new_pad_caps != NULL)
    gst_caps_unref (new_pad_caps);
   
  /* Unreference the sink pad */
  gst_object_unref (sink_pad);
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

    // set up SPI
    newSPI();

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

    // Prep GStreamer
    GstBus *bus;
    GstMessage *msg;
    GstStateChangeReturn ret;

    // Init GStreamer
    gst_init(&argc, &argv);

    /* Create the elements */
    /*rpicamsrc bitrate=4500000 preview=0 ! video/x-h264, width=1296, height=972,framereate=49/1 ! h264parse ! /
    rtph264pay config-interval=1 pt=96 ! udpsink port=8888 host=192.168.10.1*/
    data.pipeline = gst_pipeline_new ("rasp_capture");
    data.source = gst_element_factory_make ("rpicamsrc", "source");
    data.tee = gst_element_factory_make ("tee", "tee");
    data.stream_queue = gst_element_factory_make ("queue", "stream_queue");
    data.parse = gst_element_factory_make ("h264parse", "parse");
    data.pay = gst_element_factory_make ("rtph264pay", "pay");
    data.stream_sink = gst_element_factory_make ("udpsink", "stream_sink");
    data.file_queue = gst_element_factory_make ("queue", "file_queue");
    data.file_parse = gst_element_factory_make ("h264parse", "file_parse");
    data.file_sink = gst_element_factory_make ("filesink", "filesink");
    data.filter = gst_element_factory_make ("capsfilter", "caps_filter");

    if (!data.pipeline || !data.source || !data.filter || !data.parse || !data.tee || !data.stream_queue || !data.pay || !data.stream_sink || !data.file_queue || !data.file_sink) {
        g_printerr ("One element could not be created. Exiting.\n");
        return -1;
    }

    GstCaps *cap = gst_caps_from_string("video/x-h264, width=1280, height=720, framerate=49/1, profile=baseline");

    g_object_set(data.pay, "config-interval", 1, "pt", 96, NULL);
    g_object_set(data.source, "bitrate", 4500000, "preview", FALSE, "inline-headers", TRUE, NULL);
    g_object_set(data.stream_sink, "port", 8888, "host", "192.168.10.1", NULL);
    g_object_set(data.filter, "caps", cap, NULL);
    gst_caps_unref(cap);

    gst_bin_add_many (GST_BIN (data.pipeline), data.source, data.filter, data.tee, data.stream_queue, data.parse, data.pay, data.stream_sink, NULL);

    
    if (gst_element_link_many(data.source, data.filter, data.tee, NULL) != TRUE) {
        g_printerr ("Elements could not be linked.\n");
        gst_object_unref (data.pipeline);
        return -1;
    }
    
    if (gst_element_link_many (data.tee, data.stream_queue, data.parse, data.pay, data.stream_sink, NULL) != TRUE) 
    {
        g_printerr ("Elements could not be linked.\n");
        gst_object_unref (data.pipeline);
        return -1;
    }

    //g_signal_connect (data.tee, "pad-added", G_CALLBACK (pad_added_handler), &data);

    
    /*filter1 = gst_element_factory_make ("capsfilter", NULL);
    gst_util_set_object_arg (G_OBJECT (filter1), "caps",
      "video/x-raw, width=320, height=240, "
      "format={ I420, YV12, YUY2, UYVY, AYUV, Y41B, Y42B, "
      "YVYU, Y444, v210, v216, NV12, NV21, UYVP, A420, YUV9, YVU9, IYU1 }");*/

    /* Start playing */
    g_print ("Now playing...");
    ret = gst_element_set_state (data.pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        g_printerr ("Unable to set the pipeline to the playing state.\n");
        gst_object_unref (data.pipeline);
        return -1;
    }

    //GstPipeline *pipeline = init_gst_pipeline();
    bus = gst_pipeline_get_bus(GST_PIPELINE(data.pipeline));

    // Start video pipeline
    //gst_element_set_state(GST_ELEMENT(pipeline), GST_STATE_PLAYING);

    while (!stop) {
        err = 0;
        if (verbose) printf("Opening socket...\n");

        // discard all received packet until avrspi is not connected (don't flood!)
        int udpBufSize = 0;
        socklen_t optlen = sizeof(udpBufSize);
        setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &udpBufSize, optlen);

        loops();

        while (!stop) {
            msg = gst_bus_pop(bus);
            if (msg != NULL) {
                GError *err;
                gchar *debug_info;
           
                switch (GST_MESSAGE_TYPE (msg)) {
                    case GST_MESSAGE_ERROR:
                        gst_message_parse_error (msg, &err, &debug_info);
                        g_printerr ("Error received from element %s: %s\n", GST_OBJECT_NAME (msg->src), err->message);
                        g_printerr ("Debugging information: %s\n", debug_info ? debug_info : "none");
                        g_clear_error (&err);
                        g_free (debug_info);
                        stop = 1;
                        break;
                    case GST_MESSAGE_EOS:
                        g_print ("End-Of-Stream reached.\n");
                        stop = 1;
                        break;
                    case GST_MESSAGE_STATE_CHANGED:
                        /* We are only interested in state-changed messages from the pipeline */
                        if (GST_MESSAGE_SRC (msg) == GST_OBJECT (data.pipeline)) {
                            GstState old_state, new_state, pending_state;
                            gst_message_parse_state_changed (msg, &old_state, &new_state, &pending_state);
                            g_print ("Pipeline state changed from %s to %s:\n",
                                gst_element_state_get_name (old_state), gst_element_state_get_name (new_state));
                        }
                        break;
                    default:
                        /* We should not reach here */
                        g_printerr ("Unexpected message received.\n");
                        break;
                }
                gst_message_unref(msg);
            }
        }

        if (err)
            sleep(2);
    }

    close(sock);

    if (verbose) printf("Closing.\n");

    // Stop video pipeline and clean up
    gst_element_set_state(GST_ELEMENT(data.pipeline), GST_STATE_NULL);
    gst_object_unref (data.pipeline);
    gst_object_unref(GST_OBJECT(bus));

    exit(EXIT_SUCCESS);
}