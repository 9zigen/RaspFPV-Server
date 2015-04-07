CXX=g++
CXX_OPTS= -Wall -g -O2 -I../ 

CC=cc
CC_OPTS=-lrt -lm -lc -lgcc

LIBS=`pkg-config --libs gstreamer-1.0`
CFLAGS=`pkg-config --cflags gstreamer-1.0`

INSTALL=install

OBJS=camera_controller.o routines.o NazaDecoderLib.o rs232.o spi.o

%.o: %.c                                                                         
	$(CXX) -c $(CXX_OPTS) $(CFLAGS) $< -o $@ 

all: $(OBJS)  
	$(CC) $(LIBS) $(OBJS)  -o camera_controller $(LDFLAGS) $(CC_OPTS) 

install:
	$(INSTALL) -m 0755 -d $(DESTDIR)/etc/init.d
	$(INSTALL) -m 0755 -d $(DESTDIR)/usr/local/bin
	$(INSTALL) -m 755 udp_controller $(DESTDIR)/usr/local/bin/
	$(INSTALL) -m 755 utils/ctrl_udp_controller.init $(DESTDIR)/etc/init.d/ctrl_udp_controller

uninstall:
	[ -z "$(DESTDIR)" ] && update-rc.d -f udp_controller remove || :
	sudo rm -f $(INSTALLED_FILES)
	
clean:
	rm -rf udp_controller
	rm -rf *.o