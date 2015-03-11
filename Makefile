CXX=g++
CXX_OPTS= -Wall -g -O2 -I../ 

CC=cc
CC_OPTS=-lrt -lm -lc -lgcc

INSTALL=install

OBJS=camera_controller.o routines.o NazaDecoderLib.o rs232.o

%.o: %.c                                                                         
	$(CXX) -c $(CXX_OPTS) $< -o $@ 

all: $(OBJS)  
	$(CC) $(CFLAGS) $(OBJS)  -o camera $(LDFLAGS) $(CC_OPTS) 

install:
	$(INSTALL) -m 0755 -d $(DESTDIR)/etc/init.d
	$(INSTALL) -m 0755 -d $(DESTDIR)/usr/local/bin
	$(INSTALL) -m 755 camera $(DESTDIR)/usr/local/bin/
	$(INSTALL) -m 755 camera.sh $(DESTDIR)/usr/local/bin/
	$(INSTALL) -m 755 utils/ioscamera $(DESTDIR)/etc/init.d/

clean:
	rm -rf camera_server
	rm -rf *.o