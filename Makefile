CXX=g++
CXX_OPTS= -Wall -g -O2 -I../ 

CC=cc
CC_OPTS=-lrt -lm -lc -lgcc

INSTALL=install

OBJS=camera_controller.o routines.o NazaDecoderLib.o rs232.o spi.o

%.o: %.c                                                                         
	$(CXX) -c $(CXX_OPTS) $< -o $@ 

all: $(OBJS)  
	$(CC) $(CFLAGS) $(OBJS)  -o camera_controller $(LDFLAGS) $(CC_OPTS) 

install:
	$(INSTALL) -m 0755 -d $(DESTDIR)/etc/init.d
	$(INSTALL) -m 0755 -d $(DESTDIR)/usr/local/bin
	$(INSTALL) -m 755 camera_controller $(DESTDIR)/usr/local/bin/
	$(INSTALL) -m 755 utils/camera.sh $(DESTDIR)/usr/local/bin/
	$(INSTALL) -m 755 utils/camera_controller.init $(DESTDIR)/etc/init.d/camera_controller

uninstall:
	[ -z "$(DESTDIR)" ] && update-rc.d -f camera_controller remove || :
	sudo rm -f $(INSTALLED_FILES)
	
clean:
	rm -rf camera_controller
	rm -rf *.o