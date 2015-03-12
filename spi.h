#ifndef __SPI_H
#define __SPI_H

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <iostream>

typedef struct _SPIInterface SPIInterface;

SPIInterface * spi_new(int bus, int device);
void spi_dispose(SPIInterface *spi);

int spi_transaction(SPIInterface *spi, uint8_t *txbuffer, uint8_t *rxbuffer, int length);

#endif