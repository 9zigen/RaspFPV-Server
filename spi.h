#ifndef __SPI_H
#define __SPI_H

#include <stdint.h>

typedef struct _SPIInterface SPIInterface;

SPIInterface * spi_new(int bus, int device);
void spi_dispose(SPIInterface *spi);

int spi_transaction(SPIInterface *spi, uint8_t *txbuffer, uint8_t *rxbuffer, int length);

#endif