#include "spi.h"

struct _SPIInterface {
    int fd;
    uint8_t bits_per_word;
    uint32_t max_speed;
};

SPIInterface * spi_new(int bus, int device) {
    SPIInterface * spi = (SPIInterface*)calloc(1, sizeof(SPIInterface));

    char path[255];
    snprintf(path, sizeof(path), "/dev/spidev%d.%d", bus, device);

    if ( (spi->fd = open(path, O_RDWR, 0)) == -1 ) {
        fprintf(stderr, "Couldn't open SPI interface %s: %s\n", path, strerror(errno));
        free(spi);
        return NULL;
    }

    if ( ioctl(spi->fd, SPI_IOC_RD_BITS_PER_WORD, &spi->bits_per_word) == -1 ||
         ioctl(spi->fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi->max_speed) == -1 ) {
        fprintf(stderr, "Couldn't read SPI interface %s: %s\n", path, strerror(errno));
        close(spi->fd);
        free(spi);
        return NULL;
    }

    return spi;
}

void spi_dispose(SPIInterface *spi) {
    if ( spi->fd ) {
        close(spi->fd);
    }
    free(spi);
}

int spi_transaction(SPIInterface *spi, uint8_t *txbuffer, uint8_t *rxbuffer, int length) {
    struct spi_ioc_transfer transfer;
    memset(&transfer, 0, sizeof(transfer));

    transfer.tx_buf = (unsigned long)txbuffer;
    transfer.rx_buf = (unsigned long)rxbuffer;
    transfer.len = length;
    transfer.speed_hz = spi->max_speed;
    transfer.bits_per_word = spi->bits_per_word;
    transfer.delay_usecs = 0;

    if ( ioctl(spi->fd, SPI_IOC_MESSAGE(1), &transfer) < 0 ) {
        fprintf(stderr, "Couldn't communicate with SPI interface: %s\n", strerror(errno));
        return -1;
    }

    return 0;
}