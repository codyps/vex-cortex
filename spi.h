#ifndef SPI_H_
#define SPI_H_

#include "vex_hw.h"
#include <stdbool.h>

void print_m2u(spi_packet_vex *m2u);
void print_joystick(struct oi_data *oi);
void spi_packet_init_m2u(spi_packet_vex *m2u);
void spi_packet_init_u2m(spi_packet_vex *u2m);
bool is_master_ready(void);
void vex_spi_xfer(spi_packet_vex *m2u, spi_packet_vex *u2m);
void vex_spi_process_packets(spi_packet_vex *m2u, spi_packet_vex *u2m);
void spi_init(void);

#endif