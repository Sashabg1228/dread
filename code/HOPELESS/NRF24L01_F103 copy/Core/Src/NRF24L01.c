/*
 * NRF24L01.c
 *
 *  Created on: Feb 19, 2024
 *      Author: sasukoboy
 */

#include "stm32f1xx_hal.h"
#include "NRF24L01.h"
#include <string.h>

extern SPI_HandleTypeDef hspi1;
#define NRF24_SPI 	&hspi1

#define NRF24_CE_BOARD	GPIOA
#define NRF24_CE_PIN	GPIO_PIN_10

#define NRF24_CSN_BOARD	GPIOA
#define NRF24_CSN_PIN	GPIO_PIN_15


void ce_enable (void)
{
  HAL_GPIO_WritePin(NRF24_CE_BOARD, NRF24_CE_PIN, GPIO_PIN_SET);
}
void ce_disable (void)
{
  HAL_GPIO_WritePin(NRF24_CE_BOARD, NRF24_CE_PIN, GPIO_PIN_RESET);
}


void csn_select (void)
{
  HAL_GPIO_WritePin(NRF24_CSN_BOARD, NRF24_CSN_PIN, GPIO_PIN_RESET);
}
void csn_unselect (void)
{
  HAL_GPIO_WritePin(NRF24_CSN_BOARD, NRF24_CSN_PIN, GPIO_PIN_SET);
}


// write single byte to the particular register
void nrf24_write_reg (uint8_t reg, uint8_t data)
{
  uint8_t buffer[2];
  buffer[0] = reg | (1 << 5);
  buffer[1] = data;

  // Pull CSN pin LOW to select the device
  csn_select();

  HAL_SPI_Transmit(NRF24_SPI, buffer, 2, 1000);

  // Pull CSN pin HIGH to unselect the divice
  csn_unselect();
}
// write multiple bytes to the particular register
void nrf24_write_reg_multi (uint8_t reg, uint8_t *data, size_t size)
{
  uint8_t buffer[6];
  buffer[0] = reg | (1 << 5);
  buffer[1] = data[0];
  buffer[2] = data[1];
  buffer[3] = data[2];
  buffer[4] = data[3];
  buffer[5] = data[4];

  // Pull CSN pin LOW to select the device
  csn_select();

  HAL_SPI_Transmit(NRF24_SPI, buffer, size + 1, 1000);

  // Pull CSN pin HIGH to unselect the divice
  csn_unselect();
}

// read single byte from the particular register
uint8_t nrf24_read_reg (uint8_t reg)
{
  uint8_t data = 0;

  // Pull CSN pin LOW to select the device
  csn_select();

  HAL_SPI_Transmit(NRF24_SPI, &reg, 1, 100);
  HAL_SPI_Receive(NRF24_SPI, &data, 1, 100);

  // Pull CSN pin HIGH to unselect the divice
  csn_unselect();

  return data;
}
// read multiple bytes from the particular register
void nrf24_read_reg_multi (uint8_t reg, uint8_t *data, size_t size)
{
  // Pull CSN pin LOW to select the device
  csn_select();

  HAL_SPI_Transmit(NRF24_SPI, &reg, 1, 100);
  HAL_SPI_Receive(NRF24_SPI, data, size, 1000);

  // Pull CSN pin HIGH to unselect the device
  csn_unselect();
}

// send command to the RF module
// select the device before invoking the function
void nrf24_write_cmd (uint8_t cmd)
{
  HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, 100);
}



// initialize the RF module
void NRF24_init (void)
{
  // disable the device before initializing
  ce_disable();

  nrf24_write_reg(CONFIG, 0);		// to be configured later
  nrf24_write_reg(EN_AA, 0); 		// No auto ACK
  nrf24_write_reg(EN_RXADDR, 0);	// disabling the data pipes
  nrf24_write_reg(SETUP_AW, 0x03);	// TX/RX address length = 5 bytes
  nrf24_write_reg(SETUP_RETR, 0);	// No retransmissions
  nrf24_write_reg(RF_CH, 0);		// will be set up during TX or RX configuration
  nrf24_write_reg(RF_SETUP, 0x06);	// Output power = 0db, data rate = 2 Mbps
}


// set up the TX mode
void NRF24_TX_mode (uint8_t *address, uint16_t channel)
{
  nrf24_write_reg(RF_CH, channel);		// select the frequency channel
  nrf24_write_reg_multi(TX_ADDR, address, 5);	// set up the TX address

  // power up the device
  uint8_t config = nrf24_read_reg(CONFIG);
  config |= (1 << 1);
  nrf24_write_reg(CONFIG, config);
}

int load_payload (uint8_t *data, size_t size)
{
  if (32 < size)
  {
      return 1; // too big size
  }

  uint8_t buffer[33];
  buffer[0] = W_TX_PAYLOAD;
  memcpy(buffer + 1, data, size);

  // Pull CSN pin HIGH to LOW
  csn_unselect();
  csn_select();


  HAL_SPI_Transmit(NRF24_SPI, buffer, size + 1, 1000);

  // Pull CSN pin HIGH to unselect the device
  csn_unselect();

  return 0; // success
}

// transmit the data
uint8_t NRF24_transmit (uint8_t *data)
{
  load_payload(data, 32);


  ce_enable();

  HAL_Delay(1); // data send

  ce_disable();


  csn_select();

  uint8_t fifostatus = nrf24_read_reg(FIFO_STATUS);

  if ((fifostatus & (1 << 4)) && !(fifostatus & (1 << 3)))
  {
    nrf24_write_cmd(FLUSH_TX);

    csn_unselect();

    return 0;	// success
  }

  csn_unselect();

  return 1;	// failure
}


// set up the RX mode
void NRF24_RX_mode (uint8_t *address, uint16_t channel)
{
  // disable the device before configuring
  // ce_disable();

  nrf24_write_reg(RF_CH, channel);		// select the frequency channel

  // select data pipe 1
  uint8_t en_rxaddr = nrf24_read_reg(EN_RXADDR);
  en_rxaddr |= (1 << 1);
  nrf24_write_reg(EN_RXADDR, en_rxaddr);

  NRF24_debug();

  nrf24_write_reg_multi(RX_ADDR_P1, address, 5);	// set up the TX address

  NRF24_debug();

  nrf24_write_reg(RX_PW_P1, 32);		// payload lenght of pipe 1 = 32 bytes

  // power up the device
  uint8_t config = nrf24_read_reg(CONFIG);
  config |= (1 << 1) | (1 << 0);
  nrf24_write_reg(CONFIG, config);

  // enable the device after configuring
  // ce_enable();
}
// check if data is received on specific pipeline
uint8_t is_data_received (int pipenum)
{
  uint8_t status = nrf24_read_reg(STATUS);

  if ((status&(1 << 6)) && (status&(pipenum << 1)))
  {
    nrf24_write_reg(STATUS, (1 << 6));

    return 0;	// success
  }

  return 1;	// failure
}
// receive data
void NRF24_receive (uint8_t *data)
{
  //uint8_t cmd;

  // select the device
  csn_select();

  // payload command
  //cmd = R_RX_PAYLOAD;
  nrf24_write_cmd(R_RX_PAYLOAD);

  // receive the payload
  HAL_SPI_Receive(NRF24_SPI, data, 32, 1000);

  HAL_Delay(1);

  // command
  //cmd = FLUSH_RX;
  nrf24_write_cmd(FLUSH_RX);

  // unselect the device
  csn_unselect();
}


void NRF24_debug (void)
{
  uint8_t __attribute__((unused)) config = nrf24_read_reg(CONFIG);
  uint8_t __attribute__((unused)) en_aa = nrf24_read_reg(EN_AA);
  uint8_t __attribute__((unused)) en_rxaddr = nrf24_read_reg(EN_RXADDR);
  uint8_t __attribute__((unused)) setup_aw = nrf24_read_reg(SETUP_AW);
  uint8_t __attribute__((unused)) setup_retr = nrf24_read_reg(SETUP_RETR);
  uint8_t __attribute__((unused)) rf_ch = nrf24_read_reg(RF_CH);
  uint8_t __attribute__((unused)) rf_setup = nrf24_read_reg(RF_SETUP);
  uint8_t __attribute__((unused)) status = nrf24_read_reg(STATUS);
  uint8_t __attribute__((unused)) observe_tx = nrf24_read_reg(OBSERVE_TX);
  uint8_t __attribute__((unused)) cd = nrf24_read_reg(CD);
  uint8_t __attribute__((unused)) rx_addr_p0[5];
  nrf24_read_reg_multi(RX_ADDR_P0, rx_addr_p0, 5);
  uint8_t __attribute__((unused)) rx_addr_p1[5];
  nrf24_read_reg_multi(RX_ADDR_P1, rx_addr_p1, 5);
  uint8_t __attribute__((unused)) rx_addr_p2 = nrf24_read_reg(RX_ADDR_P2);
  uint8_t __attribute__((unused)) rx_addr_p3 = nrf24_read_reg(RX_ADDR_P3);
  uint8_t __attribute__((unused)) rx_addr_p4 = nrf24_read_reg(RX_ADDR_P4);
  uint8_t __attribute__((unused)) rx_addr_p5 = nrf24_read_reg(RX_ADDR_P5);
  uint8_t __attribute__((unused)) tx_addr[5];
  nrf24_read_reg_multi(TX_ADDR, tx_addr, 5);
  uint8_t __attribute__((unused)) rx_pw_p0 = nrf24_read_reg(RX_PW_P0);
  uint8_t __attribute__((unused)) rx_pw_p1 = nrf24_read_reg(RX_PW_P1);
  uint8_t __attribute__((unused)) rx_pw_p2 = nrf24_read_reg(RX_PW_P2);
  uint8_t __attribute__((unused)) rx_pw_p3 = nrf24_read_reg(RX_PW_P3);
  uint8_t __attribute__((unused)) rx_pw_p4 = nrf24_read_reg(RX_PW_P4);
  uint8_t __attribute__((unused)) rx_pw_p5 = nrf24_read_reg(RX_PW_P5);
  uint8_t __attribute__((unused)) fifo_status = nrf24_read_reg(FIFO_STATUS);
  uint8_t __attribute__((unused)) dynpd = nrf24_read_reg(DYNPD);
  uint8_t __attribute__((unused)) feature = nrf24_read_reg(FEATURE);

  return;
}
