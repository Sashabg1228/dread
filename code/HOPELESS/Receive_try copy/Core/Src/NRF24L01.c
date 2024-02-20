/*
 * NRF24L01.c
 *
 *  Created on: Feb 19, 2024
 *      Author: sasukoboy
 */

#include "stm32f1xx_hal.h"
#include "NRF24L01.h"

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
  uint8_t buffer = reg | (1 << 5);

  // Pull CSN pin LOW to select the device
  csn_select();

  HAL_SPI_Transmit(NRF24_SPI, buffer, 1, 100);
  HAL_SPI_Transmit(NRF24_SPI, data, size, 1000);

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
  // Pull CSN pin LOW to select the device
  csn_select();

  HAL_SPI_Transmit(NRF24_SPI, &cmd, 2, 100);

  // Pull CSN pin HIGH to unselect the device
  csn_unselect();
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

  // enable the device after initializing
  ce_enable();
}


// set up the TX mode
void NRF24_TX_mode (uint8_t *address, uint16_t channel)
{
  // disable the device before configuring
  ce_disable();

  nrf24_write_reg(RF_CH, channel);		// select the frequency channel
  nrf24_write_reg_multi(TX_ADDR, address, 5);	// set up the TX address

  // power up the device
  uint8_t config = nrf24_read_reg(CONFIG);
  config |= (1 << 1);
  nrf24_write_reg(CONFIG, config);

  // enable the device after configuring
  ce_enable();
}
// transmit the data
uint8_t NRF24_transmit (uint8_t *data)
{
  // select the device
  csn_select();

  // command to send the payload
  nrf24_write_cmd(W_TX_PAYLOAD);

  // send the payload
  HAL_SPI_Transmit(NRF24_SPI, data, 32, 1000);

  // unselect the device
  csn_unselect();

  HAL_Delay(1);

  uint8_t fifostatus = nrf24_read_reg(FIFO_STATUS);

  if ((fifostatus & (1 << 4)) && !(fifostatus & (1 << 3)))
  {
    // select the device
    csn_select();

    nrf24_write_cmd(FLUSH_TX);

    // unselect the device
    csn_unselect();

    return 0;	// success
  }

  return 1;	// failure
}


// set up the RX mode
void NRF24_RX_mode (uint8_t *address, uint16_t channel)
{
  // disable the device before configuring
  ce_disable();

  nrf24_write_reg(RF_CH, channel);		// select the frequency channel

  // select data pipe 1
  uint8_t en_rxaddr = nrf24_read_reg(EN_RXADDR);
  en_rxaddr |= (1 << 1);
  nrf24_write_reg(EN_RXADDR, en_rxaddr);

  nrf24_write_reg_multi(RX_ADDR_P1, address, 5);	// set up the TX address

  nrf24_write_reg(RX_PW_P1, 32);		// payload lenght of pipe 1 = 32 bytes

  // power up the device
  uint8_t config = nrf24_read_reg(CONFIG);
  config |= (1 << 1) | (1 << 0);
  nrf24_write_reg(CONFIG, config);

  // enable the device after configuring
  ce_enable();
}
// check if data is received on specific pipeline
uint8_t is_data_received (int pipenum)
{
  uint8_t status = nrf24_read_reg(STATUS);

  if ((status & (1 << 6)) && (status & (pipenum << 1)))
  {
    nrf24_write_reg(STATUS, (1 << 6));

    return 0;	// success
  }

  return 1;	// failure
}
// receive data
void NRF24_receive (uint8_t *data)
{
  // select the device
  csn_select();

  // payload command
  nrf24_write_cmd(R_RX_PAYLOAD);

  // receive the payload
  HAL_SPI_Receive(NRF24_SPI, data, 32, 1000);

  HAL_Delay(1);

  // command
  nrf24_write_cmd(FLUSH_RX);

  // unselect the device
  csn_unselect();
}

