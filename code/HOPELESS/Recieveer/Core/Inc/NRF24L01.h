/*
 * NRF24L01.h
 *
 *  Created on: Feb 19, 2024
 *      Author: sasukoboy
 */

#ifndef INC_NRF24L01_H_
#define INC_NRF24L01_H_

typedef struct
{
  uint8_t hand_position;	// hand position	(-100 ~ +100)
  uint8_t left_speed;		// left wheel speed 	(-100 ~ +100)
  uint8_t right_speed;		// right wheel speed	(-100 ~ +100)
  uint8_t weapon;		// weapon on/off	   (0 / 1)
} Payload;


// read all registers of the RF module
void NRF24_debug (void);


// initialize the RF module
void NRF24_init (void);


// set up the TX mode
void NRF24_TX_mode (const uint8_t *address, const uint16_t channel);
// transmit the data
int NRF24_transmit (const Payload *payload);
// send the payload to the RF module
int load_payload (const Payload *payload);


// set up the RX mode
void NRF24_RX_mode (const uint8_t *address, const uint16_t channel);
// check if data is received on specific pipeline
uint8_t is_data_received (const int pipeline);
// receive data
int NRF24_receive (Payload *payload);


/* Memory Map */
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD	    0x1C
#define FEATURE	    0x1D

/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF

#endif /* INC_NRF24L01_H_ */
