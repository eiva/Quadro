#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_spi.h>

#include <nRF24L01.h>

// RX-TX address.
uint8_t nRF24_RX_addr[nRF24_RX_ADDR_WIDTH] = {0xDB,0xDB,0xDB,0xDB,0xDB};
uint8_t nRF24_TX_addr[nRF24_TX_ADDR_WIDTH] = {0xDB,0xDB,0xDB,0xDB,0xDB};


// Chip Enable Activates RX or TX mode
#define CE_L() GPIO_ResetBits(nRF24_CE_PORT,nRF24_CE_PIN)
#define CE_H() GPIO_SetBits(nRF24_CE_PORT,nRF24_CE_PIN)

// SPI Chip Select
#define CSN_L() GPIO_ResetBits(SPI_GPIO_PORT,SPI_CS_PIN)
#define CSN_H() GPIO_SetBits(SPI_GPIO_PORT,SPI_CS_PIN)


/* nRF24L0 commands */
#define nRF24_CMD_RREG             0x00  // R_REGISTER -> Read command and status registers
#define nRF24_CMD_WREG             0x20  // W_REGISTER -> Write command and status registers
#define nRF24_CMD_R_RX_PAYLOAD     0x61  // R_RX_PAYLOAD -> Read RX payload
#define nRF24_CMD_W_TX_PAYLOAD     0xA0  // W_TX_PAYLOAD -> Write TX payload
#define nRF24_CMD_FLUSH_TX         0xE1  // FLUSH_TX -> Flush TX FIFO
#define nRF24_CMD_FLUSH_RX         0xE2  // FLUSH_RX -> Flush RX FIFO
#define nRF24_CMD_REUSE_TX_PL      0xE3  // REUSE_TX_PL -> Reuse last transmitted payload
#define nRF24_CMD_NOP              0xFF  // No operation (to read status register)

/* nRF24L0 registers */
#define nRF24_REG_CONFIG           0x00  // Configuration register
#define nRF24_REG_EN_AA            0x01  // Enable "Auto acknowledgment"
#define nRF24_REG_EN_RXADDR        0x02  // Enable RX addresses
#define nRF24_REG_SETUP_AW         0x03  // Setup of address widths
#define nRF24_REG_SETUP_RETR       0x04  // Setup of automatic retranslation
#define nRF24_REG_RF_CH            0x05  // RF channel
#define nRF24_REG_RF_SETUP         0x06  // RF setup register
#define nRF24_REG_STATUS           0x07  // Status register
#define nRF24_REG_OBSERVE_TX       0x08  // Transmit observe register
#define nRF24_REG_CD               0x09  // Carrier detect
#define nRF24_REG_RX_ADDR_P0       0x0A  // Receive address data pipe 0
#define nRF24_REG_RX_ADDR_P1       0x0B  // Receive address data pipe 1
#define nRF24_REG_RX_ADDR_P2       0x0C  // Receive address data pipe 2
#define nRF24_REG_RX_ADDR_P3       0x0D  // Receive address data pipe 3
#define nRF24_REG_RX_ADDR_P4       0x0E  // Receive address data pipe 4
#define nRF24_REG_RX_ADDR_P5       0x0F  // Receive address data pipe 5
#define nRF24_REG_TX_ADDR          0x10  // Transmit address
#define nRF24_REG_RX_PW_P0         0x11  // Number of bytes in RX payload id data pipe 0
#define nRF24_REG_RX_PW_P1         0x12  // Number of bytes in RX payload id data pipe 1
#define nRF24_REG_RX_PW_P2         0x13  // Number of bytes in RX payload id data pipe 2
#define nRF24_REG_RX_PW_P3         0x14  // Number of bytes in RX payload id data pipe 3
#define nRF24_REG_RX_PW_P4         0x15  // Number of bytes in RX payload id data pipe 4
#define nRF24_REG_RX_PW_P5         0x16  // Number of bytes in RX payload id data pipe 5
#define nRF24_REG_FIFO_STATUS      0x17  // FIFO status register
#define nRF24_REG_DYNPD            0x1C  // Enable dynamic payload length
#define nRF24_REG_FEATURE          0x1D  // Feature register

/* nRF24L0 bits */
#define nRF24_MASK_RX_DR           0x40  // Mask interrupt caused by RX_DR
#define nRF24_MASK_TX_DS           0x20  // Mask interrupt caused by TX_DS
#define nRF24_MASK_MAX_RT          0x10  // Mask interrupt caused by MAX_RT
#define nRF24_FIFO_RX_EMPTY        0x01  // RX FIFO empty flag
#define nRF24_FIFO_RX_FULL         0x02  // RX FIFO full flag

// Settings

#define RF_CHANNEL 0x01 // Channel #1
#define RF_TRANSMIT_MODE 0x0F

// GPIO and SPI initialization
void nRF24_init() {
	GPIO_InitTypeDef PORT;
	GPIO_StructInit(&PORT);
	PORT.GPIO_Speed = GPIO_Speed_50MHz;

	// Setup SPI GPIO ports
	PORT.GPIO_Pin = SPI_SCK_PIN | SPI_MOSI_PIN;
	PORT.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(SPI_GPIO_PORT,&PORT);

	PORT.GPIO_Pin = SPI_MISO_PIN;
	PORT.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(SPI_GPIO_PORT,&PORT);

	// Configure CS pin as output with Push-Pull
	PORT.GPIO_Pin = SPI_CS_PIN;
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(SPI_GPIO_PORT,&PORT);

	// Configure CE pin as output with Push-Pull
	PORT.GPIO_Pin = nRF24_CE_PIN;
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(nRF24_CE_PORT,&PORT);

	// Configure IRQ pin as input with Pull-Up
	PORT.GPIO_Pin = nRF24_IRQ_PIN;
	PORT.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(nRF24_IRQ_PORT,&PORT);

	// Init SPI port.
	SPI_InitTypeDef SPI;
	SPI_StructInit(&SPI);
	SPI.SPI_Mode = SPI_Mode_Master;
	SPI.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex;

	// Set SPI to Mode 0
	SPI.SPI_CPOL = SPI_CPOL_Low;
	SPI.SPI_CPHA = SPI_CPHA_1Edge;
	SPI.SPI_CRCPolynomial = 7;
	SPI.SPI_DataSize = SPI_DataSize_8b;
	SPI.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI.SPI_NSS = SPI_NSS_Soft;
	SPI_Init(SPI_PORT,&SPI);

	// NSS must be set to '1' due to NSS_Soft settings (otherwise it will be Multimaster mode).
	SPI_NSSInternalSoftwareConfig(SPI_PORT,SPI_NSSInternalSoft_Set);

	SPI_Cmd(SPI_PORT,ENABLE);

	CSN_H();
	CE_L();
}

// Send/Receive data to nRF24L01 via SPI
// input:
//   data - byte to send
// output: received byte from nRF24L01
uint8_t nRF24_ReadWrite(uint8_t data) {
	while (SPI_I2S_GetFlagStatus(SPI_PORT,SPI_I2S_FLAG_TXE) == RESET); // Wait while DR register is not empty
	SPI_I2S_SendData(SPI_PORT,data); // Send byte to SPI
	while (SPI_I2S_GetFlagStatus(SPI_PORT,SPI_I2S_FLAG_RXNE) == RESET); // Wait to receive byte
	return SPI_I2S_ReceiveData(SPI_PORT); // Read byte from SPI bus
}

// Write new value to register
// input:
//   reg - register number
//   value - new value
// output: nRF24L01 status
uint8_t nRF24_RWReg(uint8_t reg, uint8_t value) {
	uint8_t status;

	CSN_L();
	status = nRF24_ReadWrite(reg); // Select register
	nRF24_ReadWrite(value); // Write value to register
	CSN_H();

	return status;
}

// Read nRF24L01 register
// input:
//   reg - register number
// output: register value
uint8_t nRF24_ReadReg(uint8_t reg) {
	uint8_t value;

	CSN_L();
	nRF24_ReadWrite(reg);
	value = nRF24_ReadWrite(0x00);
	CSN_H();

	return value;
}

// Get data from nRF24L01 into buffer
// input:
//   reg - register number
//   pBuf - pointer to buffer
//   count - bytes count
// output: nRF24L01 status
uint8_t nRF24_ReadBuf(uint8_t reg, uint8_t *pBuf, uint8_t count) {
	uint8_t status,i;

	CSN_L();
	status = nRF24_ReadWrite(reg);
	for (i = 0; i < count; i++) pBuf[i] = nRF24_ReadWrite(0);
	CSN_L();

	return status;
}

// Send buffer to nRF24L01
// input:
//   reg - register number
//   pBuf - pointer to buffer
//   count - bytes count
// output: nRF24L01 status
uint8_t nRF24_WriteBuf(uint8_t reg, uint8_t *pBuf, uint8_t count) {
	uint8_t status,i;

	CSN_L();
	status = nRF24_ReadWrite(reg);
	for (i = 0; i < count; i++) nRF24_ReadWrite(*pBuf++);
	CSN_H();

	return status;
}

// Check if nRF24L01 present (send byte sequence, read it back and compare)
// return:
//   0 - looks like an nRF24L01 is online
//   1 - received sequence differs from original
uint8_t nRF24_Check(void) {
	uint8_t txbuf[5] = { 0xA9,0xA9,0xA9,0xA9,0xA9 };
	uint8_t rxbuf[5];
	uint8_t i;

	nRF24_WriteBuf(nRF24_CMD_WREG | nRF24_REG_TX_ADDR,txbuf,5); // Write fake TX address
    nRF24_ReadBuf(nRF24_REG_TX_ADDR,rxbuf,5); // Try to read TX_ADDR register
    for (i = 0; i < 5; i++) if (rxbuf[i] != txbuf[i]) return 1;

    return 0;
}

// Put nRF24L01 in RX mode
void nRF24_RXMode(uint8_t RX_PAYLOAD) {
	CE_L();
	
	nRF24_WriteBuf(nRF24_CMD_WREG | nRF24_REG_RX_ADDR_P1,nRF24_RX_addr,nRF24_RX_ADDR_WIDTH); // Set static RX address

	nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_EN_RXADDR,0x03);//0x01); // Enable data pipe 0

	//nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_RF_CH,0x6E); // Set frequency channel 110 (2.510MHz)
	nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_RF_CH,RF_CHANNEL); // Set frequency channel 110 (2.510MHz)

	nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_RX_PW_P0,RX_PAYLOAD); // Set RX payload length (10 bytes)
	nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_RX_PW_P1,RX_PAYLOAD); // Set RX payload length (10 bytes)

	//nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_RF_SETUP, RF_TRANSMIT_MODE);//0x06); // Setup: 1Mbps, 0dBm, LNA off
	nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_CONFIG, 0x0B); // Config: CRC on (1 bytes), Power UP, RX/TX ctl = PRX

	CE_H();
	
	nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_STATUS, (1 << 5) | (1 << 4));

	CSN_L();
	nRF24_ReadWrite(nRF24_CMD_FLUSH_RX); // Flush RX FIFO buffer
	CSN_H();
}

// Put nRF24L01 in TX mode
void nRF24_TXMode(void) {
	CE_L();
	nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_CONFIG,0x02); // Config: Power UP
	nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_EN_AA,0x3F); // Enable ShockBurst for data pipe 0
	nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_SETUP_RETR,0x1A); // Auto retransmit: wait 500us, 10 retries
	nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_RF_CH, RF_CHANNEL); // Set frequency channel 110 (2.510MHz)
	nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_RF_SETUP,RF_TRANSMIT_MODE); // Setup: 1Mbps, 0dBm, LNA off
	nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_CONFIG,0x0B); // Config: CRC on (2 bytes), Power UP, RX/TX ctl = PTX
}

// Check if data is available for reading
// return:
//   0 -> no data
//   1 -> RX_DR is set or some bytes present in FIFO
uint8_t nRF24_DataReady(void) {
    uint8_t status;

    status = nRF24_ReadReg(nRF24_REG_STATUS);
    if (status & nRF24_MASK_RX_DR) return 1;

    // Checking RX_DR isn't good enough, there's can be some data in FIFO
    status = nRF24_ReadReg(nRF24_REG_FIFO_STATUS);

    return (status & nRF24_FIFO_RX_EMPTY) ? 0 : 1;
}

// Check that sending is still in progress.
// return:
//   0 -> no data
//   1 -> RX_DR is set or some bytes present in FIFO
uint8_t nRF24_IsSending(void) {
   return 0;
}

uint8_t nRF24_RXPacket(uint8_t* pBuf, uint8_t RX_PAYLOAD) {
	uint8_t status;

	status = nRF24_ReadReg(nRF24_REG_STATUS); // Read status register
    if (status & nRF24_MASK_RX_DR) {
    	//if ((status & 0x0E) == 0) {
    		// pipe 0
    		CSN_L();
    		nRF24_ReadBuf(nRF24_CMD_R_RX_PAYLOAD,pBuf,RX_PAYLOAD); // read received payload from RX FIFO buffer
    		CSN_H();
    	//}
		nRF24_ReadWrite(nRF24_CMD_FLUSH_RX); // Flush RX FIFO buffer
		nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_STATUS,status | 0x70); // Clear RX_DR, TX_DS, MAX_RT flags
	    //return nRF24_MASK_RX_DR;
	    return status;
    }

    // Some banana happens
	nRF24_ReadWrite(nRF24_CMD_FLUSH_RX); // Flush RX FIFO buffer
	nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_STATUS,status | 0x70); // Clear RX_DR, TX_DS, MAX_RT flags
    return status;
}

// Send data packet
// input:
//   pBuf - buffer with data to send
// return:
//   nRF24_MASK_MAX_RT - if transmit failed with maximum auto retransmit count
//   nRF24_MAX_TX_DS - if transmit succeed
//   contents of STATUS register otherwise
uint8_t nRF24_TXPacket(uint8_t * pBuf, uint8_t TX_PAYLOAD) {
    uint8_t status;

    CE_L();
    nRF24_WriteBuf(nRF24_CMD_WREG | nRF24_REG_TX_ADDR,nRF24_TX_addr,nRF24_TX_ADDR_WIDTH); // Set static TX address
    nRF24_WriteBuf(nRF24_CMD_WREG | nRF24_REG_RX_ADDR_P0,nRF24_RX_addr,nRF24_RX_ADDR_WIDTH); // Set static RX address for auto ack
    nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_EN_AA,0x01); // Enable auto acknowledgement for data pipe 0
    nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_SETUP_RETR,0x1A); // Automatic retransmission: wait 500us, 10 retries
	nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_RF_CH, RF_CHANNEL);
    nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_RF_SETUP,RF_TRANSMIT_MODE); // Setup: 1Mbps, 0dBm, LNA on
    nRF24_WriteBuf(nRF24_CMD_W_TX_PAYLOAD,pBuf,TX_PAYLOAD); // Write specified buffer to FIFO
    nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_CONFIG,0x0E); // Config: CRC on (2 bytes), Power UP, RX/TX ctl = PTX
    CE_H();

    CE_L();
    status = nRF24_ReadReg(nRF24_REG_STATUS); // Read status register
	nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_STATUS,status | 0x70); // Clear RX_DR, TX_DS, MAX_RT flags
    if (status & nRF24_MASK_MAX_RT) {
        // Auto retransmit counter exceeds the programmed maximum limit. FIFO is not removed.
        nRF24_RWReg(nRF24_CMD_FLUSH_TX,0xFF); // Flush TX FIFO buffer
        return nRF24_MASK_MAX_RT;
    };
    if (status & nRF24_MASK_TX_DS) {
        // Transmit ok
        nRF24_RWReg(nRF24_CMD_FLUSH_TX,0xFF); // Flush TX FIFO buffer
        return nRF24_MASK_TX_DS;
    }

    // Should not be there.
    return status;
}

// Clear all IRQ flags
void nRF24_ClearIRQFlags(void) {
	uint8_t status;

    status = nRF24_ReadReg(nRF24_REG_STATUS);
	nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_STATUS,status | 0x70); // Clear RX_DR, TX_DS, MAX_RT flags
}
