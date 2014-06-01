
// SPI Configuration
#define SPI_PORT      SPI1
#define SPI_SCK_PIN   GPIO_Pin_5     // PA5
#define SPI_MISO_PIN  GPIO_Pin_6     // PA6
#define SPI_MOSI_PIN  GPIO_Pin_7     // PA7
#define SPI_CS_PIN    GPIO_Pin_4     // PA4
#define SPI_GPIO_PORT GPIOA

// nRF24L01 CE (Chip Enable) pin
#define nRF24_CE_PORT     GPIOB
#define nRF24_CE_PIN      GPIO_Pin_11    // PB11

// nRF24L01 IRQ pin
#define nRF24_IRQ_PORT    GPIOB
#define nRF24_IRQ_PIN     GPIO_Pin_10    // PB10


/* Some constants */
#define nRF24_RX_ADDR_WIDTH        5    // nRF24 RX address width
#define nRF24_TX_ADDR_WIDTH        5    // nRF24 TX address width


/* Variables */
extern uint8_t nRF24_RX_addr[nRF24_RX_ADDR_WIDTH];
extern uint8_t nRF24_TX_addr[nRF24_TX_ADDR_WIDTH];


/* Function prototypes */
void nRF24_init();

uint8_t nRF24_RWReg(uint8_t reg, uint8_t value);
uint8_t nRF24_ReadReg(uint8_t reg);
uint8_t nRF24_ReadBuf(uint8_t reg, uint8_t *pBuf, uint8_t count);
uint8_t nRF24_WriteBuf(uint8_t reg, uint8_t *pBuf, uint8_t count);

uint8_t nRF24_Check(void);

void nRF24_RXMode(uint8_t RX_PAYLOAD);
void nRF24_TXMode(void);
uint8_t nRF24_DataReady(void);
uint8_t nRF24_IsSending(void);

uint8_t nRF24_TXPacket(uint8_t * pBuf, uint8_t TX_PAYLOAD);
uint8_t nRF24_RXPacket(uint8_t* pBuf, uint8_t RX_PAYLOAD);
void nRF24_ClearIRQFlags(void);
