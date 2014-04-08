#include "FreeRTOS.h"
//#include "semphr.h"
#include "i2c_bsp.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_it.h"
//#include "task.h"

#define I2C_USE_DMA

#define I2C_MAX_BUF_LEN 64
#define I2C_TIMEOUT         ((uint32_t)0x1000)
#define I2C_LONG_TIMEOUT         ((uint32_t)(10 * I2C_TIMEOUT))

#ifdef I2C_USE_DMA
#define I2C_DMA_TX_IRQn              DMA1_Stream4_IRQn
#define I2C_DMA_RX_IRQn              DMA1_Stream2_IRQn

#define I2C_DMA                      DMA1   
#define I2C_DMA_CHANNEL              DMA_Channel_3
#define I2C_DMA_STREAM_TX            DMA1_Stream4
#define I2C_DMA_STREAM_RX            DMA1_Stream2   
#define I2C_DMA_CLK                  RCC_AHB1Periph_DMA1
#define I2C_DR_Address               &(I2C3->DR)

#define I2C_DMA_TX_IRQHandler        DMA1_Stream4_IRQHandler
#define I2C_DMA_RX_IRQHandler        DMA1_Stream2_IRQHandler   

#define I2C_TX_DMA_FLAG_FEIF             DMA_FLAG_FEIF4
#define I2C_TX_DMA_FLAG_DMEIF            DMA_FLAG_DMEIF4
#define I2C_TX_DMA_FLAG_TEIF             DMA_FLAG_TEIF4
#define I2C_TX_DMA_FLAG_HTIF             DMA_FLAG_HTIF4
#define I2C_TX_DMA_FLAG_TCIF             DMA_FLAG_TCIF4
#define I2C_RX_DMA_FLAG_FEIF             DMA_FLAG_FEIF2
#define I2C_RX_DMA_FLAG_DMEIF            DMA_FLAG_DMEIF2
#define I2C_RX_DMA_FLAG_TEIF             DMA_FLAG_TEIF2
#define I2C_RX_DMA_FLAG_HTIF             DMA_FLAG_HTIF2
#define I2C_RX_DMA_FLAG_TCIF             DMA_FLAG_TCIF2

volatile uint8_t i2c_rx_state;
volatile uint8_t i2c_tx_state;
#endif

I2C_TypeDef *i2c_bus[] = {
	I2C1,
	I2C2,
	I2C3
};

typedef struct {
	i2c_dev *dev;
	uint8_t tx_idx;
	uint8_t rx_idx;
	uint8_t *tx_buf;
	uint8_t *rx_buf;
	uint8_t tx_len;
	uint8_t rx_len;
} i2c_msg;

void i2c_init(uint8_t ch, uint32_t clock);
int8_t i2c_xfer(i2c_msg *msg);
i2c_msg *g_msg;
volatile uint8_t i2c_wait_flag;
#ifdef I2C_USE_DMA
DMA_InitTypeDef    DMA_InitStructure; 
NVIC_InitTypeDef   NVIC_InitStructure;
#endif

void i2c_init(uint8_t ch, uint32_t clock)
{
	I2C_InitTypeDef I2C_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	uint8_t flag = 0;

	I2C_DeInit(i2c_bus[ch]);
	switch (ch) {
		case 0:
			/* Enable the i2c bus peripheral clock */
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

			/* i2c_bus SCL and SDA pins configuration -------------------------------------*/
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_9;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
			GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
			GPIO_Init(GPIOB, &GPIO_InitStructure);    
			/* Connect pins to I2C peripheral */
			GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
			GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);

			/* i2c bus peripheral configuration */
			I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
			I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
			I2C_InitStructure.I2C_OwnAddress1 = 0x0;
			I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
			I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
			I2C_InitStructure.I2C_ClockSpeed = clock;
			flag = 1;
			break;
		case 1:
			break;
		case 2:
			/* Enable the i2c bus peripheral clock */
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3, ENABLE);

			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

			/* i2c_bus SCL and SDA pins configuration -------------------------------------*/
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
			GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
			GPIO_Init(GPIOA, &GPIO_InitStructure);    

			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
			GPIO_Init(GPIOC, &GPIO_InitStructure);    

			GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_I2C3);
			GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_I2C3);

			/* i2c bus peripheral configuration */
			I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
			I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
			I2C_InitStructure.I2C_OwnAddress1 = 0x0;
			I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
			I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
			I2C_InitStructure.I2C_ClockSpeed = clock;

#ifdef I2C_USE_DMA
			/* Configure and enable I2C DMA TX Channel interrupt */
			NVIC_InitStructure.NVIC_IRQChannel = I2C_DMA_TX_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);

			/* Configure and enable I2C DMA RX Channel interrupt */
			NVIC_InitStructure.NVIC_IRQChannel = I2C_DMA_RX_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
			NVIC_Init(&NVIC_InitStructure);  

			/*!< I2C DMA TX and RX channels configuration */
			/* Enable the DMA clock */
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

			/* Clear any pending flag on Rx Stream  */
			DMA_ClearFlag(I2C_DMA_STREAM_TX, I2C_TX_DMA_FLAG_FEIF | I2C_TX_DMA_FLAG_DMEIF | I2C_TX_DMA_FLAG_TEIF | \
											   I2C_TX_DMA_FLAG_HTIF | I2C_TX_DMA_FLAG_TCIF);
			/* Disable the EE I2C Tx DMA stream */
			DMA_Cmd(I2C_DMA_STREAM_TX, DISABLE);
			/* Configure the DMA stream for the EE I2C peripheral TX direction */
			DMA_DeInit(I2C_DMA_STREAM_TX);
			DMA_InitStructure.DMA_Channel = I2C_DMA_CHANNEL;
			DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)I2C_DR_Address;
			DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)0;    /* This parameter will be configured durig communication */;
			DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral; /* This parameter will be configured durig communication */
			DMA_InitStructure.DMA_BufferSize = 0xFFFF;              /* This parameter will be configured durig communication */
			DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
			DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
			DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
			DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
			DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
			DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
			DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
			DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
			DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
			DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
			DMA_Init(I2C_DMA_STREAM_TX, &DMA_InitStructure);

			/* Clear any pending flag on Rx Stream */
			DMA_ClearFlag(I2C_DMA_STREAM_RX, I2C_RX_DMA_FLAG_FEIF | I2C_RX_DMA_FLAG_DMEIF | I2C_RX_DMA_FLAG_TEIF | \
											   I2C_RX_DMA_FLAG_HTIF | I2C_RX_DMA_FLAG_TCIF);
			/* Disable the EE I2C DMA Rx stream */
			DMA_Cmd(I2C_DMA_STREAM_RX, DISABLE);
			/* Configure the DMA stream for the EE I2C peripheral RX direction */
			DMA_DeInit(I2C_DMA_STREAM_RX);
			DMA_Init(I2C_DMA_STREAM_RX, &DMA_InitStructure);

			/* Enable the DMA Channels Interrupts */
			DMA_ITConfig(I2C_DMA_STREAM_TX, DMA_IT_TC, ENABLE);
			DMA_ITConfig(I2C_DMA_STREAM_RX, DMA_IT_TC, ENABLE);      
#endif
			flag = 1;
			break;
		default:
			return ;
	}

	if (flag) {
		/* Enable the I2C peripheral */
		I2C_Init(i2c_bus[ch], &I2C_InitStructure);
		I2C_Cmd(i2c_bus[ch], ENABLE);  
#ifdef I2C_USE_DMA
		I2C_DMACmd(i2c_bus[ch], ENABLE);
#endif
	}
}

int8_t i2c_write_byte(i2c_dev *dev, uint8_t reg, uint8_t value)
{
	return i2c_write_bytes(dev, reg, 1, &value);
}

int8_t i2c_write_bytes(i2c_dev *dev, uint8_t reg, uint32_t len, uint8_t *data)
{
	uint8_t tx_buf[I2C_MAX_BUF_LEN];
	uint8_t i;

	i2c_tx_state = 1;
	tx_buf[0] = reg;
	for (i=0; i<len; i++) {
		tx_buf[1+i] = data[i];
	}

	i2c_msg msg = {dev, 0, 0, tx_buf, (void *)0, len+1, 0};
	return i2c_xfer(&msg);
}

uint8_t i2c_read_byte(i2c_dev *dev, uint8_t reg)
{
	uint8_t value;
	i2c_read_bytes(dev, reg, 1, &value);

	return value;
}

int8_t i2c_read_bytes(i2c_dev *dev, uint8_t reg, uint32_t len, uint8_t *data)
{
	uint8_t tx_buf[I2C_MAX_BUF_LEN];
	
	i2c_rx_state = 1;
	tx_buf[0] = reg;
	i2c_msg msg = {dev, 0, 0, tx_buf, data, 1, len};
	return i2c_xfer(&msg);
}

int8_t i2c_write_bit(i2c_dev *dev, uint8_t reg, uint8_t bit, uint8_t val)
{
	uint8_t byte, mask;

	mask = 1 << bit;
	mask = ~mask;
	byte = i2c_read_byte(dev, reg) & mask;
	byte |= (val << bit);

	//serial_print("bit: %d mask: 0x%02x value: 0x%02x 0x%02x\r\n", bit, mask, val, byte);

	return i2c_write_byte(dev, reg, byte);
}

int8_t i2c_write_bits(i2c_dev *dev, uint8_t reg, uint8_t bit, uint8_t len, uint8_t val)
{
	uint8_t byte, mask;

	mask = 0xff << (7 - bit); 
//	serial_print("1 mask: 0x%02x\r\n", mask);
	mask >>= (8 - len); 
//	serial_print("2 mask: 0x%02x\r\n", mask);
	mask <<= (bit - len + 1);
//	serial_print("3 mask: 0x%02x\r\n", mask);
	mask = ~mask;

//	serial_print("bit %d len %d mask: 0x%02x\r\n", bit, len, mask);

	byte = i2c_read_byte(dev, reg) & mask;
	byte |= (val << (bit - len +1));

//	serial_print("4 byte: 0x%02x\r\n", byte);


	return i2c_write_byte(dev, reg, byte);
}

uint8_t i2c_read_bit(i2c_dev *dev, uint8_t reg, uint8_t bit)
{
	return i2c_read_byte(dev, reg) & ( 1 << bit);
}

uint8_t i2c_read_bits(i2c_dev *dev, uint8_t reg, uint8_t bit, uint8_t len)
{
	uint8_t byte;

	byte = i2c_read_byte(dev, reg);
#if 0
	/* LSB */
	byte <<= (bit + len - 1);
	byte >>= (8 - len);
#endif

	/* MSB */
	byte <<= (7 - bit);
	byte >>= (8 - len);

	return byte;
}

#ifdef I2C_USE_DMA
void I2C_DMAConfig(uint32_t pBuffer, uint32_t BufferSize, uint32_t Direction)
{ 
	/* Initialize the DMA with the new parameters */
	if (Direction == sEE_DIRECTION_TX) {
		/* Configure the DMA Tx Stream with the buffer address and the buffer size */
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)pBuffer;
		DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;    
		DMA_InitStructure.DMA_BufferSize = (uint32_t)BufferSize;  
		DMA_Init(I2C_DMA_STREAM_TX, &DMA_InitStructure);  
	} else { 
		/* Configure the DMA Rx Stream with the buffer address and the buffer size */
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)pBuffer;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
		DMA_InitStructure.DMA_BufferSize = (uint32_t)BufferSize;      
		DMA_Init(I2C_DMA_STREAM_RX, &DMA_InitStructure);    
	  }
}

int8_t i2c_xfer(i2c_msg *msg)
{
	volatile uint32_t timeout;

	/*!< While the bus is busy */
	timeout = I2C_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(i2c_bus[msg->dev->bus], I2C_FLAG_BUSY)) {
		if((timeout--) == 0) return -1;
	}
  
	/*!< Send START condition */
	I2C_GenerateSTART(i2c_bus[msg->dev->bus], ENABLE);
  
	/*!< Test on EV5 and clear it (cleared by reading SR1 then writing to DR) */
	timeout = I2C_FLAG_TIMEOUT;
	while(!I2C_CheckEvent(i2c_bus[msg->dev->bus], I2C_EVENT_MASTER_MODE_SELECT)) {
		if((timeout--) == 0) return -1;
	}
  
	/*!< Send EEPROM address for write */
	I2C_Send7bitAddress(i2c_bus[msg->dev->bus], msg->dev->addr, I2C_Direction_Transmitter);

	/*!< Test on EV6 and clear it */
	timeout = I2C_FLAG_TIMEOUT;
	while(!I2C_CheckEvent(i2c_bus[msg->dev->bus], I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
		if((timeout--) == 0) return -1;
	} 

	I2C_SendData(i2c_bus[msg->dev->bus], msg->tx_buf[msg->tx_idx++]);

	/*!< Test on EV8 and clear it */
	timeout = I2C_FLAG_TIMEOUT;
	while(!I2C_CheckEvent(i2c_bus[msg->dev->bus], I2C_EVENT_MASTER_BYTE_TRANSMITTING)) {
		if((timeout--) == 0) return -1;
	}

	/*!< Test on EV8 and clear it */
	timeout = I2C_FLAG_TIMEOUT;
	while(I2C_GetFlagStatus(i2c_bus[msg->dev->bus], I2C_FLAG_BTF) == RESET) {
		if((timeout--) == 0) return -1;
	}
  
	if (msg->rx_len > 0) {
		/*!< Send STRAT condition a second time */  
		I2C_GenerateSTART(i2c_bus[msg->dev->bus], ENABLE);
	  
		/*!< Test on EV5 and clear it (cleared by reading SR1 then writing to DR) */
		timeout = I2C_FLAG_TIMEOUT;
		while(!I2C_CheckEvent(i2c_bus[msg->dev->bus], I2C_EVENT_MASTER_MODE_SELECT)) {
			if((timeout--) == 0) return -1;
		} 
	  
		/*!< Send reg address for read */
		I2C_Send7bitAddress(i2c_bus[msg->dev->bus], msg->tx_buf[msg->tx_idx++], I2C_Direction_Receiver);  
	  
		/* If number of data to be read is 1, then DMA couldn't be used */
		/* One Byte Master Reception procedure (POLLING) ---------------------------*/
		if (msg->rx_len < 2) {
			/* Wait on ADDR flag to be set (ADDR is still not cleared at this level */
			timeout = I2C_FLAG_TIMEOUT;
			while(I2C_GetFlagStatus(i2c_bus[msg->dev->bus], I2C_FLAG_ADDR) == RESET) {
				if((timeout--) == 0) return -1;
			}     
		
			/*!< Disable Acknowledgement */
			I2C_AcknowledgeConfig(i2c_bus[msg->dev->bus], DISABLE);   
		
			/* Clear ADDR register by reading SR1 then SR2 register (SR1 has already been read) */
			(void)i2c_bus[msg->dev->bus]->SR2;
		
			/*!< Send STOP Condition */
			I2C_GenerateSTOP(i2c_bus[msg->dev->bus], ENABLE);
		
			/* Wait for the byte to be received */
			timeout = I2C_FLAG_TIMEOUT;
			while(I2C_GetFlagStatus(i2c_bus[msg->dev->bus], I2C_FLAG_RXNE) == RESET) {
				if((timeout--) == 0) return -1;
			}
		
			/*!< Read the byte received from the EEPROM */
			msg->rx_buf[msg->rx_idx++] = I2C_ReceiveData(i2c_bus[msg->dev->bus]);
		
			/*!< Decrement the read bytes counter */
			msg->rx_len--;        
		
			/* Wait to make sure that STOP control bit has been cleared */
			timeout = I2C_FLAG_TIMEOUT;
			while(i2c_bus[msg->dev->bus]->CR1 & I2C_CR1_STOP) {
				if((timeout--) == 0) return -1;
			}
		
			/*!< Re-Enable Acknowledgement to be ready for another reception */
			I2C_AcknowledgeConfig(i2c_bus[msg->dev->bus], ENABLE);    
		} else/* More than one Byte Master Reception procedure (DMA) -----------------*/ {
			/*!< Test on EV6 and clear it */
			timeout = I2C_FLAG_TIMEOUT;
			while(!I2C_CheckEvent(i2c_bus[msg->dev->bus], I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
				if((timeout--) == 0) return -1;
			}  

			/* Configure the DMA Rx Channel with the buffer address and the buffer size */
			I2C_DMAConfig((uint32_t)msg->rx_buf, (uint16_t)msg->rx_len, sEE_DIRECTION_RX);

			/* Inform the DMA that the next End Of Transfer Signal will be the last one */
			I2C_DMALastTransferCmd(i2c_bus[msg->dev->bus], ENABLE);

			/* Enable the DMA Rx Stream */
			DMA_Cmd(I2C_DMA_STREAM_RX, ENABLE);
		}

		/* If all operations OK, return sEE_OK (0) */
		while (i2c_rx_state != 0) ;
		return 0;
	}

	if (msg->tx_len > 0) {
		I2C_DMAConfig((uint32_t)(msg->tx_buf + msg->tx_idx), (uint16_t)(msg->tx_len - msg->tx_idx), sEE_DIRECTION_TX);
		DMA_Cmd(I2C_DMA_STREAM_TX, ENABLE);

		while (i2c_tx_state != 0) ;
		return 0;
	}
}

void I2C_DMA_TX_IRQHandler(void)
{
	serial_println("i2c tx irq");
	volatile uint32_t timeout;

	/* Check if the DMA transfer is complete */
	if(DMA_GetFlagStatus(I2C_DMA_STREAM_TX, I2C_TX_DMA_FLAG_TCIF) != RESET) {  
		/* Disable the DMA Tx Stream and Clear TC flag */  
		DMA_Cmd(I2C_DMA_STREAM_TX, DISABLE);
		DMA_ClearFlag(I2C_DMA_STREAM_TX, I2C_TX_DMA_FLAG_TCIF);

		/*!< Wait till all data have been physically transferred on the bus */
		timeout = I2C_LONG_TIMEOUT;
		while(!I2C_GetFlagStatus(I2C3, I2C_FLAG_BTF)) {
			if((timeout--) == 0) return;
		}
    
		/*!< Send STOP condition */
		I2C_GenerateSTOP(I2C3, ENABLE);
    
		/* Reset the variable holding the number of data to be written */
		i2c_tx_state = 0;  
	}
}

void I2C_DMA_RX_IRQHandler(void)
{
	serial_println("i2c rx irq");
	/* Check if the DMA transfer is complete */
	if(DMA_GetFlagStatus(I2C_DMA_STREAM_RX, I2C_RX_DMA_FLAG_TCIF) != RESET) {      
		/*!< Send STOP Condition */
		I2C_GenerateSTOP(I2C3, ENABLE);    
    
		/* Disable the DMA Rx Stream and Clear TC Flag */  
		DMA_Cmd(I2C_DMA_STREAM_RX, DISABLE);
		DMA_ClearFlag(I2C_DMA_STREAM_RX, I2C_RX_DMA_FLAG_TCIF);
    
		/* Reset the variable holding the number of data to be read */
		i2c_rx_state = 0;
	}
}
#else
int8_t i2c_xfer(i2c_msg *msg)
{
	volatile uint32_t timeout;
	int8_t status = 0;

	timeout = I2C_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(i2c_bus[msg->dev->bus], I2C_FLAG_BUSY)) {
		if((timeout--) == 0) {
			status = -1;
			goto error;
		}
	}

	if (msg->tx_len > 0) {
		/* Start the config sequence */
		I2C_GenerateSTART(i2c_bus[msg->dev->bus], ENABLE);

		/* Test on EV5 and clear it */
		timeout = I2C_LONG_TIMEOUT;
		while (!I2C_CheckEvent(i2c_bus[msg->dev->bus], I2C_EVENT_MASTER_MODE_SELECT)) {
			if((timeout--) == 0) {
				status = -2;
				goto error;
			}
		}

		/* Transmit the slave address and enable writing operation */
		I2C_Send7bitAddress(i2c_bus[msg->dev->bus], msg->dev->addr, I2C_Direction_Transmitter);

		/* Test on EV6 and clear it */
		timeout = I2C_LONG_TIMEOUT;
		while (!I2C_CheckEvent(i2c_bus[msg->dev->bus], I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
			if((timeout--) == 0) {
				status = -3;
				goto error;
			}
		}

		while (msg->tx_idx < msg->tx_len) {
			/* Transmit the first address for write operation */
			I2C_SendData(i2c_bus[msg->dev->bus], msg->tx_buf[msg->tx_idx++]);

			/* Test on EV8 and clear it */
			timeout = I2C_LONG_TIMEOUT;
			while (!I2C_CheckEvent(i2c_bus[msg->dev->bus], I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
				if((timeout--) == 0) {
					status = -4;
					goto error;
				}
			}
		}
	}

	if (msg->rx_len > 0) {
		/*!< Send START condition a second time */  
		I2C_GenerateSTART(i2c_bus[msg->dev->bus], ENABLE);

		/*!< Test on EV5 and clear it (cleared by reading SR1 then writing to DR) */
		timeout = I2C_LONG_TIMEOUT;
		while(!I2C_CheckEvent(i2c_bus[msg->dev->bus], I2C_EVENT_MASTER_MODE_SELECT)) {
			if((timeout--) == 0) {
				status = -5;
				goto error;
			}
		} 

		/*!< Send Codec address for read */
		I2C_Send7bitAddress(i2c_bus[msg->dev->bus], msg->dev->addr, I2C_Direction_Receiver);  

		/* Wait on ADDR flag to be set (ADDR is still not cleared at this level */
		timeout = I2C_LONG_TIMEOUT;
		while (!I2C_CheckEvent(i2c_bus[msg->dev->bus], I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
			if((timeout--) == 0) {
				status = -6;
				goto error;
			}
		}

		while (msg->rx_idx < msg->rx_len) {
			if (msg->rx_idx + 1 == msg->rx_len) {
				/*!< Disable Acknowledgment */
				I2C_AcknowledgeConfig(i2c_bus[msg->dev->bus], DISABLE);   

				/*!< Send STOP Condition */
				I2C_GenerateSTOP(i2c_bus[msg->dev->bus], ENABLE);
			}

			/* Wait for the byte to be received */
			timeout = I2C_LONG_TIMEOUT;
			while (!I2C_CheckEvent(i2c_bus[msg->dev->bus], I2C_EVENT_MASTER_BYTE_RECEIVED)) {
				if((timeout--) == 0) {
					status = -7;
					goto error;
				}
			}

			/*!< Read the byte received from the Codec */
			msg->rx_buf[msg->rx_idx++] = I2C_ReceiveData(i2c_bus[msg->dev->bus]);
		}

		/*!< Re-Enable Acknowledgment to be ready for another reception */
		I2C_AcknowledgeConfig(i2c_bus[msg->dev->bus], ENABLE);  
	} else {
		/* End the configuration sequence */
		I2C_GenerateSTOP(i2c_bus[msg->dev->bus], ENABLE);  
	}

error:
	return status;
}
#endif
