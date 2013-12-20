#include "FreeRTOS.h"
//#include "semphr.h"
#include "i2c_bsp.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_it.h"
//#include "task.h"

#define I2C_MAX_BUF_LEN 64

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
uint8_t i2c_xfer(i2c_msg *msg);
i2c_msg *g_msg;
volatile uint8_t i2c_wait_flag;

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
		case 2:
			/* not implement */
			break;
		default:
			return ;
	}

	if (flag) {
		/* Enable the I2C peripheral */
		I2C_Cmd(i2c_bus[ch], ENABLE);  
		I2C_Init(i2c_bus[ch], &I2C_InitStructure);
	}
}

uint8_t i2c_write_byte(i2c_dev *dev, uint8_t reg, uint8_t value)
{
	uint8_t tx_buf[I2C_MAX_BUF_LEN] = {reg, value};

	i2c_msg msg = {dev, 0, 0, tx_buf, (void *)0, 2, 0};
	return i2c_xfer(&msg);
}

uint8_t i2c_write_bytes(i2c_dev *dev, uint8_t reg, uint32_t len, uint8_t *data)
{
	uint8_t tx_buf[I2C_MAX_BUF_LEN];
	uint8_t i;

	tx_buf[0] = reg;
	for (i=0; i<len; i++) {
		tx_buf[1+i] = data[i];
	}

	i2c_msg msg = {dev, 0, 0, tx_buf, (void *)0, len+1, 0};
	return i2c_xfer(&msg);
}

uint8_t i2c_read_byte(i2c_dev *dev, uint8_t reg)
{
	uint8_t tx_buf[I2C_MAX_BUF_LEN];
	uint8_t rx_buf[I2C_MAX_BUF_LEN];

	tx_buf[0] = reg;

	i2c_msg msg = {dev, 0, 0, tx_buf, rx_buf, 1, 1};
	i2c_xfer(&msg);

	return rx_buf[0];
}

uint8_t i2c_read_bytes(i2c_dev *dev, uint8_t reg, uint32_t len, uint8_t *data)
{
	uint8_t tx_buf[I2C_MAX_BUF_LEN];
	
	tx_buf[0] = reg;

	i2c_msg msg = {dev, 0, 0, tx_buf, data, 1, len};
	return i2c_xfer(&msg);
}

uint8_t i2c_write_bit(i2c_dev *dev, uint8_t reg, uint8_t bit, uint8_t val)
{
	uint8_t byte, mask;

	mask = 1 << bit;
	mask = ~mask;
	byte = i2c_read_byte(dev, reg) & mask;
	byte |= (val << bit);

//	serial_print("bit: %d mask: 0x%02x value: 0x%02x 0x%02x\r\n", bit, mask, val, byte);

	return i2c_write_byte(dev, reg, byte);
}

uint8_t i2c_write_bits(i2c_dev *dev, uint8_t reg, uint8_t bit, uint8_t len, uint8_t val)
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


uint8_t i2c_xfer(i2c_msg *msg)
{
	uint32_t timeout;

	portDISABLE_INTERRUPTS();

	timeout = 10000;
	while(I2C_GetFlagStatus(i2c_bus[msg->dev->bus_num], I2C_FLAG_BUSY)) {
		if((timeout--) == 0)
			return -1;
	}

	if (msg->tx_len > 0) {
		/* Start the config sequence */
		I2C_GenerateSTART(i2c_bus[msg->dev->bus_num], ENABLE);

		/* Test on EV5 and clear it */
		timeout = 10000;
		while (!I2C_CheckEvent(i2c_bus[msg->dev->bus_num], I2C_EVENT_MASTER_MODE_SELECT)) {
			if((timeout--) == 0)
				return -2;
		}

		/* Transmit the slave address and enable writing operation */
		I2C_Send7bitAddress(i2c_bus[msg->dev->bus_num], msg->dev->addr, I2C_Direction_Transmitter);

		/* Test on EV6 and clear it */
		timeout = 10000;
		while (!I2C_CheckEvent(i2c_bus[msg->dev->bus_num], I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
			if((timeout--) == 0)
				return -3;
		}

		while (msg->tx_idx < msg->tx_len) {
			/* Transmit the first address for write operation */
			I2C_SendData(i2c_bus[msg->dev->bus_num], msg->tx_buf[msg->tx_idx++]);

			/* Test on EV8 and clear it */
			timeout = 10000;
			while (!I2C_CheckEvent(i2c_bus[msg->dev->bus_num], I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
				if((timeout--) == 0)
					return -4;
			}
		}
	}

	if (msg->rx_len > 0) {
		/*!< Send START condition a second time */  
		I2C_GenerateSTART(i2c_bus[msg->dev->bus_num], ENABLE);

		/*!< Test on EV5 and clear it (cleared by reading SR1 then writing to DR) */
		timeout = 10000;
		while(!I2C_CheckEvent(i2c_bus[msg->dev->bus_num], I2C_EVENT_MASTER_MODE_SELECT)) {
			if((timeout--) == 0)
				return -5;
		} 

		/*!< Send Codec address for read */
		I2C_Send7bitAddress(i2c_bus[msg->dev->bus_num], msg->dev->addr, I2C_Direction_Receiver);  

		/* Wait on ADDR flag to be set (ADDR is still not cleared at this level */
		timeout = 10000;
		while (!I2C_CheckEvent(i2c_bus[msg->dev->bus_num], I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
			if((timeout--) == 0)
				return -6;
		}

		while (msg->rx_idx < msg->rx_len) {
			if (msg->rx_idx + 1 == msg->rx_len) {
				/*!< Disable Acknowledgment */
				I2C_AcknowledgeConfig(i2c_bus[msg->dev->bus_num], DISABLE);   

				/*!< Send STOP Condition */
				I2C_GenerateSTOP(i2c_bus[msg->dev->bus_num], ENABLE);
			}

			/* Wait for the byte to be received */
			timeout = 10000;
			while (!I2C_CheckEvent(i2c_bus[msg->dev->bus_num], I2C_EVENT_MASTER_BYTE_RECEIVED)) {
				if((timeout--) == 0)
					return -7;
			}

			/*!< Read the byte received from the Codec */
			msg->rx_buf[msg->rx_idx++] = I2C_ReceiveData(i2c_bus[msg->dev->bus_num]);
		}

		/*!< Re-Enable Acknowledgment to be ready for another reception */
		I2C_AcknowledgeConfig(i2c_bus[msg->dev->bus_num], ENABLE);  
	} else {
		/* End the configuration sequence */
		I2C_GenerateSTOP(i2c_bus[msg->dev->bus_num], ENABLE);  
	}

	portDISABLE_INTERRUPTS();

	return 0;
}


#if 0
uint32_t g_ev_log[100], g_ev_cnt = 0;

uint8_t i2c_xfer(i2c_msg *msg)
{
	uint32_t timeout, i;
	NVIC_InitTypeDef NVIC_InitStructure;

	g_msg = msg;

	timeout = 10;
	while(I2C_GetFlagStatus(i2c_bus[g_msg->dev->bus_num], I2C_FLAG_BUSY)) {
		serial_println("bus busy");
		if((timeout--) == 0) {
			return -1;
		}
	}
	g_ev_cnt = 0;

	I2C_GenerateSTART(i2c_bus[g_msg->dev->bus_num], ENABLE);
	I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, ENABLE);
	//I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_ERR, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	i2c_wait_flag = 1;
	while (i2c_wait_flag) ;
	I2C_AcknowledgeConfig(I2C1, ENABLE);
	Delay(100000);

	serial_println("xfer done");

	for (i=0; i<g_ev_cnt; i++) {
		serial_println("0x%08x", g_ev_log[i]);
	}

	return 0;
}

/* (1) send tx buf
 * (2) receive rx buf
 */

void I2C1_EV_IRQHandler(void)
{
	uint32_t event;

	event = I2C_GetLastEvent(I2C1);

	if (g_ev_cnt < 100) {
		g_ev_log[g_ev_cnt++] = event;
	}

	switch (event) {
		case I2C_EVENT_MASTER_MODE_SELECT:                 /* EV5 */
			serial_println("EV5, addr: 0x%02x", g_msg->dev->addr);
			if (g_msg->tx_idx < g_msg->tx_len) { /* tx buf not empty, send tx first */
				/* Master Transmitter ----------------------------------------------*/
				I2C_Send7bitAddress(I2C1, g_msg->dev->addr, I2C_Direction_Transmitter);
			} else if (g_msg->rx_idx < g_msg->rx_len) {
				/* Master Receiver -------------------------------------------------*/
				I2C_Send7bitAddress(I2C1, g_msg->dev->addr, I2C_Direction_Receiver);
			}
			break;

		case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:  /* EV6 */
			serial_println("EV6, reg: 0x%02x", g_msg->tx_buf[g_msg->tx_idx]);
			/* Send the first data */
			I2C_SendData(I2C1, g_msg->tx_buf[g_msg->tx_idx++]);
			break;

			/* Test on I2C1 EV8 and clear it */
		case I2C_EVENT_MASTER_BYTE_TRANSMITTING:  /* Without BTF, EV8 */    
			serial_println("EV8");
			if (g_msg->tx_idx < g_msg->tx_len) {
				serial_println("EV8, data: 0x%02x", g_msg->tx_buf[g_msg->tx_idx]);
				I2C_SendData(I2C1, g_msg->tx_buf[g_msg->tx_idx++]);
			} else {
				if (g_msg->rx_idx < g_msg->rx_len) { /* send repeat cond */
					serial_println("EV8, start rep");
					I2C_GenerateSTART(I2C1, ENABLE);
				} else {
					serial_println("EV8, tx complete");
					I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);
					I2C_GenerateSTOP(I2C1, ENABLE);
					i2c_wait_flag = 0;
				}
			}
			break;

		case I2C_EVENT_MASTER_BYTE_TRANSMITTED: /* With BTF EV8-2 */
			serial_println("EV8_2");
#if 0
			if (g_msg->tx_idx == g_msg->tx_len) {
				if (g_msg->rx_idx == g_msg->rx_len) {
					I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);
					serial_println("EV8_2, tx complete");
					I2C_GenerateSTOP(I2C1, ENABLE);
					i2c_wait_flag = 0;
				}
			}
#endif
			break;

			/* Master Receiver -------------------------------------------------------*/
		case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED:
			serial_println("EV6 Recv");
			if(g_msg->rx_len == 0 && g_msg->rx_len == 1) {
				serial_println("EV6 send stop");
				/* Disable I2C1 acknowledgement */
				I2C_AcknowledgeConfig(I2C1, DISABLE);
				/* Send I2C1 STOP Condition */
				I2C_GenerateSTOP(I2C1, ENABLE);
			}

#if 0
			if (g_msg->rx_idx + 1 == g_msg->rx_len) {
				I2C_AcknowledgeConfig(I2C1, DISABLE);
				I2C_GenerateSTOP(I2C1, ENABLE);
			}
			if(g_msg->rx_len == 1) {
				serial_println("EV6 Recv len eq 1");
				/* Disable I2C1 acknowledgement */
				I2C_AcknowledgeConfig(I2C1, DISABLE);
				/* Send I2C1 STOP Condition */
				I2C_GenerateSTOP(I2C1, ENABLE);
			}
#endif
			break;
   
			/* Test on I2C1 EV7 and clear it */
		case I2C_EVENT_MASTER_BYTE_RECEIVED:
			serial_println("EV7, receive data");
			g_msg->rx_buf[g_msg->rx_idx++] = I2C_ReceiveData(I2C1);
#if 0
			if (g_msg->rx_idx + 1 == g_msg->rx_len && g_msg->rx_len > 1) {
				serial_println("EV7, last byte");
				/* Disable I2C1 acknowledgement */
				I2C_AcknowledgeConfig(I2C1, DISABLE);
				/* Send I2C1 STOP Condition */
				I2C_GenerateSTOP(I2C1, ENABLE);
			}

			if (g_msg->rx_idx == g_msg->rx_len) {
				I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_ERR | I2C_IT_BUF, DISABLE);
				serial_println("EV7, rx complete");
				i2c_wait_flag = 0;
			}
#endif
			break;

		case 0x30044:
			serial_println("EV7_2");
			/* Store I2C1 received data */
			if (g_msg->rx_idx < g_msg->rx_len && g_msg->rx_len == 1)
				g_msg->rx_buf[g_msg->rx_idx++] = I2C_ReceiveData(I2C1);
#if 1
			if (g_msg->rx_idx + 1 == g_msg->rx_len && g_msg->rx_len > 1) {
				serial_println("EV7, send stop");
				/* Disable I2C1 acknowledgement */
				I2C_AcknowledgeConfig(I2C1, DISABLE);
				/* Send I2C1 STOP Condition */
				I2C_GenerateSTOP(I2C1, ENABLE);
			}

			if (g_msg->rx_idx == g_msg->rx_len) {
				serial_println("EV7, disable int");
				I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_ERR | I2C_IT_BUF, DISABLE);
				i2c_wait_flag = 0;
			}

#endif
			break;

		default:
			serial_println("unknown EV: 0x%08x", event);
	}
}

void I2C1_ER_IRQHandler(void) {
	serial_println("i2c1 error");
	/* Check on I2C2 AF flag and clear it */
	if (I2C_GetITStatus(I2C1, I2C_IT_AF))
		I2C_ClearITPendingBit(I2C2, I2C_IT_AF);
}
#endif
