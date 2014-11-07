#include "FreeRTOS.h"
#include "hal.h"
#include "semphr.h"
#include "io.h"

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED 
#pragma     data_alignment = 4 
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_vcp.h"
//Library config for this project!!!!!!!!!!!
#include "stm32f4xx_conf.h"

LINE_CODING linecoding =
  {
    115200, /* baud rate*/
    0x00,   /* stop bits-1*/
    0x00,   /* parity - none*/
    0x08    /* nb. of bits 8*/
  };


extern uint8_t  APP_Rx_Buffer [];
extern uint32_t APP_Rx_ptr_in;

static uint16_t VCP_Init     (void);
static uint16_t VCP_DeInit   (void);
static uint16_t VCP_Ctrl     (uint32_t Cmd, uint8_t* Buf, uint32_t Len);
static uint16_t VCP_DataTx   (uint8_t* Buf, uint32_t Len);
static uint16_t VCP_DataRx   (uint8_t* Buf, uint32_t Len);

CDC_IF_Prop_TypeDef VCP_fops = 
{
  VCP_Init,
  VCP_DeInit,
  VCP_Ctrl,
  VCP_DataTx,
  VCP_DataRx
};

static uint16_t VCP_Init(void)
{
  return USBD_OK;
}

static uint16_t VCP_DeInit(void)
{
  return USBD_OK;
}

static uint16_t VCP_Ctrl (uint32_t Cmd, uint8_t* Buf, uint32_t Len)
{ 
  switch (Cmd)
  {
  case SEND_ENCAPSULATED_COMMAND:
    /* Not  needed for this driver */
    break;

  case GET_ENCAPSULATED_RESPONSE:
    /* Not  needed for this driver */
    break;

  case SET_COMM_FEATURE:
    /* Not  needed for this driver */
    break;

  case GET_COMM_FEATURE:
    /* Not  needed for this driver */
    break;

  case CLEAR_COMM_FEATURE:
    /* Not  needed for this driver */
    break;

  case SET_LINE_CODING:
	/* Not  needed for this driver */ 
    break;

  case GET_LINE_CODING:
    Buf[0] = (uint8_t)(linecoding.bitrate);
    Buf[1] = (uint8_t)(linecoding.bitrate >> 8);
    Buf[2] = (uint8_t)(linecoding.bitrate >> 16);
    Buf[3] = (uint8_t)(linecoding.bitrate >> 24);
    Buf[4] = linecoding.format;
    Buf[5] = linecoding.paritytype;
    Buf[6] = linecoding.datatype; 
    break;

  case SET_CONTROL_LINE_STATE:
    /* Not  needed for this driver */
    break;

  case SEND_BREAK:
    /* Not  needed for this driver */
    break;    
    
  default:
    break;
  }

  return USBD_OK;
}

static uint16_t VCP_COMConfig(uint8_t Conf)
{
  return USBD_OK;
}

static uint16_t VCP_DataTx (uint8_t* Buf, uint32_t Len)
{
	uint32_t i=0;
	while(i < Len)
	{
		APP_Rx_Buffer[APP_Rx_ptr_in] = *(Buf + i);
		APP_Rx_ptr_in++;
  		i++;
		/* To avoid buffer overflow */
		if(APP_Rx_ptr_in == APP_RX_DATA_SIZE)
		{
			APP_Rx_ptr_in = 0;
		}  
	}
	
  return USBD_OK;
}

#define APP_TX_BUF_SIZE 128
uint8_t APP_Tx_Buffer[APP_TX_BUF_SIZE];
uint32_t APP_tx_ptr_head;
uint32_t APP_tx_ptr_tail;

static uint16_t VCP_DataRx (uint8_t* Buf, uint32_t Len)
{
  uint32_t i;

  for (i = 0; i < Len; i++)
  {
	 APP_Tx_Buffer[APP_tx_ptr_head] = *(Buf + i);
	 APP_tx_ptr_head++;
	  if(APP_tx_ptr_head == APP_TX_BUF_SIZE)
		 APP_tx_ptr_head = 0;

	  if(APP_tx_ptr_head == APP_tx_ptr_tail)
		 return USBD_FAIL;
  } 
	
  return USBD_OK;
}

int usb_vcp_io_init(bus_t bus)
{
	return 0;
}

int usb_vcp_request_dma(bus_t bus)
{
	return 0;
}

int usb_vcp_bus_enable(bus_t bus, void *arg)
{
	USBConfig();
	return 0;
}

int usb_vcp_bus_cfg(bus_t bus, void *cfg)
{
	return 0;
}

int usb_vcp_bus_xfer(bus_t bus, int32_t addr, char *buf, uint32_t len, DIRECTION dir)
{
	if (dir == IN)
		VCP_DataRx(buf, len);
	else if (dir == OUT)
		VCP_DataTx(buf, len);
	return 0;
}

struct hal_bus usb_vcp = {
	.name = "usb_vcp",
	.use_dma = false,
	.use_int = false,
	.bus = BUS(USB, 0),
	.io_init = usb_vcp_io_init,
	.request_dma = usb_vcp_request_dma,
	.bus_enable = usb_vcp_bus_enable,
	.bus_cfg = usb_vcp_bus_cfg,
	.xfer = usb_vcp_bus_xfer,
};

int usb_vcp_bus_init()
{
	hal_bus_register(&usb_vcp);

	return 0;
}

hal_driver_init(usb_vcp_bus_init);
