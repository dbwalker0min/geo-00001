/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v2.0_Cube
  * @brief          : Usb device for Virtual Com Port.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
#include "SEGGER_RTT.h"
#include <cmsis_os2.h>
#include "usb_io.h"
#include <stdbool.h>

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */
/** manage the variable length circular buffer **/
static uint8_t *volatile rx_inptr, *volatile rx_outptr, *volatile rx_limit;
static uint8_t *tx_inptr, *tx_outptr, *tx_limit;
volatile bool host_port_open;

USBD_CDC_LineCodingTypeDef line_coding = {
    .bitrate = 115200,
    .datatype = 8,
    .format = 0,
    .paritytype = 0,
};

osEventFlagsId_t usb_events;

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  SEGGER_RTT_WriteString(0, "Initialize CDC\n");
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  rx_inptr = rx_outptr = UserRxBufferFS;
  tx_inptr = tx_outptr = UserTxBufferFS;

  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */

  switch (cmd) {

  case CDC_GET_LINE_CODING:
    SEGGER_RTT_WriteString(0, "Get line coding\n");
    memcpy(pbuf, &line_coding, sizeof(line_coding));
    break;

  case CDC_SET_LINE_CODING:
    SEGGER_RTT_WriteString(0, "Set line coding\n");
    break;

  case CDC_SET_CONTROL_LINE_STATE: {
    USBD_SetupReqTypedef *req = (USBD_SetupReqTypedef *) pbuf;

    SEGGER_RTT_printf(0, "Set line state %d\n", req->wValue);
    host_port_open = req->wValue & 1;
  }
    break;

  case CDC_SEND_BREAK:
  case CDC_SEND_ENCAPSULATED_COMMAND:
  case CDC_GET_ENCAPSULATED_RESPONSE:
  case CDC_SET_COMM_FEATURE:
  case CDC_GET_COMM_FEATURE:
  case CDC_CLEAR_COMM_FEATURE:
  default:
    SEGGER_RTT_printf(0, "Unhandled control request %d\n", cmd);
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */

  // add this to the buffer by specifying the new start address
  rx_inptr += *Len;

  // TODO: check for buffer overflow.
  // if the buffer doesn't have enough space for a complete packet, mark the end
  if (rx_inptr - UserRxBufferFS + CDC_DATA_FS_OUT_PACKET_SIZE > APP_RX_DATA_SIZE) {
    rx_limit = rx_inptr;
    rx_inptr = UserRxBufferFS;
  }
  osEventFlagsSet(usb_events, 1);

  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, rx_inptr);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/**
  * @brief  CDC_TransmitCplt_FS
  *         Data transmited callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 13 */
  unsigned tx_len;
  Buf = tx_outptr;

  if (tx_outptr > tx_inptr) {
    // send the last part of the buffer
    tx_len = APP_TX_DATA_SIZE - (tx_outptr - UserTxBufferFS);
    tx_outptr = UserTxBufferFS;
  } else {
    tx_len = tx_inptr - tx_outptr;
    tx_outptr = tx_inptr;
  }
  CDC_Transmit_FS(Buf, tx_len);


  UNUSED(Len);
  UNUSED(epnum);
  /* USER CODE END 13 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

static const osEventFlagsAttr_t usb_event_attributes = {
    .name = "USB",
};


void   init_usb_io() {
  usb_events = osEventFlagsNew(&usb_event_attributes);
}

char get_usb_char() {
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;

  while (!host_port_open || rx_inptr == rx_outptr) {
    osEventFlagsWait(usb_events, 1, osFlagsWaitAny, osWaitForever);
    osEventFlagsClear(usb_events, 1);
  }

  // pull a character out of the buffer
  char ch = *rx_outptr++;
  if (rx_outptr > rx_inptr && rx_outptr == rx_limit)
    rx_outptr = UserRxBufferFS;
  return ch;
}

void write_usb(void *buf, unsigned buf_len) {
  // copy the buffer to to the circular buffer
  if (tx_inptr + buf_len - UserTxBufferFS > APP_TX_DATA_SIZE) {
    // the new buffer spans the circular buffer so
    // copy in two stages
    unsigned len1 = tx_inptr - UserTxBufferFS;
    unsigned len2 = buf_len - len1;

    memcpy(tx_inptr, buf, len1);
    memcpy(UserTxBufferFS, buf + len1, buf_len - len1);
    tx_inptr = UserTxBufferFS + len2;
  } else {
    memcpy(tx_inptr, buf, buf_len);
    tx_inptr += buf_len;
  }

  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return ;
  }
  // start sending data by pretending that the last transmission was complete
  CDC_TransmitCplt_FS(0, 0, 0);

}

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
