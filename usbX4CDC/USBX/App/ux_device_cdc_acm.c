/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ux_device_cdc_acm.c
  * @author  MCD Application Team
  * @brief   USBX Device applicative file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "ux_device_cdc_acm.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "app_usbx_device.h"
#include "usb_drd_fs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_RX_DATA_SIZE                          (2048 * 4)
#define APP_TX_DATA_SIZE                          (2048 * 4)

/* Rx/TX flag */
#define RX_NEW_RECEIVED_DATA                      0x01
#define TX_NEW_TRANSMITTED_DATA                   0x02

/* Data length for vcp */
#define VCP_WORDLENGTH8                           8
#define VCP_WORDLENGTH9                           9

/* the minimum baudrate */
#define MIN_BAUDRATE                              9600
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
PCD_HandleTypeDef *hpcd = &hpcd_USB_DRD_FS;

/* Data received over uart are stored in this buffer */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/* Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* Increment this pointer or roll it back to
start address when data are received over USART */
uint32_t UserTxBufPtrIn;

/* Increment this pointer or roll it back to
start address when data are sent over USB */
uint32_t UserTxBufPtrOut;

/* uart3 handler */
//extern UART_HandleTypeDef huart1;
extern TX_EVENT_FLAGS_GROUP EventFlag;
UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_PARAMETER CDC_VCP_LineCoding =
{
  115200, /* baud rate */
  0x00,   /* stop bits-1 */
  0x00,   /* parity - none */
  0x08    /* nb. of bits 8 */
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
//static void USBD_CDC_VCP_Config(UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_PARAMETER *);
extern void Error_Handler(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @param  cdc Instance
  * @retval none
  */
void CDC_Init_FS(void *cdc_acm)
{

  /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
void CDC_DeInit_FS(void *cdc_acm)
{
  /* USER CODE BEGIN 4 */

  // causes device to not reregister
  //HAL_PCD_DeInit(hpcd);

  /* USER CODE END 4 */
}

/**
  * @brief  Function implementing usbx_cdc_acm_thread_entry.
  * @param arg: Not used
  * @retval None
  */
void usbx_cdc_acm_read_thread_entry(ULONG arg)
{
  UX_SLAVE_DEVICE *device;
  UX_SLAVE_INTERFACE *data_interface;
  UX_SLAVE_CLASS_CDC_ACM *cdc_acm;
  ULONG actual_length;
  ULONG ux_status = UX_SUCCESS;
  ULONG senddataflag = 0;

  /* Get device */
  device = &_ux_system_slave->ux_system_slave_device;
  tx_thread_sleep(25);

  while (1)
  {
    /* Check if device is configured */
    if (device->ux_slave_device_state == UX_DEVICE_CONFIGURED)
    {
      /* Get Data interface */
      tx_thread_sleep(5);
      data_interface = device->ux_slave_device_first_interface->ux_slave_interface_next_interface;

      /* Compares two memory blocks ux_slave_class_name and _ux_system_slave_class_cdc_acm_name */
      ux_status = ux_utility_memory_compare(data_interface->ux_slave_interface_class->ux_slave_class_name,
                                            _ux_system_slave_class_cdc_acm_name,
                                            ux_utility_string_length_get(_ux_system_slave_class_cdc_acm_name));

      /* Check Compares success */
      if (ux_status == UX_SUCCESS)
      {
        cdc_acm =  data_interface->ux_slave_interface_class_instance;

#ifndef UX_DEVICE_CLASS_CDC_ACM_TRANSMISSION_DISABLE
        /* Set transmission_status to UX_FALSE for the first time */
        cdc_acm -> ux_slave_class_cdc_acm_transmission_status = UX_FALSE;
#endif /* UX_DEVICE_CLASS_CDC_ACM_TRANSMISSION_DISABLE */

        /* Read the received data in blocking mode */
        ux_device_class_cdc_acm_read(cdc_acm, (UCHAR *)UserRxBufferFS, 64, &actual_length);
        if (actual_length != 0)
        {
        	ux_status = ux_device_class_cdc_acm_write(cdc_acm, (UCHAR *)UserRxBufferFS, actual_length, NULL);
        	continue;

          /* Wait until the requested flag TX_NEW_TRANSMITTED_DATA is received */
          if (tx_event_flags_get(&EventFlag, TX_NEW_TRANSMITTED_DATA, TX_OR_CLEAR,
                                 &senddataflag, TX_WAIT_FOREVER) != TX_SUCCESS)
          {
            Error_Handler();
          }
        }
      }
      tx_thread_sleep(10);
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    }
    else
    {
      tx_thread_sleep(1);
    }
  }
}

/**
  * @brief  Function implementing usbx_cdc_acm_write_thread_entry.
  * @param arg: Not used
  * @retval None
  */
void usbx_cdc_acm_write_thread_entry(ULONG arg)
{
  UX_SLAVE_DEVICE    *device;
  UX_SLAVE_INTERFACE *data_interface;
  UX_SLAVE_CLASS_CDC_ACM *cdc_acm;
  ULONG actual_length;
  ULONG receivedataflag = 0;
  ULONG buffptr;
  ULONG buffsize;
  UINT ux_status = UX_SUCCESS;
return;
  while (1)
  {
    /* Wait until the requested flag RX_NEW_RECEIVED_DATA is received */
    if (tx_event_flags_get(&EventFlag, RX_NEW_RECEIVED_DATA, TX_OR_CLEAR,
                           &receivedataflag, TX_WAIT_FOREVER) != TX_SUCCESS)
    {
      Error_Handler();
    }

    /* Get the device */
    device = &_ux_system_slave->ux_system_slave_device;

    /* Get the data interface */
    data_interface = device->ux_slave_device_first_interface->ux_slave_interface_next_interface;

    /* Get the cdc Instance */
    cdc_acm = data_interface->ux_slave_interface_class_instance;

#ifndef UX_DEVICE_CLASS_CDC_ACM_TRANSMISSION_DISABLE
    cdc_acm -> ux_slave_class_cdc_acm_transmission_status = UX_FALSE;
#endif

    /* Check if there is a new data to send */
    if (UserTxBufPtrOut != UserTxBufPtrIn)
    {
      /* Check buffer overflow and Rollback */
      if (UserTxBufPtrOut > UserTxBufPtrIn)
      {
        buffsize = APP_RX_DATA_SIZE - UserTxBufPtrOut;
      }
      else
      {
        /* Calculate data size */
        buffsize = UserTxBufPtrIn - UserTxBufPtrOut;
      }

      /* Copy UserTxBufPtrOut in buffptr */
      buffptr = UserTxBufPtrOut;

      /* Send data over the class cdc_acm_write */
      ux_status = ux_device_class_cdc_acm_write(cdc_acm,
                                                (UCHAR *)(&UserTxBufferFS[buffptr]),
                                                buffsize, &actual_length);

      /* Check if dataset is correctly transmitted */
      if (ux_status == UX_SUCCESS)
      {
        /* Increment the UserTxBufPtrOut pointer */
        UserTxBufPtrOut += buffsize;

        /* Rollback UserTxBufPtrOut if it equal to APP_TX_DATA_SIZE */
        if (UserTxBufPtrOut == APP_TX_DATA_SIZE)
        {
          UserTxBufPtrOut = 0;
        }
      }
    }
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
