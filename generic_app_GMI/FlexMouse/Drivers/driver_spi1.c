/**
  ********************************************************************************************************************************
  * @file    drv_spi1.c 
  * @author  Logan Schaufler
  * @brief   Main Driver function/s for serial protocol with SPI1 hardware
  * @details Protocol Modbus
  *          To Transmitt data : put data into SPI1SeqMem_Tx, and call this function
  *                              SPI_ITConfig(SPI1, SPI_IT_TXE, ENABLE);
  ********************************************************************************************************************************
  */

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "driver_spi1.h"
#include "main.h"
#include "macros.h"
#include "stm32g0xx_ll_spi.h"
#include "module_spi1.h"

#include <stdio.h>

// #include "module_AutoAck.h"
//#include "stm32g0xx_ll_crc.h"
//#include "stm32g0xx_ll_spi.h"
//#include "stm32g0xx_it.h"

/* Private variables ---------------------------------------------------------*/

//SPI1_Control *SPI1Control;
//SPI_HandleTypeDef hspi1;

void SPI1_TransferError_Callback(void);
//void SPI1_Tx_Callback(void);
void  SPI1_Rx_Callback(void);
void WaitAndCheckEndOfTransfer(void);
void Decode_SPI_Rx_Buffer(uint8_t*);

uint8_t aRxBuffer[3] = {0,0,0};
//uint8_t aRxBuffer[8] = {0,0,0,0,0,0,0,0};
//uint32_t aRxBuffer[8] = {0};
static uint8_t receive_index = 0;
//uint8_t ubNbDataToReceive = sizeof(aTxBuffer);

//LS EDIT
//uint8_t aTxBuffer[] = {1,2,3,4,5,6,7,8};
//uint8_t ubNbDataToTransmit = sizeof(aTxBuffer);
//__IO uint8_t TransmitIndex_u8 = 0;


void SPI1_Init(void)
{
  MX_SPI1_Init();

  /* Configure the SPI1 FIFO Threshold */
  LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_QUARTER);
  
   /* Enable SPI1 */
  //LL_SPI_Enable(SPI1);  //LS

  /* Configure SPI1 transfer interrupts */
  /* Enable TXE   Interrupt */
  //LL_SPI_EnableIT_TXE(SPI1);
  //LL_SPI_DisableIT_TXE(SPI1);  

  /* Enable RXNE  Interrupt */
  LL_SPI_EnableIT_RXNE(SPI1);
  
  /* Enable SPI1 Error Interrupt */
  LL_SPI_EnableIT_ERR(SPI1);

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

  //LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

  /**/    
  GPIO_InitStruct.Pin = SPI1_MISO_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = SPI1_GPIO_AF;
  LL_GPIO_Init(SPI1_MISO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = SPI1_GPIO_AF;
  LL_GPIO_Init(SPI1_MOSI_Port, &GPIO_InitStruct);
  
   /**/
  GPIO_InitStruct.Pin = SPI1_SCK_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = SPI1_GPIO_AF;
  LL_GPIO_Init(SPI1_SCK_Port, &GPIO_InitStruct);

  /* SPI1 interrupt Init */
  NVIC_SetPriority(SPI1_IRQn, 1);
  NVIC_EnableIRQ(SPI1_IRQn);
  //NVIC_DisableIRQ(SPI1_IRQn);

  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV256;    //Baud Rate = 250,000 kBit/sec
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 7;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_DisableNSSPulseMgt(SPI1);
  
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

void SPI1_Interrupt_Handler(void)
{
  /* Check RXNE flag value in ISR register */
  if(LL_SPI_IsActiveFlag_RXNE(SPI1))
  {
    /* Call function Slave Reception Callback */
    SPI1_Rx_Callback();
  }
  // /* LS NOT NEEDED IF THIS MICRO IS IN MASTER MODE Check RXNE flag value in ISR register */
  // else if(LL_SPI_IsActiveFlag_TXE(SPI1))
  // {
  //   /* Call function Slave Reception Callback */
  //   SPI1_Tx_Callback();
  // }

  /* Check STOP flag value in ISR register */
  else if(LL_SPI_IsActiveFlag_OVR(SPI1))
  {
    /* Call Error function */
    SPI1_TransferError_Callback();
  }
}

/**
  * @brief  Function called from SPI1 IRQ Handler when TXE flag is set
  *         Function is in charge  to transmit byte on SPI lines.
  * @param  None
  * @retval None
  */
// void SPI1_Tx_Callback(void)
// {
  /* Write character in Data register.
  TXE flag is cleared by reading data in DR register */

//}

/**
  * @brief  Function called in case of error detected in SPI IT Handler
  * @param  None
  * @retval None
  */
void SPI1_TransferError_Callback(void)
{
  /* Disable RXNE  Interrupt             */
  LL_SPI_DisableIT_RXNE(SPI1);

  /* Disable TXE   Interrupt             */
  LL_SPI_DisableIT_TXE(SPI1);
}

/**
  * @brief  Function called in case of error detected in SPI IT Handler
  * @param  None
  * @retval None
  */
void TransferHandler(void)
{
  LL_SPI_Enable(SPI1);
  WaitAndCheckEndOfTransfer();
}

/**
  * @brief  Wait end of transfer and check if received Data are well.
  * @param  None
  * @retval None
  */
void WaitAndCheckEndOfTransfer(void)
{
  // /* 1 - Wait end of transmission */
  //while (TransmitIndex_u8 != ubNbDataToTransmit)
  //while (test_index != ubNbDataToTransmit) 
  //{
  //}
  /* Disable TXE Interrupt */
  LL_SPI_DisableIT_TXE(SPI1);

  /* 2 - Wait end of reception */
  //while (ubNbDataToReceive > ubReceiveIndex)
  //{
  //}
  /* Disable RXNE Interrupt */
  LL_SPI_DisableIT_RXNE(SPI1);

  // /* 3 - Compare Transmit data to receive data */
  // if(Buffercmp8((uint8_t*)aTxBuffer, (uint8_t*)aRxBuffer, ubNbDataToTransmit))
  // {
  //   /* Processing Error */
  //   //LED_Blinking(LED_BLINK_ERROR);  //LS COMEBACK
  // }
  // else
  // {
  // }
}
    
/**
  * @brief  Function called from SPI1 IRQ Handler when RXNE flag is set
  *         Function is in charge of retrieving received byte from SPI lines.
  * @param  None
  * @retval None
  */
void  SPI1_Rx_Callback(void)
{
  /* Read character in Data register.
  RXNE flag is cleared by reading data in DR register */
  aRxBuffer[receive_index] = LL_SPI_ReceiveData8(SPI1);
  receive_index++;
  if(aRxBuffer[0] == 0x55)  //Only accept 0x55 Master sync byte
  {
    if(receive_index == sizeof(aRxBuffer)) //FUTURE COME UP WITH METHODOLOGY FOR MESSAGES THAT ARE LONGER THAN 3 BYTES
    {
      Decode_SPI_Rx_Buffer(aRxBuffer);
      receive_index = 0;
    }
  }
  else //Clear anything that does not have Master sync byte
  {
    receive_index = 0;
    for(uint8_t clearing_index = 0; clearing_index < sizeof(aRxBuffer); clearing_index++)
    {
      aRxBuffer[clearing_index] = 0;
    }
  }
  
}
