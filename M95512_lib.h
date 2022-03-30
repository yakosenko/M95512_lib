/*
 * M95512_lib.h
 *
 *  Created on: 24 вер. 2020 р.
 *      Author: Denys.S.Yakosenko
 */

#ifndef M95512_LIB_H_
#define M95512_LIB_H_

#include "stm32f10x.h"
#include "string.h"

#define M95512_vChipSelectActive() (M95512_SPI_GPIO_PORT_CS->BRR = M95512_SPI_PIN_CS)
#define M95512_vChipSelectDisable() (M95512_SPI_GPIO_PORT_CS->BSRR = M95512_SPI_PIN_CS)

#define M95512_vStartTransaction(u32TxPointer, u16Size)\
M95512_DMA_TX_CHANNEL->CMAR = u32TxPointer;\
M95512_DMA_TX_CHANNEL->CNDTR = u16Size;\
M95512_stContext.u16RxBytesCounter = u16Size;\
DMA_Cmd(M95512_DMA_TX_CHANNEL, ENABLE)

//----------------------------------DMA---------------------------------------------
#define M95512_DMA_RCC_AHBxPeriphClockCmd			RCC_AHBPeriphClockCmd
#define M95512_RCC_AHBxPeriph_DMAx					RCC_AHBPeriph_DMA1

#define M95512_DMA_TX_CHANNEL						DMA1_Channel5
#define M95512_DMA_TX_IRQn							DMA1_Channel5_IRQn
#define M95512_DMA_TX_IRQHandler					DMA1_Channel5_IRQHandler
#define M95512_DMA_TX_FLAG							DMA1_IT_TC5

#define M95512_BUFFSIZE								8
//----------------------------------SPI---------------------------------------------
#define M95512_SPI_CORE									SPI2
#define M95512_SPI_PORT_PeriphClockCmd					RCC_APB1PeriphClockCmd
#define M95512_SPI_PORT_Peripch							RCC_APB1Periph_SPI2
#define M95512_SPI_IRQn									SPI2_IRQn
#define M95512_SPI_IRQHandler							SPI2_IRQHandler
//-----------------------------------------------------------------------------------

//---------------------------------GPIO SPI------------------------------------------
//-----------------------------------CS----------------------------------------------
#define M95512_SPI_GPIO_PORT_CS							GPIOB
#define M95512_SPI_PIN_CS								GPIO_Pin_12
#define M95512_SPI_RCC_Peripch_GPIO_CS					RCC_APB2Periph_GPIOB
//----------------------------------SCLK---------------------------------------------
#define M95512_SPI_GPIO_PORT_SCLK						GPIOB
#define M95512_SPI_PIN_SCLK								GPIO_Pin_13
#define M95512_SPI_RCC_Peripch_GPIO_SCLK				RCC_APB2Periph_GPIOB
//----------------------------------MOSI---------------------------------------------
#define M95512_SPI_GPIO_PORT_MOSI						GPIOB
#define M95512_SPI_PIN_MOSI								GPIO_Pin_15
#define M95512_SPI_RCC_Peripch_GPIO_MOSI				RCC_APB2Periph_GPIOB
//----------------------------------MISO---------------------------------------------
#define M95512_SPI_GPIO_PORT_MISO						GPIOB
#define M95512_SPI_PIN_MISO								GPIO_Pin_14
#define M95512_SPI_RCC_Peripch_GPIO_MISO				RCC_APB2Periph_GPIOB

#define NOPE				0xFF
#define WRITE_ENABLE		0x06
#define WRITE_DISABLE		0x04
#define READ_STATUS_REG		0x05
#define WRITE_STATUS_REG	0x01
#define READ_MEM_ARRAY		0x03
#define WRITE_MEM_ARRAY		0x02

typedef struct
{
	uint8_t SRWDbit : 1;
	uint8_t nope : 3;
	uint8_t BPbits : 2;
	uint8_t WELbit : 1;
	uint8_t WIPbit : 1;
}M95512_tstStatusReg;

typedef union
{
	uint8_t StatusByte;
	M95512_tstStatusReg StatusStr;
}M95512_tunStatusReg;

typedef struct
{
	uint8_t u8Instruction;
	M95512_tunStatusReg StatusReg;
} M95512_tstStatusTrancStr;

typedef enum
{
	WAIT_STATE = 0,
	WRITE_ARRAY_WREN,
	WRITE_ARRAY_WREN_WAIT,
	WRITE_ARRAY_INSTR,
	WRITE_ARRAY_INSTR_WAIT,
	WRITE_ARRAY_SENDING,
	READ_ARRAY_INSTR,
	READ_ARRAY_INSTR_WAIT,
	READ_ARRAY_SENDING,
	WAIT_END_TRANS
} M95512_tenStates;

typedef struct
{
	M95512_tenStates enCurrentState;
	_Bool bFlagBusy;
	uint16_t u16CurrentMemoryAddr;
	uint16_t u16CurrentByteNum;
	uint32_t u32CurrentAddrTx;
	uint8_t *p8CurrentAddrRx;
	uint8_t *p8GlobalAddrRx;
	_Bool bTxFlag;
	_Bool bRxFlag;
	uint8_t u16RxBytesCounter;
	uint8_t u8RxIter;
	M95512_tstStatusTrancStr StatusReg;
} M95512_tstContext;

void M95512_vInit(void);
void M95512_vMain(void);

_Bool M95512_vGetStatusReg(void);
_Bool M95512_vSetBPBits(uint8_t u8NewBP);
_Bool M95512_vWriteEnableComm(void);
_Bool M95512_vWriteDisableComm(void);
_Bool M95512_bReadMemoryArray(uint16_t u16AddrMem, uint16_t u16ByteNum, uint32_t u32AddrRx);
_Bool M95512_bWriteMemoryArray(uint16_t u16AddrMem, uint16_t u16ByteNum, uint32_t u32AddrTx);

#endif /* M95512_LIB_H_ */
