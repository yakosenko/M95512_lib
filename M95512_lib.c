/*
 * M95512_lib.c
 *
 *  Created on: 24 вер. 2020 р.
 *      Author: Denys.S.Yakosenko
 */

#include "M95512_lib.h"

void M95512_vInit_SPI_DMA(void);

uint8_t M95512_SPI_TXBUFF[M95512_BUFFSIZE];
uint8_t M95512_SPI_RXBUFF[M95512_BUFFSIZE];

M95512_tstContext M95512_stContext;

void M95512_vInit(void)
{
	M95512_vInit_SPI_DMA();
}

void M95512_vInit_SPI_DMA(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef  DMA_InitStructure;

	/* Enable clocking for hw port pins, sw CS pins ans SPI core */
	RCC_APB2PeriphClockCmd(	M95512_SPI_RCC_Peripch_GPIO_CS, ENABLE);
	RCC_APB2PeriphClockCmd(	M95512_SPI_RCC_Peripch_GPIO_SCLK, ENABLE);
	RCC_APB2PeriphClockCmd(	M95512_SPI_RCC_Peripch_GPIO_MOSI, ENABLE);
	RCC_APB2PeriphClockCmd(	M95512_SPI_RCC_Peripch_GPIO_MISO, ENABLE);

	M95512_SPI_PORT_PeriphClockCmd(M95512_SPI_PORT_Peripch, ENABLE);

	// Configuration Hardware pins
	GPIO_InitStructure.GPIO_Pin   	= M95512_SPI_PIN_SCLK;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  	= GPIO_Mode_AF_PP;
	GPIO_Init(M95512_SPI_GPIO_PORT_SCLK, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin   	= M95512_SPI_PIN_MOSI;
	GPIO_Init(M95512_SPI_GPIO_PORT_MOSI, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin   	= M95512_SPI_PIN_MISO;
	GPIO_InitStructure.GPIO_Mode  	= GPIO_Mode_IN_FLOATING;
	GPIO_Init(M95512_SPI_GPIO_PORT_MISO, &GPIO_InitStructure);


	/* Configure Software pins */
	GPIO_InitStructure.GPIO_Pin   = M95512_SPI_PIN_CS;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_Init(M95512_SPI_GPIO_PORT_CS, &GPIO_InitStructure);
	M95512_vChipSelectDisable();//

	/* SPI configuration */
	SPI_InitStructure.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode              = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize          = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL              = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA              = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS               = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
	SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB;

	SPI_Init(M95512_SPI_CORE, &SPI_InitStructure);
	SPI_Cmd(M95512_SPI_CORE, ENABLE);

	M95512_DMA_RCC_AHBxPeriphClockCmd(M95512_RCC_AHBxPeriph_DMAx, ENABLE);
	//DMA TX
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(M95512_SPI_CORE->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)M95512_SPI_TXBUFF;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = 0;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

	DMA_Init(M95512_DMA_TX_CHANNEL, &DMA_InitStructure);

	SPI_I2S_DMACmd(M95512_SPI_CORE, SPI_I2S_DMAReq_Tx, ENABLE);

	NVIC_EnableIRQ(M95512_SPI_IRQn);
	NVIC_EnableIRQ(M95512_DMA_TX_IRQn);

	SPI_I2S_ITConfig(M95512_SPI_CORE, SPI_I2S_IT_RXNE, ENABLE);
	DMA_ITConfig(M95512_DMA_TX_CHANNEL, DMA_IT_TC, ENABLE);
}

void M95512_DMA_TX_IRQHandler(void)
{
	if(DMA_GetITStatus(M95512_DMA_TX_FLAG) == SET)
	{
		DMA_ClearITPendingBit(M95512_DMA_TX_FLAG);
		DMA_Cmd(M95512_DMA_TX_CHANNEL, DISABLE);
		M95512_stContext.bTxFlag = 1;
	}
}

void M95512_SPI_IRQHandler(void)
{
	if (SPI_I2S_GetITStatus(M95512_SPI_CORE, SPI_I2S_IT_RXNE) != RESET)
	  {
		SPI_I2S_ClearITPendingBit(M95512_SPI_CORE, SPI_I2S_IT_RXNE);

		if(M95512_stContext.u16RxBytesCounter != 0)
		{
			if(M95512_stContext.p8CurrentAddrRx != NULL)
			{
				M95512_stContext.p8CurrentAddrRx[M95512_stContext.u8RxIter] = SPI_I2S_ReceiveData(M95512_SPI_CORE);
				(M95512_stContext.u8RxIter)++;
			}
			else
				SPI_I2S_ReceiveData(M95512_SPI_CORE);

			(M95512_stContext.u16RxBytesCounter)--;

			if(M95512_stContext.u16RxBytesCounter == 0)
				M95512_stContext.bRxFlag = 1;
		}
	  }
}

//-------------------------------------------------------------------------------------------------------------
_Bool M95512_vGetStatusReg(void)
{
	_Bool bOutput = 0;


	if(M95512_stContext.bFlagBusy != 1)
	{
		M95512_vChipSelectActive();

		M95512_SPI_TXBUFF[0] = READ_STATUS_REG;
		M95512_SPI_TXBUFF[1] = NOPE;

		M95512_stContext.p8CurrentAddrRx = &(M95512_stContext.StatusReg);

		M95512_vStartTransaction((uint32_t) M95512_SPI_TXBUFF, 2);

		M95512_stContext.enCurrentState = WAIT_END_TRANS;
		M95512_stContext.bFlagBusy = 1;
		bOutput = 1;
	}

	return bOutput;
}
/*
_Bool M95512_vSetBPBits(uint8_t u8NewBP)
{
	_Bool bOutput = 0;

	if(M95512_stContext.bFlagBusy != 1)
	{
		M95512_vChipSelectActive();

		stStatusReg.StatusReg.StatusStr.BPbits = u8NewBP & 0x03;

		M95512_SPI_TXBUFF[0] = WRITE_STATUS_REG;
		M95512_SPI_TXBUFF[1] = stStatusReg.StatusReg.StatusByte;

		M95512_stContext.p8CurrentAddrRx = NULL;

		M95512_vStartTransaction((uint32_t) M95512_SPI_TXBUFF, 2);

		M95512_stContext.enCurrentState = WAIT_END_TRANS;
		M95512_stContext.bFlagBusy = 1;
		bOutput = 1;
	}

	return bOutput;
}*/

_Bool M95512_vWriteEnableComm(void)
{
	_Bool bOutput = 0;

	if(M95512_stContext.bFlagBusy != 1)
	{
		M95512_vChipSelectActive();

		M95512_SPI_TXBUFF[0] = WRITE_ENABLE;

		M95512_stContext.p8CurrentAddrRx = NULL;

		M95512_vStartTransaction((uint32_t) M95512_SPI_TXBUFF, 1);

		M95512_stContext.enCurrentState = WAIT_END_TRANS;
		M95512_stContext.bFlagBusy = 1;
		bOutput = 1;
	}

	return bOutput;
}

_Bool M95512_vWriteDisableComm(void)
{
	_Bool bOutput = 0;

	if(M95512_stContext.bFlagBusy != 1)
	{
	M95512_vChipSelectActive();

	M95512_SPI_TXBUFF[0] = WRITE_DISABLE;

	M95512_stContext.p8CurrentAddrRx = NULL;

	M95512_vStartTransaction((uint32_t) M95512_SPI_TXBUFF, 1);

	M95512_stContext.enCurrentState = WAIT_END_TRANS;
	M95512_stContext.bFlagBusy = 1;
	bOutput = 1;
	}

	return bOutput;
}

_Bool M95512_bReadMemoryArray(uint16_t u16AddrMem, uint16_t u16ByteNum, uint32_t u32AddrRx)
{
	_Bool bOutput = 0;

	if(M95512_stContext.bFlagBusy != 1)
	{
		M95512_stContext.u16CurrentMemoryAddr = u16AddrMem;
		M95512_stContext.u16CurrentByteNum = u16ByteNum;
		M95512_stContext.p8GlobalAddrRx = (uint8_t*) u32AddrRx;

		M95512_stContext.enCurrentState = READ_ARRAY_INSTR;
		bOutput = 1;
		M95512_stContext.bFlagBusy = 1;
	}

	return bOutput;
}

_Bool M95512_bWriteMemoryArray(uint16_t u16AddrMem, uint16_t u16ByteNum, uint32_t u32AddrTx)
{
	_Bool bOutput = 0;

	if(M95512_stContext.bFlagBusy != 1)
	{
		M95512_stContext.u16CurrentMemoryAddr = u16AddrMem;
		M95512_stContext.u16CurrentByteNum = u16ByteNum;
		M95512_stContext.u32CurrentAddrTx = u32AddrTx;
		M95512_stContext.p8CurrentAddrRx = NULL; // dont need reading feedback

		M95512_stContext.enCurrentState = WRITE_ARRAY_WREN;
		bOutput = 1;
		M95512_stContext.bFlagBusy = 1;
	}

	return bOutput;
}

void M95512_vMain(void)
{
	switch(M95512_stContext.enCurrentState)
	{

	case WAIT_STATE:
	{

	} break;

	case WRITE_ARRAY_WREN:
	{
		M95512_vChipSelectActive();

		M95512_SPI_TXBUFF[0] = WRITE_ENABLE;

		M95512_vStartTransaction((uint32_t) M95512_SPI_TXBUFF, 1);

		M95512_stContext.enCurrentState = WRITE_ARRAY_WREN_WAIT;
	} break;

	case WRITE_ARRAY_WREN_WAIT:
	{
		if(M95512_stContext.bTxFlag == 1 && M95512_stContext.bRxFlag == 1)
		{
			M95512_vChipSelectDisable();
			M95512_stContext.bTxFlag = 0;
			M95512_stContext.bRxFlag = 0;
			M95512_stContext.u8RxIter = 0;
			M95512_stContext.enCurrentState = WRITE_ARRAY_INSTR;
		}
	} break;

	case WRITE_ARRAY_INSTR:
	{
		M95512_vChipSelectActive();

		M95512_SPI_TXBUFF[0] = WRITE_MEM_ARRAY;
		M95512_SPI_TXBUFF[1] = (uint8_t) ((M95512_stContext.u16CurrentMemoryAddr & 0xFF00) >> 8);
		M95512_SPI_TXBUFF[2] = (uint8_t) (M95512_stContext.u16CurrentMemoryAddr & 0x00FF);

		M95512_vStartTransaction((uint32_t) M95512_SPI_TXBUFF, 3);

		M95512_stContext.enCurrentState = WRITE_ARRAY_INSTR_WAIT;
	} break;

	case WRITE_ARRAY_INSTR_WAIT:
	{
		if(M95512_stContext.bTxFlag == 1 && M95512_stContext.bRxFlag == 1)
		{
			M95512_stContext.bTxFlag = 0;
			M95512_stContext.bRxFlag = 0;
			M95512_stContext.u8RxIter = 0;
			M95512_stContext.enCurrentState = WRITE_ARRAY_SENDING;
		}
	} break;

	case WRITE_ARRAY_SENDING:
	{
		M95512_vStartTransaction((uint32_t) M95512_stContext.u32CurrentAddrTx, M95512_stContext.u16CurrentByteNum);

		M95512_stContext.enCurrentState = WAIT_END_TRANS;
	} break;

	case READ_ARRAY_INSTR:
	{
		M95512_vChipSelectActive();

		M95512_SPI_TXBUFF[0] = READ_MEM_ARRAY;
		M95512_SPI_TXBUFF[1] = (uint8_t) ((M95512_stContext.u16CurrentMemoryAddr & 0xFF00) >> 8);
		M95512_SPI_TXBUFF[2] = (uint8_t) (M95512_stContext.u16CurrentMemoryAddr & 0x00FF);

		M95512_stContext.p8CurrentAddrRx = M95512_SPI_RXBUFF;

		M95512_vStartTransaction((uint32_t) M95512_SPI_TXBUFF, 3);

		M95512_stContext.enCurrentState = READ_ARRAY_INSTR_WAIT;
	} break;

	case READ_ARRAY_INSTR_WAIT:
	{
		if(M95512_stContext.bTxFlag == 1 && M95512_stContext.bRxFlag == 1)
		{
			M95512_stContext.bRxFlag = 0;
			M95512_stContext.u8RxIter = 0;
			M95512_stContext.bTxFlag = 0;
			M95512_stContext.enCurrentState = READ_ARRAY_SENDING;
		}
	} break;

	case READ_ARRAY_SENDING:
	{
		M95512_stContext.p8CurrentAddrRx = M95512_stContext.p8GlobalAddrRx;

		M95512_vStartTransaction((uint32_t) M95512_SPI_TXBUFF, M95512_stContext.u16CurrentByteNum);

		M95512_stContext.enCurrentState = WAIT_END_TRANS;
	} break;

	case  WAIT_END_TRANS:
	{
		if(M95512_stContext.bTxFlag == 1 && M95512_stContext.bRxFlag == 1)
		{
			M95512_vChipSelectDisable();

			M95512_stContext.bTxFlag = 0;
			M95512_stContext.bRxFlag = 0;
			M95512_stContext.u8RxIter = 0;
			M95512_stContext.u16RxBytesCounter = 0;
			M95512_stContext.bFlagBusy = 0;
			M95512_stContext.p8CurrentAddrRx = NULL;
			M95512_stContext.enCurrentState = WAIT_STATE;
		}
	} break;

	}
}
