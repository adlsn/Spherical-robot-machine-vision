
#include "stm32f4xx.h"
#include "uart.h"
#include "String.h"


#define BUFFER_SIZE1 50
static DMA_InitTypeDef DMA_InitStructureTx;
static DMA_InitTypeDef DMA_InitStructureRx;
static char TxBuffer[BUFFER_SIZE1];
static char RxBuffer[BUFFER_SIZE1];
__IO u16 RxCount = 0;

void USART1_Init(void)   //´®¿Ú³õÊ¼»¯
{
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA2, ENABLE);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    USART_DMACmd(USART1, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);
		
    DMA_InitStructureTx.DMA_BufferSize = BUFFER_SIZE1 ;
    DMA_InitStructureTx.DMA_FIFOMode = DMA_FIFOMode_Disable ;
    DMA_InitStructureTx.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
    DMA_InitStructureTx.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
    DMA_InitStructureTx.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructureTx.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructureTx.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructureTx.DMA_PeripheralBaseAddr =(uint32_t) (&(USART1->DR)) ;
    DMA_InitStructureTx.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_InitStructureTx.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructureTx.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructureTx.DMA_Priority = DMA_Priority_High;
    /* Configure TX DMA */
    DMA_InitStructureTx.DMA_Channel = DMA_Channel_4 ;
    DMA_InitStructureTx.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
    DMA_InitStructureTx.DMA_Memory0BaseAddr =(uint32_t)TxBuffer ;
    DMA_Init(DMA2_Stream7,&DMA_InitStructureTx);
    DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);
    DMA_InitStructureRx.DMA_BufferSize = BUFFER_SIZE1 ;
    DMA_InitStructureRx.DMA_FIFOMode = DMA_FIFOMode_Disable ;
    DMA_InitStructureRx.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
    DMA_InitStructureRx.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
    DMA_InitStructureRx.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructureRx.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructureRx.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructureRx.DMA_PeripheralBaseAddr =(uint32_t) (&(USART1->DR)) ;
    DMA_InitStructureRx.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_InitStructureRx.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructureRx.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructureRx.DMA_Priority = DMA_Priority_High;
    /* Configure RX DMA */
    DMA_InitStructureRx.DMA_Channel = DMA_Channel_4 ;
    DMA_InitStructureRx.DMA_DIR = DMA_DIR_PeripheralToMemory ;
    DMA_InitStructureRx.DMA_Memory0BaseAddr =(uint32_t)RxBuffer ;
    DMA_Init(DMA2_Stream5,&DMA_InitStructureRx);
    DMA_Cmd(DMA2_Stream5,ENABLE);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART1, USART_IT_IDLE,ENABLE);
    USART_Cmd(USART1, ENABLE);
}

void USART1_IRQHandler(void)
{
	 if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
        USART_ReceiveData(USART1);
        RxCount = BUFFER_SIZE1 - DMA_GetCurrDataCounter(DMA2_Stream5);
        DMA_Cmd(DMA2_Stream5, DISABLE);
        DMA_DeInit(DMA2_Stream5);
        DMA_InitStructureRx.DMA_Memory0BaseAddr = (uint32_t)RxBuffer;
        DMA_InitStructureRx.DMA_BufferSize = BUFFER_SIZE1;
        DMA_Init(DMA2_Stream5, &DMA_InitStructureRx);
        DMA_Cmd(DMA2_Stream5, ENABLE);
    }
    USART_ClearITPendingBit(USART1, USART_IT_IDLE | USART_IT_PE | USART_IT_FE | USART_IT_NE);

}

u8 USART1_RecBuffer(u8 *buffer)
{
    u16 i;
    i = RxCount;
    RxCount = 0;
    memcpy(buffer, RxBuffer, i);
    return i;
}
u8 USART1_SendBuffer(u8 *buffer, u16 n)
{
    memcpy(TxBuffer, buffer, n);
    DMA_Cmd(DMA2_Stream7, DISABLE);
    DMA_DeInit(DMA2_Stream7);
    DMA_InitStructureTx.DMA_Memory0BaseAddr = (uint32_t)TxBuffer;
    DMA_InitStructureTx.DMA_BufferSize = n;
    DMA_Init(DMA2_Stream7, &DMA_InitStructureTx);
    DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);
    DMA_Cmd(DMA2_Stream7, ENABLE);
    return 0;
}


