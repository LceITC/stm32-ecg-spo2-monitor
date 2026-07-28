#include "DMA.h"

static volatile uint16_t dma_sample;
static volatile uint8_t sample_ready;

/*
 * ADC1 固定映射到 DMA1 Channel1。ADC 数据寄存器和目标变量均为
 * 16 位宽，循环长度为 1，每次转换完成产生一次中断。
 */
void DMA_ECG_Init(void)
{
    DMA_InitTypeDef dma;
    NVIC_InitTypeDef nvic;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit(DMA1_Channel1);
    dma.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
    dma.DMA_MemoryBaseAddr = (uint32_t)&dma_sample;
    dma.DMA_DIR = DMA_DIR_PeripheralSRC;
    dma.DMA_BufferSize = 1U;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Disable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_High;
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &dma);
    DMA_ClearITPendingBit(DMA1_IT_GL1);
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);

    nvic.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1U;
    nvic.NVIC_IRQChannelSubPriority = 0U;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    sample_ready = 0U;
    DMA_Cmd(DMA1_Channel1, ENABLE);
}

bool DMA_ECG_TakeSample(uint16_t *sample)
{
    bool available;

    if (sample == 0)
    {
        return false;
    }

    /* 临界区只覆盖一次 16 位复制和标志清除，避免与 ISR 竞争。 */
    __disable_irq();
    available = sample_ready != 0U;
    if (available)
    {
        *sample = dma_sample;
        sample_ready = 0U;
    }
    __enable_irq();

    return available;
}

void DMA_ECG_Reset(void)
{
    __disable_irq();
    sample_ready = 0U;
    DMA_ClearITPendingBit(DMA1_IT_GL1);
    __enable_irq();
}

/* 若 TC 标志存在但 ISR 未清除，说明 DMA 中断链路可能异常。 */
bool DMA_ECG_TransferCompletePending(void)
{
    return DMA_GetFlagStatus(DMA1_FLAG_TC1) == SET;
}

/* 中断名称必须与 GCC 启动文件中的向量名称完全一致。 */
void DMA1_Channel1_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_IT_TC1) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_IT_TC1);
        sample_ready = 1U;
    }
}
