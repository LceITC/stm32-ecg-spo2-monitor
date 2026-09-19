#include "DMA.h"

/* 32个槽位实际可保存31点，可吸收约310ms主循环阻塞。 */
#define DMA_ECG_QUEUE_SIZE 32U

/* DMA硬件每次直接写入的单个16位ADC目标。 */
static volatile uint16_t dma_sample;

/* ISR按时间顺序复制样本的软件环形队列。 */
static volatile uint16_t sample_queue[DMA_ECG_QUEUE_SIZE];

/* 主循环下一次读取的位置，仅在取样或队列溢出时推进。 */
static volatile uint8_t queue_read_index;

/* ISR下一次写入的位置，等于读指针时表示队列为空。 */
static volatile uint8_t queue_write_index;

/* 读取PRIMASK，供临界区结束时恢复进入前的全局中断状态。 */
static uint32_t DMA_ECG_GetInterruptState(void)
{
    uint32_t interrupt_state; /* 0表示进入函数前全局中断处于开启状态。 */

    __asm volatile("MRS %0, primask" : "=r"(interrupt_state));
    return interrupt_state;
}

/* 计算环形队列的下一个索引，到达数组末尾时回绕到0。 */
static uint8_t DMA_ECG_NextQueueIndex(uint8_t index)
{
    ++index;
    if (index == DMA_ECG_QUEUE_SIZE)
    {
        index = 0U;
    }

    return index;
}

/*
 * ADC1 固定映射到 DMA1 Channel1。ADC 数据寄存器和目标变量均为
 * 16 位宽，循环长度为 1，每次转换完成产生一次中断。中断将样本立即
 * 放入软件环形队列，避免主循环短暂忙于软件 I2C、OLED 或串口时丢点。
 */
void DMA_ECG_Init(void)
{
    DMA_InitTypeDef dma; /* DMA1 Channel1的数据宽度、方向和地址配置。 */
    NVIC_InitTypeDef nvic; /* DMA传输完成中断的NVIC优先级配置。 */

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

    queue_read_index = 0U;
    queue_write_index = 0U;
    DMA_Cmd(DMA1_Channel1, ENABLE);
}

bool DMA_ECG_TakeSample(uint16_t *sample)
{
    bool available; /* true表示进入临界区后发现队列非空。 */
    uint32_t interrupt_state; /* 保存进入临界区前的PRIMASK。 */

    if (sample == 0)
    {
        return false;
    }

    /* 临界区只覆盖一次 16 位复制和读指针更新，避免与 ISR 竞争。 */
    interrupt_state = DMA_ECG_GetInterruptState();
    __disable_irq();
    available = queue_read_index != queue_write_index;
    if (available)
    {
        *sample = sample_queue[queue_read_index];
        queue_read_index = DMA_ECG_NextQueueIndex(queue_read_index);
    }
    if (interrupt_state == 0U)
    {
        __enable_irq();
    }

    return available;
}

/* 来源切换时丢弃旧通道样本，并清除DMA1 Channel1全部中断标志。 */
void DMA_ECG_Reset(void)
{
    uint32_t interrupt_state; /* 保存清队列前的全局中断状态。 */

    interrupt_state = DMA_ECG_GetInterruptState();
    __disable_irq();
    queue_read_index = 0U;
    queue_write_index = 0U;
    DMA_ClearITPendingBit(DMA1_IT_GL1);
    if (interrupt_state == 0U)
    {
        __enable_irq();
    }
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
        uint8_t next_write_index; /* 当前样本写入后应到达的队列位置。 */

        DMA_ClearITPendingBit(DMA1_IT_TC1);
        next_write_index = DMA_ECG_NextQueueIndex(queue_write_index);

        /*
         * 队列满时丢弃最旧样本并保留最新时序。正常 100 Hz 主循环不会
         * 触发此分支，31 个有效槽可吸收约 310 ms 的短暂阻塞。
         */
        if (next_write_index == queue_read_index)
        {
            queue_read_index = DMA_ECG_NextQueueIndex(queue_read_index);
        }

        sample_queue[queue_write_index] = dma_sample;
        queue_write_index = next_write_index;
    }
}
