/*
 * ============================================================================
 *  ecg_acquisition.c —— ECG 100 Hz 采集链路实现
 *
 *  功能：初始化和管理 TIM2 → ADC1 → DMA1 的硬件采集链路，
 *        提供样本读取、来源切换和故障诊断接口。
 *
 *  硬件链路：
 *    TIM2 (100Hz PWM, CC2 内部事件)
 *      → ADC1 (单通道、外部触发、非连续转换)
 *        → DMA1 Channel1 (循环模式, 每次转换搬运一个16位值)
 *          → 软件环形队列 (由 DMA ISR 填充)
 *            → 主循环 (通过 ECG_Acquisition_TakeSample 消费)
 *
 *  故障诊断：
 *    当 GetTick() - last_sample_tick ≥ 200ms 时触发检查。
 *    按优先级定位：TIM2 时基 → DMA 中断 → ADC 触发链路。
 * ============================================================================
 */

#include "ecg_acquisition.h"
#include "ADC.h"
#include "DMA.h"
#include "TIM2.h"
#include "delay.h"

/*
 * ECG_SAMPLE_TIMEOUT_MS：采样超时阈值，200 ms。
 * 100 Hz 理论周期为 10 ms，200 ms ≈ 20 个采样周期。
 * 远超正常波动范围，可判定为链路故障。
 */
#define ECG_SAMPLE_TIMEOUT_MS 200U

/*
 * ---- 模块级静态变量 ----
 */

/*
 * last_sample_tick：最近一次成功从 DMA 队列取到 ECG 样本的系统毫秒时间。
 * 由 TakeSample 刷新，在 PollFault 中与当前时间比较判断超时。
 */
static uint32_t last_sample_tick;

/*
 * acquisition_available：ADC 校准成功且 TIM2 已经启动后置 1。
 * 为 0 时所有操作（取样本、切换来源）返回 false/不执行。
 */
static uint8_t acquisition_available;

/*
 * fault_reported：防止同一个持续故障在主循环中被重复报告。
 * 首次检测到故障时置 1，只有新样本到来（TakeSample 成功）时才清除。
 */
static uint8_t fault_reported;

/*
 * current_source：MCU 当前实际写入 ADC 规则序列的 ECG 来源。
 * 用于 SelectSource 中判断是否真的需要切换（避免重复切换）。
 */
static ECG_Source current_source;

/*
 * ============================================================================
 *  ECG_Acquisition_Init —— 初始化 100 Hz ECG 采集链路
 *
 *  初始化顺序（严格固定，不可调换）：
 *    ① DMA_ECG_Init() —— 先配置 DMA，使 ADC 启动时就有可用的搬运目标。
 *    ② ADC_ECG_Init() —— 配置 ADC 通道、触发源、校准，完成后使能外部触发。
 *    ③ TIM2_ECG_Init() + TIM2_ECG_Start() —— 最后启动时基，开始 100Hz 触发。
 *
 *  如果 ADC 校准失败（超时），TIM2 不会被启动，acquisition_available 保持 0。
 *
 *  返回值：ECG_AcquisitionInitStatus 枚举，指示初始化结果。
 * ============================================================================
 */
ECG_AcquisitionInitStatus ECG_Acquisition_Init(void)
{
    /*
     * adc_status：保存底层 ADC 校准结果，然后转换为上层枚举返回。
     * 如果 ADC 校准成功再继续初始化 TIM2。
     */
    ADC_ECG_InitStatus adc_status;

    /* 初始化开始前，所有标志处于安全状态 */
    acquisition_available = 0U;
    fault_reported = 0U;

    /* ========== 步骤①：初始化 DMA ========== */
    DMA_ECG_Init();

    /* ========== 步骤②：初始化 ADC（含校准） ========== */
    current_source = ECG_SOURCE_AD8232;
    adc_status = ADC_ECG_Init(ADC_ECG_SOURCE_AD8232);

    /* 复位校准超时 → 返回对应的错误码 */
    if (adc_status == ADC_ECG_INIT_RESET_CALIBRATION_TIMEOUT)
    {
        return ECG_ACQUISITION_INIT_RESET_CALIBRATION_TIMEOUT;
    }

    /* 正式校准超时 → 返回对应的错误码 */
    if (adc_status == ADC_ECG_INIT_CALIBRATION_TIMEOUT)
    {
        return ECG_ACQUISITION_INIT_CALIBRATION_TIMEOUT;
    }

    /* ========== 步骤③：初始化并启动 TIM2 ========== */
    TIM2_ECG_Init();

    /* 记录第一个样本的基准时间 */
    last_sample_tick = GetTick();

    /* 标记采集链路可用 */
    acquisition_available = 1U;

    /* 启动定时器，开始 100 Hz ADC 触发 */
    TIM2_ECG_Start();

    return ECG_ACQUISITION_INIT_OK;
}

/*
 * ============================================================================
 *  ECG_Acquisition_SelectSource —— 安全切换 ECG 输入源
 *
 *  切换时序（严格）：
 *    ① 停止 TIM2 → 阻止新的 ADC 转换触发
 *    ② 禁用 ADC 外部触发（双重保险）
 *    ③ 等待 20 μs → 确保正在进行的最后一次转换完成
 *    ④ DMA 软件队列复位 → 丢弃旧通道的残留样本
 *    ⑤ ADC 规则通道切换 → 指向新来源的 GPIO 引脚
 *    ⑥ 恢复 ADC 外部触发 + 启动 TIM2 → 新来源开始 100 Hz 采集
 *
 *  参数 source：目标 ECG 来源（ECG_SOURCE_AD8232 或 ECG_SOURCE_DIRECT）。
 *  返回值：
 *    true  — 切换成功
 *    false — 采集链路未就绪、参数无效或目标就是当前源（无切换必要）
 * ============================================================================
 */
bool ECG_Acquisition_SelectSource(ECG_Source source)
{
    /*
     * adc_source：与 source 对应的 ADC 模块底层通道枚举。
     * 在 ADC.h 中定义，是 ADC_ECG_Source 枚举类型。
     */
    ADC_ECG_Source adc_source;

    /*
     * 前置条件检查：
     * - 采集链路必须已经初始化成功
     * - source 必须是两个合法值之一
     */
    if (acquisition_available == 0U ||  (source != ECG_SOURCE_AD8232 && source != ECG_SOURCE_DIRECT) )
    {
        return false;
    }

    /*
     * 如果目标来源就是当前来源，不执行任何切换操作，直接返回 true。
     * 避免重复切换引入的不必要中断和样本丢失。
     */
    if (source == current_source)
    {
        return true;
    }

    /* 将上层枚举转换为 ADC 底层通道枚举 */
    adc_source = source == ECG_SOURCE_DIRECT ? ADC_ECG_SOURCE_DIRECT : ADC_ECG_SOURCE_AD8232;

    /* ========== 步骤①：停止 TIM2 触发 ========== */
    TIM2_ECG_Stop();

    /* ========== 步骤②：禁用 ADC 外部触发 ========== */
    ADC_ExternalTrigConvCmd(ADC1, DISABLE);

    /* ========== 步骤③：等待 20 μs ========== */
    DelayUs(20U);

    /* ========== 步骤④：清空 DMA 旧样本 ========== */
    DMA_ECG_Reset();

    /* ========== 步骤⑤：切换 ADC 通道 ========== */
    ADC_ECG_SelectSource(adc_source);

    /* 更新当前来源记录 */
    current_source = source;

    /*
     * 重新设置看门狗时间戳和新故障允许标志。
     * 确保切换后不会立即触发超时故障。
     */
    last_sample_tick = GetTick();
    fault_reported = 0U;

    /* ========== 步骤⑥：恢复触发和定时器 ========== */
    ADC_ExternalTrigConvCmd(ADC1, ENABLE);
    TIM2_ECG_Start();

    return true;
}

/*
 * ============================================================================
 *  ECG_Acquisition_TakeSample —— 从 DMA 软件队列取出一个 ECG 样本
 *
 *  作用：从 DMA ISR 填充的软件环形队列中取出一个按时间排序的样本。
 *  成功后：
 *    - 刷新 last_sample_tick（采样看门狗时间戳）
 *    - 清除 fault_reported 标志，允许后续新故障再次上报
 *
 *  参数 sample：接收 12 位 ADC 结果的无符号 16 位指针。
 *  返回值：
 *    true  — 成功取到样本，*sample 已被写入
 *    false — 队列为空、sample 为 NULL、或采集链路未就绪
 * ============================================================================
 */
bool ECG_Acquisition_TakeSample(uint16_t *sample)
{
    /*
     * 采集链路未就绪或 DMA 队列为空时返回 false。
     * DMA_ECG_TakeSample 内部已处理临界区保护。
     */
    if (acquisition_available == 0U || !DMA_ECG_TakeSample(sample))
    {
        return false;
    }

    /*
     * 成功取到样本：刷新看门狗时间戳。
     * 这确保 poll fault 不会因主循环短暂阻塞而误报。
     */
    last_sample_tick = GetTick();

    /*
     * 清除故障上报标志。新样本到来意味着之前的故障可能已经恢复，
     * 允许后续再次上报。
     */
    fault_reported = 0U;

    return true;
}

/*
 * ============================================================================
 *  ECG_Acquisition_PollFault —— 非阻塞检查 200ms 采样超时
 *
 *  诊断逻辑（按优先级降序）：
 *    ① 前置条件：链路就绪 + 尚未上报本次故障 + 距上次样本 ≥ 200ms
 *    ② TIM2 检查：TIM2_CC2 标志是否从未置位？
 *       → 是：TIM2 时基未启动或配置错误
 *    ③ DMA 检查：DMA TC 标志是否仍挂起（未被 ISR 清除）？
 *       → 是：DMA 中断链路异常（ISR 未触发或 NVIC 配置错误）
 *    ④ 综合诊断：TIM2 运行且 DMA 正常，但无样本
 *       → ADC 外部触发丢失或 ADC/DMA 请求握手异常
 *
 *  防重复上报机制：
 *    同一次持续故障只上报一次。fault_reported 置 1 后，
 *    只有新的样本到来（TakeSample 成功）才会清除此标志。
 *
 *  返回值：
 *    ECG_ACQUISITION_FAULT_NONE     — 正常，无故障
 *    ECG_ACQUISITION_FAULT_TIM2     — TIM2 时基故障
 *    ECG_ACQUISITION_FAULT_DMA      — DMA 中断链路异常
 *    ECG_ACQUISITION_FAULT_TRIGGER  — ADC/DMA 触发链路异常
 * ============================================================================
 */
ECG_AcquisitionFault ECG_Acquisition_PollFault(void)
{
    /*
     * 前置条件检查（三个条件同时满足才进入诊断）：
     * ① acquisition_available == 1：采集链路已初始化
     * ② fault_reported == 0：本次故障尚未上报过
     * ③ (GetTick() - last_sample_tick) >= 200ms：确实超时
     * 任一条件不满足 → 返回 NONE
     */
    if (acquisition_available == 0U || fault_reported != 0U ||
        (uint32_t)(GetTick() - last_sample_tick) < ECG_SAMPLE_TIMEOUT_MS)
    {
        return ECG_ACQUISITION_FAULT_NONE;
    }

    /* 标记已上报，防止重复报告 */
    fault_reported = 1U;

    /*
     * 诊断①：TIM2 CC2 事件检查
     * TIM2_ECG_CompareEventSeen() 返回 TIM2 SR 寄存器的 CC2IF 标志。
     * 如果从未产生过 CC2 事件，说明 TIM2 根本未运行或配置错误。
     */
    if (!TIM2_ECG_CompareEventSeen())
    {
        return ECG_ACQUISITION_FAULT_TIM2;
    }

    /*
     * 诊断②：DMA TC 标志检查
     * 正常工作时，DMA 每次传输完成触发 ISR，ISR 清除 TC 标志。
     * 如果 TC 标志仍挂起，说明上次传输完成后 ISR 未被调用。
     */
    if (DMA_ECG_TransferCompletePending())
    {
        return ECG_ACQUISITION_FAULT_DMA;
    }

    /*
     * 诊断③：综合链路异常
     * TIM2 产生了 CC2 事件，DMA TC 标志正常，但没有样本到达队列。
     * 可能是 ADC 外部触发配置丢失，或 ADC 到 DMA 的请求握手异常。
     */
    return ECG_ACQUISITION_FAULT_TRIGGER;
}
