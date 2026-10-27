#include "stm32f10x.h"
#include <stdint.h>

/*==========================================================
  ADS8689 GPIO定义
==========================================================*/

#define ADS8689_CS_LOW()      GPIO_ResetBits(GPIOA, GPIO_Pin_4)
#define ADS8689_CS_HIGH()     GPIO_SetBits(GPIOA, GPIO_Pin_4)

#define ADS8689_RST_LOW()     GPIO_ResetBits(GPIOB, GPIO_Pin_1)
#define ADS8689_RST_HIGH()    GPIO_SetBits(GPIOB, GPIO_Pin_1)

#define ADS8689_RVS_READ()    GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0)

/*==========================================================
  参数定义
==========================================================*/

#define STM32_CAN_ID              0x285

/*
 * ADS8689内部参考：
 * VREF = 4.096V
 *
 * 量程选择：
 * 0 ～ 1.25 × VREF
 * 0 ～ 5.12V
 */
#define ADS8689_FULL_SCALE_UV     5120000UL

#define ADS8689_RANGE_REG         0x14
#define ADS8689_RANGE_0_5V12      0x000B

/*
 * 采样任务计数。
 * 与单一flag相比，计数器不会因连续中断而直接覆盖前一次任务。
 */
volatile uint16_t g_sample_pending = 0;

volatile uint32_t g_ms = 0;

/*==========================================================
  函数声明
==========================================================*/

void RCC_Config(void);
void GPIO_Config(void);
void SPI1_Config(void);
void CAN_Config(void);
void TIM2_1000Hz_Config(void);
void SysTick_Config_ms(void);

void delay_ms(uint32_t ms);

uint8_t SPI1_TransferByte(uint8_t tx);
uint32_t ADS8689_Transfer32(uint32_t tx);

void ADS8689_Reset(void);
void ADS8689_WriteHalfWord(uint16_t address, uint16_t data);
void ADS8689_Init(void);

uint16_t ADS8689_ReadConversion(void);
uint32_t ADS8689_RawToMicrovolt(uint16_t raw);

void CAN_Send_ADS8689_Frame(uint16_t raw,
                            uint32_t voltage_uV,
                            uint8_t counter);

/*==========================================================
  主函数
==========================================================*/

int main(void)
{
    uint16_t raw;
    uint32_t voltage_uV;
    uint8_t counter = 0;

    RCC_Config();
    GPIO_Config();
    SPI1_Config();
    CAN_Config();
    SysTick_Config_ms();

    ADS8689_Init();

    /*
     * ADS8689的数据具有流水线关系。
     * 第一次读取可能是复位后的旧数据，因此先读一次并丢弃。
     * CS上升沿会启动下一次转换。
     */
    (void)ADS8689_ReadConversion();

    TIM2_1000Hz_Config();

    while (1)
    {
        if (g_sample_pending > 0)
        {
            __disable_irq();

            if (g_sample_pending > 0)
            {
                g_sample_pending--;
            }

            __enable_irq();

            /*
             * 读取上一周期完成的转换值，
             * 当前SPI帧结束后启动下一次转换。
             */
            raw = ADS8689_ReadConversion();

            voltage_uV = ADS8689_RawToMicrovolt(raw);

            CAN_Send_ADS8689_Frame(raw,
                                   voltage_uV,
                                   counter);

            counter++;
        }
    }
}

/*==========================================================
  RCC时钟
==========================================================*/

void RCC_Config(void)
{
    RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_GPIOA |
        RCC_APB2Periph_GPIOB |
        RCC_APB2Periph_AFIO  |
        RCC_APB2Periph_SPI1,
        ENABLE);

    RCC_APB1PeriphClockCmd(
        RCC_APB1Periph_TIM2 |
        RCC_APB1Periph_CAN1,
        ENABLE);
}

/*==========================================================
  GPIO
==========================================================*/

void GPIO_Config(void)
{
    GPIO_InitTypeDef gpio;

    /* PA4：ADS8689 CS */
    gpio.GPIO_Pin = GPIO_Pin_4;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &gpio);

    ADS8689_CS_HIGH();

    /* PA5：SPI1_SCK，PA7：SPI1_MOSI */
    gpio.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);

    /* PA6：SPI1_MISO */
    gpio.GPIO_Pin = GPIO_Pin_6;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    /* PB0：ADS8689 RVS */
    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &gpio);

    /* PB1：ADS8689 RST */
    gpio.GPIO_Pin = GPIO_Pin_1;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &gpio);

    ADS8689_RST_HIGH();

    /* PA11：CAN_RX */
    gpio.GPIO_Pin = GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio);

    /* PA12：CAN_TX */
    gpio.GPIO_Pin = GPIO_Pin_12;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);
}

/*==========================================================
  SPI1
==========================================================*/

void SPI1_Config(void)
{
    SPI_InitTypeDef spi;

    SPI_I2S_DeInit(SPI1);

    spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_Mode = SPI_Mode_Master;
    spi.SPI_DataSize = SPI_DataSize_8b;

    /*
     * ADS8689上电默认SPI-00-S：
     * CPOL=0
     * CPHA=0
     */
    spi.SPI_CPOL = SPI_CPOL_Low;
    spi.SPI_CPHA = SPI_CPHA_1Edge;

    spi.SPI_NSS = SPI_NSS_Soft;

    /*
     * PCLK2 = 72MHz
     * SPI时钟 = 72MHz / 16 = 4.5MHz
     *
     * 32位传输时间约：
     * 32 / 4.5MHz ≈ 7.1μs
     *
     * 对1ms采样周期完全足够。
     */
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;

    spi.SPI_FirstBit = SPI_FirstBit_MSB;
    spi.SPI_CRCPolynomial = 7;

    SPI_Init(SPI1, &spi);
    SPI_Cmd(SPI1, ENABLE);
}

uint8_t SPI1_TransferByte(uint8_t tx)
{
    uint32_t timeout = 100000;

    while (SPI_I2S_GetFlagStatus(
               SPI1,
               SPI_I2S_FLAG_TXE) == RESET)
    {
        if (--timeout == 0)
        {
            return 0;
        }
    }

    SPI_I2S_SendData(SPI1, tx);

    timeout = 100000;

    while (SPI_I2S_GetFlagStatus(
               SPI1,
               SPI_I2S_FLAG_RXNE) == RESET)
    {
        if (--timeout == 0)
        {
            return 0;
        }
    }

    return (uint8_t)SPI_I2S_ReceiveData(SPI1);
}

uint32_t ADS8689_Transfer32(uint32_t tx)
{
    uint32_t rx = 0;

    ADS8689_CS_LOW();

    rx |= ((uint32_t)SPI1_TransferByte(
               (uint8_t)(tx >> 24))) << 24;

    rx |= ((uint32_t)SPI1_TransferByte(
               (uint8_t)(tx >> 16))) << 16;

    rx |= ((uint32_t)SPI1_TransferByte(
               (uint8_t)(tx >> 8))) << 8;

    rx |= ((uint32_t)SPI1_TransferByte(
               (uint8_t)tx));

    /*
     * 等待SPI忙标志清零，
     * 防止最后一个位尚未发送完成就拉高CS。
     */
    while (SPI_I2S_GetFlagStatus(
               SPI1,
               SPI_I2S_FLAG_BSY) == SET)
    {
    }

    ADS8689_CS_HIGH();

    return rx;
}

/*==========================================================
  ADS8689
==========================================================*/

void ADS8689_Reset(void)
{
    ADS8689_RST_LOW();

    /*
     * 数据手册要求RST低脉冲至少100ns，
     * 这里延时1ms，裕量充分。
     */
    delay_ms(1);

    ADS8689_RST_HIGH();

    /*
     * 上电复位完成可能需要约20ms，
     * 这里等待25ms。
     */
    delay_ms(25);
}

void ADS8689_WriteHalfWord(uint16_t address,
                           uint16_t data)
{
    uint32_t command;

    /*
     * 半字写命令格式：
     *
     * 0xD0 + 9位寄存器地址 + 16位数据
     */
    command =
        0xD0000000UL |
        (((uint32_t)address & 0x01FFUL) << 16) |
        (uint32_t)data;

    (void)ADS8689_Transfer32(command);
}

void ADS8689_Init(void)
{
    ADS8689_Reset();

    /*
     * 设置输入范围：
     * 0 ～ 1.25 × 4.096V
     * 即0 ～ 5.12V
     */
    ADS8689_WriteHalfWord(
        ADS8689_RANGE_REG,
        ADS8689_RANGE_0_5V12);

    delay_ms(2);
}

uint16_t ADS8689_ReadConversion(void)
{
    uint32_t rx;
    uint32_t timeout = 100000;

    /*
     * 等待转换完成。
     * RVS在转换结束后变高。
     */
    while (ADS8689_RVS_READ() == Bit_RESET)
    {
        if (--timeout == 0)
        {
            /*
             * 超时返回0。
             * 后续可以增加错误标志。
             */
            return 0;
        }
    }

    /*
     * 发送NOP，同时读取上一次转换结果。
     */
    rx = ADS8689_Transfer32(0x00000000UL);

    /*
     * 默认输出数据格式下，
     * 16位转换值位于32位输出字的高16位。
     */
    return (uint16_t)(rx >> 16);
}

uint32_t ADS8689_RawToMicrovolt(uint16_t raw)
{
    /*
     * 单极性0～5.12V：
     *
     * voltage_uV =
     * raw × 5120000 / 65536
     *
     * 加32768用于四舍五入。
     */
    return (uint32_t)(
        ((uint64_t)raw *
         ADS8689_FULL_SCALE_UV +
         32768ULL) /
        65536ULL);
}

/*==========================================================
  TIM2：1000Hz
==========================================================*/

void TIM2_1000Hz_Config(void)
{
    TIM_TimeBaseInitTypeDef tim;
    NVIC_InitTypeDef nvic;

    /*
     * STM32F103常见72MHz系统时钟：
     *
     * APB1 = 36MHz
     * APB1定时器时钟 = 72MHz
     *
     * 72MHz / 7200 = 10kHz
     * 10kHz / 10 = 1000Hz
     *
     * 周期 = 1ms
     */
    tim.TIM_Prescaler = 7200 - 1;
    tim.TIM_Period = 10 - 1;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(TIM2, &tim);

    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    nvic.NVIC_IRQChannel = TIM2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&nvic);

    TIM_Cmd(TIM2, ENABLE);
}


/*==========================================================
  CAN初始化：500kbps
==========================================================*/

void CAN_Config(void)
{
    CAN_InitTypeDef can;
    CAN_FilterInitTypeDef filter;

    CAN_DeInit(CAN1);
    CAN_StructInit(&can);

    can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = ENABLE;
    can.CAN_AWUM = DISABLE;
    can.CAN_NART = DISABLE;
    can.CAN_RFLM = DISABLE;
    can.CAN_TXFP = DISABLE;
    can.CAN_Mode = CAN_Mode_Normal;

    /*
     * APB1=36MHz
     * 总TQ=1+13+4=18
     *
     * 36MHz / 4 / 18 = 500kbps
     */
    can.CAN_SJW = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_13tq;
    can.CAN_BS2 = CAN_BS2_4tq;
    can.CAN_Prescaler = 4;

    CAN_Init(CAN1, &can);

    filter.CAN_FilterNumber = 0;
    filter.CAN_FilterMode = CAN_FilterMode_IdMask;
    filter.CAN_FilterScale = CAN_FilterScale_32bit;
    filter.CAN_FilterIdHigh = 0x0000;
    filter.CAN_FilterIdLow = 0x0000;
    filter.CAN_FilterMaskIdHigh = 0x0000;
    filter.CAN_FilterMaskIdLow = 0x0000;
    filter.CAN_FilterFIFOAssignment = CAN_FIFO0;
    filter.CAN_FilterActivation = ENABLE;

    CAN_FilterInit(&filter);
}

/*==========================================================
  CAN发送
==========================================================*/

void CAN_Send_ADS8689_Frame(uint16_t raw,
                            uint32_t voltage_uV,
                            uint8_t counter)
{
    CanTxMsg tx;
    uint8_t mailbox;
    uint8_t status;
    uint32_t timeout = 10000;

    tx.StdId = STM32_CAN_ID;
    tx.ExtId = 0x00;
    tx.IDE = CAN_ID_STD;
    tx.RTR = CAN_RTR_DATA;
    tx.DLC = 8;

    /* Byte0～1：raw，大端 */
    tx.Data[0] = (uint8_t)(raw >> 8);
    tx.Data[1] = (uint8_t)raw;

    /* Byte2～5：电压μV，大端 */
    tx.Data[2] = (uint8_t)(voltage_uV >> 24);
    tx.Data[3] = (uint8_t)(voltage_uV >> 16);
    tx.Data[4] = (uint8_t)(voltage_uV >> 8);
    tx.Data[5] = (uint8_t)voltage_uV;

    tx.Data[6] = counter;
    tx.Data[7] = 0xA5;

    mailbox = CAN_Transmit(CAN1, &tx);

    /*
     * 这里设置短超时。
     * 不建议无限等待，否则可能破坏1kHz采样节拍。
     */
    do
    {
        status = CAN_TransmitStatus(CAN1, mailbox);
        timeout--;
    }
    while ((status == CAN_TxStatus_Pending) &&
           (timeout > 0));
}

/*==========================================================
  SysTick
==========================================================*/

void SysTick_Config_ms(void)
{
    SysTick_Config(SystemCoreClock / 1000);
}

void delay_ms(uint32_t ms)
{
    uint32_t start = g_ms;

    while ((g_ms - start) < ms)
    {
    }
}