//-----------------------------------------------------------------------------
// 作    者：adoredee
// 创建时间：2019.05.13
// 描    述：实验二 UART中断接收串口通信驱动LED显示
// 版    本：
//-----------------------------------------------------------------------------
// Copyright (C) 2019 Shanghai Jiao Tong University
//-----------------------------------------------------------------------------
#include "hw_psc_C6748.h"
#include "soc_C6748.h"
#include "interrupt.h"
#include "lcdkC6748.h"
#include "hw_types.h"        // 宏命令
#include "uart.h"
#include "psc.h"
#include "gpio.h"
#include "timer.h"
// 228MHz，定时一秒
#define TMR_PERIOD_LSB32               (0x0D970100)    // 低32位，0x0D970100 ——> 228 000 000
#define TMR_PERIOD_MSB32               (0x0)           // 高32位
#define _HWREG(x) (*((volatile Uint32*)(x)))
#define PINMUX0 _HWREG(0x01C14120)    // 多路寄存器0的地址 0X8000000
#define PINMUX5 _HWREG(0x01C14134)    // 多路寄存器5的地址 0X8000
#define PINMUX13 _HWREG(0x01C14154)   // 多路寄存器13的地址 0X8800
typedef unsigned int Uint32;

/****************************************************************************/
/*                         函数定义                                                                          */
/****************************************************************************/
static void TimerSetUp64Bit(void);    // 定时器初始化
static void TimerIntrSetUp(void);     // 定时器中断
static void UARTIsr(void);            // 中断服务函数ISR

/****************************************************************************/
/*                         全局变量                                                                          */
/****************************************************************************/
unsigned char k = 0;
unsigned int rxFlag = 0;        //判断是否有字符输入的标志
unsigned char rxData = '0';     //UART接收的单个字符
unsigned int rxASCII = 0;       //接收字符对应的ASCII码
unsigned int rxLow = 0;         //ASCII码对应的16进制数的低位
unsigned int rxHigh = 0;        //ASCII码对应的16进制数的高位
unsigned int rx[] = {0};        //存储一次性输入的所有字符对应的低位、高位的数组
unsigned int mode = 0;          //数组中不同数对应的LED点亮状态
unsigned int length = 0;        //用来记录数组中共有多少个数

/*多路寄存器13的15-12位置为8h，配置为GP6[12]功能 */
void _GPIOBank6Pin12PinMuxSetup()
{
    PINMUX13 &= ~(0xf << 12);    // 0X800，即先将多路寄存器13的15-12位置清零，类似关闭开关作用
    PINMUX13 |= (0x8 << 12);     // 0X8800，再将多路寄存器13的15-12位置赋值，类似打开开关作用
}

/*多路寄存器13的11-8位置为8h，配置为GP6[13]功能*/
void _GPIOBank6Pin13PinMuxSetup()
{
    PINMUX13 &= ~(0xf << 8);   // 0X8000
    PINMUX13 |= (0x8 << 8);    // 0X8800
}
/*多路寄存器5的15-12位置为8h，配置为GP2[12]功能*/
void _GPIOBank2Pin12PinMuxSetup()
{
    PINMUX5 &= ~(0xf << 12);    // 0X0
    PINMUX5 |= (0x8 << 12);     // 0X8000
}

/*多路寄存器0的27-24位置为8h，配置为GP0[9]功能*/
void _GPIOBank0Pin9PinMuxSetup()
{
    PINMUX0 &= ~(0xf << 24);    // 0X0
    PINMUX0 |= (0x8 << 24);     // 0X8000000
}

int main(void)
{
/****************************************************************************/
/*                                                                          */
/*                  GPIO使能配置                                                                                */
/*                                                                          */
/****************************************************************************/
    PSCModuleControl(SOC_PSC_1_REGS,
                     HW_PSC_GPIO,
                     PSC_POWERDOMAIN_ALWAYS_ON,
		             PSC_MDCTL_NEXT_ENABLE);

/****************************************************************************/
/*                                                                          */
/*                  GPIO 管脚复用配置                                                                       */
/*                                                                          */
/****************************************************************************/
	_GPIOBank6Pin12PinMuxSetup();
	_GPIOBank6Pin13PinMuxSetup();
	_GPIOBank2Pin12PinMuxSetup();
	_GPIOBank0Pin9PinMuxSetup();

/****************************************************************************/
/*                                                                          */
/*                   GPIO 管脚初始化                                                                         */
/*                                                                          */
/****************************************************************************/
	/*
	    GPIODirModeSet设置管脚的方向，设置GPIO的输出方向
	    第一个参数：GPIO模块在内存中的地址
	    第二个参数：GPIO口的序号，C6748共有144个序号
	*/
    GPIODirModeSet(SOC_GPIO_0_REGS, 109, GPIO_DIR_OUTPUT);    // GP6[12]的序号为109
    GPIODirModeSet(SOC_GPIO_0_REGS, 110, GPIO_DIR_OUTPUT);    // GP6[13]的序号为110
    GPIODirModeSet(SOC_GPIO_0_REGS, 45, GPIO_DIR_OUTPUT);     // GP2[12]的序号为45
    GPIODirModeSet(SOC_GPIO_0_REGS, 10, GPIO_DIR_OUTPUT);     // GP0[9] 的序号为10
    unsigned int intFlags = 0;
    unsigned int config = 0;

/****************************************************************************/
/*                                                                          */
/*                  UART串口初始化配置                                                                      */
/*                                                                          */
/****************************************************************************/
    // 1. 使能UART2模块
    PSCModuleControl(SOC_PSC_1_REGS,
                     HW_PSC_UART2,     // UART2的PSC模块
                     PSC_POWERDOMAIN_ALWAYS_ON,
		             PSC_MDCTL_NEXT_ENABLE);

    // 此函数选择要使用的UART引脚, UART引脚与片上系统(SoC)中其他外围设备的引脚进行复用。
    UARTPinMuxSetup(2, FALSE);

    // 使能UART2，启动发射器和接收器
    UARTEnable(SOC_UART_2_REGS);

    // 2. 配置UATR参数
    config = UART_WORDL_8BITS;    // 1个停止位，8位字符，没有奇偶校验

   /*    配置UART参数
   void UARTConfigSetExpClk(unsigned int  baseAdd,  // Uart在内存中的地址
                            unsigned int  uartClk,  // 提供给UART模块的时钟频率。
                            unsigned int  baudrate, // 波特率设置
                            unsigned int  config,   // 设置停止位、数据位数及奇偶性检验
                            unsigned int  overSampRate)  // 过采样率(13x或16x)。
   */
    UARTConfigSetExpClk(SOC_UART_2_REGS,
                        SOC_UART_2_MODULE_FREQ,
                        BAUD_115200,    // 波特率：115200
                        config,
                        UART_OVER_SAMP_RATE_16);

    //
    /*
    FIFO是先进先出缓冲区的意思，即串口接收到的数据可以先进入FIFO，不必马上进入中断服务程序接收，这样可以节省CPU时间。
       对于发送数据也一样可以把要发送的数据一起写入FIFO，串口控制器按照写入的顺序依次发送出去。
     */
    UARTFIFOEnable(SOC_UART_2_REGS);

    UARTFIFOLevelSet(SOC_UART_2_REGS, UART_RX_TRIG_LEVEL_1);

/************************************************************************* ***/
/*                                                                           */
/*                     UART串口中断 配置                                                                     */
/*                                                                           */
 /****************************************************************************/
    TimerSetUp64Bit();    // 定时器初始化
    TimerIntrSetUp();     // 定时器中断

    // 使能中断
    intFlags |= (UART_INT_LINE_STAT  |  \
                 UART_INT_TX_EMPTY |    \
                 UART_INT_RXDATA_CTI);

    TimerDisable(SOC_TMR_2_REGS, TMR_TIMER12);
    TimerIntEnable(SOC_TMR_2_REGS, TMR_INT_TMR12_NON_CAPT_MODE);
    UARTIntEnable(SOC_UART_2_REGS, intFlags);

    while(1);

}

/*
** \brief   Interrupt Service Routine(ISR) to be executed on UART interrupts.
**          Depending on the source of interrupt, this
**          1> writes to the serial communication console, or
**          2> reads from the serial communication console, or
**          3> reads the byte in RBR if receiver line error has occured.
*/
/*****************************************************************************/
/*                                                                           */
/*                    UART串口中断服务函数ISR                                 */
/*                                                                           */
/*****************************************************************************/
static void UARTIsr()
{
    unsigned int int_id = 0;
    // 确定中断源
    int_id = UARTIntStatus(SOC_UART_2_REGS);

#ifdef _TMS320C6X
    // Clear UART2 system interrupt in DSPINTC
    // 清除 UART2 系统中断
    IntEventClear(SYS_INT_UART2_INT);
#else
    /* Clears the system interupt status of UART2 in AINTC. */
    // 清除 UART2 系统中断
    IntSystemStatusClear(SYS_INT_UARTINT2);
#endif

    /* Checked if the cause is transmitter empty condition.*/
    // 接收中断
    if(UART_INTID_RX_DATA == int_id)
        {
        	// rxData即为接收到的字符
            rxData= UARTCharGetNonBlocking(SOC_UART_2_REGS);
            // 当该字符不是一次性输入的字符串的结束符时，执行以下操作 */
            if ((rxData != '\r') && (rxData != '\n'))
            {
            	// 将标志位置高电平
            	rxFlag = 1;
            	// 将该字符转化成ASCII码
            	rxASCII = rxData - '0' + 48;
            	// 将该ASCII码对应的16进制数低位存入数组
    			rxLow = rxASCII % 16;
    			rx[length] = rxLow;
    			length++;
    			// 将该ASCII码对应的16进制数高位存入数组
    			rxHigh = rxASCII / 16;
    			rx[length] = rxHigh;
    			// length用来统计数组中共多少个数
    			length++;
    	        /* Write a byte into the THR if THR is free. */
    	        UARTCharPutNonBlocking(SOC_UART_2_REGS, 'a');
            }
            /* 当该字符是一次性输入的字符串的结束符时，使能TIMER */
            else if (rxData=='\n')
            {
    	    /* Write a byte into the THR if THR is free. */
    	    UARTCharPutNonBlocking(SOC_UART_2_REGS, 'c');
            }
        }
    return;
}

/************************************************************************* ***/
/*                                                                           */
/*                  定时器中断服务函数                                                                        */
/*                                                                           */
 /****************************************************************************/
static void TimerIsr(void)
        {
            // 刚进入中断时要先关掉该中断
            TimerIntDisable(SOC_TMR_2_REGS, TMR_INT_TMR12_NON_CAPT_MODE);
        #ifdef _TMS320C6X
            // Clear interrupt status in DSPINTC
            IntEventClear(SYS_INT_T64P2_TINTALL);
        #else
            /* Clear the interrupt status in AINTC */
            IntSystemStatusClear(SYS_INT_TIMR2_ALL);
        #endif
            TimerIntStatusClear(SOC_TMR_2_REGS, TMR_INT_TMR12_NON_CAPT_MODE);

            // 确保接收到了字符
            if (rxFlag == 1)
            {
            	   // 根据数组中存储的数字决定LED状态
            		mode = rx[k];
            	   // mode不同时，LED点亮状态不同，如 mode = 0x5 = 0101，则LED3和LED5被点亮
            		switch (mode)
            		    			{
            		    			case 0x0:
            		    			        GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_LOW);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_LOW);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_LOW);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_LOW);
            		    					break;
            		    			case 0x1:
            		    			        GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_LOW);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_LOW);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_LOW);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_HIGH);
            		    					break;
            		    			case 0x2:
            		    			        GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_LOW);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_LOW);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 45,  GPIO_PIN_HIGH);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_LOW);
            		    					break;
            		    			case 0x3:
            		    			        GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_LOW);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_LOW);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_HIGH);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_HIGH);
            		    					break;
            		    			case 0x4:
            		    			        GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_LOW);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_HIGH);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_LOW);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_LOW);
            		    					break;
            		    			case 0x5:
            		    			        GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_LOW);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_HIGH);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_LOW);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_HIGH);
            		    					break;
            		    			case 0x6:
            		    			        GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_LOW);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_HIGH);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_HIGH);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_LOW);
            		    					break;
            		    			case 0x7:
            		    			        GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_LOW);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_HIGH);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_HIGH);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_HIGH);
            		    					break;
            		    			case 0x8:
            		    			        GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_HIGH);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_LOW);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_LOW);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_LOW);
            		    					break;
            		    			case 0x9:
            		    			        GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_HIGH);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_LOW);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_LOW);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_HIGH);
            		    					break;
            		    			case 0xa:
            		    			        GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_HIGH);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_LOW);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_HIGH);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_LOW);
            		    					break;
            		    			case 0xb:
            		    			        GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_HIGH);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_LOW);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_HIGH);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_HIGH);
            		    					break;
            		    			case 0xc:
            		    			        GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_HIGH);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_HIGH);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_LOW);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_LOW);
            		    					break;
            		    			case 0xd:
            		    			        GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_HIGH);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_HIGH);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_LOW);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_HIGH);
            		    					break;
            		    			case 0xe:
            		    			        GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_HIGH);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_HIGH);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_HIGH);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_LOW);
            		    					break;
            		    			case 0xf:
            		    			        GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_HIGH);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_HIGH);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_HIGH);
            		    					GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_HIGH);
            		    					break;
            		    			}
            	k = k + 1;
            	// 数组中的所有数字状态实现完毕
            	if (k == length)
            	{
            	    rxFlag = 0;
            	    mode = 0;
            	    TimerDisable(SOC_TMR_2_REGS, TMR_TIMER12);
            	}
        		// 再次开放中断
        		TimerIntEnable(SOC_TMR_2_REGS, TMR_INT_TMR12_NON_CAPT_MODE);
            	}
        }

/****************************************************************************/
/*                                                                          */
/*                          定时器初始化                                                                  */
/*                                                                          */
/****************************************************************************/
static void TimerSetUp64Bit(void)
{
    // 配置 定时器/计数器 2 为64位模式，使用内部时钟
    TimerConfigure(SOC_TMR_2_REGS, TMR_CFG_64BIT_CLK_INT);
    // 设置周期
    TimerPeriodSet(SOC_TMR_2_REGS, TMR_TIMER12, TMR_PERIOD_LSB32);
    TimerPeriodSet(SOC_TMR_2_REGS, TMR_TIMER34, TMR_PERIOD_MSB32);
    // 使能定时器
    TimerEnable(SOC_TMR_2_REGS, TMR_TIMER12, TMR_ENABLE_CONT);
}


/****************************************************************************/
/*                                                                          */
/*                          定时器中断                                                                      */
/*                                                                          */
/****************************************************************************/
static void TimerIntrSetUp(void)
{
#ifdef _TMS320C6X
    // 初始化DSP中断，必须在这个函数运行之后再去配置其他的中断映射。
    IntDSPINTCInit();
    // 使能全局中断
    IntGlobalEnable();

    // UART中断注册 ，中断映射号为5
    IntRegister(C674X_MASK_INT5, UARTIsr);
    IntEventMap(C674X_MASK_INT5, SYS_INT_UART2_INT);
    // 使能映射号为5的DSP中断
    IntEnable(C674X_MASK_INT5);

    // TIMER中断注册 ，中断映射号为4
    IntRegister(C674X_MASK_INT4, TimerIsr);
    IntEventMap(C674X_MASK_INT4, SYS_INT_T64P2_TINTALL);
    // 使能映射号为4的DSP中断
    IntEnable(C674X_MASK_INT4);
#endif
}
/****************************END OF FILE*************************************/
