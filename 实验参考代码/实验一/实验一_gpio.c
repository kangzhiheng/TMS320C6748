//-----------------------------------------------------------------------------
// 作    者：adoredee
// 创建时间：2019.04.28
// 描    述：实验一 GPIO驱动LED与定时器中断
// 版    本：
//-----------------------------------------------------------------------------
// Copyright (C) 2019 Shanghai Jiao Tong University
//-----------------------------------------------------------------------------

/*
    实验一：GPIO驱动LED与定时器中断
  	基本要求：使用GPIO控制四个LED灯，并按照下列顺序循环重复（各状态间的时间间隔大约为500ms）：
		 (1）LED0 亮（其他全灭）；
		（2）LED1 亮（其他全灭）；
		（3）LED2 亮（其他全灭）；
		（4）LED3 亮（其他全灭）；
		（5）LED2 亮（其他全灭）；
		（6）LED1 亮（其他全灭）；
		（7）LED0 亮（其他全灭）；
     拓展1：在基本要求的基础上，加入以下顺序（各状态间的时间间隔大约为500ms）：
		（8）LED0 LED1 亮（其他全灭）；
		（9）LED0 LED1 LED2 亮（其他全灭）；
		（10）全亮；
		（11）LED0 灭（其他全亮）；
		（12）LED1 灭（其他全亮）；
		（13）LED2 灭（其他全亮）；
		（14）LED3 灭（其他全亮）；
		（15）LED2 灭（其他全亮）；
		（16）LED1 灭（其他全亮）；
		（17）LED0 灭（其他全亮）；
		（18）全灭
    拓展2：在基本要求和拓展1的基础上，要求各状态间的时间间隔利用定时器中断实现500ms的延时。
*/

#include "gpio.h"        // 通用输入输出口宏及设备抽象层函数声明，具体的函数可在对应的.c文件中查阅
#include "psc.h"         // 电源与睡眠控制宏及设备抽象层函数声明，查阅psc.c查看函数说明

#include "soc_C6748.h"   // DSP C6748 外设寄存器
#include "lcdkC6748.h"   // 

/****************************************************************************/
/*              LOCAL FUNCTION PROTOTYPES                                   */
/****************************************************************************/
static void Delay(volatile unsigned int delay);    // 延时函数

/****************************************************************************/
/*              GLOBAL VARIABLES                                            */
/****************************************************************************/

/****************************************************************************/
/*             LOCAL FUNCTION DEFINITIONS                                   */
/****************************************************************************/
/*
    volatile关键字 —— 对变量的存取不能缓存到寄存器，每次使用时需要在内存中重新存取。
	编译器对访问该变量的代码就不再进行优化，即系统总是重新从它所在的内存读取数据，
	从而可以提供对特殊地址的稳定访问
*/
#define _HWREG(x) (*((volatile Uint32*)(x)))    
#define PINMUX0 _HWREG(0x01C14120)     // 多路寄存器0的地址 0X8000000
#define PINMUX5 _HWREG(0x01C14134)     // 多路寄存器5的地址 0X8000
#define PINMUX13 _HWREG(0x01C14154)    // 多路寄存器13的地址 0X8800
static unsigned char mode = 1;
typedef unsigned int Uint32;

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
/*                  外设使能配置                                             */
/*                                                                          */
/****************************************************************************/
    PSCModuleControl(SOC_PSC_1_REGS,     // PSC在内存中的基本地址，GPIO属于PSC1模块
	    			 HW_PSC_GPIO,        // GPIO的本地PSC模块号是3
					 PSC_POWERDOMAIN_ALWAYS_ON,    // 默认
					 PSC_MDCTL_NEXT_ENABLE);       // 默认
/****************************************************************************/
/*                                                                          */
/*                  GPIO 管脚复用配置                                        */
/*                                                                          */
/****************************************************************************/
    // GPIO管脚复用配置，通用输入输出口
	_GPIOBank6Pin12PinMuxSetup();
	_GPIOBank6Pin13PinMuxSetup();
	_GPIOBank2Pin12PinMuxSetup();
	_GPIOBank0Pin9PinMuxSetup();

/****************************************************************************/
/*                                                                          */
/*                   GPIO 管脚初始化                                         */
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

/****************************************************************************/
/*                                                                          */
/*              LED跑马灯                                                   */
/*                                                                          */
/****************************************************************************/
    
    /*
	开发板上有4个LED灯(D4、D5、D6和D7，认为是LED0、LED1、LED2、LED3)，每个LED、GPIO管脚及序号的对应关系如下：
	     LED        GPIO管脚   序号
		D4(LED0) —— GP6[13] —— 110
		D5(LED1) —— GP6[12] —— 109
		D6(LED2) —— GP2[12] —— 45
		D7(LED3) —— GP0[9] —— 10
	*/
    while(1)
    {
    	switch (mode)
    		{
			/*
	        GPIOPinWrite函数改变当前管脚的状态
	        */
    		/* 点亮LED0：设置LED0的GPIO引脚GP6[13]为高电平，其他为低电平 */
    		case 1: GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_HIGH);
					GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_LOW);
					GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_LOW);
					GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_LOW);
					break;
			/* 点亮LED1：设置LED1的GPIO引脚GP6[12]为高电平，其他为低电平 */
    		case 2: GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_LOW);
					GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_HIGH);
					GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_LOW);
					GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_LOW);
					break;
			/* 点亮LED2：设置LED2的GPIO引脚GP2[12]为高电平，其他为低电平 */
    		case 3: GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_LOW);
					GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_LOW);
					GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_HIGH);
					GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_LOW);
					break;
    	    /* 点亮LED3：设置LED3的GPIO引脚GP0[9]为高电平，其他为低电平 */
    		case 4: GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_LOW);
					GPIOPinWrite(SOC_GPIO_0_REGS,109, GPIO_PIN_LOW);
					GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_LOW);
					GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_HIGH);
					break;
            /* 点亮LED2：设置LED2的GPIO引脚GP2[12]为高电平，其他为低电平 */
    		case 5: GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_LOW);
					GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_LOW);
					GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_HIGH);
					GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_LOW);
					break;
    	    /* 点亮LED1：设置LED1的GPIO引脚GP6[12]为高电平，其他为低电平 */
    		case 6: GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_LOW);
    		        GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_HIGH);
                    GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_LOW);
					GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_LOW);
					break;
    	    /* 点亮LED0：设置LED0的GPIO引脚GP6[13]为高电平，其他为低电平 */
    		case 7: GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_HIGH);
					GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_LOW);
					GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_LOW);
					GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_LOW);
					break;
            /* 点亮LED0.1：设置LED0的GPIO引脚GP6[13]、LED1的GPIO引脚GP6[12]为高电平，其他为低电平 */
    		case 8: GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_HIGH);
					GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_HIGH);
					GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_LOW);
					GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_LOW);
					break;
    	    /* 点亮LED0.1.2：设置LED3的GPIO引脚GP0[9]为低电平，其他为高电平 */
    		case 9: GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_HIGH);
    		        GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_HIGH);
                    GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_HIGH);
					GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_LOW);
					break;
    	    /* 点亮LED0.1.2.3：设置GPIO引脚GP6[13]、GP6[12]、GP2[12]、GP0[9]全部为高电平 */
    		case 10: GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_HIGH);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_HIGH);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_HIGH);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_HIGH);
					 break;
            /* 熄灭LED0（其他全亮）：设置LED0的GPIO引脚GP6[13]为低电平，其他为高电平 */
    		case 11: GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_LOW);
					GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_HIGH);
					GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_HIGH);
					GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_HIGH);
					break;
    	    /* 熄灭LED1（其他全亮）：设置LED1的GPIO引脚GP6[12]为低电平，其他为高电平 */
    		case 12: GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_HIGH);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_LOW);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_HIGH);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_HIGH);
					 break;
			/* 熄灭LED2（其他全亮）：设置LED2的GPIO引脚GP2[12]为低电平，其他为高电平 */
    		case 13: GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_HIGH);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_HIGH);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_LOW);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_HIGH);
					 break;
            /* 熄灭LED3（其他全亮）：设置LED3的GPIO引脚GP0[9]为低电平，其他为高电平 */
    		case 14: GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_HIGH);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_HIGH);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_HIGH);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_LOW);
					 break;
    	    /* 熄灭LED2（其他全亮）：设置LED2的GPIO引脚GP2[12]为低电平，其他为高电平 */
    		case 15: GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_HIGH);
    		         GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_HIGH);
                     GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_LOW);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_HIGH);
					 break;
	       /* 熄灭LED1（其他全亮）：设置LED1的GPIO引脚GP6[12]为低电平，其他为高电平 */
    		case 16: GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_HIGH);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_LOW);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_HIGH);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_HIGH);
					 break;
           /* 熄灭LED0（其他全亮）：设置LED0的GPIO引脚GP6[13]为低电平，其他为高电平 */
    		case 17: GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_LOW);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_HIGH);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_HIGH);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_HIGH);
					 break;
    	    /* 全灭：设置GPIO引脚GP6[13]、GP6[12]、GP2[12]、GP0[9]全部为低电平 */
    		case 18: GPIOPinWrite(SOC_GPIO_0_REGS,110, GPIO_PIN_LOW);
    		         GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_LOW);
                     GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_LOW);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_LOW);
					 break;
    }

    /*计数变量进行更新，从1-18进行循环*/
    if (mode <= 18) 
	    mode=mode+1;
    else 
	    mode = 1;

	Delay(1000000);    // 延时

    }
}

// 延时函数
static void Delay(volatile unsigned int delay)
{
    while(delay--);
}
