//-----------------------------------------------------------------------------
// 作    者：adoredee
// 创建时间：2019.05.20
// 描    述：实验三  EPWM基础实验
// 版    本：
//-----------------------------------------------------------------------------
// Copyright (C) 2019 Shanghai Jiao Tong University
//-----------------------------------------------------------------------------

#include "uart.h"
#include "psc.h"
#include "soc_C6748.h"
#include "interrupt.h"
#include "hw_types.h"
#include "hw_syscfg0_C6748.h"
#include "lcdkC6748.h"
#include "ehrpwm.h"

/* Enable to demonstrate High resolution capability*/
/* #define HR_EN	1 */

/* Enable one of the macros in addition to BASIC_WF to get the corresponding 
 * waveform
 * */
#ifndef HR_EN
	#define BASIC_WF	1
	#define DB_GEN	1 
	/*#define CHP_EN	1 */
	/* #define TRIP_EN	1 */
	/* #define SYNC_EN	1 */  /* Note: Reset the target before enabling this */
#endif

#define CLOCK_DIV_VAL		10    // 十分频

/****************************************************************************/
/*                                                                          */
/*                            函数定义                                                                     */
/*                                                                          */
/****************************************************************************/
static void SetupIntc(void);    // 配置中断控制器以产生PWM中断
static void PWMEventIsr(void);  // PWM中断服务函数
static void PWMTZIsr(void);     // PWM错误控制器中断服务函数

// 默认配置
	/*This feature is useful to position the edges at very high resolution.
	 * The MEP (micro-edge position) is added to the raising edge of the PWM signal. 
	 * */
#ifdef HR_EN
	static void HighResWaveform(void);
#else

	/* Generate basic waveform on PWM1A. With the period set to 256 count, the PWM is 
	 * configured to toggle when count equals 63 and 191, resulting in a square waveform. 
	 * The PWM is configured to gentrate interrupt when count == CMPB.
	 * */
	static void GenPWM1A_Basic(void);

	/* In additon to the basic waveform, the software sync in is triggered when interupt 
	 * occurs (count == CMPB). On sync-in, the counter is reloaded to 250. Now the 
	 * counter will count from 250 and reset after 255. 
	 * */
	#ifdef SYNC_EN
		static void SyncWaveform(void);

	#elif DB_GEN
	/* In addition to the basic waveform, dead band is added. In this example dead band 
	 * is added to the Raising edge of PWM1A signal.
	 * */
	static void GenDeadBandWavefrom(void);

	#elif CHP_EN
	/* In addition to the basic waveform, the waveform is chopped using the chopper 
	 * sub-module. In this case the width of the first pulse, duty cycle of the remaining 
	 * pulses and frequency of the carrier signal can be configured.
	 * */
	static void GenChopperWaveform(void);

	#else /* TRIP_EN */
	/* In addition to the basic waveform, the trip feature is demonstrated. The trip signal 
	 * is software triggered cyclically after a delay. On each trip event the signal is toggled.
	 * */
	static void TripWaveform(void);
	#endif
#endif


int main(void)
{
	// 中断相关配置，这个函数没有修改
	SetupIntc();
	// EPWM配置，基地址选择，模式选择，powerDomain标志
	PSCModuleControl(SOC_PSC_1_REGS,    // PSC在内存中的基本地址，EHRPWM属于PSC1模块
	                 HW_PSC_EHRPWM,     // EHRPW的本地PSC模块号为3
	                 PSC_POWERDOMAIN_ALWAYS_ON,
	                 PSC_MDCTL_NEXT_ENABLE);
	// EPWMxA管脚复用配置
	EHRPWM1PinMuxSetup();    

	// EPWMxA设置
	#ifdef BASIC_WF
	    GenPWM1A_Basic();    // EPWMxA设置
	#endif

	// 启用同步
	#ifdef SYNC_EN
	    GenPWM1A_Basic();    // EPWMxA设置
	    SyncWaveform();      // 启用同步，同步后，计数器将使用新值。
	#endif

	// EPWMxA死区参数配置，具体配置见3.3.1
	#ifdef DB_GEN
	    GenPWM1A_Basic();         // EPWMxA设置
	    GenDeadBandWavefrom();    // 死区模块的设置
	#endif

	// EPWMxA斩波模式配置
	#ifdef CHP_EN
	    GenPWM1A_Basic();         // EPWMxA设置
	    GenChopperWaveform();     // 斩波器设置，允许高频载波信号调制由动作限定符和死带子模块产生的PWM波形。
	#endif
	// EPWMxA错误控制模块
	#ifdef TRIP_EN
	    GenPWM1A_Basic();         // EPWMxA设置
	    TripWaveform();           // 错误控制模块设置
	#endif
	// EPWMxA参数配置
	#ifdef HR_EN
	    HighResWaveform();
	#endif
	while(1)
    {
		volatile bool eventCount = EHRPWMETEventCount(SOC_EHRPWM_1_REGS);    // 返回发生的事件数
		eventCount = eventCount;
	}
}

#ifndef HR_EN  

/****************************************************************************/
/*                                                                          */
/*                              EPWMxA设置                                                             */
/*                                                                          */
/****************************************************************************/
static void GenPWM1A_Basic(void)
{
/****************************************************************************/
/*                                                                          */
/*                       PWM 周期和频率的设置                                                         */
/*                                                                          */
/****************************************************************************/
    // 配置时基模块的时钟分频器
    /*
     * soc_C6748.h里的589行附近 ——> SOC_EHRPWM_1_MODULE_FREQ = SOC_SYSCLK_2_FREQ
     * SOC_SYSCLK_2_FREQ = SOC_SYSCLK_1_FREQ/2，注意SOC_SYSCLK_1_FREQ的频率是300MHz还是456MHz
     */
    EHRPWMTimebaseClkConfig(SOC_EHRPWM_1_REGS,
                            SOC_EHRPWM_1_MODULE_FREQ/CLOCK_DIV_VAL ,
                            SOC_EHRPWM_1_MODULE_FREQ);
    // 配置PWM频率，设置频率为15k，向上计数
    // 若SOC_SYSCLK_1_FREQ = 300MHz，则SOC_SYSCLK_2_FREQ = 150MHz；
    EHRPWMPWMOpFreqSet(SOC_EHRPWM_1_REGS,
                       SOC_EHRPWM_1_MODULE_FREQ/CLOCK_DIV_VAL,         // 设置时基频率，输出为150MHz / 10 = 15MHz
                       (SOC_EHRPWM_1_MODULE_FREQ/CLOCK_DIV_VAL)/1000,  // 设置PWM输出，输出为150MhHz / 1000 = 15kHz
                       EHRPWM_COUNT_UP,    // 三种计数方法：up,down,uo_down
                       EHRPWM_SHADOW_WRITE_DISABLE);

    // 禁止同步
    EHRPWMTimebaseSyncDisable(SOC_EHRPWM_1_REGS);
    // 禁止同步输出模式设置
    EHRPWMSyncOutModeSet(SOC_EHRPWM_1_REGS, EHRPWM_SYNCOUT_DISABLE);

/****************************************************************************/
/*                                                                          */
/*                             计数模式的设置                                                         */
/*                                                                          */
/****************************************************************************/
    // (625 - 375) / (625 + 375) = 0.25
    // 加载比较器A的值 = 375
    EHRPWMLoadCMPA(SOC_EHRPWM_1_REGS,
                   375,
                   EHRPWM_SHADOW_WRITE_DISABLE,    // 是否写入shadow register.
                   EHRPWM_COMPA_NO_LOAD,
                   EHRPWM_CMPCTL_OVERWR_SH_FL);
    // 加载比较器B值 =625 使得设置占空比25%
    EHRPWMLoadCMPB(SOC_EHRPWM_1_REGS,
                   625,
                   EHRPWM_SHADOW_WRITE_DISABLE,
		           EHRPWM_COMPB_NO_LOAD,
		           EHRPWM_CMPCTL_OVERWR_SH_FL);
/****************************************************************************/
/*                                                                          */
/*   配置Action限定符模块在接收事件时要对A执行的操作。 这将决定输出波形。            */
/*                                                                          */
/****************************************************************************/
    // 当计数器从0到375拉低，375到625拉高，625到后拉低
    EHRPWMConfigureAQActionOnA(SOC_EHRPWM_1_REGS,
                               EHRPWM_AQCTLA_ZRO_EPWMXALOW,
                               EHRPWM_AQCTLA_PRD_DONOTHING,
    		                   EHRPWM_AQCTLA_CAU_EPWMXAHIGH,
                               EHRPWM_AQCTLA_CAD_DONOTHING,
                               EHRPWM_AQCTLA_CBU_EPWMXALOW,
                               EHRPWM_AQCTLA_CBD_DONOTHING,
                               EHRPWM_AQSFRC_ACTSFA_DONOTHING);
    // 双路死区子模块设置
    EHRPWMDBOutput(SOC_EHRPWM_1_REGS, EHRPWM_DBCTL_OUT_MODE_BYPASS);
    // 禁止斩波子模块
    EHRPWMChopperDisable(SOC_EHRPWM_1_REGS);
    // 禁止trip事件
    EHRPWMTZTripEventDisable(SOC_EHRPWM_1_REGS, EHRPWM_TZ_ONESHOT);
    EHRPWMTZTripEventDisable(SOC_EHRPWM_1_REGS, EHRPWM_TZ_CYCLEBYCYCLE);
    // 事件触发
    // 每3次事件发生后产生中断
    EHRPWMETIntPrescale(SOC_EHRPWM_1_REGS, EHRPWM_ETPS_INTPRD_SECONDEVENT);
    // 当计数器计到625时产生事件
    EHRPWMETIntSourceSelect(SOC_EHRPWM_1_REGS, EHRPWM_ETSEL_INTSEL_TBCTREQUCMPBINC);
    EHRPWMETIntSourceSelect(SOC_EHRPWM_1_REGS, EHRPWM_ETSEL_INTSEL_TBCTREQUPRD);
    // 使能中断
    EHRPWMETIntEnable(SOC_EHRPWM_1_REGS);
    // 禁止高分辨率的能力
    EHRPWMHRDisable(SOC_EHRPWM_1_REGS);
}
#endif /* BASIC_WF */


#ifdef SYNC_EN

static void SyncWaveform(void)
{
    /* Reload CTR = 250 and count up after sync */
    // 启用同步， 生成同步事件时，将使用新值重新加载couter。 同步后，计数器将使用新值。
    EHRPWMTimebaseSyncEnable(SOC_EHRPWM_1_REGS,
                             250 ,
                             EHRPWM_COUNT_UP_AFTER_SYNC);   // 向上计数
}
#endif /* SYNC_EN */

/****************************************************************************/
/*                                                                          */
/*                            死区模块的设置                                                           */
/*                                                                          */
/****************************************************************************/
#ifdef DB_GEN
static void GenDeadBandWavefrom(void)
{
    /*
     * 选择死区子模块中延迟块的源。
     * 死区发生器有两个子模块，一个用于上升沿延迟，另一个用于下降沿延迟。
     * 当信号改变期间两个信号之间需要延迟时，可以配置此功能。
     */
    // RED ——> Raising Edge Delay，上升沿延迟
    // FED ——> Falling Edge Delay，下降沿延迟
	EHRPWMDBSourceSelect(SOC_EHRPWM_1_REGS,
	                     EHRPWM_DBCTL_IN_MODE_AREDAFED);     // A的上升沿、下降沿配置为死区参考源
	/*
	 * 选择极性，这允许在将延迟信号发送出死区子模块之前选择性地反转其中一个延迟信号。
	 */
	EHRPWMDBPolaritySelect(SOC_EHRPWM_1_REGS,
	                       EHRPWM_DBCTL_POLSEL_ALC);     // 设置为 Active Low Complementary 极性输出，互补模式
	/*
	 * 选择输出模式，允许选择性地启用或绕过用于下降沿和上升沿延迟的死区生成。
	 */
	EHRPWMDBOutput(SOC_EHRPWM_1_REGS,
	               EHRPWM_DBCTL_OUT_MODE_AREDBFED);
    // 上升沿下降沿均有输出，0xF的配置使得死区时间为1us,15 * 1 / 15000000 = 1us
	EHRPWMDBConfigureRED(SOC_EHRPWM_1_REGS, 0xF);
	EHRPWMDBConfigureFED(SOC_EHRPWM_1_REGS, 0xF);
}
#endif /* DB_GEN */

/****************************************************************************/
/*                                                                          */
/*                              斩波器设置                                                              */
/*                                                                          */
/****************************************************************************/
#ifdef CHP_EN
static void GenChopperWaveform(void)
{
    // 在斩波器模块中，PWM信号用载波信号调制，配置斩波器的占空比
    EHRPWMConfigureChopperDuty(SOC_EHRPWM_1_REGS,
                               EHRPWM_CHP_DUTY_50_PER);   // 斩波器50%占空比
    // 在斩波器子模块中，PWM信号用载波信号调制，使用此API配置载波信号的频率。
    EHRPWMConfigureChopperFreq(SOC_EHRPWM_1_REGS,
                               EHRPWM_PCCTL_CHPFREQ_DIVBY4);    // 4分频
    /*
     * 配置一次脉冲宽度。 斩波器模块可用于脉冲变压器的开关操作。
     * 单触发块提供高能量第一脉冲，以确保硬启动和快速电源开关打开，
     * 而后续脉冲维持脉冲，确保电源开关保持打开状态。
     */
    EHRPWMConfigureChopperOSPW(SOC_EHRPWM_1_REGS,
                               0xF);    // OSPW接通的时钟数。
    // 使能斩波器
    EHRPWMChopperEnable(SOC_EHRPWM_1_REGS);
}
#endif /* CHP_EN */

/****************************************************************************/
/*                                                                          */
/*                         错误控制模块设置                                                             */
/*                                                                          */
/****************************************************************************/
#ifdef TRIP_EN
static void Delay(void)
{
   volatile unsigned int i_delay = 0;
   for(i_delay = 0; i_delay < 0x0000000F; i_delay++);
}

static void TripWaveform(void)
{

    unsigned int i_toggle = 0;    // 事件触发标志

    // 启用错误控制模块，控制信号指示外部故障，并且可以对ePWM输出进行编程，以在发生故障时作出相应的响应。
    EHRPWMTZTripEventEnable(SOC_EHRPWM_1_REGS,
                            EHRPWM_TZ_ONESHOT);    // 使能 OST或者CBC事件
                                                   // OST ——> a one-shot trip event occurs;
                                                   // CBC ——> a cycle-by-cycle trip event occurs
    // 事件发生后，会按照事先配置好的动作对 EPWM 的输出做出改变：强制拉低，强制拉高或高阻。
    EHRPWMTZForceAOnTrip(SOC_EHRPWM_1_REGS,
                         EHRPWM_TZCTL_TZA_FORCEHIGH);   // 触发事件后，强制拉高
    // 当事件发生时，子模块可以配置为中断CPU
    EHRPWMTZIntEnable(SOC_EHRPWM_1_REGS, EHRPWM_TZ_ONESHOT);

    while(1)
    {
        // 事件被触发
	    if(i_toggle)
	    {
		     EHRPWMTZForceAOnTrip(SOC_EHRPWM_1_REGS,
		                          EHRPWM_TZCTL_TZA_FORCEHIGH);    // 触发事件后，强制拉高
		     i_toggle = 0;
	    }
        else
        {
             EHRPWMTZForceAOnTrip(SOC_EHRPWM_1_REGS, EHRPWM_TZCTL_TZA_FORCELOW);
             i_toggle = 1;
        }
	
	    Delay();

	    EHRPWMTZSWFrcEvent(SOC_EHRPWM_1_REGS, EHRPWM_TZ_ONESHOT);
    }
}
#endif /* TRIP_EN */


/****************************************************************************/
/*                                                                          */
/*                          EPWMxA参数配置                                                              */
/*                                                                          */
/****************************************************************************/
#ifdef HR_EN
static void HighResWaveform(void)
{
    // 配置时基模块的时钟分频器
    EHRPWMTimebaseClkConfig(SOC_EHRPWM_1_REGS,
                            SOC_EHRPWM_1_MODULE_FREQ/2 ,
                            SOC_EHRPWM_1_MODULE_FREQ);
    // 配置PWM频率
    EHRPWMPWMOpFreqSet(SOC_EHRPWM_1_REGS,
                       SOC_EHRPWM_1_MODULE_FREQ/2,
		               (SOC_EHRPWM_1_MODULE_FREQ/2)/5,
		               EHRPWM_COUNT_UP,
		               EHRPWM_SHADOW_WRITE_DISABLE);

    // 禁止同步
    EHRPWMTimebaseSyncDisable(SOC_EHRPWM_1_REGS);

    // 禁止同步输出模式设置
    EHRPWMSyncOutModeSet(SOC_EHRPWM_1_REGS, EHRPWM_SYNCOUT_MASK);

    // 加载比较器A的值
    EHRPWMLoadCMPA(SOC_EHRPWM_1_REGS,
                   4,
                   EHRPWM_SHADOW_WRITE_DISABLE,
                   EHRPWM_CMPCTL_LOADAMODE_TBCTRZERO,
                   EHRPWM_CMPCTL_OVERWR_SH_FL);

    // 加载比较器B的值
    EHRPWMLoadCMPB(SOC_EHRPWM_1_REGS,
                   0,
                   EHRPWM_SHADOW_WRITE_DISABLE,
                   EHRPWM_CMPCTL_LOADAMODE_TBCTRZERO,
                   EHRPWM_CMPCTL_OVERWR_SH_FL);

    /****************************************************************************/
    /*                                                                          */
    /*   配置Action限定符模块在接收事件时要对A执行的操作。 这将决定输出波形。            */
    /*                                                                          */
    /****************************************************************************/
    EHRPWMConfigureAQActionOnA(SOC_EHRPWM_1_REGS,
                               EHRPWM_AQCTLA_ZRO_DONOTHING,
                               EHRPWM_AQCTLA_PRD_EPWMXALOW,
                               EHRPWM_AQCTLA_CAU_EPWMXAHIGH,
                               EHRPWM_AQCTLA_CAD_DONOTHING,
                               EHRPWM_AQCTLA_CBU_DONOTHING,
                               EHRPWM_AQCTLA_CBD_DONOTHING,
                               EHRPWM_AQSFRC_ACTSFA_DONOTHING);

    // 双路死区子模块设置
    EHRPWMDBOutput(SOC_EHRPWM_1_REGS, EHRPWM_DBCTL_OUT_MODE_BYPASS);
    // 禁止斩波子模块
    EHRPWMChopperDisable(SOC_EHRPWM_1_REGS);
    // 禁止trip事件
    EHRPWMTZTripEventDisable(SOC_EHRPWM_1_REGS, EHRPWM_TZ_ONESHOT);
    EHRPWMTZTripEventDisable(SOC_EHRPWM_1_REGS, EHRPWM_TZ_CYCLEBYCYCLE);
    // 禁止中断
    EHRPWMETIntDisable(SOC_EHRPWM_1_REGS);

    // 设置高分辨率，当计数为0时，加50个微边缘定位器
    // MEP ——> micro edge positioner
    EHRPWMLoadCMPAHR(SOC_EHRPWM_1_REGS,
                     50,
                     EHRPWM_HR_HRLOAD_CTR_ZERO);
    // 配置控制模式和边缘模式，
    EHRPWMConfigHR(SOC_EHRPWM_1_REGS,
                   EHRPWM_HR_CTLMODE_CMPAHR,
                   EHRPWM_HR_EDGEMODE_RAISING);    // 将MEP添加到PWM信号的上升沿

    /* EHRPWMHRDisable(SOC_EHRPWM_1_REGS); */

}
#endif /* HR_EN */

/****************************************************************************/
/*                                                                          */
/*                      配置中断控制器以产生PWM中断                                               */
/*                                                                          */
/****************************************************************************/
static void SetupIntc(void)
{
#ifdef _TMS320C6X
	// 初始化DSP中断控制器
	IntDSPINTCInit();

	// 注册中断向量，PWM中断服务函数号为4，错误控制器中断号为5
	IntRegister(C674X_MASK_INT4, PWMEventIsr);
	IntRegister(C674X_MASK_INT5, PWMTZIsr);

	// 将系统事件映射到DSP可屏蔽中断
	IntEventMap(C674X_MASK_INT4, SYS_INT_EHRPWM1);
	IntEventMap(C674X_MASK_INT5, SYS_INT_EHRPWM1TZ);

	// 使能DSP可屏蔽中断
	IntEnable(C674X_MASK_INT4);
	IntEnable(C674X_MASK_INT5);

	// 使能全局DSP中断
	IntGlobalEnable();
#else
    /* Initialize the ARM Interrupt Controller.*/
    IntAINTCInit();

    IntSystemStatusClear(SYS_INT_EHRPWM1);
    EHRPWMETIntClear(SOC_EHRPWM_1_REGS);
    
    /************************PWM1****************************************/
    IntRegister(SYS_INT_EHRPWM1, PWMEventIsr);
    IntChannelSet(SYS_INT_EHRPWM1, 2);
    IntSystemEnable(SYS_INT_EHRPWM1);
    /********************************************************************/
    IntRegister(SYS_INT_EHRPWM1TZ, PWMTZIsr);
    IntChannelSet(SYS_INT_EHRPWM1TZ, 2);
    IntSystemEnable(SYS_INT_EHRPWM1TZ);
    /********************************************************************/

    /* Enable IRQ in CPSR.*/
    IntMasterIRQEnable();

    /* Enable the interrupts in GER of AINTC.*/
    IntGlobalEnable();

    /* Enable the interrupts in HIER of AINTC.*/
    IntIRQEnable();
#endif
}

/****************************************************************************/
/*                                                                          */
/*                            PWM中断服务函数                                                         */
/*                                                                          */
/****************************************************************************/
static void PWMEventIsr(void)
{
#ifdef _TMS320C6X
	IntEventClear(SYS_INT_EHRPWM1);     // 清除系统中断
#else
    IntSystemStatusClear(SYS_INT_EHRPWM1);
#endif

    EHRPWMETIntClear(SOC_EHRPWM_1_REGS);   // 清楚ePWMxA中断

#ifdef SYNC_EN
    EHRPWMTriggerSWSync(SOC_EHRPWM_1_REGS);    // 生成脉冲
#endif

}

/****************************************************************************/
/*                                                                          */
/*                        PWM错误控制器中断服务函数                                               */
/*                                                                          */
/****************************************************************************/
static void PWMTZIsr(void)
{
#ifdef _TMS320C6X
	IntEventClear(SYS_INT_EHRPWM1TZ);    // 清除系统中断
#else
    IntSystemStatusClear(SYS_INT_EHRPWM1TZ);
#endif

    EHRPWMTZFlagClear(SOC_EHRPWM_1_REGS, EHRPWM_TZ_CYCLEBYCYCLE_CLEAR);
}



















