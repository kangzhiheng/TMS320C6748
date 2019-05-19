//-----------------------------------------------------------------------------
// ��    �ߣ�adoredee
// ����ʱ�䣺2019.05.20
// ��    ����ʵ����  EPWM����ʵ��
// ��    ����
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

#define CLOCK_DIV_VAL		10    // ʮ��Ƶ

/****************************************************************************/
/*                                                                          */
/*                            ��������                                                                     */
/*                                                                          */
/****************************************************************************/
static void SetupIntc(void);    // �����жϿ������Բ���PWM�ж�
static void PWMEventIsr(void);  // PWM�жϷ�����
static void PWMTZIsr(void);     // PWM����������жϷ�����

// Ĭ������
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
	// �ж�������ã��������û���޸�
	SetupIntc();
	// EPWM���ã�����ַѡ��ģʽѡ��powerDomain��־
	PSCModuleControl(SOC_PSC_1_REGS,    // PSC���ڴ��еĻ�����ַ��EHRPWM����PSC1ģ��
	                 HW_PSC_EHRPWM,     // EHRPW�ı���PSCģ���Ϊ3
	                 PSC_POWERDOMAIN_ALWAYS_ON,
	                 PSC_MDCTL_NEXT_ENABLE);
	// EPWMxA�ܽŸ�������
	EHRPWM1PinMuxSetup();    

	// EPWMxA����
	#ifdef BASIC_WF
	    GenPWM1A_Basic();    // EPWMxA����
	#endif

	// ����ͬ��
	#ifdef SYNC_EN
	    GenPWM1A_Basic();    // EPWMxA����
	    SyncWaveform();      // ����ͬ����ͬ���󣬼�������ʹ����ֵ��
	#endif

	// EPWMxA�����������ã��������ü�3.3.1
	#ifdef DB_GEN
	    GenPWM1A_Basic();         // EPWMxA����
	    GenDeadBandWavefrom();    // ����ģ�������
	#endif

	// EPWMxAն��ģʽ����
	#ifdef CHP_EN
	    GenPWM1A_Basic();         // EPWMxA����
	    GenChopperWaveform();     // ն�������ã������Ƶ�ز��źŵ����ɶ����޶�����������ģ�������PWM���Ρ�
	#endif
	// EPWMxA�������ģ��
	#ifdef TRIP_EN
	    GenPWM1A_Basic();         // EPWMxA����
	    TripWaveform();           // �������ģ������
	#endif
	// EPWMxA��������
	#ifdef HR_EN
	    HighResWaveform();
	#endif
	while(1)
    {
		volatile bool eventCount = EHRPWMETEventCount(SOC_EHRPWM_1_REGS);    // ���ط������¼���
		eventCount = eventCount;
	}
}

#ifndef HR_EN  

/****************************************************************************/
/*                                                                          */
/*                              EPWMxA����                                                             */
/*                                                                          */
/****************************************************************************/
static void GenPWM1A_Basic(void)
{
/****************************************************************************/
/*                                                                          */
/*                       PWM ���ں�Ƶ�ʵ�����                                                         */
/*                                                                          */
/****************************************************************************/
    // ����ʱ��ģ���ʱ�ӷ�Ƶ��
    /*
     * soc_C6748.h���589�и��� ����> SOC_EHRPWM_1_MODULE_FREQ = SOC_SYSCLK_2_FREQ
     * SOC_SYSCLK_2_FREQ = SOC_SYSCLK_1_FREQ/2��ע��SOC_SYSCLK_1_FREQ��Ƶ����300MHz����456MHz
     */
    EHRPWMTimebaseClkConfig(SOC_EHRPWM_1_REGS,
                            SOC_EHRPWM_1_MODULE_FREQ/CLOCK_DIV_VAL ,
                            SOC_EHRPWM_1_MODULE_FREQ);
    // ����PWMƵ�ʣ�����Ƶ��Ϊ15k�����ϼ���
    // ��SOC_SYSCLK_1_FREQ = 300MHz����SOC_SYSCLK_2_FREQ = 150MHz��
    EHRPWMPWMOpFreqSet(SOC_EHRPWM_1_REGS,
                       SOC_EHRPWM_1_MODULE_FREQ/CLOCK_DIV_VAL,         // ����ʱ��Ƶ�ʣ����Ϊ150MHz / 10 = 15MHz
                       (SOC_EHRPWM_1_MODULE_FREQ/CLOCK_DIV_VAL)/1000,  // ����PWM��������Ϊ150MhHz / 1000 = 15kHz
                       EHRPWM_COUNT_UP,    // ���ּ���������up,down,uo_down
                       EHRPWM_SHADOW_WRITE_DISABLE);

    // ��ֹͬ��
    EHRPWMTimebaseSyncDisable(SOC_EHRPWM_1_REGS);
    // ��ֹͬ�����ģʽ����
    EHRPWMSyncOutModeSet(SOC_EHRPWM_1_REGS, EHRPWM_SYNCOUT_DISABLE);

/****************************************************************************/
/*                                                                          */
/*                             ����ģʽ������                                                         */
/*                                                                          */
/****************************************************************************/
    // (625 - 375) / (625 + 375) = 0.25
    // ���رȽ���A��ֵ = 375
    EHRPWMLoadCMPA(SOC_EHRPWM_1_REGS,
                   375,
                   EHRPWM_SHADOW_WRITE_DISABLE,    // �Ƿ�д��shadow register.
                   EHRPWM_COMPA_NO_LOAD,
                   EHRPWM_CMPCTL_OVERWR_SH_FL);
    // ���رȽ���Bֵ =625 ʹ������ռ�ձ�25%
    EHRPWMLoadCMPB(SOC_EHRPWM_1_REGS,
                   625,
                   EHRPWM_SHADOW_WRITE_DISABLE,
		           EHRPWM_COMPB_NO_LOAD,
		           EHRPWM_CMPCTL_OVERWR_SH_FL);
/****************************************************************************/
/*                                                                          */
/*   ����Action�޶���ģ���ڽ����¼�ʱҪ��Aִ�еĲ����� �⽫����������Ρ�            */
/*                                                                          */
/****************************************************************************/
    // ����������0��375���ͣ�375��625���ߣ�625��������
    EHRPWMConfigureAQActionOnA(SOC_EHRPWM_1_REGS,
                               EHRPWM_AQCTLA_ZRO_EPWMXALOW,
                               EHRPWM_AQCTLA_PRD_DONOTHING,
    		                   EHRPWM_AQCTLA_CAU_EPWMXAHIGH,
                               EHRPWM_AQCTLA_CAD_DONOTHING,
                               EHRPWM_AQCTLA_CBU_EPWMXALOW,
                               EHRPWM_AQCTLA_CBD_DONOTHING,
                               EHRPWM_AQSFRC_ACTSFA_DONOTHING);
    // ˫·������ģ������
    EHRPWMDBOutput(SOC_EHRPWM_1_REGS, EHRPWM_DBCTL_OUT_MODE_BYPASS);
    // ��ֹն����ģ��
    EHRPWMChopperDisable(SOC_EHRPWM_1_REGS);
    // ��ֹtrip�¼�
    EHRPWMTZTripEventDisable(SOC_EHRPWM_1_REGS, EHRPWM_TZ_ONESHOT);
    EHRPWMTZTripEventDisable(SOC_EHRPWM_1_REGS, EHRPWM_TZ_CYCLEBYCYCLE);
    // �¼�����
    // ÿ3���¼�����������ж�
    EHRPWMETIntPrescale(SOC_EHRPWM_1_REGS, EHRPWM_ETPS_INTPRD_SECONDEVENT);
    // ���������Ƶ�625ʱ�����¼�
    EHRPWMETIntSourceSelect(SOC_EHRPWM_1_REGS, EHRPWM_ETSEL_INTSEL_TBCTREQUCMPBINC);
    EHRPWMETIntSourceSelect(SOC_EHRPWM_1_REGS, EHRPWM_ETSEL_INTSEL_TBCTREQUPRD);
    // ʹ���ж�
    EHRPWMETIntEnable(SOC_EHRPWM_1_REGS);
    // ��ֹ�߷ֱ��ʵ�����
    EHRPWMHRDisable(SOC_EHRPWM_1_REGS);
}
#endif /* BASIC_WF */


#ifdef SYNC_EN

static void SyncWaveform(void)
{
    /* Reload CTR = 250 and count up after sync */
    // ����ͬ���� ����ͬ���¼�ʱ����ʹ����ֵ���¼���couter�� ͬ���󣬼�������ʹ����ֵ��
    EHRPWMTimebaseSyncEnable(SOC_EHRPWM_1_REGS,
                             250 ,
                             EHRPWM_COUNT_UP_AFTER_SYNC);   // ���ϼ���
}
#endif /* SYNC_EN */

/****************************************************************************/
/*                                                                          */
/*                            ����ģ�������                                                           */
/*                                                                          */
/****************************************************************************/
#ifdef DB_GEN
static void GenDeadBandWavefrom(void)
{
    /*
     * ѡ��������ģ�����ӳٿ��Դ��
     * ������������������ģ�飬һ�������������ӳ٣���һ�������½����ӳ١�
     * ���źŸı��ڼ������ź�֮����Ҫ�ӳ�ʱ���������ô˹��ܡ�
     */
    // RED ����> Raising Edge Delay���������ӳ�
    // FED ����> Falling Edge Delay���½����ӳ�
	EHRPWMDBSourceSelect(SOC_EHRPWM_1_REGS,
	                     EHRPWM_DBCTL_IN_MODE_AREDAFED);     // A�������ء��½�������Ϊ�����ο�Դ
	/*
	 * ѡ���ԣ��������ڽ��ӳ��źŷ��ͳ�������ģ��֮ǰѡ���Եط�ת����һ���ӳ��źš�
	 */
	EHRPWMDBPolaritySelect(SOC_EHRPWM_1_REGS,
	                       EHRPWM_DBCTL_POLSEL_ALC);     // ����Ϊ Active Low Complementary �������������ģʽ
	/*
	 * ѡ�����ģʽ������ѡ���Ե����û��ƹ������½��غ��������ӳٵ��������ɡ�
	 */
	EHRPWMDBOutput(SOC_EHRPWM_1_REGS,
	               EHRPWM_DBCTL_OUT_MODE_AREDBFED);
    // �������½��ؾ��������0xF������ʹ������ʱ��Ϊ1us,15 * 1 / 15000000 = 1us
	EHRPWMDBConfigureRED(SOC_EHRPWM_1_REGS, 0xF);
	EHRPWMDBConfigureFED(SOC_EHRPWM_1_REGS, 0xF);
}
#endif /* DB_GEN */

/****************************************************************************/
/*                                                                          */
/*                              ն��������                                                              */
/*                                                                          */
/****************************************************************************/
#ifdef CHP_EN
static void GenChopperWaveform(void)
{
    // ��ն����ģ���У�PWM�ź����ز��źŵ��ƣ�����ն������ռ�ձ�
    EHRPWMConfigureChopperDuty(SOC_EHRPWM_1_REGS,
                               EHRPWM_CHP_DUTY_50_PER);   // ն����50%ռ�ձ�
    // ��ն������ģ���У�PWM�ź����ز��źŵ��ƣ�ʹ�ô�API�����ز��źŵ�Ƶ�ʡ�
    EHRPWMConfigureChopperFreq(SOC_EHRPWM_1_REGS,
                               EHRPWM_PCCTL_CHPFREQ_DIVBY4);    // 4��Ƶ
    /*
     * ����һ�������ȡ� ն����ģ������������ѹ���Ŀ��ز�����
     * ���������ṩ��������һ���壬��ȷ��Ӳ�����Ϳ��ٵ�Դ���ش򿪣�
     * ����������ά�����壬ȷ����Դ���ر��ִ�״̬��
     */
    EHRPWMConfigureChopperOSPW(SOC_EHRPWM_1_REGS,
                               0xF);    // OSPW��ͨ��ʱ������
    // ʹ��ն����
    EHRPWMChopperEnable(SOC_EHRPWM_1_REGS);
}
#endif /* CHP_EN */

/****************************************************************************/
/*                                                                          */
/*                         �������ģ������                                                             */
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

    unsigned int i_toggle = 0;    // �¼�������־

    // ���ô������ģ�飬�����ź�ָʾ�ⲿ���ϣ����ҿ��Զ�ePWM������б�̣����ڷ�������ʱ������Ӧ����Ӧ��
    EHRPWMTZTripEventEnable(SOC_EHRPWM_1_REGS,
                            EHRPWM_TZ_ONESHOT);    // ʹ�� OST����CBC�¼�
                                                   // OST ����> a one-shot trip event occurs;
                                                   // CBC ����> a cycle-by-cycle trip event occurs
    // �¼������󣬻ᰴ���������úõĶ����� EPWM ����������ı䣺ǿ�����ͣ�ǿ�����߻���衣
    EHRPWMTZForceAOnTrip(SOC_EHRPWM_1_REGS,
                         EHRPWM_TZCTL_TZA_FORCEHIGH);   // �����¼���ǿ������
    // ���¼�����ʱ����ģ���������Ϊ�ж�CPU
    EHRPWMTZIntEnable(SOC_EHRPWM_1_REGS, EHRPWM_TZ_ONESHOT);

    while(1)
    {
        // �¼�������
	    if(i_toggle)
	    {
		     EHRPWMTZForceAOnTrip(SOC_EHRPWM_1_REGS,
		                          EHRPWM_TZCTL_TZA_FORCEHIGH);    // �����¼���ǿ������
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
/*                          EPWMxA��������                                                              */
/*                                                                          */
/****************************************************************************/
#ifdef HR_EN
static void HighResWaveform(void)
{
    // ����ʱ��ģ���ʱ�ӷ�Ƶ��
    EHRPWMTimebaseClkConfig(SOC_EHRPWM_1_REGS,
                            SOC_EHRPWM_1_MODULE_FREQ/2 ,
                            SOC_EHRPWM_1_MODULE_FREQ);
    // ����PWMƵ��
    EHRPWMPWMOpFreqSet(SOC_EHRPWM_1_REGS,
                       SOC_EHRPWM_1_MODULE_FREQ/2,
		               (SOC_EHRPWM_1_MODULE_FREQ/2)/5,
		               EHRPWM_COUNT_UP,
		               EHRPWM_SHADOW_WRITE_DISABLE);

    // ��ֹͬ��
    EHRPWMTimebaseSyncDisable(SOC_EHRPWM_1_REGS);

    // ��ֹͬ�����ģʽ����
    EHRPWMSyncOutModeSet(SOC_EHRPWM_1_REGS, EHRPWM_SYNCOUT_MASK);

    // ���رȽ���A��ֵ
    EHRPWMLoadCMPA(SOC_EHRPWM_1_REGS,
                   4,
                   EHRPWM_SHADOW_WRITE_DISABLE,
                   EHRPWM_CMPCTL_LOADAMODE_TBCTRZERO,
                   EHRPWM_CMPCTL_OVERWR_SH_FL);

    // ���رȽ���B��ֵ
    EHRPWMLoadCMPB(SOC_EHRPWM_1_REGS,
                   0,
                   EHRPWM_SHADOW_WRITE_DISABLE,
                   EHRPWM_CMPCTL_LOADAMODE_TBCTRZERO,
                   EHRPWM_CMPCTL_OVERWR_SH_FL);

    /****************************************************************************/
    /*                                                                          */
    /*   ����Action�޶���ģ���ڽ����¼�ʱҪ��Aִ�еĲ����� �⽫����������Ρ�            */
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

    // ˫·������ģ������
    EHRPWMDBOutput(SOC_EHRPWM_1_REGS, EHRPWM_DBCTL_OUT_MODE_BYPASS);
    // ��ֹն����ģ��
    EHRPWMChopperDisable(SOC_EHRPWM_1_REGS);
    // ��ֹtrip�¼�
    EHRPWMTZTripEventDisable(SOC_EHRPWM_1_REGS, EHRPWM_TZ_ONESHOT);
    EHRPWMTZTripEventDisable(SOC_EHRPWM_1_REGS, EHRPWM_TZ_CYCLEBYCYCLE);
    // ��ֹ�ж�
    EHRPWMETIntDisable(SOC_EHRPWM_1_REGS);

    // ���ø߷ֱ��ʣ�������Ϊ0ʱ����50��΢��Ե��λ��
    // MEP ����> micro edge positioner
    EHRPWMLoadCMPAHR(SOC_EHRPWM_1_REGS,
                     50,
                     EHRPWM_HR_HRLOAD_CTR_ZERO);
    // ���ÿ���ģʽ�ͱ�Եģʽ��
    EHRPWMConfigHR(SOC_EHRPWM_1_REGS,
                   EHRPWM_HR_CTLMODE_CMPAHR,
                   EHRPWM_HR_EDGEMODE_RAISING);    // ��MEP��ӵ�PWM�źŵ�������

    /* EHRPWMHRDisable(SOC_EHRPWM_1_REGS); */

}
#endif /* HR_EN */

/****************************************************************************/
/*                                                                          */
/*                      �����жϿ������Բ���PWM�ж�                                               */
/*                                                                          */
/****************************************************************************/
static void SetupIntc(void)
{
#ifdef _TMS320C6X
	// ��ʼ��DSP�жϿ�����
	IntDSPINTCInit();

	// ע���ж�������PWM�жϷ�������Ϊ4������������жϺ�Ϊ5
	IntRegister(C674X_MASK_INT4, PWMEventIsr);
	IntRegister(C674X_MASK_INT5, PWMTZIsr);

	// ��ϵͳ�¼�ӳ�䵽DSP�������ж�
	IntEventMap(C674X_MASK_INT4, SYS_INT_EHRPWM1);
	IntEventMap(C674X_MASK_INT5, SYS_INT_EHRPWM1TZ);

	// ʹ��DSP�������ж�
	IntEnable(C674X_MASK_INT4);
	IntEnable(C674X_MASK_INT5);

	// ʹ��ȫ��DSP�ж�
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
/*                            PWM�жϷ�����                                                         */
/*                                                                          */
/****************************************************************************/
static void PWMEventIsr(void)
{
#ifdef _TMS320C6X
	IntEventClear(SYS_INT_EHRPWM1);     // ���ϵͳ�ж�
#else
    IntSystemStatusClear(SYS_INT_EHRPWM1);
#endif

    EHRPWMETIntClear(SOC_EHRPWM_1_REGS);   // ���ePWMxA�ж�

#ifdef SYNC_EN
    EHRPWMTriggerSWSync(SOC_EHRPWM_1_REGS);    // ��������
#endif

}

/****************************************************************************/
/*                                                                          */
/*                        PWM����������жϷ�����                                               */
/*                                                                          */
/****************************************************************************/
static void PWMTZIsr(void)
{
#ifdef _TMS320C6X
	IntEventClear(SYS_INT_EHRPWM1TZ);    // ���ϵͳ�ж�
#else
    IntSystemStatusClear(SYS_INT_EHRPWM1TZ);
#endif

    EHRPWMTZFlagClear(SOC_EHRPWM_1_REGS, EHRPWM_TZ_CYCLEBYCYCLE_CLEAR);
}



















