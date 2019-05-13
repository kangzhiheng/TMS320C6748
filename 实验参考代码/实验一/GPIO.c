//-----------------------------------------------------------------------------
// ��    �ߣ�adoredee
// ����ʱ�䣺2019.04.28
// ��    ����ʵ��һ GPIO����LED�붨ʱ���ж�
// ��    ����
//-----------------------------------------------------------------------------
// Copyright (C) 2019 Shanghai Jiao Tong University
//-----------------------------------------------------------------------------

/*
    ʵ��һ��GPIO����LED�붨ʱ���ж�
  	����Ҫ��ʹ��GPIO�����ĸ�LED�ƣ�����������˳��ѭ���ظ�����״̬���ʱ������ԼΪ500ms����
		 (1��LED0 ��������ȫ�𣩣�
		��2��LED1 ��������ȫ�𣩣�
		��3��LED2 ��������ȫ�𣩣�
		��4��LED3 ��������ȫ�𣩣�
		��5��LED2 ��������ȫ�𣩣�
		��6��LED1 ��������ȫ�𣩣�
		��7��LED0 ��������ȫ�𣩣�
     ��չ1���ڻ���Ҫ��Ļ����ϣ���������˳�򣨸�״̬���ʱ������ԼΪ500ms����
		��8��LED0 LED1 ��������ȫ�𣩣�
		��9��LED0 LED1 LED2 ��������ȫ�𣩣�
		��10��ȫ����
		��11��LED0 ������ȫ������
		��12��LED1 ������ȫ������
		��13��LED2 ������ȫ������
		��14��LED3 ������ȫ������
		��15��LED2 ������ȫ������
		��16��LED1 ������ȫ������
		��17��LED0 ������ȫ������
		��18��ȫ��
    ��չ2���ڻ���Ҫ�����չ1�Ļ����ϣ�Ҫ���״̬���ʱ�������ö�ʱ���ж�ʵ��500ms����ʱ��
*/

#include "gpio.h"        // ͨ����������ں꼰�豸����㺯������������ĺ������ڶ�Ӧ��.c�ļ��в���
#include "psc.h"         // ��Դ��˯�߿��ƺ꼰�豸����㺯������������psc.c�鿴����˵��

#include "soc_C6748.h"   // DSP C6748 ����Ĵ���
#include "lcdkC6748.h"   // 

/****************************************************************************/
/*              LOCAL FUNCTION PROTOTYPES                                   */
/****************************************************************************/
static void Delay(volatile unsigned int delay);    // ��ʱ����

/****************************************************************************/
/*              GLOBAL VARIABLES                                            */
/****************************************************************************/

/****************************************************************************/
/*             LOCAL FUNCTION DEFINITIONS                                   */
/****************************************************************************/
/*
    volatile�ؼ��� ���� �Ա����Ĵ�ȡ���ܻ��浽�Ĵ�����ÿ��ʹ��ʱ��Ҫ���ڴ������´�ȡ��
	�������Է��ʸñ����Ĵ���Ͳ��ٽ����Ż�����ϵͳ�������´������ڵ��ڴ��ȡ���ݣ�
	�Ӷ������ṩ�������ַ���ȶ�����
*/
#define _HWREG(x) (*((volatile Uint32*)(x)))    
#define PINMUX0 _HWREG(0x01C14120)     // ��·�Ĵ���0�ĵ�ַ 0X8000000
#define PINMUX5 _HWREG(0x01C14134)     // ��·�Ĵ���5�ĵ�ַ 0X8000
#define PINMUX13 _HWREG(0x01C14154)    // ��·�Ĵ���13�ĵ�ַ 0X8800
static unsigned char mode = 1;
typedef unsigned int Uint32;

/*��·�Ĵ���13��15-12λ��Ϊ8h������ΪGP6[12]���� */
void _GPIOBank6Pin12PinMuxSetup()
{
	PINMUX13 &= ~(0xf << 12);    // 0X800�����Ƚ���·�Ĵ���13��15-12λ�����㣬���ƹرտ�������
	PINMUX13 |= (0x8 << 12);     // 0X8800���ٽ���·�Ĵ���13��15-12λ�ø�ֵ�����ƴ򿪿�������
}

/*��·�Ĵ���13��11-8λ��Ϊ8h������ΪGP6[13]����*/
void _GPIOBank6Pin13PinMuxSetup()
{
	PINMUX13 &= ~(0xf << 8);   // 0X8000
	PINMUX13 |= (0x8 << 8);    // 0X8800
}
/*��·�Ĵ���5��15-12λ��Ϊ8h������ΪGP2[12]����*/
void _GPIOBank2Pin12PinMuxSetup()
{
	PINMUX5 &= ~(0xf << 12);    // 0X0
	PINMUX5 |= (0x8 << 12);     // 0X8000
}

/*��·�Ĵ���0��27-24λ��Ϊ8h������ΪGP0[9]����*/
void _GPIOBank0Pin9PinMuxSetup()
{
	PINMUX0 &= ~(0xf << 24);    // 0X0
	PINMUX0 |= (0x8 << 24);     // 0X8000000
}

int main(void)
{
/****************************************************************************/
/*                                                                          */
/*                  ����ʹ������                                             */
/*                                                                          */
/****************************************************************************/
    PSCModuleControl(SOC_PSC_1_REGS,     // PSC���ڴ��еĻ�����ַ��GPIO����PSC1ģ��
	    			 HW_PSC_GPIO,        // GPIO�ı���PSCģ�����3
					 PSC_POWERDOMAIN_ALWAYS_ON,    // Ĭ��
					 PSC_MDCTL_NEXT_ENABLE);       // Ĭ��
/****************************************************************************/
/*                                                                          */
/*                  GPIO �ܽŸ�������                                        */
/*                                                                          */
/****************************************************************************/
    // GPIO�ܽŸ������ã�ͨ�����������
	_GPIOBank6Pin12PinMuxSetup();
	_GPIOBank6Pin13PinMuxSetup();
	_GPIOBank2Pin12PinMuxSetup();
	_GPIOBank0Pin9PinMuxSetup();

/****************************************************************************/
/*                                                                          */
/*                   GPIO �ܽų�ʼ��                                         */
/*                                                                          */
/****************************************************************************/
    /*
	GPIODirModeSet���ùܽŵķ�������GPIO���������
	��һ��������GPIOģ�����ڴ��еĵ�ַ
	�ڶ���������GPIO�ڵ���ţ�C6748����144�����
	*/
    GPIODirModeSet(SOC_GPIO_0_REGS, 109, GPIO_DIR_OUTPUT);    // GP6[12]�����Ϊ109
    GPIODirModeSet(SOC_GPIO_0_REGS, 110, GPIO_DIR_OUTPUT);    // GP6[13]�����Ϊ110
    GPIODirModeSet(SOC_GPIO_0_REGS, 45, GPIO_DIR_OUTPUT);     // GP2[12]�����Ϊ45
    GPIODirModeSet(SOC_GPIO_0_REGS, 10, GPIO_DIR_OUTPUT);     // GP0[9] �����Ϊ10

/****************************************************************************/
/*                                                                          */
/*              LED�����                                                   */
/*                                                                          */
/****************************************************************************/
    
    /*
	����������4��LED��(D4��D5��D6��D7����Ϊ��LED0��LED1��LED2��LED3)��ÿ��LED��GPIO�ܽż���ŵĶ�Ӧ��ϵ���£�
	     LED        GPIO�ܽ�   ���
		D4(LED0) ���� GP6[13] ���� 110
		D5(LED1) ���� GP6[12] ���� 109
		D6(LED2) ���� GP2[12] ���� 45
		D7(LED3) ���� GP0[9] ���� 10
	*/
    while(1)
    {
    	switch (mode)
    		{
			/*
	        GPIOPinWrite�����ı䵱ǰ�ܽŵ�״̬
	        */
    		/* ����LED0������LED0��GPIO����GP6[13]Ϊ�ߵ�ƽ������Ϊ�͵�ƽ */
    		case 1: GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_HIGH);
					GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_LOW);
					GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_LOW);
					GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_LOW);
					break;
			/* ����LED1������LED1��GPIO����GP6[12]Ϊ�ߵ�ƽ������Ϊ�͵�ƽ */
    		case 2: GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_LOW);
					GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_HIGH);
					GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_LOW);
					GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_LOW);
					break;
			/* ����LED2������LED2��GPIO����GP2[12]Ϊ�ߵ�ƽ������Ϊ�͵�ƽ */
    		case 3: GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_LOW);
					GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_LOW);
					GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_HIGH);
					GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_LOW);
					break;
    	    /* ����LED3������LED3��GPIO����GP0[9]Ϊ�ߵ�ƽ������Ϊ�͵�ƽ */
    		case 4: GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_LOW);
					GPIOPinWrite(SOC_GPIO_0_REGS,109, GPIO_PIN_LOW);
					GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_LOW);
					GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_HIGH);
					break;
            /* ����LED2������LED2��GPIO����GP2[12]Ϊ�ߵ�ƽ������Ϊ�͵�ƽ */
    		case 5: GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_LOW);
					GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_LOW);
					GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_HIGH);
					GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_LOW);
					break;
    	    /* ����LED1������LED1��GPIO����GP6[12]Ϊ�ߵ�ƽ������Ϊ�͵�ƽ */
    		case 6: GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_LOW);
    		        GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_HIGH);
                    GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_LOW);
					GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_LOW);
					break;
    	    /* ����LED0������LED0��GPIO����GP6[13]Ϊ�ߵ�ƽ������Ϊ�͵�ƽ */
    		case 7: GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_HIGH);
					GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_LOW);
					GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_LOW);
					GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_LOW);
					break;
            /* ����LED0.1������LED0��GPIO����GP6[13]��LED1��GPIO����GP6[12]Ϊ�ߵ�ƽ������Ϊ�͵�ƽ */
    		case 8: GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_HIGH);
					GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_HIGH);
					GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_LOW);
					GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_LOW);
					break;
    	    /* ����LED0.1.2������LED3��GPIO����GP0[9]Ϊ�͵�ƽ������Ϊ�ߵ�ƽ */
    		case 9: GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_HIGH);
    		        GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_HIGH);
                    GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_HIGH);
					GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_LOW);
					break;
    	    /* ����LED0.1.2.3������GPIO����GP6[13]��GP6[12]��GP2[12]��GP0[9]ȫ��Ϊ�ߵ�ƽ */
    		case 10: GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_HIGH);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_HIGH);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_HIGH);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_HIGH);
					 break;
            /* Ϩ��LED0������ȫ����������LED0��GPIO����GP6[13]Ϊ�͵�ƽ������Ϊ�ߵ�ƽ */
    		case 11: GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_LOW);
					GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_HIGH);
					GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_HIGH);
					GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_HIGH);
					break;
    	    /* Ϩ��LED1������ȫ����������LED1��GPIO����GP6[12]Ϊ�͵�ƽ������Ϊ�ߵ�ƽ */
    		case 12: GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_HIGH);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_LOW);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_HIGH);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_HIGH);
					 break;
			/* Ϩ��LED2������ȫ����������LED2��GPIO����GP2[12]Ϊ�͵�ƽ������Ϊ�ߵ�ƽ */
    		case 13: GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_HIGH);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_HIGH);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_LOW);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_HIGH);
					 break;
            /* Ϩ��LED3������ȫ����������LED3��GPIO����GP0[9]Ϊ�͵�ƽ������Ϊ�ߵ�ƽ */
    		case 14: GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_HIGH);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_HIGH);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_HIGH);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_LOW);
					 break;
    	    /* Ϩ��LED2������ȫ����������LED2��GPIO����GP2[12]Ϊ�͵�ƽ������Ϊ�ߵ�ƽ */
    		case 15: GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_HIGH);
    		         GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_HIGH);
                     GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_LOW);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_HIGH);
					 break;
	       /* Ϩ��LED1������ȫ����������LED1��GPIO����GP6[12]Ϊ�͵�ƽ������Ϊ�ߵ�ƽ */
    		case 16: GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_HIGH);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_LOW);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_HIGH);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_HIGH);
					 break;
           /* Ϩ��LED0������ȫ����������LED0��GPIO����GP6[13]Ϊ�͵�ƽ������Ϊ�ߵ�ƽ */
    		case 17: GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_LOW);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_HIGH);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_HIGH);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_HIGH);
					 break;
    	    /* ȫ������GPIO����GP6[13]��GP6[12]��GP2[12]��GP0[9]ȫ��Ϊ�͵�ƽ */
    		case 18: GPIOPinWrite(SOC_GPIO_0_REGS,110, GPIO_PIN_LOW);
    		         GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_LOW);
                     GPIOPinWrite(SOC_GPIO_0_REGS, 45, GPIO_PIN_LOW);
					 GPIOPinWrite(SOC_GPIO_0_REGS, 10, GPIO_PIN_LOW);
					 break;
    }

    /*�����������и��£���1-18����ѭ��*/
    if (mode <= 18) 
	    mode=mode+1;
    else 
	    mode = 1;

	Delay(1000000);    // ��ʱ

    }
}

// ��ʱ����
static void Delay(volatile unsigned int delay)
{
    while(delay--);
}
