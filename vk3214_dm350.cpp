/**********************************************************************
File Name	:	ser_dm350.cpp

Author		:	xzh

Date		:	2008.3.27

Description	:	pdd user layer of uart 1

Copyright	:	SHENZHEN BestSpring Technology Co,Ltd. All rights reserved.

History		:	1. Create by xzh in 2008.3
	
************************************************************************/
#include <windows.h>
#include <types.h>
#include <ceddk.h>

#include <ddkreg.h>
#include <serhw.h>
#include <Serdbg.h>
#include "pdd_dm350_vk.h"
#include <BSP.h>

#include <dm350_clkc_lib.h>

//#include "dm350_vk3214.h"

#define ZONE_UART 1

void InitPortPinMux()
{
	RETAILMSG(ZONE_UART,(L"++InitPortPinMux %x\r\n"));
	CLKC_Grab();
	SysDm350_SetPinMuxBit(SYSCTRL_PINMUX3,PINMUX3_GIO19_SD1DATA0_UART2TXD_TWOBIT,2, TRUE);
	SysDm350_SetPinMuxBit(SYSCTRL_PINMUX3,PINMUX3_GIO20_SD1DATA1_UART2RXD_TWOBIT,2, TRUE);	
	CLKC_Release();
	GIO_Grab();
      GIO_SetDirection(GIO_PORT_1, GIO_INPUT, FALSE);
      GIO_SetIRQPort(GIO_PORT_1,GIO_IRQ_FALLING_EDGE);
      GIO_SetBankInt(GPIO_BANK0);
      GIO_Release();




	
      RETAILMSG(ZONE_UART,(L"--InitPortPinMux %x\r\n"));
}

class CPddVK3214Serial : public CPdd350Uart {
public:
    CPddVK3214Serial(LPTSTR lpActivePath, PVOID pMdd, PHWOBJ pHwObj)
        : CPdd350Uart(lpActivePath, pMdd, pHwObj)
    {
 /*   
        m_dwBaseClock = UART2_INPUT_CLOCK;
*/
    }

    ~CPddVK3214Serial() 
	{
		/*
		CLKC_Grab();
		//CLKC_EnableModule(CLKC_MODULE_UART1, FALSE);
		SysDm350_SetPinMuxBit(SYSCTRL_PINMUX3,PINMUX3_GIO19_SD1DATA0_UART2TXD_TWOBIT,0, TRUE);
		SysDm350_SetPinMuxBit(SYSCTRL_PINMUX3,PINMUX3_GIO20_SD1DATA1_UART2RXD_TWOBIT,0, TRUE);
		CLKC_Release();
		*/
		RETAILMSG(ZONE_UART,(L"~CPddVK3214Serial \r\n"));
		GIO_DeinitLibrary();
		CLKC_DeinitLibrary();
	}

    virtual BOOL Init() 
	{
     //   return 0;
     	    RETAILMSG(ZONE_UART,(L"++CPddVK3214Serial:Init \r\n"));
	    if (!GIO_InitLibrary())
		{
			return(0);		// failed
		}

		if (!CLKC_InitLibrary())
		{

			GIO_DeinitLibrary();

			return(0);		// failed
		}
		// Set up UART1 GIO pins
		if(!ForemostInitUart())
		{
			RETAILMSG(ZONE_UART,(L"ForemostInitUart fail ! \r\n"));
			return(0);
		}
		RETAILMSG(ZONE_UART,(L"--CPddVK3214Serial:Init \r\n"));
		
		return CPdd350Uart::Init();
		
       
    }

    virtual void    SetDefaultConfiguration() 
    {
        CPdd350Uart::SetDefaultConfiguration();
    }

};

CSerialPDD * CreateSerialObject(LPTSTR lpActivePath, PVOID pMdd,PHWOBJ pHwObj, DWORD DeviceArrayIndex)
{
    CSerialPDD * pSerialPDD = NULL;
    switch (DeviceArrayIndex) 
	{
      case 0:
        pSerialPDD = new CPddVK3214Serial(lpActivePath,pMdd, pHwObj);
        break;

    }
    if (pSerialPDD && !pSerialPDD->Init()) 
	{
        delete pSerialPDD;
        pSerialPDD = NULL;
    }    
    return pSerialPDD;
}

void DeleteSerialObject(CSerialPDD * pSerialPDD)
{
    if (pSerialPDD)
        delete pSerialPDD;
}

