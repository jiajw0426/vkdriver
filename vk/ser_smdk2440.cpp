//
// Copyright (c) Microsoft Corporation.  All rights reserved.
//
//
// Use of this source code is subject to the terms of the Microsoft end-user
// license agreement (EULA) under which you licensed this SOFTWARE PRODUCT.
// If you did not accept the terms of the EULA, you are not authorized to use
// this source code. For a copy of the EULA, please see the LICENSE.RTF on your
// install media.
//
/*++
THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF
ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
PARTICULAR PURPOSE.

Module Name:  

Abstract:

    Serial PDD for SamSang 2440 Development Board.

Notes: 
--*/
#include <windows.h>
#include <types.h>
#include <ceddk.h>

#include <ddkreg.h>
#include <serhw.h>
#include <Serdbg.h>
#include <pdds3c2440_ser.h>
#include <s3c2440a_base_regs.h>
#include <s3c2440a_ioport.h>
#include "vk3234.h"


// CPdd2440Serial0 is only use for UART 0 which 
// RTS & CTS is use GPH0 GPH1
// DTR & DSR is USE GPD0 GPD1
class CPdd2440Serial0 : public CPdd2440Uart {
public:
    CPdd2440Serial0(LPTSTR lpActivePath, PVOID pMdd, PHWOBJ pHwObj)
        : CPdd2440Uart(lpActivePath, pMdd, pHwObj)
        {
		DEBUG_E(TEXT("CPdd2440Serial0"),TEXT("CPdd2440Serial0")) 
        m_pIOPregs = NULL;
		DEBUG_L(TEXT("CPdd2440Serial0"),TEXT("CPdd2440Serial0")) 
    }
    ~CPdd2440Serial0() {
        if (m_pIOPregs!=NULL)
            MmUnmapIoSpace((PVOID)m_pIOPregs,0);
    }
	virtual BOOL Init() {
		   DEBUG_E(TEXT("CPdd2440Serial0"),TEXT("Init")) 
			   BOOL result;
	        PHYSICAL_ADDRESS    ioPhysicalBase = { S3C2440A_BASE_REG_PA_IOPORT, 0};
	        ULONG               inIoSpace = 0;
	        if (TranslateBusAddr(m_hParent,Internal,0, ioPhysicalBase,&inIoSpace,&ioPhysicalBase)) {
	            // Map it if it is Memeory Mapped IO.
	            m_pIOPregs = (S3C2440A_IOPORT_REG *)MmMapIoSpace(ioPhysicalBase, sizeof(S3C2440A_IOPORT_REG),FALSE);
	        }
	        if (m_pIOPregs) {
			DDKISRINFO ddi;
			if (GetIsrInfo(&ddi)== ERROR_SUCCESS && 
  	                  KernelIoControl(IOCTL_HAL_REQUEST_SYSINTR, &ddi.dwIrq, sizeof(UINT32), &ddi.dwSysintr, sizeof(UINT32), NULL))
			{
				RegSetValueEx(DEVLOAD_SYSINTR_VALNAME,REG_DWORD,(PBYTE)&ddi.dwSysintr, sizeof(UINT32));
			}
			else
				return FALSE;
			m_pIOPregs->GPHCON &= ~(0x3<<0 | 0x3<<2 | 0x3<<4 | 0x3<<6 );//tx,rx,rts,cts
			m_pIOPregs->GPHCON |=  (0x2<<0 | 0x2<<2 | 0x2<<4 | 0x2<<6 ); 
			m_pIOPregs->GPHUP  |= 0xf;
			result=CPdd2440Uart::Init();
			 DEBUG_L(TEXT("CPdd2440Serial0"),TEXT("Init")) 
			return result;
		}
		return FALSE;
	};
	virtual void    SetDefaultConfiguration() {
		CPdd2440Uart::SetDefaultConfiguration();
	}
	virtual BOOL    InitModem(BOOL bInit) {
		SetDTR(bInit);
		return CPdd2440Uart::InitModem(bInit);
	}
	virtual ULONG   GetModemStatus() {
		return (CPdd2440Uart::GetModemStatus());		//HJ_add 20100112
	}
	virtual void    SetDTR(BOOL bSet) {
	};
private:
    volatile S3C2440A_IOPORT_REG * m_pIOPregs; 
};


// CPdd2440Serial1 is only use for UART 1 which 
class CPdd2440Serial1 : public CPdd2440Uart {
public:
    CPdd2440Serial1(LPTSTR lpActivePath, PVOID pMdd, PHWOBJ pHwObj)
        : CPdd2440Uart(lpActivePath, pMdd, pHwObj)
        {
			DEBUG_E(TEXT("CPdd2440Serial1"),TEXT("CPdd2440Serial1")) 
        m_pIOPregs = NULL;
			DEBUG_L(TEXT("CPdd2440Serial1"),TEXT("CPdd2440Serial1")) 
    }
    ~CPdd2440Serial1() {
        if (m_pIOPregs!=NULL)
            MmUnmapIoSpace((PVOID)m_pIOPregs,0);
    }
    virtual BOOL Init() {
		BOOL result;
		DEBUG_E(TEXT("CPdd2440Serial1"),TEXT("init")) 
        PHYSICAL_ADDRESS    ioPhysicalBase = { S3C2440A_BASE_REG_PA_IOPORT, 0};
        ULONG               inIoSpace = 0;
        if (TranslateBusAddr(m_hParent,Internal,0, ioPhysicalBase,&inIoSpace,&ioPhysicalBase)) {
            // Map it if it is Memeory Mapped IO.
            m_pIOPregs =(S3C2440A_IOPORT_REG *) MmMapIoSpace(ioPhysicalBase, sizeof(S3C2440A_IOPORT_REG),FALSE);
        }
        if (m_pIOPregs) {
            DDKISRINFO ddi;
            if (GetIsrInfo(&ddi)== ERROR_SUCCESS && 
                    KernelIoControl(IOCTL_HAL_REQUEST_SYSINTR, &ddi.dwIrq, sizeof(UINT32), &ddi.dwSysintr, sizeof(UINT32), NULL))
            {
                RegSetValueEx(DEVLOAD_SYSINTR_VALNAME,REG_DWORD,(PBYTE)&ddi.dwSysintr, sizeof(UINT32));
            }
            else
                return FALSE;

	        m_pIOPregs->GPHCON &= ~(0x3<<8 | 0x3<<10); // uart 2 - rx, tx
	        m_pIOPregs->GPHCON |= (0x2<<8 | 0x2<<10); 
			m_pIOPregs->GPHUP |= 0x30;

             result=CPdd2440Uart::Init();
			 DEBUG_L(TEXT("CPdd2440Serial1"),TEXT("init")) 
			return result;
        }
        return FALSE;
    };
    virtual void    SetDefaultConfiguration() {
        CPdd2440Uart::SetDefaultConfiguration();
    }
    virtual ULONG   GetModemStatus() {
        return (CPdd2440Uart::GetModemStatus() | MS_CTS_ON);
    }
    virtual void    Rx_Pause(BOOL bSet) {
    }

    volatile S3C2440A_IOPORT_REG * m_pIOPregs; 
};


// CPdd2440Serial2 is only use for UART 2 which 
class CPdd2440Serial2 : public CPdd2440Uart {
public:
    CPdd2440Serial2(LPTSTR lpActivePath, PVOID pMdd, PHWOBJ pHwObj)
        : CPdd2440Uart(lpActivePath, pMdd, pHwObj)
        {
			DEBUG_E(TEXT("CPdd2440Serial2"),TEXT("CPdd2440Serial2")) 
        m_pIOPregs = NULL;
			DEBUG_L(TEXT("CPdd2440Serial2"),TEXT("CPdd2440Serial2")) 
    }
    ~CPdd2440Serial2() {
        if (m_pIOPregs!=NULL)
            MmUnmapIoSpace((PVOID)m_pIOPregs,0);
    }
    virtual BOOL Init() {
		BOOL result;
		DEBUG_E(TEXT("CPdd2440Serial2"),TEXT("init")) 
        PHYSICAL_ADDRESS    ioPhysicalBase = { S3C2440A_BASE_REG_PA_IOPORT, 0};
        ULONG               inIoSpace = 0;
        if (TranslateBusAddr(m_hParent,Internal,0, ioPhysicalBase,&inIoSpace,&ioPhysicalBase)) {
            // Map it if it is Memeory Mapped IO.
            m_pIOPregs =(S3C2440A_IOPORT_REG *) MmMapIoSpace(ioPhysicalBase, sizeof(S3C2440A_IOPORT_REG),FALSE);
        }
        if (m_pIOPregs) {
            DDKISRINFO ddi;
            if (GetIsrInfo(&ddi)== ERROR_SUCCESS && 
                    KernelIoControl(IOCTL_HAL_REQUEST_SYSINTR, &ddi.dwIrq, sizeof(UINT32), &ddi.dwSysintr, sizeof(UINT32), NULL))
            {
                RegSetValueEx(DEVLOAD_SYSINTR_VALNAME,REG_DWORD,(PBYTE)&ddi.dwSysintr, sizeof(UINT32));
            }
            else
                return FALSE;

	        m_pIOPregs->GPHCON &= ~(0x3<<12 | 0x3<<14); // uart 2 - rx, tx
	        m_pIOPregs->GPHCON |= (0x2<<12 | 0x2<<14); 
			m_pIOPregs->GPHUP |= 0xc0;

              result= CPdd2440Uart::Init();
			DEBUG_L(TEXT("CPdd2440Serial2"),TEXT("init")) 
				return result;
        }
        return FALSE;
    };
    virtual void    SetDefaultConfiguration() {
        CPdd2440Uart::SetDefaultConfiguration();
    }
    virtual ULONG   GetModemStatus() {
        return (CPdd2440Uart::GetModemStatus() | MS_CTS_ON);
    }
    virtual void    Rx_Pause(BOOL bSet) {
    }

    volatile S3C2440A_IOPORT_REG * m_pIOPregs; 
};
CSerialPDD * CreateSerialObject(LPTSTR lpActivePath, PVOID pMdd,PHWOBJ pHwObj, DWORD DeviceArrayIndex)
{
	DEBUG_E(TEXT("CreateSerialObject"),TEXT("CreateSerialObject")) 
	CSerialPDD * pSerialPDD = NULL;
	switch (DeviceArrayIndex) {
		case 0:
#if(BSP_UARTn != BSP_use_UART0)
			pSerialPDD = new CPdd2440Serial0(lpActivePath,pMdd, pHwObj);
#endif
			break;
		case 1:
		case 2:
		case 3:
        case 4:
#if(BSP_UARTn != BSP_use_UART1)
			pSerialPDD = new CPdd2440Serial1(lpActivePath,pMdd, pHwObj);
#endif
			break;
		case 5:
		case 6:
		case 7:
		case 9:
			pSerialPDD = new CPdd2440Serial2(lpActivePath,pMdd, pHwObj);
			break;
	}
	if (pSerialPDD && !pSerialPDD->Init()) {
		delete pSerialPDD;
		pSerialPDD = NULL;
	}
	DEBUG_E(TEXT("CreateSerialObject"),TEXT("CreateSerialObject"))
	return pSerialPDD;
}
void DeleteSerialObject(CSerialPDD * pSerialPDD)
{
	if (pSerialPDD)
		delete pSerialPDD;
}

