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

    Platform dependent Serial definitions for TMS320DM320.
    Ingenient Technologies, Inc.

Notes: 
--*/
#ifndef __PDD_DM350_SER_H_
#define __PDD_DM350_SER_H_

#include <cserpdd.h>
#include <cmthread.h>
#include <dm350_uart.h>
#include "Dm350_vk3214.h"
/**********************************************************/

// UART mode setting
#define UART_RXFIFO_INT        0x1   // enable receiver trigger interrupt
#define UART_TXFIFO_INT        0x2  // enable transmitter trigger interrupt
#define UART_ERR_INT              0x4   // enable ERR interrupt





//UART int  status
#define UART_INT_STATUS_RECEVIE    0x1
#define UART_INT_STATUS_SEND       0x2
#define UART_INT_STATUS_ERR         0x4
#define UART_INT_STATUS_TIMEOUT 0x8


/////////////////////////////////////////////////////////////////////////////////////////
// Required Registry Setting.
#define PC_REG_320UART_REFCLOCK_VAL_NAME TEXT("ReferenceClock")
#define PC_REG_320UART_REFCLOCK_VAL_LEN sizeof(DWORD)
#define PC_REG_320UART_IST_TIMEOUTS_VAL_NAME TEXT("ISTTimeouts")
#define PC_REG_320UART_IST_TIMEOUTS_VAL_LEN sizeof(DWORD)

////////////////////////////////////////////////////////////////////////////////////////
// WaterMarker Pairs.
typedef struct  __PAIRS {
    ULONG   Key;
    ULONG   AssociatedValue;
} PAIRS, *PPAIRS;

void InitPortPinMux();


 class CRegDm350Uart {
public:
    CRegDm350Uart(PULONG pRegAddr);
    virtual ~CRegDm350Uart() { ; };
    virtual BOOL    Init() ;
/*
	// UART Data Transmit/Receive Registers
	void Write_DATA(UINT8 byData) {m_pReg->THR = (UINT32)byData;};
	UINT16 Read_DATA(void) {return (m_pReg->RBR); };
	
	void Write_IER(UINT32 byData) {m_pReg->IER= (UINT32)byData;};
	UINT16 Read_IER(void) {return (m_pReg->IER); };

	// UART Bit Rate Set Registers
	void Write_BRSR(UINT16 uData) {m_pReg->DLL = uData&0xff;m_pReg->DLH= (uData>>8)&0xff;} ;
	UINT32 Read_BRSR(void) { UINT16 uData; uData = ((UINT16)m_pReg->DLH&0xff)<<8; uData |= (UINT16)m_pReg->DLL&0XFF;return (uData); };

	// UART Mode Setup Registers
	void Write_MSR(UINT32 uData) {m_pReg->MCR = uData;} ;
	UINT32 Read_MSR(void) { return (m_pReg->MCR); };

	// UART Receive FIFO Control Registers
	void Write_RFCR(UINT32 uData) {m_pReg->FCR = uData;} ;
	UINT32 Read_RFCR(void) { return (m_pReg->FCR); };

	// UART Transmit FIFO Control Registers
	void Write_TFCR(UINT32 uData) {m_pReg->FCR = uData;} ;
	UINT32 Read_TFCR(void) { return (m_pReg->FCR); };

	// UART Line Control Registers
	void Write_LCR(UINT32 uData) {m_pReg->LCR = uData;} ;
	UINT32 Read_LCR(void) { return (m_pReg->LCR); };

	// UART Status Registers
	UINT32 Read_SR(void)  { return (m_pReg->LSR); };
	UINT32 Read_IIR(void)  { return (m_pReg->IIR); };
*/
    virtual BOOL    Write_BaudRate(ULONG ulData,ULONG ulBaseClock);
	
    virtual void    Backup();
    virtual void    Restore();
    virtual void    DumpRegister();
protected:
    volatile      P_DM350_UART_REG const  m_pReg;
    BOOL    	m_fIsBackedUp;
	
private:
    UINT32    	m_uBRSRBackup;		// Bit Rate Set Register
    UINT32    	m_uMSRBackup;		// Mode Setup Register
    UINT32   	m_uRFCRBackup;		// Receive FIFO Control Register
    UINT32    	m_uTFCRBackup;		// Transmit FIFO Control Register
    UINT32    	m_uLCRBackup;		// Line Control Register

};

 
class CPdd350Uart: public CSerialPDD, public CMiniThread  {
public:
    CPdd350Uart (LPTSTR lpActivePath, PVOID pMdd, PHWOBJ pHwObj);
    virtual ~CPdd350Uart();
    virtual BOOL Init();
    virtual void PostInit();
    virtual BOOL MapHardware();
    virtual BOOL CreateHardwareAccess();
//  Power Manager Required Function.
    virtual void    SerialRegisterBackup() { m_pReg350Uart->Backup(); };
    virtual void    SerialRegisterRestore() { m_pReg350Uart->Restore(); };

// Implement CPddSerial Function.
// Interrupt
// This virtual function must be overrided to support Modem !!
    virtual BOOL    InitialEnableInterrupt(BOOL bEnable ) ; // Enable All the interrupt may include Xmit Interrupt.
private:
    virtual DWORD ThreadRun();   // IST
//  Tx Function.
 
public:	 
    virtual BOOL    InitXmit(BOOL bInit);
    virtual void    XmitInterruptHandler(PUCHAR pTxBuffer, ULONG *pBuffLen);
    virtual void    XmitComChar(UCHAR ComChar);
    virtual BOOL    EnableXmitInterrupt(BOOL bEnable);
    virtual BOOL    CancelXmit();
    virtual DWORD   GetWriteableSize();
protected:
    BOOL    m_XmitFifoEnable;
    HANDLE  m_XmitFlushDone;

//
//  Rx Function.
public:
    virtual BOOL    InitReceive(BOOL bInit);
    virtual ULONG   ReceiveInterruptHandler(PUCHAR pRxBuffer,ULONG *pBufflen);
    virtual ULONG   CancelReceive();
    virtual DWORD   GetWaterMark();
    virtual UINT16  GetWaterMarkBit();
protected:
    BOOL    m_bReceivedCanceled;
    DWORD   m_dwWaterMark;
//
//  Modem
//  This virtual functions must be overrided to support Modem !!
public:			
    virtual BOOL    InitModem(BOOL bInit);
    virtual void    ModemInterruptHandler() { GetModemStatus();};
    virtual ULONG   GetModemStatus();
    virtual void    SetDTR(BOOL bSet) {;};	// Do nothing
    virtual void    SetRTS(BOOL bSet) {;};	// Do nothing
//
// Line Function.
    virtual BOOL    InitLine(BOOL bInit) ;
    virtual void    LineInterruptHandler() { GetLineStatus();};
    virtual void    SetBreak(BOOL bSet) ;
    virtual BOOL    SetBaudRate(ULONG BaudRate,BOOL bIrModule) ;
    virtual BOOL    SetByteSize(ULONG ByteSize);
    virtual BOOL    SetParity(ULONG Parity);
    virtual BOOL    SetStopBits(ULONG StopBits);
//
// Line Internal Function
    UINT16          GetLineStatus();
protected:
	void DisableInterrupt(UINT32 mask);
	void EnableInterrupt(UINT32 mask);
	UINT16 GetInterruptStatus(void); 

protected:	
    PCOM_DEVICE  pDeviceGlobal;       // g_pDeviceGlobal;
	
protected:
    CRegDm350Uart *  m_pReg350Uart;
    PVOID            m_pRegVirtualAddr;

protected:
    CRegistryEdit m_ActiveReg;
//  Interrupt Handler
    DWORD       m_dwSysIntr;
    HANDLE      m_hISTEvent;
// Optional Parameter
    DWORD 		m_dwDevIndex;
    DWORD 		m_dwISTTimeout;
    //DWORD 		m_dwBaseClock;
    DWORD            m_dwChildSerial;


	

};

#endif
