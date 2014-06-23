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

    Platform dependent Serial definitions for 16550  controller.

Notes: 
--*/
#ifndef __PDDS3C2440_SER_H_
#define __PDDS3C2440_SER_H_

#include <cserpdd.h>
#include <cmthread.h>
#include <S3c2440a_intr.h>
#include "bsp_cfg.h"
#ifndef DEBUG
#define DEBUG

#endif

/////////////////////////////////////////////////////////////////////////////////////////
//// S3C2440 UART Register

///////++ UART CONTROL REGISTER ++
// Line control register bitvalue mask
#define SER2440_PARITY_MASK						0x38
#define SER2440_STOPBIT_MASK					0x4
#define SER2440_DATABIT_MASK					0x3
#define SER2440_IRMODE_MASK						0x40

// Fifo Status
#define SER2440_FIFOSTAT_MASK					0xf0

//
#define SER2440_FIFOFULL_TX						0x200
#define SER2440_FIFOCNT_MASK_TX					0xf0
#define SER2440_FIFO_DEPTH_TX					64
#define SER2440_FIFO_DEPTH_RX					64

//
#define SER2440_INT_INVALID						0x5a5affff

// Modem control register
#define SER2440_AFC								(0x10)
#define SER2440_RTS								0x1
//Receive Mode
#define RX_MODE_MASK							(0x11)
#define RX_DISABLE								(0x00)
#define RX_INTPOLL								(0x01)
#define RX_DMA0									(0x10)
#define RX_DMA1									(0x11)
//Transmit Mode
#define TX_MODE_MASK							(0x11 << 2)
#define TX_DISABLE								(0x00 << 2)
#define TX_INTPOLL								(0x01 << 2)
#define TX_DMA0									(0x10 << 2)
#define TX_DMA1									(0x11 << 2)
//Send Break Signal
#define BS_MASK									(0x01 << 4)
#define BS_NORM									(0x00 << 4)
#define BS_SEND									(0x01 << 4)
//Loop-back Mode
#define LB_MASK									(0x01 << 5)
#define LB_NORM									(0x00 << 5)
#define LB_MODE									(0x01 << 5)
//Rx Error Status Interrupt Enable
#define RX_EINT_MASK							(0x01 << 6)
#define RX_EINTGEN_OFF							(0x00 << 6)
#define RX_EINTGEN_ON							(0x01 << 6)
//Rx Time Out Enable
#define RX_TIMEOUT_MASK							(0x01 << 7)
#define RX_TIMEOUT_DIS							(0x00 << 7)
#define RX_TIMEOUT_EN							(0x01 << 7)
//Rx Interrupt Type
#define RX_INTTYPE_MASK							(0x01 << 8)
#define RX_INTTYPE_PUSE							(0x00 << 8)
#define RX_INTTYPE_LEVEL						(0x01 << 8)
//Tx Interrupt Type
#define TX_INTTYPE_MASK							(0x01 << 9)
#define TX_INTTYPE_PUSE							(0x00 << 9)
#define TX_INTTYPE_LEVEL						(0x01 << 9)
// Clock selection
#define CS_MASK									(0x01 << 10)
#define CS_PCLK									(0x00 << 10)
#define CS_UCLK									(0x01 << 10)
// UER State Error Bit.
#define UERSTATE_BREAK_DETECT					0x8
#define UERSTATE_FRAME_ERROR					0x4
#define UERSTATE_PARITY_ERROR					0x2
#define UERSTATE_OVERRUN_ERROR					0x1

/////////////////////////////////////////////////////////////////////////////////////////
// Required Registry Setting.
#define PC_REG_2440UART_INTBIT_VAL_NAME TEXT("InterruptBitsShift")
#define PC_REG_2440UART_INTBIT_VAL_LEN sizeof(DWORD)
#define PC_REG_2440UART_IST_TIMEOUTS_VAL_NAME TEXT("ISTTimeouts")
#define PC_REG_2440UART_IST_TIMEOUTS_VAL_LEN sizeof(DWORD)
/////////////////////////////////////////////////////////////////////////////////////////
#define S2440UART_INT_RXD						1
#define S2440UART_INT_TXD						2
#define S2440UART_INT_ERR						4
////////////////////////////////////////////////////////////////////////////////////////
// WaterMarker Pairs.
typedef struct  __PAIRS {
    ULONG   Key;
    ULONG   AssociatedValue;
} PAIRS, *PPAIRS;


 class CReg2440Uart {
public:
    CReg2440Uart(PULONG pRegAddr,DWORD childSerialIndex);
    virtual ~CReg2440Uart() { ; };
    virtual BOOL    Init() ;
    // We do not virtual Read & Write data because of Performance Concern.
    void    Write_ULCON(ULONG uData) { WRITE_REGISTER_ULONG( m_pReg, (uData)); };
    ULONG   Read_ULCON() { return (READ_REGISTER_ULONG(m_pReg)); } ;
    void    Write_UCON (ULONG uData) { WRITE_REGISTER_ULONG(m_pReg+1 , uData); };
    ULONG   Read_UCON() { return READ_REGISTER_ULONG(m_pReg+1 ); };
    void    Write_UFCON(ULONG uData) { WRITE_REGISTER_ULONG( m_pReg+2, uData);};
    ULONG   Read_UFCON() { return READ_REGISTER_ULONG(m_pReg + 2); };
    void    Write_UMCON(ULONG uData) { WRITE_REGISTER_ULONG(m_pReg + 3, uData);};
    ULONG   Read_UMCON() { return READ_REGISTER_ULONG(m_pReg + 3);};
    ULONG   Read_UTRSTAT() { return READ_REGISTER_ULONG(m_pReg + 4);};
    ULONG   Read_UERSTAT() { return READ_REGISTER_ULONG(m_pReg + 5);};
    ULONG   Read_UFSTAT() { return READ_REGISTER_ULONG(m_pReg + 6);};
    ULONG   Read_UMSTAT() { return READ_REGISTER_ULONG(m_pReg + 7);};
    void    Write_UTXH (UINT8 uData) { WRITE_REGISTER_ULONG( (m_pReg + 8), uData) ; };
    UINT8   Read_URXH() { return (UINT8) READ_REGISTER_ULONG(m_pReg + 9); };
    void    Write_UBRDIV(ULONG uData) { WRITE_REGISTER_ULONG( m_pReg + 10, uData );};
    ULONG   Read_UBRDIV() { return READ_REGISTER_ULONG(m_pReg + 10); };

	void    WriteVKReg(BYTE regindex,BYTE value);
	BYTE    ReadVkReg(BYTE regindex);



    virtual BOOL    Write_BaudRate(ULONG uData);
    PULONG  GetRegisterVirtualAddr() { return m_pReg; };
    virtual void    Backup();
    virtual void    Restore();
#ifdef DEBUG
    virtual void    DumpRegister();
#endif
protected:
    volatile PULONG const  m_pReg;
    BOOL    m_fIsBackedUp;
	virtual BOOL   Write_DefaultBaudRate() ;
private:
    ULONG    m_ULCONBackup;
    ULONG    m_UCONBackup;
    ULONG    m_UFCONBackup;
    ULONG    m_UMCOMBackup;
    ULONG    m_UBRDIVBackup;
    ULONG    m_ChildSerialIndex;
    ULONG    m_BaudRate;
    ULONG    m_s3c2440_pclk;
};
class CPdd2440Uart: public CSerialPDD, public CMiniThread  {
public:
    CPdd2440Uart (LPTSTR lpActivePath, PVOID pMdd, PHWOBJ pHwObj);
    virtual ~CPdd2440Uart();
    virtual BOOL Init();
    virtual void PostInit();
    virtual BOOL MapHardware();
    virtual BOOL CreateHardwareAccess();
//  Power Manager Required Function.
    virtual void    SerialRegisterBackup() { m_pReg2440Uart->Backup(); };
    virtual void    SerialRegisterRestore() { m_pReg2440Uart->Restore(); };

// Implement CPddSerial Function.
// Interrupt
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
    virtual BYTE    GetWaterMarkBit();
    virtual void    Rx_Pause(BOOL bSet) {;};
protected:
    BOOL    m_bReceivedCanceled;
    DWORD   m_dwWaterMark;
//
//  Modem
public:
    virtual BOOL    InitModem(BOOL bInit);
    virtual void    ModemInterruptHandler() { GetModemStatus();};
    virtual ULONG   GetModemStatus();
    virtual void    SetDTR(BOOL bSet) {;};
    virtual void    SetRTS(BOOL bSet);
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
    BYTE            GetLineStatus();
    virtual void    SetOutputMode(BOOL UseIR, BOOL Use9Pin) ;

protected:
    CReg2440Uart *  m_pReg2440Uart;
    PVOID           m_pRegVirtualAddr;

    volatile S3C2440A_INTR_REG   * m_pINTregs;
    DWORD           m_dwIntShift;
public:
    void    DisableInterrupt(DWORD dwInt) {
        m_pINTregs->INTSUBMSK |= (dwInt<<m_dwIntShift);
    }
    void    EnableInterrupt(DWORD dwInt) {
        m_pINTregs->INTSUBMSK &= ~(dwInt<<m_dwIntShift);
    }
    void    ClearInterrupt(DWORD dwInt) {
        m_pINTregs->SUBSRCPND = (dwInt<<m_dwIntShift);
    }
    DWORD   GetInterruptStatus() { return ((m_pINTregs->SUBSRCPND) >> m_dwIntShift);};
    DWORD   GetIntrruptMask () { return ((~(m_pINTregs->INTSUBMSK) )>> m_dwIntShift); };
    
protected:
    CRegistryEdit m_ActiveReg;
//  Interrupt Handler
    DWORD       m_dwSysIntr;
    HANDLE      m_hISTEvent;
// Optional Parameter
    DWORD m_dwDevIndex;
    DWORD m_dwISTTimeout;

};

#endif
