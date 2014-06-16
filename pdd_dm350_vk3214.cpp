/**********************************************************************
File Name	:	pdd_dm350_ser.cpp

Author		:	xzh

Date		:	2008.3.27

Description	:	pdd  layer of uart 

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
//#include <dm350_gio_lib.h>
//#include <dm350_clkc_lib.h>

//#include "dm350_vk3214.h"
#define ZONE_UART 1

#define BAUDRATE_OK  (0x00000001)
#define DATA_SIZE_OK   (0x00000002)
#define PARITY_OK        (0x00000004)
#define STOP_BIT_OK      (0x00000008)
#define RECIVE_OK      (0x00000010)
#define TX_INT_EN      (0x00000020)

#define ALL_SET_OK  ( BAUDRATE_OK | DATA_SIZE_OK |PARITY_OK|STOP_BIT_OK|RECIVE_OK)
//PCOM_DEVICE pDeviceGlobal = NULL;           //考虑放到类中去wus
#define    CHECK_MSG      0
#define    SET_TXBUFF      0
#define    IRQ_GIO1          0x2d
#define    SEND_SCAN       8
#define    NORMAL_SCAN   10



CPdd350Uart::CPdd350Uart (LPTSTR lpActivePath, PVOID pMdd, PHWOBJ pHwObj )
:   CSerialPDD(lpActivePath,pMdd, pHwObj)
,   m_ActiveReg(HKEY_LOCAL_MACHINE,lpActivePath)
,   CMiniThread (0, TRUE)   
{
    RETAILMSG(1, (L"++CPdd350Uart::CPdd350Uart\r\n"));
    //dpCurSettings.ulZoneMask = ZONE_INIT | ZONE_FUNCTION; 

    
    PCOM_DEVICE pDevice = NULL;
    pDevice = (PCOM_DEVICE)LocalAlloc( LPTR, sizeof(COM_DEVICE) );
    if ( !pDevice )
    {
        RETAILMSG( 1, (TEXT("vk3214 LocalAlloc error:%d\n"), GetLastError() ));
    }
    memset(pDevice,0 ,sizeof(COM_DEVICE));
    pDeviceGlobal = pDevice;
    InitializeCriticalSection( &pDevice->Lock );
 //   InitializeCriticalSection( &pDevice->TXcs);
	
    m_pReg350Uart = NULL;
    m_dwSysIntr = MAXDWORD;
    m_hISTEvent = NULL;
    m_dwDevIndex = 0;
    m_pRegVirtualAddr = NULL;
    
    m_dwChildSerial = 0;        //wus
	
    m_XmitFlushDone =  CreateEvent(0, FALSE, FALSE, NULL);
    m_XmitFifoEnable = FALSE;
    m_dwWaterMark = 8 ;
}

CPdd350Uart::~CPdd350Uart()
{
	RETAILMSG(1, (L"--CPdd350Uart::CPdd350Uart\r\n"));
	InitModem(FALSE);
    if (m_hISTEvent) {
        m_bTerminated=TRUE;
        ThreadStart();
        SetEvent(m_hISTEvent);
        ThreadTerminated(1000);
        InterruptDisable( m_dwSysIntr );         
        CloseHandle(m_hISTEvent);
    };
    if (m_pReg350Uart)
        delete m_pReg350Uart;
    if (m_XmitFlushDone)
        CloseHandle(m_XmitFlushDone);
    if(pDeviceGlobal)
    {
	    LocalFree(pDeviceGlobal);
	    pDeviceGlobal = NULL;
     }
        
}
BOOL CPdd350Uart::Init()
{
    if (pDeviceGlobal && pDeviceGlobal->OpenCount >= 1)
        return FALSE;
    pDeviceGlobal->OpenCount = 1;

	if ( CSerialPDD::Init() && IsKeyOpened() && m_XmitFlushDone!=NULL)
	{ 
             // IST Setup .
         	m_hISTEvent= CreateEvent(0,TRUE,FALSE,NULL);

            if (m_hISTEvent==NULL)
            	{
			RETAILMSG(1, (L"CPdd350Uart::Init m_hISTEvent creat fail \r\n"));	
		      return FALSE;
            	}
            m_dwISTTimeout = INFINITE;

            if (pDeviceGlobal)
                pDeviceGlobal->hRecEvent = m_hISTEvent;
		
		
            if (!MapHardware()) {
                return FALSE;
            }
	     if (!CreateHardwareAccess()) {
                return FALSE;
            }	
        
        return TRUE;        
    }
    return FALSE;
}

//读出是第几个子串口驱动m_dwChildSerial
BOOL CPdd350Uart::MapHardware() 
{
    if (m_dwChildSerial != 0)
        return TRUE;

    // Get IO Window From Registry
    DDKWINDOWINFO dwi;
    if ( GetWindowInfo( &dwi)!=ERROR_SUCCESS || 
            dwi.dwNumMemWindows < 1 || 
            dwi.memWindows[0].dwBase == 0 || 
            dwi.memWindows[0].dwLen <  0xd)
        return FALSE;
    DWORD dwInterfaceType;
    if (m_ActiveReg.IsKeyOpened() && 
            m_ActiveReg.GetRegValue( DEVLOAD_INTERFACETYPE_VALNAME, (PBYTE)&dwInterfaceType,sizeof(DWORD))) {
        dwi.dwInterfaceType = dwInterfaceType;
    }

    // Translate to System Address.
    //PHYSICAL_ADDRESS    ioPhysicalBase = { dwi.memWindows[0].dwBase, 0};
    m_dwChildSerial = (DWORD)dwi.memWindows[0].dwBase;
    m_dwChildSerial &= 0x0F;
    pDeviceGlobal->m_childport = m_dwChildSerial;
    RETAILMSG(1, (L"MapHardware  m_dwChildSerial = %d\r\n",m_dwChildSerial));	
	
/*
    ULONG               inIoSpace = 0;
    if (TranslateBusAddr(m_hParent,(INTERFACE_TYPE)dwi.dwInterfaceType,dwi.dwBusNumber, ioPhysicalBase,&inIoSpace,&ioPhysicalBase)) {
        // Map it if it is Memeory Mapped IO.
        m_pRegVirtualAddr = MmMapIoSpace(ioPhysicalBase, dwi.memWindows[0].dwLen,FALSE);
    }
    */
    return (m_dwChildSerial!=NULL );
   
}

BOOL CPdd350Uart::CreateHardwareAccess()
{
		DWORD irq;
		DDKISRINFO ddi;
		if (GetIsrInfo(&ddi)==ERROR_SUCCESS  ) 
		{ 
			if (ddi.dwIrq!=0 && ddi.dwIrq < 0xff)
			{
				irq = ddi.dwIrq;
			
			}

                 else
		    {
                     RETAILMSG(1, (TEXT(" CPdd350Uart::Use sysint value from hive regestriy")));
		     }
	
		}
	  if(!VK3214ComAttach(m_dwChildSerial,pDeviceGlobal,irq))
      	{
      	    return FALSE;
      	}

    return TRUE;
}
//***************************************************************************
PVOID VK3214ComAttach(DWORD dwChildSerial ,PCOM_DEVICE pDevice,DWORD irq)     
{	
    BOOL bRc = TRUE;
    DWORD dwErr = ERROR_SUCCESS;
    //DWORD irq;
    PHYSICAL_ADDRESS    ioPhysicalBase = {UART2_PA_BASE_ADDRESS, 0};
    RETAILMSG( CHECK_MSG, (TEXT("++VK3214ComAttach\r\n")));

      if((dwChildSerial  > 4) || !pDevice )
    	{
		RETAILMSG( 1, (TEXT("something  fail  dwChildSerial = %x pDevice = %x \r\n"),dwChildSerial,pDevice));
		return 0;
    	}
       pDevice->v_pVKComReg = (P_DM350_UART_REG)MmMapIoSpace(ioPhysicalBase, sizeof(DM350_UART_REG),FALSE);	

	 if(!pDevice->v_pVKComReg)
	 {
		  RETAILMSG( 1, (TEXT("pDevice->v_pVKComReg fail \r\n")));
		   return 0;
	 }


	pDevice->hComChangedEvent = CreateEvent( NULL, FALSE, FALSE, NULL);
	
    if ( !pDevice->hComChangedEvent )
    {
        RETAILMSG( 1, (TEXT("pDevice->hComChangedEvent CreateEvent error:%d\n"), GetLastError() ));
        bRc = FALSE;
        goto ret;
    }
    RETAILMSG( 1, (TEXT("VK3214ComAttach hComChangedEvent  creat \r\n")));
    
	if(pDevice->m_childport !=0  &&  ppGlobalDevice !=NULL)
	{
           ppGlobalDevice[(pDevice->m_childport)-1] = pDevice;
       }

      if(dwUseCountUARTC ==1)
      {
		//vk3214_int_set();
		irq = IRQ_GIO1;
		       
	 }
   
   /*       	
	if (!KernelIoControl(
		IOCTL_HAL_REQUEST_SYSINTR, &irq, sizeof(irq), &pDevice->sysIntr,
		sizeof(pDevice->sysIntr), NULL)) 
	{
          
		bRc = FALSE;
		 RETAILMSG( 1, (TEXT("VK3214ComAttach KernelIoControl fail \r\n")));
	      goto ret;
	}
 //#if 1     
	// Initialize interrupt
	if (!InterruptInitialize(pDevice->sysIntr, pDevice->hComChangedEvent, NULL, 0))
	{
	   bRc = FALSE;
	   RETAILMSG( 1, (TEXT("VK3214ComAttach InterruptInitialize fail \r\n")));
           goto ret;

	}
///#endif
*/
    __try 
    {

/*************************************************************/
        pDevice->hComPollThread = CreateThread( NULL, 0, 
                                                ComPollThread, 
                                                pDevice, 
                                                0, NULL );
	  SetThreadPriority(pDevice->hComPollThread,  THREAD_PRIORITY_TIME_CRITICAL );//THREAD_PRIORITY_TIME_CRITICAL DEFAULT_CE_THREAD_PRIORITY	THREAD_PRIORITY_TIME_CRITICAL
        if ( !pDevice->hComPollThread )
        {
            DEBUGMSG( ZONE_ERROR, (TEXT("CreateThread error:%d\n"), GetLastError() ));
	     RETAILMSG( 1, (TEXT("++CreateThread error:%d\r\n"), GetLastError()));
            bRc = FALSE;
            goto ret;
        }
	  RETAILMSG( 1, (TEXT("VK3214ComAttach pDevice->hComPollThread  creat   \r\n")));
	  
/*************************************************************/
	  
    } __except (EXCEPTION_EXECUTE_HANDLER) 
    {
        dwErr = GetExceptionCode();
    } 
	
    RETAILMSG(1, (TEXT("Spi Attach:%d   pDevice->sysIntr = %d  irq = %d\n"), bRc,pDevice->sysIntr,irq));

   if(bRc)
   {
       VK3214ComInit((BYTE)dwChildSerial, pDevice);
   
   }
   
ret:
    if (!bRc)
    {
        if (pDevice)
        {
            //  ReleaseRemoveLock(&pDevice->RemoveLock,NULL); 
            VK3214ComDetach( pDevice );

        }

    }

    DEBUGMSG(ZONE_INIT, (TEXT("USB2COM<DiskAttach:%d\n"), bRc));

    return(bRc ? pDevice : NULL);     

	
    //return(VK3214ComInit(dwChildSerial));
 
}

BOOL
VK3214ComDetach(PCOM_DEVICE pDevice  )
{
    BOOL bRc = TRUE;
    DWORD dwWait = 0;

    if ( ! pDevice  )
    {
        DEBUGMSG( ZONE_ERROR, (TEXT("Invalid Context!\n")));
        return FALSE;
    }
    pDevice->DeviceRemoved = TRUE;
    // wait for the polling thread to terminate
    //
    if (pDevice->hComChangedEvent && pDevice->hComPollThread)
    {

        SetEvent(pDevice->hComChangedEvent);

        WaitForSingleObject(pDevice->hComPollThread, INFINITE);
        CloseHandle(pDevice->hComPollThread);
        pDevice->hComPollThread = NULL;
		CloseHandle(pDevice->hComChangedEvent);
		pDevice->hComChangedEvent = NULL;

    }

    if (&pDevice->Lock)
    {
        DeleteCriticalSection( &pDevice->Lock );
    }
 //    if (&pDevice->TXcs)
 //   {
 //       DeleteCriticalSection( &pDevice->TXcs );
 //   }
    return bRc;
}

BOOL GetByte(PCOM_DEVICE pDevice, BYTE *ret )
{
    EnterCriticalSection(&pDevice->Lock);
    if (pDevice->length > 0)
    {
        *ret = pDevice->readbuff[pDevice->readindex];
        //  RETAILMSG(1,(TEXT("%c" ),*ret ));
        pDevice->readindex++;
        pDevice->length--;
        if (pDevice->readindex >= (BULK_BLOCK_BYTE*4))
            pDevice->readindex = 0;
        LeaveCriticalSection(&pDevice->Lock);
        return TRUE;
    }
    else
    {
        LeaveCriticalSection(&pDevice->Lock);
        return FALSE;
    }
}

 BOOL WriteByte(PCOM_DEVICE pDevice, BYTE in)
{
    if (pDevice->length < BULK_BLOCK_BYTE*4)
    {
        pDevice->readbuff[pDevice->writeindex] = in ;
        pDevice->writeindex++;
        pDevice->length++;
        if (pDevice->writeindex >= (BULK_BLOCK_BYTE*4))
            pDevice->writeindex = 0;
        return TRUE;
    }
    else
    {
        return FALSE;// buf full
    }
}
#if SET_TXBUFF
static BOOL TXGetByte(PCOM_DEVICE pDevice, BYTE *ret )
{
    //EnterCriticalSection(&pDevice->TXcs);
    if (pDevice->tx_length > 0)
    {
        *ret = pDevice->writebuff[pDevice->tx_readindex];
        //  RETAILMSG(1,(TEXT("%c" ),*ret ));
        pDevice->tx_readindex++;
        pDevice->tx_length--;
        if (pDevice->tx_readindex >= (BULK_BLOCK_BYTE))
            pDevice->tx_readindex = 0;
       // LeaveCriticalSection(&pDevice->TXcs);
        return TRUE;
    }
    else
    {
       // LeaveCriticalSection(&pDevice->TXcs);
        return FALSE;
    }
}

BOOL TXWriteByte(PCOM_DEVICE pDevice, BYTE in)
{
    if (pDevice->tx_length < BULK_BLOCK_BYTE)
    {
        pDevice->writebuff[pDevice->tx_writeindex] = in ;
        pDevice->tx_writeindex++;
        pDevice->tx_length++;
        if (pDevice->tx_writeindex >= (BULK_BLOCK_BYTE))
            pDevice->tx_writeindex = 0;
        return TRUE;
    }
    else
    {
        return FALSE;// buf full
    }
}
#endif
//***************************************************************************

void CPdd350Uart::DisableInterrupt(UINT32 mask)
{
       // RETAILMSG(CHECK_MSG, (L"CPdd350Uart DisableInterrupt\r\n"));
        EnterCriticalSection(&pDeviceGlobal->Lock);
        pDeviceGlobal->m_serial_flag &= ~mask;
        LeaveCriticalSection(&pDeviceGlobal->Lock);
}

void CPdd350Uart::EnableInterrupt(UINT32 mask)
{	
      //RETAILMSG(CHECK_MSG, (L"CPdd350Uart EnableInterrupt\r\n"));
	EnterCriticalSection(&pDeviceGlobal->Lock);
      pDeviceGlobal->m_serial_flag |= mask;
      LeaveCriticalSection(&pDeviceGlobal->Lock);
}

UINT16 CPdd350Uart::GetInterruptStatus(void) 
{
      //UINT32 data = m_pReg350Uart->Read_IIR();
	//UINT32 mask = m_pReg350Uart->Read_IER();
	UINT32 data = 0;
	UINT32 mask = 0;	  
	UINT16 status = 0;

	 if (pDeviceGlobal)
       {
             EnterCriticalSection( & pDeviceGlobal->Lock);
             if (pDeviceGlobal->dwIntrtype&UART_INT_STATUS_RECEVIE)    //UART_INT_STATUS_SEND  UART_INT_STATUS_ERR
             {
                  status |= UART_INT_STATUS_RECEVIE;
                  pDeviceGlobal->dwIntrtype = 0;
              }
              //LeaveCriticalSection( & pDeviceGlobal->Lock);
		  //EnterCriticalSection( & pDeviceGlobal->TXcs);
		 if (pDeviceGlobal->dwTXIntrtype&UART_INT_STATUS_SEND)    //UART_INT_STATUS_SEND  UART_INT_STATUS_ERR
             {
                  status |= UART_INT_STATUS_SEND;
                  pDeviceGlobal->dwTXIntrtype = 0;
              }
		  LeaveCriticalSection( & pDeviceGlobal->Lock);
              
       }
		
	return status;
}


#define MAX_RETRY 0x1000
void CPdd350Uart::PostInit()
{
    DWORD dwCount=0;
    //m_HardwareLock.Lock();
    RETAILMSG(CHECK_MSG, (L"CPdd350Uart PostInit\r\n"));
    DisableInterrupt(UART_RXFIFO_INT|UART_TXFIFO_INT|UART_ERR_INT);
    // Mask all interrupt.
	
    while ((GetInterruptStatus() & (UART_INT_STATUS_RECEVIE|UART_INT_STATUS_SEND|UART_INT_STATUS_ERR|UART_INT_STATUS_TIMEOUT))!=0 && 
            dwCount <MAX_RETRY)
    { // Interrupt.
        InitReceive(TRUE);
        InitLine(TRUE);
        dwCount++;
    }
    	  // IST Start to Run.
    //m_HardwareLock.Unlock();
    CSerialPDD::PostInit();
    CeSetPriority(m_dwPriority256);  //m_dwPriority256
    //m_pReg350Uart->DumpRegister();
    ThreadStart();  // Start IST.
}

DWORD CPdd350Uart::ThreadRun()
{
     //RETAILMSG(1,(TEXT("CPdd350Uart::ThreadRun\r\n" )));  
   while ( m_hISTEvent!=NULL && !IsTerminated()) 
   {
	
	if (WaitForSingleObject( m_hISTEvent,m_dwISTTimeout)==WAIT_OBJECT_0) 
	 {
		ResetEvent(m_hISTEvent);
		 
		 //RETAILMSG(1,(TEXT("ThreadRun interrupt in \r\n" )));  
		
	      //m_HardwareLock.Lock();  
            while (!IsTerminated() ) {
                DWORD dwData = (GetInterruptStatus() & (UART_INT_STATUS_RECEVIE|UART_INT_STATUS_SEND|UART_INT_STATUS_ERR|UART_INT_STATUS_TIMEOUT));
                 //RETAILMSG(1,  (TEXT(" CPdd350Uart::ThreadRun INT=%x\r\n"),dwData));
                if (dwData) 
		  {
                    DWORD interrupts=INTR_MODEM; // Always check Modem when we have change. It may work at polling mode.
                    if ((dwData & (UART_INT_STATUS_RECEVIE|UART_INT_STATUS_TIMEOUT))!=0)
                        interrupts |= INTR_RX;
                    if ((dwData & UART_INT_STATUS_SEND)!=0)
                        interrupts |= INTR_TX;
                    if ((dwData & UART_INT_STATUS_ERR)!=0) 
                        interrupts |= INTR_LINE|INTR_RX;
			//RETAILMSG(1,(L"uart intr= %x\r\n",interrupts));
                    NotifyPDDInterrupt((INTERRUPT_TYPE)interrupts);
                    //ClearInterrupt(dwData);// TODO
                }
                else 
                    break;
            }
            //m_HardwareLock.Unlock();   
            //InterruptDone(m_dwSysIntr);// TODO
        }
        else { // Polling Modem.
            NotifyPDDInterrupt(INTR_MODEM);
            //RETAILMSG(1,(TEXT(" CPdd350Uart::ThreadRun timeout INT=%x\r\n"),GetInterruptStatus()));
#ifdef DEBUG
            if ( ZONE_THREAD )
            	{
              ; //RETAILMSG(1,(TEXT("ThreadRun ZONE_THREAD  \r\n" ))) ;  //m_pReg350Uart->DumpRegister();
               
            	}
#endif
        }
    }
    return 1;
}
BOOL CPdd350Uart::InitialEnableInterrupt(BOOL bEnable )
{
    //m_HardwareLock.Lock();
	 RETAILMSG(CHECK_MSG, (L"CPdd350Uart InitialEnableInterrupt\r\n"));
    if (bEnable) 
        EnableInterrupt(UART_RXFIFO_INT | UART_ERR_INT );
    else
        DisableInterrupt(UART_RXFIFO_INT | UART_ERR_INT);
    //m_HardwareLock.Unlock();
    return TRUE;
}

BOOL  CPdd350Uart::InitXmit(BOOL bInit)
{
	 RETAILMSG(CHECK_MSG, (L"CPdd350Uart InitXmit\r\n"));
    if (bInit) 
    { 
        //m_HardwareLock.Lock();    
        EnableInterrupt(UART_TXFIFO_INT | UART_ERR_INT );
        //m_HardwareLock.Unlock();
    }
   // else 
    //{ // Make Sure data has been trasmit out.
        // We have to make sure the xmit is complete because MDD will shut donw the device after this return
/*
	 DWORD dwTicks = 0;
        DWORD dwUTRState;
        while (dwTicks < 1000 && 
                (((dwUTRState = GetInterruptStatus()) & (UART_INT_STATUS_SEND))!=UART_INT_STATUS_SEND  ))
		{ // Transmitter empty is not true
            DEBUGMSG(ZONE_THREAD|ZONE_WRITE,(TEXT("CPdd16550::InitXmit! Wait for UART_INT_STATUS_SEND=%x clear.\r\n"), dwUTRState));
            Sleep(5);
            dwTicks +=5;
            
        }*/
   // }
    return TRUE;
}

DWORD   CPdd350Uart::GetWriteableSize()
{
	
      //RETAILMSG(CHECK_MSG, (L"CPdd350Uart GetWriteableSize\r\n"));
      return BULK_BLOCK_BYTE/4;
}
void    CPdd350Uart::XmitInterruptHandler(PUCHAR pTxBuffer, ULONG *pBuffLen)
{
    PREFAST_DEBUGCHK(pBuffLen!=NULL);
    //RETAILMSG(ZONE_UART,(L"uart send len= %d\r\n",*pBuffLen));
    RETAILMSG(CHECK_MSG, (L"CPdd350Uart XmitInterruptHandler\r\n"));
    //m_HardwareLock.Lock();    
    if (*pBuffLen == 0) { 
        EnableXmitInterrupt(FALSE);
    }
    else 
    {
        DEBUGCHK(pTxBuffer);
        PulseEvent(m_XmitFlushDone);	
        DWORD dwDataAvaiable = *pBuffLen;
        *pBuffLen = 0;
        if ((m_DCB.fOutxCtsFlow && IsCTSOff()) ||(m_DCB.fOutxDsrFlow && IsDSROff()))
	 { // We are in flow off
            DEBUGMSG(ZONE_THREAD|ZONE_WRITE,(TEXT("CPdd16550::XmitInterruptHandler! Flow Off, Data Discard.\r\n")));
            EnableXmitInterrupt(FALSE);
        }
        else  
	 {
		if (pDeviceGlobal)
		  {		          

                       DWORD dwWriteSize = GetWriteableSize();
		          DWORD dwByteWrite=0;
		          //RETAILMSG(ZONE_UART,(TEXT("CPdd16550::XmitInterruptHandler! WriteableSize=%x to FIFO,dwDataAvaiable=%x\r\n"),
		           //          dwWriteSize,dwDataAvaiable));
		          {
			           //dwByteWrite = dwDataAvaiable;
				     for (dwByteWrite=0; dwByteWrite<dwWriteSize && dwDataAvaiable!=0;dwByteWrite++) 
                              {
					    //EnterCriticalSection( &pDeviceGlobal->TXcs );
					    //TXWriteByte( pDeviceGlobal,*pTxBuffer);
					    //LeaveCriticalSection( &pDeviceGlobal->TXcs );
					    //vk3214_g_int_dis();
                                    VK3214SendData(pDeviceGlobal, *pTxBuffer);
                                    //vk3214_g_int_en();
                                    pTxBuffer ++;
                                    dwDataAvaiable--;
                             }



		           }
			     *pBuffLen = dwByteWrite;
				EnableXmitInterrupt(TRUE);
//				EnterCriticalSection( &pDeviceGlobal->TXcs );
				pDeviceGlobal->m_serial_flag |= UART_TRANS_DATA;
				
//				LeaveCriticalSection( &pDeviceGlobal->TXcs );
				//Sleep(1);				
				SetEvent(pDeviceGlobal->hComChangedEvent);

		  }

        }
    }
    //m_HardwareLock.Unlock();
}


void    CPdd350Uart::XmitComChar(UCHAR ComChar)
{

      RETAILMSG(CHECK_MSG, (L"CPdd350Uart XmitComChar\r\n"));

 
}
BOOL    CPdd350Uart::EnableXmitInterrupt(BOOL fEnable)
{
    //m_HardwareLock.Lock();
     //RETAILMSG(CHECK_MSG, (L"CPdd350Uart EnableXmitInterrupt\r\n"));
    if (fEnable)
        EnableInterrupt(UART_TXFIFO_INT);  //VK3214EnableTXInterrupt(pDeviceGlobal);       //
    else
        DisableInterrupt(UART_TXFIFO_INT);  //VK3214DisableTXInterrupt(pDeviceGlobal);       //
    //m_HardwareLock.Unlock();
    return TRUE;
        
}
BOOL  CPdd350Uart::CancelXmit()
{
	RETAILMSG(CHECK_MSG, (L"CPdd350Uart CancelXmit\r\n"));
    return InitXmit(TRUE);     
}
static PAIRS s_HighWaterPairs[] = {
    {0, 1},
    {1, 4 },
    {2, 8 },
    {3, 14 }
};

UINT16  CPdd350Uart::GetWaterMarkBit()
{
    BYTE bReturnKey = (BYTE)s_HighWaterPairs[0].Key;
    RETAILMSG(CHECK_MSG,(TEXT("CPdd350Uart GetWaterMarkBit\r\n" )));    
    for (DWORD dwIndex=dim(s_HighWaterPairs)-1;dwIndex!=0; dwIndex --) 
   {
        if (m_dwWaterMark>=s_HighWaterPairs[dwIndex].AssociatedValue) 
	 {
            bReturnKey = (BYTE)s_HighWaterPairs[dwIndex].Key;
            break;
        }
    }
    return bReturnKey;
}
DWORD   CPdd350Uart::GetWaterMark()
{
    BYTE bReturnValue = (BYTE)s_HighWaterPairs[0].AssociatedValue;
	RETAILMSG(CHECK_MSG,(TEXT("CPdd350Uart GetWaterMark\r\n" )));    
    for (DWORD dwIndex=dim(s_HighWaterPairs)-1;dwIndex!=0; dwIndex --) 
   {
        if (m_dwWaterMark>=s_HighWaterPairs[dwIndex].AssociatedValue) 
	 {
            bReturnValue = (BYTE)s_HighWaterPairs[dwIndex].AssociatedValue;
            break;
        }
    }
    return bReturnValue;
}

// Receive
BOOL    CPdd350Uart::InitReceive(BOOL bInit)
{
	RETAILMSG(CHECK_MSG,(TEXT("CPdd350Uart VK3214com INITreceive %d  pDeviceGlobal = %x  pDeviceGlobal->childport= %x *pDeviceGlobal = %x\r\n" ),
	                                                 bInit,pDeviceGlobal,pDeviceGlobal->m_childport, *pDeviceGlobal));
     if (pDeviceGlobal)
    {
        EnterCriticalSection( & pDeviceGlobal->Lock);
        if (bInit)
        {
            UINT16 uWarterMarkBit = GetWaterMarkBit();
            if (uWarterMarkBit> 3)
                uWarterMarkBit = 3;
            uWarterMarkBit++;
            EnableInterrupt(UART_RXFIFO_INT | UART_ERR_INT  );
            pDeviceGlobal->m_serial_setup |= RECIVE_OK;
        }
        else
        {
            DisableInterrupt(UART_RXFIFO_INT | UART_ERR_INT );
            pDeviceGlobal->m_serial_setup &= ~RECIVE_OK;
        }
        //SetEvent(pDeviceGlobal->hComChangedEvent);
        LeaveCriticalSection( & pDeviceGlobal->Lock);
    }
    return TRUE;

}
ULONG   CPdd350Uart::ReceiveInterruptHandler(PUCHAR pRxBuffer,ULONG *pBufflen)
{
    DEBUGMSG(ZONE_THREAD|ZONE_READ,(TEXT("+CPdd350Uart::ReceiveInterruptHandler pRxBuffer=%x,*pBufflen=%x\r\n"),
        pRxBuffer,pBufflen!=NULL?*pBufflen:0));
    DWORD dwBytesDropped = 0;
    if (pRxBuffer && pBufflen )
    {
        DWORD dwBytesStored = 0 ;
        DWORD dwRoomLeft = *pBufflen;
        m_bReceivedCanceled = FALSE;
        while (dwRoomLeft && !m_bReceivedCanceled)
        {
            //  ULONG ulUFSTATE = 0;//m_pReg320Uart->Read_RFCR();
            //   DWORD dwNumRxInFifo = (ulUFSTATE & (0x3f<<0));
            //    DEBUGMSG(ZONE_THREAD|ZONE_READ,(TEXT("CPdd320Uart::ReceiveInterruptHandler ulUFSTATE=%x, dwNumRxInFifo=%X\r\n"),
            //        ulUFSTATE,  dwNumRxInFifo));
            //if (dwNumRxInFifo) {dwNumRxInFifo&& 
            // while ( dwRoomLeft) 
            {
                //UINT16 uLineStatus = GetLineStatus();
                UCHAR uData ;
                /* if (DataReplaced(&uData,!(uLineStatus & (0x1<<12)))) {
                     *pRxBuffer++ = uData;
                     dwRoomLeft--;
                     dwBytesStored++;                    
                 }*/
                 //RETAILMSG(CHECK_MSG,(TEXT("ReceiveInterruptHandler\r\n" )));
		  if(pDeviceGlobal->length !=0)
		  {
                    while(pDeviceGlobal->length !=0)
                    {
                        if (GetByte(pDeviceGlobal,&uData))
                        {
                              *pRxBuffer++ = uData;
                               dwRoomLeft--;
                               dwBytesStored++;         
                         }
                    }
                 }
                else
                {
                    //   pDeviceGlobal->isfifoempty = TRUE;
                    // SetEvent(pDeviceGlobal->hComChangedEvent);
                    break;
                }
                // dwNumRxInFifo --;
            }
            //  }
            //  else
            //      break;
        }
        if (m_bReceivedCanceled)
            dwBytesStored = 0;
        //  if(!pDeviceGlobal->isfifoempty)// simulate interrupt
        //  	SetEvent(m_hISTEvent);


        *pBufflen = dwBytesStored;
    }
    else
    {
        ASSERT(FALSE);
    }

    DEBUGMSG(ZONE_THREAD|ZONE_READ,(TEXT("-CPdd350Uart::ReceiveInterruptHandler pRxBuffer=%x,*pBufflen=%x,dwBytesDropped=%x\r\n"),
        pRxBuffer,pBufflen!=NULL?*pBufflen:0,dwBytesDropped));
    return dwBytesDropped;
}

ULONG   CPdd350Uart::CancelReceive()
{
    m_bReceivedCanceled = TRUE;
    //m_HardwareLock.Lock();   
    RETAILMSG(CHECK_MSG, (L"CPdd350Uart CancelReceive\r\n"));
    InitReceive(TRUE);
    //m_HardwareLock.Unlock();
    return 0;
}

BOOL    CPdd350Uart::InitModem(BOOL bInit)
{
    //m_HardwareLock.Lock();   
    //m_pReg350Uart->Write_MSR( 0x21);
   // m_HardwareLock.Unlock();
    RETAILMSG(CHECK_MSG, (L"CPdd350Uart InitModem = no modem\r\n"));
    return TRUE;
}

ULONG   CPdd350Uart::GetModemStatus()
{
      
    ULONG ulReturn =0 ; 
    //RETAILMSG(CHECK_MSG, (L"CPdd350Uart GetModemStatus\r\n"));
    return ulReturn;
}

BOOL CPdd350Uart::InitLine(BOOL bInit)
{
	//UINT32 dwbit;
    RETAILMSG(CHECK_MSG, (L"CPdd350Uart InitLine\r\n"));
    if  (bInit)
    {

        EnableInterrupt( UART_ERR_INT );
    }
    else {
        DisableInterrupt(UART_ERR_INT );
    }

    return TRUE;
}

UINT16 CPdd350Uart::GetLineStatus()
{
	RETAILMSG(CHECK_MSG, (L"CPdd350Uart GetLineStatus\r\n"));
      return 0;
       
}
void    CPdd350Uart::SetBreak(BOOL bSet)
{
     RETAILMSG(CHECK_MSG, (L"CPdd350Uart SetBreak\r\n"));
}
BOOL    CPdd350Uart::SetBaudRate(ULONG BaudRate,BOOL /*bIrModule*/)
{
    if (pDeviceGlobal)
    {
        EnterCriticalSection( & pDeviceGlobal->Lock);

       /* if (BaudRate == 4800)
        {
            pDeviceGlobal->m_serial_setup |= BAUDRATE_OK;
        }
        else
        {
            pDeviceGlobal->m_serial_setup &= ~BAUDRATE_OK;
        }*/
		
	 pDeviceGlobal->m_serial_setup |= BAUDRATE_OK;	
	 RETAILMSG(1,(TEXT("spicom set baudrate %d pDeviceGlobal = %x " ),BaudRate,pDeviceGlobal));    
	 VK3214ComSetBaudrate( pDeviceGlobal,BaudRate,(BYTE)pDeviceGlobal->m_childport);
      //  SetEvent(pDeviceGlobal->hComChangedEvent);
        LeaveCriticalSection( & pDeviceGlobal->Lock);
    }
   
    return TRUE;
}
BOOL    CPdd350Uart::SetByteSize(ULONG ByteSize)
{
    BOOL bRet = TRUE;
    RETAILMSG(CHECK_MSG,(TEXT("CPdd350Uart SetByteSize \r\n" )));  
    if (pDeviceGlobal)
    {
        EnterCriticalSection( & pDeviceGlobal->Lock);

        switch ( ByteSize )
        {
        case 7:
            pDeviceGlobal->m_serial_setup &= ~DATA_SIZE_OK;
            break;
        case 8:
            pDeviceGlobal->m_serial_setup |= DATA_SIZE_OK;
            break;
        default:
            pDeviceGlobal->m_serial_setup &= ~DATA_SIZE_OK;
            bRet = FALSE;
            break;
        }
        //SetEvent(pDeviceGlobal->hComChangedEvent);
        LeaveCriticalSection( & pDeviceGlobal->Lock);
    }
    return bRet;
}

BOOL    CPdd350Uart::SetParity(ULONG Parity)
{
    RETAILMSG(CHECK_MSG,(TEXT("CPdd350Uart SetParity" ) ));  
    BOOL bRet = TRUE;
    if (pDeviceGlobal)
    {
        EnterCriticalSection( & pDeviceGlobal->Lock);

        switch ( Parity )
        {
        case ODDPARITY:
            pDeviceGlobal->m_serial_setup &= ~PARITY_OK;
            break;
        case EVENPARITY:
            pDeviceGlobal->m_serial_setup &= ~PARITY_OK;
            break;
        case NOPARITY:
            pDeviceGlobal->m_serial_setup |= PARITY_OK;
            break;
        default:
            pDeviceGlobal->m_serial_setup &= ~PARITY_OK;
            bRet = FALSE;
            break;
        }
        //SetEvent(pDeviceGlobal->hComChangedEvent);
        LeaveCriticalSection(& pDeviceGlobal->Lock);

    }
    return bRet;
}
BOOL    CPdd350Uart::SetStopBits(ULONG StopBits)
{
    BOOL bRet = TRUE;
	 RETAILMSG(CHECK_MSG,(TEXT("CPdd350Uart SetStopBits" ) )); 
    if (pDeviceGlobal)
    {
        EnterCriticalSection(& pDeviceGlobal->Lock );

        switch ( StopBits )
        {
        case ONESTOPBIT :
            pDeviceGlobal->m_serial_setup |= STOP_BIT_OK;
            break;
        case TWOSTOPBITS :
            pDeviceGlobal->m_serial_setup &= ~STOP_BIT_OK;
            break;
        default:
            pDeviceGlobal->m_serial_setup &= ~STOP_BIT_OK;
            bRet = FALSE;
            break;
        }
       // SetEvent(pDeviceGlobal->hComChangedEvent);
        LeaveCriticalSection( & pDeviceGlobal->Lock);

    }
    return bRet;
}

DWORD   ComPollThread( LPVOID Context)
{
    PCOM_DEVICE pDevice = (PCOM_DEVICE)( Context);
    DWORD  dwReason;  //dwErr,
    BYTE intStatus;
    
    RETAILMSG(1,(TEXT("ComPollThread enter\r\n" )));  
    if ( !pDevice  )   //|| pDevice != pDeviceGlobal
    {
        DEBUGMSG( ZONE_ERROR, (TEXT("Invalid Context!\n")));
        ExitThread(ERROR_INVALID_PARAMETER);
        return ERROR_SUCCESS;
    }

    pDevice->timeout = NORMAL_SCAN;     // 100;               //INFINITE;   wus 
    pDevice->readindex = 0;
    pDevice->writeindex = 0;
    pDevice->length = 0;
    pDevice->dwIntrtype = 0;
    while (pDevice && !pDevice->DeviceRemoved )
    {
        dwReason = WaitForSingleObject(pDevice->hComChangedEvent,  pDevice->timeout );
/*
        if(dwReason == WAIT_OBJECT_0 )
        {
         	pDevice->timeout = SEND_SCAN;

         }
 */        
	   intStatus = GetVK3214IntStatus(pDevice);		
	   if((intStatus & SIFR_TFINT) != 0  &&  (pDevice->m_serial_flag & UART_TRANS_DATA) != 0 && (pDevice->m_serial_flag &UART_TXFIFO_INT)!= 0)
	   {
	   	       EnterCriticalSection( & pDevice->Lock);	
		       pDevice->dwTXIntrtype |= UART_INT_STATUS_SEND;
		       pDevice->m_serial_flag &=  ~UART_TRANS_DATA;
		       LeaveCriticalSection(&pDevice->Lock);	        
		       if (pDevice->hRecEvent)
                   {
                         SetEvent(pDevice->hRecEvent);
                    }
	       	          
	     }
	 
            EnterCriticalSection(&pDevice->Lock);
	      pDevice->tmplength = 0;
	      VK3214ComIntrPorc(pDevice );
             if (pDevice->OpenCount&&(pDevice->m_serial_setup & ALL_SET_OK) == ALL_SET_OK  && pDevice->length > 0)
            {			
                   pDevice->dwIntrtype |= UART_INT_STATUS_RECEVIE;
                   if (pDevice->hRecEvent)
                   {			   
                          SetEvent(pDevice->hRecEvent);                             
                    }

             }
 /*            
             else if(!(pDevice->m_serial_flag & UART_TRANS_DATA))
             {
             	pDevice->timeout = NORMAL_SCAN;
             }
*/
             LeaveCriticalSection(&pDevice->Lock);          
    	       Sleep(0);

    }
	
    ExitThread(ERROR_SUCCESS);

    return ERROR_SUCCESS;

}




