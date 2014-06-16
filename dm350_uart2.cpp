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

#include <windows.h>
#include <types.h>
#include <ceddk.h>

#include <ddkreg.h>
#include <serhw.h>
#include <Serdbg.h>
#include "pdd_dm350_vk.h"
#include <BSP.h>

#include <s3c2440a_uart.h>
// old #include <dm350_clkc_lib.h>

// add
#include <s3c2440a_intr.h>

#include "dm350_vk3214.h"

static void vk3214RxChars(PCOM_DEVICE pDevice);
static void delaytime(int time);

static HANDLE g_hMutexUARTC  = NULL;
DWORD dwUseCountUARTC = 0;
static volatile  PS3C2440A_UART_REG	       v_pUart2Reg	= NULL;	
//-0ld static volatile P_DM350_UART_REG	       v_pUart2Reg	= NULL;

static volatile S3C2440A_INTR_REG		*g_pINTCReg = NULL;
//-old static volatile DM350_INTC_REG			*g_pINTCReg = NULL;

static   int   iMainBaud = VK3214_DEFAULT_BAUD;



PCOM_DEVICE  ppGlobalDevice[4];
//HANDLE     g_hUartHandle[4];
HANDLE     g_hUartIntThread = NULL;
HANDLE     g_hUartIntEvent = NULL;
DWORD     g_sysIntr =SYSINTR_NOP;
//DWORD     g_UartScanTime = INFINITE;

//
// first initalize the uart2 & vk3214
/******************************3214*************************************/

void Dump_u2Reg(void)
{
      RETAILMSG(1, (L"v_pUart2Reg->PWREMU_MGMT = %x \r\n",v_pUart2Reg->PWREMU_MGMT));
	  RETAILMSG(1, (L"v_pUart2Reg->DLL = %x \r\n",v_pUart2Reg->DLL));  
      RETAILMSG(1, (L"v_pUart2Reg->DLH = %x \r\n",v_pUart2Reg->DLH));
	  RETAILMSG(1, (L"v_pUart2Reg->FCR = %x \r\n",v_pUart2Reg->FCR));
	  RETAILMSG(1, (L"v_pUart2Reg->MCR = %x \r\n",v_pUart2Reg->MCR));
	  RETAILMSG(1, (L"v_pUart2Reg->LCR = %x \r\n",v_pUart2Reg->LCR));
	  RETAILMSG(1, (L"v_pUart2Reg->IER = %x \r\n",v_pUart2Reg->IER));

}
void delay(int time)
{
     volatile int i;
     for(i=0;i<time;i++)
     	{
         	;
     	}
}
/**************************************************/

void TestVK3214(void)
{
	InitializeUart2port();
	Dump_u2Reg();
	Sleep(40);
	//Dump_VKReg();
	RETAILMSG(VK3214_DEBUG, (L"++vk global init \r\n"));
	VK3214GlobalInit();
	Sleep(40);
	Dump_VKReg();
	RETAILMSG(VK3214_DEBUG, (L"++vk childserial init \r\n"));
      ChildSerial_Init(0);
	ChildSerial_Init(1);
	ChildSerial_Init(2);
	ChildSerial_Init(3);	
	ChangeMainBaud(VK3214_MAIN_BAUD,v_pUart2Reg);		
       Dump_VKReg();
       Dump_u2Reg();
}   
/**************************************************

void VK3214_ClearReg(void)
{
	VK3214GlobalInit();	
	Dump_VKReg();
	RETAILMSG(VK3214_DEBUG, (L"++vk childserial init \r\n"));
	ChildSerial_Init(0);
	ChildSerial_Init(1);
	ChildSerial_Init(2);
	ChildSerial_Init(3);	
	
}
/**************************************************/
 void Uart2BaudRateSet(int const baudrate,/* old P_DM350_UART_REG*/PS3C2440A_UART_REG v_pUart2base)
 {
	iMainBaud = baudrate;
	//old  v_pUart2base->DLL =  0xff& (UART2_INPUT_CLOCK/16/baudrate) + 1;
	 v_pUart2base->UBRDIV =  0xff& (UART2_INPUT_CLOCK/16/baudrate) + 1;           //divider = 162000000/(16*VK3214_DEFAULT_BAUD)-VK3214_DEFAULT_BAUD bps
	//old  v_pUart2base->DLH =  0xff&((UART2_INPUT_CLOCK/16/baudrate)>>8);
	Sleep(5);
 }

//**************************************************
//功能: 修改 vk3214 主串口波特率
//输入: 波特率写入全局主串口控制寄存器(GMUCR[7:4])
//**************************************************
BOOL ChangeMainBaud(int const baudrate,/* old P_DM350_UART_REG*/ PS3C2440A_UART_REG v_pUart2base)
{	 
	 int i;
	 BYTE baudset;
	 iMainBaud = baudrate;
       if(NULL == v_pUart2base)
       {
            RETAILMSG(VK3214_DEBUG, (L"ChangeMainBaud Fail ! \r\n"));
		return(FALSE);			
       }
	for (i=0; i < 16; i++)
	{
	     if (VKbauddata[i].baudrate == baudrate)
	     {
			baudset = ((VKbauddata[i].setvalue )<< 4);
			break;
		}  
	}
     RETAILMSG(VK3214_DEBUG, (L"baudset is = %x baudrate = %d\r\n",baudset,baudrate ));
	 WriteReg(0,GMUCR,baudset);    
	 Sleep(15);
	  //old v_pUart2base->DLL =  0xff& (UART2_INPUT_CLOCK/16/VKbauddata[i].baudrate) + 1
       v_pUart2base->UBRDIV =  0xff& (UART2_INPUT_CLOCK/16/VKbauddata[i].baudrate) + 1;           //divider = 162000000/(16*VK3214_DEFAULT_BAUD)-VK3214_DEFAULT_BAUD bps
	  // v_pUart2base->DLH =  0xff&((UART2_INPUT_CLOCK/16/VKbauddata[i].baudrate)>>8);
       Sleep(15);
	 return(TRUE);

}


//*****************************************************************************S3C2440A_UART_REG

/************************************************************************/
/*功能：唤醒vk32
/************************************************************************/
BOOL Wakeup_VK3214(/* old P_DM350_UART_REG*/ PS3C2440A_UART_REG v_UartBase)
{
	BYTE   gmucr = GMUCR;
	volatile int iCount = 0;
	BYTE send = 0x80|GCR ;
	int i;
	if(!v_UartBase) 
	{
	     RETAILMSG(1, (L"Wakeup_VK3214 input fail\r\n"));   
	     return(FALSE);
	}

	//set GCR wakeup 3214  	
	
	//while(!(v_UartBase->LSR&(1<<5)));
      // v_UartBase->THR = send;	
       //while(!(v_UartBase->LSR&(1<<5)));
       //v_UartBase->THR = 0;	 
    
	for(i=0;i <40; i++)
	{
		 Sleep(5);
		while(!(v_UartBase->UTRSTAT&(1<<2)));//检查接收发送状态寄存器，UTRSTAT[2]为1说明发送缓冲寄存器为空
       	v_UartBase->UTXH = gmucr;	 //将gmucr 写入发送寄存器，读取全局主串口控制寄存器  
		while(!(v_UartBase->UTRSTAT & 1)&& iCount++ < 200000);//UTRSTAT[0]为1说明接收到数据
		if(iCount<200000)
		{
			gmucr = v_UartBase->URXH >>4 ;//从rx寄存器中读取gmucr的内容，高四位为波特率设置
			if (iMainBaud  == VKbauddata[gmucr].baudrate )
			{
                   		 RETAILMSG(1, (L"Wakeup_VK3214 ok iMainBaud = %d\r\n", iMainBaud));
                    	return(TRUE);
             	}
		    	
		}
	}
	//如果波特率设置尝试设置其他波特率
	RETAILMSG(1, (L"Wakeup_VK3214 time out fail iMainBaud = %d \r\n",iMainBaud));
	
       
	if (iMainBaud  == VK3214_DEFAULT_BAUD)
	{
	      Uart2BaudRateSet(VK3214_MAIN_BAUD, v_UartBase);
	}
	else
	{
		 Uart2BaudRateSet(VK3214_DEFAULT_BAUD, v_UartBase);
	}
	//set GCR wakeup 3214
//	Sleep(10);	     
	for(i=0;i <40; i++)
	{
		 Sleep(5);
		 while(!(v_UartBase->UTRSTAT&(1<<2)));//检查接收发送状态寄存器，UTRSTAT[2]为1说明发送缓冲寄存器为空
		 v_UartBase->UTXH = gmucr;	 //将gmucr 写入发送寄存器，读取全局主串口控制寄存器  
		 while(!(v_UartBase->UTRSTAT & 1)&& iCount++ < 200000);//UTRSTAT[0]为1说明接收到数据
		 if(iCount<200000)
		 {
			 gmucr = v_UartBase->URXH >>4 ;//从rx寄存器中读取gmucr的内容，高四位为波特率设置
			 if (iMainBaud  == VKbauddata[gmucr].baudrate )
			 {
				 RETAILMSG(1, (L"Wakeup_VK3214 ok iMainBaud = %d\r\n", iMainBaud));
				 return(TRUE);
			 }

		 }
	}
	
	 RETAILMSG(1, (L"Wakeup_VK3214  fail, Please check vk3214 !!!  iMainBaud = %d \r\n",iMainBaud));	
	 return(FALSE);

}

BOOL ForemostInitUart(void)
{
	RETAILMSG(VK3214_DEBUG, (L"++ForemostInitUart \r\n"));
	DWORD irq;  
	PHYSICAL_ADDRESS regBase = { UART2_PA_BASE_ADDRESS, 0};
	irq =0x2D;
	if (dwUseCountUARTC == 0) // Not yet initted
      {
		g_hMutexUARTC = CreateMutex(NULL,FALSE,"UARTC_MUTEX_NAME");

		RETAILMSG(1, (L"ForemostInitUart called, Mutex = [0x%X]\r\n",g_hMutexUARTC));

		if (g_hMutexUARTC) // CreateMutex succeeded
            {		
					//映射物理地址
                    v_pUart2Reg =  (PS3C2440A_UART_REG)MmMapIoSpace(regBase, sizeof(S3C2440A_UART_REG),FALSE);
					g_pINTCReg = (S3C2440A_INTR_REG *)MmMapIoSpace(regBase, sizeof(S3C2440A_INTR_REG),FALSE);
			                  
					if (v_pUart2Reg == NULL)
						{
                           ERRORMSG(VK3214_DEBUG, (L"ForemostInitUart : regBase MmMapIoSpace failed!\r\n"));
                           CloseHandle(g_hMutexUARTC);
                           g_hMutexUARTC = NULL;			
                           return(FALSE);
						}
		
			dwUseCountUARTC++;
			InitPortPinMux();	
			Sleep(100);	
			TestVK3214();
			//goto  ret; 			
                   //InitializeUart2port();
			Sleep(10);				
			vk3214_int_set();   						
			//Dump_VKReg();	 
			//if (!Wakeup_VK3214(v_pUart2Reg))
		     // {
			//     goto  ret;   
			//}
			
			g_hUartIntEvent =  CreateEvent( NULL, FALSE, FALSE, NULL);	
                   if ( !g_hUartIntEvent )
                  {
                       RETAILMSG( 1, (TEXT("pDevice->hComChangedEvent CreateEvent error:%d\n"), GetLastError() ));
                       goto  ret;   
                  }
		     if (!KernelIoControl(IOCTL_HAL_REQUEST_SYSINTR, &irq, sizeof(irq), &g_sysIntr,sizeof(g_sysIntr), NULL)) 
	           {   
		             RETAILMSG( 1, (TEXT("VK3214ComAttach KernelIoControl fail \r\n")));
		             goto  ret; 
	            }   		
		      RETAILMSG( 1, (TEXT("VK3214ComAttach KernelIoControl ok \r\n")));
 	           if (!InterruptInitialize(g_sysIntr, g_hUartIntEvent, NULL, 0))	
	           {
	                RETAILMSG( 1, (TEXT("VK3214ComAttach InterruptInitialize fail  g_sysIntr = %x  g_hUartIntEvent =%x\r\n")
	                                 ,g_sysIntr,g_hUartIntEvent));
                       goto  ret;         
	            }  

			 g_hUartIntThread = CreateThread( NULL, 0, 
                                                ComUartIntThread, 
                                                ppGlobalDevice, 
                                                0, NULL );
	              SetThreadPriority(g_hUartIntThread,  THREAD_PRIORITY_TIME_CRITICAL);
			 if ( !g_hUartIntThread )
                    {         
	                   RETAILMSG( 1, (TEXT("++Create ComUartIntThread error:%d\r\n"), GetLastError()));     
	                   goto  ret;   
        	        }
  	       
	              RETAILMSG( 1, (TEXT("VK3214ComAttachg_hUartIntThread  created   \r\n")));

			 //VK3214_ClearReg();			 
			// Sleep(40);
			 //Dump_VKReg();	 
  			//ChangeMainBaud(VK3214_MAIN_BAUD,v_pUart2Reg);
  			 //Sleep(10);
  			 //Dump_VKReg();	 
 
			return(TRUE);
		}
	}
	else
      {	// Already initted !!
		RETAILMSG(VK3214_DEBUG, (L"ForemostInitUart already called,so, just return\r\n"));

		dwUseCountUARTC++;
             //TestVK3214();
		return(TRUE);
	}

	//ERRORMSG(VK3214_DEBUG, (L"ForemostInitUart failed \r\n"));
ret:
	ForemostDeInitUart();
	return(FALSE);


}
//*****************************************************************************
void ForemostDeInitUart(void)
{
	dwUseCountUARTC = 0;
	if(g_hMutexUARTC)
	{
             CloseHandle(g_hMutexUARTC);
             g_hMutexUARTC = NULL;
	
	}
	if (g_hUartIntEvent && g_hUartIntThread)
     {

            SetEvent(g_hUartIntEvent);
            WaitForSingleObject(g_hUartIntThread, INFINITE);
            CloseHandle(g_hUartIntThread);
            g_hUartIntThread = NULL;
	     CloseHandle(g_hUartIntEvent);
		g_hUartIntEvent = NULL;
    }
    CloseHandle(g_hMutexUARTC);
	g_hMutexUARTC = NULL;
	g_sysIntr   =   SYSINTR_NOP; 
}



BOOL VK3214_Grab(void)
{
	if (g_hMutexUARTC)
     {
		WaitForSingleObject(g_hMutexUARTC,INFINITE);
	}
	else
     {
		ERRORMSG(1, (L"VK3214_Grab : Mutex not valid !!\r\n"));

		return(FALSE);
	}

	return(TRUE);
}

//*****************************************************************************
BOOL VK3214_Release(void)
{
	if (g_hMutexUARTC)
     {
		if (!ReleaseMutex(g_hMutexUARTC))
  	{
			ERRORMSG(1, (L"VK3214_Release : ReleaseMutex failed !!\r\n"));

			return(FALSE);
		}
	}
	else
      {
		ERRORMSG(1, (L"VK3214_Release : Mutex not valid !!\r\n"));

		return(FALSE);
	}

	return(TRUE);
}

//*****************************************************************************
BOOL  VK3214ComInit(BYTE dwChildPort ,PCOM_DEVICE pDevice)
{
	BYTE gir,sier,sctlr;
	
	 RETAILMSG(VK3214_DEBUG, (L"++VK3214ComInit\r\n"));
       VK3214_Grab();
	  dwChildPort--; 

	sier =  ReadReg(dwChildPort,SIER);
	
	sier = SIER_RFIEN ;   //| SIER_TRIEN
		
      WriteReg(dwChildPort,SIER,sier);
	  
      sctlr = ReadReg(dwChildPort,SCTLR);
	
	sctlr = SCTLR_UTEN |(0x3<<4 );    //WUS 0309         
	
       WriteReg(dwChildPort,SCTLR,sctlr);

        WriteReg(dwChildPort,SCONR,0);

	WriteReg(dwChildPort,SFOCR,0xff);
	
	WriteReg(dwChildPort,SFOCR,SFOCR_RFTL_2 | SFOCR_TFTL_2 | SFOCR_RFEN | SFOCR_TFEN);      //triger point     14BYTES
	
	gir=ReadReg(0,GIR);
	
	gir|= (0x10<<dwChildPort);

	WriteReg(0,GIR,gir);

      VK3214_Release();
	RETAILMSG(1, (L"VK3214ComInit Dump_VKReg()  \r\n"));	
	Dump_VKReg();	  
	RETAILMSG(1, (L"--VK3214ComInit   \r\n"));	
     return(TRUE);
}










//*****************************************************************************
BOOL InitializeUart2port(void)
{
     //int *pMainBaud = &iMainBaud;

     RETAILMSG(1, (L"++InitializeUart2port called \r\n"));    
      //InitBuf(&rxbuf);
      if(v_pUart2Reg)
     {
          v_pUart2Reg->PWREMU_MGMT = 0x8001;
          v_pUart2Reg->PWREMU_MGMT = 0xe001;
	    //v_pUart2Reg->LCR |= 0x80;     //Bit 7 has to be 1 for setting DLL and DLH
	    v_pUart2Reg->DLL =  0xff& (UART2_INPUT_CLOCK/16/VK3214_DEFAULT_BAUD);       //divider = 162000000/(16*VK3214_DEFAULT_BAUD)-VK3214_DEFAULT_BAUD bps
	    v_pUart2Reg->DLH =  0xff&((UART2_INPUT_CLOCK/16/VK3214_DEFAULT_BAUD)>>8);

          v_pUart2Reg->FCR =   FCR_FIFO_EN ;     
	    v_pUart2Reg->FCR =  (FCR_FIXFTL_2 <<6) | (FCR_RX_EN<<1) | (FCR_TX_EN<<2) | FCR_FIFO_EN;                                                                   /* Enable FIFO,  set trigger level as 14 */
	    v_pUart2Reg->MCR = 0x21;
	    v_pUart2Reg->LCR = LCR_WLS_2;             //set 8bits length
	    v_pUart2Reg->IER = 0x0;                        //disable int
	    //v_pUart2Reg->FCR =   0 ;    
	    //v_pUart2Reg->IER = 0x5;
	    iMainBaud = VK3214_DEFAULT_BAUD;
	    RETAILMSG(1, (L"--InitializeUart2port  \r\n"));
	    return TRUE;
     }
     else
     {
          RETAILMSG(1, (L"InitializeUart2port fail \r\n"));
          return FALSE;  	 
     }

}


inline void   Uart2Write(int data)    
{
	while(!(v_pUart2Reg->LSR&(1<<5)));//send ready
       v_pUart2Reg->THR = data;
}


inline int   Uart2Read(void)    
{
	volatile int iCount = 0;
	while(!(v_pUart2Reg->LSR & 1)&& iCount++ < 250000);
	if(iCount>250000)
	{
	      RETAILMSG(1, (L"Uart2Read time out fail\r\n"));
		return 0xff;
	}
	return (v_pUart2Reg->RBR);
}


void WriteReg(BYTE childSerial,BYTE regindex,BYTE value)
{
	//BYTE i;
      BYTE send = 0x80;
	send |= ((childSerial&0x3)<<4);
	send |= (regindex&0xf);	
      Uart2Write(send);
	Uart2Write(value);
}


BYTE ReadReg(BYTE childSerial,BYTE regindex)
{
	//BYTE i;
      BYTE send = 0x0;
	send |= ((childSerial&0x3)<<4);
	send |= (regindex&0xf);
      Uart2Write(send);
	send = (BYTE)Uart2Read();
    return  send;
}



void VK3214GlobalInit(void)
{
	//setup serila baudrate 9600
      WriteReg(0,GCR,0);
	WriteReg(0,GMUCR,VK_GLOABLE_BAUD<<4);                             //VK_GLOABLE_BAUD
	//setup dm350 serila badurate accordingly
	WriteReg(0,GIR,0xF0); 
	WriteReg(0,GXOFF,0); 
	WriteReg(0,GXON,0); 
       
}

void ChildSerial_Init(int childserial)
{
       if(childserial>3)
       {
       	RETAILMSG(1, (L"ChildSerial_Init Fail childserial= %d \r\n",childserial));
		return;
       }
	   
	  WriteReg(childserial,SCTLR,0x38);     //change baudrate at here
	  WriteReg(childserial,SCONR,0x0);
	  WriteReg(childserial,SFWCR,0x0);
	  WriteReg(childserial,SFOCR,0xf);
	  WriteReg(childserial,SFOCR,0xc);
	  WriteReg(childserial,SIER,0x1);  
}

void Dump_VKReg(void)
{
     int i;
	 for(i=1; i<6;i++)
	 {
	 	RETAILMSG(1, (L"Globalreg Dump_VKReg %d = %x \r\n",i,ReadReg(0,i)));
	 }
       for(i=6; i<16;i++)
	 {
	 	RETAILMSG(1, (L"childserial1 Reg %d = %x \r\n",i,ReadReg(0,i)));
	 }
	 for(i=6; i<16;i++)
	 {
	 	RETAILMSG(1, (L"childserial2 Reg %d = %x \r\n",i,ReadReg(1,i)));
	 }

       for(i=6; i<16;i++)
	 {
	 	RETAILMSG(1, (L"childserial3 Reg %d = %x \r\n",i,ReadReg(2,i)));
	 }
	 for(i=6; i<16;i++)
	 {
	 	RETAILMSG(1, (L"childserial4 Reg %d = %x \r\n",i,ReadReg(3,i)));
	 }  
	 
 
}
VOID VK3214SendData(PCOM_DEVICE pDevice, UCHAR TxData)
{
    BYTE ChildPort;
    
    if( !v_pUart2Reg)
    {
         RETAILMSG(1, (L"VK3214SendData FAIL \r\n"));  
          return;
    }
    
    ChildPort = (BYTE) pDevice->m_childport -1;
    VK3214_Grab();
    WriteReg(ChildPort,SFDR,(BYTE)TxData);
    VK3214_Release();
}

/*
void SendUart2(UINT8 *buf, DWORD size)
{
     //MYAssert(buf);
     DWORD i;
    if(!buf || !v_pUart2Reg)
    {
           return;
    }
    for(i = 0; i < size ; i++)
    {
        while(!(v_pUart2Reg->LSR&(1<<5))); //send ready
	  v_pUart2Reg->THR = buf[i];
    }	 
	
}

//void ReceiveUart2(UINT8 *buf, DWORD size)

void ReceiveUart2()
{
	//BYTE receive;
	//BYTE frame[8];
	//DWORD i;
      if(v_pUart2Reg->LSR&(LSR_RXFIFO_ERR<<7))//rx fifo error
      {
          RETAILMSG(1, (TEXT(" UART2 rx fifo error\r\n")));
      }

      if(v_pUart2Reg->LSR&(LSR_ERR_CASE<<1))//rx fifo error
      {
          RETAILMSG(1, (TEXT(" UART2 rx fifo error lsr is %x \r\n"),v_pUart2Reg->LSR));
      }
  
      while(v_pUart2Reg->LSR&(1<<0))
      {
          UartInqueue(&rxbuf,(BYTE)v_pUart2Reg->RBR);
      }

}

//*/

void  VK3214ComIntrPorc(PCOM_DEVICE pDevice)
{
	//BYTE sifr,gir,sier;
	//BYTE ChildPort ;
	//UINT32 i;
	
	//if(!v_pUart2Reg)
	//	return;
	//RETAILMSG(1,(L"VK3214ComIntrPorc \r\n"));    
	 //ChildPort =(BYTE) pDevice->m_childport -1;
	//gir=vk3xxx_read_reg(USEPORT,VK32XX_GIR);
	//VK3214_Grab();
      vk3214RxChars(pDevice);	
	//VK3214_Release();
/*
	 gir=ReadReg(0,GIR);
	
  //     RETAILMSG(1,(L"INTR GIR = 0x%x\r\n",gir));
	if(!(gir & (GIR_U1IF<<ChildPort)))//这个子通道没有中断
	{
		//RETAILMSG(1,(L"VK3214ComIntrPorc  child port no int  gir =  \r\n",gir));    
		
		vk3214RxChars(pDevice);	

		VK3214_Release();
		return;
	}
	gir &=  ~ (GIR_U1IF<<ChildPort);
		
	//sifr=vk3xxx_read_reg(USEPORT,VK32XX_SIFR);//读取状态判断是哪里产生的中断。对vk32读取SIFR
	sifr=ReadReg(ChildPort,SIFR);

	//sier=vk3xxx_read_reg(USEPORT,VK32XX_SIER);//读取状态判断是哪里产生的中断。对vk32读取SIFR
	sier=ReadReg(ChildPort,SIER);

	// RETAILMSG(1,(L"INTR SIFR = 0x%x\r\n",sifr));

	do 
	{
		if (sifr&SIFR_RFINT)
		{
		       vk3214RxChars(pDevice);
			
		}

		if ((sifr & SIFR_TFINT)&&(sier & SIER_TRIEN))
		{
			//vk32xx_tx_chars(info);
		}
		//sifr=vk3xxx_read_reg(USEPORT,VK32XX_SIFR);
		   sifr = ReadReg(ChildPort,SIFR);	
		
		//sier=vk3xxx_read_reg(USEPORT,VK32XX_SIER);//读取状态判断是哪里产生的中断。对vk32读取SIFR
		  sier= ReadReg(ChildPort,SIER);

	       RETAILMSG(1,(L"INTR SIFR = 0x%x\r\n",sifr));

	} while ((sifr & SIFR_RFINT)||((sifr & SIFR_TFINT)&&(sier & SIER_TRIEN)));  //
  */
/*
	for(i=0; i<pDevice->tmplength;i++)
	{
		RETAILMSG(1,(L"pDevice->tempbuf[i] = %x \r\n",pDevice->tempbuf[i])); 
	
	}
*/	
/*	
	//-------------------------
	//clear interrupt
	sifr &= ~SIFR_RFINT;
	WriteReg(ChildPort,SIFR,sifr);
	WriteReg(0,GIR,gir);
	VK3214_Release();
//*/		
      //RETAILMSG(1,(L"--VK3214ComIntrPorc \r\n")); 
}
//*

static BOOL WriteByte(PCOM_DEVICE pDevice, BYTE in)
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




#if 0
 static void vk3214RxChars(PCOM_DEVICE pDevice)
 {
	BYTE ssr,ChildPort;
	unsigned int ch;
      ChildPort = (BYTE)pDevice->m_childport - 1;
	//ssr=vk3xxx_read_reg(USEPORT,VK32XX_SSR);
	ssr=ReadReg(ChildPort,SSR);
	
	while (!(ssr& SSR_RFEM))
	{//接收FIFO不空
		 //得到一个字符的数据
		ch =ReadReg(ChildPort,SFDR);		

		if (ssr&(SSR_OE|SSR_FE|SSR_PE))
		{//是否有错
		    RETAILMSG(1,(L"serial receive error ssr = %x \r\n",ssr));    	
		}
		else
		{
		     // RETAILMSG(1,(TEXT("%c" ),ch));
		       WriteByte(pDevice, ch);
		       //if(pDevice->tmplength < 64)
		       //{
			 //   pDevice->tempbuf[pDevice->tmplength++] = ch;
				
		       //}
		} 
		//得到更新的状态
		ssr=ReadReg(ChildPort,SSR);	
	}

	   //RETAILMSG(1,(L"pDevice->tmplength =%d,ssr =%x\r\n",pDevice->tmplength,ssr));   

}
 
 //*/
#else

void ClearReceiveFIFO(PCOM_DEVICE pDevice)
{
       BYTE sfocr;
       sfocr = ReadReg((BYTE)pDevice->m_childport-1,SFOCR);
       sfocr |= 1;
       //WriteReg(ChildPort,SFWCR,0xc8);
	//vk3xxx_write_reg(USEPORT,VK32XX_SFOCR,0xfc);
	//WriteReg(ChildPort,SFOCR,0xfc);
        WriteReg((BYTE) pDevice->m_childport-1, SFOCR,   sfocr);  //
}

 static void vk3214RxChars(PCOM_DEVICE pDevice)
 {
	BYTE ssr,ChildPort,sfsr;
	//int i;
      ChildPort = (BYTE)pDevice->m_childport - 1;
	//ssr=ReadReg(ChildPort,SSR);
	//if (ssr&(SSR_OE|SSR_FE|SSR_PE))
	//{//是否有错
	//	ClearReceiveFIFO(pDevice);
	//	RETAILMSG(1,(L"serial receive error ssr = %x \r\n",ssr));    
	//	return ;
	//}
	VK3214_Grab();

	ssr=ReadReg(ChildPort,SSR);
	if(!(ssr& SSR_RFEM))
	{
		sfsr = ReadReg(ChildPort,SFSR);
		ReadFIFO(pDevice,sfsr&0x0f);	
	}
		
		ssr=ReadReg(ChildPort,SSR);
		while(!(ssr& SSR_RFEM))
		{//接收FIFO不空
		 	sfsr = ReadReg(ChildPort,SFSR);
		 	if((sfsr&0x0f) != 0 && (sfsr&0x0f) < 4)
		 	{
		 		break;
		 	}
	        	ReadFIFO(pDevice,sfsr&0x0f);	
	        	ssr=ReadReg(ChildPort,SSR);
		}

	VK3214_Release();
}
//*/
#endif

static void delaytime(int time)
{
	volatile int iCount ;
	while(time--)
	{
	     iCount =0;
	}
}
VOID ReadFIFO(PCOM_DEVICE pDevice, BYTE Length)
{
     //unsigned int ch;
     BYTE send = 0x40;
     BYTE ChildPort = (BYTE)pDevice->m_childport-1;
     	volatile int iCount = 0;
	//RETAILMSG(1, (L"ReadFIFO Length = %d\r\n",Length));
	send |= (ChildPort<<4);
	send |= ((Length-1)&0x0f);
      Uart2Write(send);
	delaytime(30000);   //wus	
      while(v_pUart2Reg->LSR&(1<<0))
      {	    
          WriteByte(pDevice, v_pUart2Reg->RBR); 
      }
     

}
 
void VK3214ComSetBaudrate( PCOM_DEVICE pDevice,DWORD BaudRate,BYTE ChildPort)
{
	BYTE sctlr,old_sier;
	int i;
	
	ChildPort --;
	RETAILMSG(1, (TEXT(" VK3214ComSetBaudrate ChildPort = %d  dwUseCountUARTC = %x\r\n"), ChildPort,dwUseCountUARTC));
	if(!v_pUart2Reg)
	{
		RETAILMSG(1, (TEXT(" VK3214ComSetBaudrate  fail \r\n")));
		return;
	}
	VK3214_Grab();	
	
	old_sier = ReadReg(ChildPort,SIER);
	
	WriteReg(ChildPort,SIER,old_sier & (~(SIER_TRIEN| SIER_RFIEN)));
	
	
	sctlr=ReadReg(ChildPort,SCTLR);
	
	WriteReg(ChildPort,SCTLR,sctlr&(~SCTLR_UTEN));
		
	/* set the parity, stop bits and data size */
       WriteReg(ChildPort,SCONR,0);	
       
       WriteReg(ChildPort,SFOCR,0xff);
       //WriteReg(ChildPort,SFWCR,0xc8);

	//WriteReg(ChildPort,SFOCR,0xfc);
        WriteReg(ChildPort, SFOCR,  SFOCR_RFTL_2|SFOCR_RFEN | SFOCR_TFEN);  //SFOCR_TFTL_2 |SFOCR_RFTL_2|
	sctlr = (SCTLR_UTEN|(0x3<<4));     // 2: default baudrate 19200 bps
	
	for (i=0; i < 16; i++)
	{
	     if (VKbauddata[i].baudrate == BaudRate)
	     {
			sctlr = (SCTLR_UTEN|((VKbauddata[i].setvalue )<< 4));
			break;
		}  
	}
	RETAILMSG(1, (TEXT(" VK3214ComSetBaudrate = %d  setvalue = %x  \r\n"), BaudRate, sctlr));

//恢复正常操作
	WriteReg(ChildPort,SIER,old_sier);	
	
	WriteReg(ChildPort,SCTLR,sctlr);	
	//Dump_VKReg();
	VK3214_Release();
       RETAILMSG(1, (TEXT(" --VK3214ComSetBaudrate  \r\n")));


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
#endif
#if SET_TXBUFF

VOID WriteFIFO(PCOM_DEVICE pDevice, BYTE Length)
{


}
VOID VK3214SendDataFIFO(PCOM_DEVICE pDevice)
{
	BYTE   temp;
	BYTE   ssr,sfsr,length;
	BYTE  count =0;
	//VK3214_Grab();	
	 //RETAILMSG(1, (TEXT("VK3214SendDataByte   \r\n")));

      while((pDevice->tx_length !=0))  //&& ((temp & (1<<1)) == 0)
      {
 		
 		
	       VK3214_Grab();	       
	       ssr = ReadReg(pDevice->m_childport-1,SSR);
		if(ssr & SSR_TFFL)
		{
		     VK3214_Release();
		     return;
		}	       
	       //length =  sfsr>>4;
		VK3214_Release();
			
		//RETAILMSG(1, (TEXT("wesdfasdf   \r\n")));
             if (TXGetByte(pDevice,&temp))
             {
			//RETAILMSG(1, (TEXT("VK3214SendData   \r\n")));
			VK3214SendData(pDevice, temp);    				
              }
		temp = v_pUart2Reg->LSR;
	      if((temp & (1<<1)) != 0)
	       RETAILMSG(1, (TEXT("VK3214SendDataByte full  \r\n")));


              
      }

      //VK3214_Release();
}

VOID VK3214SendDataByte(PCOM_DEVICE pDevice)
{
	BYTE   temp;
	BYTE   ssr,sfsr,length;
	BYTE    count =0;
	//VK3214_Grab();	
	 //RETAILMSG(1, (TEXT("VK3214SendDataByte   \r\n")));

      while((pDevice->tx_length !=0))  //&& ((temp & (1<<1)) == 0)
      {
 		
		
	       VK3214_Grab();	       
	       ssr = ReadReg(pDevice->m_childport-1,SSR);
		if(ssr & SSR_TFFL)
		{
		     VK3214_Release();
		     return;
		}	       
	       //length =  sfsr>>4;
	      // if
		VK3214_Release();
			

             if (TXGetByte(pDevice,&temp))
             {

			VK3214SendData(pDevice, temp);    				
              }
		temp = v_pUart2Reg->LSR;
	      if((temp & (1<<1)) != 0)
	       RETAILMSG(1, (TEXT("VK3214SendDataByte full  \r\n")));
              
      }

      //VK3214_Release();
}
#endif
DWORD VK3214FIFOCheck(PCOM_DEVICE pDevice)
{
         DWORD temp =0;
         BYTE   sifr;
	  VK3214_Grab();	
	  sifr = ReadReg((BYTE)pDevice->m_childport-1,SIFR);	
	  temp = sifr & SIFR_TFINT;	 
        VK3214_Release();
	  return (temp);
	
}
VOID VK3214EnableTXInterrupt(PCOM_DEVICE pDevice)
{
    BYTE sier;
    VK3214_Grab();	
    sier = ReadReg((BYTE)pDevice->m_childport-1,SIER);	
    sier |= SIER_TRIEN;
    WriteReg((BYTE)pDevice->m_childport-1,SIER,sier);
    VK3214_Release();
}


VOID VK3214DisableTXInterrupt(PCOM_DEVICE pDevice)
{
    BYTE sier;
    VK3214_Grab();	
    sier = ReadReg((BYTE)pDevice->m_childport-1,SIER);	
    sier &= ~SIER_TRIEN;
    WriteReg((BYTE)pDevice->m_childport-1,SIER,sier);
    VK3214_Release();

}


BYTE GetVK3214IntStatus(PCOM_DEVICE pDevice)
{
	BYTE sifr;
	VK3214_Grab();	
	sifr = ReadReg((BYTE)pDevice->m_childport-1,SIFR);
	VK3214_Release();
	return (sifr);	
}


void  vk3214_g_int_dis(void)
{
	g_pINTCReg->EINT1 &= ~(1 << (45 - 32));
     		
}
void  vk3214_g_int_en(void)
{
    g_pINTCReg->EINT1 |= (1 << (45- 32));
}



void  vk3214_int_set(void)       //GPIO1 for interrupt set it
{
 if (!GIO_InitLibrary())
   {
	ERRORMSG(1, (L"serial3 : GIO_Init failed!\r\n"));
	return;		// failed
    }

	   GIO_Grab();
         if (!GIO_SetDirection(GIO_PORT_1, GIO_INPUT, FALSE) )
         {
		RETAILMSG(1, (TEXT(" vk3214_int_set SetDirection fail \r\n")));
		   
          }
	   if ( !GIO_SetIRQPort(GIO_PORT_1,GIO_IRQ_FALLING_EDGE))
	   {
	   	    RETAILMSG(1, (TEXT(" vk3214_int_set SetIRQPort fail \r\n")));
	   }
         if ( GIO_SetBankInt(GPIO_BANK0))
	   {
	   	RETAILMSG(1, (TEXT(" vk3214_int_set SetBankInt fail \r\n")));
	   }

	   
/*	   
	   if (!GIO_SetDirection(GIO_PORT_1, GIO_INPUT, FALSE) 
	   	|| !GIO_SetIRQPort(GIO_PORT_1,GIO_IRQ_FALLING_EDGE)
             || GIO_SetBankInt(GPIO_BANK0))
	   {
	   	RETAILMSG(1, (TEXT(" vk3214_int_set fail \r\n")));
	   }
*/	   
	   GIO_Release();




}



DWORD   ComUartIntThread(LPVOID Context)
{
     //HANDLE *pUartHandle = (HANDLE *)Context;
     PCOM_DEVICE *ppGlobalDec = (PCOM_DEVICE *)Context;
     DWORD  dwReason;  //dwErr,
     BYTE intStatus,temp;
     //BYTE ssr[4];
     
     RETAILMSG(1,(TEXT("ComUartIntThread enter\r\n" )));  
     if(!ppGlobalDec && ppGlobalDec != ppGlobalDevice)
    {
        DEBUGMSG( ZONE_ERROR, (TEXT("Invalid Context!\n")));
        ExitThread(ERROR_INVALID_PARAMETER);
        return ERROR_SUCCESS;
    }
    while(dwUseCountUARTC !=0)
    {
 	   dwReason = WaitForSingleObject(g_hUartIntEvent, INFINITE);
 
		VK3214_Grab();	
		intStatus = ReadReg(0,GIR);	
		VK3214_Release();
 		temp =   intStatus>>4 ;
 		intStatus &= temp;  
 	 	//RETAILMSG(1,(TEXT("ComPollThread g_hUartIntEvent %x\r\n" ),intStatus));  
		if( ((intStatus & 1) !=0)  && ppGlobalDec[0]->hComChangedEvent !=NULL)
		{
		      SetEvent(ppGlobalDec[0]->hComChangedEvent);	     
      	 	}
       	else if((intStatus & 2) !=0 && ppGlobalDec[1]->hComChangedEvent !=NULL)
		{

	      		SetEvent(ppGlobalDec[1]->hComChangedEvent);   		
       	}
		else if((intStatus & 4) !=0 && ppGlobalDec[2]->hComChangedEvent !=NULL)
		{

	      		SetEvent(ppGlobalDec[2]->hComChangedEvent);	      		
       	}
       	else if((intStatus & 8) !=0 && ppGlobalDec[3]->hComChangedEvent !=NULL)
		{

	     	 	SetEvent(ppGlobalDec[3]->hComChangedEvent);	     	 	
       	}
    		if(WAIT_OBJECT_0 == dwReason)
    		{
			InterruptDone(g_sysIntr); 
		}

   
    }

    ExitThread(ERROR_SUCCESS);
    return ERROR_SUCCESS;
}






///******************************3214*************************************/
//                       From now on, It's for uart
///******************************3214*************************************/


