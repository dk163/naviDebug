/******************************************************************************
 * Copyright (C) u-blox AG
 * u-blox AG, Thalwil, Switzerland
 *
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose without fee is hereby granted, provided that this entire notice
 * is included in all copies of any software which is or includes a copy
 * or modification of this software and in all copies of the supporting
 * documentation for such software.
 *
 * THIS SOFTWARE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
 * WARRANTY. IN PARTICULAR, NEITHER THE AUTHOR NOR U-BLOX MAKES ANY
 * REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE MERCHANTABILITY
 * OF THIS SOFTWARE OR ITS FITNESS FOR ANY PARTICULAR PURPOSE.
 *
 ******************************************************************************
 *
 * Project: libMGA
 * Purpose: Windows specific functions to help a host application to download
 *          MGA assistance data and pass it on to a u-blox GPS receiver.
 *
 *****************************************************************************/

#include <stdio.h>      // Standard input/output definitions
#include <conio.h>      // Console input/output
#include <winsock2.h>   // Socket connection
#include <process.h>    // Threads and processes
#include "tchar.h"
#include <atlbase.h>

///////////////////////////////////////////////////////////////////////////////
extern void DeviceReadThread(void *);

static HANDLE s_hComm = INVALID_HANDLE_VALUE;

///////////////////////////////////////////////////////////////////////////////

void mySleep(int seconds)
{
    Sleep(seconds * 1000);
}

void CleanUpThread()
{
}

bool openSerial(const char* port, unsigned int baudrate)
{
    // convert const char* to LPCTSTR to be used in CreateFile
    CA2T prt(port);

    // change to you preferred COM port
    s_hComm = ::CreateFile(prt,                         // port name
                           GENERIC_READ | GENERIC_WRITE,         // read and write
                           FILE_SHARE_READ | FILE_SHARE_WRITE,   // exclusive access
                           NULL,                                 // no security attributes
                           OPEN_EXISTING,                        // opens, if existing
                           FILE_ATTRIBUTE_NORMAL,                // (no) overlapped I/O
                           NULL);

    if (s_hComm != INVALID_HANDLE_VALUE)
    {
        // printf("Connected to %s @ %d\n", port, baudrate);
        COMMTIMEOUTS timeouts;
        memset(&timeouts, 0, sizeof(timeouts));
        timeouts.ReadIntervalTimeout = MAXDWORD;
        timeouts.ReadTotalTimeoutMultiplier = 0;
        timeouts.ReadTotalTimeoutConstant = 0;
        timeouts.WriteTotalTimeoutMultiplier = 0;
        timeouts.WriteTotalTimeoutConstant = 0;
        SetCommTimeouts(s_hComm, &timeouts);

        DCB m_dcbSetting;
        memset(&m_dcbSetting, 0, sizeof(DCB));
        m_dcbSetting.DCBlength = sizeof(DCB);          // sizeof(DCB)
        m_dcbSetting.BaudRate = baudrate;             // current baud rate            Change to your preferred speed
        m_dcbSetting.fBinary = TRUE;                 // binary mode, no EOF check
        m_dcbSetting.fParity = FALSE;                // enable parity checking
        m_dcbSetting.fOutxDsrFlow = FALSE;                // CTS output flow control
        m_dcbSetting.fOutxCtsFlow = FALSE;                // DSR output flow control
        m_dcbSetting.fDtrControl = DTR_CONTROL_ENABLE;   // DTR flow control type
        m_dcbSetting.fDsrSensitivity = FALSE;                // DSR sensitivity
        m_dcbSetting.fTXContinueOnXoff = FALSE;                // XOFF continues Tx
        m_dcbSetting.fOutX = FALSE;                // XON/XOFF out flow control
        m_dcbSetting.fInX = FALSE;                // XON/XOFF in flow control
        m_dcbSetting.fErrorChar = FALSE;                // enable error replacement
        m_dcbSetting.fNull = FALSE;                // enable null stripping
        m_dcbSetting.fRtsControl = RTS_CONTROL_ENABLE;   // RTS flow control
        m_dcbSetting.fAbortOnError = FALSE;                // abort reads/writes on error
        m_dcbSetting.XonLim = 10;                   // transmit XON threshold
        m_dcbSetting.XoffLim = 10;                   // transmit XOFF threshold
        m_dcbSetting.ByteSize = 8;                    // number of bits/byte, 4-8
        m_dcbSetting.Parity = NOPARITY;             // 0-4=no,odd,even,mark,space
        m_dcbSetting.StopBits = ONESTOPBIT;           // 0,1,2 = 1, 1.5, 2
        m_dcbSetting.XonChar = 0x11;                 // Tx and Rx XON character
        m_dcbSetting.XoffChar = 0x13;                 // Tx and Rx XOFF character
        m_dcbSetting.ErrorChar = 10;                   // error replacement character
        m_dcbSetting.EofChar = 0;                    // end of input character
        m_dcbSetting.EvtChar = 0;                    // received event character
        ::SetCommState(s_hComm, &m_dcbSetting);
        return true;
    }
    else
    {
        printf("Could not connect to %s @ %d\n", port, baudrate);
        return false;
    }
}

void closeSerial()
{
    if (s_hComm != INVALID_HANDLE_VALUE)
        CloseHandle(s_hComm);
}

int readSerial(void* pBuffer, unsigned int size)
{
    DWORD dwNumBytes = 0;
    ::ReadFile(s_hComm, pBuffer, size, &dwNumBytes, NULL);
    return dwNumBytes;
}


int writeSerial(const void* pBuffer, unsigned int size)
{
    DWORD written;
    ::WriteFile(s_hComm, pBuffer, size, &written, NULL);
    return written;
}

void startDeviceReadThread(void)
{
    _beginthread(DeviceReadThread, 0, NULL);
}

