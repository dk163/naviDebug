/******************************************************************************
 *
 * Copyright (C) u-blox AG
 * u-blox AG, Thalwil, Switzerland
 *
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose without fee is hereby granted, provided that this entire notice is
 * included in all copies of any software which is or includes a copy or
 * modification of this software and in all copies of the supporting
 * documentation for such software.
 *
 * THIS SOFTWARE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
 * WARRANTY. IN PARTICULAR, NEITHER THE AUTHOR NOR U-BLOX MAKES ANY
 * REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE MERCHANTABILITY OF
 * THIS SOFTWARE OR ITS FITNESS FOR ANY PARTICULAR PURPOSE.
 *
 ******************************************************************************
 *
 * Project: Android GNSS Driver
 *
 ******************************************************************************
 * $Id: ubx_serial.h 105975 2015-11-13 09:31:03Z marcus.picasso $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/hal/ubx_serial.h $
 *****************************************************************************/

#ifndef __UBX_SERIAL_H__
#define __UBX_SERIAL_H__

#include "std_types.h"
#include <fcntl.h>
#include <malloc.h>
#include <string.h>
#include <sys/select.h>

#define BAUDRATE_TABLE_SIZE 7

#ifdef UBX_SERIAL_EXTENSIVE_LOGGING
# ifndef UBX_SERIAL_EXTENSIVE_LOGGING_W
#  define UBX_SERIAL_EXTENSIVE_LOGGING_W
# endif // UBX_SERIAL_EXTENSIVE_LOGGING_W 
# ifndef UBX_SERIAL_EXTENSIVE_LOGGING_R
#  define UBX_SERIAL_EXTENSIVE_LOGGING_R
# endif // UBX_SERIAL_EXTENSIVE_LOGGING_R
#endif // UBX_SERIAL_EXTENSIVE_LOGGING


#if defined (UBX_SERIAL_EXTENSIVE_LOGGING_W) || defined (UBX_SERIAL_EXTENSIVE_LOGGING_R)
# define UBXSERLOG LOGD
#else
# define UBXSERLOG(...) do { } while(0)
#endif // defined (UBX_SERIAL_EXTENSIVE_LOGGING_W) || defined (UBX_SERIAL_EXTENSIVE_LOGGING_R)

#define I2C_SLAVE 0
#define I2C_RDWR 0

class CSerialPort
{
public: // Functions
    CSerialPort();
    ~CSerialPort();

    bool openSerial(const char * pTty, int ttybaud, int blocksize, TX_READY_CONF_t const *txReadyConf);

    int rselect(int maxFd, fd_set *rfds, struct timeval const *timeout);

    int setbaudrate(int ttybaud);

    int changeBaudrate(char * ptty, int * pBaudrate, const unsigned char * pBuf, int length);

    void baudrateIncrease(int *baudrate) const;

    int retrieveErrors() const;

    void closeSerial();

    bool fdSet(fd_set &rfds, int &rMaxFd) const;

    bool fdIsSet(fd_set &rfds) const;

#if defined (UBX_SERIAL_EXTENSIVE_LOGGING_W) || defined (UBX_SERIAL_EXTENSIVE_LOGGING_R)
    void _LogBufferAsHex(const void *pBuffer, unsigned int size, bool bIsWrite) const;
#endif // defined (UBX_SERIAL_EXTENSIVE_LOGGING_W) || defined (UBX_SERIAL_EXTENSIVE_LOGGING_R)

    ssize_t readSerial(void *pBuffer, unsigned int size) const;

    ssize_t writeSerial(const void *pBuffer, size_t size) const;

	bool isFdOpen(void) const;
	
private: // Functions
    //! Extract the TX-Ready state and disables it on error
    bool i2cIsTxReadySet(void);

    //! Read amount of available byte from the receiver output registers
    ssize_t i2cGetNumAvailBytes(void) const;

    ssize_t i2cRead(void *buf, size_t count) const;

    bool i2cSetFd(fd_set &rfds, int &rMaxFd) const;
    
    void i2cConfigTxReady(TX_READY_CONF_t const *txReadyConf);

    //! Check the value of the provided GPIO number
    int i2cCheckGpioValue(unsigned int hostGpio) const;

    int settermios(int ttybaud, int blocksize);

private: // Variables
    int m_fd;
	bool m_i2c;

    static const int s_baudrateTable[BAUDRATE_TABLE_SIZE];
    static const unsigned int s_i2cRecvAddr;
    static const char s_sysfsGpioDir[];

	//! container of error counters
#if defined serial_icounter_struct
    static struct serial_icounter_struct s_einfo;
#endif

    ssize_t m_i2cDataAvailableInBytes;

    TX_READY_CONF_t m_txReadyConf;
};


#endif /* __UBX_SERIAL_H__ */
