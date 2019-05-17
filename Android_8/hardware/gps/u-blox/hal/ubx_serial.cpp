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
 * $Id: ubx_serial.cpp 114244 2016-05-02 14:55:52Z fabio.robbiani $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/hal/ubx_serial.cpp $
 *****************************************************************************/
/*!
  \file
  \brief  Serial utility functions

  Utility for accessing the serial port in a easier way
*/

#include "ubx_serial.h"
#include "../agnss/helper/helperFunctions.h"

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <ctype.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <assert.h>
#include <poll.h>

#ifdef UBX_SERIAL_EXTENSIVE_LOGGING
# ifndef UBX_SERIAL_EXTENSIVE_LOGGING_W
#  define UBX_SERIAL_EXTENSIVE_LOGGING_W
# endif // UBX_SERIAL_EXTENSIVE_LOGGING_W 
# ifndef UBX_SERIAL_EXTENSIVE_LOGGING_R
#  define UBX_SERIAL_EXTENSIVE_LOGGING_R
# endif // UBX_SERIAL_EXTENSIVE_LOGGING_R
#endif // UBX_SERIAL_EXTENSIVE_LOGGING


#if defined (ANDROID_BUILD)
#include <termios.h>
#else
#include <sys/termios.h>
#endif

#if defined serial_icounter_struct
#include <linux/serial.h>
#endif
#include <linux/i2c.h>
#if (PLATFORM_SDK_VERSION >= 21 /* >= 5.0 */ )
#include <linux/i2c-dev.h>
#endif

#include "std_types.h"
#include "std_lang_def.h"
#include "std_macros.h"
#include "ubx_log.h"
const char CSerialPort::s_sysfsGpioDir[] = "/sys/class/gpio";
const unsigned int CSerialPort::s_i2cRecvAddr = 0x42;
const int CSerialPort::s_baudrateTable[BAUDRATE_TABLE_SIZE] =
{
    4800,
    9600,
    19200,
    38400,
    57600,
    115200,
    230400,
};

CSerialPort::CSerialPort()
{
    m_fd = -1;
    m_i2c = false;
    m_i2cDataAvailableInBytes = 0;
    memset(&m_txReadyConf, 0, sizeof(m_txReadyConf));
}

CSerialPort::~CSerialPort()
{
}

bool CSerialPort::openSerial(const char * pTty, int ttybaud, int blocksize, TX_READY_CONF_t const *txReadyConf)
{
    UBXSERLOG("CSerialPort::openSerial: pTty = %s, ttybaud = %d, blocksize = %d", pTty, ttybaud, blocksize);

    closeSerial(); // Make sure the serial interface is closed
    m_i2c = ( strncmp(pTty, "/dev/i2c", 8) == 0 );
    m_fd = open(pTty, (m_i2c ? 0 : O_NOCTTY)
              | O_RDWR
#ifdef O_EXLOCK                 /* for Linux */
              | O_EXLOCK
#endif
        );
    if (m_fd < 0)
    {
        UBX_LOG(LCAT_ERROR, "Cannot open %s port %s, return %d: '%s'", m_i2c?"i2c":"serial", pTty, m_fd, strerror(errno));
        return false;
    }

    if (m_i2c)
    {
        int io = ioctl(m_fd, I2C_SLAVE, s_i2cRecvAddr);
        unsigned char reg = 0xFF;
        if (io < 0)
        {
            UBX_LOG(LCAT_ERROR, "Accessing the i2c device failed: %s", strerror(errno));
            close (m_fd);
            m_fd = -1;
            return false;
        }
        UBX_LOG(LCAT_VERBOSE, "Configured driver to access the receiver over I2C %s on address 0x%02X", pTty, s_i2cRecvAddr);

        i2cConfigTxReady(txReadyConf);

        ssize_t reg_wr_res = write(m_fd, &reg, 1);
        if (reg_wr_res != 1)
        {
            UBX_LOG(LCAT_ERROR, "Writing the receiver register 0x%02X over I2C failed (return value: %i): %s", reg, reg_wr_res, strerror(errno));
            close (m_fd);
            m_fd = -1;
            return false;
        }
    }
    else
    {
        if (settermios(ttybaud, blocksize) == -1)
        {
            UBX_LOG(LCAT_ERROR, "Cannot invoke settermios: '%s'", strerror(errno));
            close (m_fd);
            m_fd = -1;
            return false;
        }
#ifdef BIDIRECTIONAL
        tcflush(m_fd, TCIOFLUSH);
#else
        tcflush(m_fd, TCIFLUSH);
#endif
        fcntl(m_fd, F_SETFL, 0);

#if defined serial_icounter_struct
        /* Initialize the error counters */
        if (!ioctl(m_fd,TIOCGICOUNT, &einfo))
        {
            /* Nothing to do... */
        }
        else
        {
            /* error, cannot read the error counters */
            UBX_LOG(LCAT_WARNING, "unable to read error counters!!!");
        }
#endif
    }

    UBX_LOG(LCAT_VERBOSE, "%s port opened, fd = %d", m_i2c?"I2C":"Serial", m_fd);

    return true;
}

int CSerialPort::rselect(int maxFd, fd_set *rfds, struct timeval const *timeout)
{
    if (m_fd < 0 || !rfds || !timeout)
    {
        UBX_LOG(LCAT_ERROR, "Cannot use rselect, port not open or timeout undefined!");
        return -1;
    }
    int res = 0;

    if(m_i2c)
    {
        fd_set rfds_copy;

        rfds_copy = *rfds;

        // Define I2C Polling frequency
        unsigned long i2c_poll_freq_hz = 20;

        // Interval of wakening
        struct timeval interval;
        interval.tv_sec = 0;
        interval.tv_usec = 1000000 / i2c_poll_freq_hz;

        // Wakeups before timing out
        unsigned long maxWakeups =   i2c_poll_freq_hz * timeout->tv_sec
                                 + ( i2c_poll_freq_hz * timeout->tv_usec ) / 1000000;

        // If timeout->tv_sec and timeout->tv_usec is 0, iterate through the loop below
        // once to make sure something is actually checked for data.
        if(maxWakeups == 0)
            maxWakeups=1;

        for (unsigned long i=0; i < maxWakeups && res == 0; i++)
        {
            // By default use the normal interval defined above
            struct timeval currInterval;
            memcpy(&currInterval, &interval, sizeof(currInterval));


            // Reset the rfds to the original value (code below will change it)
            *rfds = rfds_copy;

            // Check first if I2C data is available using TX-Ready (if available)
            // and polling the corresponding registers in the receiver
            bool i2c_has_data = false;
            if(i2cIsTxReadySet())
            {
                m_i2cDataAvailableInBytes = i2cGetNumAvailBytes();
                i2c_has_data = ( m_i2cDataAvailableInBytes > 0 );
            }

            // If I2C has data, make sure that we don't sleep in select()
            // (we just poll and immediately return, sleeping does not make
            // sense anymore if we already know there is data available)
            if (i2c_has_data)
                memset(&currInterval, 0, sizeof(currInterval));

            // Get the status of the other (non-I2C) file descriptors
            res = select(maxFd, rfds, NULL, NULL, &currInterval);
            if (res < 0) // In case of error let the caller handle this problem
                return res;
            
            // Set the I2C flag in the fd_set if necessary and increase
            // res by one to reflect the data available in the I2C interface
            if (i2c_has_data)
            {
                ++res;
                i2cSetFd(*rfds, maxFd);
            }
            
        }
 
    }
    else
    {
        struct timeval currInterval;
        memcpy(&currInterval, timeout, sizeof(currInterval));
        res = select(maxFd, rfds, NULL, NULL, &currInterval);
    }
    return res;
}

int CSerialPort::setbaudrate(int ttybaud)
{
    return settermios(ttybaud, -1);
}

int CSerialPort::changeBaudrate(char * pTty, int * pBaudrate, const unsigned char * pBuf, int length)
{
    unsigned long newbaudrate = 0;


    if (m_i2c)
    {
        *pBaudrate = (int)newbaudrate;
        return 1;
    }
    if (length != 28)      return 0;
    if (pBuf[0] != 0xb5)  return 0;
    if (pBuf[1] != 0x62)  return 0;
    if (pBuf[2] != 0x06)  return 0;
    if (pBuf[3] != 0x00)  return 0;
    if (pBuf[4] != 0x14)  return 0;
    if (pBuf[5] != 0x00)  return 0;
    if ((pBuf[6] != 0x01) && (pBuf[6] != 0xff))  return 0;

    memcpy(&newbaudrate, pBuf+8+6,4);

    if (newbaudrate != (unsigned long) *pBaudrate)
    {
        int tmp = *pBaudrate;

        close (m_fd);

        UBX_LOG(LCAT_WARNING, "Attempting to change baudrate from %d to %lu on %s)",tmp,newbaudrate, pTty);

        /* Wait a while... */
        usleep(100000);

        if (!openSerial(pTty, (int) newbaudrate, 2, NULL))
        {
            // failed ?!?
            UBX_LOG(LCAT_ERROR, "%s: %s", pTty, strerror(errno));
            openSerial(pTty, *pBaudrate, 2, NULL);
            return 0;
        }
        else 
        {

            *pBaudrate = (int)newbaudrate;
            UBX_LOG(LCAT_WARNING, "Changed baudrate from to %i on %s)",*pBaudrate, pTty);
            return 1;
        }
    }

    return 0;
}

void CSerialPort::baudrateIncrease(int *pBaudrate) const
{
    int i;
    int oldBaudrate = *pBaudrate;

    /* check next baudrate */
    for (i = 0; i< BAUDRATE_TABLE_SIZE; i++)
    {
        if ( *pBaudrate < s_baudrateTable[i])
        {
            *pBaudrate = s_baudrateTable[i];
            break;
        }
    }
    if (i == BAUDRATE_TABLE_SIZE)
    {
        *pBaudrate = s_baudrateTable[0];
    }

    UBX_LOG(LCAT_WARNING, "Attempting to change baudrate from %d to %d", oldBaudrate, *pBaudrate);

    return;
}

int CSerialPort::retrieveErrors() const
{
    int res = 0;
    if (m_i2c)
        return res;
#if defined serial_icounter_struct
    struct serial_icounter_struct einfo_tmp;

    /* read the counters */
    if (!ioctl(m_fd,TIOCGICOUNT, &einfo_tmp))
    {
        /* check if something has changed */
        if (einfo_tmp.frame != einfo.frame)
        {
            UBX_LOG(LCAT_WARNING, "Frame error occurred!!!! (%d)",einfo_tmp.frame - einfo.frame);

            /* return the number of frame errors */
            res = einfo_tmp.frame - einfo.frame;
        }

        else if (einfo_tmp.brk != einfo.brk)
        {
            UBX_LOG(LCAT_WARNING, "Breaks occurred!!!! (%d)",einfo_tmp.brk - einfo.brk);

            /* return the number of frame errors */
            res = einfo_tmp.brk - einfo.brk;
        }

        /* update the stored counters */
        memcpy(&einfo, &einfo_tmp, sizeof(einfo));
    }
    else
    {
        /* cannot read the counters! ignore, as we might be talking to a device without error counters */
    }
#endif
    return res;
}

void CSerialPort::closeSerial()
{
    if (m_fd > 0)
        close(m_fd);
    m_fd = -1;
    m_i2c = false;
}

/*! Extract the TX-Ready state and disables the TX-Ready feature if an error
    occured while accessing it.

    \return                  false if the TX Ready is not indicating data
                             available. true if TX Ready indicates that
                             data is ready or if the TX Ready feature
                             is disabled.
*/
bool CSerialPort::i2cIsTxReadySet(void)
{
    bool result = true; // If TX-Ready feature is disabled, assume TX-Ready is always on
    if(m_txReadyConf.enabled)
    {
        switch(i2cCheckGpioValue(m_txReadyConf.hostGpio))
        {
            case 0:
            {
                result = false;
                break;
            }
            case 1:
            {
                result = true;
                break;
            }
            default:
            {
                UBX_LOG(LCAT_ERROR, "I2C: An invalid value was detected as TX-Ready state. Disabling TX-Ready.");
                memset(&m_txReadyConf, 0, sizeof(m_txReadyConf));
                result = true;
                break;
            }
        }
    }

    return result;
}

/*! Get the number of bytes available on the receiver connected by I2C

    \return                  The number of bytes available for reading
                             on success and -1 on error
*/
ssize_t CSerialPort::i2cGetNumAvailBytes(void) const
{
    if (m_fd <= 0)
        return -1;

    struct i2c_rdwr_ioctl_data {
        struct i2c_msg __user *msgs;
        __u32 nmsgs;
    };
    struct i2c_rdwr_ioctl_data rdwr;
    struct i2c_msg out_in[2], *out, *in;
    unsigned char out_data;
    unsigned char in_data[2];
    ssize_t result = -1;
        
    out = out_in;
    in = out_in + 1;

    out_data = 0xFD;

    out->addr = s_i2cRecvAddr;
    out->flags = 0; // indicates a write
    out->len = 1;
    out->buf = &out_data;
        
    in->addr = s_i2cRecvAddr;
    in->flags = I2C_M_RD;
    in->len = 2;
    in->buf = in_data;

    in_data[0] = 0;
    in_data[1] = 0;
        
    rdwr.nmsgs = 2;
    rdwr.msgs = out_in;

        
    int io = ioctl(m_fd, I2C_RDWR, &rdwr);

    if (io < 0)
    {
        UBXSERLOG("IOCTL for reading nob registers failed.");
    }
    else
    {
        result = in_data[0] * 256 + in_data[1];
    }
    return result;
}

bool CSerialPort::i2cSetFd(fd_set &rfds, int &rMaxFd) const
{
    if (m_fd <= 0)
        return false;
    if ((m_fd + 1) > rMaxFd)
        rMaxFd = m_fd + 1;
    FD_SET(m_fd, &rfds);

    return true;
}

bool CSerialPort::fdSet(fd_set &rfds, int &rMaxFd) const
{
    if(m_i2c)
    {
        ((void)(rfds));
        ((void)(rMaxFd));
        if (m_fd <= 0)
            return false;
    }
    else
    {
        if (m_fd <= 0)
            return false;
        if ((m_fd + 1) > rMaxFd)
            rMaxFd = m_fd + 1;
        FD_SET(m_fd, &rfds);
    }
	return true;
}

bool CSerialPort::fdIsSet(fd_set &rfds) const
{
    if ((m_fd > 0)  && FD_ISSET(m_fd, &rfds))
        return true;
    return false;
}

#if defined (UBX_SERIAL_EXTENSIVE_LOGGING_W) || defined (UBX_SERIAL_EXTENSIVE_LOGGING_R)
void CSerialPort::_LogBufferAsHex(const void *pBuffer, unsigned int size, bool bIsWrite) const
{
    char *sHex, *sHexPtr, *sPrintable, *sPrintablePtr;
    const unsigned char *pBufPtr = (unsigned char const *) pBuffer;
    
    if (size <= 0) return;
    
    sHex = (char *) malloc(size * 3 + 1);
    if (!sHex)
        return;



    pBufPtr = (unsigned char const *) pBuffer;
    sHexPtr = sHex;
    for(unsigned int i = 0; i < size; ++i)
    {
        sprintf(sHexPtr, "%02X ", *pBufPtr);
        sHexPtr += 3;
        pBufPtr++;
        
        if ((((i+1) % 256) == 0) || (i == (size - 1)))
        {
            *sHexPtr = '\0';
            UBXSERLOG("serial hex: %u %c '%s'", size, bIsWrite ? '>' : '<', sHex);
            sHexPtr = sHex;
        }
        
    }
    free(sHex);
    
    sPrintable = (char *) malloc(size + 1);
    if (!sPrintable)
        return;
    
    pBufPtr = (unsigned char const *) pBuffer;
    sPrintablePtr = sPrintable;
    for(unsigned int i = 0; i < size; ++i)
    {
        sprintf(sPrintablePtr, "%c", isprint(*pBufPtr) ? *pBufPtr : '?');
        pBufPtr++;
        sPrintablePtr++;
        if ((((i+1) % 512) == 0) || (i == (size - 1)))
        {
            *sPrintablePtr = '\0';
            UBXSERLOG("serial str: %u %c '%s'", size, bIsWrite ? '>' : '<', sPrintable);
            sPrintablePtr = sPrintable;
        }
    }
    free(sPrintable);
}
#endif // defined (UBX_SERIAL_EXTENSIVE_LOGGING_W) || defined (UBX_SERIAL_EXTENSIVE_LOGGING_R)

ssize_t CSerialPort::readSerial(void *pBuffer, unsigned int size) const
{
    int r;


    if (m_fd <= 0)
        return -1;

    if(m_i2c)
        r = i2cRead(pBuffer, size);
    else
	    r = read(m_fd, pBuffer, size);
    
#ifdef UBX_SERIAL_EXTENSIVE_LOGGING_R
    _LogBufferAsHex(pBuffer, r, false);
#endif // UBX_SERIAL_EXTENSIVE_LOGGING_R

    return r;
}

ssize_t CSerialPort::i2cRead(void *buf, size_t count) const
{
    if(m_fd < 0)
        return -1;

    ssize_t r = 0;

    int read_bytes = count;

    if (read_bytes > m_i2cDataAvailableInBytes)
        read_bytes = m_i2cDataAvailableInBytes;

    if (read_bytes > 0)
        r = read(m_fd, buf, read_bytes);

    return r;
}

ssize_t CSerialPort::writeSerial(const void *pBuffer, size_t size) const
{
    if(size > SSIZE_MAX)
        return -1;

    unsigned char* p;


#ifdef UBX_SERIAL_EXTENSIVE_LOGGING_W
    _LogBufferAsHex(pBuffer, size, true);
#endif // UBX_SERIAL_EXTENSIVE_LOGGING_W

    if (m_fd <= 0)
        return -1;
    if (!m_i2c)
        return write(m_fd, pBuffer, size);

    p = (unsigned char*)malloc(size+1);
    if (!p)
        return 0;
    p[0] = 0xFF; // allways address the stream register
    memcpy(p+1, pBuffer, size);
    ssize_t written = write(m_fd, p, size+1);

    // Clean up
    // The caller does not know about the additional byte
    // sent. Thus, make sure the caller gets the expected
    // return value.
    if (written > 0)
        written --;
    free(p);

    return written;
}

bool CSerialPort::isFdOpen(void) const
{
    return m_fd > 0;
}

int CSerialPort::settermios(int ttybaud, int blocksize)
{
    struct termios  termIos;


    if (m_i2c)	return 0;

    if (tcgetattr(m_fd, &termIos) < 0)
    {
        return (-1);
    }
    cfmakeraw (&termIos);
    termIos.c_cflag |= CLOCAL;
/*    {
        int cnt;

        for (cnt = 0; cnt < NCCS; cnt++)
        {
            termIos.c_cc[cnt] = -1;
        }
    } */

    if (blocksize >= 0)
        termIos.c_cc[VMIN] = (unsigned char) blocksize;
    termIos.c_cc[VTIME] = 0;


#if (B4800 != 4800)
    /*
     * Only paleolithic systems need this.
     */

    switch (ttybaud)
    {
    case 300:
        ttybaud = B300;
        break;
    case 1200:
        ttybaud = B1200;
        break;
    case 2400:
        ttybaud = B2400;
        break;
    case 4800:
        ttybaud = B4800;
        break;
    case 9600:
        ttybaud = B9600;
        break;
    case 19200:
        ttybaud = B19200;
        break;
    case 38400:
        ttybaud = B38400;
        break;
    case 57600:
        ttybaud = B57600;
        break;
    case 115200:
        ttybaud = B115200;
        break;
    default:
        ttybaud = B4800;
        break;
    }
#endif

    if (cfsetispeed(&termIos, (unsigned int) ttybaud) != 0)
    {
        return (-1);
    }
#ifdef BIDIRECTIONAL
    if (cfsetospeed(&termIos, ttybaud) != 0)
    {
        return (-1);
    }
#endif
    if (tcsetattr(m_fd, TCSANOW, &termIos) < 0)
    {
        return (-1);
    }


    return 0;
}


void CSerialPort::i2cConfigTxReady(TX_READY_CONF_t const *txReadyConf)
{
    if(txReadyConf && txReadyConf->enabled)
    {
        if(i2cCheckGpioValue(txReadyConf->hostGpio) >= 0)
        {
            if(txReadyConf->recvPio <= 23)
            {
                // Send B5 62 06 00 14 00 00 00 B5 00 84 00 00 00 00 00 00 00 07 00 03 00 00 00 00 00 5D 8C
                UBX_LOG(LCAT_VERBOSE, "Setting I2C TX-Ready PIO on the receiver to %i and check the host GPIO to %i ", txReadyConf->recvPio, txReadyConf->hostGpio);

                unsigned char cfgPrtI2cPayload[] = { 0x00, 0x00 , 0x81 , 0x00 , 0x84 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x07 , 0x00 , 0x03 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 };
                
                // Set value of the PIO to use
                cfgPrtI2cPayload[2] |= ((unsigned char) txReadyConf->recvPio) << 2;
                unsigned char * pkt = NULL;
                ssize_t pktSize = createUbx(&pkt, 0x06, 0x00, cfgPrtI2cPayload, sizeof(cfgPrtI2cPayload));
                if(pktSize > 0)
                {
                    ssize_t writtenSize = writeSerial(pkt, pktSize);
                    if(writtenSize == pktSize)
                    {
                        // Everything is fine, copy the passed config for later usage
                        memmove(&m_txReadyConf, txReadyConf, sizeof(m_txReadyConf));
                    }
                    else
                    {
                        UBX_LOG(LCAT_ERROR, "An error occured while trying to configure the receiver to use TX-Ready!"
                                            " Disable I2C TX-Ready feature and use polling instead.");
                    }
                    free(pkt);
                }
                else
                {
                    UBX_LOG(LCAT_ERROR, "An error occured while trying to create the configuration packet for TX-Ready!"
                                        " Disable I2C TX-Ready feature and use polling instead.");
                }
            }
            else
            {
                UBX_LOG(LCAT_ERROR, "The provided PIO does not exist or can not"
                                    " be configured to act as TX-Ready on the receiver"
                                    " Disable I2C TX-Ready feature and use polling instead.");
            }
        }
        else
        {
            UBX_LOG(LCAT_ERROR, "The provided GPIO does not exist or can not"
                                " be configured to act as interrupt!"
                                " Disable I2C TX-Ready feature and use polling instead.");
        }
    }
    else
    {
        UBX_LOG(LCAT_VERBOSE, "The I2C TX-Ready feature is not configured. Use polling instead.");
    }
}

/*! Will extract the GPIO state from the sysfs for the value provided
    \param hostGpio          : Number of the GPIO to access in /sys/class/gpio

    \return                    0, if the state of the GPIO is low
                               1, if the state of the GPIO is high
                               -1 if an error occured and the state could
                               not be determinded
*/
int CSerialPort::i2cCheckGpioValue(unsigned int hostGpio) const
{
    int result = -1;
    char *valuePath = NULL;

    int validStr = ubx_asprintf(&valuePath, "%s/gpio%d/value", s_sysfsGpioDir, hostGpio);
    if(validStr > 0)
    {
        int gpioValueFd = open(valuePath, O_RDONLY);
        if(gpioValueFd >= 0)
        {
            char value = 0;
            int rRes=read(gpioValueFd, &value, sizeof(value));
            if(rRes == 1)
            {
                // If the value is neither '0' nor '1', this is an error
                if(value == '0')
                    result = 0;
                else if(value == '1')
                    result = 1;
            }
            close(gpioValueFd);
        }
        free(valuePath);
    }

    return result;
}


