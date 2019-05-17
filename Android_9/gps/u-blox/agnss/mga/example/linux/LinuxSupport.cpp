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
 * Purpose: Linux specific functions to help a host application to download
 *          MGA assistance data and pass it on to a u-blox GPS receiver.
 *
 *****************************************************************************/

#include <stdio.h>   // Standard input/output definitions
#include <string.h>  // String function definitions
#include <unistd.h>  // UNIX standard function definitions
#include <fcntl.h>   // File control definitions
#include <errno.h>   // Error number definitions
#include <termios.h> // POSIX terminal control definitions
#include <pthread.h>

///////////////////////////////////////////////////////////////////////////////
// serial port configuration
// #define COMPORT "/dev/ttyACM0"
// #define BAUDRATE B9600
///////////////////////////////////////////////////////////////////////////////

extern void* DeviceReadThread(void *);

static int s_fd = 0;
static pthread_t id;

///////////////////////////////////////////////////////////////////////////////
void mySleep(int seconds)
{
    sleep(seconds);
}

void startDeviceReadThread(void)
{
    pthread_create(&id, NULL, DeviceReadThread, NULL);
}

void CleanUpThread()
{
    pthread_join(id, NULL);
}

bool openSerial(const char* port, unsigned int baudrate)
{
    int br;

    // convert the baud rate
    switch(baudrate)
    {
        case 460800:    br=B460800; break;
        case 230400:    br=B230400; break;
        case 115200:    br=B115200; break;
        case 57600:     br=B57600;  break;
        case 38400:     br=B38400;  break;
        case 19200:     br=B19200;  break;
        case 9600:      br=B9600;   break;
        default:        br=B9600;
    }

    // change to you preferred serial port
    s_fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (s_fd != -1)
    {
        fcntl(s_fd, F_SETFL, FNDELAY);

        struct termios options;
        tcgetattr( s_fd, &options );

        cfsetispeed( &options, br );
        cfsetospeed( &options, br);
        cfmakeraw(&options);
        options.c_cflag |= ( CLOCAL | CREAD );  // ignore modem control lines and enable receiver

        // set the character size
        options.c_cflag &= ~CSIZE; // mask the character size bits
        options.c_cflag |= CS8;    // select 8 data bits

        // set no parity (8N1)
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;

        // disable Hardware flow control
        //  options.c_cflag &= ~CNEW_RTSCTS;  -- not supported

        // enable raw input
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

        // disable software flow control
        options.c_iflag &= ~(IXON | IXOFF | IXANY);

        // chose raw (not processed) output
        options.c_oflag &= ~OPOST;
        if ( tcsetattr( s_fd, TCSANOW, &options ) == -1 )
            printf ("Error with tcsetattr = %s\n", strerror ( errno ) );

        fcntl(s_fd, F_SETFL, 0);
    }

    return (s_fd > -1);

}

void closeSerial()
{
    if (s_fd)
        close(s_fd);
}

int readSerial(void *pBuffer, unsigned int size)
{
    return read(s_fd, pBuffer, size);
}

int writeSerial(const void *pBuffer, unsigned int size)
{
    if (s_fd <= 0)
        return -1;

   return write(s_fd, pBuffer, size);
};


