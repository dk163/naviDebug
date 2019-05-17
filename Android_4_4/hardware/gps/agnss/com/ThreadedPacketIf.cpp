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
 * $Id: ThreadedPacketIf.cpp 104534 2015-10-20 14:11:53Z fabio.robbiani $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/agnss/com/ThreadedPacketIf.cpp $
 *****************************************************************************/
 
/*! \file
    \ref CThreadedPacketIf implementation.

    \brief
    Selected functions of the abstract receiver interface class 
    \ref CThreadedPacketIf are implemented here.
*/

#include "ThreadedPacketIf.h"
#include "../helper/helperTypes.h"
#include <errno.h>
#include <stdlib.h>
#include <sys/un.h>
#include <fcntl.h>
#include <new>

/*! The constructor will setup the object in creation, but will
    not initialise any interfaces yet. This has to be done by
    calling \ref open
*/
CThreadedPacketIf::CThreadedPacketIf()
    : _com(NULL)
    , _isOpen(false)
    , _fd_packet_usr(-1)
    , _fd_packet_thread(-1)
{

}


/*! The destructor will clean up the ressources used during the lifetime
    of the object. Unfortunately it is required to call \ref close from
    all derived objects. This is the case because the thread which is
    used within this class indirectly requires functions implemented
    in derived class. Thus it is important to halt the thread before
    the destructor of the object of the dervied class finishes and
    the function becomes pure virtual.
*/
//lint -e{1540}
CThreadedPacketIf::~CThreadedPacketIf()
{
    // close() has to be called by the derived classes!
}

/*! The open function will make prepare the operation of the object
    by opening the communication thread and handing it over the
    file descriptors it should listen to as defined by the derviced
    class. Additionally an interface will be created that can be
    used by the application to communicate with the derived class.

    \return On success a file descriptor will be returned which
            makes it possible to communicate (read/write) with
            the object of the type of the derived class over a
            sequential datagram interface. On failure -1 will
            be returned.
*/
int CThreadedPacketIf::open()
{
    if(isOpen())
        return -1;

    int result = -1;
    _com=new (std::nothrow) CComThread(); 
    int sock_fds[2] = { -1, -1};

    if( socketpair(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK, 0, sock_fds) )
    {
        // Something went wrong - clean up
        int errno_bak = errno;
        errno = errno_bak;
    }
    else
    {
        _fd_packet_usr = sock_fds[0];
        _fd_packet_thread = sock_fds[1];
        int timeout_ms = 0;
        int *fds = NULL;
        int fds_num = impl_open(&fds, &timeout_ms);
        if(fds_num >= 0)
        {
            // If the impl_open call created a dynamically
            // created array of file descriptors increase
            // its size by 1 to make room for the packet
            // interface. Otherwise dynamically create a
            // new interface
            CComThread::COM_CONFIG_t com_config;
            com_config.func_process = process;
            com_config.func_timeout = timeout;
            com_config.func_error = errorOccurred;
            com_config.func_notify = notify;
            com_config.timeout_ms = timeout_ms;
            com_config.context = (void *)this;
            bool success=true;
            _isOpen=true; // For correct operation the thread needs this indirectly
            if(_com->start(&com_config))
            {
                success = (_com->add(_fd_packet_thread) >= 0 );
                for(int i=0; i < fds_num && success; ++i)
                {
                    success = ( _com->add(fds[i]) >= 0 );
                }
            }

            if(success)
            {
                result=_fd_packet_usr;
            }
            else
            {
                close();
            }
            // If the if(fds_tmp) part is entered,
            // fds will be reassigned and must be freed.
            // If it is not entered, it might still
            // hold the memory allocated by impl_open, in
            // which case free has to be called as well.
            // If impl_open() did not do anything to fds,
            // free will do nothing either.
            //lint -e(449)
            free(fds);
        }
    }
    return result;
}

/*! This function will terminate the running thread and make
    sure the object will clean up all the ressources created
    during the \ref open call. This function has to be called
    in the destructor of the derived class!    

    \return On success 0 is returned. Otherwise -1.
*/
int CThreadedPacketIf::close()
{
    int result=-1;
    if(isOpen())
    {
        result=close_no_check();
    }
    return result;
}

/*! The return value will indicate if the \ref open
    has been successfully called before.

    \return            Will return true if the function
                       is properly set up and false
                       otherwise.
*/
bool CThreadedPacketIf::isOpen()
{
    return _isOpen;
}

/*! Add the file descriptor to the handling list of the thread
    and get its location for interaction later on.

    \param fd    : file descriptor to be added
    \return      : On success the position of the file descriptor
                   in the list. Otherwise a negative number
*/
ssize_t CThreadedPacketIf::add(int fd)
{
    if(!isOpen())
        return -1;
    assert(_com);

    return _com->add(fd);
}

/*! Remove the file descriptor from the handling list of the thread

    \param pos   : file descriptor at this position shall be removed
    \return      : true on success, false otherwise
*/
bool CThreadedPacketIf::removeAndClose(size_t pos)
{
    if(!isOpen())
        return false;
    assert(_com);

    return _com->removeAndClose(pos);
}

/*! This function acts as wrapper to \ref CComThread::writeTo and will
    only be possible to use if the object has been setup successfully
    with \ref open.

    \param dest  : Specifies the index of the file descriptor to which the
                   the data should be written. The index is defined by
                   the order in which the file descripter were retrieved
                   from the \ref impl_open call
                   (starting with 1 and ending with size_fds-1, while 0
                   refers to the packet interface created for this object).
    \param buf   : Points to a buffer from which the data should be taken
                   that is written to the file descriptor specified by dest
    \param size  : Specifies the number of elements which should be read
                   read from buf for writing.
    \return        On success the number of bytes written to the file
                   descriptor are returned. On failure -1 will be returned.
*/
ssize_t CThreadedPacketIf::writeTo(size_t dest, unsigned char const * buf, size_t size)
{
    assert(isOpen());
    assert(_com);
    ssize_t result=-1;
    if(_com->writeTo(dest, buf, size))
    {
        result=size;
    }
    return result;
}

/*! This static function will be called if any data could be retrieved
    from a specified file descriptor. It will pass it over to the function
    of the same name belonging to the actual object.

    \param context  : Must contain the 'this' pointer to the actual
                      actual object on which the function of the same name
                      without this parameter will be executed.
    \param source   : Will be handed over to the actual function
    \param buf      : Will be handed over to the actual function
    \param size     : Will be handed over to the actual function
    \return         : The number of bytes processed. Must be
                      smaller than size. If less bytes are
                      processed than passed to the function,
                      the remaining bytes will be passed again
                      during the next call to the function together
                      with additional data if available
*/
size_t CThreadedPacketIf::process( void const * context
                                 , size_t source
                                 , unsigned char * buf
                                 , size_t size )
{
    assert(context);
    assert(buf);
    assert(size);
    return ((CThreadedPacketIf*) const_cast<void*>(context))->process(source, buf, size);
}

/*! This function will be called if any data could be retrieved
    from a specified file descriptor. It will pass it over
    to a function implemented in a derived class for further
    processing.    

    \param source   : Specifies which file descriptor the data has been
                      retrieved. The argument itself is NOT the actual
                      file descriptor, but the index of the file descriptor
                      passed to \ref open. The first index
                      (index 0) refers to the packet interface created by
                      \ref open.
    \param buf      : The data that must be processed. Will not be NULL
    \param size     : Size of buf in bytes. Will not be 0
    \return         : The number of bytes processed. Must be
                      smaller than size. If less bytes are
                      processed than passed to the function,
                      the remaining bytes will be passed again
                      during the next call to the function together
                      with additional data if available
*/
size_t CThreadedPacketIf::process(size_t source, unsigned char * buf, size_t size)
{
    assert(isOpen());
    assert(buf);
    assert(size);
    return impl_process(source, buf, size);
}

/*! This static function will be called by \ref CComThread from the thread
    if a timeout has occured while waiting for data in the specified
    interfaces.It will pass it over to the function of the same name
    belonging to the actual object.

    \param context  : Must contain the 'this' pointer to the actual
                      actual object on which the function of the same name
                      without this parameter will be executed.
*/
void CThreadedPacketIf::timeout(void const * context)
{
    assert(context);
    ((CThreadedPacketIf*) const_cast<void*>(context))->timeout();
}

/*! Will be called by \ref CComThread from the thread if a timeout has occured
    while waiting for data in the specified interfaces. It will, on the other
    hand, call \ref impl_timeout implemented by a derived class, to pass this
    information on to the relevant part of object.
*/
void CThreadedPacketIf::timeout()
{
    assert(isOpen());
    impl_timeout();
}

/*! This static function will be called by \ref CComThread from the thread if
    an error has occured. 

    \param context  : Must contain the 'this' pointer to the actual
                      actual object on which the function of the same name
                      without this parameter will be executed.
    \param operation: Will be handed over to the actual function
    \param fdindex  : Will be handed over to the actual function \param correrrno: Will be handed over to the actual function
*/
void CThreadedPacketIf::errorOccurred( void const * context
                                     , int operation
                                     , ssize_t fdindex
                                     , int correrrno )
{
    assert(context);
    ((CThreadedPacketIf *) const_cast<void*>(context))->errorOccurred( operation
                                                                     , fdindex
                                                                     , correrrno);
}

/*! Will be called by \ref CComThread from the thread if an error has occured.
    It will, on the other hand, call \ref impl_errorOccurred, implemented by
       a derived class, to pass this information on to the relevant part of object.

    \param operation:  Will indicate which operation led to
                       the error. Valid values are
                       COMTHREAD_ERR_POLL
                       COMTHREAD_ERR_READ
                       COMTHREAD_ERR_MEMORY
    \param fdindex  :  Indicates which of the file descriptors
                       was involved during the error. If an
                       operation failed which does not relate
                       to a specific file descriptor, -1 will
                       be passed.
    \param correrrno:  The errno value that was set during the
                       operation that caused the error
*/
void CThreadedPacketIf::errorOccurred(int operation, ssize_t fdindex, int correrrno)
{
    impl_errorOccurred(operation, fdindex, correrrno);
}

/*! This static function will be called from the thread if data is available
    from a source. If func_notify has not been defined in \ref _config
    true will be returned.

    \param context             : Must contain the 'this' pointer to the actual
                                 actual object on which the function of the
                                 same name without this parameter will be
                                 executed.
    \param source              : Number of the file descriptor from which
                                 data is available
    \return                      true if the data should be read and passed
                                 to the process function. false if the
                                 data will be handled otherwise.
*/
bool CThreadedPacketIf::notify( void const * context, size_t source, CComThread::NOTIF_t type)
{
    assert(context);
    return ((CThreadedPacketIf *) const_cast<void*>(context))->notify(source, type);
}

/*! This function will be called from the thread if data is available
    from a source.

    \param source              : Number of the file descriptor from which
                                 data is available
    \param type                : Type of event that occurred
    \return                      true if the data should be read and passed
                                 to the \ref process function. false if the
                                 data will be handled otherwise.
*/
bool CThreadedPacketIf::notify(size_t source, CComThread::NOTIF_t type)
{
    assert(isOpen());
    return impl_notify(source, type);
}

/*! This function will terminate the running thread and make
    sure the object will clean up all the ressources created
    during the \ref open call. This is used by \ref close.
    The difference between these two functions is that
    close_no_check does not require _isOpen to be set
    and can thus also be used during \ref open to clean
    up the object    

    \return On success 0 is returned. Otherwise -1.
*/
int CThreadedPacketIf::close_no_check()
{
    if(_com)
    {
        delete _com;
        _com=NULL;
    }
    int result=impl_close();
    if(_fd_packet_usr>=0)
    {
        ::close(_fd_packet_usr);
        _fd_packet_usr = -1;
    }
    if(_fd_packet_thread>=0)
    {
        // This file descriptor was closed by _com
        _fd_packet_thread = -1;
    }
    _isOpen=false;
    return result;
}

